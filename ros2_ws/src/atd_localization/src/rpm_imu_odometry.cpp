#include "rpm_imu_filter.hpp"
#include <algorithm>
#include <Eigen/Geometry>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include "atd_interfaces/msg/pico_telemetry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace atd_localization
{
class RpmImuOdometry : public rclcpp::Node
{
  using Pico = atd_interfaces::msg::PicoTelemetry;
  using Imu = sensor_msgs::msg::Imu;
  using Speed = geometry_msgs::msg::TwistStamped;
  using Odom = nav_msgs::msg::Odometry;
  struct Gyro {int64_t time; double rate;};
  struct Wheel {int64_t time; uint64_t board; uint32_t edges; std::string session;};
  static constexpr int64_t max_gap = 200000000;
  static constexpr int64_t gyro_gap = 30000000;

public:
  explicit RpmImuOdometry(const rclcpp::NodeOptions & options)
  : Node("rpm_imu_ekf", options), buffer_(get_clock()), listener_(buffer_,
      tf2_ros::StaticTransformListener::RequiredInterfaces(get_node_base_interface(),
        get_node_logging_interface(), get_node_parameters_interface(), get_node_topics_interface()), false)
  {
    const auto model_topic = declare_parameter<std::string>("model_speed_topic", "");
    speed_sd_ = positive("speed_std_mps", 0.10);
    gyro_sd_ = positive("gyro_std_rps", 0.03);
    speed_walk_ = positive("speed_walk", 2.);
    rate_walk_ = positive("yaw_rate_walk", 1.);
    output_ = create_publisher<Odom>("/localization/rpm_imu/odom", 10);
    gyro_sub_ = create_subscription<Imu>(declare_parameter<std::string>("gyro_topic", "/camera/camera/gyro/sample"),
      rclcpp::SensorDataQoS(), [this](Imu::ConstSharedPtr m) {gyro(*m);});
    if (model_topic.empty()) {
      pulses_ = positive("pulses_per_meter", 0.);
      wheel_sub_ = create_subscription<Pico>("/vehicle/pico/telemetry", 10,
        [this](Pico::ConstSharedPtr m) {wheel(*m);});
    } else {
      speed_sub_ = create_subscription<Speed>(model_topic, rclcpp::SensorDataQoS(),
        [this](Speed::ConstSharedPtr m) {model_speed(*m);});
    }
  }

private:
  double positive(const std::string & name, double initial)
  {
    const double v = declare_parameter<double>(name, initial);
    if (!std::isfinite(v) || v <= 0.) {throw std::invalid_argument(name + " must be positive");}
    return v;
  }

  void reset_segment()
  {
    filter_ = RpmImuFilter();
    frame_.clear();
  }

  void gyro(const Imu & msg)
  {
    const int64_t t = rclcpp::Time(msg.header.stamp).nanoseconds();
    if (t <= 0 || msg.header.frame_id.empty() || msg.angular_velocity_covariance[0] < 0) {return;}
    // The camera mount and optical frame are static; never assume identity.
    try {
      auto q = buffer_.lookupTransform("base_link", msg.header.frame_id, tf2::TimePointZero).transform.rotation;
      const auto & v = msg.angular_velocity;
      const double rate = (Eigen::Quaterniond(q.w, q.x, q.y, q.z) *
        Eigen::Vector3d(v.x, v.y, v.z)).z();
      if (!std::isfinite(rate)) {return;}
      if (!gyro_.empty() && t <= gyro_.back().time) {return;}
      gyro_.push_back({t, rate});
      while (gyro_.size() > 1024) {gyro_.pop_front();}
      process();
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "D455 static TF: %s", e.what());
    }
  }

  void wheel(const Pico & msg)
  {
    Wheel w{rclcpp::Time(msg.header.stamp).nanoseconds(), msg.board_us, msg.rpm_edges, msg.instance_id};
    if (w.time <= 0 || w.session.empty() || !msg.rpm_seen) {return;}
    if (wheel_) {
      const Wheel p = *wheel_;
      if (w.session != p.session || w.time < p.time || w.board < p.board ||
          w.board - p.board > uint64_t(max_gap / 1000) || w.time - p.time > max_gap) {
        restart(w.time);
      } else if (w.time == p.time || w.board == p.board) {
        return;
      } else {
        observe(SpeedInterval::from_edges(p.time, w.time, p.edges, w.edges, w.board-p.board, pulses_));
      }
    }
    wheel_ = w;
  }

  void model_speed(const Speed & msg)
  {
    const int64_t t = rclcpp::Time(msg.header.stamp).nanoseconds();
    const double v = msg.twist.linear.x;
    if (t <= 0 || msg.header.frame_id != "base_link" || !std::isfinite(v)) {return;}
    if (last_speed_) {
      const auto [previous, speed] = *last_speed_;
      if (t == previous) {return;}
      if (t < previous || t - previous > max_gap) {
        restart(t);
      } else {
        observe(SpeedInterval::from_model(previous, t, speed, v));
      }
    }
    last_speed_.emplace(t, v);
  }

  void restart(int64_t t)
  {
    intervals_.clear();
    reset_segment();
    if (!gyro_.empty() && gyro_.back().time > t + max_gap) {gyro_.clear();}
  }

  void observe(const SpeedInterval & interval)
  {
    intervals_.push_back(interval);
    if (intervals_.size() > 32) {intervals_.pop_front(); reset_segment();}
    process();
  }

  void process()
  {
    while (!intervals_.empty() && !gyro_.empty()) {
      const auto interval = intervals_.front();
      if (gyro_.back().time < interval.end) {return;}
      intervals_.pop_front();
      while (gyro_.size() > 1 && gyro_[1].time <= interval.start) {gyro_.pop_front();}
      if (gyro_.front().time > interval.start) {reset_segment(); continue;}
      double angle = 0.;
      bool covered = true;
      for (size_t i = 1; i < gyro_.size(); ++i) {
        const auto & g = gyro_[i - 1];
        const auto & h = gyro_[i];
        if (h.time - g.time > gyro_gap) {covered = false; break;}
        const auto left = std::max(interval.start, g.time), right = std::min(interval.end, h.time);
        const double mid_fraction = (0.5 * ((left - g.time) + (right - g.time))) / (h.time - g.time);
        angle += (g.rate + mid_fraction * (h.rate - g.rate)) * ((right - left) * 1e-9);
        if (h.time >= interval.end) {break;}
      }
      if (!covered) {reset_segment(); continue;}
      const double dt = (interval.end - interval.start) * 1e-9;
      filter_.step(dt, interval.speed, angle / dt, speed_sd_ * speed_sd_ + interval.variance,
                   gyro_sd_ * gyro_sd_, speed_walk_, rate_walk_);
      if (frame_.empty()) {
        frame_ = "rpm_imu_odom_" + std::to_string(interval.start) + "_" + std::to_string(++segment_);
      }
      publish(interval.end);
    }
  }

  void publish(int64_t stamp)
  {
    Odom msg;
    msg.header.stamp = rclcpp::Time(stamp);
    msg.header.frame_id = frame_;
    msg.child_frame_id = "base_link";
    const auto & x = filter_.state;
    msg.pose.pose.position.x = x(0);
    msg.pose.pose.position.y = x(1);
    msg.pose.pose.orientation.z = std::sin(x(2) * .5);
    msg.pose.pose.orientation.w = std::cos(x(2) * .5);
    msg.twist.twist.linear.x = x(3);
    msg.twist.twist.angular.z = x(4);
    const int pose_indices[] = {0, 1, 5}, twist_indices[] = {0, 5};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        msg.pose.covariance[6 * pose_indices[i] + pose_indices[j]] = filter_.covariance(i, j);
      }
    }
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 2; ++j) {
        msg.twist.covariance[6 * twist_indices[i] + twist_indices[j]] = filter_.covariance(i + 3, j + 3);
      }
    }
    output_->publish(msg);
  }

  double pulses_{0.}, speed_sd_, gyro_sd_, speed_walk_, rate_walk_;
  std::string frame_;
  uint64_t segment_{0};
  RpmImuFilter filter_;
  tf2_ros::Buffer buffer_;
  tf2_ros::StaticTransformListener listener_;
  std::deque<Gyro> gyro_;
  std::optional<Wheel> wheel_;
  std::optional<std::pair<int64_t, double>> last_speed_;
  std::deque<SpeedInterval> intervals_;
  rclcpp::Subscription<Speed>::SharedPtr speed_sub_;
  rclcpp::Publisher<Odom>::SharedPtr output_;
  rclcpp::Subscription<Imu>::SharedPtr gyro_sub_;
  rclcpp::Subscription<Pico>::SharedPtr wheel_sub_;
};
}  // namespace atd_localization
RCLCPP_COMPONENTS_REGISTER_NODE(atd_localization::RpmImuOdometry)
