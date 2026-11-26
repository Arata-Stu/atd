#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rmw/rmw.h>
#include <rosbag2_cpp/clocks/time_controller_clock.hpp>
#include <rosbag2_interfaces/srv/is_paused.hpp>
#include <rosbag2_interfaces/srv/pause.hpp>
#include <rosbag2_interfaces/srv/resume.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace atd_localization {
namespace {
using namespace std::chrono_literals;
using BagMessage = rosbag2_storage::SerializedBagMessage;
using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;
using Imu = sensor_msgs::msg::Imu;
using TF = tf2_msgs::msg::TFMessage;
using Clock = rosgraph_msgs::msg::Clock;
using Bool = std_msgs::msg::Bool;
using Header = std_msgs::msg::Header;
using IsPaused = rosbag2_interfaces::srv::IsPaused;
using Pause = rosbag2_interfaces::srv::Pause;
using Resume = rosbag2_interfaces::srv::Resume;
using Trigger = std_srvs::srv::Trigger;

template<class T>
std::unique_ptr<T> read_sample(const BagMessage & sample)
{
  // Read directly from the stored CDR buffer. The typed publisher transfers the
  // allocated message to rclcpp; no serialized image publisher exists.
  auto message = std::make_unique<T>();
  if (rmw_deserialize(sample.serialized_data.get(),
    rosidl_typesupport_cpp::get_message_type_support_handle<T>(), message.get()) != RMW_RET_OK)
  {
    throw std::runtime_error("bag メッセージを復元できません: " + sample.topic_name);
  }
  return message;
}
}  // namespace

class StereoBagInput : public rclcpp::Node
{
public:
  explicit StereoBagInput(const rclcpp::NodeOptions & options)
  : Node("atd_sensor_bag_input", options)
  {
    const auto uri = declare_parameter<std::string>("bag_path", "");
    const double rate = declare_parameter("rate", 1.);
    const double offset = declare_parameter("start_offset_s", 0.);
    const int required = declare_parameter("required_image_readers", 2);
    if (uri.empty() || !std::isfinite(rate) || rate <= 0. || !std::isfinite(offset) ||
      offset < 0. || required < 1 || !options.use_intra_process_comms())
    {
      throw std::invalid_argument("bag パス・再生倍率・開始位置・intra-process の設定を確認してください");
    }
    required_readers_ = static_cast<size_t>(required);
    const auto name = declare_parameter<std::string>("player_name", "atd_replay_player");
    // Use the usual player identity for /clock and /tf_static. Sensor endpoints
    // belong to this component and share the container context with the tracker.
    control_ = std::make_shared<rclcpp::Node>(name, get_namespace(),
      rclcpp::NodeOptions().context(options.context()).use_global_arguments(false)
      .start_parameter_services(false).start_parameter_event_publisher(false)
      .append_parameter_override("start_type_description_service", false));
    const auto control_name = std::string(control_->get_fully_qualified_name());

    rosbag2_storage::StorageFilter filter;
    const bool calibration = declare_parameter("publish_calibration", true);
    const bool trace = declare_parameter("trace_publications", false);
    const std::array<std::string, 2> sides{"left", "right"};
    for (size_t i = 0; i < sides.size(); ++i) {
      const auto camera = "/camera/camera/infra" + std::to_string(i + 1);
      image_names_[i] = declare_parameter(sides[i] + "_image", camera + "/image_rect_raw");
      info_names_[i] = declare_parameter(sides[i] + "_info", camera + "/camera_info");
      images_[i] = create_publisher<Image>(image_names_[i], rclcpp::SensorDataQoS().keep_last(12));
      filter.topics.push_back(image_names_[i]);
      if (trace) {
        traces_[i] = create_publisher<Header>("~/" + sides[i] + "_published", rclcpp::QoS(256));
      }
      if (calibration) {
        infos_[i] = create_publisher<Info>(info_names_[i], rclcpp::SensorDataQoS());
        filter.topics.push_back(info_names_[i]);
      }
    }
    tf_name_ = declare_parameter<std::string>("tf_static", "/tf_static");
    if (calibration) {
      tf_ = control_->create_publisher<TF>(tf_name_, rclcpp::QoS(rclcpp::KeepAll()).transient_local());
      filter.topics.push_back(tf_name_);
    }
    imu_name_ = declare_parameter<std::string>("imu_topic", "");
    if (!imu_name_.empty()) {
      imu_ = create_publisher<Imu>(imu_name_, rclcpp::SensorDataQoS().keep_last(512));
      filter.topics.push_back(imu_name_);
    }
    rosbag2_storage::StorageOptions storage;
    storage.uri = uri;
    reader_ = rosbag2_transport::ReaderWriterFactory::make_reader(storage);
    reader_->open(storage);
    reader_->set_filter(filter);
    const auto & metadata = reader_->get_metadata();
    if (offset > std::chrono::duration<double>(metadata.duration).count()) {
      throw std::invalid_argument("開始位置が bag の収録時間を超えています");
    }
    const auto start = std::chrono::duration_cast<std::chrono::nanoseconds>(
      metadata.starting_time.time_since_epoch()).count() + std::llround(offset * 1e9);
    if (offset > 0.) { reader_->seek(start); }
    clock_ = std::make_unique<rosbag2_cpp::TimeControllerClock>(start, std::chrono::steady_clock::now, 20ms, true);
    clock_->set_rate(rate);
    clock_pub_ = control_->create_publisher<Clock>("/clock", rclcpp::ClockQoS());
    finished_ = create_publisher<Bool>("~/finished", rclcpp::QoS(1).transient_local());
    clock_timer_ = create_wall_timer(25ms, [this] {
      Clock msg;
      msg.clock = rclcpp::Time(clock_->now());
      clock_pub_->publish(msg);
    });
    paused_ = create_service<IsPaused>(control_name + "/is_paused",
      [this](IsPaused::Request::SharedPtr, IsPaused::Response::SharedPtr response) {
        response->paused = clock_->is_paused();
      });
    pause_ = create_service<Pause>(control_name + "/pause",
      [this](Pause::Request::SharedPtr, Pause::Response::SharedPtr) {clock_->pause();});
    resume_ = create_service<Resume>(control_name + "/resume",
      [this](Resume::Request::SharedPtr request, Resume::Response::SharedPtr response) {
        if (request->resume_time.sec != 0 || request->resume_time.nanosec != 0) {
          response->return_code = Resume::Response::RETURN_CODE_RESUME_FAILED;
          response->error_string = "bag 再生には即時再開を指定してください";
          return;
        }
        if (complete_ || !connected()) {
          response->return_code = Resume::Response::RETURN_CODE_RESUME_FAILED;
          response->error_string = "再生が完了済み、またはセンサーの intra-process 購読が不足しています";
          return;
        }
        clock_->resume();
        if (!worker_.joinable()) { worker_ = std::thread([this] { play(); }); }
        response->return_code = Resume::Response::RETURN_CODE_SUCCESS;
      });
    connections_ = create_service<Trigger>("~/connections",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        response->success = connected();
        response->message = statistics();
      });
  }

  ~StereoBagInput() override
  {
    stopping_ = true;
    clock_->pause();
    clock_->wakeup();
    if (worker_.joinable()) { worker_.join(); }
  }

private:
  bool connected() const
  {
    return images_[0]->get_intra_process_subscription_count() >= required_readers_ &&
           images_[1]->get_intra_process_subscription_count() >= required_readers_ &&
           (!imu_ || imu_->get_intra_process_subscription_count() > 0);
  }

  std::string statistics() const
  {
    std::ostringstream s;
    s << "{\"left_published\":" << image_counts_[0] << ",\"right_published\":" << image_counts_[1]
      << ",\"left_intra_readers\":" << images_[0]->get_intra_process_subscription_count()
      << ",\"right_intra_readers\":" << images_[1]->get_intra_process_subscription_count()
      << ",\"left_info_published\":" << info_counts_[0] << ",\"right_info_published\":" << info_counts_[1]
      << ",\"imu_published\":" << imu_count_ << ",\"imu_intra_readers\":"
      << (imu_ ? imu_->get_intra_process_subscription_count() : 0) << "}";
    return s.str();
  }

  void publish(const BagMessage & sample)
  {
    for (size_t i = 0; i < images_.size(); ++i) {
      if (sample.topic_name == image_names_[i]) {
        auto image = read_sample<Image>(sample);
        std::unique_ptr<Header> notification;
        if (traces_[i]) { notification = std::make_unique<Header>(image->header); }
        images_[i]->publish(std::move(image));
        ++image_counts_[i];
        if (notification) { traces_[i]->publish(std::move(notification)); }
        return;
      }
      if (infos_[i] && sample.topic_name == info_names_[i]) {
        infos_[i]->publish(read_sample<Info>(sample));
        ++info_counts_[i];
        return;
      }
    }
    if (imu_ && sample.topic_name == imu_name_) {
      imu_->publish(read_sample<Imu>(sample));
      ++imu_count_;
    } else if (tf_ && sample.topic_name == tf_name_) {
      tf_->publish(read_sample<TF>(sample));
    }
  }

  void play()
  {
    bool success = false;
    try {
      while (!stopping_ && reader_->has_next()) {
        const auto sample = reader_->read_next();
        while (!stopping_ && (clock_->is_paused() || clock_->now() < sample->recv_timestamp)) {
          clock_->sleep_until(sample->recv_timestamp);
        }
        if (!stopping_) { publish(*sample); }
      }
      success = image_counts_[0] > 0 && image_counts_[1] > 0 && (!imu_ || imu_count_ > 0);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "bag センサー再生に失敗しました: %s", e.what());
    }
    if (stopping_ || !rclcpp::ok(get_node_base_interface()->get_context())) { return; }
    clock_->pause();
    complete_ = true;
    RCLCPP_INFO(get_logger(), "bag センサー発行 %s", statistics().c_str());
    Bool done;
    done.data = success;
    finished_->publish(done);
  }

  std::shared_ptr<rclcpp::Node> control_;
  std::array<std::string, 2> image_names_, info_names_;
  std::string tf_name_, imu_name_;
  std::array<rclcpp::Publisher<Image>::SharedPtr, 2> images_;
  std::array<rclcpp::Publisher<Info>::SharedPtr, 2> infos_;
  std::array<rclcpp::Publisher<Header>::SharedPtr, 2> traces_;
  rclcpp::Publisher<Imu>::SharedPtr imu_;
  rclcpp::Publisher<TF>::SharedPtr tf_;
  rclcpp::Publisher<Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<Bool>::SharedPtr finished_;
  rclcpp::TimerBase::SharedPtr clock_timer_;
  rclcpp::Service<IsPaused>::SharedPtr paused_;
  rclcpp::Service<Pause>::SharedPtr pause_;
  rclcpp::Service<Resume>::SharedPtr resume_;
  rclcpp::Service<Trigger>::SharedPtr connections_;
  size_t required_readers_;
  std::array<std::atomic<size_t>, 2> image_counts_{}, info_counts_{};
  std::atomic<size_t> imu_count_{0};
  std::atomic<bool> stopping_{false}, complete_{false};
  std::unique_ptr<rosbag2_cpp::Reader> reader_;
  std::unique_ptr<rosbag2_cpp::TimeControllerClock> clock_;
  std::thread worker_;
};
}  // namespace atd_localization

RCLCPP_COMPONENTS_REGISTER_NODE(atd_localization::StereoBagInput)
