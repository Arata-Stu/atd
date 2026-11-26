#include "calibration.hpp"
#include "confirmation.hpp"
#include "node.hpp"
#include "stereo_imu_buffer.hpp"
#include "vio_map_alignment.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <unistd.h>
#include <deque>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <regex>
#include <stdexcept>
#include <vector>

#include <atd_interfaces/msg/localization_status.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/buffer_core.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <System.h>

namespace atd_orbslam3 {
namespace fs = std::filesystem;
using Image = sensor_msgs::msg::Image;
using Info = sensor_msgs::msg::CameraInfo;
using Imu = sensor_msgs::msg::Imu;
using Header = std_msgs::msg::Header;
using Odometry = nav_msgs::msg::Odometry;
using Status = atd_interfaces::msg::LocalizationStatus;
using Trigger = std_srvs::srv::Trigger;
using Rigid = Eigen::Isometry3d;
using namespace std::chrono_literals;

namespace {
std::int64_t ns(const Header & h) { return rclcpp::Time(h.stamp).nanoseconds(); }

class WorkDir {
public:
  WorkDir()
  {
    auto pattern = (fs::temp_directory_path() / "atd-orbslam3-XXXXXX").string();
    if (!mkdtemp(pattern.data())) { throw std::runtime_error("cannot create ORB-SLAM3 temporary directory"); }
    path = pattern;
  }
  ~WorkDir() { std::error_code error; fs::remove_all(path, error); }
  fs::path path;
};

Rigid rigid(const geometry_msgs::msg::Transform & t)
{
  Eigen::Quaterniond q(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
  Eigen::Vector3d xyz(t.translation.x, t.translation.y, t.translation.z);
  if (!xyz.allFinite() || !std::isfinite(q.norm()) || std::abs(q.norm() - 1.) > .001) {
    throw std::runtime_error("invalid measured static transform");
  }
  return Eigen::Translation3d(xyz) * q.normalized();
}

geometry_msgs::msg::Pose pose_msg(const Rigid & pose)
{
  geometry_msgs::msg::Pose result;
  result.position.x = pose.translation().x();
  result.position.y = pose.translation().y();
  result.position.z = pose.translation().z();
  Eigen::Quaterniond q(pose.linear());
  q.normalize();
  result.orientation.x = q.x(); result.orientation.y = q.y();
  result.orientation.z = q.z(); result.orientation.w = q.w();
  return result;
}

geometry_msgs::msg::TransformStamped tf_msg(const Rigid & value, const Header & header, const char * child)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header = header;
  tf.child_frame_id = child;
  const auto pose = pose_msg(value);
  tf.transform.translation.x = pose.position.x;
  tf.transform.translation.y = pose.position.y;
  tf.transform.translation.z = pose.position.z;
  tf.transform.rotation = pose.orientation;
  return tf;
}

Rigid rigid(const Sophus::SE3f & pose)
{
  return Eigen::Translation3d(pose.translation().cast<double>()) *
         pose.unit_quaternion().cast<double>().normalized();
}

cv::Mat view(const Image & image, const Calibration & camera)
{
  if ((image.encoding != "mono8" && image.encoding != "8UC1") ||
      int(image.width) != camera.width || int(image.height) != camera.height ||
      image.step < image.width || image.data.size() != std::size_t(image.step) * image.height) {
    throw std::runtime_error("invalid rectified mono8 stereo image, encoding, geometry or stride");
  }
  // TrackStereo accepts const cv::Mat& and clones its inputs. Keep the ROS message alive for the call.
  return cv::Mat(camera.height, camera.width, CV_8UC1, const_cast<unsigned char *>(image.data.data()), image.step);
}

bool sha256(const std::string & value) { return std::regex_match(value, std::regex("[a-f0-9]{64}")); }
}  // namespace

class OrbslamNode : public rclcpp::Node {
public:
  explicit OrbslamNode(const rclcpp::NodeOptions & options)
    : Node("orbslam3_node", options),
      confirm_(param("confirm_frames", 3), param("max_pose_gap_s", .25),
               param("max_speed_mps", 12.), param("max_angular_speed_rps", 12.)), alignment_(confirm_)
  {
    const auto mode = param<std::string>("mode", "mapping");
    fixed_ = mode == "localization";
    if (!fixed_ && mode != "mapping") { throw std::invalid_argument("mode must be mapping or localization"); }
    const bool vio = param("stereo_inertial", false);
    const double fps = param("camera_fps", 90.);
    const double tolerance = param("stereo_tolerance_s", .001);
    max_age_ = param("max_age_s", 1.);
    if (!std::isfinite(fps) || fps < 1. || fps > 1000. || !std::isfinite(tolerance) ||
        tolerance < 0. || tolerance >= .5 / fps || !std::isfinite(max_age_) || max_age_ <= 0.) {
      throw std::invalid_argument("invalid camera frequency, stereo tolerance or freshness");
    }
    fps_ = static_cast<int>(std::lround(fps));
    tolerance_ns_ = std::llround(tolerance * 1e9);
    frames_[0] = param<std::string>("left_frame", "camera_infra1_optical_frame");
    frames_[1] = param<std::string>("right_frame", "camera_infra2_optical_frame");
    if (frames_[0].empty() || frames_[1].empty() || frames_[0] == frames_[1]) {
      throw std::invalid_argument("distinct optical frames are required");
    }
    atlas_ = param<std::string>("atlas_path", "");
    vocab_ = param<std::string>("vocabulary_path", ATD_ORB_VOCABULARY);
    map_id_ = param<std::string>("map_id", "");
    map_sha_ = param<std::string>("map_sha256", "");
    hdmap_sha_ = param<std::string>("hdmap_sha256", "");
    if (!fs::is_regular_file(vocab_)) { throw std::invalid_argument("missing ORB-SLAM3 vocabulary: " + vocab_); }
    if (vio && !fixed_ && !atlas_.empty()) {
      throw std::invalid_argument("reusable visual stereo atlases require stereo-only mapping");
    }
    if (!atlas_.empty() && (!atlas_.is_absolute() || atlas_.filename() != "atlas.osa")) {
      throw std::invalid_argument("atlas_path must be an absolute path ending in atlas.osa");
    }
    if (fixed_) {
      if (map_id_.empty() || !sha256(map_sha_) || (!hdmap_sha_.empty() && !sha256(hdmap_sha_)) ||
          !fs::is_regular_file(atlas_)) {
        throw std::invalid_argument("localization requires verified map identity and an existing atlas");
      }
      reference_ = MapCamera::load(atlas_.parent_path() / "camera.yml");
    } else if (!atlas_.empty() && (!fs::is_directory(atlas_.parent_path()) ||
        fs::exists(atlas_) || fs::exists(atlas_.parent_path() / "camera.yml") ||
        fs::exists(atlas_.parent_path() / "cloud.json"))) {
      throw std::invalid_argument("use a new map staging directory; refusing to replace assets");
    }
    Inertial calibration;
    calibration.frame = param<std::string>("imu_frame", "");
    calibration.hz = param("imu_frequency", 0.);
    calibration.gyro_noise = param("gyro_noise_density", 0.);
    calibration.accel_noise = param("accel_noise_density", 0.);
    calibration.gyro_walk = param("gyro_random_walk", 0.);
    calibration.accel_walk = param("accel_random_walk", 0.);
    if (vio) { calibration.validate(); inertial_ = calibration; }

    const auto sensor = rclcpp::SensorDataQoS().keep_last(12);
    for (int side = 0; side < 2; ++side) {
      const auto topic = "/camera/camera/infra" + std::to_string(side + 1);
      images_[side] = create_subscription<Image>(topic + "/image_rect_raw", sensor,
        [this, side](Image::ConstSharedPtr image, const rclcpp::MessageInfo & info) {
          ++received_images_[side];
          if (info.get_rmw_message_info().from_intra_process) { ++ipc_images_[side]; }
          if (finished_ || ns(image->header) <= last_stamp_) { return; }
          auto & queue = image_queue_[side];
          if (!queue.empty() && ns(queue.back()->header) >= ns(image->header)) { return; }
          queue.push_back(std::move(image));
          if (queue.size() > 12) { queue.pop_front(); }
          drain();
        });
      infos_[side] = create_subscription<Info>(topic + "/camera_info", sensor,
        [this, side](Info::ConstSharedPtr info) {
          const auto & last = camera_info_[side];
          if (!last || last->width != info->width || last->height != info->height ||
              last->header.frame_id != info->header.frame_id || last->p != info->p || last->r != info->r ||
              last->binning_x != info->binning_x || last->binning_y != info->binning_y || last->roi != info->roi) {
            camera_info_[side] = std::move(info);
            calibration_dirty_ = true;
            drain();
          }
        });
    }
    static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(100).transient_local().reliable(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr message) {
        for (const auto & tf : message->transforms) {
          if (!tf_.setTransform(tf, "atd_orbslam3", true)) { throw std::runtime_error("invalid static transform"); }
        }
        calibration_dirty_ = true;
        drain();
      });
    if (vio) {
      imu_sub_ = create_subscription<Imu>("/camera/camera/imu", rclcpp::SensorDataQoS().keep_last(512),
        [this](Imu::ConstSharedPtr imu, const rclcpp::MessageInfo & info) {
          ++received_imu_;
          if (info.get_rmw_message_info().from_intra_process) { ++ipc_imu_; }
          if (finished_) { return; }
          const auto stamp = ns(imu->header);
          if (stamp <= imu_buffer_.latest_stamp()) { return; }
          const auto & a = imu->linear_acceleration;
          const auto & g = imu->angular_velocity;
          if (imu->header.frame_id != inertial_->frame || !std::isfinite(a.x) || !std::isfinite(a.y) ||
              !std::isfinite(a.z) || !std::isfinite(g.x) || !std::isfinite(g.y) || !std::isfinite(g.z)) {
            throw std::runtime_error("invalid combined IMU frame or values");
          }
          imu_buffer_.push(stamp, {static_cast<float>(a.x), static_cast<float>(a.y), static_cast<float>(a.z),
                                  static_cast<float>(g.x), static_cast<float>(g.y), static_cast<float>(g.z),
                                  stamp * 1e-9});
          drain();
        });
      imu_skipped_ = create_publisher<Header>("/orbslam3/imu_skipped", rclcpp::QoS(1).transient_local());
    }
    ready_ = create_publisher<std_msgs::msg::Bool>("/orbslam3/ready", rclcpp::QoS(1).transient_local());
    odometry_ = create_publisher<Odometry>("/visual_slam/vis/slam_odometry", rclcpp::QoS(12));
    processed_ = create_publisher<Header>("/orbslam3/processed", rclcpp::QoS(1).transient_local());
    if (fixed_) {
      status_ = create_publisher<Status>("/localization/status", 10);
      frame_status_ = create_publisher<Status>("/orbslam3/frame_status", 128);
      transforms_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      timer_ = create_wall_timer(100ms, [this]() { watch(); });
      if (vio) {
        vio_odometry_ = create_publisher<Odometry>("/orbslam3/vio_odometry", 128);
        map_match_ = create_publisher<Header>("/orbslam3/map_match", 128);
      }
    } else if (!atlas_.empty()) {
      save_ = create_service<Trigger>("/orbslam3/save_map",
        [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) { save_map(*response); });
    }
    last_clock_ = now().nanoseconds();
    session_ = work_.path.filename().string();
  }

  ~OrbslamNode() override
  {
    RCLCPP_INFO(get_logger(),
      "ORB 入力: 左 %zu / 右 %zu / IMU %zu 件、うち intra-process 左 %zu / 右 %zu / IMU %zu 件",
      received_images_[0], received_images_[1], received_imu_, ipc_images_[0], ipc_images_[1], ipc_imu_);
    if (tracked_pairs_ > 0) {
      RCLCPP_INFO(get_logger(),
        "ORB 計測: 左右画像の受信 %zu / %zu、TrackStereo %zu 対、平均 %.2f ms / 最大 %.2f ms、"
        "撮影周期 (%.2f ms) を超えた呼び出し %zu 回",
        received_images_[0], received_images_[1], tracked_pairs_, tracking_total_ms_ / tracked_pairs_,
        tracking_max_ms_, 1000.0 / fps_, over_camera_period_);
      if (fixed_core_) {
        RCLCPP_INFO(get_logger(), "1 対の平均処理時間の内訳: Stereo-Inertial %.2f ms、固定地図照合 %.2f ms",
          (tracking_total_ms_ - match_total_ms_) / tracked_pairs_, match_total_ms_ / tracked_pairs_);
      }
    }
    if (core_ && !shutdown_) {
      try { core_->Shutdown(); }
      catch (const std::exception & e) { RCLCPP_WARN(get_logger(), "%s", e.what()); }
    }
    if (fixed_core_) {
      try { fixed_core_->Shutdown(); }
      catch (const std::exception & e) { RCLCPP_WARN(get_logger(), "%s", e.what()); }
    }
  }

private:
  template<class T> T param(const std::string & key, T def)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(key, def, descriptor);
  }

  void drain()
  {
    if (finished_ || !camera_info_[0] || !camera_info_[1]) { return; }
    if ((calibration_dirty_ || !core_) && !prepare()) { return; }
    auto & left = image_queue_[0];
    auto & right = image_queue_[1];
    while (!left.empty() && !right.empty()) {
      const auto stamp = ns(left.front()->header);
      const auto difference = ns(right.front()->header) - stamp;
      if (difference > tolerance_ns_ || stamp <= last_stamp_) { left.pop_front(); continue; }
      if (difference < -tolerance_ns_) { right.pop_front(); continue; }
      if (inertial_ && imu_buffer_.latest_stamp() < stamp) { return; }
      auto left_image = left.front(), right_image = right.front();
      if (left_image->header.frame_id != frames_[0] || right_image->header.frame_id != frames_[1]) {
        throw std::runtime_error("unexpected stereo image optical frames");
      }
      left.pop_front(); right.pop_front();
      if (!finished_) { process(*left_image, *right_image); }
      if (finished_) { return; }
    }
  }

  bool prepare()
  {
    const auto & left_frame = frames_[0];
    const auto & right_frame = frames_[1];
    // RealSense ROS 4.56.3 expresses the right projection in the left optical frame.
    if (camera_info_[0]->header.frame_id != left_frame ||
        (camera_info_[1]->header.frame_id != right_frame && camera_info_[1]->header.frame_id != left_frame)) {
      throw std::runtime_error("stereo image and projection frames disagree");
    }
    const auto calibration = Calibration::from(*camera_info_[0], *camera_info_[1]);
    if (core_ && !camera_->matches(calibration)) {
      throw std::runtime_error("camera calibration or optical frames changed; restart localization");
    }
    if (!tf_.canTransform("base_link", left_frame, tf2::TimePointZero, nullptr) ||
        !tf_.canTransform(left_frame, right_frame, tf2::TimePointZero, nullptr) ||
        (inertial_ && !tf_.canTransform(inertial_->frame, left_frame, tf2::TimePointZero, nullptr))) {
      return false;
    }
    const auto mount = rigid(tf_.lookupTransform("base_link", left_frame, tf2::TimePointZero).transform);
    const auto stereo = rigid(tf_.lookupTransform(left_frame, right_frame, tf2::TimePointZero).transform);
    if ((stereo.translation() - Eigen::Vector3d(calibration.baseline, 0., 0.)).norm() > .002 ||
        Eigen::AngleAxisd(stereo.linear()).angle() > .01) {
      throw std::runtime_error("rectified CameraInfo baseline disagrees with the static stereo transform");
    }
    if (core_) {
      calibration_dirty_ = false;
      if (!base_from_camera_.matrix().isApprox(mount.matrix(), 1e-6)) {
        throw std::runtime_error("camera mount changed while tracking; restart localization");
      }
      return true;
    }
    if (fixed_ && !reference_->calibration.matches(calibration)) {
      throw std::runtime_error("live stereo calibration differs from the saved map");
    }
    base_from_camera_ = mount;
    if (inertial_) {
      inertial_->imu_from_camera = rigid(tf_.lookupTransform(inertial_->frame, left_frame, tf2::TimePointZero).transform);
    }
    fs::path stem;
    if (fixed_) { stem = atlas_; stem.replace_extension(); }
    else if (!atlas_.empty()) { stem = work_.path / "atlas"; }
    const auto settings = work_.path / "settings.yml";
    if (fixed_ && inertial_) {
      // Atlas deserialization restores ORB's static ID counters. Load it before starting VIO.
      const auto fixed_settings = work_.path / "fixed.yml";
      write_settings(fixed_settings, calibration, fps_, nullptr, stem, true);
      fixed_core_ = std::make_unique<ORB_SLAM3::System>(vocab_, fixed_settings.string(),
        ORB_SLAM3::System::STEREO, false, 0, "", true);
      write_settings(settings, calibration, fps_, &*inertial_, {}, false, false);
      core_ = std::make_unique<ORB_SLAM3::System>(vocab_, settings.string(),
        ORB_SLAM3::System::IMU_STEREO, false, 0, "", false, fixed_core_.get());
    } else {
      write_settings(settings, calibration, fps_, inertial_ ? &*inertial_ : nullptr, stem, fixed_);
      core_ = std::make_unique<ORB_SLAM3::System>(vocab_, settings.string(),
        inertial_ ? ORB_SLAM3::System::IMU_STEREO : ORB_SLAM3::System::STEREO, false, 0, "", fixed_);
    }
    camera_ = calibration;
    calibration_dirty_ = false;
    std_msgs::msg::Bool ready;
    ready.data = true;
    ready_->publish(ready);
    return true;
  }

  void process(const Image & left, const Image & right)
  {
    const auto stamp = ns(left.header);
    std::vector<ORB_SLAM3::IMU::Point> measurements;
    if (inertial_) {
      measurements = imu_buffer_.take(stamp, last_stamp_, inertial_->hz);
      if (measurements.empty()) {
        imu_skipped_->publish(left.header);
        return;
      }
    }
    const auto input_left = view(left, *camera_);
    const auto input_right = view(right, *camera_);
    const auto begin = std::chrono::steady_clock::now();
    const auto tcw = core_->TrackStereo(input_left, input_right, stamp * 1e-9, measurements);
    std::optional<Rigid> matched_pose;
    if (fixed_core_) {
      const auto match_start = std::chrono::steady_clock::now();
      const auto camera_from_map_atlas = fixed_core_->TrackFixedFrameFrom(*core_);
      if (fixed_core_->GetTrackingState() == ORB_SLAM3::Tracking::OK &&
          fixed_core_->GetFixedMapInliers() > 0 && camera_from_map_atlas.matrix().allFinite()) {
        matched_pose = reference_->map_from_atlas * rigid(camera_from_map_atlas).inverse() * base_from_camera_.inverse();
      }
      match_total_ms_ += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - match_start).count();
    }
    const std::chrono::duration<double, std::milli> elapsed = std::chrono::steady_clock::now() - begin;
    ++tracked_pairs_;
    tracking_total_ms_ += elapsed.count();
    tracking_max_ms_ = std::max(tracking_max_ms_, elapsed.count());
    if (elapsed.count() > 1000.0 / fps_) { ++over_camera_period_; }
    last_stamp_ = stamp;
    const bool ok = core_->GetTrackingState() == ORB_SLAM3::Tracking::OK &&
                    (!inertial_ || core_->GetTimeFromIMUInit() > 0.) &&
                    tcw.matrix().allFinite() && (!fixed_ || fixed_core_ || core_->GetFixedMapInliers() > 0);
    if (fixed_core_) {
      const long origin = ok ? core_->GetTrackingMapOriginId() : -1;
      if (origin < 0) {
        clear_tracking("慣性初期化済みの VIO 追跡を待っています", ever_tracked_ ? "lost" : "localizing", &left.header);
      } else {
        observe_vio(left.header, origin, rigid(tcw).inverse() * base_from_camera_.inverse(), matched_pose);
      }
    } else if (!ok) {
      clear_tracking("選択した地図への画像照合を待っています", ever_tracked_ ? "lost" : "localizing", &left.header);
    } else {
      const long origin = core_->GetTrackingMapOriginId();
      if (origin >= 0) {
        if (!fixed_ && origin_ && *origin_ != origin) {
          // A new submap has independent coordinates. Count processed input without splicing
          // its unrelated trajectory into the original map or publishing a reusable Atlas.
          if (!changed_map_) {
            RCLCPP_WARN(get_logger(), "Active submap changed. Suppressing poses from other coordinate origins.");
          }
          changed_map_ = true;
          processed_->publish(left.header);
          return;
        }
        origin_ = origin;
        const auto camera_from_atlas = rigid(tcw);
        if (!reference_) { reference_ = MapCamera{*camera_, base_from_camera_ * camera_from_atlas}; }
        const Rigid pose = reference_->map_from_atlas * camera_from_atlas.inverse() * base_from_camera_.inverse();
        observe(left.header, pose);
      } else {
        clear_tracking("地図の基準キーフレームを待っています", ever_tracked_ ? "lost" : "localizing", &left.header);
      }
    }
    processed_->publish(left.header);
  }

  void observe(const Header & image, const Rigid & pose)
  {
    Header h = image;
    h.frame_id = "map";
    Odometry odom;
    odom.header = h;
    odom.child_frame_id = "base_link";
    odom.pose.pose = pose_msg(pose);
    if (!fixed_) { odometry_->publish(odom); return; }

    const auto stamp = ns(image);
    if (!fresh(image)) { return; }
    const auto delta = previous_pose_ ? Rigid(previous_pose_->inverse() * pose) : Rigid::Identity();
    const Eigen::AngleAxisd rotation(delta.linear());
    const bool was_tracking = confirm_.tracking();
    const bool accepted = confirm_.observe(stamp, delta.translation().norm(), rotation.angle());
    const double dt = (stamp - previous_stamp_) * 1e-9;
    previous_pose_ = pose;
    previous_stamp_ = stamp;
    if (!accepted) {
      state_ = ever_tracked_ ? "lost" : "localizing";
      reason_ = "連続する画像で地図上の姿勢を確認しています";
      publish_invalid(&image);
      return;
    }
    // Immutable-map tracking within this epoch. Start a new map->odom correction on reacquisition,
    // keeping the odometry origin continuous across gaps. Never differentiate across a correction.
    if (was_tracking) { odom_from_base_ = odom_from_base_ * delta; }
    const Rigid map_from_odom = pose * odom_from_base_.inverse();
    Header odom_header = h;
    odom_header.frame_id = "odom";
    transforms_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{
      tf_msg(map_from_odom, h, "odom"), tf_msg(odom_from_base_, odom_header, "base_link")});
    odom.twist.twist.linear.x = delta.translation().x() / dt;
    odom.twist.twist.angular.z = rotation.angle() * rotation.axis().z() / dt;
    publish_tracking(odom, "保存地図の特徴点と連続する画像の姿勢が整合しました");
  }

  void observe_vio(const Header & image, long origin, const Rigid & vio_from_base,
                   const std::optional<Rigid> & matched_pose)
  {
    if (!fresh(image)) { return; }
    const bool corrected = alignment_.observe(ns(image), origin, vio_from_base, matched_pose);
    Odometry odom;
    odom.header = image;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose = pose_msg(alignment_.odom_from_base());
    odom.twist.twist.linear.x = alignment_.speed();
    odom.twist.twist.angular.z = alignment_.yaw_rate();
    vio_odometry_->publish(odom);
    const auto local_tf = tf_msg(alignment_.odom_from_base(), odom.header, "base_link");
    if (!alignment_.map_from_odom()) {
      transforms_->sendTransform(local_tf);
      state_ = ever_tracked_ ? "lost" : "localizing";
      reason_ = "VIO と同じ画像時刻の保存地図照合を確認しています";
      publish_invalid(&image);
      return;
    }
    odom.header.frame_id = "map";
    if (corrected) { map_match_->publish(odom.header); }
    const Rigid & map_from_odom = *alignment_.map_from_odom();
    transforms_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{
      tf_msg(map_from_odom, odom.header, "odom"), local_tf});
    odom.pose.pose = pose_msg(map_from_odom * alignment_.odom_from_base());
    publish_tracking(odom, corrected ? "保存地図との画像照合で VIO の地図座標を補正しました" :
                                       "直近の保存地図照合を基準に Stereo-Inertial で追跡しています");
  }

  bool fresh(const Header & image)
  {
    const double age = (now().nanoseconds() - ns(image)) * 1e-9;
    if (age >= -.1 && age <= max_age_) { return true; }
    clear_tracking("画像時刻が現在の ROS 時刻と一致しません", "stale", &image);
    return false;
  }

  void publish_tracking(const Odometry & odom, const char * reason)
  {
    odometry_->publish(odom);
    state_ = "tracking";
    reason_ = reason;
    ever_tracked_ = true;
    auto status = status_message();
    status.header = odom.header;
    status.pose = odom.pose.pose;
    status.speed_mps = odom.twist.twist.linear.x;
    status.yaw_rate_rps = odom.twist.twist.angular.z;
    status_->publish(status);
    frame_status_->publish(status);
  }

  Status status_message() const
  {
    Status status;
    status.map_id = map_id_; status.map_sha256 = map_sha_; status.hdmap_sha256 = hdmap_sha_;
    status.session_id = session_; status.epoch = fixed_core_ ? alignment_.epoch() : confirm_.epoch();
    status.state = state_; status.reason = reason_;
    return status;
  }

  void publish_invalid(const Header * image = nullptr)
  {
    if (!fixed_) { return; }
    auto status = status_message();
    status.header.frame_id = "map";
    status.header.stamp = now();
    const auto invalid = std::numeric_limits<double>::quiet_NaN();
    auto & p = status.pose.position;
    auto & q = status.pose.orientation;
    p.x = p.y = p.z = q.x = q.y = q.z = q.w = invalid;
    status.speed_mps = status.yaw_rate_rps = invalid;
    status_->publish(status);
    // One decision per processed stereo image. Heartbeats have no corresponding image.
    if (image) {
      status.header.stamp = image->stamp;
      frame_status_->publish(status);
    }
  }

  void clear_tracking(const std::string & reason, const std::string & state, const Header * image = nullptr)
  {
    confirm_.invalidate();
    if (fixed_core_) { alignment_.invalidate(); }
    previous_pose_.reset();
    state_ = state;
    reason_ = reason;
    publish_invalid(image);
  }

  void watch()
  {
    const auto clock = now().nanoseconds();
    if (clock < last_clock_) {
      finished_ = true;
      clear_tracking("ROS時刻が巻き戻りました。再起動してください", "stale");
    } else if (!finished_ && last_stamp_ > 0 && (clock - last_stamp_) * 1e-9 > max_age_) {
      clear_tracking("新しいステレオ観測が途絶えました", "stale");
    } else if (state_ != "tracking") {
      publish_invalid();
    }
    last_clock_ = clock;
  }

  void save_map(Trigger::Response & response)
  {
    try {
      if (finished_ || changed_map_ || !origin_ || !core_ || !reference_ || !camera_ ||
          core_->GetTrackingMapOriginId() != *origin_) {
        throw std::runtime_error("no stable single map available to save");
      }
      finished_ = true;
      shutdown_ = true;
      core_->Shutdown();
      const fs::path staged = work_.path / "atlas.osa";
      const fs::path calibration = work_.path / "camera.yml";
      if (!fs::is_regular_file(staged) || fs::file_size(staged) == 0) {
        throw std::runtime_error("ORB-SLAM3 did not save the Atlas");
      }
      reference_->save(calibration);
      const auto cloud = work_.path / "cloud.json";
      reference_->save_cloud(cloud, core_->GetSavedMapPoints());
      const auto verified = MapCamera::load(calibration);
      if (!verified.calibration.matches(*camera_) ||
          !verified.map_from_atlas.matrix().isApprox(reference_->map_from_atlas.matrix())) {
        throw std::runtime_error("saved stereo calibration or map frame failed verification");
      }
      // The caller publishes the whole staging directory only after this service succeeds.
      fs::copy_file(calibration, atlas_.parent_path() / "camera.yml");
      fs::copy_file(cloud, atlas_.parent_path() / "cloud.json");
      fs::copy_file(staged, atlas_);
      response.success = true;
      response.message = "ステレオ地図・カメラ校正・地図座標系の点群を保存しました";
    } catch (const std::exception & error) {
      response.success = false;
      response.message = error.what();
    }
  }

  WorkDir work_;
  std::array<std::size_t, 2> received_images_{}, ipc_images_{};
  std::size_t received_imu_ = 0, ipc_imu_ = 0;
  std::size_t tracked_pairs_{}, over_camera_period_{};
  double tracking_total_ms_{}, tracking_max_ms_{}, match_total_ms_{};
  Confirmation confirm_;
  VioMapAlignment alignment_;
  tf2::BufferCore tf_;
  std::optional<Calibration> camera_;
  std::optional<Inertial> inertial_;
  std::optional<MapCamera> reference_;
  std::optional<long> origin_;
  std::optional<Rigid> previous_pose_;
  Rigid base_from_camera_ = Rigid::Identity(), odom_from_base_ = Rigid::Identity();
  std::array<std::string, 2> frames_;
  std::array<Info::ConstSharedPtr, 2> camera_info_;
  std::array<std::deque<Image::ConstSharedPtr>, 2> image_queue_;
  StereoImuBuffer imu_buffer_;
  std::unique_ptr<ORB_SLAM3::System> fixed_core_, core_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transforms_;
  std::array<rclcpp::Subscription<Image>::SharedPtr, 2> images_;
  std::array<rclcpp::Subscription<Info>::SharedPtr, 2> infos_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_sub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_, vio_odometry_;
  rclcpp::Publisher<Status>::SharedPtr status_, frame_status_;
  rclcpp::Publisher<Header>::SharedPtr processed_, imu_skipped_, map_match_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_;
  rclcpp::Service<Trigger>::SharedPtr save_;
  rclcpp::TimerBase::SharedPtr timer_;
  fs::path atlas_;
  std::string session_, vocab_, map_id_, map_sha_, hdmap_sha_;
  std::string state_ = "localizing", reason_ = "ステレオ画像・校正・車体の固定TFを待っています";
  bool calibration_dirty_ = true, changed_map_ = false;
  bool fixed_ = false, finished_ = false, shutdown_ = false, ever_tracked_ = false;
  int fps_ = 0;
  std::int64_t last_stamp_ = 0, previous_stamp_ = 0;
  std::int64_t tolerance_ns_ = 0, last_clock_ = 0;
  double max_age_ = 0.;
};

std::shared_ptr<rclcpp::Node> make_orbslam_node(const rclcpp::NodeOptions & options)
{
  return std::make_shared<OrbslamNode>(options);
}
}  // namespace atd_orbslam3

RCLCPP_COMPONENTS_REGISTER_NODE(atd_orbslam3::OrbslamNode)
