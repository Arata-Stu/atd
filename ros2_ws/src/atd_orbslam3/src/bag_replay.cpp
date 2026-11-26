#include "node.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <rosbag2_transport/player.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>

namespace {
using Image = sensor_msgs::msg::Image;
using Imu = sensor_msgs::msg::Imu;
using BagMessage = rosbag2_storage::SerializedBagMessage;
using Player = rosbag2_transport::Player;
using namespace std::chrono_literals;

const std::array<std::string, 2> image_topics{
  "/camera/camera/infra1/image_rect_raw", "/camera/camera/infra2/image_rect_raw"};

template<class T>
void publish_sample(const BagMessage & sample, rclcpp::Publisher<T> & publisher)
{
  auto message = std::make_unique<T>();
  const auto * type = rosidl_typesupport_cpp::get_message_type_support_handle<T>();
  if (rmw_deserialize(sample.serialized_data.get(), type, message.get()) != RMW_RET_OK) {
    throw std::runtime_error("cannot deserialize bag message on " + sample.topic_name);
  }
  publisher.publish(std::move(message));
}

void run()
{
  auto ipc = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto tracker = atd_orbslam3::make_orbslam_node(ipc);
  auto input = std::make_shared<rclcpp::Node>("orbslam3_bag_input", ipc);
  rosbag2_storage::StorageOptions storage;
  storage.uri = input->declare_parameter<std::string>("bag_path", "");
  const auto imu_topic = input->declare_parameter<std::string>("imu_topic", "");
  const auto name = input->declare_parameter<std::string>("player_name", "atd_replay_player");
  const double offset = input->declare_parameter("start_offset_s", 0.);
  const double rate = input->declare_parameter("rate", 1.);
  if (storage.uri.empty() || !std::isfinite(offset) || offset < 0. || !std::isfinite(rate) || rate <= 0.) {
    throw std::invalid_argument("invalid replay bag, rate or start offset");
  }

  rosbag2_transport::PlayOptions options;
  options.read_ahead_queue_size = 64;
  options.clock_publish_frequency = 40.;
  options.rate = static_cast<float>(rate);
  options.start_offset = std::llround(offset * 1e9);
  options.start_paused = true;
  options.disable_keyboard_controls = true;
  options.disable_loan_message = true;
  options.progress_bar_update_rate = 3;
  options.topics_to_filter.assign(image_topics.begin(), image_topics.end());
  if (!imu_topic.empty()) { options.topics_to_filter.push_back(imu_topic); }

  // Player owns timing, /clock and the pause services. Its generic ROS publishers
  // cannot deliver typed intra-process messages. Give them private, unused names;
  // only our publishers below offer the recorded sensor topics.
  options.topic_remapping_options = {"--ros-args"};
  for (const auto & topic : options.topics_to_filter) {
    options.topic_remapping_options.insert(options.topic_remapping_options.end(),
      {"-r", topic + ":=~/serialized" + topic});
  }
  std::array<rclcpp::Publisher<Image>::SharedPtr, 2> images;
  for (std::size_t i = 0; i < images.size(); ++i) {
    images[i] = input->create_publisher<Image>(image_topics[i], rclcpp::SensorDataQoS().keep_last(12));
  }
  rclcpp::Publisher<Imu>::SharedPtr imu;
  if (!imu_topic.empty()) {
    imu = input->create_publisher<Imu>(imu_topic, rclcpp::SensorDataQoS().keep_last(512));
  }
  auto finished = input->create_publisher<std_msgs::msg::Bool>("~/finished", rclcpp::QoS(1).transient_local());

  std::array<std::size_t, 3> published{};
  std::atomic<bool> failed{false};
  auto player = std::make_shared<Player>(
    storage, options, name, rclcpp::NodeOptions().use_global_arguments(false));
  player->add_on_play_message_pre_callback([&](const std::shared_ptr<BagMessage> & sample) {
    if (failed.load()) { return; }
    try {
      if (sample->topic_name == image_topics[0]) {
        publish_sample(*sample, *images[0]);
        ++published[0];
      } else if (sample->topic_name == image_topics[1]) {
        publish_sample(*sample, *images[1]);
        ++published[1];
      } else if (imu && sample->topic_name == imu_topic) {
        publish_sample(*sample, *imu);
        ++published[2];
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(input->get_logger(), "%s", e.what());
      failed.store(true);
    }
  });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tracker);
  executor.add_node(input);
  executor.add_node(player);
  if (!player->play() || !player->wait_for_playback_to_start(10s)) {
    throw std::runtime_error("could not start paused ORB bag player");
  }
  // Keep the tracker and map-save service alive after the player's last publish.
  // The supervisor waits for the final image's processed/skip acknowledgement.
  rclcpp::TimerBase::SharedPtr monitor;
  monitor = input->create_wall_timer(50ms, [&] {
    if (failed.load()) { throw std::runtime_error("ORB bag sensor playback failed"); }
    if (player->wait_for_playback_to_finish(0s)) {
      RCLCPP_INFO(input->get_logger(), "bag センサー発行: 左 %zu / 右 %zu / IMU %zu 件",
        published[0], published[1], published[2]);
      std_msgs::msg::Bool done;
      done.data = true;
      finished->publish(done);
      monitor->cancel();
    }
  });
  executor.spin();
  player->stop();
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    run();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("orbslam3_replay_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
