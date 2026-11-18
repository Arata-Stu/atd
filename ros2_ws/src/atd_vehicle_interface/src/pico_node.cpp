#include "atd_vehicle_interface/protocol.hpp"
#include "atd_vehicle_interface/serial.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

#include "atd_interfaces/msg/pico_telemetry.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace atd_vehicle_interface
{
namespace
{
double steady_now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

// この段階ではCDC1監視だけを提供する。制御ポートを開くAPIを持たない。
class PicoNode : public rclcpp::Node
{
public:
  explicit PicoNode(const rclcpp::NodeOptions & options)
  : Node("atd_vehicle_interface", options)
  {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    serial_filter_ = declare_parameter<std::string>("serial", "", fixed);
    path_filter_ = declare_parameter<std::string>("sensor_path", "", fixed);
    sysfs_ = declare_parameter<std::string>("sysfs_dir", "/sys/class/tty", fixed);
    dev_ = declare_parameter<std::string>("device_dir", "/dev", fixed);
    timeout_ = declare_parameter<double>("telemetry_timeout_s", 0.3, fixed);
    const int scan_ms = declare_parameter<int>("scan_period_ms", 1000, fixed);
    if (!std::isfinite(timeout_) || timeout_ < 0.05 || timeout_ > 5.0 || scan_ms < 100 || scan_ms > 10000) {
      throw std::invalid_argument("telemetry_timeout_s: .05..5, scan_period_ms: 100..10000");
    }
    boot_id_ = std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + "-" + std::to_string(::getpid());
    telemetry_pub_ = create_publisher<atd_interfaces::msg::PicoTelemetry>("vehicle/pico/telemetry", rclcpp::QoS(1).reliable());
    status_pub_ = create_publisher<std_msgs::msg::String>("vehicle/pico/status", rclcpp::QoS(1).reliable());
    io_timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {poll();});
    scan_timer_ = create_wall_timer(std::chrono::milliseconds(scan_ms), [this]() {scan();});
    status_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {status();});
    scan();
    status();
  }

private:
  std::string session() const {return boot_id_ + ":" + std::to_string(epoch_);}
  void disconnect(const std::string & reason)
  {
    serial_.close();
    progress_ = Progress{};
    discarded_total_ += lines_.discarded();
    lines_ = Lines{};
    info_seen_ = false;
    stream_confirmed_ = false;
    reason_ = reason;
  }

  void scan()
  {
    if (serial_.connected()) {return;}
    try {
      const auto devices = discover(sysfs_, dev_, serial_filter_, path_filter_);
      if (devices.size() != 1) {
        device_ = {};
        reason_ = devices.empty() ? "waiting_for_device" : "multiple_devices";
        return;
      }
      device_ = devices.front();
      serial_.open(device_.path);
      ++epoch_;
      connected_at_ = steady_now();
      last_poll_ = connected_at_;
      info_seen_ = false;
      stream_confirmed_ = false;
      reason_ = "awaiting_stream";
      serial_.request_stream();
    } catch (const std::exception & error) {
      disconnect("serial_error");
      last_error_ = error.what();
    }
  }

  void poll()
  {
    if (!serial_.connected()) {return;}
    const double now = steady_now();
    // transport timeout未満のexecutor停止でも、RC期限を超えるOS滞留は読み出さない。
    if (polling_gap_expired(last_poll_, now) ||
      (progress_.have() ? !progress_.fresh(now, timeout_) : now - connected_at_ > timeout_))
    {
      disconnect("telemetry_stale");
      return;
    }
    last_poll_ = now;
    try {
      serial_.request_stream();
      std::array<char, 512> bytes{};
      for (size_t total = 0; total < 4096;) {
        const size_t count = serial_.read(bytes.data(), bytes.size());
        if (!count) {break;}
        total += count;
        lines_.feed(bytes.data(), count, [this](std::string_view line) {receive(line);});
      }
    } catch (const std::exception & error) {
      disconnect("serial_error");
      last_error_ = error.what();
    }
  }

  void receive(std::string_view line)
  {
    if (line.starts_with("JPBB Rev6 Pico v0.1;") && line.find("CDC1=JPX1") != std::string_view::npos) {
      info_seen_ = true;
      return;
    }
    if (line.starts_with("OK STREAM 50Hz") && info_seen_) {
      stream_confirmed_ = true;
      return;
    }
    const auto sample = parse_telemetry(line);
    if (!sample) {++malformed_; return;}
    // INFO/STREAMの応答より前のフレームは、前セッションの送信残りかもしれない。
    if (!stream_confirmed_) {++rejected_; return;}
    const double now = steady_now();
    const auto result = progress_.accept(*sample, now);
    if (result == Progress::Result::rejected) {++rejected_; return;}
    if (result == Progress::Result::reset) {++epoch_; ++board_resets_;}
    stamp_ = this->now();
    last_error_.clear();
    const bool rc_valid = progress_.rc_valid(now, timeout_);
    reason_ = rc_valid ? "receiving" : "rc_unavailable";
    atd_interfaces::msg::PicoTelemetry message;
    message.header.stamp = stamp_;
    message.header.frame_id = "pico";
    message.instance_id = session();
    message.serial = device_.serial;
    message.sequence_number = sample->sequence;
    message.board_us = sample->board_us;
    message.sbus_age_ms = sample->sbus_age_ms;
    message.sbus_frames = sample->sbus_frames;
    message.sbus_lost_frames = sample->sbus_lost_frames;
    message.sbus_failsafe_frames = sample->sbus_failsafe_frames;
    message.uart_errors = sample->uart_errors;
    message.sbus_flags = sample->sbus_flags;
    message.channels = sample->channels;
    message.rpm_edges = sample->rpm_edges;
    message.rpm_last_us32 = sample->rpm_last_us32;
    message.rpm_period_us32 = sample->rpm_period_us32;
    message.rpm_period_recent = sample->rpm_period_recent;
    message.rpm_seen = sample->rpm_seen;
    message.vbec_mv = sample->vbec_mv;
    message.power_ready = sample->power_ready;
    message.sbus_fresh = sample->sbus_fresh;
    message.active_path = sample->active_path;
    message.fault_bits = sample->fault_bits;
    message.tx0_dropped = sample->tx0_dropped;
    message.tx1_dropped = sample->tx1_dropped;
    message.rx0_rejected = sample->rx0_rejected;
    message.rx1_rejected = sample->rx1_rejected;
    message.rc_valid = rc_valid;
    telemetry_pub_->publish(message);
  }

  void status()
  {
    const double now = steady_now();
    YAML::Node state;
    state["connected"] = serial_.connected();
    state["streaming"] = serial_.connected() && progress_.fresh(now, timeout_);
    state["serial"] = device_.serial;
    state["path"] = device_.path;
    state["instance_id"] = session();
    state["telemetry_timeout_s"] = timeout_;
    state["reason"] = serial_.connected() && progress_.have() && !progress_.rc_valid(now, timeout_) ? "rc_unavailable" : reason_;
    state["error"] = last_error_;
    state["malformed_frames"] = malformed_;
    state["rejected_frames"] = rejected_;
    state["discarded_lines"] = discarded_total_ + lines_.discarded();
    state["board_resets"] = board_resets_;
    if (!serial_.connected() || !progress_.have()) {
      state["telemetry"] = YAML::Node(YAML::NodeType::Null);
    } else {
      const auto & sample = progress_.last();
      YAML::Node value;
      value["stamp"] = stamp_.seconds();
      value["age_s"] = std::max(0.0, now - progress_.received_at());
      value["stale"] = !progress_.fresh(now, timeout_);
      value["rc_valid"] = progress_.rc_valid(now, timeout_);
      if (const auto age = progress_.rc_age(now)) {value["rc_age_s"] = *age;}
      else {value["rc_age_s"] = YAML::Node(YAML::NodeType::Null);}
      value["instance_id"] = session();
      value["serial"] = device_.serial;
      value["sequence"] = sample.sequence;
      value["board_us"] = sample.board_us;
      value["sbus_age_ms"] = sample.sbus_age_ms;
      value["sbus_frames"] = sample.sbus_frames;
      value["sbus_lost_frames"] = sample.sbus_lost_frames;
      value["sbus_failsafe_frames"] = sample.sbus_failsafe_frames;
      value["uart_errors"] = sample.uart_errors;
      value["sbus_flags"] = sample.sbus_flags;
      for (const auto channel : sample.channels) {value["channels"].push_back(channel);}
      value["rpm_edges"] = sample.rpm_edges;
      value["rpm_last_us32"] = sample.rpm_last_us32;
      value["rpm_period_us32"] = sample.rpm_period_us32;
      value["rpm_period_recent"] = sample.rpm_period_recent;
      value["rpm_seen"] = sample.rpm_seen;
      value["vbec_mv"] = sample.vbec_mv;
      value["power_ready"] = sample.power_ready;
      value["sbus_fresh"] = sample.sbus_fresh;
      value["active_path"] = sample.active_path;
      value["fault_bits"] = sample.fault_bits;
      value["tx0_dropped"] = sample.tx0_dropped;
      value["tx1_dropped"] = sample.tx1_dropped;
      value["rx0_rejected"] = sample.rx0_rejected;
      value["rx1_rejected"] = sample.rx1_rejected;
      state["telemetry"] = value;
    }
    std_msgs::msg::String message;
    message.data = YAML::Dump(state);
    status_pub_->publish(message);
  }

  std::string serial_filter_, path_filter_, sysfs_, dev_, boot_id_, reason_{"waiting_for_device"}, last_error_;
  uint64_t epoch_{}, malformed_{}, rejected_{}, discarded_total_{}, board_resets_{};
  double timeout_{0.3}, connected_at_{}, last_poll_{};
  bool info_seen_{}, stream_confirmed_{};
  Device device_;
  Serial serial_;
  Lines lines_;
  Progress progress_;
  rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<atd_interfaces::msg::PicoTelemetry>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr io_timer_, scan_timer_, status_timer_;
};
}  // namespace atd_vehicle_interface

RCLCPP_COMPONENTS_REGISTER_NODE(atd_vehicle_interface::PicoNode)
