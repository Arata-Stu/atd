#include "atd_rc_input/core.hpp"

#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <set>
#include <unistd.h>

#include "atd_interfaces/msg/operator_event.hpp"
#include "atd_interfaces/msg/pico_telemetry.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace atd_rc_input
{
namespace
{
double steady_now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void keys(const YAML::Node & node, const std::set<std::string> & wanted)
{
  if (!node.IsMap() || node.size() != wanted.size()) {throw std::invalid_argument("invalid profile keys");}
  std::set<std::string> seen;
  for (const auto & item : node) {
    const auto key = item.first.as<std::string>();
    if (!wanted.contains(key) || !seen.insert(key).second) {throw std::invalid_argument("unknown or duplicate key");}
  }
}
int number(const YAML::Node & value)
{
  if (!value.IsScalar() || value.Tag() == "!" || value.Tag() == "tag:yaml.org,2002:str") {
    throw std::invalid_argument("expected integer YAML scalar");
  }
  return value.as<int>();
}
Profile load(const std::string & path)
{
  if (std::filesystem::file_size(path) > 65536) {throw std::invalid_argument("profile exceeds 64 KiB");}
  const auto root = YAML::LoadFile(path);
  keys(root, {"version", "debounce_ms", "roles"});
  if (number(root["version"]) != 1) {throw std::invalid_argument("unsupported profile version");}
  keys(root["roles"], {"recording", "auxiliary_1", "auxiliary_2"});
  Profile result;
  result.debounce_ms = number(root["debounce_ms"]);
  for (size_t i = 0; i < role_names.size(); ++i) {
    const auto value = root["roles"][role_names[i]];
    keys(value, {"channel", "low_threshold", "high_threshold", "inverted"});
    const auto inverted = value["inverted"];
    if (!inverted.IsScalar() || inverted.Tag() == "!" || inverted.Tag() == "tag:yaml.org,2002:str" ||
      (inverted.Scalar() != "true" && inverted.Scalar() != "false"))
    {
      throw std::invalid_argument("inverted must be a YAML bool");
    }
    result.roles[i] = {number(value["channel"]), number(value["low_threshold"]),
      number(value["high_threshold"]), inverted.as<bool>()};
  }
  result.validate();
  return result;
}
YAML::Node profile_yaml(const Profile & profile)
{
  YAML::Node node;
  node["version"] = 1;
  node["debounce_ms"] = profile.debounce_ms;
  for (size_t i = 0; i < role_names.size(); ++i) {
    const auto & b = profile.roles[i];
    auto value = node["roles"][role_names[i]];
    value["channel"] = b.channel;
    value["low_threshold"] = b.low;
    value["high_threshold"] = b.high;
    value["inverted"] = b.inverted;
  }
  return node;
}
}  // namespace

class RcInputNode : public rclcpp::Node
{
public:
  explicit RcInputNode(const rclcpp::NodeOptions & options)
  : Node("atd_rc_input_node", options)
  {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    path_ = declare_parameter<std::string>("profile_path", "/data/results/runs/input/propo.yaml", fixed);
    const bool configuring = declare_parameter<bool>("start_configuring", false, fixed);
    if (configuring) {core_.configure(true, steady_now());}
    const auto qos = rclcpp::QoS(1).reliable().durability_volatile();
    status_pub_ = create_publisher<std_msgs::msg::String>("rc/status", qos);
    events_pub_ = create_publisher<atd_interfaces::msg::OperatorEvent>("rc/events", rclcpp::QoS(10).reliable());
    input_sub_ = create_subscription<atd_interfaces::msg::PicoTelemetry>("vehicle/pico/telemetry", qos,
      [this](atd_interfaces::msg::PicoTelemetry::ConstSharedPtr input) {sample(*input);});
    configure_ = create_service<std_srvs::srv::SetBool>("~/configure",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if (!request->data && !error_.empty()) {
          response->message = "save and reload a valid profile first";
          return;
        }
        response->success = core_.configure(request->data, steady_now());
        response->message = response->success ?
          (request->data ? "role events suspended; Pico driving unchanged" : "role events resumed; baseline will be reacquired") :
          "fresh valid S.BUS input and a valid profile are required";
        publish();
      });
    reload_ = create_service<std_srvs::srv::Trigger>("~/reload_profile",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!core_.configuring()) {response->message = "enter role configuration mode first"; return;}
        response->success = reload();
        response->message = response->success ? "roles loaded; events remain suspended" : error_;
        publish();
      });
    reload();
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {publish();});
    publish();
  }

private:
  bool reload()
  {
    try {
      auto candidate = load(path_);
      core_.profile(candidate);
      error_.clear();
      return true;
    } catch (const std::exception & error) {
      error_ = error.what();
      RCLCPP_WARN(get_logger(), "RC profile: %s", error_.c_str());
      return false;
    }
  }
  void sample(const atd_interfaces::msg::PicoTelemetry & message)
  {
    Sample input;
    input.instance_id = message.instance_id;
    input.serial = message.serial;
    input.sequence = message.sequence_number;
    input.board_us = message.board_us;
    input.sbus_frames = message.sbus_frames;
    input.sbus_age_ms = message.sbus_age_ms;
    input.flags = message.sbus_flags;
    input.sbus_fresh = message.sbus_fresh;
    input.channels = message.channels;
    input.rc_valid = message.rc_valid;
    const double stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9;
    const double source_age = now().seconds() - stamp;
    input.receipt_age_s = std::max(0.0, source_age);
    if (message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000U ||
      !std::isfinite(source_age) || source_age < -0.05 || source_age > 0.25)
    {
      input.rc_valid = false;
    }
    const auto actions = core_.input(input, steady_now());
    // Only accepted advancing frames become source timestamps for observations.
    if (core_.accepted()) {input_stamp_ = stamp;}
    for (auto action : actions) {
      atd_interfaces::msg::OperatorEvent event;
      event.header = message.header;
      event.source = "propo";
      event.action = action;
      events_pub_->publish(event);
    }
  }
  void publish()
  {
    const double now = steady_now();
    const auto reason = core_.reason(now);
    YAML::Node node;
    node["instance_id"] = instance_id_;
    node["node"] = get_fully_qualified_name();
    node["configuring"] = core_.configuring();
    node["profile_valid"] = core_.profile_valid();
    node["profile_path"] = path_;
    node["profile"] = profile_yaml(core_.profile());
    node["ready"] = reason == "ready";
    node["reason"] = reason;
    node["error"] = error_;
    node["input_instance_id"] = core_.have_sample() ? core_.sample().instance_id : "";
    node["serial"] = core_.have_sample() ? core_.sample().serial : "";
    node["input_age_s"] = core_.have_sample() ? YAML::Node(core_.age(now)) : YAML::Node();
    node["input_stamp"] = core_.have_sample() ? YAML::Node(input_stamp_) : YAML::Node();
    for (size_t i = 0; i < role_names.size(); ++i) {
      const auto & binding = core_.profile().roles[i];
      auto role = node["roles"][role_names[i]];
      role["channel"] = binding.channel;
      role["raw"] = binding.channel && core_.have_sample() ?
        YAML::Node(core_.sample().channels[binding.channel - 1]) : YAML::Node();
      role["position"] = name(core_.position(i));
    }
    std_msgs::msg::String output;
    output.data = YAML::Dump(node);
    status_pub_->publish(output);
  }
  Core core_;
  std::string path_, error_;
  const std::string instance_id_{std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) +
    "-" + std::to_string(::getpid())};
  double input_stamp_{0};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<atd_interfaces::msg::OperatorEvent>::SharedPtr events_pub_;
  rclcpp::Subscription<atd_interfaces::msg::PicoTelemetry>::SharedPtr input_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr configure_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace atd_rc_input

RCLCPP_COMPONENTS_REGISTER_NODE(atd_rc_input::RcInputNode)
