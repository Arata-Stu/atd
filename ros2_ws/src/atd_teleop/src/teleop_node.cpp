#include "atd_teleop/core.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <functional>
#include <memory>
#include <set>
#include <utility>

#include "atd_interfaces/msg/drive_command.hpp"
#include "atd_interfaces/msg/operator_event.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace atd_teleop
{
namespace
{
double steady_now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void keys(const YAML::Node & node, const std::set<std::string> & expected)
{
  if (!node.IsMap() || node.size() != expected.size()) {throw std::invalid_argument("invalid profile keys");}
  std::set<std::string> found;
  for (const auto & entry : node) {
    const auto key = entry.first.as<std::string>();
    if (!expected.contains(key) || !found.insert(key).second) {
      throw std::invalid_argument("unknown or duplicate profile key: " + key);
    }
  }
}

void numeric(const YAML::Node & node)
{
  // YAML quoted/explicit-string numbers are not silently coerced by .as<T>().
  if (!node.IsScalar() || node.Tag() == "!" || node.Tag() == "tag:yaml.org,2002:str") {
    throw std::invalid_argument("expected numeric YAML scalar");
  }
}

template<size_t N>
int index(const YAML::Node & node, const std::array<const char *, N> & names)
{
  if (!node.IsScalar()) {throw std::invalid_argument("input name must be a string");}
  const auto name = node.as<std::string>();
  if (name.empty()) {return -1;}
  const auto found = std::find(names.begin(), names.end(), name);
  if (found == names.end()) {throw std::invalid_argument("unknown input name: " + name);}
  return static_cast<int>(std::distance(names.begin(), found));
}

Axis axis(const YAML::Node & node)
{
  keys(node, {"axis", "subtract_axis", "scale"});
  numeric(node["scale"]);
  Axis result{index(node["axis"], axis_names), index(node["subtract_axis"], axis_names), node["scale"].as<double>()};
  result.validate();
  return result;
}

Profile load(const std::string & path)
{
  if (std::filesystem::file_size(path) > 65536) {throw std::invalid_argument("profile exceeds 64 KiB");}
  const auto root = YAML::LoadFile(path);
  keys(root, {"version", "axes", "buttons"});
  numeric(root["version"]);
  if (root["version"].as<int>() != 1) {throw std::invalid_argument("unsupported profile version");}
  keys(root["axes"], {"steering", "throttle"});
  keys(root["buttons"], {"enable", "record_start", "record_stop"});
  Profile result;
  result.steering = axis(root["axes"]["steering"]);
  result.throttle = axis(root["axes"]["throttle"]);
  result.enable = index(root["buttons"]["enable"], button_names);
  result.record_start = index(root["buttons"]["record_start"], button_names);
  result.record_stop = index(root["buttons"]["record_stop"], button_names);
  result.validate();
  return result;
}

YAML::Node axis_yaml(const Axis & binding)
{
  YAML::Node result;
  result["axis"] = binding.axis < 0 ? "" : axis_names[binding.axis];
  result["subtract_axis"] = binding.subtract < 0 ? "" : axis_names[binding.subtract];
  result["scale"] = binding.scale;
  return result;
}

YAML::Node profile_yaml(const Profile & profile)
{
  YAML::Node result;
  result["version"] = 1;
  result["axes"]["steering"] = axis_yaml(profile.steering);
  result["axes"]["throttle"] = axis_yaml(profile.throttle);
  result["buttons"]["enable"] = profile.enable < 0 ? "" : button_names[profile.enable];
  result["buttons"]["record_start"] = profile.record_start < 0 ? "" : button_names[profile.record_start];
  result["buttons"]["record_stop"] = profile.record_stop < 0 ? "" : button_names[profile.record_stop];
  return result;
}
}  // namespace

class TeleopNode : public rclcpp::Node
{
public:
  explicit TeleopNode(const rclcpp::NodeOptions & options)
  : Node("atd_teleop_node", options)
  {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    profile_path_ = declare_parameter<std::string>("profile_path", "/data/results/runs/input/teleop.yaml", fixed);
    const bool configuring = declare_parameter<bool>("start_configuring", false, fixed);
    const double input_timeout = declare_parameter<double>("input_timeout_s", 0.25, fixed);
    const double status_timeout = declare_parameter<double>("status_timeout_s", 2.5, fixed);
    const double rate = declare_parameter<double>("publish_rate_hz", 100.0, fixed);
    if (!std::isfinite(input_timeout) || input_timeout < 0.01 || input_timeout > 5.0 ||
      !std::isfinite(status_timeout) || status_timeout < 1.0 || status_timeout > 30.0 ||
      !std::isfinite(rate) || rate < 1.0 || rate > 1000.0)
    {
      throw std::invalid_argument("input_timeout_s: .01..5, status_timeout_s: 1..30, publish_rate_hz: 1..1000");
    }
    core_ = Core(input_timeout, status_timeout);
    if (configuring) {core_.configure(true, steady_now());}
    const auto qos = rclcpp::QoS(1).reliable().durability_volatile();
    command_pub_ = create_publisher<atd_interfaces::msg::DriveCommand>("teleop/command", qos);
    event_pub_ = create_publisher<atd_interfaces::msg::OperatorEvent>("teleop/events", qos);
    status_pub_ = create_publisher<std_msgs::msg::String>("teleop/status", qos);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", qos,
      [this](sensor_msgs::msg::Joy::ConstSharedPtr message) {input(*message);});
    ready_sub_ = create_subscription<std_msgs::msg::Bool>("joy/ready", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        core_.ready(message->data, steady_now());
        status();
      });
    source_sub_ = create_subscription<std_msgs::msg::String>("joy/status", qos,
      [this](std_msgs::msg::String::ConstSharedPtr message) {source(*message);});
    configure_ = create_service<std_srvs::srv::SetBool>("~/configure",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
      std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if (!request->data && !profile_error_.empty()) {
          response->message = "save and reload a valid role profile before resuming";
          return;
        }
        response->success = core_.configure(request->data, steady_now());
        response->message = response->success ?
          (request->data ? "configuration mode: requests suspended" : "input mapping resumed; drive permission is separate") :
          "require drive axes or a recording button, fresh Joy/ready/status, and release all controls";
        publish();
        status(true);
      });
    reload_ = create_service<std_srvs::srv::Trigger>("~/reload_profile",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!core_.configuring()) {response->message = "enter configuration mode first"; return;}
        response->success = load_profile();
        response->message = response->success ? "role profile loaded; requests remain suspended" : profile_error_;
        publish();
        status(true);
      });
    load_profile();
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate)), [this]() {publish();});
    status_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {status(true);});
    publish();
    status(true);
  }

private:
  bool load_profile()
  {
    try {
      auto candidate = load(profile_path_);
      core_.set_profile(candidate);
      profile_error_.clear();
      return true;
    } catch (const std::exception & error) {
      // Candidate validation completes before replacing the active profile.
      profile_error_ = error.what();
      RCLCPP_WARN(get_logger(), "Teleop profile: %s", profile_error_.c_str());
      return false;
    }
  }

  void source(const std_msgs::msg::String & message)
  {
    try {
      if (message.data.size() > 65536) {throw std::invalid_argument("Joy status exceeds 64 KiB");}
      const auto data = YAML::Load(message.data);
      const auto generation = data["generation"].as<int64_t>();
      if (generation < 0) {throw std::invalid_argument("invalid Joy generation");}
      const std::string instance = data["instance_id"].as<std::string>("");
      input_session_ = instance + ":" + std::to_string(generation);
      const bool available = data["connected"].as<bool>() && data["profile_valid"].as<bool>() &&
        !data["configuring"].as<bool>() && data["ready"].as<bool>();
      core_.source(input_session_, available, steady_now());
    } catch (const std::exception &) {
      core_.source("invalid", false, steady_now());
    }
    status();
  }

  void input(const sensor_msgs::msg::Joy & message)
  {
    int64_t stamp_ns = -1;
    if (message.header.stamp.sec >= 0 && message.header.stamp.nanosec < 1000000000U) {
      stamp_ns = static_cast<int64_t>(message.header.stamp.sec) * 1000000000LL + message.header.stamp.nanosec;
    }
    const auto events = core_.input(message.axes, message.buttons, stamp_ns, steady_now());
    input_header_ = message.header;
    for (const auto action : events) {
      atd_interfaces::msg::OperatorEvent event;
      event.header = message.header;
      event.source = "joy";
      event.action = action;
      event_pub_->publish(event);
    }
    status();
  }

  void publish()
  {
    const auto output = core_.output(steady_now());
    atd_interfaces::msg::DriveCommand message;
    message.header = input_header_;
    message.header.stamp.sec = static_cast<int32_t>(output.stamp_ns / 1000000000LL);
    message.header.stamp.nanosec = static_cast<uint32_t>(output.stamp_ns % 1000000000LL);
    message.source = "joy";
    message.valid = output.valid;
    message.steering = output.steering;
    message.throttle = output.throttle;
    message.enable_requested = output.enable_requested;
    command_pub_->publish(message);
    status();
  }

  void status(bool force = false)
  {
    const double now = steady_now();
    const auto output = core_.output(now);
    const std::string reason = core_.reason(now);
    const std::string events_reason = core_.events_reason(now);
    const std::string state = reason + ":" + events_reason + ":" + input_session_ + ":" + profile_error_;
    if (!force && state == status_state_) {return;}
    status_state_ = state;
    YAML::Node data;
    data["node"] = get_fully_qualified_name();
    data["configuring"] = core_.configuring();
    data["profile_valid"] = core_.profile_valid();
    data["profile_path"] = profile_path_;
    data["ready"] = output.valid;
    data["reason"] = reason;
    data["events_ready"] = core_.events_ready(now);
    data["events_reason"] = events_reason;
    data["error"] = profile_error_;
    data["input_session"] = input_session_;
    data["steering"] = output.steering;
    data["throttle"] = output.throttle;
    data["enable_requested"] = output.enable_requested;
    data["profile"] = profile_yaml(core_.profile());
    std_msgs::msg::String message;
    message.data = YAML::Dump(data);
    status_pub_->publish(message);
  }

  Core core_;
  std::string profile_path_, profile_error_, input_session_, status_state_;
  std_msgs::msg::Header input_header_;
  rclcpp::Publisher<atd_interfaces::msg::DriveCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<atd_interfaces::msg::OperatorEvent>::SharedPtr event_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr source_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr configure_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_;
  rclcpp::TimerBase::SharedPtr timer_, status_timer_;
};
}  // namespace atd_teleop

RCLCPP_COMPONENTS_REGISTER_NODE(atd_teleop::TeleopNode)
