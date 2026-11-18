#include "atd_vehicle_description/layout.hpp"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <filesystem>
#include <memory>
#include <set>
#include <utility>
#include <unistd.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

namespace atd_vehicle_description
{
namespace
{
void keys(const YAML::Node & node, const std::set<std::string> & wanted)
{
  if (!node.IsMap() || node.size() != wanted.size()) {throw std::invalid_argument("invalid layout keys");}
  std::set<std::string> seen;
  for (const auto & entry : node) {
    const auto key = entry.first.as<std::string>();
    if (!wanted.contains(key) || !seen.insert(key).second) {throw std::invalid_argument("unknown or duplicate key");}
  }
}
std::optional<double> number(const YAML::Node & node)
{
  if (node.IsNull()) {return std::nullopt;}
  if (!node.IsScalar() || node.Tag() == "!" || node.Tag() == "tag:yaml.org,2002:str") {
    throw std::invalid_argument("expected numeric or null layout value");
  }
  return node.as<double>();
}
Layout parse(const YAML::Node & root)
{
  keys(root, {"version", "dimensions", "camera"});
  const auto version = root["version"];
  if (!version.IsScalar() || version.Scalar() != "1" ||
    (version.Tag() != "?" && version.Tag() != "tag:yaml.org,2002:int")) {
    throw std::invalid_argument("layout version must be integer 1");
  }
  keys(root["dimensions"], std::set<std::string>(dimensions.begin(), dimensions.end()));
  Layout result;
  for (size_t i = 0; i < dimensions.size(); ++i) {
    const auto field = root["dimensions"][dimensions[i]];
    keys(field, {"value_m", "source"});
    result.size[i] = {number(field["value_m"]), field["source"].as<std::string>()};
  }
  const auto camera = root["camera"];
  keys(camera, {"enabled", "translation_m", "rotation_rpy_rad", "source"});
  if (!camera["enabled"].IsScalar() || camera["enabled"].Tag() == "!" ||
    camera["enabled"].Tag() == "tag:yaml.org,2002:str" ||
    (camera["enabled"].Scalar() != "true" && camera["enabled"].Scalar() != "false")) {
    throw std::invalid_argument("camera.enabled must be a YAML bool");
  }
  result.camera_enabled = camera["enabled"].as<bool>();
  keys(camera["translation_m"], {"x", "y", "z"});
  keys(camera["rotation_rpy_rad"], {"roll", "pitch", "yaw"});
  const std::array<const char *, 6> axes{"x", "y", "z", "roll", "pitch", "yaw"};
  for (size_t i = 0; i < axes.size(); ++i) {
    result.camera[i] = number(camera[i < 3 ? "translation_m" : "rotation_rpy_rad"][axes[i]]);
  }
  result.camera_source = camera["source"].as<std::string>();
  result.validate();
  return result;
}
YAML::Node yaml_profile(const Layout & value)
{
  YAML::Node root;
  root["version"] = 1;
  for (size_t i = 0; i < dimensions.size(); ++i) {
    root["dimensions"][dimensions[i]]["value_m"] = value.size[i].value ? YAML::Node(*value.size[i].value) : YAML::Node();
    root["dimensions"][dimensions[i]]["source"] = value.size[i].source;
  }
  root["camera"]["enabled"] = value.camera_enabled;
  root["camera"]["source"] = value.camera_source;
  const std::array<const char *, 6> axes{"x", "y", "z", "roll", "pitch", "yaw"};
  for (size_t i = 0; i < axes.size(); ++i) {
    root["camera"][i < 3 ? "translation_m" : "rotation_rpy_rad"][axes[i]] =
      value.camera[i] ? YAML::Node(*value.camera[i]) : YAML::Node();
  }
  return root;
}
}  // namespace
class VehicleDescriptionNode : public rclcpp::Node
{
public:
  explicit VehicleDescriptionNode(const rclcpp::NodeOptions & options)
  : Node("atd_vehicle_description_node", options)
  {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    path_ = declare_parameter<std::string>("profile_path", "/data/results/runs/vehicle/layout.yaml", fixed);
    for (const auto & [parameter, expected] : std::array<std::pair<const char *, const char *>, 4>{{
      {"base_frame", "base_link"}, {"rear_axle_frame", "rear_axle"},
      {"front_axle_frame", "front_axle"}, {"camera_frame", "camera_link"}}})
    {
      if (declare_parameter<std::string>(parameter, expected, fixed) != expected) {
        throw std::invalid_argument("vehicle frame IDs are fixed");
      }
    }
    try {
      if (std::filesystem::file_size(path_) > 65536) {throw std::invalid_argument("layout exceeds 64 KiB");}
      layout_ = parse(YAML::LoadFile(path_));
      valid_ = true;
    } catch (const std::exception & error) {error_ = error.what(); RCLCPP_WARN(get_logger(), "%s", error_.c_str());}
    status_ = create_publisher<std_msgs::msg::String>("vehicle/layout/status", rclcpp::QoS(1).reliable());
    broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
    std::vector<geometry_msgs::msg::TransformStamped> output;
    tf_stamp_ = now();
    for (const auto & transform : transforms(layout_)) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = tf_stamp_;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = transform.child;
      tf.transform.translation.x = transform.translation[0];
      tf.transform.translation.y = transform.translation[1];
      tf.transform.translation.z = transform.translation[2];
      tf.transform.rotation.x = transform.quaternion[0];
      tf.transform.rotation.y = transform.quaternion[1];
      tf.transform.rotation.z = transform.quaternion[2];
      tf.transform.rotation.w = transform.quaternion[3];
      output.push_back(tf);
      frames_.push_back(transform.child);
    }
    broadcaster_->sendTransform(output);
    published_ = true;
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {publish();});
    publish();
  }
private:
  void publish()
  {
    YAML::Node state;
    state["instance_id"] = instance_id_;
    state["stamp"] = now().seconds();
    state["tf_stamp"] = tf_stamp_.seconds();
    state["profile_path"] = path_;
    state["profile_valid"] = valid_;
    state["active_profile"] = yaml_profile(layout_);
    state["tf_published"] = published_;
    state["published_frames"] = frames_;
    state["reason"] = valid_ ? "ready" : "profile_missing_or_invalid";
    state["error"] = error_;
    std_msgs::msg::String message;
    message.data = YAML::Dump(state);
    status_->publish(message);
  }
  Layout layout_;
  std::string path_, error_;
  const std::string instance_id_{std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + "-" + std::to_string(::getpid())};
  bool valid_{false}, published_{false};
  rclcpp::Time tf_stamp_{0, 0, RCL_ROS_TIME};
  std::vector<std::string> frames_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace atd_vehicle_description
RCLCPP_COMPONENTS_REGISTER_NODE(atd_vehicle_description::VehicleDescriptionNode)
