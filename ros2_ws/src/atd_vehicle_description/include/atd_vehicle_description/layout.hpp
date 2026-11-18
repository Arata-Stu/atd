#pragma once
#include <array>
#include <cmath>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace atd_vehicle_description
{
inline constexpr std::array<const char *, 7> dimensions{
  "wheelbase", "front_track", "rear_track", "body_length", "body_width", "body_height", "rear_overhang"};
struct Dimension {std::optional<double> value; std::string source{"unmeasured"};};
struct Layout
{
  std::array<Dimension, 7> size{};
  bool camera_enabled{false};
  std::array<std::optional<double>, 6> camera{};  // x,y,z,roll,pitch,yaw in m/rad.
  std::string camera_source{"unmeasured"};
  static void source(const std::string & value, bool known)
  {
    if ((value != "unmeasured" && value != "manual" && value != "cad_candidate") ||
      ((value == "unmeasured") == known)) {throw std::invalid_argument("source and unknown values disagree");}
  }
  void validate() const
  {
    for (size_t i = 0; i < size.size(); ++i) {
      const auto & d = size[i];
      source(d.source, d.value.has_value());
      if (d.value && (!std::isfinite(*d.value) || *d.value < 0 || *d.value > 20 || (i != 6 && *d.value == 0))) {
        throw std::invalid_argument("invalid vehicle dimension");
      }
    }
    bool any = false, all = true;
    for (size_t i = 0; i < camera.size(); ++i) {
      any = any || camera[i].has_value(); all = all && camera[i].has_value();
      if (camera[i] && (!std::isfinite(*camera[i]) || std::abs(*camera[i]) > (i < 3 ? 20 : std::numbers::pi))) {
        throw std::invalid_argument("invalid camera pose");
      }
    }
    source(camera_source, any);
    if (camera_enabled && !all) {throw std::invalid_argument("camera pose is incomplete");}
  }
};
struct Transform
{
  std::string child;
  std::array<double, 3> translation{};
  std::array<double, 4> quaternion{0, 0, 0, 1};
};
inline std::array<double, 4> quaternion(double roll, double pitch, double yaw)
{
  const auto cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const auto cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const auto cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  return {sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy};
}
inline std::vector<Transform> transforms(const Layout & value)
{
  value.validate();
  std::vector<Transform> result{{"rear_axle", {}, {0, 0, 0, 1}}};
  if (value.size[0].value) {result.push_back({"front_axle", {*value.size[0].value, 0, 0}, {0, 0, 0, 1}});}
  if (value.camera_enabled) {
    result.push_back({"camera_link", {*value.camera[0], *value.camera[1], *value.camera[2]},
      quaternion(*value.camera[3], *value.camera[4], *value.camera[5])});
  }
  return result;
}
}  // namespace atd_vehicle_description
