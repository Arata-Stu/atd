#include "atd_vehicle_description/layout.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>
using namespace atd_vehicle_description;
void check(bool value, const char * message)
{if (!value) {std::cerr << message << '\n'; std::exit(1);}}
int main()
{
  Layout layout;
  auto frames = transforms(layout);
  check(frames.size() == 1 && frames[0].child == "rear_axle", "unknown layout only defines rear axle alias");
  check(frames[0].translation == std::array<double, 3>{0, 0, 0}, "rear axle shares base origin");
  layout.size[0] = {.257, "manual"};
  frames = transforms(layout);
  check(frames.size() == 2 && frames[1].translation[0] == .257, "known wheelbase defines front axle in +x");
  layout.camera[0] = .1;
  layout.camera_source = "manual";
  check(transforms(layout).size() == 2, "partial disabled camera cannot publish identity mount");
  layout.camera_enabled = true;
  bool rejected = false;
  try {transforms(layout);} catch (const std::invalid_argument &) {rejected = true;}
  check(rejected, "enabled camera requires every explicit coordinate");
  layout.camera = {.1, -.02, .2, 0., 0., std::numbers::pi / 2};
  frames = transforms(layout);
  const auto & camera = frames.back();
  check(camera.child == "camera_link" && frames.size() == 3, "publish mounting frame only, no RealSense internals");
  check(camera.translation == std::array<double, 3>{.1, -.02, .2}, "mount coordinates preserve SI values and signs");
  check(std::abs(camera.quaternion[2] - std::sqrt(.5)) < 1e-12 &&
    std::abs(camera.quaternion[3] - std::sqrt(.5)) < 1e-12, "positive yaw rotates camera +x toward base +y");
  const auto q = quaternion(.3, -.7, 1.1);
  double norm = 0;
  for (auto value : q) {norm += value * value;}
  check(std::abs(norm - 1) < 1e-12, "RPY quaternion remains normalized");
  layout.camera[3] = std::numeric_limits<double>::quiet_NaN();
  rejected = false;
  try {transforms(layout);} catch (const std::invalid_argument &) {rejected = true;}
  check(rejected, "nonfinite pose rejected");
  layout = {};
  layout.size[0] = {.2, "unmeasured"};
  rejected = false;
  try {transforms(layout);} catch (const std::invalid_argument &) {rejected = true;}
  check(rejected, "measured value cannot claim unmeasured source");
  std::cout << "Vehicle layout: unknowns, measured frames, explicit mount, quaternion, and bounds passed\n";
}
