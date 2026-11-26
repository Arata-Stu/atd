#include "../src/vio_map_alignment.hpp"

#include <cmath>
#include <stdexcept>

using atd_orbslam3::VioMapAlignment;
using Pose = VioMapAlignment::Pose;

void check(bool condition)
{
  if (!condition) { throw std::runtime_error("unexpected VIO/map alignment"); }
}

Pose pose(double x, double y, double yaw)
{
  return Eigen::Translation3d(x, y, 0.) * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}

int main()
{
  VioMapAlignment state(atd_orbslam3::Confirmation(2, .25, 10., 5.));
  const Pose first = pose(12., -4., .7);
  const Pose map_from_odom = pose(-3., 8., -1.3);
  const Pose step = pose(.1, 0., .02);
  const Pose second = first * step;
  const Pose third = second * step;

  check(!state.observe(100000000, 1, first, map_from_odom));
  check(!state.map_from_odom() && std::isnan(state.speed()));
  check(state.odom_from_base().isApprox(Pose::Identity()));
  check(state.observe(200000000, 1, second, map_from_odom * step));
  check(state.map_from_odom()->isApprox(map_from_odom));
  check(state.odom_from_base().isApprox(step));
  check(std::abs(state.speed() - 1.) < 1e-9 && std::abs(state.yaw_rate() - .2) < 1e-9);
  check(!state.observe(300000000, 1, third, std::nullopt));
  check(state.map_from_odom()->isApprox(map_from_odom));
  check(state.odom_from_base().isApprox(step * step));
  // Confirming a different correction moves only the map-frame result.
  const Pose corrected = pose(7., -5., .1);
  const Pose fourth = third * step;
  const Pose fifth = fourth * step;
  check(!state.observe(400000000, 1, fourth, corrected * step * step * step));
  check(state.observe(500000000, 1, fifth, corrected * step * step * step * step));
  check(state.map_from_odom()->isApprox(corrected));
  check(state.odom_from_base().isApprox(first.inverse() * fifth));
  const Pose last_odom = state.odom_from_base();
  // A new VIO map has unrelated coordinates and requires a fresh visual anchor.
  check(!state.observe(600000000, 2, pose(-20., 4., -3.), std::nullopt));
  check(!state.map_from_odom() && state.epoch() == 1 && std::isnan(state.speed()));
  check(state.odom_from_base().isApprox(last_odom));
  check(!state.observe(700000000, 2, first, map_from_odom));
  check(state.observe(800000000, 2, second, map_from_odom * step));
  state.invalidate();
  state.invalidate();
  check(!state.map_from_odom() && state.epoch() == 2);
  check(!state.observe(900000000, 2, first, map_from_odom));
  check(!state.map_from_odom() && std::isnan(state.speed()));
}
