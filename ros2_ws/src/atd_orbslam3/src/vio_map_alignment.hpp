#pragma once

#include "confirmation.hpp"

#include <cstdint>
#include <limits>
#include <optional>
#include <Eigen/Geometry>

namespace atd_orbslam3 {

// Both input poses describe base_link at the same stereo image timestamp.
// A visual match corrects map->odom; it never changes the local odometry.
class VioMapAlignment {
public:
  using Pose = Eigen::Isometry3d;
  explicit VioMapAlignment(const Confirmation & confirmation) : confirmation_(confirmation) {}

  void invalidate()
  {
    if (map_from_odom_) { ++epoch_; }
    map_from_odom_.reset();
    previous_vio_.reset();
    previous_match_.reset();
    confirmation_.invalidate();
  }

  bool observe(std::int64_t stamp, long origin, const Pose & vio_from_base,
               const std::optional<Pose> & map_from_base)
  {
    if (origin != origin_ || stamp <= stamp_) { invalidate(); }
    speed_ = yaw_rate_ = std::numeric_limits<double>::quiet_NaN();
    if (previous_vio_) {
      const double dt = (stamp - stamp_) * 1e-9;
      const Pose delta = previous_vio_->inverse() * vio_from_base;
      odom_from_base_ = odom_from_vio_ * vio_from_base;
      const Eigen::AngleAxisd rotation(delta.linear());
      speed_ = delta.translation().x() / dt;
      yaw_rate_ = rotation.angle() * rotation.axis().z() / dt;
    } else {
      // Each uninterrupted VIO segment has its own origin. Transform its absolute
      // poses directly so that reporting odometry adds no per-frame integration drift.
      odom_from_vio_ = odom_from_base_ * vio_from_base.inverse();
    }
    previous_vio_ = vio_from_base;
    stamp_ = stamp;
    origin_ = origin;

    if (!map_from_base) {
      previous_match_.reset();
      confirmation_.invalidate();
      return false;
    }
    const Pose delta = previous_match_ ? Pose(previous_match_->inverse() * *map_from_base) : Pose::Identity();
    previous_match_ = map_from_base;
    if (!confirmation_.observe(stamp, delta.translation().norm(), Eigen::AngleAxisd(delta.linear()).angle())) {
      return false;
    }
    map_from_odom_ = *map_from_base * odom_from_base_.inverse();
    return true;
  }

  const Pose & odom_from_base() const { return odom_from_base_; }
  const std::optional<Pose> & map_from_odom() const { return map_from_odom_; }
  std::uint32_t epoch() const { return epoch_; }
  double speed() const { return speed_; }
  double yaw_rate() const { return yaw_rate_; }

private:
  Confirmation confirmation_;
  std::optional<Pose> previous_vio_, previous_match_, map_from_odom_;
  Pose odom_from_base_ = Pose::Identity(), odom_from_vio_ = Pose::Identity();
  std::uint32_t epoch_ = 0;
  std::int64_t stamp_ = 0;
  long origin_ = -1;
  double speed_ = std::numeric_limits<double>::quiet_NaN(), yaw_rate_ = speed_;
};
}  // namespace atd_orbslam3
