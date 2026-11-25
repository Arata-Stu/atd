#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace atd_orbslam3 {

// Consecutive observations must agree in the selected metric map before a pose reaches consumers.
class Confirmation {
public:
  Confirmation(int frames, double max_gap, double max_speed, double max_angular_speed)
    : frames_(frames), max_gap_(max_gap), max_speed_(max_speed), max_angular_speed_(max_angular_speed)
  {
    if (frames < 2 || !positive(max_gap) || !positive(max_speed) || !positive(max_angular_speed)) {
      throw std::invalid_argument("invalid fixed-map confirmation parameters");
    }
  }
  bool observe(std::int64_t stamp, double displacement, double angle)
  {
    const auto dt = (stamp - last_stamp_) * 1e-9;
    if (count_ && (!(dt > 0 && dt <= max_gap_) || !std::isfinite(displacement) ||
        !std::isfinite(angle) || displacement > dt * max_speed_ || angle > dt * max_angular_speed_)) {
      invalidate();
    }
    if (count_ < frames_) { ++count_; }
    last_stamp_ = stamp;
    return tracking();
  }
  void invalidate()
  {
    if (tracking()) { ++epoch_; }
    count_ = 0;
  }
  bool tracking() const { return count_ == frames_; }
  std::uint32_t epoch() const { return epoch_; }

private:
  static bool positive(double value) { return std::isfinite(value) && value > 0.; }
  int frames_, count_ = 0;
  double max_gap_, max_speed_, max_angular_speed_;
  std::int64_t last_stamp_ = 0;
  std::uint32_t epoch_ = 0;
};
}  // namespace atd_orbslam3
