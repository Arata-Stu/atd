#pragma once

#include <algorithm>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <vector>
#include <ImuTypes.h>

namespace atd_orbslam3 {

class StereoImuBuffer {
public:
  using Point = ORB_SLAM3::IMU::Point;

  std::int64_t latest_stamp() const { return latest_; }

  void push(std::int64_t stamp, const Point & point)
  {
    if (stamp <= latest_) { return; }
    latest_ = stamp;
    samples_.push_back({stamp, point});
    if (samples_.size() > 4096) { samples_.pop_front(); }
  }

  // The caller waits until latest_stamp() reaches this image. A skipped image
  // leaves every sample in the buffer for the next stereo interval.
  std::vector<Point> take(std::int64_t stamp, std::int64_t previous_image, double hz)
  {
    const double gap = std::max(.05, 4. / hz);
    auto last = last_sent_;
    auto end = samples_.begin();
    std::vector<Point> points;
    for (; end != samples_.end() && end->stamp <= stamp; ++end) {
      if (previous_image && (end->stamp - last) * 1e-9 > gap) {
        throw std::runtime_error("gap in combined IMU observations during stereo-inertial tracking");
      }
      if (previous_image || (stamp - end->stamp) * 1e-9 <= gap) { points.push_back(end->point); }
      last = end->stamp;
    }
    if (!last) { return {}; }  // Stereo images can precede the first IMU observation.
    if ((stamp - last) * 1e-9 > gap) {
      throw std::runtime_error("combined IMU does not cover the stereo observation");
    }
    // Pinned Tracking::PreintegrateIMU keeps its first sample at or after
    // image time - 1 ms. It needs a distinct earlier sample to integrate.
    const double boundary = stamp * 1e-9 - .001;
    const bool carried = has_boundary_ && last_sent_ * 1e-9 < boundary;
    if (points.empty() || (!carried && (points.size() < 2 || points.front().t >= boundary))) {
      return {};
    }
    samples_.erase(samples_.begin(), end);
    last_sent_ = last;
    // On the first call the core has no previous stereo frame.
    has_boundary_ = previous_image && points.back().t >= boundary;
    return points;
  }

private:
  struct Sample { std::int64_t stamp; Point point; };
  std::deque<Sample> samples_;
  std::int64_t latest_ = 0, last_sent_ = 0;
  bool has_boundary_ = false;
};
}  // namespace atd_orbslam3
