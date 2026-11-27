#include "../src/rpm_imu_filter.hpp"
#include <cmath>
#include <iostream>

int main()
{
  // Four edges including a uint32 wrap in 50 ms, at 80 pulses/m.
  // The ROS interval differs slightly from the board clock.
  using atd_localization::SpeedInterval;
  const auto rpm = SpeedInterval::from_edges(100, 52000100, UINT32_MAX-2, 1, 50000, 80.);
  const auto model = SpeedInterval::from_model(100, 52000100, .75, 1.25);
  if (rpm.speed != 1. || model.speed != rpm.speed || model.variance != 0. || rpm.variance <= 0.) {
    std::cerr << "interval speed conversion failed\n";
    return 1;
  }
  atd_localization::RpmImuFilter filter;
  for (int i = 0; i < 100; ++i) {
    filter.step(.02, 1., 0., .01, .001, 2., 1.);
  }
  if (std::abs(filter.state(0) - 2.) > .002 || std::abs(filter.state(1)) > .001) {
    std::cerr << "straight trajectory failed\n";
    return 1;
  }
  for (int i = 0; i < 100; ++i) {
    filter.step(.02, 1., 1., .01, .001, 2., 1.);
  }
  Eigen::SelfAdjointEigenSolver<atd_localization::RpmImuFilter::Matrix> eig(filter.covariance);
  if (std::abs(filter.state(0) - 2. - std::sin(2.)) > .01 ||
      std::abs(filter.state(1) - (1. - std::cos(2.))) > .01 ||
      std::abs(filter.state(2) - 2.) > .01 || eig.eigenvalues().minCoeff() < -1e-10) {
    std::cerr << "turn trajectory or covariance failed\n";
    return 1;
  }
  return 0;
}
