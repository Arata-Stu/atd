#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>

namespace atd_localization
{
// One interval of body-frame forward speed. The Pico has a board clock for
// pulse counting and a host stamp for pairing with the camera IMU.
struct SpeedInterval
{
  int64_t start, end;
  double speed, variance;

  static SpeedInterval from_edges(int64_t start, int64_t end, uint32_t before, uint32_t after,
                                  uint64_t board_elapsed_us, double pulses_per_meter)
  {
    const double resolution = 1e6 / (pulses_per_meter * board_elapsed_us);
    return {start, end, static_cast<uint32_t>(after - before) * resolution,
            resolution * resolution / 6.};
  }

  static SpeedInterval from_model(int64_t start, int64_t end, double before, double after)
  {
    return {start, end, 0.5 * (before + after), 0.};
  }
};

// Planar state at the rear axle: x, y, heading, forward speed and yaw rate.
struct RpmImuFilter
{
  using Matrix = Eigen::Matrix<double, 5, 5>;
  Eigen::Matrix<double, 5, 1> state = Eigen::Matrix<double, 5, 1>::Zero();
  Matrix covariance = Matrix::Zero();

  RpmImuFilter()
  {
    covariance(3, 3) = 25.;
    covariance(4, 4) = 4.;
  }

  void step(double dt, double speed, double yaw_rate, double speed_variance,
            double gyro_variance, double speed_walk, double rate_walk)
  {
    // Rates are interval-average measurements. Estimate them before propagating
    // the pose over that same interval, including pose/rate cross covariance.
    covariance(3, 3) += dt * speed_walk * speed_walk;
    covariance(4, 4) += dt * rate_walk * rate_walk;
    Eigen::Matrix2d noise = Eigen::Vector2d(speed_variance, gyro_variance).asDiagonal();
    Eigen::Matrix2d innovation = covariance.bottomRightCorner<2, 2>() + noise;
    Eigen::Matrix<double, 5, 2> gain =
      innovation.ldlt().solve(covariance.rightCols<2>().transpose()).transpose();
    Eigen::Vector2d residual = Eigen::Vector2d(speed, yaw_rate) - state.tail<2>();
    state += gain * residual;
    Matrix update = Matrix::Identity();
    update.rightCols<2>() -= gain;
    covariance = (update * covariance * update.transpose() + gain * noise * gain.transpose()).eval();

    const double mid = state(2) + 0.5 * dt * state(4);
    const double c = std::cos(mid), s = std::sin(mid), distance = dt * state(3);
    Matrix jacobian = Matrix::Identity();
    jacobian(0, 2) = -distance * s;
    jacobian(1, 2) = distance * c;
    jacobian(0, 3) = dt * c;
    jacobian(1, 3) = dt * s;
    jacobian(0, 4) = 0.5 * dt * jacobian(0, 2);
    jacobian(1, 4) = 0.5 * dt * jacobian(1, 2);
    state(0) += distance * c;
    state(1) += distance * s;
    state(2) += dt * state(4);
    covariance = (jacobian * covariance * jacobian.transpose()).eval();
  }
};
}  // namespace atd_localization
