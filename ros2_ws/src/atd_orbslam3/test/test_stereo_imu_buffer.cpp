#include "../src/stereo_imu_buffer.hpp"
#include <stdexcept>

using atd_orbslam3::StereoImuBuffer;
constexpr std::int64_t ms = 1000000;

void check(bool condition)
{
  if (!condition) { throw std::runtime_error("unexpected stereo/IMU window"); }
}

void push(StereoImuBuffer & buffer, int time_ms)
{
  const auto stamp = time_ms * ms;
  buffer.push(stamp, {1.f, 2.f, 3.f, 4.f, 5.f, 6.f, stamp * 1e-9});
}

int main()
{
  StereoImuBuffer imu;
  push(imu, 3);
  check(imu.take(2 * ms, 0, 400.).empty());  // Image before IMU starts.
  check(imu.take(3 * ms, 0, 400.).empty());  // A single initial sample cannot integrate.
  push(imu, 6);
  push(imu, 6);  // A retransmitted or late sample is not new evidence.
  push(imu, 5);
  push(imu, 16);
  auto initial = imu.take(10 * ms, 0, 400.);
  check(initial.size() == 2 && initial.front().t == 3 * ms * 1e-9);
  check(imu.take(20 * ms, 10 * ms, 400.).empty());  // Just the 16 ms sample.
  push(imu, 29);
  push(imu, 31);
  auto recovered = imu.take(29 * ms, 10 * ms, 400.);
  check(recovered.size() == 2 && recovered.front().t == 16 * ms * 1e-9);
  check(recovered.back().t == 29 * ms * 1e-9 && recovered.back().w.z() == 6.f);
  // The core retains the 29 ms boundary. Do not resend it with the next real sample.
  auto next = imu.take(31 * ms, 29 * ms, 400.);
  check(next.size() == 1 && next.front().t == 31 * ms * 1e-9);
  check(imu.take(40 * ms, 31 * ms, 400.).empty());
  push(imu, 90);
  try {
    imu.take(90 * ms, 31 * ms, 400.);
    return 1;
  } catch (const std::runtime_error &) {}

  // Two samples both inside the core's terminal 1 ms window count as only one.
  StereoImuBuffer burst;
  burst.push(1900000, {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, .0019});
  burst.push(1950000, {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, .00195});
  check(burst.take(2 * ms, 0, 400.).empty());
  check(burst.take(4 * ms, 0, 400.).size() == 2);
}
