#include "atd_sensors/probe_request.hpp"

#include <cstdlib>
#include <iostream>

void check(bool condition)
{
  if (!condition) {std::cerr << "D455 probe request check failed\n"; std::exit(1);}
}

int main()
{
  using namespace atd_sensors;
  const auto infrared = parse_stream("infra1:640:480:90");
  check(infrared.stream == "infra1" && infrared.width == 640 && infrared.height == 480 && infrared.fps == 90);
  const auto accel = parse_stream("accel:200");
  check(accel.stream == "accel" && !accel.width && !accel.height && accel.fps == 200);
  for (const auto * value : {"depth:0:0:0", "infra1:640:480", "infra3:640:480:90", "gyro:-1",
    "color:640:480:90:extra", "accel:ANY", "accel:2001", "depth:999999999999999999:480:30", ""})
  {
    bool rejected = false;
    try {parse_stream(value);} catch (const std::invalid_argument &) {rejected = true;}
    check(rejected);
  }
  check(json_string("serial\"\\\n") == "\"serial\\\"\\\\\\u000a\"");
  std::cout << "D455 exact request/range/JSON escaping tests passed\n";
}
