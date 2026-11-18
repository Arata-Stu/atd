#include "atd_sensors/probe_request.hpp"

#include <algorithm>
#include <iostream>
#include <set>
#include <tuple>

#include <librealsense2/rs.hpp>

namespace
{
using atd_sensors::json_string;
using atd_sensors::StreamRequest;

std::string info(const rs2::device & device, rs2_camera_info key)
{
  return device.supports(key) ? device.get_info(key) : "";
}

bool is_d455(const rs2::device & device)
{
  return info(device, RS2_CAMERA_INFO_NAME).find("D455") != std::string::npos;
}

std::string stream_name(const rs2::stream_profile & profile)
{
  if (profile.stream_type() == RS2_STREAM_INFRARED) {
    if (profile.stream_index() == 1) {return "infra1";}
    if (profile.stream_index() == 2) {return "infra2";}
  }
  if (profile.stream_type() == RS2_STREAM_COLOR) {return "color";}
  if (profile.stream_type() == RS2_STREAM_DEPTH) {return "depth";}
  if (profile.stream_type() == RS2_STREAM_ACCEL) {return "accel";}
  if (profile.stream_type() == RS2_STREAM_GYRO) {return "gyro";}
  return "";
}

rs2_stream stream_type(const std::string & key)
{
  if (key == "infra1" || key == "infra2") {return RS2_STREAM_INFRARED;}
  if (key == "color") {return RS2_STREAM_COLOR;}
  if (key == "depth") {return RS2_STREAM_DEPTH;}
  return key == "accel" ? RS2_STREAM_ACCEL : RS2_STREAM_GYRO;
}

rs2_format stream_format(const std::string & key)
{
  if (key == "infra1" || key == "infra2") {return RS2_FORMAT_Y8;}
  if (key == "color") {return RS2_FORMAT_RGB8;}
  if (key == "depth") {return RS2_FORMAT_Z16;}
  return RS2_FORMAT_MOTION_XYZ32F;
}

std::vector<rs2::stream_profile> profiles(const rs2::device & device)
{
  std::vector<rs2::stream_profile> result;
  for (const auto & sensor : device.query_sensors()) {
    const auto streams = sensor.get_stream_profiles();
    result.insert(result.end(), streams.begin(), streams.end());
  }
  return result;
}

bool matches(const rs2::stream_profile & profile, const StreamRequest & request)
{
  if (stream_name(profile) != request.stream || profile.format() != stream_format(request.stream) ||
    profile.fps() != request.fps) {return false;}
  if (request.width) {
    const auto video = profile.as<rs2::video_stream_profile>();
    return video && video.width() == request.width && video.height() == request.height;
  }
  return true;
}

void print_device(const rs2::device & device)
{
  std::cout << "{\"serial\":" << json_string(info(device, RS2_CAMERA_INFO_SERIAL_NUMBER))
            << ",\"name\":" << json_string(info(device, RS2_CAMERA_INFO_NAME))
            << ",\"is_d455\":" << (is_d455(device) ? "true" : "false")
            << ",\"usb_type\":" << json_string(info(device, RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR))
            << ",\"firmware\":" << json_string(info(device, RS2_CAMERA_INFO_FIRMWARE_VERSION))
            << ",\"profiles\":[";
  using Key = std::tuple<std::string, std::string, int, int, int>;
  std::set<Key> unique;
  for (const auto & profile : profiles(device)) {
    const auto key = stream_name(profile);
    if (key.empty()) {continue;}
    const auto video = profile.as<rs2::video_stream_profile>();
    unique.emplace(key, rs2_format_to_string(profile.format()), video ? video.width() : 0,
      video ? video.height() : 0, profile.fps());
  }
  bool first = true;
  for (const auto & [stream, format, width, height, fps] : unique) {
    if (!first) {std::cout << ',';}
    first = false;
    std::cout << "{\"stream\":" << json_string(stream) << ",\"format\":" << json_string(format)
              << ",\"width\":" << width << ",\"height\":" << height << ",\"fps\":" << fps << '}';
  }
  std::cout << "]}";
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    std::string serial;
    bool validate = false;
    std::vector<StreamRequest> requests;
    for (int i = 1; i < argc; ++i) {
      const std::string argument = argv[i];
      if (argument == "--serial" && i + 1 < argc) {serial = argv[++i];}
      else if (argument == "--stream" && i + 1 < argc) {requests.push_back(atd_sensors::parse_stream(argv[++i]));}
      else if (argument == "--validate") {validate = true;}
      else if (argument == "--list") {}
      else if (argument == "--help") {
        std::cout << "atd_realsense_probe [--list] [--serial SERIAL] [--validate --stream infra1:640:480:90 ...]\n";
        return 0;
      } else {throw std::invalid_argument("unknown or incomplete argument: " + argument);}
    }
    if (validate && serial.empty()) {throw std::invalid_argument("--validate requires an explicit --serial");}
    if (!validate && !requests.empty()) {throw std::invalid_argument("--stream requires --validate");}
    rs2::context context;
    const auto devices = context.query_devices();
    if (!validate) {
      std::cout << "{\"schema_version\":1,\"devices\":[";
      bool first = true;
      for (const auto & device : devices) {
        if (!serial.empty() && info(device, RS2_CAMERA_INFO_SERIAL_NUMBER) != serial) {continue;}
        if (!first) {std::cout << ',';}
        first = false;
        print_device(device);
      }
      std::cout << "]}\n";
      return 0;
    }
    rs2::device selected;
    for (const auto & device : devices) {
      if (info(device, RS2_CAMERA_INFO_SERIAL_NUMBER) == serial) {selected = device; break;}
    }
    if (!selected || !is_d455(selected)) {throw std::invalid_argument("selected D455 is not connected: " + serial);}
    const auto available = profiles(selected);
    std::set<std::string> requested;
    rs2::config config;
    config.disable_all_streams();
    config.enable_device(serial);
    for (const auto & request : requests) {
      if (!requested.insert(request.stream).second) {throw std::invalid_argument("duplicate stream: " + request.stream);}
      if (std::none_of(available.begin(), available.end(), [&](const auto & p) {return matches(p, request);})) {
        throw std::invalid_argument("unsupported requested stream: " + request.stream + " " +
          std::to_string(request.width) + "x" + std::to_string(request.height) + "x" + std::to_string(request.fps));
      }
      const int index = request.stream == "infra1" ? 1 : request.stream == "infra2" ? 2 : 0;
      config.enable_stream(stream_type(request.stream), index, request.width, request.height,
        stream_format(request.stream), request.fps);
    }
    // All-OFF launches no camera node. Resolving an empty config would enable SDK defaults.
    rs2::pipeline pipeline(context);
    const bool resolves = requests.empty() || config.can_resolve(pipeline);
    std::cout << "{\"can_resolve\":" << (resolves ? "true" : "false")
              << ",\"all_streams_off\":" << (requests.empty() ? "true" : "false")
              << ",\"serial\":" << json_string(serial) << "}\n";
    return resolves ? 0 : 2;
  } catch (const std::exception & error) {
    std::cerr << "D455 SDK query: " << error.what() << '\n';
    return 1;
  }
}
