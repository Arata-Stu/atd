#pragma once

#include <array>
#include <charconv>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace atd_sensors
{
struct StreamRequest
{
  std::string stream;
  int width{}, height{}, fps{};
};

inline int positive(std::string_view text, int maximum)
{
  int value{};
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
  if (text.empty() || parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
    value <= 0 || value > maximum || text.front() < '0' || text.front() > '9')
  {
    throw std::invalid_argument("stream dimensions/FPS must be positive decimal integers");
  }
  return value;
}

// Video: infra1:640:480:90, motion: accel:200. Zero/ANY cannot hide unsupported requests.
inline StreamRequest parse_stream(std::string_view text)
{
  std::vector<std::string_view> fields;
  while (true) {
    const auto end = text.find(':');
    fields.push_back(text.substr(0, end));
    if (end == std::string_view::npos) {break;}
    text.remove_prefix(end + 1);
  }
  StreamRequest result;
  result.stream = fields.front();
  const bool motion = result.stream == "accel" || result.stream == "gyro";
  const bool video = result.stream == "infra1" || result.stream == "infra2" ||
    result.stream == "color" || result.stream == "depth";
  if ((!motion && !video) || fields.size() != (motion ? 2U : 4U)) {
    throw std::invalid_argument("expected infra1/infra2/color/depth:width:height:fps or accel/gyro:fps");
  }
  if (motion) {result.fps = positive(fields[1], 2000);}
  else {
    result.width = positive(fields[1], 16384);
    result.height = positive(fields[2], 16384);
    result.fps = positive(fields[3], 2000);
  }
  return result;
}

inline std::string json_string(std::string_view value)
{
  static constexpr char hex[] = "0123456789abcdef";
  std::string result = "\"";
  for (const unsigned char c : value) {
    if (c == '"' || c == '\\') {result += '\\'; result += static_cast<char>(c);}
    else if (c < 32U) {result += "\\u00"; result += hex[c >> 4U]; result += hex[c & 15U];}
    else {result += static_cast<char>(c);}
  }
  result += '"';
  return result;
}
}  // namespace atd_sensors
