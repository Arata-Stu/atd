#pragma once

#include <array>
#include <charconv>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>

namespace atd_vehicle_interface
{
inline constexpr uint32_t kRcFreshnessMs = 50;
inline constexpr double kRcFreshnessSeconds = static_cast<double>(kRcFreshnessMs) / 1000.0;

// 前回からRCの有効期限を超えて止まった受信ループでは、滞留frameを採用しない。
// previous は接続時刻、以後は各pollの開始時刻を使う。
inline bool polling_gap_expired(double previous, double now)
{
  return now < previous || now > previous + kRcFreshnessSeconds;
}

inline bool supported_telemetry_info(std::string_view line)
{
  const bool known_firmware = line.starts_with("JPBB Rev6 Pico v0.1;") ||
    line.starts_with("JPBB Rev6 Pico v0.2.0-led;");
  return known_firmware && line.find("; CDC1=JPX1;") != std::string_view::npos;
}

struct Telemetry
{
  uint32_t sequence{};
  uint64_t board_us{};
  uint32_t sbus_age_ms{}, sbus_frames{}, sbus_lost_frames{}, sbus_failsafe_frames{}, uart_errors{};
  uint8_t sbus_flags{};
  std::array<uint16_t, 16> channels{};
  uint32_t rpm_edges{}, rpm_last_us32{}, rpm_period_us32{};
  bool rpm_period_recent{}, rpm_seen{};
  uint16_t vbec_mv{};
  bool power_ready{}, sbus_fresh{};
  uint8_t active_path{};
  uint32_t fault_bits{}, tx0_dropped{}, tx1_dropped{}, rx0_rejected{}, rx1_rejected{};
};

inline uint16_t crc16(std::string_view text)
{
  uint16_t crc = 0xffffU;
  for (const unsigned char byte : text) {
    crc ^= static_cast<uint16_t>(byte << 8U);
    for (unsigned bit = 0; bit < 8; ++bit) {
      crc = static_cast<uint16_t>((crc & 0x8000U) ? (crc << 1U) ^ 0x1021U : crc << 1U);
    }
  }
  return crc;
}

inline bool decimal(std::string_view text, uint64_t maximum, uint64_t & value)
{
  if (text.empty()) {return false;}
  for (const char c : text) {if (c < '0' || c > '9') {return false;}}
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  return result.ec == std::errc{} && result.ptr == text.data() + text.size() && value <= maximum;
}

// JPX1,T はCRCを除いて40フィールド。別形式を推測で補完しない。
inline std::optional<Telemetry> parse_telemetry(std::string_view line)
{
  if (!line.empty() && line.back() == '\n') {line.remove_suffix(1);}
  if (!line.empty() && line.back() == '\r') {line.remove_suffix(1);}
  if (line.empty() || line.size() >= 512) {return std::nullopt;}
  const auto delimiter = line.rfind(',');
  if (delimiter == std::string_view::npos || line.size() - delimiter != 5) {return std::nullopt;}
  const auto crc_text = line.substr(delimiter + 1);
  unsigned received{};
  for (const char c : crc_text) {
    unsigned digit{};
    if (c >= '0' && c <= '9') {digit = static_cast<unsigned>(c - '0');}
    else if (c >= 'a' && c <= 'f') {digit = static_cast<unsigned>(c - 'a' + 10);}
    else if (c >= 'A' && c <= 'F') {digit = static_cast<unsigned>(c - 'A' + 10);}
    else {return std::nullopt;}
    received = (received << 4U) | digit;
  }
  const auto payload = line.substr(0, delimiter);
  if (received != crc16(payload)) {return std::nullopt;}
  std::array<std::string_view, 40> fields{};
  size_t count{}, start{};
  while (start <= payload.size()) {
    if (count == fields.size()) {return std::nullopt;}
    const auto end = payload.find(',', start);
    fields[count++] = payload.substr(start, end == std::string_view::npos ? end : end - start);
    if (end == std::string_view::npos) {break;}
    start = end + 1;
  }
  if (count != fields.size() || fields[0] != "JPX1" || fields[1] != "T") {return std::nullopt;}
  std::array<uint64_t, 40> values{};
  for (size_t i = 2; i < fields.size(); ++i) {
    uint64_t maximum = std::numeric_limits<uint32_t>::max();
    if (i == 3) {maximum = std::numeric_limits<uint64_t>::max();}
    if (i == 9) {maximum = 15;}
    if (i >= 10 && i < 26) {maximum = 2047;}
    if (i == 29 || i == 30 || i == 32 || i == 33) {maximum = 1;}
    if (i == 31) {maximum = 65535;}
    if (i == 34) {maximum = 3;}
    if (!decimal(fields[i], maximum, values[i])) {return std::nullopt;}
  }
  Telemetry result;
  result.sequence = static_cast<uint32_t>(values[2]);
  result.board_us = values[3];
  result.sbus_age_ms = static_cast<uint32_t>(values[4]);
  result.sbus_frames = static_cast<uint32_t>(values[5]);
  result.sbus_lost_frames = static_cast<uint32_t>(values[6]);
  result.sbus_failsafe_frames = static_cast<uint32_t>(values[7]);
  result.uart_errors = static_cast<uint32_t>(values[8]);
  result.sbus_flags = static_cast<uint8_t>(values[9]);
  for (size_t i = 0; i < result.channels.size(); ++i) {result.channels[i] = static_cast<uint16_t>(values[i + 10]);}
  result.rpm_edges = static_cast<uint32_t>(values[26]);
  result.rpm_last_us32 = static_cast<uint32_t>(values[27]);
  result.rpm_period_us32 = static_cast<uint32_t>(values[28]);
  result.rpm_period_recent = values[29] != 0;
  result.rpm_seen = values[30] != 0;
  result.vbec_mv = static_cast<uint16_t>(values[31]);
  result.power_ready = values[32] != 0;
  result.sbus_fresh = values[33] != 0;
  result.active_path = static_cast<uint8_t>(values[34]);
  result.fault_bits = static_cast<uint32_t>(values[35]);
  result.tx0_dropped = static_cast<uint32_t>(values[36]);
  result.tx1_dropped = static_cast<uint32_t>(values[37]);
  result.rx0_rejected = static_cast<uint32_t>(values[38]);
  result.rx1_rejected = static_cast<uint32_t>(values[39]);
  return result;
}

class Lines
{
public:
  template<class Callback>
  void feed(const char * bytes, size_t size, Callback ready)
  {
    for (size_t i = 0; i < size; ++i) {
      const auto c = static_cast<unsigned char>(bytes[i]);
      if (c == '\r' || c == '\n') {
        if (dropping_) {++discarded_;}
        else if (size_) {ready(std::string_view(buffer_.data(), size_));}
        size_ = 0;
        dropping_ = false;
      } else if (!dropping_) {
        if (c < 32 || c > 126 || size_ == buffer_.size()) {
          dropping_ = true;
          size_ = 0;
        } else {buffer_[size_++] = static_cast<char>(c);}
      }
    }
  }
  uint64_t discarded() const {return discarded_;}
private:
  std::array<char, 511> buffer_{};
  size_t size_{};
  bool dropping_{};
  uint64_t discarded_{};
};

inline bool advancing(uint32_t current, uint32_t previous)
{
  const uint32_t delta = current - previous;
  return delta != 0 && delta < 0x80000000U;
}

class Progress
{
public:
  enum class Result {accepted, reset, rejected};
  Result accept(const Telemetry & sample, double now)
  {
    bool reset = have_ && sample.board_us < last_.board_us;
    if (have_ && !reset && (sample.board_us <= last_.board_us || !advancing(sample.sequence, last_.sequence))) {
      return Result::rejected;
    }
    if (reset) {have_rc_ = false;}
    const bool changed = !have_ || reset || advancing(sample.sbus_frames, last_.sbus_frames);
    // テレメトリの再配信では、RC入力そのものの期限を延長しない。
    if (changed && sample.sbus_frames && sample.sbus_fresh && !(sample.sbus_flags & 12U) && sample.sbus_age_ms <= kRcFreshnessMs) {
      rc_at_ = now - static_cast<double>(sample.sbus_age_ms) / 1000.0;
      have_rc_ = true;
    }
    if (!sample.sbus_fresh || (sample.sbus_flags & 12U) || sample.sbus_age_ms > kRcFreshnessMs) {have_rc_ = false;}
    last_ = sample;
    have_ = true;
    at_ = now;
    return reset ? Result::reset : Result::accepted;
  }
  bool fresh(double now, double timeout) const {return have_ && now >= at_ && now - at_ <= timeout;}
  bool rc_valid(double now, double timeout) const
  {
    return fresh(now, timeout) && have_rc_ && now >= rc_at_ && now - rc_at_ <= kRcFreshnessSeconds;
  }
  std::optional<double> rc_age(double now) const
  {
    if (!have_rc_ || now < rc_at_) {return std::nullopt;}
    return now - rc_at_;
  }
  const Telemetry & last() const {return last_;}
  double received_at() const {return at_;}
  bool have() const {return have_;}
private:
  Telemetry last_{};
  bool have_{}, have_rc_{};
  double at_{}, rc_at_{};
};
}  // namespace atd_vehicle_interface
