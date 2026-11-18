#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace atd_rc_input
{
inline constexpr std::array<const char *, 3> role_names{"recording", "auxiliary_1", "auxiliary_2"};
inline constexpr size_t channel_count{16};
inline constexpr int first_auxiliary_channel{4};
enum class Position {unknown, low, high, middle, unassigned};
inline const char * name(Position value)
{
  switch (value) {
    case Position::low: return "low";
    case Position::high: return "high";
    case Position::middle: return "middle";
    case Position::unassigned: return "unassigned";
    default: return "unknown";
  }
}

struct Binding
{
  int channel{0}, low{700}, high{1300};
  bool inverted{false};
  Position position(uint16_t value) const
  {
    if (!channel) {return Position::unassigned;}
    if (value <= low) {return inverted ? Position::high : Position::low;}
    if (value >= high) {return inverted ? Position::low : Position::high;}
    return Position::middle;
  }
};

struct Profile
{
  int debounce_ms{100};
  std::array<Binding, 3> roles{};
  void validate() const
  {
    if (debounce_ms < 0 || debounce_ms > 1000) {throw std::invalid_argument("invalid debounce_ms");}
    for (size_t i = 0; i < roles.size(); ++i) {
      const auto & b = roles[i];
      if ((b.channel != 0 && (b.channel < first_auxiliary_channel || b.channel > static_cast<int>(channel_count))) ||
        b.low < 0 || b.low >= b.high || b.high > 2047)
      {
        throw std::invalid_argument("invalid auxiliary channel or thresholds");
      }
      for (size_t j = 0; j < i; ++j) {
        if (b.channel && b.channel == roles[j].channel) {throw std::invalid_argument("duplicate channel");}
      }
    }
  }
};

struct Sample
{
  std::string instance_id, serial;
  uint32_t sequence{0}, sbus_frames{0}, sbus_age_ms{0};
  uint64_t board_us{0};
  uint8_t flags{0};
  bool sbus_fresh{false}, rc_valid{false};
  double receipt_age_s{0};  // Pico receiver receipt -> callback delay.
  std::array<uint16_t, channel_count> channels{};
};

class Core
{
public:
  void profile(const Profile & value)
  {
    value.validate();
    profile_ = value;
    profile_valid_ = true;
    reset();
  }
  bool configure(bool enabled, double now)
  {
    if (!enabled && (!profile_valid_ || !fresh(now))) {return false;}
    configuring_ = enabled;
    reset();
    return true;
  }
  std::vector<uint8_t> input(const Sample & value, double now)
  {
    accepted_ = false;
    std::vector<uint8_t> events;
    if (!std::isfinite(now) || !std::isfinite(value.receipt_age_s) || value.receipt_age_s < 0 ||
      value.instance_id.empty() || value.serial.empty()) {invalidate(); return events;}
    for (auto channel : value.channels) {
      if (channel > 2047) {invalidate(); return events;}
    }
    if (!have_sample_ || value.instance_id != sample_.instance_id || value.serial != sample_.serial) {
      invalidate();
    } else {
      const uint32_t delta = value.sequence - sample_.sequence;
      if (delta == 0 || delta >= 0x80000000U) {return events;}
      if (value.board_us <= sample_.board_us) {invalidate(); return events;}
      if (now - received_at_ > 0.25 || now - progress_at_ > 0.05) {reset();}
    }
    const bool advanced = !have_sample_ || value.sbus_frames != sample_.sbus_frames;
    if (have_sample_ && advanced && static_cast<uint32_t>(value.sbus_frames - sample_.sbus_frames) >= 0x80000000U) {
      reset();
    }
    sample_ = value;
    accepted_ = true;
    received_at_ = now - value.receipt_age_s;
    have_sample_ = true;
    if (advanced) {progress_at_ = received_at_ - static_cast<double>(value.sbus_age_ms) / 1000.0;}
    if (!fresh(now) || configuring_ || !profile_valid_) {reset(); return events;}
    if (!advanced) {return events;}
    for (size_t i = 0; i < profile_.roles.size(); ++i) {
      const auto & b = profile_.roles[i];
      if (!b.channel) {continue;}
      const auto position = b.position(value.channels[b.channel - 1]);
      auto & state = states_[i];
      if (position == Position::middle) {state.candidate = Position::unknown; continue;}
      if (state.candidate != position) {state.candidate = position; state.since = now;}
      if (now - state.since + 1e-9 < profile_.debounce_ms / 1000.0) {continue;}
      if (position != state.stable) {
        const bool baseline = state.stable == Position::unknown;
        state.stable = position;
        if (!baseline && i == 0) {events.push_back(position == Position::high ? 1 : 2);}
      }
    }
    return events;
  }
  bool fresh(double now) const
  {
    return have_sample_ && std::isfinite(now) && now >= received_at_ && now - received_at_ <= 0.25 &&
           now >= progress_at_ && now - progress_at_ <= 0.05 && sample_.rc_valid && sample_.sbus_fresh &&
           sample_.sbus_age_ms <= 50 && (sample_.flags & 12U) == 0;
  }
  bool baseline() const
  {
    for (size_t i = 0; i < profile_.roles.size(); ++i) {
      if (profile_.roles[i].channel && states_[i].stable == Position::unknown) {return false;}
    }
    return true;
  }
  std::string reason(double now)
  {
    if (!fresh(now)) {reset();}
    if (configuring_) {return "configuring";}
    if (!profile_valid_) {return "profile_missing_or_invalid";}
    if (!have_sample_ || now - received_at_ > 0.25) {return "usb_unavailable";}
    if (!sample_.rc_valid || !sample_.sbus_fresh || (sample_.flags & 12U)) {return "rc_signal_invalid";}
    if (!fresh(now)) {return "rc_stale";}
    return baseline() ? "ready" : "baseline_pending";
  }
  bool configuring() const {return configuring_;}
  bool profile_valid() const {return profile_valid_;}
  const Profile & profile() const {return profile_;}
  const Sample & sample() const {return sample_;}
  bool have_sample() const {return have_sample_;}
  bool accepted() const {return accepted_;}
  double age(double now) const {return have_sample_ ? std::max(0.0, now - received_at_) : -1.0;}
  Position position(size_t role) const
  {
    const auto & b = profile_.roles.at(role);
    return b.channel && have_sample_ ? b.position(sample_.channels[b.channel - 1]) : Position::unassigned;
  }

private:
  struct State {Position stable{Position::unknown}, candidate{Position::unknown}; double since{0};};
  void reset() {states_ = {};}
  void invalidate() {have_sample_ = false; reset();}
  Profile profile_;
  Sample sample_;
  std::array<State, 3> states_{};
  double received_at_{0}, progress_at_{0};
  bool have_sample_{false}, profile_valid_{false}, configuring_{false}, accepted_{false};
};
}  // namespace atd_rc_input
