#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace atd_teleop
{
inline constexpr std::array<const char *, 8> axis_names{
  "left_x", "left_y", "right_x", "right_y", "l2", "r2", "dpad_x", "dpad_y"};
inline constexpr std::array<const char *, 17> button_names{
  "cross", "circle", "triangle", "square", "l1", "r1", "l2", "r2",
  "create", "options", "ps", "l3", "r3", "dpad_up", "dpad_down", "dpad_left", "dpad_right"};

struct Axis
{
  int axis{-1}, subtract{-1};
  double scale{1.0};
  bool assigned() const {return axis >= 0 || subtract >= 0;}
  void validate() const
  {
    if (axis < -1 || axis >= 8 || subtract < -1 || subtract >= 8 ||
      !std::isfinite(scale) || scale < -1.0 || scale > 1.0)
    {
      throw std::invalid_argument("invalid axis assignment");
    }
  }
  float map(const std::vector<float> & values) const
  {
    const double value = axis < 0 ? 0.0 : values[axis];
    const double negative = subtract < 0 ? 0.0 : values[subtract];
    return static_cast<float>(std::clamp((value - negative) * scale, -1.0, 1.0));
  }
};

struct Profile
{
  Axis steering, throttle;
  int enable{-1}, record_start{-1}, record_stop{-1};
  bool assigned() const {return steering.assigned() && throttle.assigned();}
  bool drive_requested(bool valid, bool enable_pressed) const
  {
    return valid && (enable < 0 || enable_pressed);
  }
  bool events_assigned() const {return record_start >= 0 || record_stop >= 0;}
  bool usable() const {return assigned() || events_assigned();}
  void validate() const
  {
    steering.validate();
    throttle.validate();
    const std::array<int, 3> buttons{enable, record_start, record_stop};
    for (size_t i = 0; i < buttons.size(); ++i) {
      if (buttons[i] < -1 || buttons[i] >= 17) {throw std::invalid_argument("invalid button assignment");}
      for (size_t j = i + 1; j < buttons.size(); ++j) {
        if (buttons[i] >= 0 && buttons[i] == buttons[j]) {
          throw std::invalid_argument("button roles must be distinct");
        }
      }
    }
  }
};

struct Output
{
  int64_t stamp_ns{0};
  bool valid{false};
  float steering{0.0F}, throttle{0.0F};
  bool enable_requested{false};
};

// ROS 時刻と watchdog の steady 時刻を分離する。steady 時刻を注入できるため
// ROS/デバイスを用意せずに途絶・設定中・再接続の振る舞いを検証できる。
class Core
{
public:
  explicit Core(double input_timeout = 0.25, double status_timeout = 2.5)
  : input_timeout_(input_timeout), status_timeout_(status_timeout)
  {
    if (!std::isfinite(input_timeout_) || input_timeout_ <= 0.0 ||
      !std::isfinite(status_timeout_) || status_timeout_ <= 0.0)
    {
      throw std::invalid_argument("timeouts must be positive finite seconds");
    }
  }

  void set_profile(const Profile & profile)
  {
    profile.validate();
    profile_ = profile;
    have_profile_ = true;
    reset();
    have_input_ = false;
  }

  void source(const std::string & session, bool available, double now)
  {
    if (!have_source_ || session != session_) {
      reset();
      have_input_ = false;
      have_ready_ = false;
      session_ = session;
    }
    have_source_ = true;
    source_available_ = available;
    source_at_ = now;
    if (!available) {reset();}
  }

  void ready(bool value, double now)
  {
    have_ready_ = true;
    ready_ = value;
    ready_at_ = now;
    if (!value) {reset();}
  }

  // false は現在の全入力解放を確認して初めて受け入れる。
  bool configure(bool enabled, double now)
  {
    if (!enabled && (!available(now, false) || !released())) {return false;}
    configuring_ = enabled;
    reset();
    return true;
  }

  bool configuring() const {return configuring_;}
  bool profile_valid() const {return have_profile_;}
  const Profile & profile() const {return profile_;}
  bool events_ready(double now) const {return profile_.events_assigned() && available(now) && armed_;}

  std::vector<uint8_t> input(
    const std::vector<float> & axes, const std::vector<int32_t> & buttons,
    int64_t stamp_ns, double now)
  {
    std::vector<uint8_t> events;
    if (!shape_valid(axes, buttons) || stamp_ns < 0) {
      have_input_ = false;
      reset();
      return events;
    }
    // 同じ入力を別 node が再配信しても、入力期限を更新しない。
    if (have_input_ && stamp_ns <= input_stamp_) {return events;}
    // callback の停止期間に timer が動かなかった場合も履歴を破棄する。
    if (have_input_ && now - input_at_ > input_timeout_) {reset();}
    axes_ = axes;
    buttons_ = buttons;
    input_stamp_ = stamp_ns;
    input_at_ = now;
    have_input_ = true;
    if (!available(now)) {reset(); return events;}
    if (!armed_) {
      if (released()) {armed_ = true; previous_buttons_ = buttons_;}
      return events;
    }
    const auto pressed = [this](int index) {
        return index >= 0 && buttons_[index] && !previous_buttons_[index];
      };
    if (pressed(profile_.record_start)) {events.push_back(1);}
    if (pressed(profile_.record_stop)) {events.push_back(2);}
    previous_buttons_ = buttons_;
    return events;
  }

  Output output(double now)
  {
    Output result;
    result.stamp_ns = have_input_ ? input_stamp_ : 0;
    if (!available(now)) {reset(); return result;}
    if (!armed_ || !profile_.assigned()) {return result;}
    result.valid = true;
    result.steering = profile_.steering.map(axes_);
    result.throttle = profile_.throttle.map(axes_);
    result.enable_requested = profile_.enable >= 0 && buttons_[profile_.enable];
    return result;
  }

  std::string reason(double now) const
  {
    if (configuring_) {return "configuring";}
    if (!have_profile_) {return "profile_missing_or_invalid";}
    if (!profile_.assigned()) {return "steering_and_throttle_are_unassigned";}
    return input_reason(now);
  }

  std::string events_reason(double now) const
  {
    if (configuring_) {return "configuring";}
    if (!have_profile_) {return "profile_missing_or_invalid";}
    if (!profile_.events_assigned()) {return "recording_buttons_are_unassigned";}
    return input_reason(now);
  }

  static bool shape_valid(const std::vector<float> & axes, const std::vector<int32_t> & buttons)
  {
    return axes.size() == axis_names.size() && buttons.size() == button_names.size() &&
           axes[4] >= 0.0F && axes[5] >= 0.0F &&
           std::all_of(axes.begin(), axes.end(), [](float v) {return std::isfinite(v) && std::abs(v) <= 1.0F;}) &&
           std::all_of(buttons.begin(), buttons.end(), [](int32_t v) {return v == 0 || v == 1;});
  }

private:
  std::string input_reason(double now) const
  {
    if (!std::isfinite(now) || !have_source_ || !source_available_ || now < source_at_ || now - source_at_ > status_timeout_) {
      return "joy_status_unavailable";
    }
    if (!have_ready_ || !ready_ || now < ready_at_ || now - ready_at_ > input_timeout_) {return "joy_not_ready";}
    if (!have_input_ || now < input_at_ || now - input_at_ > input_timeout_) {return "joy_input_unavailable";}
    if (!armed_) {return "release_all_controls";}
    return "ready";
  }
  bool available(double now, bool check_configuring = true) const
  {
    return (!check_configuring || !configuring_) && have_profile_ && profile_.usable() &&
           have_source_ && source_available_ && now >= source_at_ && now - source_at_ <= status_timeout_ &&
           have_ready_ && ready_ && now >= ready_at_ && now - ready_at_ <= input_timeout_ &&
           have_input_ && now >= input_at_ && now - input_at_ <= input_timeout_;
  }
  bool released() const
  {
    return std::all_of(axes_.begin(), axes_.end(), [](float v) {return std::abs(v) <= 0.1F;}) &&
           std::none_of(buttons_.begin(), buttons_.end(), [](int32_t v) {return v != 0;});
  }
  void reset()
  {
    armed_ = false;
    previous_buttons_.assign(button_names.size(), 0);
  }
  Profile profile_;
  double input_timeout_, status_timeout_;
  double input_at_{0.0}, ready_at_{0.0}, source_at_{0.0};
  bool have_profile_{false}, configuring_{false}, armed_{false};
  bool have_source_{false}, source_available_{false}, have_ready_{false}, ready_{false}, have_input_{false};
  std::string session_;
  int64_t input_stamp_{0};
  std::vector<float> axes_;
  std::vector<int32_t> buttons_, previous_buttons_ = std::vector<int32_t>(button_names.size(), 0);
};
}  // namespace atd_teleop
