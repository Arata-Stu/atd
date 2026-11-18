#include "atd_teleop/core.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using atd_teleop::Core;
using atd_teleop::Profile;

void check(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

struct Rig
{
  Core core;
  double now{0.0};
  int64_t stamp{1};
  std::vector<float> axes = std::vector<float>(8, 0.0F);
  std::vector<int32_t> buttons = std::vector<int32_t>(17, 0);
  Rig()
  {
    Profile profile;
    profile.steering = {0, -1, -1.0};
    profile.throttle = {5, 4, 0.6};
    profile.enable = 5;
    profile.record_start = 0;
    profile.record_stop = 1;
    core.set_profile(profile);
    core.source("process-a:1", true, now);
    core.ready(true, now);
  }
  auto send()
  {
    core.ready(true, now);
    return core.input(axes, buttons, stamp++, now);
  }
};

void mapping_and_limits()
{
  Rig rig;
  check(rig.send().empty(), "neutral initialization must have no events");
  auto out = rig.core.output(rig.now);
  check(out.valid && !out.enable_requested && out.stamp_ns == 1, "neutral may initialize input availability");
  rig.axes[0] = .7F;
  rig.axes[5] = .9F;
  rig.axes[4] = .4F;
  rig.buttons[5] = 1;
  rig.send();
  out = rig.core.output(rig.now);
  check(out.valid && std::abs(out.steering + .7F) < 1e-6F, "axis inversion");
  check(std::abs(out.throttle - .3F) < 1e-6F && out.enable_requested, "trigger difference and enable request");
  Profile profile = rig.core.profile();
  profile.throttle.scale = 1.0;
  profile.throttle.axis = 0;
  profile.throttle.subtract = 1;
  rig.core.set_profile(profile);
  rig.axes.assign(8, 0.0F);
  rig.buttons.assign(17, 0);
  rig.send();
  rig.axes[0] = 1.0F;
  rig.axes[1] = -1.0F;
  rig.send();
  check(rig.core.output(rig.now).throttle == 1.0F, "subtracted full range must clamp");
}

void events_are_edges_and_require_release()
{
  Rig rig;
  rig.buttons[0] = 1;
  check(rig.send().empty() && !rig.core.output(0).valid, "startup held button must not be an event");
  rig.buttons[0] = 0;
  check(rig.send().empty() && rig.core.output(0).valid, "release enables input availability");
  rig.buttons[0] = 1;
  check(rig.send() == std::vector<uint8_t>{1}, "start press once");
  check(rig.send().empty(), "holding must not repeat");
  rig.buttons[0] = 0;
  rig.send();
  rig.buttons[1] = 1;
  check(rig.send() == std::vector<uint8_t>{2}, "stop press once");
  rig.core.ready(false, 0);
  check(!rig.core.output(0).valid && rig.send().empty(), "disconnect held button must not restart");
  check(!rig.core.output(0).valid, "reconnect requires release");
}

void timeouts_do_not_refresh_stale_input()
{
  Rig rig;
  rig.send();
  check(rig.core.output(.2).valid && rig.core.output(.2).stamp_ns == 1, "timer must retain input stamp");
  rig.core.ready(true, .3);
  check(!rig.core.output(.3).valid, "fresh ready cannot revive old input");
  rig.core.input(rig.axes, rig.buttons, 1, .3);
  check(!rig.core.output(.3).valid, "duplicate input stamp cannot renew freshness");
  rig.now = .3;
  rig.buttons[0] = 1;
  check(rig.send().empty() && !rig.core.output(.3).valid, "timeout resets edge/held state");
  rig.buttons[0] = 0;
  rig.send();
  rig.core.ready(true, 3);
  rig.core.input(rig.axes, rig.buttons, 5, 3);
  check(!rig.core.output(3).valid, "fresh joy cannot revive stale source status");
}

void session_config_and_invalid_values()
{
  Rig rig;
  rig.send();
  rig.core.source("process-b:1", true, 0);
  check(!rig.core.output(0).valid, "driver restart with reused generation resets input");
  rig.axes[5] = .8F;
  rig.send();
  check(!rig.core.output(0).valid, "driver restart held trigger requires release");
  rig.axes[5] = 0;
  rig.send();
  check(rig.core.output(0).valid, "neutral input after session change");
  check(rig.core.configure(true, 0) && !rig.core.output(0).valid, "configuration suspends output");
  rig.buttons[0] = 1;
  rig.send();
  check(!rig.core.configure(false, 0), "cannot resume while a button is held");
  rig.buttons[0] = 0;
  rig.send();
  check(rig.core.configure(false, 0), "resume neutral input");
  check(rig.send().empty() && rig.core.output(0).valid, "resumption resets button history");
  Profile candidate = rig.core.profile();
  candidate.record_stop = candidate.record_start;
  bool rejected = false;
  try {rig.core.set_profile(candidate);} catch (const std::invalid_argument &) {rejected = true;}
  check(rejected && rig.core.profile().record_stop == 1, "invalid reload must retain active profile");
  rig.buttons[0] = 1;
  rig.core.set_profile(rig.core.profile());
  check(rig.send().empty() && !rig.core.output(0).valid, "profile replacement must reset held button history");
  rig.buttons[0] = 0;
  rig.send();
  rig.axes[0] = std::numeric_limits<float>::quiet_NaN();
  check(rig.send().empty() && !rig.core.output(0).valid, "NaN input invalidates output");
  rig.axes[0] = std::numeric_limits<float>::infinity();
  check(rig.send().empty() && !rig.core.output(0).valid, "infinity invalidates output");
  rig.axes[0] = 1.1F;
  check(rig.send().empty() && !rig.core.output(0).valid, "out-of-range input invalidates output");
  rig.axes[0] = 0.0F;
  rig.axes[4] = -0.1F;
  check(rig.send().empty() && !rig.core.output(0).valid, "negative L2 trigger invalidates output");
  rig.axes[4] = 0.0F;
  rig.axes[5] = -0.1F;
  check(rig.send().empty() && !rig.core.output(0).valid, "negative R2 trigger invalidates output");
  rig.axes[5] = 0.0F;
  rig.axes.pop_back();
  check(rig.send().empty() && !rig.core.output(0).valid, "malformed axes invalidates output");
  rig.axes.assign(8, 0.0F);
  rig.buttons[0] = 2;
  check(rig.send().empty() && !rig.core.output(0).valid, "malformed button value invalidates output");
  rig.buttons.assign(17, 0);
  rig.send();
  Profile empty;
  rig.core.set_profile(empty);
  rig.send();
  check(!rig.core.output(0).valid, "unassigned roles never produce a valid request");
}

void recording_only_never_enables_driving()
{
  Rig rig;
  Profile profile;
  profile.record_start = 0;
  profile.record_stop = 1;
  profile.enable = 5;
  rig.core.set_profile(profile);
  check(rig.core.configure(true, rig.now), "recording-only configuration can start");
  rig.send();
  check(rig.core.configure(false, rig.now), "recording-only roles can resume after release");
  check(rig.send().empty() && rig.core.events_ready(rig.now), "released recording buttons establish baseline");
  check(rig.core.reason(rig.now) == "steering_and_throttle_are_unassigned" &&
    rig.core.events_reason(rig.now) == "ready", "drive and recording readiness remain separate");
  for (int i = 0; i < 3; ++i) {
    const auto output = rig.core.output(rig.now);
    check(!output.valid && output.steering == 0 && output.throttle == 0 && !output.enable_requested,
      "timer publications remain invalid neutral without driving axes");
  }
  rig.axes[0] = .8F;
  rig.axes[5] = .6F;
  rig.buttons[5] = 1;
  rig.buttons[0] = 1;
  check(rig.send() == std::vector<uint8_t>{1}, "recording-only start edge survives drive output ticks");
  auto output = rig.core.output(rig.now);
  check(!output.valid && output.steering == 0 && output.throttle == 0 && !output.enable_requested,
    "unassigned axes and enable button must never leak driving values");
  check(rig.send().empty(), "held recording-only button emits once");
  rig.buttons[0] = 0;
  rig.send();
  rig.core.output(rig.now);
  rig.buttons[1] = 1;
  check(rig.send() == std::vector<uint8_t>{2}, "recording-only stop edge emits once");
}

void recording_only_keeps_freshness_release_and_reconnect_guards()
{
  Rig rig;
  Profile profile;
  profile.record_start = 0;
  rig.core.set_profile(profile);
  rig.core.configure(true, rig.now);
  rig.buttons[0] = 1;
  rig.send();
  check(!rig.core.configure(false, rig.now), "recording-only resume still rejects held buttons");
  rig.buttons[0] = 0;
  rig.axes[0] = .2F;
  rig.send();
  check(!rig.core.configure(false, rig.now), "recording-only resume still requires all axes released");
  rig.axes[0] = 0;
  rig.send();
  check(rig.core.configure(false, rig.now), "single recording button is enough to resume");
  rig.send();
  rig.buttons[0] = 1;
  check(rig.send() == std::vector<uint8_t>{1}, "single configured start button works");
  rig.now = .3;
  rig.core.ready(true, rig.now);
  check(!rig.core.events_ready(rig.now), "fresh ready cannot revive stale event input");
  rig.core.input(rig.axes, rig.buttons, rig.stamp - 1, rig.now);
  check(!rig.core.events_ready(rig.now), "duplicate stamp cannot revive recording events");
  check(rig.send().empty() && !rig.core.events_ready(rig.now), "held button after timeout cannot synthesize start");
  rig.buttons[0] = 0;
  rig.send();
  check(rig.core.events_ready(rig.now), "release reestablishes recording baseline");
  rig.core.source("new-joy-process:1", true, rig.now);
  rig.buttons[0] = 1;
  check(rig.send().empty() && !rig.core.events_ready(rig.now), "reconnect held start requires release");
  rig.buttons[0] = 0;
  rig.send();
  rig.buttons[0] = 1;
  check(rig.send() == std::vector<uint8_t>{1}, "new press after reconnect baseline records");
  rig.core.ready(false, rig.now);
  check(!rig.core.events_ready(rig.now), "Joy not-ready disables recording immediately");
  rig.now = 3.1;
  rig.send();
  check(!rig.core.events_ready(rig.now) && rig.core.events_reason(rig.now) == "joy_status_unavailable",
    "fresh Joy alone cannot revive expired session status");
}

void empty_or_enable_only_profile_cannot_resume()
{
  for (const int enabled : {-1, 5}) {
    Rig rig;
    Profile profile;
    profile.enable = enabled;
    rig.core.set_profile(profile);
    rig.core.configure(true, rig.now);
    rig.send();
    check(!rig.core.configure(false, rig.now), "empty and enable-only profiles cannot resume");
    check(!rig.core.events_ready(rig.now) && !rig.core.output(rig.now).valid, "no usable roles stay inactive");
  }
  Rig rig;
  Profile profile;
  profile.record_stop = 1;
  rig.core.set_profile(profile);
  rig.send();
  rig.buttons[1] = 1;
  check(rig.send() == std::vector<uint8_t>{2}, "stop-only recording profile remains usable");
}

int main()
{
  mapping_and_limits();
  events_are_edges_and_require_release();
  timeouts_do_not_refresh_stale_input();
  session_config_and_invalid_values();
  recording_only_never_enables_driving();
  recording_only_keeps_freshness_release_and_reconnect_guards();
  empty_or_enable_only_profile_cannot_resume();
  std::cout << "teleop core: mapping, events, freshness, sessions, configuration, malformed input passed\n";
}
