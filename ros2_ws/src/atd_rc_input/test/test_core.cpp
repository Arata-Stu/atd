#include "atd_rc_input/core.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace atd_rc_input;
void check(bool value, const char * message)
{
  if (!value) {std::cerr << message << '\n'; std::exit(1);}
}
struct Rig
{
  Core core;
  Sample sample;
  double now{0};
  Rig()
  {
    Profile profile;
    profile.roles[0].channel = 4;
    core.profile(profile);
    sample.instance_id = "usb-session-1";
    sample.serial = "pico-1";
    sample.rc_valid = true;
    sample.sbus_fresh = true;
    sample.channels.fill(992);
  }
  std::vector<uint8_t> tick(bool advance_sbus = true)
  {
    now += 0.02;
    ++sample.sequence;
    sample.board_us += 20000;
    if (advance_sbus) {++sample.sbus_frames;}
    return core.input(sample, now);
  }
  std::vector<uint8_t> settle(uint16_t raw, size_t channel = 4)
  {
    sample.channels[channel - 1] = raw;
    std::vector<uint8_t> events;
    for (int i = 0; i < 7; ++i) {
      auto out = tick();
      events.insert(events.end(), out.begin(), out.end());
    }
    return events;
  }
};
int main()
{
  {
    Rig r;
    check(r.settle(1800).empty(), "held start at startup must only establish baseline");
    check(r.core.reason(r.now) == "ready", "stable profile ready");
    auto events = r.settle(200);
    check(events.size() == 1 && events[0] == 2, "low transition emits stop once");
    check(r.settle(200).empty(), "held low must not repeat stop");
    events = r.settle(1800);
    check(events.size() == 1 && events[0] == 1, "high transition emits start once");
  }
  {
    Rig r;
    r.settle(200);
    r.sample.channels[3] = 1800;
    for (int i = 0; i < 4; ++i) {check(r.tick().empty(), "debounce must suppress short change");}
    check(r.settle(992).empty(), "middle band must not emit event");
    check(r.settle(1800) == std::vector<uint8_t>{1}, "stable high after middle emits one start");
    check(r.core.configure(true, r.now), "enter configuration");
    check(r.settle(200).empty(), "configuration suppresses events");
    check(r.core.configure(false, r.now), "fresh signal resumes roles without steering release");
    check(r.settle(1800).empty(), "resume held high must reacquire baseline");
  }
  {
    Rig r;
    r.settle(200);
    r.sample.instance_id = "new-usb-session";
    check(r.settle(1800).empty(), "reconnection high only establishes baseline");
    r.sample.flags = 4;
    check(r.tick().empty() && !r.core.fresh(r.now), "frame_lost invalidates immediately");
    r.sample.flags = 0;
    check(r.settle(200).empty(), "recovered low must not synthesize stop");
    r.sample.flags = 8;
    check(r.tick().empty() && !r.core.fresh(r.now), "failsafe invalidates immediately");
  }
  {
    Rig r;
    r.settle(200);
    const double received = r.now;
    r.now += 0.02;
    check(r.core.input(r.sample, r.now).empty() && !r.core.accepted(), "duplicate sequence rejected");
    check(std::abs(r.core.age(r.now) - (r.now - received)) < 1e-8, "duplicate may not renew source age");
    r.sample.channels[3] = 1800;
    for (int i = 0; i < 10; ++i) {check(r.tick(false).empty(), "repeated sbus snapshot must not trigger events");}
    check(!r.core.fresh(r.now), "frozen sbus must expire despite advancing telemetry");
    check(!r.core.configure(false, r.now), "stale source cannot resume");
    check(r.settle(1800).empty(), "new sbus after expiry only baselines");
    check(r.core.reason(r.now + 0.3) == "usb_unavailable", "USB expiry is reported");
  }
  {
    Rig r;
    auto p = r.core.profile();
    p.roles[0].channel = 6;
    p.roles[0].inverted = true;
    r.core.profile(p);
    r.sample.channels[5] = 1800;
    for (int i = 0; i < 7; ++i) {check(r.tick().empty(), "new profile high raw only baselines");}
    r.sample.channels[5] = 200;
    std::vector<uint8_t> events;
    for (int i = 0; i < 7; ++i) {auto result = r.tick(); events.insert(events.end(), result.begin(), result.end());}
    check(events == std::vector<uint8_t>{1}, "remapped inverted CH6 emits start at low raw");
    r.sample.channels[5] = 2048;
    check(r.tick().empty() && !r.core.have_sample(), "malformed channels invalidate source");
  }
  {
    Rig r;
    r.settle(200);
    r.sample.channels[3] = 1800;
    r.sample.receipt_age_s = 0.08;
    for (int i = 0; i < 10; ++i) {check(r.tick().empty(), "queued old frames cannot trigger recording");}
    check(!r.core.fresh(r.now) && !r.core.configure(false, r.now), "old source receipt cannot resume events");
    r.sample.receipt_age_s = 0.035;
    r.sample.sbus_age_ms = 20;
    check(r.tick().empty() && !r.core.fresh(r.now), "transport delay and S.BUS age share the freshness budget");
    r.sample.receipt_age_s = 0;
    r.sample.sbus_age_ms = 0;
    check(r.settle(1800).empty(), "fresh recovery must reacquire baseline");
    check(r.settle(200) == std::vector<uint8_t>{2}, "fresh transition emits stop after recovery");
  }
  {
    for (const int channel : {10, 16}) {
      Rig r;
      r.settle(200);
      auto p = r.core.profile();
      p.roles[0].channel = channel;
      p.roles[0].inverted = channel == 16;
      r.core.profile(p);
      const uint16_t low = p.roles[0].inverted ? 1800 : 200;
      const uint16_t high = p.roles[0].inverted ? 200 : 1800;
      check(r.settle(high, channel).empty(), "remapped extended channel must reacquire baseline");
      check(r.core.position(0) == Position::high, "extended channel position follows configured input");
      check(r.settle(1800).empty(), "previous CH4 cannot emit events after remapping");
      check(r.settle(low, channel) == std::vector<uint8_t>{2}, "extended channel low transition emits stop");
      check(r.settle(high, channel) == std::vector<uint8_t>{1}, "extended channel high transition emits start");
      r.sample.instance_id = "extended-channel-reconnected";
      check(r.settle(low, channel).empty(), "extended channel reconnect must not synthesize stop");
      check(r.settle(high, channel) == std::vector<uint8_t>{1}, "extended channel start follows new baseline");
    }
  }
  {
    Profile p;
    for (const int channel : {-1, 1, 2, 3, 17}) {
      p.roles[0].channel = channel;
      bool rejected = false;
      try {p.validate();} catch (const std::invalid_argument &) {rejected = true;}
      check(rejected, "Pico driving and out-of-range channels must not be assignable");
    }
    p.roles[0].channel = 4;
    p.roles[1].channel = 4;
    bool rejected = false;
    try {p.validate();} catch (const std::invalid_argument &) {rejected = true;}
    check(rejected, "duplicate roles rejected");
  }
  std::cout << "RC roles: mapping, debounce, baseline, freshness, reconnect and invalid input passed\n";
}
