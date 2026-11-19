#include "rev6_led.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  uint32_t elapsed_ms;
  bool on;
} sample_t;

/* These are visible pulse edges, independently written from the implementation.
 * Samples immediately before and at an edge catch a one-tick change in meaning.
 */
static const sample_t waiting[] = {
    {0U, true}, {119U, true}, {120U, false}, {1999U, false},
    {2000U, true}, {2119U, true}, {2120U, false}};
static const sample_t usb_link[] = {
    {0U, true}, {119U, true}, {120U, false}, {249U, false},
    {250U, true}, {369U, true}, {370U, false}, {1999U, false},
    {2000U, true}};
static const sample_t rc_ready[] = {
    {0U, true}, {119U, true}, {120U, false}, {249U, false},
    {250U, true}, {369U, true}, {370U, false}, {499U, false},
    {500U, true}, {619U, true}, {620U, false}, {1999U, false},
    {2000U, true}};
static const sample_t rc_active[] = {
    {0U, true}, {499U, true}, {500U, false}, {999U, false},
    {1000U, true}, {1499U, true}, {1500U, false}};
static const sample_t host_active[] = {
    {0U, true}, {899U, true}, {900U, false}, {999U, false},
    {1000U, true}, {1899U, true}, {1900U, false}};
static const sample_t fault[] = {
    {0U, true}, {99U, true}, {100U, false}, {199U, false},
    {200U, true}, {299U, true}, {300U, false}};

static rev6_core_output_t indication(rev6_led_mode_t mode) {
  rev6_core_output_t output = {0};
  output.servo_us = output.esc_us = 1500U;
  if (mode == REV6_LED_WAITING || mode == REV6_LED_USB_LINK) {
    output.fault_bits = REV6_FAULT_POWER;
  } else {
    output.power_ready = output.output_enabled = true;
  }
  if (mode == REV6_LED_RC_READY || mode == REV6_LED_RC_ACTIVE) {
    output.sbus_fresh = output.receiver_latched = true;
  }
  if (mode == REV6_LED_RC_ACTIVE) {
    output.active_path = 1U;
  } else if (mode == REV6_LED_HOST_ACTIVE) {
    output.active_path = 2U;
    output.host_armed = true;
  } else if (mode == REV6_LED_FAULT) {
    output.active_path = 3U;
    output.fault_bits = REV6_FAULT_RECEIVER_LOSS;
  }
  return output;
}

static void check_pattern(rev6_led_mode_t mode, bool usb,
                            const sample_t *samples, size_t count) {
  rev6_led_t led;
  rev6_led_init(&led);
  rev6_core_output_t output = indication(mode);
  rev6_core_output_t before;
  memcpy(&before, &output, sizeof(before));
  for (size_t i = 0U; i < count; ++i) {
    assert(rev6_led_update(&led, &output, usb, 12345U + samples[i].elapsed_ms)
           == samples[i].on);
    assert(led.mode == mode);
    assert(memcmp(&before, &output, sizeof(output)) == 0);
  }
}

#define CHECK_PATTERN(mode, usb, samples) \
  check_pattern(mode, usb, samples, sizeof(samples) / sizeof(samples[0]))

static void visible_pulse_edges(void) {
  CHECK_PATTERN(REV6_LED_WAITING, false, waiting);
  CHECK_PATTERN(REV6_LED_USB_LINK, true, usb_link);
  CHECK_PATTERN(REV6_LED_RC_READY, false, rc_ready);
  CHECK_PATTERN(REV6_LED_RC_ACTIVE, false, rc_active);
  CHECK_PATTERN(REV6_LED_HOST_ACTIVE, true, host_active);
  CHECK_PATTERN(REV6_LED_FAULT, true, fault);
  assert(strcmp(rev6_led_mode_name(REV6_LED_WAITING), "WAITING") == 0);
  assert(strcmp(rev6_led_mode_name(REV6_LED_USB_LINK), "USB_LINK") == 0);
  assert(strcmp(rev6_led_mode_name(REV6_LED_RC_READY), "RC_READY") == 0);
  assert(strcmp(rev6_led_mode_name(REV6_LED_RC_ACTIVE), "RC_ACTIVE") == 0);
  assert(strcmp(rev6_led_mode_name(REV6_LED_HOST_ACTIVE), "HOST_ACTIVE") == 0);
  assert(strcmp(rev6_led_mode_name((rev6_led_mode_t)99), "UNKNOWN") == 0);
}

static void phase_changes_clock_wrap_and_long_gaps(void) {
  rev6_led_t led;
  rev6_led_init(&led);
  rev6_core_output_t output = indication(REV6_LED_WAITING);
  assert(rev6_led_update(&led, &output, false, 100U));
  assert(!rev6_led_update(&led, &output, false, 600U));
  assert(led.phase_ms == 500U);
  /* A new indication starts now; it does not inherit the old dark phase. */
  assert(rev6_led_update(&led, &output, true, 600U));
  assert(led.mode == REV6_LED_USB_LINK && led.phase_ms == 0U);
  assert(!rev6_led_update(&led, &output, true, 800U));
  output = indication(REV6_LED_RC_READY);
  assert(rev6_led_update(&led, &output, true, 800U));
  assert(led.mode == REV6_LED_RC_READY && led.phase_ms == 0U);
  assert(!rev6_led_update(&led, &output, true, 1000U));
  output.fault_bits = REV6_FAULT_RC_SIGNAL;
  assert(rev6_led_update(&led, &output, true, 1000U));
  assert(led.mode == REV6_LED_FAULT && led.phase_ms == 0U);

  rev6_led_init(&led);
  output = indication(REV6_LED_WAITING);
  const uint32_t start = UINT32_MAX - 50U;
  assert(rev6_led_update(&led, &output, false, start));
  assert(rev6_led_update(&led, &output, false, 49U)); /* elapsed 100 */
  assert(led.phase_ms == 100U);
  assert(rev6_led_update(&led, &output, false, 68U));
  assert(!rev6_led_update(&led, &output, false, 69U));
  assert(!rev6_led_update(&led, &output, false, 1948U));
  assert(rev6_led_update(&led, &output, false, 1949U));
  assert(led.phase_ms == 0U);

  rev6_led_init(&led);
  assert(rev6_led_update(&led, &output, true, 5U));
  /* Reducing a long gap before adding to the short phase avoids overflow. */
  assert(rev6_led_update(&led, &output, true, 3000000255U));
  assert(led.phase_ms == 250U);
  assert(!rev6_led_update(&led, &output, true, 3000000375U));
  assert(led.phase_ms == 370U);
}

static void faults_override_connected_and_active_indications(void) {
  static const uint32_t faults[] = {
      REV6_FAULT_USB, REV6_FAULT_RECEIVER_LOSS, REV6_FAULT_MODE_UNKNOWN,
      REV6_FAULT_RC_SIGNAL, REV6_FAULT_COMMAND_TIMEOUT,
      REV6_FAULT_HEARTBEAT_TIMEOUT, REV6_FAULT_COMMAND_VALUE, 1UL << 31};
  for (size_t i = 0U; i < sizeof(faults) / sizeof(faults[0]); ++i) {
    rev6_led_t led;
    rev6_led_init(&led);
    rev6_core_output_t output = indication(REV6_LED_HOST_ACTIVE);
    output.sbus_fresh = true;
    output.fault_bits = faults[i];
    assert(rev6_led_update(&led, &output, true, 0U));
    assert(led.mode == REV6_LED_FAULT);
    /* Real faults remain visible even before the first BEC qualification. */
    rev6_led_init(&led);
    output = indication(REV6_LED_USB_LINK);
    output.fault_bits |= faults[i];
    assert(rev6_led_update(&led, &output, true, 0U));
    assert(led.mode == REV6_LED_FAULT && !led.power_seen);
  }

  rev6_led_t led;
  rev6_led_init(&led);
  rev6_core_output_t output = indication(REV6_LED_USB_LINK);
  output.selector = 1U; /* HOST permission alone must not look armed. */
  rev6_led_update(&led, &output, true, 0U);
  assert(led.mode == REV6_LED_USB_LINK);
  output = indication(REV6_LED_HOST_ACTIVE);
  output.host_armed = false;
  rev6_led_update(&led, &output, true, 1U);
  assert(led.mode == REV6_LED_USB_LINK);
  output.host_armed = true;
  rev6_led_update(&led, &output, false, 2U);
  assert(led.mode != REV6_LED_HOST_ACTIVE);
  output = indication(REV6_LED_RC_ACTIVE);
  output.output_enabled = false;
  rev6_led_update(&led, &output, true, 3U);
  assert(led.mode == REV6_LED_RC_READY);
}

/* Check the entire real controller state, not just PWM values. Display updates
 * must not change arming, permission, fault history, or timeout timestamps.
 */
static void observe(rev6_core_t *core, rev6_led_t *led, bool usb,
                       uint32_t now, rev6_led_mode_t mode) {
  rev6_core_t before;
  memcpy(&before, core, sizeof(before));
  rev6_led_update(led, rev6_core_output(core), usb, now);
  assert(led->mode == mode);
  assert(memcmp(&before, core, sizeof(before)) == 0);
}

static jpbb_command_t host(int steering, unsigned throttle) {
  jpbb_command_t command = {0};
  command.kind = JPBB_COMMAND_HOST_FRAME;
  command.flags = JPBB_HOST_FLAG_COMMAND_VALID | JPBB_HOST_FLAG_HOST_REQUEST;
  command.steering_milli = (int16_t)steering;
  command.throttle_milli = (uint16_t)throttle;
  return command;
}

static rev6_sbus_frame_t receiver(rev6_mode_t mode) {
  rev6_sbus_frame_t frame = {0};
  for (unsigned i = 0U; i < REV6_SBUS_CHANNELS; ++i) {
    frame.channels[i] = REV6_SBUS_RAW_MID;
  }
  frame.channels[REV6_SBUS_MODE_CHANNEL] = mode == REV6_MODE_RC
      ? REV6_SBUS_RAW_MIN : REV6_SBUS_RAW_MAX;
  return frame;
}

static void usb_bench_power_history_and_recovery(void) {
  rev6_core_t core;
  rev6_led_t led;
  rev6_core_init(&core);
  rev6_led_init(&led);
  rev6_core_tick(&core, 0U, false, false);
  observe(&core, &led, false, 0U, REV6_LED_WAITING);
  rev6_core_tick(&core, 10U, false, true);
  observe(&core, &led, true, 10U, REV6_LED_USB_LINK);
  assert(!led.power_seen && core.state.fault_bits == REV6_FAULT_POWER);
  const jpbb_command_t neutral = host(0, 0U);
  assert(!rev6_core_command(&core, &neutral, 11U));
  observe(&core, &led, true, 11U, REV6_LED_USB_LINK);
  assert(!core.state.output_enabled && !core.state.host_armed);
  rev6_core_tick(&core, 20U, true, true);
  rev6_core_tick(&core, 119U, true, true);
  observe(&core, &led, true, 119U, REV6_LED_USB_LINK);
  assert(!led.power_seen);
  rev6_core_tick(&core, 120U, true, true);
  observe(&core, &led, true, 120U, REV6_LED_USB_LINK);
  assert(led.power_seen && core.state.power_ready);
  rev6_core_tick(&core, 121U, false, true);
  observe(&core, &led, true, 121U, REV6_LED_FAULT);
  assert(!core.state.output_enabled);
  rev6_core_tick(&core, 122U, true, true);
  rev6_core_tick(&core, 221U, true, true);
  observe(&core, &led, true, 221U, REV6_LED_FAULT);
  rev6_core_tick(&core, 222U, true, true);
  observe(&core, &led, true, 222U, REV6_LED_USB_LINK);
  assert(!core.state.host_armed); /* The LED never arms a recovered controller. */

  rev6_led_init(&led);
  rev6_core_init(&core);
  rev6_core_tick(&core, 0U, false, true);
  observe(&core, &led, true, 0U, REV6_LED_USB_LINK);
  assert(!led.power_seen); /* A new boot has no previous BEC-loss history. */
}

static void host_without_receiver_timeout_and_usb_loss(void) {
  rev6_core_t core;
  rev6_led_t led;
  rev6_core_init(&core);
  rev6_led_init(&led);
  rev6_core_tick(&core, 0U, true, true);
  rev6_core_tick(&core, 100U, true, true);
  observe(&core, &led, true, 100U, REV6_LED_USB_LINK);
  jpbb_command_t ping = {0};
  ping.kind = JPBB_COMMAND_PING;
  assert(rev6_core_command(&core, &ping, 101U));
  observe(&core, &led, true, 101U, REV6_LED_USB_LINK);
  assert(!core.state.host_armed && !core.state.receiver_latched);
  const jpbb_command_t neutral = host(0, 0U);
  const jpbb_command_t motion = host(-600, 400U);
  assert(rev6_core_command(&core, &neutral, 102U));
  observe(&core, &led, true, 102U, REV6_LED_HOST_ACTIVE);
  assert(rev6_core_command(&core, &motion, 103U));
  observe(&core, &led, true, 103U, REV6_LED_HOST_ACTIVE);
  assert(core.state.servo_us == 1200U && core.state.esc_us == 1300U);
  rev6_core_tick(&core, 303U, true, true);
  observe(&core, &led, true, 303U, REV6_LED_HOST_ACTIVE);
  rev6_core_tick(&core, 304U, true, true);
  observe(&core, &led, true, 304U, REV6_LED_FAULT);
  assert(core.state.fault_bits & REV6_FAULT_COMMAND_TIMEOUT);
  assert(!core.state.host_armed && core.state.esc_us == 1500U);
  assert(rev6_core_command(&core, &neutral, 305U));
  observe(&core, &led, true, 305U, REV6_LED_HOST_ACTIVE);
  rev6_core_tick(&core, 306U, true, false);
  observe(&core, &led, false, 306U, REV6_LED_FAULT);
  assert(core.state.fault_bits & REV6_FAULT_USB);
  rev6_core_tick(&core, 307U, true, true);
  observe(&core, &led, true, 307U, REV6_LED_FAULT);
  assert(rev6_core_command(&core, &neutral, 308U));
  observe(&core, &led, true, 308U, REV6_LED_HOST_ACTIVE);
  jpbb_command_t manual = {0};
  manual.kind = JPBB_COMMAND_MANUAL;
  assert(rev6_core_command(&core, &manual, 309U));
  observe(&core, &led, true, 309U, REV6_LED_USB_LINK);
  assert(!core.state.receiver_latched && !core.state.host_armed);
}

static void bec_only_rc_usb_changes_and_receiver_loss(void) {
  rev6_core_t core;
  rev6_led_t led;
  rev6_core_init(&core);
  rev6_led_init(&led);
  rev6_core_tick(&core, 0U, true, false);
  rev6_core_tick(&core, 100U, true, false);
  observe(&core, &led, false, 100U, REV6_LED_WAITING);
  assert(core.state.fault_bits == 0U && core.state.output_enabled);
  rev6_sbus_frame_t frame = receiver(REV6_MODE_RC);
  for (uint32_t now = 100U; now <= 140U; ++now) {
    rev6_core_tick(&core, now, true, false);
    if (now % 10U == 0U) { rev6_core_note_sbus(&core, &frame, now); }
  }
  observe(&core, &led, false, 140U, REV6_LED_RC_ACTIVE);
  rev6_core_tick(&core, 141U, true, true);
  observe(&core, &led, true, 141U, REV6_LED_RC_ACTIVE);
  rev6_core_tick(&core, 142U, true, false);
  observe(&core, &led, false, 142U, REV6_LED_RC_ACTIVE);
  assert(core.state.fault_bits == 0U);
  frame.frame_lost = true;
  rev6_core_note_sbus(&core, &frame, 150U);
  observe(&core, &led, false, 150U, REV6_LED_RC_ACTIVE);
  rev6_core_tick(&core, 190U, true, true);
  observe(&core, &led, true, 190U, REV6_LED_RC_ACTIVE);
  rev6_core_tick(&core, 191U, true, true);
  observe(&core, &led, true, 191U, REV6_LED_FAULT);
  assert(core.state.receiver_latched && !core.state.sbus_fresh);
  assert(core.state.fault_bits & REV6_FAULT_RECEIVER_LOSS);
  /* The visible state does not silently turn a lost receiver into USB-only. */
  const jpbb_command_t neutral = host(0, 0U);
  assert(!rev6_core_command(&core, &neutral, 192U));
  observe(&core, &led, true, 192U, REV6_LED_FAULT);
}

static void cli_command_stream_does_not_hide_heartbeat_timeout(void) {
  rev6_core_t core;
  rev6_led_t led;
  rev6_core_init(&core);
  rev6_led_init(&led);
  rev6_core_tick(&core, 0U, true, true);
  rev6_core_tick(&core, 100U, true, true);
  jpbb_command_t command = {0};
  command.kind = JPBB_COMMAND_SET;
  command.servo_us = command.esc_us = 1500U;
  assert(rev6_core_command(&core, &command, 100U));
  observe(&core, &led, true, 100U, REV6_LED_USB_LINK);
  command.kind = JPBB_COMMAND_AUTO;
  assert(rev6_core_command(&core, &command, 100U));
  observe(&core, &led, true, 100U, REV6_LED_HOST_ACTIVE);
  command.kind = JPBB_COMMAND_STREAM;
  command.esc_us = 1300U;
  for (uint32_t now = 200U; now <= 600U; now += 100U) {
    rev6_core_tick(&core, now, true, true);
    assert(rev6_core_command(&core, &command, now));
    observe(&core, &led, true, now, REV6_LED_HOST_ACTIVE);
  }
  assert(core.state.host_armed); /* Exactly 500ms since AUTO is still fresh. */
  rev6_core_tick(&core, 601U, true, true);
  observe(&core, &led, true, 601U, REV6_LED_FAULT);
  assert(core.state.fault_bits == REV6_FAULT_HEARTBEAT_TIMEOUT);
  assert(!core.state.host_armed && core.state.esc_us == 1500U);
}

static void receiver_permission_is_not_host_control_and_failsafe_is_immediate(void) {
  rev6_core_t core;
  rev6_led_t led;
  rev6_core_init(&core);
  rev6_led_init(&led);
  rev6_sbus_frame_t frame = receiver(REV6_MODE_HOST);
  for (uint32_t now = 0U; now <= 100U; ++now) {
    rev6_core_tick(&core, now, true, false);
    if (now % 10U == 0U) { rev6_core_note_sbus(&core, &frame, now); }
  }
  observe(&core, &led, false, 100U, REV6_LED_RC_READY);
  assert(core.state.selector == 1U && !core.state.host_armed);
  rev6_core_tick(&core, 101U, true, true);
  observe(&core, &led, true, 101U, REV6_LED_RC_READY);
  const jpbb_command_t neutral = host(0, 0U);
  assert(rev6_core_command(&core, &neutral, 102U));
  observe(&core, &led, true, 102U, REV6_LED_HOST_ACTIVE);
  frame.failsafe = true;
  rev6_core_note_sbus(&core, &frame, 103U);
  observe(&core, &led, true, 103U, REV6_LED_FAULT);
  assert(core.state.fault_bits & REV6_FAULT_RC_SIGNAL);
  assert(!core.state.host_armed && core.state.esc_us == 1500U);
}

int main(void) {
  visible_pulse_edges();
  phase_changes_clock_wrap_and_long_gaps();
  faults_override_connected_and_active_indications();
  usb_bench_power_history_and_recovery();
  host_without_receiver_timeout_and_usb_loss();
  bec_only_rc_usb_changes_and_receiver_loss();
  cli_command_stream_does_not_hide_heartbeat_timeout();
  receiver_permission_is_not_host_control_and_failsafe_is_immediate();
  puts("Pico LED timing/controller integration tests: PASS");
  return 0;
}
