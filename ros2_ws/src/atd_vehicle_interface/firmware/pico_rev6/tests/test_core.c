#include "rev6_core.h"

#include <assert.h>
#include <stdio.h>

static const rev6_core_output_t *out(const rev6_core_t *core) {
  return rev6_core_output(core);
}

static jpbb_command_t host(int steering, unsigned throttle, unsigned reverse,
                            unsigned brake) {
  jpbb_command_t command = {0};
  command.kind = JPBB_COMMAND_HOST_FRAME;
  command.steering_milli = (int16_t)steering;
  command.throttle_milli = (uint16_t)throttle;
  command.reverse_milli = (uint16_t)reverse;
  command.brake_milli = (uint16_t)brake;
  command.flags = JPBB_HOST_FLAG_COMMAND_VALID | JPBB_HOST_FLAG_HOST_REQUEST;
  return command;
}

static jpbb_command_t cli(jpbb_command_kind_t kind, uint16_t servo,
                           uint16_t esc) {
  jpbb_command_t command = {0};
  command.kind = kind;
  command.servo_us = servo;
  command.esc_us = esc;
  return command;
}

static rev6_sbus_frame_t receiver(rev6_mode_t mode) {
  rev6_sbus_frame_t frame = {0};
  for (unsigned channel = 0U; channel < 16U; ++channel) {
    frame.channels[channel] = REV6_SBUS_RAW_MID;
  }
  frame.channels[REV6_SBUS_STEERING_CHANNEL] = REV6_SBUS_RAW_MIN;
  frame.channels[REV6_SBUS_THROTTLE_CHANNEL] = REV6_SBUS_RAW_MAX;
  if (mode == REV6_MODE_UNKNOWN) {
    frame.channels[REV6_SBUS_MODE_CHANNEL] = REV6_SBUS_RAW_MID;
  } else {
    const bool high = (mode == REV6_MODE_HOST) !=
                      (REV6_SBUS_MODE_REVERSED != 0);
    frame.channels[REV6_SBUS_MODE_CHANNEL] =
        high ? REV6_SBUS_RAW_MAX : REV6_SBUS_RAW_MIN;
  }
  return frame;
}

static void ready_usb_only(rev6_core_t *core) {
  rev6_core_init(core);
  rev6_core_tick(core, 0U, true, true);
  rev6_core_tick(core, 100U, true, true);
  assert(out(core)->power_ready);
  assert(out(core)->fault_bits == 0U);
  assert(out(core)->selector == 1U);
  assert(out(core)->active_path == 0U);
}

/* Model 1 ms control and a receiver frame every 10 ms. */
static void run_receiver(rev6_core_t *core, const rev6_sbus_frame_t *frame,
                           uint32_t from, uint32_t through, bool usb) {
  for (uint32_t now = from; now <= through; ++now) {
    rev6_core_tick(core, now, true, usb);
    if ((now - from) % 10U == 0U) {
      rev6_core_note_sbus(core, frame, now);
    }
  }
}

static void boot_power_confirmation_and_exact_neutral_arm(void) {
  rev6_core_t core;
  rev6_core_init(&core);
  assert(!out(&core)->output_enabled);
  assert(out(&core)->servo_us == 1500U && out(&core)->esc_us == 1500U);
  jpbb_command_t neutral = host(0, 0, 0, 0);
  jpbb_command_t tiny = host(1, 0, 0, 0);
  jpbb_command_t motion = host(-600, 500, 0, 0);
  rev6_core_tick(&core, 0U, true, true);
  assert(!rev6_core_command(&core, &neutral, 1U));
  rev6_core_tick(&core, 99U, true, true);
  assert(!out(&core)->power_ready);
  rev6_core_tick(&core, 100U, true, true);
  assert(out(&core)->power_ready && out(&core)->output_enabled);
  assert(!out(&core)->host_armed); /* Earlier neutral was not retained. */
  assert(!rev6_core_command(&core, &motion, 100U));
  assert(!rev6_core_command(&core, &tiny, 100U));
  assert(!out(&core)->host_armed);
  assert(rev6_core_command(&core, &neutral, 100U));
  assert(out(&core)->host_armed && out(&core)->active_path == 2U);
  assert(rev6_core_command(&core, &motion, 101U));
  assert(out(&core)->servo_us == 1200U && out(&core)->esc_us == 1250U);
}

static void host_polarity_ranges_flags_and_duplicate_sequence(void) {
  rev6_core_t core;
  ready_usb_only(&core);
  jpbb_command_t command = host(0, 0, 0, 0);
  assert(rev6_core_command(&core, &command, 100U));
  command = host(-1000, 1000, 0, 0);
  assert(rev6_core_command(&core, &command, 101U));
  assert(out(&core)->servo_us == 1000U && out(&core)->esc_us == 1000U);
  command = host(1000, 0, 1000, 0);
  assert(rev6_core_command(&core, &command, 102U));
  assert(out(&core)->servo_us == 2000U && out(&core)->esc_us == 2000U);
  command = host(-1, 800, 0, 300); /* Legacy brake priority. */
  assert(rev6_core_command(&core, &command, 103U));
  assert(out(&core)->servo_us == 1500U && out(&core)->esc_us == 1650U);
  /* Old JPB1 does not reject repeated C sequence numbers. */
  assert(rev6_core_command(&core, &command, 104U));
  command = host(0, 1, 1, 0);
  assert(!rev6_core_command(&core, &command, 105U));
  assert(!out(&core)->host_armed);
  assert((out(&core)->fault_bits & REV6_FAULT_COMMAND_VALUE) != 0U);
  command = host(0, 0, 0, 0);
  command.flags = JPBB_HOST_FLAG_COMMAND_VALID; /* Neutral disarm clears fault. */
  assert(rev6_core_command(&core, &command, 106U));
  assert(out(&core)->fault_bits == 0U && out(&core)->active_path == 0U);
  command.flags |= 4U;
  assert(!rev6_core_command(&core, &command, 107U));
  command = host(1001, 0, 0, 0);
  assert(!rev6_core_command(&core, &command, 108U));
}

static void rc_regeneration_without_usb_and_mode_confirmation(void) {
  rev6_core_t core;
  rev6_core_init(&core);
  const rev6_sbus_frame_t rc = receiver(REV6_MODE_RC);
  run_receiver(&core, &rc, 0U, 39U, false);
  assert(out(&core)->active_path != 1U);
  run_receiver(&core, &rc, 40U, 100U, false);
  assert(out(&core)->active_path == 1U && out(&core)->fault_bits == 0U);
  assert(out(&core)->servo_us ==
         (REV6_SBUS_STEERING_REVERSED ? 2000U : 1000U));
  assert(out(&core)->esc_us ==
         (REV6_SBUS_THROTTLE_REVERSED ? 1000U : 2000U));
  assert(out(&core)->selector == 0U && out(&core)->receiver_latched);

  const rev6_sbus_frame_t pc = receiver(REV6_MODE_HOST);
  run_receiver(&core, &pc, 110U, 209U, false);
  assert(out(&core)->active_path == 1U); /* 100 ms HOST confirmation. */
  run_receiver(&core, &pc, 210U, 210U, false);
  assert(out(&core)->active_path == 0U && out(&core)->selector == 1U);
  jpbb_command_t neutral = host(0, 0, 0, 0);
  assert(!rev6_core_command(&core, &neutral, 210U));
  rev6_core_tick(&core, 211U, true, true);
  assert(rev6_core_command(&core, &neutral, 211U));
  jpbb_command_t motion = host(0, 400, 0, 0);
  assert(rev6_core_command(&core, &motion, 212U));
  run_receiver(&core, &rc, 220U, 259U, true);
  assert(out(&core)->host_armed); /* RC override after 40 ms confirmation. */
  run_receiver(&core, &rc, 260U, 260U, true);
  assert(!out(&core)->host_armed && out(&core)->active_path == 1U);
  run_receiver(&core, &pc, 270U, 370U, true);
  assert(out(&core)->active_path == 0U && !out(&core)->host_armed);
  assert(!rev6_core_command(&core, &motion, 371U));
  assert(rev6_core_command(&core, &neutral, 372U));
}

static void middle_switch_is_unknown_after_100_ms(void) {
  rev6_core_t core;
  rev6_core_init(&core);
  const rev6_sbus_frame_t rc = receiver(REV6_MODE_RC);
  run_receiver(&core, &rc, 0U, 100U, true);
  const rev6_sbus_frame_t middle = receiver(REV6_MODE_UNKNOWN);
  run_receiver(&core, &middle, 110U, 209U, true);
  assert(out(&core)->active_path == 1U);
  run_receiver(&core, &middle, 210U, 210U, true);
  assert(out(&core)->active_path == 3U && out(&core)->selector == 0U);
  assert((out(&core)->fault_bits & REV6_FAULT_MODE_UNKNOWN) != 0U);
  assert(out(&core)->servo_us == 1500U && out(&core)->esc_us == 1500U);
}

static void first_faulted_sbus_latches_and_immediately_revokes_usb_only(void) {
  rev6_core_t core;
  ready_usb_only(&core);
  jpbb_command_t neutral = host(0, 0, 0, 0);
  jpbb_command_t motion = host(0, 500, 0, 0);
  assert(rev6_core_command(&core, &neutral, 100U));
  assert(rev6_core_command(&core, &motion, 101U));
  rev6_sbus_frame_t failed = receiver(REV6_MODE_HOST);
  failed.failsafe = true;
  rev6_core_note_sbus(&core, &failed, 102U);
  assert(out(&core)->receiver_latched && !out(&core)->host_armed);
  assert(!out(&core)->sbus_fresh && out(&core)->active_path == 3U);
  assert(out(&core)->servo_us == 1500U && out(&core)->esc_us == 1500U);
  assert(!rev6_core_command(&core, &neutral, 103U));
  rev6_core_tick(&core, 1000U, true, true);
  assert(!rev6_core_command(&core, &neutral, 1000U));
  assert(out(&core)->receiver_latched); /* Never USB-only after any frame. */

  ready_usb_only(&core);
  assert(rev6_core_command(&core, &neutral, 100U));
  failed.failsafe = false;
  failed.frame_lost = true;
  rev6_core_note_sbus(&core, &failed, 101U);
  assert(out(&core)->receiver_latched && !out(&core)->host_armed);
  assert(!rev6_core_command(&core, &neutral, 102U));

  /* Even a healthy first frame cannot keep the old USB-only arm while CH3 is
   * still undergoing its first 100 ms authority confirmation. */
  ready_usb_only(&core);
  assert(rev6_core_command(&core, &neutral, 100U));
  failed.frame_lost = false;
  rev6_core_note_sbus(&core, &failed, 101U);
  assert(!out(&core)->host_armed && out(&core)->selector == 0U);
  assert(!rev6_core_command(&core, &neutral, 102U));
}

static void lost_frames_do_not_extend_freshness_and_failsafe_is_immediate(void) {
  rev6_core_t core;
  rev6_core_init(&core);
  rev6_sbus_frame_t rc = receiver(REV6_MODE_RC);
  run_receiver(&core, &rc, 0U, 100U, false);
  rc.frame_lost = true;
  rc.channels[REV6_SBUS_STEERING_CHANNEL] = REV6_SBUS_RAW_MAX;
  rev6_core_note_sbus(&core, &rc, 140U);
  rev6_core_tick(&core, 150U, true, false);
  assert(out(&core)->sbus_fresh && out(&core)->active_path == 1U);
  assert(out(&core)->servo_us ==
         (REV6_SBUS_STEERING_REVERSED ? 2000U : 1000U));
  rev6_core_tick(&core, 151U, true, true);
  assert(!out(&core)->sbus_fresh && out(&core)->active_path == 3U);
  assert(out(&core)->rx_us[0] == 0U);
  jpbb_command_t neutral = host(0, 0, 0, 0);
  assert(!rev6_core_command(&core, &neutral, 152U));

  rev6_core_init(&core);
  rev6_sbus_frame_t pc = receiver(REV6_MODE_HOST);
  run_receiver(&core, &pc, 0U, 100U, true);
  assert(rev6_core_command(&core, &neutral, 100U));
  jpbb_command_t motion = host(0, 800, 0, 0);
  assert(rev6_core_command(&core, &motion, 101U));
  pc.failsafe = true;
  rev6_core_note_sbus(&core, &pc, 102U);
  assert(!out(&core)->host_armed && out(&core)->esc_us == 1500U);
  assert(out(&core)->active_path == 3U);
  assert(!rev6_core_command(&core, &neutral, 103U));

  /* Loss of HOST-authority SBUS must also bypass CH3's 100 ms loss debounce. */
  rev6_core_init(&core);
  pc.failsafe = false;
  run_receiver(&core, &pc, 0U, 100U, true);
  assert(rev6_core_command(&core, &neutral, 100U));
  rev6_core_tick(&core, 150U, true, true);
  assert(out(&core)->host_armed);
  rev6_core_tick(&core, 151U, true, true);
  assert(!out(&core)->host_armed && out(&core)->active_path == 3U);
}

static void power_loss_usb_loss_and_timeout_require_new_neutral(void) {
  rev6_core_t core;
  ready_usb_only(&core);
  jpbb_command_t neutral = host(0, 0, 0, 0);
  jpbb_command_t motion = host(400, 600, 0, 0);
  assert(rev6_core_command(&core, &neutral, 100U));
  assert(rev6_core_command(&core, &motion, 101U));
  rev6_core_tick(&core, 102U, false, true);
  assert(!out(&core)->output_enabled && !out(&core)->host_armed);
  rev6_core_tick(&core, 103U, true, true);
  assert(!rev6_core_command(&core, &neutral, 104U));
  rev6_core_tick(&core, 202U, true, true);
  assert(!out(&core)->output_enabled);
  rev6_core_tick(&core, 203U, true, true);
  assert(out(&core)->output_enabled && !out(&core)->host_armed);
  assert(out(&core)->fault_bits == 0U);
  assert(!rev6_core_command(&core, &motion, 204U));
  assert(rev6_core_command(&core, &neutral, 205U));
  assert(rev6_core_command(&core, &motion, 206U));
  rev6_core_tick(&core, 207U, true, false);
  assert(!out(&core)->host_armed && out(&core)->esc_us == 1500U);
  rev6_core_tick(&core, 208U, true, true);
  assert(!rev6_core_command(&core, &motion, 209U));
  assert(rev6_core_command(&core, &neutral, 210U));
  assert(out(&core)->fault_bits == 0U);
  assert(rev6_core_command(&core, &motion, 211U));
  rev6_core_tick(&core, 411U, true, true);
  assert(out(&core)->host_armed); /* age 200 ms is permitted. */
  /* A late motion frame must not conceal the timeout by refreshing first. */
  assert(!rev6_core_command(&core, &motion, 412U));
  assert(!out(&core)->host_armed);
  assert((out(&core)->fault_bits & REV6_FAULT_COMMAND_TIMEOUT) != 0U);
  neutral.flags = JPBB_HOST_FLAG_COMMAND_VALID;
  assert(rev6_core_command(&core, &neutral, 413U));
  assert(out(&core)->fault_bits == 0U && out(&core)->active_path == 0U);
  neutral.flags |= JPBB_HOST_FLAG_HOST_REQUEST;
  assert(rev6_core_command(&core, &neutral, 414U));
}

static void expired_receiver_stays_invalid_after_a_full_clock_cycle(void) {
  rev6_core_t core;
  rev6_core_init(&core);
  rev6_sbus_frame_t rc = receiver(REV6_MODE_RC);
  run_receiver(&core, &rc, 0U, 100U, false);
  assert(out(&core)->sbus_fresh && out(&core)->active_path == 1U);
  rev6_core_tick(&core, 151U, true, false);
  assert(!out(&core)->sbus_fresh);
  /* Simulate the remainder of a silent uint32_t ms cycle. A timestamp-only
   * freshness check would call the old frame fresh again at clock value 100. */
  rev6_core_tick(&core, UINT32_MAX, true, false);
  rev6_core_tick(&core, 100U, true, false);
  assert(!out(&core)->sbus_fresh && out(&core)->receiver_latched);
  assert(out(&core)->active_path == 3U && out(&core)->rx_us[0] == 0U);
  assert(out(&core)->servo_us == 1500U && out(&core)->esc_us == 1500U);
  rc.channels[REV6_SBUS_STEERING_CHANNEL] = REV6_SBUS_RAW_MAX;
  run_receiver(&core, &rc, 110U, 150U, false);
  assert(out(&core)->sbus_fresh && out(&core)->active_path == 1U);
  assert(out(&core)->servo_us ==
         (REV6_SBUS_STEERING_REVERSED ? 1000U : 2000U));
}

static void cli_staging_heartbeat_and_invalidation(void) {
  rev6_core_t core;
  ready_usb_only(&core);
  jpbb_command_t auto_command = cli(JPBB_COMMAND_AUTO, 0U, 0U);
  jpbb_command_t neutral = cli(JPBB_COMMAND_SET, 1500U, 1500U);
  jpbb_command_t stream = cli(JPBB_COMMAND_STREAM, 1600U, 1200U);
  assert(!rev6_core_command(&core, &auto_command, 100U));
  assert(rev6_core_command(&core, &stream, 100U));
  assert(!rev6_core_command(&core, &auto_command, 101U));
  assert(rev6_core_command(&core, &neutral, 102U));
  assert(rev6_core_command(&core, &auto_command, 102U));
  assert(out(&core)->host_armed);
  for (uint32_t now = 103U; now <= 602U; ++now) {
    rev6_core_tick(&core, now, true, true);
    if (now % 100U == 0U) {
      assert(rev6_core_command(&core, &stream, now));
    }
  }
  assert(out(&core)->host_armed); /* heartbeat age exactly 500 ms */
  rev6_core_tick(&core, 603U, true, true);
  assert(!out(&core)->host_armed);
  assert((out(&core)->fault_bits & REV6_FAULT_HEARTBEAT_TIMEOUT) != 0U);
  assert(!rev6_core_command(&core, &auto_command, 604U));
  assert(rev6_core_command(&core, &neutral, 605U));
  assert(rev6_core_command(&core, &auto_command, 606U));
  jpbb_command_t heartbeat = cli(JPBB_COMMAND_HEARTBEAT, 0U, 0U);
  assert(rev6_core_command(&core, &heartbeat, 607U));
  jpbb_command_t manual = cli(JPBB_COMMAND_MANUAL, 0U, 0U);
  assert(rev6_core_command(&core, &manual, 608U));
  assert(!out(&core)->host_armed);
  assert(!rev6_core_command(&core, &auto_command, 609U));
  assert(rev6_core_command(&core, &neutral, 610U));
  rev6_core_tick(&core, 611U, false, true);
  rev6_core_tick(&core, 612U, true, true);
  rev6_core_tick(&core, 712U, true, true);
  assert(!rev6_core_command(&core, &auto_command, 712U));
  assert(rev6_core_command(&core, &neutral, 713U));
  assert(rev6_core_command(&core, &auto_command, 714U));
  stream.esc_us = 999U;
  assert(!rev6_core_command(&core, &stream, 715U));
  assert(!out(&core)->host_armed);
}

static void timestamps_wrap_and_power_glitch_restarts_confirmation(void) {
  rev6_core_t core;
  rev6_core_init(&core);
  rev6_core_tick(&core, UINT32_MAX - 50U, true, true);
  rev6_core_tick(&core, 48U, true, true);
  assert(!out(&core)->power_ready);
  rev6_core_tick(&core, 49U, true, true);
  assert(out(&core)->power_ready);
  rev6_core_tick(&core, 50U, false, true);
  rev6_core_tick(&core, 51U, true, true);
  rev6_core_tick(&core, 150U, true, true);
  assert(!out(&core)->power_ready);
  rev6_core_tick(&core, 151U, true, true);
  assert(out(&core)->power_ready);

  ready_usb_only(&core);
  jpbb_command_t neutral = host(0, 0, 0, 0);
  assert(rev6_core_command(&core, &neutral, UINT32_MAX - 10U));
  rev6_core_tick(&core, 189U, true, true);
  assert(out(&core)->host_armed);
  rev6_core_tick(&core, 190U, true, true);
  assert(!out(&core)->host_armed);
}

int main(void) {
  boot_power_confirmation_and_exact_neutral_arm();
  host_polarity_ranges_flags_and_duplicate_sequence();
  rc_regeneration_without_usb_and_mode_confirmation();
  middle_switch_is_unknown_after_100_ms();
  first_faulted_sbus_latches_and_immediately_revokes_usb_only();
  lost_frames_do_not_extend_freshness_and_failsafe_is_immediate();
  power_loss_usb_loss_and_timeout_require_new_neutral();
  expired_receiver_stays_invalid_after_a_full_clock_cycle();
  cli_staging_heartbeat_and_invalidation();
  timestamps_wrap_and_power_glitch_restarts_confirmation();
  puts("Control core tests passed");
  return 0;
}
