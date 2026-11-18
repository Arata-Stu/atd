#include "rev6_core.h"

#include <stddef.h>
#include <string.h>

_Static_assert(REV6_SBUS_STEERING_CHANNEL < REV6_SBUS_CHANNELS,
               "Steering SBUS channel must be 0..15");
_Static_assert(REV6_SBUS_THROTTLE_CHANNEL < REV6_SBUS_CHANNELS,
               "Throttle SBUS channel must be 0..15");
_Static_assert(REV6_SBUS_MODE_CHANNEL < REV6_SBUS_CHANNELS,
               "Mode SBUS channel must be 0..15");
_Static_assert(REV6_SBUS_RAW_MIN < REV6_SBUS_RAW_MID &&
                   REV6_SBUS_RAW_MID < REV6_SBUS_RAW_MAX &&
                   REV6_SBUS_RAW_MAX <= 2047U,
               "SBUS calibration must satisfy 0 <= min < mid < max <= 2047");

static bool fresh(uint32_t now, uint32_t then, uint32_t timeout) {
  return (uint32_t)(now - then) <= timeout;
}

static uint16_t channel_us(uint16_t raw, bool reversed) {
  uint32_t pulse;
  if (raw <= REV6_SBUS_RAW_MIN) {
    pulse = REV6_PWM_MIN_US;
  } else if (raw >= REV6_SBUS_RAW_MAX) {
    pulse = REV6_PWM_MAX_US;
  } else if (raw <= REV6_SBUS_RAW_MID) {
    const uint32_t span = REV6_SBUS_RAW_MID - REV6_SBUS_RAW_MIN;
    pulse = REV6_PWM_MIN_US +
            ((uint32_t)(raw - REV6_SBUS_RAW_MIN) * 500U + span / 2U) / span;
  } else {
    const uint32_t span = REV6_SBUS_RAW_MAX - REV6_SBUS_RAW_MID;
    pulse = REV6_PWM_NEUTRAL_US +
            ((uint32_t)(raw - REV6_SBUS_RAW_MID) * 500U + span / 2U) / span;
  }
  return (uint16_t)(reversed ? 3000U - pulse : pulse);
}

static bool sbus_fresh(const rev6_core_t *core, uint32_t now_ms) {
  return core->good_sbus_received && !core->sbus_failsafe &&
         fresh(now_ms, core->last_good_sbus_ms, REV6_SBUS_FRESH_MS);
}

static void disarm(rev6_core_t *core, bool invalidate_command) {
  core->host_armed = false;
  if (invalidate_command) {
    core->command_received = false;
    core->heartbeat_received = false;
    core->cli_staged = false;
    core->desired_servo_us = REV6_PWM_NEUTRAL_US;
    core->desired_esc_us = REV6_PWM_NEUTRAL_US;
  }
}

static bool host_permitted(const rev6_core_t *core, uint32_t now_ms) {
  return !core->receiver_latched ||
         (sbus_fresh(core, now_ms) && core->mode == REV6_MODE_HOST);
}

static void update_mode(rev6_core_t *core, uint32_t now_ms, bool valid) {
  rev6_mode_t observed = REV6_MODE_UNKNOWN;
  if (valid) {
    if (core->rx_us[2] <= 1300U) {
      observed = REV6_MODE_RC;
    } else if (core->rx_us[2] >= 1700U) {
      observed = REV6_MODE_HOST;
    }
  }
  if (observed == core->mode) {
    core->mode_candidate = observed;
    core->mode_candidate_since_ms = now_ms;
    return;
  }
  if (observed != core->mode_candidate) {
    core->mode_candidate = observed;
    core->mode_candidate_since_ms = now_ms;
    return;
  }
  const uint32_t confirmation = observed == REV6_MODE_RC ? 40U : 100U;
  if ((uint32_t)(now_ms - core->mode_candidate_since_ms) >= confirmation) {
    core->mode = observed;
    core->mode_candidate_since_ms = now_ms;
  }
}

static void evaluate(rev6_core_t *core, uint32_t now_ms) {
  /* Once stale, a stored frame stays invalid until another good frame arrives.
   * The 1 ms caller observes expiry before the uint32_t ms clock can wrap back
   * into the old timestamp's freshness window after a full cycle of silence. */
  if (core->good_sbus_received &&
      !fresh(now_ms, core->last_good_sbus_ms, REV6_SBUS_FRESH_MS)) {
    core->good_sbus_received = false;
  }
  const bool rc_valid = sbus_fresh(core, now_ms);
  update_mode(core, now_ms, rc_valid);

  if (!core->state.power_ready || !core->usb_ready ||
      (core->receiver_latched && !rc_valid) ||
      (core->receiver_latched && core->mode != REV6_MODE_HOST)) {
    if (core->host_armed && !core->usb_ready) {
      core->host_faults |= REV6_FAULT_USB;
    }
    disarm(core, true);
  } else if (core->host_armed) {
    uint32_t expired = 0U;
    if (!core->command_received ||
        !fresh(now_ms, core->last_command_ms, REV6_COMMAND_TIMEOUT_MS)) {
      expired |= REV6_FAULT_COMMAND_TIMEOUT;
    }
    if (!core->heartbeat_received ||
        !fresh(now_ms, core->last_heartbeat_ms, REV6_HEARTBEAT_TIMEOUT_MS)) {
      expired |= REV6_FAULT_HEARTBEAT_TIMEOUT;
    }
    if (expired != 0U) {
      core->host_faults |= expired;
      disarm(core, true);
    }
  }

  /* Ready RC operation must not inherit unrelated previous HOST faults. */
  if (core->state.power_ready && rc_valid && core->mode == REV6_MODE_RC) {
    core->host_faults = 0U;
  }
  uint32_t faults = core->host_faults;
  if (!core->state.power_ready) {
    faults |= REV6_FAULT_POWER;
  }
  if (core->receiver_latched) {
    if (!rc_valid) {
      faults |= REV6_FAULT_RECEIVER_LOSS;
    } else if (core->mode == REV6_MODE_UNKNOWN) {
      faults |= REV6_FAULT_MODE_UNKNOWN;
    }
    if (core->sbus_failsafe) {
      faults |= REV6_FAULT_RC_SIGNAL;
    }
  }

  rev6_core_output_t *out = &core->state;
  out->servo_us = REV6_PWM_NEUTRAL_US;
  out->esc_us = REV6_PWM_NEUTRAL_US;
  out->output_enabled = out->power_ready;
  out->selector = host_permitted(core, now_ms) ? 1U : 0U;
  out->active_path = faults == 0U ? 0U : 3U;
  if (out->power_ready && rc_valid && core->mode == REV6_MODE_RC) {
    out->servo_us = core->rx_us[0];
    out->esc_us = core->rx_us[1];
    out->active_path = 1U;
  } else if (out->power_ready && core->host_armed) {
    out->servo_us = core->desired_servo_us;
    out->esc_us = core->desired_esc_us;
    out->active_path = 2U;
  }
  out->rx_us[0] = rc_valid ? core->rx_us[0] : 0U;
  out->rx_us[1] = rc_valid ? core->rx_us[1] : 0U;
  out->rx_us[2] = rc_valid ? core->rx_us[2] : 0U;
  out->fault_bits = faults;
  out->sbus_fresh = rc_valid;
  out->receiver_latched = core->receiver_latched;
  out->host_armed = core->host_armed;
}

void rev6_core_init(rev6_core_t *core) {
  if (core == NULL) {
    return;
  }
  memset(core, 0, sizeof(*core));
  core->desired_servo_us = REV6_PWM_NEUTRAL_US;
  core->desired_esc_us = REV6_PWM_NEUTRAL_US;
  core->state.servo_us = REV6_PWM_NEUTRAL_US;
  core->state.esc_us = REV6_PWM_NEUTRAL_US;
  core->state.active_path = 3U;
  core->state.selector = 1U;
  core->state.fault_bits = REV6_FAULT_POWER;
}

void rev6_core_tick(rev6_core_t *core, uint32_t now_ms, bool power_good,
                     bool usb_ready) {
  if (core == NULL) {
    return;
  }
  core->usb_ready = usb_ready;
  if (!power_good) {
    core->power_high = false;
    core->state.power_ready = false;
    core->host_faults = 0U;
    disarm(core, true);
  } else {
    if (!core->power_high) {
      core->power_high = true;
      core->power_high_since_ms = now_ms;
    }
    if (!core->state.power_ready &&
        (uint32_t)(now_ms - core->power_high_since_ms) >= REV6_POWER_CONFIRM_MS) {
      core->state.power_ready = true;
    }
  }
  evaluate(core, now_ms);
}

void rev6_core_note_sbus(rev6_core_t *core, const rev6_sbus_frame_t *frame,
                          uint32_t now_ms) {
  if (core == NULL || frame == NULL) {
    return;
  }
  if (!core->receiver_latched) {
    /* A fault-flagged but structurally valid frame also forbids USB-only mode. */
    core->receiver_latched = true;
    disarm(core, true);
  }
  if (frame->failsafe) {
    core->sbus_failsafe = true;
    core->good_sbus_received = false;
    disarm(core, true);
  } else if (!frame->frame_lost) {
    core->sbus_failsafe = false;
    core->good_sbus_received = true;
    core->last_good_sbus_ms = now_ms;
    core->rx_us[0] = channel_us(frame->channels[REV6_SBUS_STEERING_CHANNEL],
                                REV6_SBUS_STEERING_REVERSED != 0);
    core->rx_us[1] = channel_us(frame->channels[REV6_SBUS_THROTTLE_CHANNEL],
                                REV6_SBUS_THROTTLE_REVERSED != 0);
    core->rx_us[2] = channel_us(frame->channels[REV6_SBUS_MODE_CHANNEL],
                                REV6_SBUS_MODE_REVERSED != 0);
  }
  evaluate(core, now_ms);
}

static bool host_pwm(const jpbb_command_t *command, uint16_t *servo,
                      uint16_t *esc) {
  if (command->steering_milli < -1000 || command->steering_milli > 1000 ||
      command->throttle_milli > 1000U || command->reverse_milli > 1000U ||
      command->brake_milli > 1000U ||
      (command->throttle_milli != 0U && command->reverse_milli != 0U)) {
    return false;
  }
  *servo = (uint16_t)(1500 + command->steering_milli / 2);
  /* Preserve legacy JPB1 polarity and brake priority. */
  *esc = command->brake_milli != 0U ? (uint16_t)(1500U + command->brake_milli / 2U)
         : command->reverse_milli != 0U
             ? (uint16_t)(1500U + command->reverse_milli / 2U)
             : (uint16_t)(1500U - command->throttle_milli / 2U);
  return true;
}

static bool neutral_frame(const jpbb_command_t *command) {
  /* A tiny nonzero value that rounds to 1500 us is not an arming request. */
  return command->steering_milli == 0 && command->throttle_milli == 0U &&
         command->reverse_milli == 0U && command->brake_milli == 0U;
}

bool rev6_core_command(rev6_core_t *core, const jpbb_command_t *command,
                        uint32_t now_ms) {
  if (core == NULL || command == NULL) {
    return false;
  }
  /* Check timeouts before a new frame can refresh the old armed session. */
  evaluate(core, now_ms);
  switch (command->kind) {
    case JPBB_COMMAND_NONE:
    case JPBB_COMMAND_STATUS:
    case JPBB_COMMAND_HELP:
      return true;
    case JPBB_COMMAND_MANUAL:
      disarm(core, true);
      evaluate(core, now_ms);
      return true;
    case JPBB_COMMAND_PING:
    case JPBB_COMMAND_HEARTBEAT:
      if (!core->usb_ready) {
        return false;
      }
      core->heartbeat_received = true;
      core->last_heartbeat_ms = now_ms;
      evaluate(core, now_ms);
      return true;
    case JPBB_COMMAND_SET:
    case JPBB_COMMAND_STREAM:
      if (command->servo_us < REV6_PWM_MIN_US ||
          command->servo_us > REV6_PWM_MAX_US ||
          command->esc_us < REV6_PWM_MIN_US ||
          command->esc_us > REV6_PWM_MAX_US) {
        core->host_faults |= REV6_FAULT_COMMAND_VALUE;
        disarm(core, true);
        evaluate(core, now_ms);
        return false;
      }
      if (!core->state.power_ready || !core->usb_ready ||
          !host_permitted(core, now_ms)) {
        return false;
      }
      core->desired_servo_us = command->servo_us;
      core->desired_esc_us = command->esc_us;
      core->command_received = true;
      core->last_command_ms = now_ms;
      core->cli_staged = true;
      if (command->servo_us == 1500U && command->esc_us == 1500U) {
        core->host_faults = 0U;
      }
      evaluate(core, now_ms);
      return true;
    case JPBB_COMMAND_AUTO:
      if (!core->state.power_ready || !core->usb_ready ||
          !host_permitted(core, now_ms) || !core->cli_staged ||
          !core->command_received ||
          !fresh(now_ms, core->last_command_ms, REV6_COMMAND_TIMEOUT_MS) ||
          core->desired_servo_us != 1500U || core->desired_esc_us != 1500U) {
        return false;
      }
      core->host_armed = true;
      core->heartbeat_received = true;
      core->last_heartbeat_ms = now_ms;
      core->host_faults = 0U;
      evaluate(core, now_ms);
      return core->host_armed;
    case JPBB_COMMAND_HOST_FRAME: {
      const uint16_t allowed = JPBB_HOST_FLAG_COMMAND_VALID |
                               JPBB_HOST_FLAG_HOST_REQUEST;
      uint16_t servo = 1500U;
      uint16_t esc = 1500U;
      const bool valid = (command->flags & JPBB_HOST_FLAG_COMMAND_VALID) != 0U;
      const bool request = (command->flags & JPBB_HOST_FLAG_HOST_REQUEST) != 0U;
      if ((command->flags & (uint16_t)~allowed) != 0U ||
          (valid && !host_pwm(command, &servo, &esc))) {
        core->host_faults |= REV6_FAULT_COMMAND_VALUE;
        disarm(core, true);
        evaluate(core, now_ms);
        return false;
      }
      if (!core->state.power_ready || !core->usb_ready) {
        disarm(core, true);
        evaluate(core, now_ms);
        return false;
      }
      core->heartbeat_received = true;
      core->last_heartbeat_ms = now_ms;
      core->cli_staged = false;
      if (!valid || !request) {
        disarm(core, true);
        if (valid && neutral_frame(command)) {
          core->host_faults = 0U;
        }
        evaluate(core, now_ms);
        return true;
      }
      if (!host_permitted(core, now_ms)) {
        disarm(core, true);
        evaluate(core, now_ms);
        return false;
      }
      if (!core->host_armed && !neutral_frame(command)) {
        /* Never stage motion for a later automatic arm. */
        return false;
      }
      core->host_armed = true;
      core->desired_servo_us = servo;
      core->desired_esc_us = esc;
      core->command_received = true;
      core->last_command_ms = now_ms;
      if (neutral_frame(command)) {
        core->host_faults = 0U;
      }
      evaluate(core, now_ms);
      return core->host_armed;
    }
    case JPBB_COMMAND_INVALID:
    default:
      return false;
  }
}

const rev6_core_output_t *rev6_core_output(const rev6_core_t *core) {
  return core == NULL ? NULL : &core->state;
}
