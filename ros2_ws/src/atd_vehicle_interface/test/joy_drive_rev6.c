/* Host fixture driving the bundled Rev6 control loop and CDC0 wire format. */
#include "rev6_core.h"
#include "rev6_transport.h"
#include "jpbb_board.h"

#include <stdio.h>

static rev6_core_t core;
static uint32_t now_ms;
static uint16_t rc_steer;
static uint16_t rc_throttle;

/* Board hooks for the optional text STATUS command in jpbb_protocol.c. */
void jpbb_status_snapshot(jpbb_status_t *status) {
  *status = (jpbb_status_t){0};
}

void jpbb_pwm_get(uint16_t *servo_us, uint16_t *esc_us) {
  *servo_us = rev6_core_output(&core)->servo_us;
  *esc_us = rev6_core_output(&core)->esc_us;
}

void fixture_init(void) {
  now_ms = 0;
  rc_steer = REV6_SBUS_RAW_MIN;
  rc_throttle = REV6_SBUS_RAW_MAX;
  rev6_core_init(&core);
  rev6_core_tick(&core, 0, true, true);
}

void fixture_receiver(uint16_t steering, uint16_t throttle) {
  rc_steer = steering;
  rc_throttle = throttle;
}

void fixture_step(int duration_ms, int mode_raw, int usb_connected) {
  rev6_sbus_frame_t frame = {0};
  frame.channels[REV6_SBUS_STEERING_CHANNEL] = rc_steer;
  frame.channels[REV6_SBUS_THROTTLE_CHANNEL] = rc_throttle;
  frame.channels[REV6_SBUS_MODE_CHANNEL] = (uint16_t)mode_raw;
  for (int i = 0; i < duration_ms; ++i) {
    rev6_core_tick(&core, ++now_ms, true, usb_connected != 0);
    if (mode_raw >= 0 && now_ms % 10 == 0) {
      rev6_core_note_sbus(&core, &frame, now_ms);
    }
  }
}

int fixture_command(const char *line) {
  jpbb_command_t cmd;
  return jpbb_protocol_parse(line, &cmd) && rev6_core_command(&core, &cmd, now_ms);
}

const char *fixture_status(uint32_t sequence) {
  const rev6_core_output_t *s = rev6_core_output(&core);
  static char packet[192];
  snprintf(packet, sizeof(packet), "JPB1,S,%u,%u,%u,%u,%u,%u,6000,%u,%u,%u",
           sequence, s->rx_us[0], s->rx_us[1], s->rx_us[2], s->servo_us, s->esc_us,
           s->selector, s->active_path, s->fault_bits);
  rev6_frame_finish(packet, sizeof(packet));
  return packet;
}
