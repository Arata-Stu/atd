#ifndef JPBB_PICO_PROTOCOL_BOARD_H
#define JPBB_PICO_PROTOCOL_BOARD_H

/* Minimal adapter for the unchanged, portable legacy JPB1 protocol module.
 * No STM32 HAL or legacy board pin definitions belong in the Pico target. */
#include <stdbool.h>
#include <stdint.h>

#define JPBB_PWM_MIN_US 1000U
#define JPBB_PWM_NEUTRAL_US 1500U
#define JPBB_SERVO_SAFE_US 1500U
#define JPBB_ESC_SAFE_US 1500U
#define JPBB_PWM_MAX_US 2000U

typedef struct {
  uint16_t pulse_us;
  uint32_t last_update_ms;
  bool valid;
} jpbb_rc_channel_t;

typedef struct {
  jpbb_rc_channel_t channel[3];
  uint16_t vbec_mv;
  bool usb_present;
  bool auto_selected;
} jpbb_status_t;

void jpbb_status_snapshot(jpbb_status_t *status);
void jpbb_pwm_get(uint16_t *servo_us, uint16_t *esc_us);

#endif
