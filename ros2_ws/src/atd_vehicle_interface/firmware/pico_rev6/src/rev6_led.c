#include "rev6_led.h"

#include <string.h>

void rev6_led_init(rev6_led_t *led) {
  memset(led, 0, sizeof(*led));
}

static rev6_led_mode_t select_mode(const rev6_led_t *led,
                                    const rev6_core_output_t *output,
                                    bool usb_ready) {
  const uint32_t faults = output->fault_bits & ~REV6_FAULT_POWER;
  if (faults != 0U || (led->power_seen && !output->power_ready)) {
    return REV6_LED_FAULT;
  }
  if (output->power_ready && output->output_enabled) {
    if (output->active_path == 2U && output->host_armed && usb_ready) {
      return REV6_LED_HOST_ACTIVE;
    }
    if (output->active_path == 1U && output->sbus_fresh) {
      return REV6_LED_RC_ACTIVE;
    }
  }
  if (output->sbus_fresh) {
    return REV6_LED_RC_READY;
  }
  return usb_ready ? REV6_LED_USB_LINK : REV6_LED_WAITING;
}

bool rev6_led_update(rev6_led_t *led, const rev6_core_output_t *output,
                     bool usb_ready, uint32_t now_ms) {
  led->power_seen = led->power_seen || output->power_ready;
  const rev6_led_mode_t mode = select_mode(led, output, usb_ready);
  const uint16_t period = mode == REV6_LED_FAULT ? 200U
      : (mode == REV6_LED_RC_ACTIVE || mode == REV6_LED_HOST_ACTIVE)
          ? 1000U : 2000U;
  if (!led->initialized || mode != led->mode) {
    /* Start a new visible pattern immediately at every state change. */
    led->initialized = true;
    led->mode = mode;
    led->phase_ms = 0U;
  } else {
    const uint32_t elapsed = now_ms - led->last_ms;
    led->phase_ms = (uint16_t)((led->phase_ms + elapsed % period) % period);
  }
  led->last_ms = now_ms;
  const uint16_t phase = led->phase_ms;
  switch (mode) {
    case REV6_LED_RC_ACTIVE:
      return phase < 500U; /* 0.5 s on / 0.5 s off. */
    case REV6_LED_HOST_ACTIVE:
      return phase < 900U; /* Mostly lit, 0.1 s off each second. */
    case REV6_LED_FAULT:
      return phase < 100U; /* Five evenly spaced flashes per second. */
    case REV6_LED_RC_READY:
      return phase < 120U || (phase >= 250U && phase < 370U) ||
             (phase >= 500U && phase < 620U);
    case REV6_LED_USB_LINK:
      return phase < 120U || (phase >= 250U && phase < 370U);
    case REV6_LED_WAITING:
    default:
      return phase < 120U;
  }
}

const char *rev6_led_mode_name(rev6_led_mode_t mode) {
  switch (mode) {
    case REV6_LED_WAITING: return "WAITING";
    case REV6_LED_USB_LINK: return "USB_LINK";
    case REV6_LED_RC_READY: return "RC_READY";
    case REV6_LED_RC_ACTIVE: return "RC_ACTIVE";
    case REV6_LED_HOST_ACTIVE: return "HOST_ACTIVE";
    case REV6_LED_FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}
