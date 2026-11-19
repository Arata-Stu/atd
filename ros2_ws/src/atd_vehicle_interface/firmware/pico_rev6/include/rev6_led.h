#ifndef REV6_LED_H
#define REV6_LED_H

#include <stdbool.h>
#include <stdint.h>

#include "rev6_core.h"

typedef enum {
  REV6_LED_WAITING = 0,
  REV6_LED_USB_LINK,
  REV6_LED_RC_READY,
  REV6_LED_RC_ACTIVE,
  REV6_LED_HOST_ACTIVE,
  REV6_LED_FAULT
} rev6_led_mode_t;

/* Storage owned by the control timer, not another control authority. */
typedef struct {
  rev6_led_mode_t mode;
  bool initialized;
  bool power_seen;
  uint32_t last_ms;
  uint16_t phase_ms;
} rev6_led_t;

void rev6_led_init(rev6_led_t *led);
/* Non-blocking GPIO level; call from the control tick after core evaluation.
 * USB link means enumeration, not a running Joy driver or an armed controller.
 * POWER alone is expected before any BEC-ready observation (USB-only bench).
 * Once BEC was ready, a loss is displayed as FAULT until power recovers.
 * The controller's fault bits/output policy are never changed by this module.
 */
bool rev6_led_update(rev6_led_t *led, const rev6_core_output_t *output,
                     bool usb_ready, uint32_t now_ms);
const char *rev6_led_mode_name(rev6_led_mode_t mode);

#endif
