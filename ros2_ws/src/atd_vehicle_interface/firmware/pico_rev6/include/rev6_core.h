#ifndef REV6_CORE_H
#define REV6_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "jpbb_protocol.h"
#include "rev6_sbus.h"

/* Zero-based channel indices. Calibrate these defaults on the actual receiver. */
#ifndef REV6_SBUS_STEERING_CHANNEL
#define REV6_SBUS_STEERING_CHANNEL 0U
#endif
#ifndef REV6_SBUS_THROTTLE_CHANNEL
#define REV6_SBUS_THROTTLE_CHANNEL 1U
#endif
#ifndef REV6_SBUS_MODE_CHANNEL
#define REV6_SBUS_MODE_CHANNEL 2U
#endif
#ifndef REV6_SBUS_STEERING_REVERSED
#define REV6_SBUS_STEERING_REVERSED 0
#endif
#ifndef REV6_SBUS_THROTTLE_REVERSED
#define REV6_SBUS_THROTTLE_REVERSED 0
#endif
#ifndef REV6_SBUS_MODE_REVERSED
#define REV6_SBUS_MODE_REVERSED 0
#endif
#ifndef REV6_SBUS_RAW_MIN
#define REV6_SBUS_RAW_MIN 172U
#endif
#ifndef REV6_SBUS_RAW_MID
#define REV6_SBUS_RAW_MID 992U
#endif
#ifndef REV6_SBUS_RAW_MAX
#define REV6_SBUS_RAW_MAX 1811U
#endif

#define REV6_PWM_MIN_US 1000U
#define REV6_PWM_MAX_US 2000U
#define REV6_PWM_NEUTRAL_US 1500U
#define REV6_SBUS_FRESH_MS 50U
#define REV6_POWER_CONFIRM_MS 100U
#define REV6_COMMAND_TIMEOUT_MS 200U
#define REV6_HEARTBEAT_TIMEOUT_MS 500U

#define REV6_FAULT_POWER (1UL << 0)
#define REV6_FAULT_USB (1UL << 1)
#define REV6_FAULT_RECEIVER_LOSS (1UL << 2)
#define REV6_FAULT_MODE_UNKNOWN (1UL << 3)
#define REV6_FAULT_RC_SIGNAL (1UL << 4)
#define REV6_FAULT_COMMAND_TIMEOUT (1UL << 5)
#define REV6_FAULT_HEARTBEAT_TIMEOUT (1UL << 6)
#define REV6_FAULT_COMMAND_VALUE (1UL << 7)

typedef enum {
  REV6_MODE_UNKNOWN = 0,
  REV6_MODE_RC,
  REV6_MODE_HOST
} rev6_mode_t;

typedef struct {
  uint16_t servo_us;
  uint16_t esc_us;
  /* False suppresses PWM pulses. The UV circuit gates/clamps independently. */
  bool output_enabled;
  uint16_t rx_us[3]; /* selected steering / throttle / authority, not SBUS raw */
  uint8_t selector; /* JPB1: 0 = RC, 1 = HOST permitted */
  uint8_t active_path; /* JPB1: 0 = disabled, 1 = RC, 2 = HOST, 3 = failsafe */
  uint32_t fault_bits;
  bool power_ready;
  bool sbus_fresh;
  bool receiver_latched;
  bool host_armed;
} rev6_core_output_t;

/* Public storage for static allocation. Treat fields as private to this module. */
typedef struct {
  rev6_core_output_t state;
  bool power_high;
  bool usb_ready;
  uint32_t power_high_since_ms;
  bool receiver_latched;
  bool good_sbus_received;
  bool sbus_failsafe;
  uint32_t last_good_sbus_ms;
  uint16_t rx_us[3];
  rev6_mode_t mode;
  rev6_mode_t mode_candidate;
  uint32_t mode_candidate_since_ms;
  bool host_armed;
  bool command_received;
  bool heartbeat_received;
  bool cli_staged;
  uint32_t last_command_ms;
  uint32_t last_heartbeat_ms;
  uint16_t desired_servo_us;
  uint16_t desired_esc_us;
  uint32_t host_faults;
} rev6_core_t;

void rev6_core_init(rev6_core_t *core);
/* Call at 1 ms intervals. All time arguments are a wrapping monotonic ms clock. */
void rev6_core_tick(rev6_core_t *core, uint32_t now_ms, bool power_good,
                     bool usb_ready);
/* Call for every structurally valid SBUS frame, even failsafe/frame_lost frames. */
void rev6_core_note_sbus(rev6_core_t *core, const rev6_sbus_frame_t *frame,
                          uint32_t now_ms);
/* Parse first with jpbb_protocol_parse(). STATUS/HELP are side-effect free.
 * SET/STREAM stage/update PWM; AUTO can arm only a fresh neutral SET/STREAM.
 * A HOST_FRAME arms only when that same frame explicitly requests neutral HOST.
 * Sequence duplicate rejection is intentionally left compatible with old JPB1.
 */
bool rev6_core_command(rev6_core_t *core, const jpbb_command_t *command,
                        uint32_t now_ms);
/* Snapshot while holding the same short IRQ exclusion as the writers. */
const rev6_core_output_t *rev6_core_output(const rev6_core_t *core);

#endif
