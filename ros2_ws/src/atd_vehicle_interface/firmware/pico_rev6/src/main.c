#include "jpbb_board.h"
#include "jpbb_protocol.h"
#include "rev6_core.h"
#include "rev6_hal.h"
#include "rev6_led.h"
#include "rev6_sbus.h"
#include "rev6_transport.h"

#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "tusb.h"
#include "device/dcd.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Only the 1 kHz control callback feeds the hardware watchdog. UART and control
 * share priority; lower-priority RPM updates take a short IRQ exclusion.
 * Main takes short IRQ-disabled snapshots; USB/parsing/formatting stay outside. */
#define REV6_WATCHDOG_MS 100U
#define REV6_STATUS_PERIOD_MS 50U
#define REV6_SENSOR_PERIOD_MS 20U
#define REV6_RX_BUDGET 128U

typedef struct {
  rev6_sbus_frame_t frame;
  rev6_rpm_t rpm;
  uint32_t sbus_frames;
  uint32_t sbus_lost_frames;
  uint32_t sbus_failsafe_frames;
  uint32_t sbus_last_ms;
  uint32_t uart_errors;
} sensor_state_t;

static rev6_core_t core;
static rev6_led_t led;
static rev6_sbus_decoder_t decoder;
static sensor_state_t sensors;
static volatile bool usb_healthy;
static volatile uint32_t usb_fault_epoch;
static volatile uint16_t vbec_mv;
static bool vbec_measured;
static rev6_line_t lines[2];
static uint32_t tx_dropped[2];
static uint32_t rx_rejected[2];
static bool jpb1_active;
static bool sensor_stream;
static bool boot_requested;
static struct repeating_timer control_timer;

static uint32_t millis(void) {
  return to_ms_since_boot(get_absolute_time());
}

static void usb_fault(void) {
  const uint32_t state = save_and_disable_interrupts();
  ++usb_fault_epoch;
  usb_healthy = false;
  restore_interrupts(state);
}

/* Sticky event history is required: reset/suspend and reconfiguration/resume
 * can otherwise be consumed in a single tud_task() call, hiding the outage. */
void tud_event_hook_cb(uint8_t rhport, uint32_t eventid, bool in_isr) {
  (void)rhport;
  (void)in_isr;
  if (eventid == DCD_EVENT_BUS_RESET || eventid == DCD_EVENT_UNPLUGGED ||
      eventid == DCD_EVENT_SUSPEND) {
    usb_fault();
  }
}

void tud_umount_cb(void) {
  usb_fault();
}

static void copy_output(rev6_core_output_t *output) {
  const uint32_t state = save_and_disable_interrupts();
  *output = *rev6_core_output(&core);
  restore_interrupts(state);
}

void jpbb_pwm_get(uint16_t *servo, uint16_t *esc) {
  rev6_core_output_t output;
  copy_output(&output);
  *servo = output.servo_us;
  *esc = output.esc_us;
}

void jpbb_status_snapshot(jpbb_status_t *status) {
  rev6_core_output_t output;
  copy_output(&output);
  memset(status, 0, sizeof(*status));
  for (size_t i = 0U; i < 3U; ++i) {
    status->channel[i].pulse_us = output.rx_us[i];
    status->channel[i].valid = output.sbus_fresh;
  }
  status->vbec_mv = vbec_mv;
  status->usb_present = usb_healthy;
  status->auto_selected = output.selector == 1U;
}

static void sbus_byte(uint8_t byte, uint32_t now_us) {
  static uint32_t last_byte_us;
  static bool have_byte;
  if (have_byte && now_us - last_byte_us > 2000U) {
    rev6_sbus_init(&decoder);
  }
  last_byte_us = now_us;
  have_byte = true;
  rev6_sbus_frame_t frame;
  if (rev6_sbus_feed(&decoder, byte, &frame)) {
    const uint32_t now = millis();
    sensors.frame = frame;
    ++sensors.sbus_frames;
    sensors.sbus_lost_frames += frame.frame_lost ? 1U : 0U;
    sensors.sbus_failsafe_frames += frame.failsafe ? 1U : 0U;
    sensors.sbus_last_ms = now;
    rev6_core_note_sbus(&core, &frame, now);
  }
}

static void sbus_error(void) {
  ++sensors.uart_errors;
  rev6_sbus_init(&decoder);
}

static void rpm_edge(uint32_t now_us) {
  rev6_rpm_edge(&sensors.rpm, now_us);
}

static bool control_tick(struct repeating_timer *timer) {
  (void)timer;
  const uint32_t now = millis();
  if (rev6_hal_take_uv_fault()) {
    rev6_core_tick(&core, now, false, usb_healthy);
  }
  rev6_core_tick(&core, now, rev6_hal_uv_good(), usb_healthy);
  rev6_rpm_expire(&sensors.rpm, time_us_32());
  const rev6_core_output_t *output = rev6_core_output(&core);
  rev6_hal_pwm(output->servo_us, output->esc_us, output->output_enabled);
  rev6_hal_led(rev6_led_update(&led, output, usb_healthy, now));
  watchdog_update();
  return true;
}

/* A frame is queued whole or dropped whole. Never block on host reads and
 * never leave half a CSV frame in the software TX FIFO under backpressure. */
static void send_bytes(uint8_t port, const char *text, size_t length) {
  if (!usb_healthy || length == 0U) {
    return;
  }
  if (tud_cdc_n_write_available(port) < length) {
    ++tx_dropped[port];
    return;
  }
  if (tud_cdc_n_write(port, text, (uint32_t)length) != length) {
    ++tx_dropped[port];
  }
}

static void send_text(uint8_t port, const char *text) {
  send_bytes(port, text, strlen(text));
}

static void send_status(uint32_t sequence) {
  rev6_core_output_t output;
  char packet[192];
  copy_output(&output);
  const int count = snprintf(packet, sizeof(packet),
      "JPB1,S,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%lu",
      (unsigned long)sequence, output.rx_us[0], output.rx_us[1], output.rx_us[2],
      output.servo_us, output.esc_us, vbec_mv, output.selector, output.active_path,
      (unsigned long)output.fault_bits);
  if (count > 0 && (size_t)count < sizeof(packet)) {
    send_bytes(0U, packet, rev6_frame_finish(packet, sizeof(packet)));
  }
}

static bool append(char *text, size_t capacity, size_t *used, const char *format, ...) {
  va_list args;
  va_start(args, format);
  const int count = vsnprintf(text + *used, capacity - *used, format, args);
  va_end(args);
  if (count < 0 || (size_t)count >= capacity - *used) {
    return false;
  }
  *used += (size_t)count;
  return true;
}

static void send_sensors(uint32_t sequence) {
  sensor_state_t snapshot;
  rev6_core_output_t output;
  const uint32_t state = save_and_disable_interrupts();
  snapshot = sensors;
  output = *rev6_core_output(&core);
  restore_interrupts(state);
  char packet[480];
  size_t used = 0U;
  const uint64_t now_us = time_us_64();
  const uint32_t age = snapshot.sbus_frames ? millis() - snapshot.sbus_last_ms : UINT32_MAX;
  const unsigned flags = (snapshot.frame.digital17 ? 1U : 0U) |
                         (snapshot.frame.digital18 ? 2U : 0U) |
                         (snapshot.frame.frame_lost ? 4U : 0U) |
                         (snapshot.frame.failsafe ? 8U : 0U);
  bool ok = append(packet, sizeof(packet), &used,
      "JPX1,T,%lu,%llu,%lu,%lu,%lu,%lu,%lu,%u",
      (unsigned long)sequence, (unsigned long long)now_us, (unsigned long)age,
      (unsigned long)snapshot.sbus_frames, (unsigned long)snapshot.sbus_lost_frames,
      (unsigned long)snapshot.sbus_failsafe_frames, (unsigned long)snapshot.uart_errors, flags);
  for (size_t i = 0U; ok && i < 16U; ++i) {
    ok = append(packet, sizeof(packet), &used, ",%u", snapshot.frame.channels[i]);
  }
  ok = ok && append(packet, sizeof(packet), &used,
      ",%lu,%lu,%lu,%u,%u,%u,%u,%u,%u,%lu,%lu,%lu,%lu,%lu",
      (unsigned long)snapshot.rpm.edges, (unsigned long)snapshot.rpm.last_edge_us,
      (unsigned long)snapshot.rpm.period_us,
      rev6_rpm_period_recent(&snapshot.rpm, (uint32_t)now_us) ? 1U : 0U,
      snapshot.rpm.seen_edge ? 1U : 0U, vbec_mv, output.power_ready ? 1U : 0U,
      output.sbus_fresh ? 1U : 0U, output.active_path, (unsigned long)output.fault_bits,
      (unsigned long)tx_dropped[0], (unsigned long)tx_dropped[1],
      (unsigned long)rx_rejected[0], (unsigned long)rx_rejected[1]);
  if (ok) {
    send_bytes(1U, packet, rev6_frame_finish(packet, sizeof(packet)));
  } else {
    ++tx_dropped[1];
  }
}

static void send_led_status(void) {
  rev6_core_output_t output;
  rev6_led_mode_t mode;
  bool power_seen, usb_ready;
  const uint32_t state = save_and_disable_interrupts();
  output = *rev6_core_output(&core);
  mode = led.mode;
  power_seen = led.power_seen;
  usb_ready = usb_healthy;
  restore_interrupts(state);
  char response[256];
  const int count = snprintf(response, sizeof(response),
      "LED state=%s usb=%u power_ready=%u power_seen=%u sbus_fresh=%u "
      "host_armed=%u path=%u fault=0x%02lx\r\n",
      rev6_led_mode_name(mode), usb_ready ? 1U : 0U,
      output.power_ready ? 1U : 0U, power_seen ? 1U : 0U,
      output.sbus_fresh ? 1U : 0U, output.host_armed ? 1U : 0U,
      output.active_path, (unsigned long)output.fault_bits);
  if (count > 0 && (size_t)count < sizeof(response)) {
    send_bytes(1U, response, (size_t)count);
  }
}

static void diagnostic_line(const char *line) {
  if (strcmp(line, "STREAM ON") == 0) {
    sensor_stream = true;
    send_text(1U, "OK STREAM 50Hz; IMU=DISABLED\r\n");
  } else if (strcmp(line, "STREAM OFF") == 0) {
    sensor_stream = false;
    send_text(1U, "OK STREAM OFF\r\n");
  } else if (strcmp(line, "BOOTSEL") == 0) {
    /* Updating is a service action with ESC/BEC switched off. */
    if (vbec_measured && !rev6_hal_uv_good() && vbec_mv < 1000U) {
      boot_requested = true;
    } else {
      send_text(1U, "ERR switch ESC/BEC power off before BOOTSEL\r\n");
    }
  } else if (strcmp(line, "LED") == 0) {
    send_led_status();
  } else if (strcmp(line, "INFO") == 0 || strcmp(line, "HELP") == 0) {
    send_text(1U, "JPBB Rev6 Pico v0.2.0-led; PWM=100Hz; CDC0=JPB1; CDC1=JPX1; "
        "SBUS=100000/8E2; IMU=DISABLED; RPM=raw edges/period; "
        "INFO | LED | STREAM ON | STREAM OFF | BOOTSEL (BEC off)\r\n");
  } else if (line[0] != '\0') {
    send_text(1U, "ERR use INFO, LED, STREAM ON/OFF or BOOTSEL\r\n");
  }
}

static void control_line(const char *line) {
  jpbb_command_t command;
  if (!jpbb_protocol_parse(line, &command)) {
    ++rx_rejected[0];
    if (!jpb1_active) { send_text(0U, "ERR invalid command\r\n"); }
    return;
  }
  const uint32_t state = save_and_disable_interrupts();
  /* Re-evaluate fault/USB/timeout conditions before accepting a newly arrived
   * command; otherwise an overdue old arm could survive until the next IRQ. */
  rev6_core_tick(&core, millis(), rev6_hal_uv_good(), usb_healthy);
  const bool accepted = rev6_core_command(&core, &command, millis());
  restore_interrupts(state);
  if (command.kind == JPBB_COMMAND_HOST_FRAME) {
    jpb1_active = true;
    return;
  }
  if (command.kind == JPBB_COMMAND_STATUS) {
    char response[256];
    const size_t count = jpbb_protocol_status(response, sizeof(response));
    send_bytes(0U, response, count);
  } else if (command.kind == JPBB_COMMAND_HELP) {
    char response[192];
    send_bytes(0U, response, jpbb_protocol_help(response, sizeof(response)));
  } else if (command.kind == JPBB_COMMAND_PING || command.kind == JPBB_COMMAND_HEARTBEAT) {
    send_text(0U, "PONG\r\n");
  } else if (command.kind != JPBB_COMMAND_STREAM && command.kind != JPBB_COMMAND_NONE) {
    send_text(0U, accepted ? "OK\r\n" : "ERR refused: check power, RC permission and neutral command\r\n");
  }
}

static void poll_lines(uint8_t port) {
  for (unsigned i = 0U; i < REV6_RX_BUDGET && tud_cdc_n_available(port); ++i) {
    const int byte = tud_cdc_n_read_char(port);
    if (byte < 0) { break; }
    switch (rev6_line_feed(&lines[port], (uint8_t)byte)) {
      case REV6_LINE_READY:
        if (port == 0U) { control_line(lines[port].text); }
        else { diagnostic_line(lines[port].text); }
        break;
      case REV6_LINE_DISCARDED:
        ++rx_rejected[port];
        break;
      default:
        break;
    }
  }
}

int main(void) {
  rev6_hal_init();
  rev6_core_init(&core);
  rev6_led_init(&led);
  rev6_sbus_init(&decoder);
  const unsigned timer_irq = hardware_alarm_get_irq_num(
      alarm_pool_timer_alarm_num(alarm_pool_get_default()));
  irq_set_priority(timer_irq, REV6_INPUT_IRQ_PRIORITY);
  rev6_hal_inputs_start(sbus_byte, sbus_error, rpm_edge);
  watchdog_enable(REV6_WATCHDOG_MS, false);
  if (!add_repeating_timer_us(-1000, control_tick, NULL, &control_timer)) {
    /* No control timer: leave no pulses and let the watchdog restart. */
    for (;;) { tight_loop_contents(); }
  }

  bool usb_initialized = false;
  bool previous_vbus = false;
  bool previous_ready = false;
  uint32_t last_adc = 0U, last_status = 0U, last_sensors = 0U;
  uint32_t status_sequence = 0U, sensor_sequence = 0U;
  uint32_t handled_usb_epoch = 0U;
  for (;;) {
    const uint32_t now = millis();
    const bool vbus = rev6_hal_vbus();
    if (vbus && !usb_initialized) {
      tusb_init();
      irq_set_priority(USBCTRL_IRQ, REV6_USB_IRQ_PRIORITY);
      usb_initialized = true;
    }
    if (usb_initialized) {
      if (vbus != previous_vbus) {
        if (vbus) { tud_connect(); }
        else { tud_disconnect(); }
      }
      tud_task();
    }
    previous_vbus = vbus;
    const bool ready = vbus && usb_initialized && tud_mounted() && !tud_suspended();
    const uint32_t epoch = usb_fault_epoch;
    const bool usb_interrupted = epoch != handled_usb_epoch;
    if (usb_interrupted) {
      const uint32_t state = save_and_disable_interrupts();
      rev6_core_tick(&core, millis(), rev6_hal_uv_good(), false);
      restore_interrupts(state);
    }
    if (!ready || usb_interrupted) {
      jpb1_active = false;
      sensor_stream = false;
      rev6_line_reset(&lines[0]);
      rev6_line_reset(&lines[1]);
      if (usb_initialized && (previous_ready || usb_interrupted)) {
        tud_cdc_n_write_clear(0U); tud_cdc_n_write_clear(1U);
        tud_cdc_n_read_flush(0U); tud_cdc_n_read_flush(1U);
      }
    }
    /* An event arriving while queues are cleared must be handled next time;
     * do not overwrite its unhealthy latch with a sampled earlier ready=true. */
    const uint32_t state = save_and_disable_interrupts();
    handled_usb_epoch = epoch;
    usb_healthy = ready && usb_fault_epoch == epoch;
    restore_interrupts(state);
    previous_ready = ready;
    if (now - last_adc >= 10U) {
      last_adc = now;
      vbec_mv = rev6_hal_vbec_mv();
      vbec_measured = true;
    }
    if (ready) {
      poll_lines(0U);
      poll_lines(1U);
      if (jpb1_active && now - last_status >= REV6_STATUS_PERIOD_MS) {
        last_status = now;
        send_status(status_sequence++);
      }
      if (sensor_stream && now - last_sensors >= REV6_SENSOR_PERIOD_MS) {
        last_sensors = now;
        send_sensors(sensor_sequence++);
      }
      tud_cdc_n_write_flush(0U); tud_cdc_n_write_flush(1U);
    }
    if (boot_requested) {
      rev6_hal_pwm(1500U, 1500U, false);
      cancel_repeating_timer(&control_timer);
      reset_usb_boot(0U, 0U);
    }
    tight_loop_contents();
  }
}
