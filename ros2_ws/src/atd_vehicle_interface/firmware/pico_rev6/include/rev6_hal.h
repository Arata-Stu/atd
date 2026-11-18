#ifndef REV6_HAL_H
#define REV6_HAL_H

#include <stdbool.h>
#include <stdint.h>

/* GPIO numbers, not the Pico module's physical pad numbers. */
enum {
    REV6_GPIO_SBUS = 5,
    REV6_GPIO_SERVO = 6,
    REV6_GPIO_ESC = 7,
    REV6_GPIO_UV_GOOD = 9,
    REV6_GPIO_IMU_SCK = 10,
    REV6_GPIO_IMU_MOSI = 11,
    REV6_GPIO_IMU_MISO = 12,
    REV6_GPIO_IMU_CS = 13,
    REV6_GPIO_IMU_INT = 14,
    REV6_GPIO_RPM = 15,
    REV6_GPIO_VBUS = 24,
    REV6_GPIO_LED = 25,
    REV6_GPIO_VBEC_ADC = 26,
    REV6_VBEC_ADC_CHANNEL = 0,
    REV6_INPUT_IRQ_PRIORITY = 64,
    REV6_USB_IRQ_PRIORITY = 128,
    REV6_RPM_IRQ_PRIORITY = 192,
    REV6_PWM_HZ = 100,
    REV6_PWM_PERIOD_US = 10000,
    REV6_SBUS_BAUD = 100000
};

/* Initialize inactive PWM and park the not-yet-selected SPI IMU interface. */
void rev6_hal_init(void);
bool rev6_hal_uv_good(void);
/* Control-timer-only: consume the sticky raw GP9 falling-edge latch, including
 * a fault that recovered between ticks. GP9 CPU IRQ delivery stays disabled.
 */
bool rev6_hal_take_uv_fault(void);
bool rev6_hal_vbus(void);

/* Positive pulse widths at the connectors; disabled means no pulse (Low).
 * Both channels share the 100 Hz period and update at the next PWM wrap.
 * The control layer must validate vehicle-specific pulse limits.
 */
void rev6_hal_pwm(uint16_t servo_us, uint16_t esc_us, bool enabled);

/* Main-loop-only diagnostic; uses nominal 3.3 V ADC reference, not calibration. */
uint16_t rev6_hal_vbec_mv(void);

/* Call once on the control core after rev6_hal_init(). SBUS callbacks run at
 * priority 64; RPM runs at priority 192 with only its state update IRQ-masked.
 * Callbacks must be bounded and must not use USB or ADC.
 * SBUS timestamps are UART FIFO service times, not wire-edge timestamps.
 * sbus_error must reset any partially decoded frame. RPM is optional.
 */
void rev6_hal_inputs_start(void (*sbus_byte)(uint8_t, uint32_t),
                           void (*sbus_error)(void),
                           void (*rpm_edge)(uint32_t));

#endif
