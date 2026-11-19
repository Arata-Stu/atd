#include "rev6_hal.h"

#include <stddef.h>

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/regs/io_bank0.h"
#include "hardware/regs/uart.h"
#include "hardware/structs/io_bank0.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "pico/time.h"

#define REV6_SBUS_UART uart1
#define REV6_SBUS_IRQ UART1_IRQ
#define REV6_UART_ISR_MAX_BYTES 64u

#define REV6_UART_DATA_ERRORS (UART_UARTDR_FE_BITS | UART_UARTDR_PE_BITS | \
                               UART_UARTDR_BE_BITS | UART_UARTDR_OE_BITS)
#define REV6_UART_STATUS_ERRORS (UART_UARTRSR_FE_BITS | UART_UARTRSR_PE_BITS | \
                                 UART_UARTRSR_BE_BITS | UART_UARTRSR_OE_BITS)
#define REV6_UART_ERROR_MASK (UART_UARTIMSC_FEIM_BITS | UART_UARTIMSC_PEIM_BITS | \
                              UART_UARTIMSC_BEIM_BITS | UART_UARTIMSC_OEIM_BITS)
#define REV6_UART_ERROR_CLEAR (UART_UARTICR_FEIC_BITS | UART_UARTICR_PEIC_BITS | \
                               UART_UARTICR_BEIC_BITS | UART_UARTICR_OEIC_BITS)

_Static_assert(REV6_GPIO_UV_GOOD == 9, "Review raw UV-fault latch mapping");

static unsigned int output_slice;
static void (*on_sbus_byte)(uint8_t, uint32_t);
static void (*on_sbus_error)(void);
static void (*on_rpm_edge)(uint32_t);

static void input_without_pulls(unsigned int pin) {
    gpio_init(pin);
    gpio_disable_pulls(pin);
    gpio_set_dir(pin, GPIO_IN);
}

static void parked_output(unsigned int pin, bool value) {
    gpio_init(pin);
    gpio_put(pin, value);
    gpio_set_dir(pin, GPIO_OUT);
}

static void sbus_uart_irq(void) {
    uart_hw_t *hw = uart_get_hw(REV6_SBUS_UART);

    /* RSR also catches an overrun even when no further byte arrives. Reset the
     * decoder before accepting any bytes after a UART error. DR error bits
     * below identify and discard the affected FIFO byte independently.
     */
    uint32_t errors = hw->rsr & REV6_UART_STATUS_ERRORS;
    hw->rsr = 0;
    hw->icr = REV6_UART_ERROR_CLEAR;
    if (errors != 0 && on_sbus_error != NULL) {
        on_sbus_error();
    }

    for (unsigned int n = 0; n < REV6_UART_ISR_MAX_BYTES; ++n) {
        if (!uart_is_readable(REV6_SBUS_UART)) {
            break;
        }
        uint32_t data = hw->dr;
        if ((data & REV6_UART_DATA_ERRORS) != 0) {
            if (on_sbus_error != NULL) {
                on_sbus_error();
            }
            continue;
        }
        if (on_sbus_byte != NULL) {
            on_sbus_byte((uint8_t)data, time_us_32());
        }
    }
}

static void rpm_gpio_irq(unsigned int gpio, uint32_t events) {
    if (gpio == REV6_GPIO_RPM && (events & GPIO_IRQ_EDGE_FALL) != 0 &&
        on_rpm_edge != NULL) {
        /* The board's NPN stage inverts the external RPM signal. */
        uint32_t now_us = time_us_32();
        /* A noisy optional sensor must not outrank SBUS or the control timer.
         * Serialize only the short RPM state update with timer-side expiry.
         */
        uint32_t saved_interrupts = save_and_disable_interrupts();
        on_rpm_edge(now_us);
        restore_interrupts(saved_interrupts);
    }
}

void rev6_hal_init(void) {
    /* A High MCU output turns on Q7/Q8 and holds the external connector Low.
     * Latch High before enabling the SIO drivers, then hand over to inverted
     * PWM with compare=0. External R29/R30 also bias reset/Hi-Z pins High.
     */
    parked_output(REV6_GPIO_SERVO, true);
    parked_output(REV6_GPIO_ESC, true);
    gpio_set_drive_strength(REV6_GPIO_SERVO, GPIO_DRIVE_STRENGTH_4MA);
    gpio_set_drive_strength(REV6_GPIO_ESC, GPIO_DRIVE_STRENGTH_4MA);

    output_slice = pwm_gpio_to_slice_num(REV6_GPIO_SERVO);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, (float)clock_get_hz(clk_sys) / 1000000.0f);
    pwm_config_set_wrap(&config, REV6_PWM_PERIOD_US - 1u);
    pwm_config_set_output_polarity(&config, true, true);
    pwm_init(output_slice, &config, false);
    pwm_set_both_levels(output_slice, 0, 0);
    pwm_set_enabled(output_slice, true);
    gpio_set_function(REV6_GPIO_SERVO, GPIO_FUNC_PWM);
    gpio_set_function(REV6_GPIO_ESC, GPIO_FUNC_PWM);

    /* Physical U1 pad 12 is GP9. GP8 / pad 11 is not connected on Rev6. */
    input_without_pulls(REV6_GPIO_UV_GOOD);
    /* INTR edge latches exist independently of per-core interrupt enables.
     * Keep this GPIO masked so the SDK's GPIO callback cannot acknowledge it
     * before the control timer consumes it. Clear pre-initialization history.
     */
    gpio_set_irq_enabled(REV6_GPIO_UV_GOOD,
                         GPIO_IRQ_LEVEL_LOW | GPIO_IRQ_LEVEL_HIGH |
                         GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, false);
    gpio_acknowledge_irq(REV6_GPIO_UV_GOOD, GPIO_IRQ_EDGE_FALL);
    input_without_pulls(REV6_GPIO_VBUS);
    parked_output(REV6_GPIO_LED, false);

    /* SPI1-compatible wiring is reserved, but no sensor is assumed present.
     * Keep CS inactive and leave SPI disabled until a module is selected.
     */
    parked_output(REV6_GPIO_IMU_CS, true);
    parked_output(REV6_GPIO_IMU_SCK, false);
    parked_output(REV6_GPIO_IMU_MOSI, false);
    input_without_pulls(REV6_GPIO_IMU_MISO);
    input_without_pulls(REV6_GPIO_IMU_INT);
    gpio_pull_down(REV6_GPIO_IMU_INT);
    gpio_set_irq_enabled(REV6_GPIO_IMU_INT,
                         GPIO_IRQ_LEVEL_LOW | GPIO_IRQ_LEVEL_HIGH |
                         GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, false);
    input_without_pulls(REV6_GPIO_SBUS);
    input_without_pulls(REV6_GPIO_RPM);

    adc_init();
    adc_gpio_init(REV6_GPIO_VBEC_ADC);
    adc_select_input(REV6_VBEC_ADC_CHANNEL);
}

bool rev6_hal_uv_good(void) {
    return gpio_get(REV6_GPIO_UV_GOOD);
}

bool rev6_hal_take_uv_fault(void) {
    /* RP2040 Datasheet 2.19.3: edge events are stored in raw INTR and cleared
     * by writing INTR, before PROC0_INTE/INTS masking. GP9 is INTR1 bit 6.
     * gpio_get_irq_event_mask() reads masked INTS and cannot be used here.
     */
    bool pending = (io_bank0_hw->intr[1] & IO_BANK0_INTR1_GPIO9_EDGE_LOW_BITS) != 0;
    if (pending) {
        gpio_acknowledge_irq(REV6_GPIO_UV_GOOD, GPIO_IRQ_EDGE_FALL);
    }
    /* Do not acknowledge an empty latch: an edge arriving after the read
     * must remain pending for the next control tick.
     */
    return pending;
}

bool rev6_hal_vbus(void) {
    return gpio_get(REV6_GPIO_VBUS);
}

void rev6_hal_pwm(uint16_t servo_us, uint16_t esc_us, bool enabled) {
    uint16_t servo_level = 0;
    uint16_t esc_level = 0;
    if (enabled) {
        /* Fail closed on impossible pulse lengths instead of a constant High
         * connector level. Vehicle-specific limits belong to the controller.
         */
        if (servo_us < REV6_PWM_PERIOD_US) {
            servo_level = servo_us;
        }
        if (esc_us < REV6_PWM_PERIOD_US) {
            esc_level = esc_us;
        }
    }
    pwm_set_both_levels(output_slice, servo_level, esc_level);
}

uint16_t rev6_hal_vbec_mv(void) {
    adc_select_input(REV6_VBEC_ADC_CHANNEL);
    uint32_t raw = adc_read();
    /* R27=100k, R28=47k. 12-bit ADC code / 4096, nominal VREF=3300mV.
     * The 100nF filter has a ~3.2ms time constant; this is diagnostic only.
     */
    const uint32_t denominator = 4096u * 47u;
    return (uint16_t)((raw * 3300u * 147u + denominator / 2u) / denominator);
}

void rev6_hal_led(bool on) {
    gpio_put(REV6_GPIO_LED, on);
}

void rev6_hal_inputs_start(void (*sbus_byte)(uint8_t, uint32_t),
                           void (*sbus_error)(void),
                           void (*rpm_edge)(uint32_t)) {
    uint32_t saved_interrupts = save_and_disable_interrupts();
    on_sbus_byte = sbus_byte;
    on_sbus_error = sbus_error;
    on_rpm_edge = rpm_edge;

    irq_set_enabled(REV6_SBUS_IRQ, false);
    uart_init(REV6_SBUS_UART, REV6_SBUS_BAUD);
    uart_set_hw_flow(REV6_SBUS_UART, false, false);
    uart_set_format(REV6_SBUS_UART, 8, 2, UART_PARITY_EVEN);
    uart_set_fifo_enabled(REV6_SBUS_UART, true);
    gpio_set_function(REV6_GPIO_SBUS, GPIO_FUNC_UART);
    /* Q3 already converts inverted SBUS to ordinary UART idle-High. */
    gpio_set_inover(REV6_GPIO_SBUS, GPIO_OVERRIDE_NORMAL);

    irq_set_exclusive_handler(REV6_SBUS_IRQ, sbus_uart_irq);
    irq_set_priority(REV6_SBUS_IRQ, REV6_INPUT_IRQ_PRIORITY);
    uart_get_hw(REV6_SBUS_UART)->rsr = 0;
    uart_get_hw(REV6_SBUS_UART)->icr = REV6_UART_ERROR_CLEAR;
    uart_set_irqs_enabled(REV6_SBUS_UART, true, false);
    uart_get_hw(REV6_SBUS_UART)->imsc |= REV6_UART_ERROR_MASK;
    irq_set_enabled(REV6_SBUS_IRQ, true);

    irq_set_priority(IO_IRQ_BANK0, REV6_RPM_IRQ_PRIORITY);
    gpio_acknowledge_irq(REV6_GPIO_RPM, GPIO_IRQ_EDGE_FALL);
    gpio_set_irq_enabled_with_callback(REV6_GPIO_RPM, GPIO_IRQ_EDGE_FALL,
                                       rpm_edge != NULL, rpm_gpio_irq);
    restore_interrupts(saved_interrupts);
}
