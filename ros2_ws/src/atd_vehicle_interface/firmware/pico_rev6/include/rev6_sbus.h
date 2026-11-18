#ifndef REV6_SBUS_H
#define REV6_SBUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define REV6_SBUS_FRAME_BYTES 25U
#define REV6_SBUS_CHANNELS 16U

/* Bytes must already be electrically inverted by the board / UART setup. */
typedef struct {
  uint16_t channels[REV6_SBUS_CHANNELS];
  bool digital17;
  bool digital18;
  bool frame_lost;
  bool failsafe;
  uint8_t footer;
} rev6_sbus_frame_t;

typedef struct {
  uint8_t bytes[REV6_SBUS_FRAME_BYTES];
  size_t count;
} rev6_sbus_decoder_t;

void rev6_sbus_init(rev6_sbus_decoder_t *decoder);
/* A true return means structurally valid, including frames with fault flags. */
bool rev6_sbus_decode(const uint8_t bytes[REV6_SBUS_FRAME_BYTES],
                      rev6_sbus_frame_t *frame);
/* Reset with init() on a UART framing/parity/overrun error or an inter-frame gap. */
bool rev6_sbus_feed(rev6_sbus_decoder_t *decoder, uint8_t byte,
                    rev6_sbus_frame_t *frame);

#endif
