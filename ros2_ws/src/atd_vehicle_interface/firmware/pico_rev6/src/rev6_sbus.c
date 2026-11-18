#include "rev6_sbus.h"

#include <string.h>

static bool valid_footer(uint8_t byte) {
  return byte == 0x00U || byte == 0x04U || byte == 0x14U ||
         byte == 0x24U || byte == 0x34U;
}

void rev6_sbus_init(rev6_sbus_decoder_t *decoder) {
  if (decoder != NULL) {
    memset(decoder, 0, sizeof(*decoder));
  }
}

bool rev6_sbus_decode(const uint8_t bytes[REV6_SBUS_FRAME_BYTES],
                      rev6_sbus_frame_t *frame) {
  if (bytes == NULL || frame == NULL || bytes[0] != 0x0FU ||
      (bytes[23] & 0xF0U) != 0U || !valid_footer(bytes[24])) {
    return false;
  }
  rev6_sbus_frame_t decoded = {0};
  for (size_t channel = 0U; channel < REV6_SBUS_CHANNELS; ++channel) {
    const size_t bit = channel * 11U;
    const size_t index = 1U + bit / 8U;
    /* Reading the flags byte on the final channel is harmless: the mask only
     * includes the 11 channel bits, all of which are in bytes 1..22. */
    const uint32_t word = (uint32_t)bytes[index] |
                          ((uint32_t)bytes[index + 1U] << 8U) |
                          ((uint32_t)bytes[index + 2U] << 16U);
    decoded.channels[channel] = (uint16_t)((word >> (bit % 8U)) & 0x07FFU);
  }
  decoded.digital17 = (bytes[23] & 0x01U) != 0U;
  decoded.digital18 = (bytes[23] & 0x02U) != 0U;
  decoded.frame_lost = (bytes[23] & 0x04U) != 0U;
  decoded.failsafe = (bytes[23] & 0x08U) != 0U;
  decoded.footer = bytes[24];
  *frame = decoded;
  return true;
}

bool rev6_sbus_feed(rev6_sbus_decoder_t *decoder, uint8_t byte,
                    rev6_sbus_frame_t *frame) {
  if (decoder == NULL || frame == NULL) {
    return false;
  }
  if (decoder->count == 0U && byte != 0x0FU) {
    return false;
  }
  decoder->bytes[decoder->count++] = byte;
  if (decoder->count < REV6_SBUS_FRAME_BYTES) {
    return false;
  }
  if (rev6_sbus_decode(decoder->bytes, frame)) {
    decoder->count = 0U;
    return true;
  }

  /* Keep a possible embedded header after a corrupt/truncated frame. Payload
   * may also contain 0x0F, so only the full structural test accepts a frame. */
  for (size_t index = 1U; index < REV6_SBUS_FRAME_BYTES; ++index) {
    if (decoder->bytes[index] == 0x0FU) {
      decoder->count = REV6_SBUS_FRAME_BYTES - index;
      memmove(decoder->bytes, decoder->bytes + index, decoder->count);
      return false;
    }
  }
  decoder->count = 0U;
  return false;
}
