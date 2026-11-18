#include "rev6_sbus.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void pack(uint8_t bytes[25], const uint16_t channels[16], uint8_t flags,
                  uint8_t footer) {
  memset(bytes, 0, 25U);
  bytes[0] = 0x0FU;
  for (size_t channel = 0U; channel < 16U; ++channel) {
    for (size_t bit = 0U; bit < 11U; ++bit) {
      if ((channels[channel] & (1U << bit)) != 0U) {
        const size_t position = 11U * channel + bit;
        bytes[1U + position / 8U] |= (uint8_t)(1U << (position % 8U));
      }
    }
  }
  bytes[23] = flags;
  bytes[24] = footer;
}

static void decode_channels_flags_and_footers(void) {
  const uint16_t channels[16] = {0U, 1U, 2047U, 1024U, 0x555U, 0x2AAU,
                                  172U, 1811U, 992U, 17U, 255U, 256U,
                                  511U, 512U, 1023U, 1800U};
  const uint8_t footers[] = {0x00U, 0x04U, 0x14U, 0x24U, 0x34U};
  uint8_t bytes[25];
  rev6_sbus_frame_t frame = {0};
  for (size_t footer = 0U; footer < sizeof(footers); ++footer) {
    for (uint8_t flags = 0U; flags < 16U; ++flags) {
      pack(bytes, channels, flags, footers[footer]);
      assert(rev6_sbus_decode(bytes, &frame));
      for (size_t channel = 0U; channel < 16U; ++channel) {
        assert(frame.channels[channel] == channels[channel]);
      }
      assert(frame.digital17 == ((flags & 1U) != 0U));
      assert(frame.digital18 == ((flags & 2U) != 0U));
      assert(frame.frame_lost == ((flags & 4U) != 0U));
      assert(frame.failsafe == ((flags & 8U) != 0U));
      assert(frame.footer == footers[footer]);
    }
  }

  /* A literal wire packet exercises the packing boundary independently. */
  memset(bytes, 0xFF, sizeof(bytes));
  bytes[0] = 0x0FU;
  bytes[23] = 0U;
  bytes[24] = 0U;
  assert(rev6_sbus_decode(bytes, &frame));
  for (size_t channel = 0U; channel < 16U; ++channel) {
    assert(frame.channels[channel] == 2047U);
  }
}

static void reject_bad_structure_without_overwriting_output(void) {
  uint8_t bytes[25];
  const uint16_t channels[16] = {0};
  rev6_sbus_frame_t frame = {0};
  frame.channels[0] = 123U;
  frame.frame_lost = true;
  const rev6_sbus_frame_t before = frame;
  pack(bytes, channels, 0U, 0U);
  bytes[0] = 0xF0U; /* Not software-inverted again. */
  assert(!rev6_sbus_decode(bytes, &frame));
  assert(memcmp(&before, &frame, sizeof(frame)) == 0);
  bytes[0] = 0x0FU;
  bytes[23] = 0x10U;
  assert(!rev6_sbus_decode(bytes, &frame));
  bytes[23] = 0U;
  bytes[24] = 0x05U;
  assert(!rev6_sbus_decode(bytes, &frame));
  assert(!rev6_sbus_decode(NULL, &frame));
  assert(!rev6_sbus_decode(bytes, NULL));
}

static unsigned feed_bytes(rev6_sbus_decoder_t *decoder, const uint8_t *bytes,
                            size_t count, rev6_sbus_frame_t *frame) {
  unsigned received = 0U;
  for (size_t index = 0U; index < count; ++index) {
    if (rev6_sbus_feed(decoder, bytes[index], frame)) {
      ++received;
    }
  }
  return received;
}

static void stream_noise_reset_and_resynchronization(void) {
  const uint16_t channels[16] = {172U, 1811U, 992U};
  uint8_t good[25];
  pack(good, channels, 0U, 0U);
  rev6_sbus_frame_t frame = {0};
  rev6_sbus_decoder_t decoder;
  rev6_sbus_init(&decoder);
  const uint8_t noise[] = {0U, 0xFFU, 0x80U, 0xF0U};
  assert(feed_bytes(&decoder, noise, sizeof(noise), &frame) == 0U);
  assert(feed_bytes(&decoder, good, 24U, &frame) == 0U);
  assert(rev6_sbus_feed(&decoder, good[24], &frame));
  assert(frame.channels[0] == 172U && frame.channels[1] == 1811U);
  assert(feed_bytes(&decoder, good, 25U, &frame) == 1U);

  /* Truncation followed by a full frame recovers its header, even when the
   * initial corrupt 25-byte window consumes part of the good frame. */
  rev6_sbus_init(&decoder);
  const uint8_t truncated[] = {0x0FU, 0x80U, 0x80U, 0x80U, 0x80U, 0x80U};
  assert(feed_bytes(&decoder, truncated, sizeof(truncated), &frame) == 0U);
  assert(feed_bytes(&decoder, good, 25U, &frame) == 1U);

  uint8_t corrupt[25];
  memcpy(corrupt, good, sizeof(corrupt));
  corrupt[24] = 0xFFU;
  rev6_sbus_init(&decoder);
  assert(feed_bytes(&decoder, corrupt, sizeof(corrupt), &frame) == 0U);
  assert(feed_bytes(&decoder, good, sizeof(good), &frame) == 1U);

  /* The UART owner resets on a hardware error or a measured gap. */
  assert(feed_bytes(&decoder, good, 12U, &frame) == 0U);
  rev6_sbus_init(&decoder);
  assert(feed_bytes(&decoder, good, sizeof(good), &frame) == 1U);
  assert(!rev6_sbus_feed(NULL, 0x0FU, &frame));
  assert(!rev6_sbus_feed(&decoder, 0x0FU, NULL));
  rev6_sbus_init(NULL);
}

int main(void) {
  decode_channels_flags_and_footers();
  reject_bad_structure_without_overwriting_output();
  stream_noise_reset_and_resynchronization();
  puts("SBUS decoder tests passed");
  return 0;
}
