#include "rev6_transport.h"
#include "jpbb_protocol.h"

#include <stdio.h>
#include <string.h>

void rev6_line_reset(rev6_line_t *line) {
  memset(line, 0, sizeof(*line));
}

rev6_line_result_t rev6_line_feed(rev6_line_t *line, uint8_t byte) {
  if (byte == '\n' || byte == '\r') {
    const bool dropping = line->dropping;
    const size_t length = line->length;
    line->text[length] = '\0';
    line->length = 0U;
    line->dropping = false;
    return dropping ? REV6_LINE_DISCARDED : (length ? REV6_LINE_READY : REV6_LINE_WAIT);
  }
  if (line->dropping) {
    return REV6_LINE_WAIT;
  }
  /* NUL and non-ASCII controls must not turn a longer packet into a valid
   * command prefix when handed to the C-string parser. */
  if ((byte < 32U && byte != '\t') || byte > 126U ||
      line->length + 1U >= sizeof(line->text)) {
    line->dropping = true;
    line->length = 0U;
    return REV6_LINE_WAIT;
  }
  line->text[line->length++] = (char)byte;
  return REV6_LINE_WAIT;
}

size_t rev6_frame_finish(char *buffer, size_t capacity) {
  const size_t length = strlen(buffer);
  if (length + 8U > capacity) {
    return 0U;
  }
  const uint16_t crc = jpbb_protocol_crc16(buffer, length);
  const int written = snprintf(buffer + length, capacity - length, ",%04X\r\n", crc);
  return written == 7 ? length + 7U : 0U;
}

void rev6_rpm_edge(rev6_rpm_t *rpm, uint32_t now_us) {
  ++rpm->edges;
  if (rpm->seen_edge) {
    rpm->period_us = now_us - rpm->last_edge_us;
    rpm->have_period = rpm->period_us != 0U && rpm->period_us <= 0x7fffffffU;
  }
  rpm->last_edge_us = now_us;
  rpm->seen_edge = true;
}

bool rev6_rpm_period_recent(const rev6_rpm_t *rpm, uint32_t now_us) {
  if (!rpm->have_period) {
    return false;
  }
  uint32_t timeout = rpm->period_us <= 0x7fffffffU / 3U
                         ? rpm->period_us * 3U : 0x7fffffffU;
  if (timeout < 1000000U) {
    timeout = 1000000U;
  }
  return now_us - rpm->last_edge_us <= timeout;
}

void rev6_rpm_expire(rev6_rpm_t *rpm, uint32_t now_us) {
  if (!rev6_rpm_period_recent(rpm, now_us)) {
    rpm->have_period = false;
  }
}
