#ifndef REV6_TRANSPORT_H
#define REV6_TRANSPORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define REV6_LINE_CAPACITY 96U
typedef struct {
  char text[REV6_LINE_CAPACITY];
  size_t length;
  bool dropping;
} rev6_line_t;

typedef enum { REV6_LINE_WAIT, REV6_LINE_READY, REV6_LINE_DISCARDED } rev6_line_result_t;

void rev6_line_reset(rev6_line_t *line);
/* READY text remains available until the next byte. No truncated prefix is
 * ever accepted; invalid/overlong lines are discarded through the delimiter. */
rev6_line_result_t rev6_line_feed(rev6_line_t *line, uint8_t byte);
/* Append the legacy CRC and CRLF to a NUL-terminated payload. */
size_t rev6_frame_finish(char *buffer, size_t capacity);

typedef struct {
  uint32_t edges;
  uint32_t last_edge_us;
  uint32_t period_us;
  bool seen_edge;
  bool have_period;
} rev6_rpm_t;

void rev6_rpm_edge(rev6_rpm_t *rpm, uint32_t now_us);
/* Absence of recent edges is not a sensor-disconnect diagnosis. */
bool rev6_rpm_period_recent(const rev6_rpm_t *rpm, uint32_t now_us);
/* Called periodically so an old period cannot become recent again after a
 * complete 32-bit microsecond-clock wrap with no further sensor edges. */
void rev6_rpm_expire(rev6_rpm_t *rpm, uint32_t now_us);

#endif
