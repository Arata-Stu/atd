#include "rev6_transport.h"
#include "jpbb_protocol.h"
#include "jpbb_board.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

void jpbb_status_snapshot(jpbb_status_t *status) { memset(status, 0, sizeof(*status)); }
void jpbb_pwm_get(uint16_t *servo, uint16_t *esc) { *servo = *esc = 1500U; }

int main(void) {
  rev6_line_t line = {0};
  const char *command = "PING\r\n";
  unsigned ready = 0U;
  for (const char *c = command; *c; ++c) {
    if (rev6_line_feed(&line, (uint8_t)*c) == REV6_LINE_READY) {
      assert(strcmp(line.text, "PING") == 0);
      ++ready;
    }
  }
  assert(ready == 1U);
  for (unsigned i = 0U; i < 100U; ++i) { rev6_line_feed(&line, 'X'); }
  assert(rev6_line_feed(&line, '\n') == REV6_LINE_DISCARDED);
  rev6_line_feed(&line, 'A'); rev6_line_feed(&line, 0U);
  assert(rev6_line_feed(&line, '\n') == REV6_LINE_DISCARDED);
  for (size_t i = 0U; i < REV6_LINE_CAPACITY - 1U; ++i) { rev6_line_feed(&line, 'Q'); }
  assert(rev6_line_feed(&line, '\n') == REV6_LINE_READY);
  assert(strlen(line.text) == REV6_LINE_CAPACITY - 1U);
  assert(rev6_line_feed(&line, '\n') == REV6_LINE_WAIT);
  char packet[96] = "JPB1,C,1,0,0,0,0,3";
  assert(rev6_frame_finish(packet, sizeof(packet)) > 0U);
  jpbb_command_t parsed;
  assert(jpbb_protocol_parse(packet, &parsed));
  assert(parsed.sequence == 1U && parsed.flags == 3U);
  char too_small[8] = "HELLO";
  assert(rev6_frame_finish(too_small, sizeof(too_small)) == 0U);

  rev6_rpm_t rpm = {0};
  assert(!rev6_rpm_period_recent(&rpm, 0U));
  rev6_rpm_edge(&rpm, 0xfffffff0U);
  assert(!rpm.have_period);
  rev6_rpm_edge(&rpm, 0x10U);
  assert(rpm.edges == 2U && rpm.period_us == 32U);
  assert(rev6_rpm_period_recent(&rpm, 0x20U));
  assert(!rev6_rpm_period_recent(&rpm, 1000017U));
  rev6_rpm_expire(&rpm, 1000017U);
  assert(!rev6_rpm_period_recent(&rpm, 0x20U)); /* still invalid after clock wrap */
  rev6_rpm_edge(&rpm, 0x10U);
  assert(!rpm.have_period); /* simultaneous/noise edge isn't a valid period */
  rpm.edges = UINT32_MAX;
  rev6_rpm_edge(&rpm, 0x20U);
  assert(rpm.edges == 0U);
  puts("Pico transport/RPM tests: PASS");
  return 0;
}
