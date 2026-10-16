#ifndef C_TAKEOVER_H
#define C_TAKEOVER_H
#include <stdint.h>

/* Shared by destinations A/B/D only at the already-passed-C handoff.
 * A single continuous group of 1..3 black channels is a possible line.
 * Disconnected groups and wide black features must not steer the car. */
static uint8_t c_takeover_line(uint8_t bits)
{
  uint8_t count = 0U, group_started = 0U, group_ended = 0U;
  uint8_t channel;
  for (channel = 0U; channel < 8U; channel++) {
    if ((bits & (1U << channel)) != 0U) {
      if (group_ended) return 0U;
      group_started = 1U;
      count++;
    } else if (group_started) group_ended = 1U;
  }
  return count >= 1U && count <= 3U;
}

static uint8_t c_takeover_needs_align(uint8_t bits)
{
  return c_takeover_line(bits) && (bits & 0x18U) == 0U;
}
#endif
