#include "reference_steps.h"
#include <string.h>

const ReferenceStep reference_steps[5] = {
  {'L', 546,  "@REF1:L0546"},
  {'F', 2194, "@REF2:F2194"},
  {'L', 4193, "@REF3:L4193"},
  /* Original 1146 counts + extra 35 mm at 5.73 counts/mm (201 counts). */
  {'B', 1347, "@REF4:B1347"},
  {'R', 4474, "@REF5:R4474"}
};

/* Exact full-line matching: corrupt bytes cannot change the target value.
 * The caller advances expected as soon as a step starts, so echoes cannot
 * restart a step. Noise outside a frame is silently discarded.
 */
uint8_t reference_feed(ReferenceParser *p, uint8_t byte, uint8_t expected)
{
  if (byte == '@') { p->used = 0U; p->collecting = 1U; }
  if (!p->collecting) return 0U;
  if (byte == '\r') return 0U;
  if (byte == '\n') {
    p->line[p->used] = 0;
    p->collecting = 0U;
    if (expected < 5U && strcmp(p->line, reference_steps[expected].command) == 0) return 1U;
    if (expected == 0U && p->used == 11U && memcmp(p->line, "@BASE:", 6U) == 0) {
      uint32_t value = 0U;
      for (uint8_t i = 6U; i < 11U; ++i) {
        if (p->line[i] < '0' || p->line[i] > '9') return 0U;
        value = value * 10U + (uint8_t)(p->line[i] - '0');
      }
      if (value < 36000U) { p->anchor = (uint16_t)value; return 2U; }
    }
    return 0U;
  }
  if (byte < 32U || byte > 126U || p->used >= sizeof(p->line) - 1U) {
    p->collecting = p->used = 0U; return 0U;
  }
  p->line[p->used++] = (char)byte;
  return 0U;
}

int32_t reference_yaw_delta(uint16_t start, uint16_t current)
{
  int32_t delta = (int32_t)current - start;
  if (delta > 18000) delta -= 36000;
  if (delta < -18000) delta += 36000;
  return delta;
}

int32_t reference_encoder_progress(int16_t a, int16_t b, int16_t c, int8_t direction)
{
  /* F: A/B negative, C positive. Signed progress also permits a small trim. */
  int32_t v[3] = {-(int32_t)a, -(int32_t)b, c};
  for (uint8_t i = 0U; i < 3U; ++i)
    for (uint8_t j = i + 1U; j < 3U; ++j)
      if (v[j] < v[i]) { int32_t temp = v[i]; v[i] = v[j]; v[j] = temp; }
  return direction * v[1];
}
