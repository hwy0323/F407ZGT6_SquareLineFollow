#include "entry_plan.h"
#if TASK4_CASE == 2
/* Latest accepted loaded-car side-entry route, rotated to the CD edge. */
const ReferenceStep reference_steps[ENTRY_STEP_COUNT]={
  {'L',546},{'F',2194},{'L',4193},{'B',1347},{'R',4474}
};
#else
/* Original F209 -> L90 -> B200, converted at 5.73 counts/mm. */
const ReferenceStep reference_steps[ENTRY_STEP_COUNT]={
  {'F',1198},{'L',9000},{'B',1146}
};
#endif
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
