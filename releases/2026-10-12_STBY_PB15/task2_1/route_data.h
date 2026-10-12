#ifndef TASK2_1_ROUTE_DATA_H
#define TASK2_1_ROUTE_DATA_H

#include <stdint.h>

typedef struct {
  char direction;
  uint16_t value;
  uint8_t source_step;
} RouteStep;

/* Verified reverse-entry sequence from the BC garage-line stop.
 * F/B values are millimetres; L/R values are degrees.
 * No extra 100 mm offset and no exit steps.
 */
static const RouteStep route_steps[] = {
  {'F', 209U, 1U},
  {'L',  90U, 2U},
  {'B', 200U, 3U}
};

#define ROUTE_STEP_COUNT (sizeof(route_steps) / sizeof(route_steps[0]))

#endif
