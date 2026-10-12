#ifndef SIDE_PARK_AUTO_ROUTE_DATA_H
#define SIDE_PARK_AUTO_ROUTE_DATA_H

#include <stdint.h>

/* Confirmed 2026-10-10 side-parking entry, starting at the garage-line stop.
 * F/B values are millimetres; L/R values are degrees.
 */
typedef struct {
  char direction;
  uint16_t value;
  uint8_t source_step;
} RouteStep;

static const RouteStep route_steps[] = {
  {'L',   7U, 1U},
  {'F', 370U, 2U},
  {'L',  38U, 3U},
  {'B', 190U, 4U},
  {'R',  42U, 5U}
};

#define ROUTE_STEP_COUNT (sizeof(route_steps) / sizeof(route_steps[0]))

#endif
