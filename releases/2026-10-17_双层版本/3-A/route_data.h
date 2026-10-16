#ifndef GARAGE_TO_A_ROUTE_H
#define GARAGE_TO_A_ROUTE_H
#include <stdint.h>
typedef struct { char direction; uint16_t value; uint8_t source_step; } RouteStep;
/* Start at the already reached all-black FF garage gate. */
static const RouteStep route_steps[] = {
  {'F', 100U, 5U},
  {'R',  50U, 6U},
  {'F', 180U, 7U},
  {'R',  60U, 8U},
  {'F', 200U, 9U}
};
#define ROUTE_STEP_COUNT (sizeof(route_steps) / sizeof(route_steps[0]))
#endif
