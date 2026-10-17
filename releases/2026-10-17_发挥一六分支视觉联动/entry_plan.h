#ifndef TASK4_RUNTIME_ENTRY_PLAN_H
#define TASK4_RUNTIME_ENTRY_PLAN_H
#include "task4_plan.h"
#include "task4_config.h"
extern Task4Plan parking_plan;
/* Runtime selections for the already field-tested common controller. */
#define ENTRY_NAME "TASK4_SIX_BRANCH_VISION"
#define ENTRY_EDGE (parking_plan.edge)
#define ENTRY_REQUIRED_CORNERS (parking_plan.corners)
#define ENTRY_PASS_LINES (2U*(parking_plan.corners-1U))
#define ENTRY_LINE_CENTER (parking_plan.first_line_center)
#define ENTRY_LINE_WINDOW TASK4_LINE_WINDOW
#define ENTRY_STEP_COUNT (parking_plan.step_count)
#define ENTRY_WAIT_MS (parking_plan.garage_pause_ms)
#define reference_steps (parking_plan.steps)
#endif
