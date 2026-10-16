#ifndef TASK4_ENTRY_PLAN_H
#define TASK4_ENTRY_PLAN_H
#include <stdint.h>
#ifndef TASK4_CASE
#error Select TASK4_CASE 1=BC reverse, 2=CD side, 3=DA reverse
#endif
typedef struct { char direction; int32_t target; } ReferenceStep;
/* All distances below are encoder counts, all angles are 0.01 degree. */
#if TASK4_CASE == 1
#define ENTRY_NAME "TASK4_BC_REVERSE_UNTESTED"
#define ENTRY_START_FRAME "@TASK4:BC:RUN"
#define ENTRY_EDGE "BC"
#define ENTRY_REQUIRED_CORNERS 1U
#define ENTRY_LINE_CENTER 7550L
#define ENTRY_STEP_COUNT 3U
#define ENTRY_WAIT_MS 200U
#elif TASK4_CASE == 2
#define ENTRY_NAME "TASK4_CD_SIDE_UNTESTED"
#define ENTRY_START_FRAME "@TASK4:CD:RUN"
#define ENTRY_EDGE "CD"
#define ENTRY_REQUIRED_CORNERS 2U
#define ENTRY_LINE_CENTER 13252L
#define ENTRY_STEP_COUNT 5U
#define ENTRY_WAIT_MS 1200U
#elif TASK4_CASE == 3
#define ENTRY_NAME "TASK4_DA_REVERSE_UNTESTED"
#define ENTRY_START_FRAME "@TASK4:DA:RUN"
#define ENTRY_EDGE "DA"
#define ENTRY_REQUIRED_CORNERS 3U
#define ENTRY_LINE_CENTER 19673L
#define ENTRY_STEP_COUNT 3U
#define ENTRY_WAIT_MS 200U
#else
#error Invalid TASK4_CASE
#endif
#define ENTRY_LINE_WINDOW 500L
#define ENTRY_PASS_LINES (2U*(ENTRY_REQUIRED_CORNERS-1U))
extern const ReferenceStep reference_steps[ENTRY_STEP_COUNT];
int32_t reference_yaw_delta(uint16_t start,uint16_t current);
int32_t reference_encoder_progress(int16_t a,int16_t b,int16_t c,int8_t direction);
#endif
