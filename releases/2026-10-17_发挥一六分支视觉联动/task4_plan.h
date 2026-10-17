#ifndef TASK4_PLAN_H
#define TASK4_PLAN_H
#include <stdint.h>
#define TASK4_MAX_STEPS 5U
typedef struct {char direction;int32_t target;} ReferenceStep;
typedef struct {
  char edge[3];
  uint8_t corners,reverse,branch_id,step_count,profile_source;
  uint16_t garage_pause_ms;
  int32_t first_line_center;
  ReferenceStep steps[TASK4_MAX_STEPS];
} Task4Plan;
uint8_t task4_make_plan(const char *color,const char *shape,const char *action,Task4Plan *out);
uint8_t task4_plan_valid(const Task4Plan *plan);
int32_t task4_line_position(uint8_t edge,uint8_t reverse,uint8_t boundary);
int32_t reference_yaw_delta(uint16_t start,uint16_t current);
int32_t reference_encoder_progress(int16_t a,int16_t b,int16_t c,int8_t direction);
#endif
