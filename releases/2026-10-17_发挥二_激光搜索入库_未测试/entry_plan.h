#ifndef EXT2_ENTRY_PLAN_H
#define EXT2_ENTRY_PLAN_H
#include <stdint.h>
#include "search_logic.h"
typedef struct { char direction; int32_t target; } ReferenceStep;
extern ReferenceStep reference_steps[7];
extern uint8_t reference_count;
#define ENTRY_STEP_COUNT reference_count
void search_plan_reset(void);
uint8_t search_plan_return(int32_t forward_actual);
uint8_t search_plan_entry(GarageType type);
int32_t reference_yaw_delta(uint16_t start,uint16_t current);
int32_t reference_encoder_progress(int16_t a,int16_t b,int16_t c,int8_t direction);
#endif
