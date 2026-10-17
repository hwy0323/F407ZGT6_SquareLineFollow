#include "task4_plan.h"
#include "task4_config.h"
#include <string.h>
static const ReferenceStep reverse_steps[3]={{'F',1198},{'L',9000},{'B',1146}};
static const ReferenceStep side_steps[5]={{'L',546},{'F',2194},{'L',4193},{'B',1347},{'R',4474}};
/* Motion profile provenance, not acceptance of this new integrated firmware:
 * 0=transferred/not field-tested, 1=local same-edge source, 2=user external test. */
static const uint8_t source_status[6]={1,0,0,1,2,1};
int32_t task4_line_position(uint8_t edge,uint8_t reverse,uint8_t boundary)
{
  if(edge<1U || edge>3U || reverse>1U || boundary>1U)return -1L;
  int32_t along=reverse?TASK4_REVERSE_ALONG_MM:TASK4_SIDE_ALONG_MM;
  int32_t half=(along*TASK4_COUNTS_PER_100_MM+50L)/100L/2L;
  return garage_midpoint_counts[edge-1U]+(boundary?half:-half);
}
uint8_t task4_plan_valid(const Task4Plan *plan)
{
  static const char edges[3][3]={"BC","CD","DA"};
  if(!plan || plan->corners<1U || plan->corners>3U || plan->reverse>1U)return 0U;
  uint8_t branch=(uint8_t)(2U*(plan->corners-1U)+(plan->reverse?0U:1U));
  if(plan->branch_id!=branch || memcmp(plan->edge,edges[plan->corners-1U],3U) ||
     plan->step_count!=(plan->reverse?3U:5U) ||
     plan->first_line_center!=task4_line_position(plan->corners,plan->reverse,0U)+first_line_trim[branch])return 0U;
  for(uint8_t i=0;i<plan->step_count;i++){
    const ReferenceStep *source=plan->reverse?&reverse_steps[i]:&side_steps[i];
    if(plan->steps[i].direction!=source->direction || plan->steps[i].target<=0 ||
       plan->steps[i].target!=source->target+motion_trim[branch][i])return 0U;
  }
  return 1U;
}
uint8_t task4_make_plan(const char *color,const char *shape,const char *action,Task4Plan *out)
{
  if(!out)return 0U;
  memset(out,0,sizeof(*out));
  if(!color || !shape || !action)return 0U;
  Task4Plan plan={0};
  if(!strcmp(color,"RED") && !strcmp(action,"REVERSE"))plan.reverse=1U;
  else if(strcmp(color,"YELLOW") || strcmp(action,"SIDE"))return 0U;
  if(!strcmp(shape,"CIRCLE")){memcpy(plan.edge,"BC",3U);plan.corners=1U;}
  else if(!strcmp(shape,"TRIANGLE")){memcpy(plan.edge,"CD",3U);plan.corners=2U;}
  else if(!strcmp(shape,"SQUARE")){memcpy(plan.edge,"DA",3U);plan.corners=3U;}
  else return 0U;
  plan.branch_id=(uint8_t)(2U*(plan.corners-1U)+(plan.reverse?0U:1U));
  plan.step_count=plan.reverse?3U:5U;
  plan.profile_source=source_status[plan.branch_id];
  plan.garage_pause_ms=plan.reverse?200U:1200U;
  plan.first_line_center=task4_line_position(plan.corners,plan.reverse,0U)+first_line_trim[plan.branch_id];
  for(uint8_t i=0;i<plan.step_count;i++){
    plan.steps[i]=plan.reverse?reverse_steps[i]:side_steps[i];
    plan.steps[i].target+=motion_trim[plan.branch_id][i];
  }
  if(!task4_plan_valid(&plan))return 0U;
  *out=plan;return 1U;
}
int32_t reference_yaw_delta(uint16_t start,uint16_t current)
{
  int32_t delta=(int32_t)current-start;
  if(delta>18000)delta-=36000;
  if(delta< -18000)delta+=36000;
  return delta;
}
int32_t reference_encoder_progress(int16_t a,int16_t b,int16_t c,int8_t direction)
{
  int32_t values[3]={-(int32_t)a,-(int32_t)b,c};
  for(uint8_t i=0;i<3U;i++)for(uint8_t j=i+1U;j<3U;j++)
    if(values[j]<values[i]){int32_t temp=values[i];values[i]=values[j];values[j]=temp;}
  return direction*values[1];
}
