#include "entry_plan.h"
ReferenceStep reference_steps[7];
uint8_t reference_count;
void search_plan_reset(void)
{
  reference_steps[0]=(ReferenceStep){'F',PROBE_COUNTS};reference_count=1U;
}
uint8_t search_plan_return(int32_t forward_actual)
{
  if(forward_actual<PROBE_COUNTS-40 || forward_actual>PROBE_COUNTS+40)return 0U;
  /* Return the measured forward travel, not a second unrelated nominal value. */
  reference_steps[1]=(ReferenceStep){'B',forward_actual};reference_count=2U;return 1U;
}
uint8_t search_plan_entry(GarageType type)
{
  if(type==GARAGE_REVERSE){
    reference_steps[2]=(ReferenceStep){'F',1198};
    reference_steps[3]=(ReferenceStep){'L',9000};
    reference_steps[4]=(ReferenceStep){'B',1146};
    reference_count=5U;return 1U;
  }
  if(type==GARAGE_SIDE){
    reference_steps[2]=(ReferenceStep){'L',546};
    reference_steps[3]=(ReferenceStep){'F',2194};
    reference_steps[4]=(ReferenceStep){'L',4193};
    reference_steps[5]=(ReferenceStep){'B',1347};
    reference_steps[6]=(ReferenceStep){'R',4474};
    reference_count=7U;return 1U;
  }
  return 0U;
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
  int32_t v[3]={-(int32_t)a,-(int32_t)b,c};
  for(uint8_t i=0;i<3;i++)for(uint8_t j=i+1;j<3;j++)
    if(v[j]<v[i]){int32_t t=v[i];v[i]=v[j];v[j]=t;}
  return direction*v[1];
}
