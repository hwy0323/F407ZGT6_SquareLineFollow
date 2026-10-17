#include "gimbal_pose.h"
uint16_t gimbal_pulse_us(uint16_t angle,uint16_t full_range)
{
  if(!full_range || angle>full_range)return 0U;
  return (uint16_t)(GIMBAL_MIN_PULSE_US+
    ((uint32_t)angle*(GIMBAL_MAX_PULSE_US-GIMBAL_MIN_PULSE_US)+full_range/2U)/full_range);
}
uint8_t gimbal_pose_for_task(uint8_t task,GimbalPose *p)
{
  if(!p || (task!=3U && task!=4U))return 0U;
  p->upper_deg=150U;p->lower_deg=task==3U?90U:75U;
  p->upper_us=gimbal_pulse_us(p->upper_deg,GIMBAL_UPPER_RANGE_DEG);
  p->lower_us=gimbal_pulse_us(p->lower_deg,GIMBAL_LOWER_RANGE_DEG);
  return p->upper_us!=0U && p->lower_us!=0U;
}
