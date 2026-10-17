#ifndef GIMBAL_PWM_H
#define GIMBAL_PWM_H
#include "gimbal_pose.h"
uint8_t gimbal_pwm_init(void);
uint8_t gimbal_begin_task(uint8_t task,uint32_t now);
uint8_t gimbal_is_settled(uint32_t now);
extern volatile uint32_t gimbal_status[8];
#endif
