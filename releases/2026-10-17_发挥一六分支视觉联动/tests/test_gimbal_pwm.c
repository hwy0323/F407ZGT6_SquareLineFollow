#include <assert.h>
#include <stdio.h>
#include "../gimbal_pwm.c"
int main(void)
{
  assert(!gimbal_begin_task(3,0) && !gimbal_is_settled(9999));
  assert(gimbal_pwm_init());
  assert(servo_timer.Init.Prescaler==83 && servo_timer.Init.Period==19999);
  assert(fake_running3 && fake_running4 && servo_timer.c3==0 && servo_timer.c4==0);
  assert(!gimbal_is_settled(9999)); /* No commanded boot position. */
  assert(gimbal_begin_task(4,100));
  assert(servo_timer.c3==2167 && servo_timer.c4==1056 && !gimbal_is_settled(1099));
  assert(gimbal_is_settled(1100));
  assert(gimbal_begin_task(3,2000));
  assert(servo_timer.c3==2167 && servo_timer.c4==1167 && !gimbal_is_settled(2999));
  assert(gimbal_is_settled(3000));
  uint32_t old=servo_timer.c4;
  assert(!gimbal_begin_task(5,4000) && servo_timer.c4==old);
  assert(gimbal_begin_task(4,UINT32_MAX-500U));
  assert(!gimbal_is_settled(498U) && gimbal_is_settled(499U));
  fake_fail=1;assert(!gimbal_pwm_init() && !gimbal_begin_task(3,6000));
  puts("PASS PWM: PA2/CH3 upper, PA3/CH4 lower, 50Hz, boot zero duty, both poses, settling/wrap and init failure");
}
