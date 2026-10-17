#ifdef GIMBAL_PWM_HOST_TEST
#include "tests/fake_pwm_hal.h"
#else
#include "main.h"
#endif
#include "gimbal_pwm.h"
static TIM_HandleTypeDef servo_timer;
static uint8_t initialized,commanded;
static uint32_t command_ms;
/* magic, task, upper_deg, lower_deg, upper_us, lower_us, command_tick, commanded */
volatile uint32_t gimbal_status[8]={0x5352564FU};

uint8_t gimbal_pwm_init(void)
{
  GPIO_InitTypeDef pin={0};TIM_OC_InitTypeDef channel={0};
  initialized=commanded=0U;
  __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_TIM5_CLK_ENABLE();
  /* PA2=TIM5_CH3 upper 180 degrees; PA3=TIM5_CH4 lower 270 degrees. */
  pin.Pin=GPIO_PIN_2|GPIO_PIN_3;pin.Mode=GPIO_MODE_AF_PP;
  pin.Pull=GPIO_NOPULL;pin.Speed=GPIO_SPEED_FREQ_HIGH;pin.Alternate=GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOA,&pin);
  /* APB1 timer clock is 84 MHz at the existing 168 MHz system clock. */
  servo_timer.Instance=TIM5;servo_timer.Init.Prescaler=84U-1U;
  servo_timer.Init.CounterMode=TIM_COUNTERMODE_UP;servo_timer.Init.Period=20000U-1U;
  servo_timer.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  servo_timer.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE;
  if(HAL_TIM_PWM_Init(&servo_timer)!=HAL_OK)return 0U;
  channel.OCMode=TIM_OCMODE_PWM1;channel.Pulse=0U;
  channel.OCPolarity=TIM_OCPOLARITY_HIGH;channel.OCFastMode=TIM_OCFAST_DISABLE;
  if(HAL_TIM_PWM_ConfigChannel(&servo_timer,&channel,TIM_CHANNEL_3)!=HAL_OK ||
     HAL_TIM_PWM_ConfigChannel(&servo_timer,&channel,TIM_CHANNEL_4)!=HAL_OK)return 0U;
  /* Zero duty at boot: no position pulse or automatic 90-degree reset. */
  if(HAL_TIM_PWM_Start(&servo_timer,TIM_CHANNEL_3)!=HAL_OK ||
     HAL_TIM_PWM_Start(&servo_timer,TIM_CHANNEL_4)!=HAL_OK)return 0U;
  initialized=1U;return 1U;
}
uint8_t gimbal_begin_task(uint8_t task,uint32_t now)
{
  GimbalPose p;
  if(!initialized || !gimbal_pose_for_task(task,&p))return 0U;
  __HAL_TIM_SET_COMPARE(&servo_timer,TIM_CHANNEL_3,p.upper_us);
  __HAL_TIM_SET_COMPARE(&servo_timer,TIM_CHANNEL_4,p.lower_us);
  command_ms=now;commanded=1U;
  gimbal_status[1]=task;gimbal_status[2]=p.upper_deg;gimbal_status[3]=p.lower_deg;
  gimbal_status[4]=p.upper_us;gimbal_status[5]=p.lower_us;
  gimbal_status[6]=now;gimbal_status[7]=1U;
  return 1U;
}
uint8_t gimbal_is_settled(uint32_t now)
{
  /* Timed settling only: hobby servos supply no measured angle feedback. */
  return commanded && (uint32_t)(now-command_ms)>=GIMBAL_SETTLE_MS;
}
