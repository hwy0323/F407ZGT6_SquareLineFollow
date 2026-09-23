#include "gimbal_test.h"

/*
 * Direct control of the upper hobby servo.
 * TIM5 CH3 on PA2 outputs a 50 Hz signal with a 1 us timer tick.
 * The 0.5...2.5 ms pulse range represents 0...180 degrees.
 */
#define SERVO_TIMER_PERIOD_US 20000U
#define SERVO_STEP_TIME_MS 100U
#define SERVO_MIN_ANGLE_DEG 80U
#define SERVO_MAX_ANGLE_DEG 100U
#define SERVO_START_ANGLE_DEG 90U
#define KEY_DEBOUNCE_MS 30U

static TIM_HandleTypeDef htim5;
static uint8_t upper_angle = SERVO_START_ANGLE_DEG;
static int8_t upper_direction = 1;
static uint32_t last_step_tick;
static uint32_t key_change_tick;
static uint8_t key_raw_pressed;
static uint8_t key_stable_pressed;
static uint8_t gimbal_running;

static uint32_t Servo_AngleToPulseUs(uint8_t angle)
{
  /* 0 degrees = 500 us, 180 degrees = 2500 us. */
  return 500U + ((uint32_t)angle * 2000U) / 180U;
}

static void Servo_SetUpperAngle(uint8_t angle)
{
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3,
                        Servo_AngleToPulseUs(angle));
}

uint8_t GimbalTest_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  TIM_OC_InitTypeDef pwm = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM5_CLK_ENABLE();

  /* PA2 = TIM5 CH3, connected to the upper servo's yellow signal wire. */
  gpio.Pin = GPIO_PIN_2;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* TIM5 clock is 84 MHz with the project's 168 MHz clock configuration. */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 84U - 1U;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = SERVO_TIMER_PERIOD_US - 1U;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) {
    return 0U;
  }

  pwm.OCMode = TIM_OCMODE_PWM1;
  pwm.Pulse = Servo_AngleToPulseUs(SERVO_START_ANGLE_DEG);
  pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
  pwm.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &pwm, TIM_CHANNEL_3) != HAL_OK) {
    return 0U;
  }
  upper_angle = SERVO_START_ANGLE_DEG;
  upper_direction = 1;
  last_step_tick = HAL_GetTick();
  key_change_tick = last_step_tick;
  key_raw_pressed = 0U;
  key_stable_pressed = 0U;
  gimbal_running = 0U;
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  return 1U;
}

void GimbalTest_Process(uint32_t now_ms)
{
  uint8_t pressed =
      (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) ? 1U : 0U;

  if (pressed != key_raw_pressed) {
    key_raw_pressed = pressed;
    key_change_tick = now_ms;
  }

  if ((now_ms - key_change_tick >= KEY_DEBOUNCE_MS) &&
      (key_stable_pressed != key_raw_pressed)) {
    key_stable_pressed = key_raw_pressed;
    if (key_stable_pressed != 0U) {
      if (gimbal_running == 0U) {
        upper_angle = SERVO_START_ANGLE_DEG;
        upper_direction = 1;
        Servo_SetUpperAngle(upper_angle);
        if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3) == HAL_OK) {
          gimbal_running = 1U;
          last_step_tick = now_ms;
          HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }
      } else {
        (void)HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_3);
        gimbal_running = 0U;
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      }
    }
  }

  if (gimbal_running == 0U) return;
  if (now_ms - last_step_tick < SERVO_STEP_TIME_MS) {
    return;
  }
  last_step_tick = now_ms;

  if (upper_direction > 0) {
    if (upper_angle >= SERVO_MAX_ANGLE_DEG) {
      upper_direction = -1;
      upper_angle--;
    } else {
      upper_angle++;
    }
  } else {
    if (upper_angle <= SERVO_MIN_ANGLE_DEG) {
      upper_direction = 1;
      upper_angle++;
    } else {
      upper_angle--;
    }
  }

  Servo_SetUpperAngle(upper_angle);
}
