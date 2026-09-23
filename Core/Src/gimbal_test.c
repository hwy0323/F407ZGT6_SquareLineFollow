#include "gimbal_test.h"

/*
 * C06A/C06B gimbal controller serial protocol:
 *   FF FE bottom upper 00 00 00 00 00 checksum
 * checksum is the XOR of bytes 2...8.
 * A zero angle tells the controller not to update that axis.
 */
#define GIMBAL_BAUD_RATE 115200U
#define GIMBAL_START_DELAY_MS 1000U
#define GIMBAL_STEP_TIME_MS 100U
#define GIMBAL_UPPER_MIN_DEG 60U
#define GIMBAL_UPPER_MAX_DEG 120U
#define GIMBAL_UPPER_START_DEG 90U

static UART_HandleTypeDef huart3;
static uint8_t upper_angle = GIMBAL_UPPER_START_DEG;
static int8_t upper_direction = 1;
static uint32_t start_tick;
static uint32_t last_step_tick;

static HAL_StatusTypeDef Gimbal_SendUpperAngle(uint8_t angle)
{
  uint8_t frame[10] = {
    0xFFU, 0xFEU,
    0x00U, /* Bottom axis: zero means do not update it. */
    angle,
    0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
    0x00U
  };
  uint8_t checksum = 0U;
  uint8_t i;

  for (i = 2U; i <= 8U; i++) {
    checksum ^= frame[i];
  }
  frame[9] = checksum;

  return HAL_UART_Transmit(&huart3, frame, sizeof(frame), 20U);
}

uint8_t GimbalTest_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();

  /* Only TX is needed: F407 PC10 -> C06B PB11 (RX). */
  gpio.Pin = GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOC, &gpio);

  huart3.Instance = USART3;
  huart3.Init.BaudRate = GIMBAL_BAUD_RATE;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    return 0U;
  }

  upper_angle = GIMBAL_UPPER_START_DEG;
  upper_direction = 1;
  start_tick = HAL_GetTick();
  last_step_tick = start_tick;
  return 1U;
}

void GimbalTest_Process(uint32_t now_ms)
{
  if (now_ms - start_tick < GIMBAL_START_DELAY_MS) {
    return;
  }
  if (now_ms - last_step_tick < GIMBAL_STEP_TIME_MS) {
    return;
  }
  last_step_tick = now_ms;

  (void)Gimbal_SendUpperAngle(upper_angle);

  if (upper_direction > 0) {
    if (upper_angle >= GIMBAL_UPPER_MAX_DEG) {
      upper_direction = -1;
      upper_angle--;
    } else {
      upper_angle++;
    }
  } else {
    if (upper_angle <= GIMBAL_UPPER_MIN_DEG) {
      upper_direction = 1;
      upper_angle++;
    } else {
      upper_angle--;
    }
  }
}
