#include "route_telemetry.h"
#include <stdio.h>
#include <string.h>

/* DAP TX -> PG9 (USART6_RX), DAP RX <- PG14 (USART6_TX). */
static UART_HandleTypeDef huart6;

static void Send_Text(const char *text)
{
  (void)HAL_UART_Transmit(&huart6, (uint8_t *)text,
                          (uint16_t)strlen(text), 100U);
}

uint8_t RouteTelemetry_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_USART6_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_9 | GPIO_PIN_14;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOG, &gpio);

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200U;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  return (HAL_UART_Init(&huart6) == HAL_OK) ? 1U : 0U;
}

void RouteTelemetry_SendReady(void)
{
  Send_Text("READY,press_key_to_start\r\n");
  Send_Text("HEADER,ms,raw,used,error,lf,rf,lr,rr,left,right,state,turns\r\n");
}

void RouteTelemetry_SendStart(uint32_t now_ms)
{
  char text[48];
  (void)snprintf(text, sizeof(text), "EVENT,START,%lu\r\n",
                 (unsigned long)now_ms);
  Send_Text(text);
}

void RouteTelemetry_SendStop(uint32_t elapsed_ms, const char *reason)
{
  char text[64];
  (void)snprintf(text, sizeof(text), "EVENT,STOP,%lu,%s\r\n",
                 (unsigned long)elapsed_ms, reason);
  Send_Text(text);
}

void RouteTelemetry_SendSample(uint32_t elapsed_ms,
                               uint8_t sensor_raw,
                               uint8_t sensor_used,
                               int16_t sensor_error,
                               int32_t encoder_lf,
                               int32_t encoder_rf,
                               int32_t encoder_lr,
                               int32_t encoder_rr,
                               int16_t left_command,
                               int16_t right_command,
                               uint8_t state,
                               uint8_t turns)
{
  char text[144];

  (void)snprintf(text, sizeof(text),
                 "D,%lu,%02X,%02X,%d,%ld,%ld,%ld,%ld,%d,%d,%u,%u\r\n",
                 (unsigned long)elapsed_ms,
                 (unsigned int)sensor_raw,
                 (unsigned int)sensor_used,
                 (int)sensor_error,
                 (long)encoder_lf,
                 (long)encoder_rf,
                 (long)encoder_lr,
                 (long)encoder_rr,
                 (int)left_command,
                 (int)right_command,
                 (unsigned int)state,
                 (unsigned int)turns);
  Send_Text(text);
}

uint8_t RouteTelemetry_ReadByte(uint8_t *value)
{
  if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) == RESET) {
    return 0U;
  }
  *value = (uint8_t)(huart6.Instance->DR & 0xFFU);
  return 1U;
}
