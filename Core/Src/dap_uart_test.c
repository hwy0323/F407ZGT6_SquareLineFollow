#include "dap_uart_test.h"
#include <string.h>

/*
 * Wireless DAP virtual serial port:
 *   DAP TX -> PG9  (USART6_RX)
 *   DAP RX <- PG14 (USART6_TX)
 *   115200 baud, 8 data bits, no parity, 1 stop bit.
 */
#define UART_REPORT_PERIOD_MS 500U

static UART_HandleTypeDef huart6;
static uint32_t last_report_tick;

static void Send_Text(const char *text)
{
  (void)HAL_UART_Transmit(&huart6, (uint8_t *)text,
                          (uint16_t)strlen(text), 100U);
}

uint8_t DapUartTest_Init(void)
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
  if (HAL_UART_Init(&huart6) != HAL_OK) {
    return 0U;
  }

  last_report_tick = HAL_GetTick();
  Send_Text("\r\nDAP_UART6_TEST_READY\r\n");
  return 1U;
}

void DapUartTest_Process(uint32_t now_ms)
{
  uint8_t received_byte;

  /* Echo characters sent from COM14 to verify the reverse direction. */
  if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET) {
    received_byte = (uint8_t)(huart6.Instance->DR & 0xFFU);
    (void)HAL_UART_Transmit(&huart6, &received_byte, 1U, 20U);
  }

  if (now_ms - last_report_tick < UART_REPORT_PERIOD_MS) {
    return;
  }

  last_report_tick = now_ms;
  Send_Text("UART6_OK\r\n");
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
