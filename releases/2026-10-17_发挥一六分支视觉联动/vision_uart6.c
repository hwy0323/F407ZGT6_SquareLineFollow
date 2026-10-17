#include "main.h"
#include "vision_uart6.h"
#include <string.h>

/* IRQ only stores bytes. All parsing and frame sending runs in main. */
#define RX_SIZE 512U
#define TX_SLOTS 8U
static volatile uint8_t rx[RX_SIZE];
static volatile uint16_t head, tail;
static volatile uint8_t overflow;
static char tx[TX_SLOTS][MV_MAX_FRAME];
static uint16_t tx_length[TX_SLOTS], tx_offset;
static uint8_t tx_head, tx_tail, tx_used, tx_fault;
volatile uint32_t vision_rx_bytes, vision_rx_errors, vision_rx_overflows;
volatile uint32_t vision_tx_bytes;
static UART_HandleTypeDef vision_uart;
static char trace_rx[MV_MAX_FRAME];
static uint16_t trace_used;

void vision_uart6_reset(void)
{
  uint32_t mask=__get_PRIMASK();__disable_irq();
  tail=head;overflow=0U;
  tx_head=tx_tail=tx_used=tx_fault=0U;tx_offset=trace_used=0U;
  __set_PRIMASK(mask);
}

void vision_uart6_init(void)
{
  GPIO_InitTypeDef pin = {0};
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_USART6_CLK_ENABLE();
  pin.Pin = GPIO_PIN_9 | GPIO_PIN_14;
  pin.Mode = GPIO_MODE_AF_PP;
  pin.Pull = GPIO_PULLUP;
  pin.Speed = GPIO_SPEED_FREQ_HIGH;
  pin.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOG, &pin);
  vision_uart.Instance = USART6;
  vision_uart.Init.BaudRate = 115200U;
  vision_uart.Init.WordLength = UART_WORDLENGTH_8B;
  vision_uart.Init.StopBits = UART_STOPBITS_1;
  vision_uart.Init.Parity = UART_PARITY_NONE;
  vision_uart.Init.Mode = UART_MODE_TX_RX;
  vision_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  vision_uart.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&vision_uart) != HAL_OK) Error_Handler();
  SET_BIT(USART6->CR1, USART_CR1_RXNEIE);
  HAL_NVIC_SetPriority(USART6_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(USART6_IRQn);
}

void USART6_IRQHandler(void)
{
  uint32_t status = USART6->SR;
  if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
    (void)USART6->DR;
    ++vision_rx_errors;
    overflow = 1U;
    return;
  }
  if (status & USART_SR_RXNE) {
    uint8_t value = (uint8_t)USART6->DR;
    uint16_t next = (uint16_t)((head + 1U) & (RX_SIZE - 1U));
    ++vision_rx_bytes;
    if (next == tail) { overflow = 1U; ++vision_rx_overflows; }
    else { rx[head] = value; __DMB(); head = next; }
  }
}

uint8_t vision_uart6_send(const char *data, uint16_t length)
{
  if (!length || length > MV_MAX_FRAME || tx_used == TX_SLOTS) {
    tx_fault = 1U;
    return 0U;
  }
  memcpy(tx[tx_head], data, length);
  tx_length[tx_head] = length;
  tx_head = (uint8_t)((tx_head + 1U) % TX_SLOTS);
  ++tx_used;
  return 1U;
}

uint8_t vision_uart6_fault(void) { return tx_fault; }

void vision_uart6_poll(MvClient *client, uint32_t now)
{
  if (overflow) {
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    tail = head;
    overflow = 0U;
    __set_PRIMASK(mask);
    mv_reset_rx(client);
    trace_used=0U;
  }
  /* Bound work per turn, so noise cannot starve motor/KEY handling. */
  for (unsigned int i = 0U; i < 256U && tail != head; ++i) {
    uint8_t value = rx[tail];
    tail = (uint16_t)((tail + 1U) & (RX_SIZE - 1U));
    if(value=='$')trace_used=0U;
    if(trace_used<MV_MAX_FRAME)trace_rx[trace_used++]=(char)value;
    if(value=='\n'){
      vision_trace("RX",trace_rx,trace_used);trace_used=0U;
    }
    MvPhase old_phase=client->phase;
    mv_feed(client, value, now);
    if(old_phase!=MV_HELLO && client->phase==MV_HELLO){
      /* Drop stale SID requests queued before detecting the peer reboot. */
      tx_head=tx_tail=tx_used=0U;tx_offset=0U;
    }
  }
  if (tx_used && (USART6->SR & USART_SR_TXE)) {
    USART6->DR = (uint8_t)tx[tx_tail][tx_offset++];
    ++vision_tx_bytes;
    if (tx_offset == tx_length[tx_tail]) {
      vision_trace("TX",tx[tx_tail],tx_length[tx_tail]);
      tx_offset = 0U;
      tx_tail = (uint8_t)((tx_tail + 1U) % TX_SLOTS);
      --tx_used;
    }
  }
}
