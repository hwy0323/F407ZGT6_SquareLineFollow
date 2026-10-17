#ifndef TASK4_FAKE_APP_HAL_H
#define TASK4_FAKE_APP_HAL_H
#include <stdint.h>
typedef struct { uint32_t unused; } GPIO_TypeDef;
typedef struct { uint32_t Pin,Mode,Pull; } GPIO_InitTypeDef;
typedef struct { uint32_t SR,DR; } FakeUart;
typedef struct { uint32_t DEMCR; } FakeCore;
typedef struct { uint32_t CYCCNT,CTRL; } FakeDwt;
extern GPIO_TypeDef fake_a,fake_b,fake_c;
extern FakeUart fake_uart;
extern FakeCore fake_core;
extern FakeDwt fake_dwt;
extern uint32_t fake_now;
extern uint8_t fake_key;
#define GPIOA (&fake_a)
#define GPIOB (&fake_b)
#define GPIOC (&fake_c)
#define USART1 (&fake_uart)
#define CoreDebug (&fake_core)
#define DWT (&fake_dwt)
#define GPIO_PIN_15 0x8000U
#define GPIO_PIN_13 0x2000U
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_PULLDOWN 2U
#define UART4_IRQn 0U
#define USART_SR_PE 1U
#define USART_SR_FE 2U
#define USART_SR_NE 4U
#define USART_SR_ORE 8U
#define USART_SR_RXNE 0x20U
#define USART_SR_TXE 0x80U
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
static inline void HAL_Init(void){}
static inline uint32_t HAL_GetTick(void){return fake_now;}
static inline void HAL_IncTick(void){fake_now++;}
static inline void HAL_NVIC_DisableIRQ(uint32_t irq){(void)irq;}
static inline void HAL_NVIC_EnableIRQ(uint32_t irq){(void)irq;}
static inline void HAL_GPIO_Init(GPIO_TypeDef *port,GPIO_InitTypeDef *pin){(void)port;(void)pin;}
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *port,uint32_t pin,uint32_t state){(void)port;(void)pin;(void)state;}
static inline uint32_t HAL_GPIO_ReadPin(GPIO_TypeDef *port,uint32_t pin){(void)port;(void)pin;return fake_key;}
#endif
