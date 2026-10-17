#include <stdint.h>
#include "stm32f4xx.h"
extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
extern int main(void);
extern void SystemInit(void), __libc_init_array(void);
extern void SysTick_Handler(void), USART1_IRQHandler(void), USART3_IRQHandler(void);
extern void UART4_IRQHandler(void), USART6_IRQHandler(void);
static void Default_Handler(void)
{
  /* Faults must not leave a previously enabled H-bridge driving. */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOEEN;
  (void)RCC->AHB1ENR;
  GPIOB->BSRR = (1UL << 31); /* Reset PB15; leave all other pins unchanged. */
  GPIOB->MODER = (GPIOB->MODER & ~(3UL << 30)) | (1UL << 30);
  GPIOE->BSRR = (1UL << 16); /* Fault: PE0 laser pointer off. */
  GPIOE->MODER = (GPIOE->MODER & ~3UL) | 1UL;
  __DSB();
  while (1) { }
}
void Reset_Handler(void)
{
  uint32_t *src = &_sidata, *dst = &_sdata;
  while (dst < &_edata) *dst++ = *src++;
  for (dst = &_sbss; dst < &_ebss; ) *dst++ = 0U;
  SystemInit();
  __libc_init_array();
  (void)main();
  Default_Handler();
}
__attribute__((used, section(".isr_vector")))
const uintptr_t vectors[98] = {
  [0] = (uintptr_t)&_estack,
  [1] = (uintptr_t)Reset_Handler,
  [2 ... 14] = (uintptr_t)Default_Handler,
  [15] = (uintptr_t)SysTick_Handler,
  [16 ... 52] = (uintptr_t)Default_Handler,
  [53] = (uintptr_t)USART1_IRQHandler,
  [54] = (uintptr_t)Default_Handler,
  [55] = (uintptr_t)USART3_IRQHandler,
  [56 ... 67] = (uintptr_t)Default_Handler,
  [68] = (uintptr_t)UART4_IRQHandler,
  [69 ... 86] = (uintptr_t)Default_Handler,
  [87] = (uintptr_t)USART6_IRQHandler,
  [88 ... 97] = (uintptr_t)Default_Handler
};
