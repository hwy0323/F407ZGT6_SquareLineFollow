#include <stdint.h>

extern uint32_t _estack, _sidata, _sdata, _edata, _sbss, _ebss;
extern int main(void);
extern void SystemInit(void);
extern void __libc_init_array(void);
extern void NMI_Handler(void), HardFault_Handler(void), MemManage_Handler(void), BusFault_Handler(void), UsageFault_Handler(void), SVC_Handler(void), DebugMon_Handler(void), PendSV_Handler(void), SysTick_Handler(void);
extern void USART1_IRQHandler(void);

void Default_Handler(void) { while (1) { } }
void Reset_Handler(void)
{
  uint32_t *source = &_sidata;
  uint32_t *destination = &_sdata;
  while (destination < &_edata) *destination++ = *source++;
  for (destination = &_sbss; destination < &_ebss; ) *destination++ = 0;
  SystemInit();
  __libc_init_array();
  (void)main();
  Default_Handler();
}

__attribute__((used, section(".isr_vector")))
const uintptr_t g_pfnVectors[98] = {
  [0] = (uintptr_t)&_estack, [1] = (uintptr_t)Reset_Handler,
  [2] = (uintptr_t)NMI_Handler, [3] = (uintptr_t)HardFault_Handler,
  [4] = (uintptr_t)MemManage_Handler, [5] = (uintptr_t)BusFault_Handler,
  [6] = (uintptr_t)UsageFault_Handler, [11] = (uintptr_t)SVC_Handler,
  [12] = (uintptr_t)DebugMon_Handler, [14] = (uintptr_t)PendSV_Handler,
  [15] = (uintptr_t)SysTick_Handler,
  [16 ... 52] = (uintptr_t)Default_Handler,
  [53] = (uintptr_t)USART1_IRQHandler,
  [54 ... 97] = (uintptr_t)Default_Handler
};
