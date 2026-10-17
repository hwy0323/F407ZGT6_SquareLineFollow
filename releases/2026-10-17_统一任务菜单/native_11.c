#include "native_bridge.h"
#define GIT_VERIFIED_CORNER
#define TASK1_AB
#define main legacy_11_main
#define SysTick_Handler legacy_11_tick_irq
#define Error_Handler legacy_11_error
#define USART1_IRQHandler legacy_11_uart
#define USART3_IRQHandler legacy_11_uart3
#define UART4_IRQHandler native_11_irq
#define HAL_UART_Transmit menu_native_transmit
#include "../2026-10-17_双层版本/task1_1/main.c"
#include "legacy_setup.inc"
void native_11_init(void){menu_legacy_init();}
void native_11_tick(void)
{
  uint32_t now=HAL_GetTick();menu_legacy_sample(now);
  if(now-menu_control>=CONTROL_MS){menu_control=now;control(now);}
  if(now-menu_motion>=100U){menu_motion=now;check_encoder_motion(now);}
  if(now-menu_report>=REPORT_MS){menu_report=now;report(now);}
}
uint8_t native_11_ready(void){return menu_legacy_ready();}
uint8_t native_11_start(void){start_run();return state!=STOPPED;}
uint8_t native_11_busy(void){return state!=STOPPED;}
void native_11_stop(void){state=STOPPED;stop_motors();}
