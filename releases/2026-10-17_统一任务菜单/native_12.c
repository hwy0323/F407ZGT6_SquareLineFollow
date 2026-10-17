#include "native_bridge.h"
#define GIT_VERIFIED_CORNER
#define TASK1_ONE_LAP
#define main legacy_12_main
#define SysTick_Handler legacy_12_tick_irq
#define Error_Handler legacy_12_error
#define USART1_IRQHandler legacy_12_uart
#define USART3_IRQHandler legacy_12_uart3
#define UART4_IRQHandler native_12_irq
#define HAL_UART_Transmit menu_native_transmit
#include "../2026-10-17_双层版本/task1_2/main.c"
#include "legacy_setup.inc"
void native_12_init(void){menu_legacy_init();}
void native_12_tick(void)
{
  uint32_t now=HAL_GetTick();menu_legacy_sample(now);
  if(now-menu_control>=CONTROL_MS){menu_control=now;control(now);}
  if(now-menu_motion>=100U){menu_motion=now;check_encoder_motion(now);}
  if(now-menu_report>=REPORT_MS){menu_report=now;report(now);}
}
uint8_t native_12_ready(void){return menu_legacy_ready();}
uint8_t native_12_start(void){start_run();return state!=STOPPED;}
uint8_t native_12_busy(void){return state!=STOPPED;}
void native_12_stop(void){state=STOPPED;stop_motors();}
