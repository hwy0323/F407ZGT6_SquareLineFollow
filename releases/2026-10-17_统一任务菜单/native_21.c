#include "native_bridge.h"
#define GIT_VERIFIED_CORNER
#define PIVOT_TEST
#define PIVOT_MULTI
#define ROUTE_REPLAY
#define REVERSE_PARK_AUTO
#define main legacy_21_main
#define SysTick_Handler legacy_21_tick_irq
#define Error_Handler legacy_21_error
#define USART1_IRQHandler legacy_21_uart
#define USART3_IRQHandler legacy_21_uart3
#define UART4_IRQHandler native_21_irq
#define HAL_UART_Transmit menu_native_transmit
#include "../2026-10-17_双层版本/task2_1/main.c"
#include "legacy_setup.inc"
void native_21_init(void){menu_legacy_init();}
void native_21_tick(void)
{
  uint32_t now=HAL_GetTick();menu_legacy_sample(now);
  if(auto_phase==2U && now-auto_pause_start_ms>=AUTO_GARAGE_PAUSE_MS){
    route_begin(now);auto_phase=route_active?3U:4U;
  }
  if(auto_phase==3U && !route_active && !pivot_active)auto_phase=4U;
  if(now-menu_control>=CONTROL_MS){
    menu_control=now;
    if(auto_phase==1U && state!=STOPPED)control(now);
    pivot_control(now);route_tick(now);
  }
  if(auto_phase==1U && now-menu_motion>=100U){menu_motion=now;check_encoder_motion(now);}
  if(now-menu_report>=REPORT_MS){menu_report=now;if(auto_phase<=2U)report(now);else pivot_report(now);}
}
uint8_t native_21_ready(void){return menu_legacy_ready();}
uint8_t native_21_start(void){start_run();if(state==FOLLOW)auto_phase=1U;return auto_phase==1U;}
uint8_t native_21_busy(void){return state!=STOPPED || pivot_active || route_active || auto_phase==2U;}
void native_21_stop(void){state=STOPPED;pivot_active=route_active=route_waiting=0;auto_phase=4U;stop_motors();}
