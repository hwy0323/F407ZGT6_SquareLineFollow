#include "native_bridge.h"
#define GIT_VERIFIED_CORNER
#define PIVOT_TEST
#define PIVOT_MULTI
#define ROUTE_REPLAY
#define SIDE_PARK_AUTO
/* 给侧停动作表加名前缀，避免与激光任务的独立动作表同名。 */
#define reference_steps side_reference_steps
#define reference_feed side_reference_feed
#define reference_yaw_delta side_reference_yaw_delta
#define reference_encoder_progress side_reference_encoder_progress
#define main legacy_22_main
#define SysTick_Handler legacy_22_tick_irq
#define Error_Handler legacy_22_error
#define USART1_IRQHandler legacy_22_uart
#define USART3_IRQHandler legacy_22_uart3
#define UART4_IRQHandler native_22_irq
#define HAL_UART_Transmit menu_native_transmit
#include "../2026-10-17_双层版本/task2_2/reference_steps.c"
#include "../2026-10-17_双层版本/task2_2/main.c"
#include "legacy_setup.inc"
void native_22_init(void){menu_legacy_init();}
void native_22_tick(void)
{
  uint32_t now=HAL_GetTick();menu_legacy_sample(now);
  if(now-menu_control>=CONTROL_MS){
    menu_control=now;if(auto_phase==1U && state!=STOPPED)control(now);
    reference_control(now);combined_tick(now);
  }
  if(auto_phase==1U && now-menu_motion>=100U){menu_motion=now;check_encoder_motion(now);}
  if(now-menu_report>=REPORT_MS){menu_report=now;if(auto_phase<=2U)report(now);else reference_report(now);}
}
uint8_t native_22_ready(void){return menu_legacy_ready();}
uint8_t native_22_start(void){start_run();return auto_phase==1U && state==FOLLOW;}
uint8_t native_22_busy(void){return state!=STOPPED || pivot_active || auto_phase==2U || auto_phase==3U;}
void native_22_stop(void){combined_abort("MENU_STOP");}
