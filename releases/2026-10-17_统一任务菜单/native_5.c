#include "native_bridge.h"
#define reference_yaw_delta laser_reference_yaw_delta
#define reference_encoder_progress laser_reference_encoder_progress
#define main legacy_5_main
#define SysTick_Handler legacy_5_tick_irq
#define Error_Handler legacy_5_error
#define USART1_IRQHandler legacy_5_uart
#define USART3_IRQHandler legacy_5_uart3
#define UART4_IRQHandler native_5_irq
#define HAL_UART_Transmit menu_native_transmit
#include "../2026-10-17_发挥二_激光搜索入库_未测试/entry_plan.c"
#include "../2026-10-17_发挥二_激光搜索入库_未测试/search_logic.c"
#include "../2026-10-17_发挥二_激光搜索入库_未测试/main.c"
#include "legacy_setup.inc"
void native_5_init(void)
{
  menu_legacy_init();laser_init();
  char line[100];snprintf(line,sizeof(line),"MENU,LASER_INIT,status=%u,threshold_mm=%u\r\n",laser_status,LASER_TYPE_THRESHOLD_MM);send_text(line);
}
void native_5_tick(void)
{
  uint32_t now=HAL_GetTick();menu_legacy_sample(now);
  if(now-menu_control>=10U){
    menu_control=now;search_safety(now);
    if(auto_phase==SEARCH)search_navigation(now);
    if(auto_phase>IDLE && auto_phase<DONE){reference_control(now);search_tick(now);}
  }
  if(now-menu_report>=100U){menu_report=now;report(now);if(reference_next)reference_report(now);}
  if(auto_phase==IDLE || auto_phase>=DONE)stop_motors();
}
uint8_t native_5_ready(void){return laser_status==4U && menu_legacy_ready();}
uint8_t native_5_start(void){start_run();return auto_phase==SEARCH;}
uint8_t native_5_busy(void){return auto_phase>IDLE && auto_phase<DONE;}
void native_5_stop(void){combined_abort("MENU_STOP");}
