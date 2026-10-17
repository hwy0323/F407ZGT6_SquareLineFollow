#include "native_bridge.h"
#include "car_routes.h"
#include "../2026-10-17_发挥一六分支视觉联动/parking_controller.h"
#include <string.h>
#define DECLARE(n) void native_##n##_init(void),native_##n##_tick(void),native_##n##_irq(void),native_##n##_stop(void); \
  uint8_t native_##n##_ready(void),native_##n##_start(void),native_##n##_busy(void)
DECLARE(11);DECLARE(12);DECLARE(21);DECLARE(22);DECLARE(5);
void completion_select(NativeId id);
uint8_t completion_done(void);
typedef struct {
  void (*init)(void),(*tick)(void),(*irq)(void),(*stop)(void);
  uint8_t (*ready)(void),(*start)(void),(*busy)(void);
} NativeOps;
#define OPS(n) {native_##n##_init,native_##n##_tick,native_##n##_irq,native_##n##_stop,native_##n##_ready,native_##n##_start,native_##n##_busy}
#define ROUTE(n) {route_##n##_init,route_##n##_tick,route_##n##_imu_irq,route_##n##_stop,route_##n##_preflight,route_##n##_start,route_##n##_busy}
static const NativeOps tasks[NATIVE_COUNT]={
  OPS(11),OPS(12),OPS(21),OPS(22),ROUTE(A),ROUTE(B),ROUTE(C),ROUTE(D),
  {parking_init,parking_tick,parking_imu_irq,parking_stop,parking_preflight,parking_start,parking_busy},OPS(5)
};
static NativeId active=NATIVE_COUNT;
HAL_StatusTypeDef menu_native_transmit(UART_HandleTypeDef *uart,uint8_t *data,uint16_t size,uint32_t timeout)
{
  if(uart->Instance==USART1 && size && size<320U){
    char line[320];memcpy(line,data,size);line[size]=0;native_event(line);
  }
  return HAL_UART_Transmit(uart,data,size,timeout);
}
void native_select(NativeId id)
{
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
  HAL_NVIC_DisableIRQ(UART4_IRQn);
  if(active<NATIVE_COUNT)tasks[active].stop();
  active=id;completion_select(id);
  if(id<NATIVE_COUNT)tasks[id].init();
}
void native_tick(void){if(active<NATIVE_COUNT)tasks[active].tick();}
void native_imu_irq(void){if(active<NATIVE_COUNT)tasks[active].irq();else (void)UART4->DR;}
uint8_t native_ready(void){return active<NATIVE_COUNT && tasks[active].ready();}
uint8_t native_start(void){return active<NATIVE_COUNT && tasks[active].start();}
uint8_t native_busy(void){return active<NATIVE_COUNT && tasks[active].busy();}
uint8_t native_done(void){return active==NATIVE_4?parking_done():completion_done();}
void native_stop(void)
{
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
  if(active<NATIVE_COUNT)tasks[active].stop();
}
uint8_t native_plan(const char *color,const char *shape,const char *action)
{return active==NATIVE_4 && parking_configure(color,shape,action);}
