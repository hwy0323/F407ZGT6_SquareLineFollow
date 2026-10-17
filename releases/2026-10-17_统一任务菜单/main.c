#include "main.h"
#include "menu_app.h"
#include "menu_ui.h"
#include "menu_command.h"
#include "keys.h"
#include "native.h"
#include "oled.h"
#include "gimbal_pwm.h"
#include "vision_uart6.h"
#include <stdio.h>
#include <string.h>
_Static_assert(ALL_MENU_COUNT==MENU_ITEM_COUNT,"menu item counts must match");
static MenuApp app;
static MenuUi ui;
static MenuKey keys[2];
static UART_HandleTypeDef dap;
static volatile uint8_t rx[128],head,tail,rx_fault,remote_stop;
static MenuCommand command;
volatile uint32_t unified_menu_status[16];
uint32_t app_now(void){return HAL_GetTick();}
void SysTick_Handler(void){HAL_IncTick();}
void app_force_stop(void)
{
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
}
void app_reboot(void)
{
  app_force_stop();HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);NVIC_SystemReset();
}
uint8_t app_screen_ready(void){return OLED_Is_Ready();}
void Error_Handler(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_GPIOE_CLK_ENABLE();GPIO_InitTypeDef p={0};
  app_force_stop();p.Pin=GPIO_PIN_15;p.Mode=GPIO_MODE_OUTPUT_PP;p.Pull=GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB,&p);
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);p.Pin=GPIO_PIN_0;HAL_GPIO_Init(GPIOE,&p);
  while(1){}
}
void app_log(const char *line)
{
  (void)HAL_UART_Transmit(&dap,(uint8_t*)line,(uint16_t)strlen(line),60U);
}
void vision_trace(const char *direction,const char *data,uint16_t size)
{
  if(app.phase==APP_RUNNING)return;
  char line[240],clean[MV_MAX_FRAME+1U];uint16_t used=0;
  for(uint16_t i=0;i<size && used<MV_MAX_FRAME;i++){
    uint8_t b=(uint8_t)data[i];if(b!='\r' && b!='\n')clean[used++]=b>=32 && b<=126?(char)b:'.';
  }
  clean[used]=0;snprintf(line,sizeof(line),"V6_%s,%s\r\n",direction,clean);app_log(line);
}
void USART1_IRQHandler(void)
{
  uint32_t flags=USART1->SR;uint8_t b=(uint8_t)USART1->DR;
  if(flags&(USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)){rx_fault=1;return;}
  if(flags&USART_SR_RXNE){
    if(b=='X'||b=='x'){app_force_stop();remote_stop=1;return;}
    uint8_t next=(head+1U)&127U;
    if(next==tail)rx_fault=1;else{rx[head]=b;head=next;}
  }
}
void USART3_IRQHandler(void){}
void UART4_IRQHandler(void){native_imu_irq();}
static void io_init(void)
{
  GPIO_InitTypeDef p={0};
  __HAL_RCC_GPIOA_CLK_ENABLE();__HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();__HAL_RCC_GPIOD_CLK_ENABLE();__HAL_RCC_GPIOE_CLK_ENABLE();
  /* 先写低电平，再启用输出，避免初始化时闪亮。PE0 只接激光笔控制信号。 */
  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
  p.Pin=GPIO_PIN_0;p.Mode=GPIO_MODE_OUTPUT_PP;p.Pull=GPIO_PULLDOWN;p.Speed=GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE,&p);
  app_force_stop();p.Pin=GPIO_PIN_15;p.Mode=GPIO_MODE_OUTPUT_PP;p.Pull=GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB,&p);HAL_GPIO_WritePin(GPIOD,0x00FFU,GPIO_PIN_RESET);
  p.Pin=0x00FFU;p.Pull=GPIO_NOPULL;HAL_GPIO_Init(GPIOD,&p);
  p.Pin=GPIO_PIN_15;p.Mode=GPIO_MODE_INPUT;p.Pull=GPIO_PULLUP;HAL_GPIO_Init(GPIOA,&p);
  p.Pin=GPIO_PIN_6;HAL_GPIO_Init(GPIOE,&p);
  p.Pin=GPIO_PIN_13;p.Mode=GPIO_MODE_OUTPUT_PP;p.Pull=GPIO_NOPULL;HAL_GPIO_Init(GPIOC,&p);
}
static void dap_init(void)
{
  __HAL_RCC_USART1_CLK_ENABLE();GPIO_InitTypeDef p={0};
  p.Pin=GPIO_PIN_9|GPIO_PIN_10;p.Mode=GPIO_MODE_AF_PP;p.Pull=GPIO_PULLUP;
  p.Speed=GPIO_SPEED_FREQ_HIGH;p.Alternate=GPIO_AF7_USART1;HAL_GPIO_Init(GPIOA,&p);
  dap.Instance=USART1;dap.Init.BaudRate=115200U;dap.Init.WordLength=UART_WORDLENGTH_8B;
  dap.Init.StopBits=UART_STOPBITS_1;dap.Init.Parity=UART_PARITY_NONE;
  dap.Init.Mode=UART_MODE_TX_RX;dap.Init.HwFlowCtl=UART_HWCONTROL_NONE;dap.Init.OverSampling=UART_OVERSAMPLING_16;
  if(HAL_UART_Init(&dap)!=HAL_OK)Error_Handler();
  SET_BIT(USART1->CR1,USART_CR1_RXNEIE);HAL_NVIC_SetPriority(USART1_IRQn,7U,0U);HAL_NVIC_EnableIRQ(USART1_IRQn);
}
static void read_inputs(uint32_t now)
{
  if(remote_stop || rx_fault){
    remote_stop=rx_fault=0;tail=head;memset(&command,0,sizeof(command));
    menu_app_stop(&app,"DAP_STOP_OR_ERROR",now);return;
  }
  uint8_t active=app.phase!=APP_MENU;
  KeyEvent events[2];
  for(uint8_t i=0;i<2;i++){
    uint8_t high=HAL_GPIO_ReadPin(i?GPIOE:GPIOA,i?GPIO_PIN_6:GPIO_PIN_15)==GPIO_PIN_SET;
    events[i]=menu_key_tick(&keys[i],i,high,active,now);
  }
  if(events[0]==KEY_STOP || events[1]==KEY_STOP){menu_app_stop(&app,"KEY_STOP",now);return;}
  /* 同时松开时，确认优先；不会先切到另一题再意外启动。 */
  if(events[1]==KEY_START)menu_app_start(&app,now);
  else if(events[0]==KEY_NEXT)menu_app_next(&app);
  while(tail!=head){
    uint8_t b=rx[tail];tail=(tail+1U)&127U;
    uint8_t event=menu_command_feed(&command,b);
    if(event==1)menu_app_start(&app,HAL_GetTick());
    else if(event==2)menu_app_next(&app);
  }
}
static void draw(void)
{
  if(app.phase==APP_RUNNING || !OLED_Is_Ready())return;
  if(app.phase==APP_MENU){
    ui.selected=app.selected;ui.view=MENU_VIEW;MenuUi_Draw(&ui,HAL_GetTick());return;
  }
  OLED_Clear();OLED_Draw_String(0,0,"TASK");OLED_Draw_String(0,6,menu_app_label(app.selected));
  const char *title=app.phase==APP_POSE?"SERVO":app.phase==APP_VISION?"WAIT":
    app.phase==APP_FINISH?(app.success?"DONE":!strcmp(app.message,"RESERVED")?"RESERVED":"STOP"):"START";
  uint8_t scale=strlen(title)>5?2:3;
  OLED_Draw_Big_String((128-(int16_t)(strlen(title)*6U*scale-scale))/2,17,scale,title);
  if(app.phase==APP_FINISH)OLED_Draw_String(6,0,app.message);
  else if(app.result.kind==MV_ROMAN){
    char line[25];snprintf(line,sizeof(line),"NUMBER %u TO %c",app.result.number,app.result.target);OLED_Draw_String(6,0,line);
  }else if(app.result.kind==MV_SHAPE){
    OLED_Draw_String(5,0,app.result.color);OLED_Draw_String(6,0,app.result.shape);OLED_Draw_String(7,0,app.result.action);
  }
  OLED_Refresh();
}
int main(void)
{
  HAL_Init();io_init();route_clock_init();
  CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CYCCNT=0;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
  dap_init();(void)gimbal_pwm_init();
  MenuUi_Init(&ui);menu_app_init(&app);(void)OLED_Init();
  app_log("READY,ALL_TASK_MENU_PB15,BOOT_STOPPED,PA15=next,PE6=start,X=stop\r\n");
  uint32_t retry=0,report=0;
  while(1){
    uint32_t now=HAL_GetTick();read_inputs(now);menu_app_tick(&app,HAL_GetTick());now=HAL_GetTick();
    uint8_t laser_on=menu_app_laser_on(&app);
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,laser_on?GPIO_PIN_SET:GPIO_PIN_RESET);
    if(app.phase==APP_MENU && !OLED_Is_Ready() && now-retry>=1000U){retry=now;(void)OLED_Init();app.dirty=1;}
    if(app.dirty && app.phase!=APP_RUNNING){app.dirty=0;draw();}
    unified_menu_status[0]=0x4D454E55;unified_menu_status[1]=app.selected;unified_menu_status[2]=app.phase;
    unified_menu_status[3]=app.success;unified_menu_status[4]=OLED_Is_Ready();
    unified_menu_status[5]=app.vision.phase;unified_menu_status[6]=vision_rx_bytes;unified_menu_status[7]=vision_tx_bytes;
    unified_menu_status[8]=laser_on;unified_menu_status[9]=ALL_MENU_COUNT;
    if(now-report>=1000U){
      report=now;char line[100];snprintf(line,sizeof(line),"MENU,STATUS,%s,phase=%u,oled=%u,vision=%u\r\n",
        menu_app_label(app.selected),app.phase,OLED_Is_Ready(),app.vision.phase);app_log(line);
    }
    if(app.phase!=APP_RUNNING){app_force_stop();HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,(now/500U)&1U?GPIO_PIN_SET:GPIO_PIN_RESET);}
    HAL_Delay(1U);
  }
}
