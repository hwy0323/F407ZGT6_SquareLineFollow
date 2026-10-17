#include <assert.h>
#include <stdio.h>
#include <string.h>
#define TASK4_HOST_TEST
#define main firmware_main
#include "../main.c"
#undef main
GPIO_TypeDef fake_a,fake_b,fake_c;
FakeUart fake_uart={USART_SR_TXE,0};
FakeCore fake_core;
FakeDwt fake_dwt;
uint32_t fake_now;
uint8_t fake_key=1U;
volatile uint32_t vision_rx_bytes,vision_rx_errors,vision_rx_overflows,vision_tx_bytes;
volatile uint32_t gimbal_status[8];
static uint8_t fake_ready,fake_busy,pose_ok,pose_stuck,uart_fault;
static uint32_t pose_ms;
static unsigned starts,stops,pose_calls,sent_count;
static uint8_t pose_task;
static char sent[128][MV_MAX_FRAME+1U];
static Task4Plan fake_plan;
void parking_clock_init(void){}
void parking_init(void){}
void parking_tick(void){}
void parking_imu_irq(void){}
uint8_t parking_configure(const char *color,const char *shape,const char *action){return task4_make_plan(color,shape,action,&fake_plan);}
uint8_t parking_preflight(void){return fake_ready;}
uint8_t parking_busy(void){return fake_busy;}
uint8_t parking_start(void){starts++;fake_busy=1;return 1;}
void parking_stop(void){stops++;fake_busy=0;}
uint8_t parking_done(void){return 1;}
const Task4Plan *parking_current_plan(void){return &fake_plan;}
void vision_uart6_init(void){}
void vision_uart6_reset(void){uart_fault=0U;}
uint8_t vision_uart6_fault(void){return uart_fault;}
void vision_uart6_poll(MvClient *client,uint32_t now){(void)client;(void)now;}
uint8_t vision_uart6_send(const char *data,uint16_t length){
 assert(sent_count<128 && length<=MV_MAX_FRAME);
 memcpy(sent[sent_count],data,length);sent[sent_count++][length]=0;return 1U;
}
uint8_t gimbal_pwm_init(void){return 1U;}
uint8_t gimbal_begin_task(uint8_t task,uint32_t now){pose_calls++;pose_task=task;pose_ms=now;return pose_ok;}
uint8_t gimbal_is_settled(uint32_t now){return !pose_stuck && now-pose_ms>=1000U;}
static void tick(uint32_t now){fake_now=now;app_tick(now);}
static void feed(const char *body,uint32_t now){
 char wire[193];uint8_t sum=0;
 for(const char*p=body;*p;p++)sum^=(uint8_t)*p;
 int n=snprintf(wire,sizeof(wire),"$%s*%02X\r\n",body,sum);assert(n>0 && n<=192);
 fake_now=now;
 for(int i=0;i<n;i++)mv_feed(&vision,(uint8_t)wire[i],now);
}
static void sync_peer(void){
 memset(&vision,0,sizeof(vision));memset(&start_command,0,sizeof(start_command));
 memset((void*)&vision_last_result,0,sizeof(vision_last_result));
 mission=IDLE;route_used=remote_start=remote_stop=dap_fault=0;memset(&fake_plan,0,sizeof(fake_plan));
 dap_head=dap_tail=0;fake_key=1;fake_ready=pose_ok=1;fake_busy=pose_stuck=uart_fault=0;
 starts=stops=pose_calls=sent_count=0;preflight_tracking=0;
 fake_now=0;mv_init(&vision,vision_uart6_send,0);tick(1);
 feed("MV,1,READY,1234ABCD,0,IDLE,41,0,0",10);
 feed("MV,1,ACK,1234ABCD,42,0,IDLE",20);
 assert(vision.phase==MV_READY && mission==IDLE);
}
static void recognize(void){
 request(100);assert(mission==POSITIONING && pose_calls==1 && pose_task==4);
 tick(1099);assert(mission==POSITIONING && vision.task==0);
 tick(1100);assert(mission==IDENTIFYING && vision.task==4 && vision.seq==43);
}
static void card(const char *color,const char *shape,const char *action,uint32_t now){
 char body[160];snprintf(body,sizeof(body),"MV,1,RESULT,1234ABCD,43,4,SHAPE,%s,%s,%s,93",color,shape,action);
 feed(body,now);tick(now);
}
static void dap_bytes(const char *text){
 for(;*text;text++){fake_uart.SR=USART_SR_TXE|USART_SR_RXNE;fake_uart.DR=(uint8_t)*text;USART1_IRQHandler();}
 fake_uart.SR=USART_SR_TXE;
}
int main(void){
 const char *shapes[]={"CIRCLE","TRIANGLE","SQUARE"};
 for(unsigned i=0;i<6;i++){
  const char *color=(i&1)?"YELLOW":"RED";const char *action=(i&1)?"SIDE":"REVERSE";
  sync_peer();recognize();card(color,shapes[i/2],action,1200);
  assert(mission==PREFLIGHT && fake_plan.branch_id==i && starts==0);
  tick(1399);assert(starts==0);tick(1400);assert(starts==1 && mission==DRIVING);
  card(color,shapes[i/2],action,1410);assert(starts==1);
  fake_busy=0;tick(1420);assert(mission==FINISHED && route_used);
  request(1500);assert(mission==FINISHED && starts==1);
 }
 sync_peer();dap_bytes("GgT\n@TASK4:BC:RUN\n");tick(100);assert(mission==IDLE && !starts && !pose_calls);
 dap_bytes(TASK4_START_FRAME);tick(110);assert(mission==IDLE);
 dap_bytes("\n");tick(120);assert(mission==POSITIONING && pose_calls==1);
 remote_stop=1;tick(130);assert(mission==IDLE);tick(1200);assert(!starts && mission==IDLE);
 sync_peer();recognize();feed("MV,1,FAIL,1234ABCD,43,4,TIMEOUT",1200);tick(1200);assert(mission==IDLE && !starts);
 sync_peer();recognize();card("RED","CIRCLE","SIDE",1200);assert(mission==IDENTIFYING && !starts);
 sync_peer();recognize();card("RED","CIRCLE","REVERSE",1200);fake_ready=0;tick(4200);assert(mission==IDLE && !starts);
 sync_peer();recognize();remote_stop=1;tick(1150);card("RED","CIRCLE","REVERSE",1200);assert(mission==IDLE && !starts);
 sync_peer();pose_ok=0;request(100);assert(mission==IDLE && !starts && vision.task==0);
 sync_peer();pose_stuck=1;request(100);feed("MV,1,PONG,1234ABCD,0",5000);tick(5101);assert(mission==IDLE && !starts);
 sync_peer();recognize();card("YELLOW","SQUARE","SIDE",1200);tick(1400);assert(mission==DRIVING);
 uart_fault=1;tick(1410);assert(mission==FINISHED && !fake_busy && starts==1);
 sync_peer();recognize();card("RED","TRIANGLE","REVERSE",1200);tick(1400);assert(mission==DRIVING);
 feed("MV,1,READY,87654321,0,IDLE,0,0,0",1410);tick(1410);assert(mission==FINISHED && !fake_busy);
 sync_peer();recognize();card("RED","CIRCLE","REVERSE",1200);tick(1400);dap_fault=1;tick(1410);assert(mission==FINISHED && !fake_busy);
 puts("PASS TASK4 APP: all six cards, pose 4 before SET4, 200ms A-line preflight, full-frame start, duplicate/cancel/failure/link/session/UART stops, once-per-reset");
 return 0;
}
