#include "../common/entry_plan.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef struct {int16_t count;} Timer;
static Timer enc_a,enc_b,enc_c,enc_d;
static uint8_t pivot_active,fake_key=1U,reference_run_used;
static uint16_t pivot_start_yaw,imu_yaw_x100;
static uint32_t pivot_last_stop_ms,pivot_start_ms,imu_last_ms,imu_frames,fake_now;
static int16_t pivot_encoder_start[4],left_command,right_command;
static char last_log[256];
static unsigned start_calls;
#define GPIOA 0
#define GPIO_PIN_15 15
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
static uint32_t HAL_GetTick(void){return fake_now;}
static int HAL_GPIO_ReadPin(int a,int b){(void)a;(void)b;return fake_key;}
static uint32_t sample_age_ms(uint32_t now,uint32_t stamp){return now-stamp;}
static int16_t enc_count(Timer*t){return t->count;}
static int16_t clamp(int16_t v,int16_t l){return v>l?l:v< -l?-l:v;}
static void stop_motors(void){left_command=right_command=0;}
static void drive(int16_t a,int16_t b){left_command=a;right_command=b;}
static void send_text(const char*s){snprintf(last_log,sizeof(last_log),"%s",s);}
#include "../common/entry_motion.inc"
enum {STOPPED,FOLLOW};
static int state;
static uint8_t auto_phase,entry_confirmed,turns,parking_events;
static uint32_t auto_pause_start_ms;
static void stop_run(const char*s){(void)s;state=STOPPED;stop_motors();}
static void start_run(void){if(!reference_run_used){start_calls++;reference_run_used=1;auto_phase=1;state=FOLLOW;}}
struct FakeUart {uint32_t SR,DR;};
static struct FakeUart fake_uart;
#define USART1 (&fake_uart)
#define USART_SR_PE 1U
#define USART_SR_FE 2U
#define USART_SR_NE 4U
#define USART_SR_ORE 8U
#define USART_SR_RXNE 32U
#include "../common/entry_workflow.inc"
static void reset(void){
  reference_next=reference_running=reference_phase=reference_halted=0;
  pivot_active=reference_run_used=0;pivot_last_stop_ms=0;
  fake_now=imu_last_ms=1000;imu_frames=1;imu_yaw_x100=10000;fake_key=1;
  enc_a.count=enc_b.count=enc_c.count=enc_d.count=0;
  state=STOPPED;auto_phase=entry_confirmed=turns=parking_events=0;
  rx_head=rx_tail=0;start_calls=0;last_log[0]=0;stop_motors();
  (void)combined_start_feed(0);
}
static void endpoint(uint8_t i,int32_t offset){
  const ReferenceStep*s=&reference_steps[i];int32_t actual=s->target+offset;
  if(s->direction=='L')imu_yaw_x100=(pivot_start_yaw+36000-actual)%36000;
  else if(s->direction=='R')imu_yaw_x100=(pivot_start_yaw+actual)%36000;
  else{
    int sign=s->direction=='F'?1:-1;
    enc_a.count=pivot_encoder_start[0]-sign*actual;
    enc_b.count=pivot_encoder_start[1]-sign*actual;
    enc_c.count=pivot_encoder_start[2]+sign*actual;
  }
}
static void serial(const char*s){while(*s){fake_uart.SR=USART_SR_RXNE;fake_uart.DR=(uint8_t)*s++;USART1_IRQHandler();}entry_read_dap();}
int main(void){
  assert(reference_yaw_delta(35900,100)==200);
  assert(reference_yaw_delta(100,35900)==-200);
  assert(reference_encoder_progress(-100,-101,99,1)==100);
  reset();serial("GgTtQ\n");assert(start_calls==0 && !pivot_active);
  serial("@TASK4:ZZ:RUN\n");assert(start_calls==0);
  serial(ENTRY_START_FRAME "\n");assert(start_calls==1 && reference_run_used);
  serial(ENTRY_START_FRAME "\n");assert(start_calls==1);
  serial("X");assert(auto_phase==4 && !pivot_active && !left_command);
  reset();serial(ENTRY_START_FRAME);assert(start_calls==0);serial("\n");assert(start_calls==1);
  reset();serial("@TASK4:BC:\x01RUN\n");assert(start_calls==0);
  /* The workflow independently rejects a wrong target before any entry step. */
  reset();auto_phase=2;entry_confirmed=0;fake_now=ENTRY_WAIT_MS+1000;combined_tick(fake_now);
  assert(auto_phase==4 && reference_halted && !pivot_active);
  /* Correct target, then complete all actual motion steps. */
  reset();auto_phase=2;entry_confirmed=1;turns=ENTRY_REQUIRED_CORNERS;parking_events=ENTRY_PASS_LINES;
  fake_now=imu_last_ms=ENTRY_WAIT_MS+1000;combined_tick(fake_now);
  assert(pivot_active && reference_next==1);
  for(uint8_t i=0;i<ENTRY_STEP_COUNT;i++){
    assert(reference_running==i && pivot_active);
    reference_control(fake_now);
    reference_phase=1;reference_phase_ms=fake_now;endpoint(i,0);
    fake_now+=200;imu_last_ms=fake_now;reference_control(fake_now);
    assert(!pivot_active && !reference_halted && strstr(last_log,",DONE,"));
    fake_now+=150;imu_last_ms=fake_now;combined_tick(fake_now);
  }
  assert(auto_phase==4 && !pivot_active && !left_command && strstr(last_log,"TASK4_ENTRY_DONE"));
  reference_report(fake_now);assert(strstr(last_log,"REF,"));
  /* Accepted offsets are distinct from precision; larger errors stop. */
  for(uint8_t i=0;i<ENTRY_STEP_COUNT;i++){
    int linear=reference_steps[i].direction=='F'||reference_steps[i].direction=='B';
    int limit=linear?40:(reference_steps[i].target<1500?100:250);
    for(int extra=0;extra<2;extra++){
      reset();reference_next=i;reference_start(fake_now);
      reference_phase=1;reference_phase_ms=fake_now;endpoint(i,limit+extra);
      fake_now+=200;imu_last_ms=fake_now;reference_control(fake_now);
      assert(!pivot_active && reference_halted==extra);
      assert(strstr(last_log,extra?",END_ERROR,":",DONE_OFFSET,"));
      if(extra){auto_phase=3;combined_tick(fake_now);assert(auto_phase==4&&!left_command);}
    }
  }
  reset();reference_start(fake_now);fake_key=0;reference_control(fake_now);assert(reference_halted && strstr(last_log,",KEY,"));
  reset();reference_start(fake_now);imu_frames=0;reference_control(fake_now);assert(reference_halted && strstr(last_log,",IMU_STALE,"));
  reset();reference_start(fake_now);fake_now+=21001;imu_last_ms=fake_now;reference_control(fake_now);assert(reference_halted && strstr(last_log,",TIMEOUT,"));
  reset();reference_start(fake_now);fake_now+=800;imu_last_ms=fake_now;reference_control(fake_now);assert(reference_halted && !left_command);
  /* Valid error/overflow flags abort instead of becoming a movement command. */
  reset();auto_phase=1;state=FOLLOW;fake_uart.SR=USART_SR_FE|USART_SR_RXNE;fake_uart.DR='G';USART1_IRQHandler();entry_read_dap();
  assert(auto_phase==4 && !left_command);
  reset();auto_phase=1;state=FOLLOW;for(int i=0;i<140;i++){fake_uart.SR=USART_SR_RXNE;fake_uart.DR='Q';USART1_IRQHandler();}entry_read_dap();
  assert(auto_phase==4 && !left_command);
  /* Power-up held KEY does not launch on release; a later full press does. */
  reset();fake_key=0;entry_read_key(1000);entry_read_key(1040);
  fake_key=1;entry_read_key(1050);entry_read_key(1080);assert(start_calls==0);
  fake_key=0;entry_read_key(1090);entry_read_key(1120);
  fake_key=1;entry_read_key(1130);entry_read_key(1160);assert(start_calls==1);
  fake_key=0;entry_read_key(1170);entry_read_key(1200);assert(auto_phase==4);
  fake_key=1;entry_read_key(1210);entry_read_key(1240);assert(start_calls==1);
  printf("PASS MOTION/WORKFLOW case=%d: %u steps, full-frame single start, error stops, endpoint bounds, wrap and no endpoint pulses\n",TASK4_CASE,ENTRY_STEP_COUNT);
  return 0;
}
