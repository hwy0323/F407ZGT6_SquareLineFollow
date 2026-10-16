#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../route_data.h"
typedef struct {int16_t count;} Timer;
static Timer enc_a,enc_b,enc_c;
static int16_t pivot_encoder_start[4],left_command,right_command;
static int32_t linear_target_counts=1433,fake_yaw;
static uint32_t imu_frames=1,imu_last_ms,pivot_start_ms,now_ms;
static uint8_t fake_key=1,pivot_active;
static uint8_t route_active,route_waiting,route_index,auto_phase,turns;
static uint32_t route_start_ms;
enum {STOPPED,FOLLOW}; static int state;
static char events[4096],reason[64];
#define GPIOA 0
#define GPIO_PIN_15 15
#define GPIO_PIN_RESET 0
static uint32_t HAL_GetTick(void){return now_ms;}
static int HAL_GPIO_ReadPin(int a,int b){(void)a;(void)b;return fake_key;}
static uint32_t sample_age_ms(uint32_t n,uint32_t stamp){return n-stamp;}
static int16_t enc_count(Timer *t){return t->count;}
static int16_t clamp(int16_t x,int16_t lim){return x>lim?lim:x< -lim?-lim:x;}
static int32_t pivot_delta_x100(void){return fake_yaw;}
static void stop_motors(void){left_command=right_command=0;}
static void drive(int16_t a,int16_t b){left_command=a;right_command=b;}
static void send_text(const char *s){assert(strlen(events)+strlen(s)<sizeof(events));strcat(events,s);}
static void route_abort(const char *s){snprintf(reason,sizeof(reason),"%s",s);route_active=route_waiting=0;stop_motors();}
#include "../c_route_after_step.inc"
static void pivot_stop(const char *s){snprintf(reason,sizeof(reason),"%s",s);pivot_active=0;stop_motors();route_after_step(s);}
#include "../c_final_forward.inc"
static void reset(void){
  memset(pivot_encoder_start,0,sizeof(pivot_encoder_start));
  enc_a.count=enc_b.count=enc_c.count=0;fake_yaw=0;fake_key=1;imu_frames=1;
  now_ms=imu_last_ms=pivot_start_ms=1000;linear_target_counts=1433;
  route_active=route_waiting=route_index=auto_phase=turns=0;route_start_ms=1000;
  state=STOPPED;pivot_active=1;events[0]=reason[0]=0;stop_motors();c_final_begin(now_ms);
}
static void tick(uint32_t ms,int16_t counts){
  now_ms=imu_last_ms=ms;enc_a.count=enc_b.count=-counts;enc_c.count=counts;c_final_control(ms);
}
int main(void){
  assert(ROUTE_STEP_COUNT==5);
  const char directions[5]={'F','R','F','R','F'};
  const uint16_t distances[5]={100,50,180,60,250};
  for(uint8_t i=0;i<5;i++){assert(route_steps[i].direction==directions[i]);assert(route_steps[i].value==distances[i]);}
  assert((250*573+50)/100==1433);
  reset();tick(1000,0);assert(left_command==180&&right_command==180);
  tick(1100,1000);assert(left_command==180&&right_command==180);
  tick(1200,1250);assert(c_final_stopping&&!left_command&&!right_command);
  tick(1420,1444);assert(!pivot_active&&!strcmp(reason,"DONE"));
  /* No fine pulses or automatic retry, even when the endpoint is off. */
  reset();c_final_stopping=1;c_final_stop_ms=1000;tick(1200,1473);
  assert(!pivot_active&&!strcmp(reason,"DONE_OFFSET")&&!left_command);
  reset();c_final_stopping=1;c_final_stop_ms=1000;tick(1200,1474);
  assert(!pivot_active&&!strcmp(reason,"END_ERROR")&&!left_command);
  reset();tick(1200,-101);assert(!strcmp(reason,"MOTION_LIMIT"));
  reset();tick(1200,1514);assert(!strcmp(reason,"MOTION_LIMIT"));
  reset();fake_yaw=801;tick(1200,10);assert(!strcmp(reason,"MOTION_LIMIT"));
  reset();fake_key=0;tick(1010,0);assert(!strcmp(reason,"KEY"));
  reset();imu_frames=0;tick(1010,0);assert(!strcmp(reason,"IMU_STALE"));
  reset();now_ms=1200;c_final_control(now_ms);assert(!strcmp(reason,"IMU_STALE"));
  reset();tick(9001,0);assert(!strcmp(reason,"TIMEOUT"));
  reset();tick(1800,0);assert(!strcmp(reason,"ENCODER_STALL"));
  /* Completing the fifth step stops. No phase 2 / C turn / D-A handoff. */
  reset();route_active=1;auto_phase=3;
  for(uint8_t i=0;i<4;i++){assert(route_index==i);route_after_step(i==0?"DISTANCE_OVERSHOOT":"DONE");assert(route_active&&auto_phase==3);}
  assert(route_index==4&&!strstr(events,"EXIT_FINISHED"));
  route_after_step("DONE_OFFSET");
  assert(!route_active&&!route_waiting&&auto_phase==4&&state==STOPPED&&!left_command&&!right_command);
  assert(strstr(events,"C_REACHED_FROM_BC_GARAGE"));assert(!strstr(events,"FOLLOW_START"));
  reset();route_active=1;route_index=4;auto_phase=3;route_after_step("END_ERROR");
  assert(!route_active&&!strstr(events,"C_REACHED"));
  reset();route_active=1;route_index=4;auto_phase=3;route_after_step("DISTANCE_OVERSHOOT");
  assert(!route_active&&!strstr(events,"C_REACHED"));
  puts("PASS: five-step merged table, F1433 no-pulse control, endpoint boundaries, fault stops and C-only route completion");
  return 0;
}
