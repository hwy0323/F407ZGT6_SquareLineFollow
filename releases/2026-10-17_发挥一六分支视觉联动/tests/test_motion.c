#include "../entry_plan.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
Task4Plan parking_plan;
typedef struct {int16_t count;} Timer;
static Timer enc_a,enc_b,enc_c,enc_d;
static uint8_t pivot_active,fake_key=1U,reference_run_used;
static uint16_t pivot_start_yaw,imu_yaw_x100;
static uint32_t pivot_last_stop_ms,pivot_start_ms,imu_last_ms,imu_frames,fake_now;
static int16_t pivot_encoder_start[4],left_command,right_command;
static char last_log[256];
#define GPIOA 0
#define GPIO_PIN_15 15
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
static uint32_t HAL_GetTick(void){return fake_now;}
static int HAL_GPIO_ReadPin(int a,int b){(void)a;(void)b;return fake_key;}
static uint32_t sample_age_ms(uint32_t now,uint32_t stamp){return now-stamp;}
static int16_t enc_count(Timer *timer){return timer->count;}
static int16_t clamp(int16_t value,int16_t limit){return value>limit?limit:value< -limit?-limit:value;}
static void stop_motors(void){left_command=right_command=0;}
static void drive(int16_t left,int16_t right){left_command=left;right_command=right;}
static void send_text(const char *text){snprintf(last_log,sizeof(last_log),"%s",text);}
#include "../common/entry_motion.inc"
enum {STOPPED,FOLLOW};
static int state;
static uint8_t auto_phase,entry_confirmed,turns,parking_events,entry_finished,parking_lines_on_edge[4];
static uint32_t auto_pause_start_ms;
static void stop_run(const char *reason){(void)reason;state=STOPPED;stop_motors();}
#include "../common/entry_workflow.inc"
static void reset(void){
 reference_next=reference_running=reference_phase=reference_halted=0;
 pivot_active=reference_run_used=0;pivot_last_stop_ms=0;
 fake_now=imu_last_ms=1000;imu_frames=1;imu_yaw_x100=10000;fake_key=1;
 enc_a.count=enc_b.count=enc_c.count=enc_d.count=0;
 state=STOPPED;auto_phase=entry_confirmed=turns=parking_events=entry_finished=0;
 memset(parking_lines_on_edge,0,sizeof(parking_lines_on_edge));last_log[0]=0;stop_motors();
}
static void endpoint(uint8_t index,int32_t offset){
 const ReferenceStep *step=&reference_steps[index];int32_t actual=step->target+offset;
 if(step->direction=='L')imu_yaw_x100=(pivot_start_yaw+36000-actual)%36000;
 else if(step->direction=='R')imu_yaw_x100=(pivot_start_yaw+actual)%36000;
 else{
  int sign=step->direction=='F'?1:-1;
  enc_a.count=pivot_encoder_start[0]-sign*actual;
  enc_b.count=pivot_encoder_start[1]-sign*actual;
  enc_c.count=pivot_encoder_start[2]+sign*actual;
 }
}
int main(void){
 const char *shapes[]={"CIRCLE","TRIANGLE","SQUARE"};
 assert(reference_yaw_delta(35900,100)==200 && reference_yaw_delta(100,35900)==-200);
 assert(reference_encoder_progress(-100,-101,99,1)==100);
 for(uint8_t branch=0;branch<6;branch++){
  assert(task4_make_plan(branch&1?"YELLOW":"RED",shapes[branch/2],branch&1?"SIDE":"REVERSE",&parking_plan));
  reset();auto_phase=2;entry_confirmed=1;turns=ENTRY_REQUIRED_CORNERS;parking_events=ENTRY_PASS_LINES;
  for(uint8_t edge=1;edge<ENTRY_REQUIRED_CORNERS;edge++)parking_lines_on_edge[edge]=2;
  fake_now=imu_last_ms=ENTRY_WAIT_MS+1000;combined_tick(fake_now);
  assert(pivot_active && reference_next==1);
  for(uint8_t i=0;i<ENTRY_STEP_COUNT;i++){
   assert(reference_running==i && pivot_active);reference_control(fake_now);
   reference_phase=1;reference_phase_ms=fake_now;endpoint(i,0);
   fake_now+=200;imu_last_ms=fake_now;reference_control(fake_now);
   assert(!pivot_active && !reference_halted && strstr(last_log,",DONE,"));
   fake_now+=150;imu_last_ms=fake_now;combined_tick(fake_now);
  }
  assert(auto_phase==4 && entry_finished && !left_command && !right_command);
  assert(strstr(last_log,"TASK4_ENTRY_DONE") && strstr(last_log,ENTRY_EDGE));
  reference_report(fake_now);assert(strstr(last_log,"REF,"));
  for(uint8_t i=0;i<ENTRY_STEP_COUNT;i++){
   int linear=reference_steps[i].direction=='F'||reference_steps[i].direction=='B';
   int limit=linear?40:(reference_steps[i].target<1500?100:250);
   for(int extra=0;extra<2;extra++){
    reset();reference_next=i;reference_start(fake_now);
    reference_phase=1;reference_phase_ms=fake_now;endpoint(i,limit+extra);
    fake_now+=200;imu_last_ms=fake_now;reference_control(fake_now);
    assert(!pivot_active && reference_halted==extra);
    assert(strstr(last_log,extra?",END_ERROR,":",DONE_OFFSET,"));
   }
  }
  reset();auto_phase=2;entry_confirmed=1;turns=ENTRY_REQUIRED_CORNERS;parking_events=ENTRY_PASS_LINES;
  fake_now=ENTRY_WAIT_MS+1000;combined_tick(fake_now);
  if(ENTRY_REQUIRED_CORNERS>1)assert(auto_phase==4 && !pivot_active && !entry_finished);
  reset();auto_phase=2;entry_confirmed=1;turns=ENTRY_REQUIRED_CORNERS+1;combined_tick(ENTRY_WAIT_MS+1000);
  assert(auto_phase==4 && !pivot_active);
  reset();reference_start(fake_now);fake_key=0;reference_control(fake_now);
  assert(reference_halted && strstr(last_log,",KEY,"));
  reset();reference_start(fake_now);imu_frames=0;reference_control(fake_now);
  assert(reference_halted && strstr(last_log,",IMU_STALE,"));
  reset();reference_start(fake_now);fake_now+=21001;imu_last_ms=fake_now;reference_control(fake_now);
  assert(reference_halted && strstr(last_log,",TIMEOUT,"));
  reset();reference_start(fake_now);fake_now+=800;imu_last_ms=fake_now;reference_control(fake_now);
  assert(reference_halted && !left_command);
 }
 puts("PASS MOTION: all six branch workflows, 3/5-step selection, exact endpoints and accepted-offset boundaries, independent edge/line-count guard, key/stale/stall/time-limit stops");
 return 0;
}
