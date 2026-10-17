#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../entry_plan.h"
typedef enum { STOPPED,FOLLOW,ENTER_CORNER,PIVOT,RECOVER } State;
typedef enum { NO_TURN,LEFT_TURN,RIGHT_TURN } Direction;
typedef enum { IDLE,SEARCH,LINE_PAUSE,PROBE_FORWARD,MEASURE,PROBE_BACK,ENTRY,DONE,FAILED } Phase;
typedef struct {int16_t count;} Timer;
static Timer enc_a,enc_b,enc_c,enc_d;
static uint8_t gray_value,turns,center_hits,feature_hits,reference_run_used,line_lost;
static uint32_t gray_frames,gray_last_ms,imu_frames,imu_last_ms;
static uint16_t imu_yaw_x100;
static State state;static Phase auto_phase;static TrackFeature feature_candidate;
static Direction turn_direction;
static int16_t left_command,right_command;
static int32_t odom_total[4],watch_position;
static uint32_t run_start_ms,state_start_ms,last_turn_complete_ms,line_lost_ms,watch_ms,phase_ms;
static uint8_t pivot_active;static uint16_t pivot_start_yaw;
static uint32_t pivot_last_stop_ms,pivot_start_ms;
static int16_t pivot_encoder_start[4],garage_origin[3];
static uint16_t garage_yaw,range_samples[RANGE_SAMPLE_COUNT],range_median;
static GarageType garage_type;
static uint8_t range_used,range_discard;
static uint32_t range_seen,last_range_poll;
static uint32_t laser_sample_count;
static uint16_t laser_distance_mm;
static uint8_t laser_status,laser_range_status;
static uint32_t fake_now;static uint8_t fake_key=1;
static int laser_mode;static unsigned laser_reads;
static char events[30000];
#define GPIOA 0
#define GPIO_PIN_15 15
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
static uint32_t HAL_GetTick(void){return fake_now;}
static int HAL_GPIO_ReadPin(int p,int n){(void)p;(void)n;return fake_key;}
static uint32_t sample_age_ms(uint32_t n,uint32_t t){int32_t age=(int32_t)(n-t);return age<0?0U:(uint32_t)age;}
static int16_t enc_count(Timer*t){return t->count;}
static int16_t clamp(int16_t v,int16_t l){return v>l?l:v< -l?-l:v;}
static void stop_motors(void){left_command=right_command=0;}
static void drive(int16_t l,int16_t r){left_command=l;right_command=r;}
static void send_text(const char*s){assert(strlen(events)+strlen(s)<sizeof(events));strcat(events,s);}
static void odom_update(void){}
static void reset_encoders(void){enc_a.count=enc_b.count=enc_c.count=enc_d.count=0;memset(odom_total,0,sizeof(odom_total));}
static uint8_t active_count(uint8_t b){uint8_t n=0;while(b){n+=b&1;b>>=1;}return n;}
static int16_t line_error(uint8_t bits){
  if(bits==8||bits==16||bits==24)return 0;
  int8_t w[8]={7,5,3,1,-1,-3,-5,-7};int sum=0,n=0;
  for(int i=0;i<8;i++)if(bits&(1<<i)){sum+=w[i];n++;}return n?sum/n:0;
}
static void record_corner_event(const char*p,uint8_t n,Direction d,uint32_t t){(void)p;(void)n;(void)d;(void)t;}
static void LaserTest_Process(void){
  assert(!left_command && !right_command);laser_reads++;
  if(laser_mode==1){laser_status=5;return;}
  if(laser_mode==2)return; /* Frozen sample counter: do not accept stale data. */
  laser_sample_count++;
}
static void combined_abort(const char*);
#include "../entry_motion.inc"
#include "../search_workflow.inc"
#include "../navigation.inc"
struct FakeUart{uint32_t SR,DR;};static struct FakeUart uart;
#define USART1 (&uart)
#define USART_SR_PE 1U
#define USART_SR_FE 2U
#define USART_SR_NE 4U
#define USART_SR_ORE 8U
#define USART_SR_RXNE 32U
static void start_run(void);
#include "../commands.inc"
#include "../start.inc"
static void reset(void){
  auto_phase=IDLE;state=STOPPED;turns=center_hits=feature_hits=reference_run_used=line_lost=0;
  pivot_active=reference_next=reference_running=reference_phase=reference_halted=0;
  pivot_last_stop_ms=0;fake_now=gray_last_ms=imu_last_ms=1000;gray_frames=imu_frames=1;
  gray_value=0x18;imu_yaw_x100=10000;fake_key=1;laser_status=4;laser_range_status=0;
  laser_sample_count=0;laser_mode=0;laser_reads=0;events[0]=0;rx_head=rx_tail=0;
  reset_encoders();stop_motors();search_plan_reset();(void)search_start_feed(0);
}
static void advance(uint32_t dt){fake_now+=dt;imu_last_ms=gray_last_ms=fake_now;}
static void serial(const char*s){while(*s){uart.SR=USART_SR_RXNE;uart.DR=(uint8_t)*s++;USART1_IRQHandler();}search_read_dap();}
static void endpoint(int32_t offset){
  ReferenceStep s=reference_steps[reference_running];int32_t actual=s.target+offset;
  if(s.direction=='L')imu_yaw_x100=(pivot_start_yaw+36000-actual)%36000;
  else if(s.direction=='R')imu_yaw_x100=(pivot_start_yaw+actual)%36000;
  else{
    int sign=s.direction=='F'?1:-1;
    enc_a.count=pivot_encoder_start[0]-sign*actual;enc_b.count=pivot_encoder_start[1]-sign*actual;
    enc_c.count=pivot_encoder_start[2]+sign*actual;
  }
}
static void finish_motion(int32_t offset){
  assert(pivot_active);reference_phase=1;reference_phase_ms=fake_now;endpoint(offset);
  advance(200);reference_control(fake_now);assert(!pivot_active);
}
static void to_measure(unsigned edge){
  reset();start_run();assert(auto_phase==SEARCH);
  turns=edge;watch_ms=fake_now;int32_t positions[4]={1000,8000,14000,20000};
  odom_total[0]=-positions[edge];gray_value=0x0F;
  for(int i=0;i<3;i++){advance(10);search_navigation(fake_now);}
  assert(auto_phase==LINE_PAUSE && !left_command && !pivot_active);
  advance(200);search_tick(fake_now);assert(auto_phase==PROBE_FORWARD && pivot_active);
  finish_motion(0);advance(150);search_tick(fake_now);
  assert(auto_phase==MEASURE && !left_command && reference_next==1);
}
static void readings(uint16_t mm){laser_distance_mm=mm;for(int i=0;i<8;i++){advance(20);search_tick(fake_now);}}
int main(void){
  unsigned runs=0;
  /* Both bay types at arbitrary positions on every edge, not fixed midpoints. */
  for(unsigned edge=0;edge<4;edge++)for(int type=1;type<=2;type++){
    to_measure(edge);readings(type==GARAGE_SIDE?159:315);
    assert(garage_type==(GarageType)type && auto_phase==PROBE_BACK && pivot_active && laser_reads==8);
    assert(reference_steps[1].target==PROBE_COUNTS);
    finish_motion(0);search_tick(fake_now);assert(auto_phase==ENTRY);
    advance(150);search_tick(fake_now);
    while(auto_phase==ENTRY){finish_motion(0);advance(150);search_tick(fake_now);}
    assert(auto_phase==DONE && !left_command && reference_next==(type==GARAGE_SIDE?7:5));runs++;
  }
  /* Classifier boundaries must select the actual parking workflow. */
  for(unsigned mm=236;mm<=238;mm++){
    to_measure(1);readings(mm);
    assert(garage_type==(mm<=237?GARAGE_SIDE:GARAGE_REVERSE) && auto_phase==PROBE_BACK);
    assert(strstr(events,"threshold_mm=237,raw_range=1"));
  }
  /* Excess spread still fails even when the median is below the threshold. */
  to_measure(1);readings(159);assert(auto_phase==PROBE_BACK);
  to_measure(1);laser_distance_mm=159;
  for(int i=0;i<7;i++){advance(20);search_tick(fake_now);}
  laser_distance_mm=190;advance(20);search_tick(fake_now);
  assert(auto_phase==FAILED && reference_next==1 && strstr(events,"LASER_TYPE_UNKNOWN"));
  to_measure(1);laser_mode=1;advance(20);search_tick(fake_now);assert(auto_phase==FAILED && !left_command);
  to_measure(1);laser_mode=2;advance(MEASURE_TIMEOUT_MS+1);search_tick(fake_now);assert(auto_phase==FAILED);
  to_measure(1);laser_range_status=4;readings(159);assert(auto_phase==MEASURE && range_used==0);
  advance(MEASURE_TIMEOUT_MS);search_tick(fake_now);assert(auto_phase==FAILED);
  to_measure(1);readings(315);finish_motion(41);search_tick(fake_now);assert(auto_phase==FAILED);
  /* Return heading is independently checked before parking. */
  to_measure(1);readings(315);finish_motion(0);imu_yaw_x100+=201;search_tick(fake_now);assert(auto_phase==FAILED);
  /* Actual forward endpoint, rather than nominal, is reversed. */
  reset();garage_origin[0]=garage_origin[1]=garage_origin[2]=0;garage_yaw=imu_yaw_x100;
  reference_start(fake_now);finish_motion(30);auto_phase=MEASURE;phase_ms=fake_now;
  range_used=range_discard=0;range_seen=0;last_range_poll=fake_now;
  laser_distance_mm=315;for(int i=0;i<5;i++){advance(40);search_tick(fake_now);}
  assert(reference_steps[1].target==317 && auto_phase==PROBE_BACK);
  /* A true square bend remains a bend, not a bay. */
  reset();start_run();odom_total[0]=-5100;gray_value=0x0F;
  for(int i=0;i<3;i++){advance(10);search_navigation(fake_now);}assert(state==ENTER_CORNER && auto_phase==SEARCH);
  advance(100);search_navigation(fake_now);assert(state==PIVOT);
  gray_value=0x18;advance(120);for(int i=0;i<3;i++){advance(10);search_navigation(fake_now);}assert(state==RECOVER);
  advance(100);search_navigation(fake_now);assert(turns==1 && state==FOLLOW);
  reset();start_run();gray_value=0;advance(10);search_navigation(fake_now);assert(auto_phase==SEARCH);
  advance(121);search_navigation(fake_now);assert(auto_phase==FAILED && !left_command);
  reset();start_run();advance(800);search_navigation(fake_now);assert(auto_phase==FAILED);
  reset();start_run();imu_frames=0;reference_start(fake_now);assert(!pivot_active);
  reset();serial("GTT\n");assert(auto_phase==IDLE);serial(SEARCH_START_FRAME);assert(auto_phase==IDLE);
  serial("\n");assert(auto_phase==SEARCH);serial("X");assert(auto_phase==FAILED);
  serial(SEARCH_START_FRAME "\n");assert(auto_phase==FAILED);
  reset();laser_status=1;start_run();assert(auto_phase==IDLE);
  reset();imu_frames=0;start_run();assert(auto_phase==IDLE);
  reset();gray_value=0xFF;start_run();assert(auto_phase==IDLE);
  reset();fake_key=0;search_read_key(fake_now);advance(30);search_read_key(fake_now);
  fake_key=1;advance(10);search_read_key(fake_now);advance(30);search_read_key(fake_now);assert(auto_phase==IDLE);
  fake_key=0;advance(10);search_read_key(fake_now);advance(30);search_read_key(fake_now);
  fake_key=1;advance(10);search_read_key(fake_now);advance(30);search_read_key(fake_now);assert(auto_phase==SEARCH);
  fake_key=0;advance(10);search_read_key(fake_now);advance(30);search_read_key(fake_now);assert(auto_phase==FAILED);
  reset();start_run();uart.SR=USART_SR_FE|USART_SR_RXNE;uart.DR='G';USART1_IRQHandler();search_read_dap();assert(auto_phase==FAILED);
  reset();start_run();for(int i=0;i<140;i++){uart.SR=USART_SR_RXNE;uart.DR='Q';USART1_IRQHandler();}search_read_dap();assert(auto_phase==FAILED);
  reset();reference_start(fake_now);fake_key=0;reference_control(fake_now);assert(reference_halted && !left_command);
  reset();reference_start(fake_now);advance(151);imu_last_ms=0;reference_control(fake_now);assert(reference_halted);
  reset();reference_start(fake_now);advance(800);reference_control(fake_now);assert(reference_halted);
  assert(reference_yaw_delta(35900,100)==200 && reference_yaw_delta(100,35900)==-200);
  assert(reference_encoder_progress(-100,-101,99,1)==100);
  reset();reference_report(fake_now);
  reset();start_run();advance(JOB_MAX_MS+1);search_safety(fake_now);assert(auto_phase==FAILED && strstr(events,"JOB_TIMEOUT"));
  reset();start_run();imu_frames=0;search_safety(fake_now);assert(auto_phase==FAILED && strstr(events,"IMU_STALE"));
  reset();start_run();gray_last_ms=0;search_navigation(fake_now);assert(auto_phase==FAILED && strstr(events,"GRAY_STALE"));
  reset();start_run();turns=3;odom_total[0]=-23793;gray_value=0x0F;
  for(int i=0;i<3;i++){advance(10);search_navigation(fake_now);}assert(auto_phase==FAILED && strstr(events,"ONE_LAP_NO_GARAGE"));
  printf("PASS APP: %u full bay/edge workflows, probe return, valid/frozen/error ranges, corners, stops, command and KEY gates\n",runs);
}
