#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../common/map_features.h"
#include "../entry_plan.h"
Task4Plan parking_plan;
#define PWM_PERIOD 8399U
#define SENSOR_SETTLE_US 50U
#define SEGMENT_RECORD_MODE 0U
#define REVERSE_RECORD_MODE 0U
#define FORWARD_REVERSE_REFERENCE_MODE 0U
#define S_CURVE_RECORD_MODE 0U
#define FORWARD_REFERENCE_MAX_RUN_MS 12000U
#define S_CURVE_SPEED 155
#define S_CURVE_KP 20
#define S_CURVE_MAX_CORRECTION 100
#define S_CURVE_LOST_SPEED 85
#define REVERSE_LEFT_SPEED 120
#define REVERSE_RIGHT_SPEED 185
#define REVERSE_LOST_SPEED 110
#define REVERSE_LOST_STOP_MS 300U
#define REVERSE_MAX_RUN_MS 8000U
#define REVERSE_MAX_COUNTS 4000
#define REVERSE_MAX_YAW_X100 5500U
#define SEGMENT_LOSS_CONFIRM_MS 500U
#define SEGMENT_LOSS_MAX_YAW_X100 2000U
#define DRIVE_SPEED 200
#define CORNER_SPEED 170
#define PIVOT_SPEED 220
#define LOST_SPEED 120
#define LOST_HOLD_MS 60U
#define LOST_STRAIGHT_MS 220U
#define LOST_SWEEP_SWITCH_MS 420U
#define LOST_SWEEP_SPEED 105
#define LOST_SWEEP_BIAS 55
#define LOST_MAX_YAW_X100 1200U
#define LOST_MAX_ENCODER_COUNTS 500
#define KP 14
#define MAX_CORRECTION 105
#define CORNER_CONFIRM_MS 30U
#define CORNER_EDGE_WINDOW_MS 150U
#define CORNER_BRAKE_MS 120U
#define CORNER_COOLDOWN_MS 300U
#define TURN_MIN_ANGLE_X100 3000U
#define TURN_MAX_ANGLE_X100 12500U
#define TURN_RECOVER_MS 100U
#define CONTROL_MS 10U
#define REPORT_MS 50U
#define SENSOR_TIMEOUT_MS 200U
#define LOST_GRACE_MS 650U
#define TURN_TIMEOUT_MS 1500U
#define MAX_RUN_MS 45000U
#define CORNER_TARGET_COUNT 4U
#define CORNER_POSITION_WINDOW 900L
#define PARKING_PAUSE_MS 100U
#define PARKING_PASS_SPEED 140
#define PARKING_PASS_MIN_COUNTS 120L
#define PARKING_PASS_MAX_COUNTS 700L
#define PARKING_PASS_MAX_MS 1700U
#define PARKING_PASS_MAX_YAW_X100 1500U
#define PARKING_PASS_LINE_LOST_MS 120U
#define PARKING_EVENT_LIMIT 6U
#define ONE_CORNER_APPROACH_TIMEOUT_MS 6000U
typedef enum {STOPPED,FOLLOW,ENTER_CORNER,PIVOT,RECOVER,WAIT_FINISH_KEY,PARKING_PAUSE,PARKING_PASS} State;
typedef enum {NO_TURN,LEFT_TURN,RIGHT_TURN} Direction;
static State state;
static Direction turn_direction,corner_candidate;
static uint8_t turns,center_hits,git_corner_hits,parking_events,parking_clean_hits,line_lost;
static uint8_t auto_phase,entry_confirmed,parking_feature_hits;
static uint8_t parking_lines_on_edge[4];
static uint32_t run_start_ms,state_start_ms,last_turn_complete_ms,last_encoder_progress_ms,line_lost_ms,parking_line_lost_ms,auto_pause_start_ms;
static int32_t parking_pass_start_count,parking_corner_progress,odom_total[4];
static uint16_t parking_pass_start_yaw_x100,imu_yaw_x100,reverse_start_yaw_x100;
static int16_t left_command,right_command;
static char stopped[80],events[12000];
static unsigned corner_starts,corner_dones;
static uint8_t active_count(uint8_t b){uint8_t n=0;while(b){n+=b&1;b>>=1;}return n;}
static int16_t clamp(int16_t x,int16_t l){return x>l?l:x< -l?-l:x;}
static void odom_update(void){}
static uint16_t yaw_change_x100(uint16_t start){(void)start;return 0;}
static void send_text(const char*s){assert(strlen(events)+strlen(s)<sizeof(events));strcat(events,s);}
static void stop_motors(void){left_command=right_command=0;}
static void drive(int16_t l,int16_t r){left_command=l;right_command=r;}
static void stop_run(const char *why){strcpy(stopped,why);state=STOPPED;auto_phase=4U;stop_motors();}
static void combined_abort(const char *why){stop_run(why);}
static uint8_t transit_lines_complete(void){
  if(!task4_plan_valid(&parking_plan) || parking_events!=ENTRY_PASS_LINES)return 0;
  for(uint8_t edge=1;edge<ENTRY_REQUIRED_CORNERS;edge++)if(parking_lines_on_edge[edge]!=2)return 0;
  return 1;
}
static void report(uint32_t now){(void)now;}
static void record_corner_event(const char*phase,uint8_t n,Direction d,uint32_t now){
  (void)n;(void)d;(void)now;
  if(!strcmp(phase,"START"))corner_starts++;
  if(!strcmp(phase,"DONE"))corner_dones++;
}
#include "../common/navigation.inc"
static uint32_t now;
static void reset(void){
  now=1000;run_start_ms=0;state=FOLLOW;auto_phase=1;
  turns=center_hits=git_corner_hits=parking_events=parking_clean_hits=line_lost=0;
  entry_confirmed=parking_feature_hits=0;parking_corner_progress=0;
  last_turn_complete_ms=line_lost_ms=parking_line_lost_ms=0;
  events[0]=stopped[0]=0;corner_starts=corner_dones=0;stop_motors();
  memset(parking_lines_on_edge,0,sizeof(parking_lines_on_edge));
}
static void set_progress(int32_t n){odom_total[0]=-n;odom_total[1]=-n;odom_total[2]=odom_total[3]=n;}
static void tick(uint8_t bits){now+=10;git_verified_control(now,bits);}
static void garage(int32_t position){
  set_progress(position);
  for(int i=0;i<3;i++)tick(0xFF);
  assert(state==PARKING_PAUSE && !entry_confirmed);
  unsigned count=parking_events;
  now+=PARKING_PAUSE_MS;tick(0xFF);assert(state==PARKING_PASS);
  set_progress(position+200);
  for(int i=0;i<10;i++)tick(0x18);
  assert(state==FOLLOW && parking_events==count && !stopped[0] && turns<ENTRY_REQUIRED_CORNERS);
}
static void corner(unsigned index){
  set_progress(true_corner_a_counts[index]);now+=CORNER_COOLDOWN_MS;
  for(int i=0;i<3;i++)tick(0x0F);
  assert(state==ENTER_CORNER);now+=100;tick(0x0F);assert(state==PIVOT);
  now+=120;for(int i=0;i<3;i++)tick(0x18);
  assert(state==RECOVER);now+=100;tick(0x18);
  assert(state==FOLLOW && turns==index+1);
}
static void target_context(void){
  reset();turns=ENTRY_REQUIRED_CORNERS;parking_events=ENTRY_PASS_LINES;
  for(uint8_t edge=1;edge<ENTRY_REQUIRED_CORNERS;edge++)parking_lines_on_edge[edge]=2;
  parking_corner_progress=ENTRY_LINE_CENTER-1600;last_turn_complete_ms=now-600;
}
static void complete_layout(unsigned layout,uint8_t target){
  const char *shapes[]={"CIRCLE","TRIANGLE","SQUARE"};
  uint8_t reverse=(uint8_t)((layout>>(target-1U))&1U);
  assert(task4_make_plan(reverse?"RED":"YELLOW",shapes[target-1U],reverse?"REVERSE":"SIDE",&parking_plan));
  reset();set_progress(1000);
  for(int i=0;i<10;i++)tick(0x38);
  assert(state==FOLLOW && parking_events==0 && !corner_starts);
  corner(0);
  for(uint8_t edge=1;edge<target;edge++){
    uint8_t other_type=(uint8_t)((layout>>(edge-1U))&1U);
    garage(task4_line_position(edge,other_type,0));
    garage(task4_line_position(edge,other_type,1));
    corner(edge);
  }
  set_progress(ENTRY_LINE_CENTER);now+=600;
  for(int i=0;i<3;i++)tick(0x0F);
  assert(state==STOPPED && auto_phase==2 && entry_confirmed);
  assert(parking_events==ENTRY_PASS_LINES && corner_starts==ENTRY_REQUIRED_CORNERS);
  assert(!left_command && !right_command && strstr(events,"TASK4_TARGET_LINE,") && strstr(events,ENTRY_EDGE));
}
static void safety_checks(uint8_t target,uint8_t reverse){
  const char *shapes[]={"CIRCLE","TRIANGLE","SQUARE"};
  assert(task4_make_plan(reverse?"RED":"YELLOW",shapes[target-1U],reverse?"REVERSE":"SIDE",&parking_plan));
  target_context();set_progress(ENTRY_LINE_CENTER-ENTRY_LINE_WINDOW-1);
  for(int i=0;i<3;i++)tick(0x0F);
  assert(!entry_confirmed && state==FOLLOW);
  set_progress(ENTRY_LINE_CENTER);
  for(int i=0;i<3;i++)tick(0x38);
  assert(!entry_confirmed && state==FOLLOW);
  target_context();set_progress(ENTRY_LINE_CENTER+ENTRY_LINE_WINDOW+1);tick(0xFF);
  assert(!strcmp(stopped,"TARGET_FIRST_LINE_MISSING") && !entry_confirmed);
  target_context();set_progress(ENTRY_LINE_CENTER);
  tick(0);now+=121;tick(0);
  assert(!strcmp(stopped,"TARGET_SEARCH_LINE_LOST") && !left_command);
  target_context();parking_events++;set_progress(ENTRY_LINE_CENTER);
  for(int i=0;i<3;i++)tick(0xFF);
  assert(!strcmp(stopped,"GARAGE_SEQUENCE_MISMATCH") && !entry_confirmed);
  target_context();turns++;tick(0x18);
  assert(!strcmp(stopped,"TARGET_EDGE_MISMATCH"));
  target_context();parking_plan.branch_id=255;tick(0x18);
  assert(!strcmp(stopped,"INVALID_PLAN") && !left_command);
}
int main(void){
  for(unsigned layout=0;layout<8;layout++)for(uint8_t target=1;target<=3;target++)complete_layout(layout,target);
  for(uint8_t target=1;target<=3;target++)for(uint8_t reverse=0;reverse<2;reverse++)safety_checks(target,reverse);
  assert(task4_make_plan("RED","SQUARE","REVERSE",&parking_plan));
  reset();corner(0);garage(task4_line_position(1,1,0));
  set_progress(true_corner_a_counts[1]);now+=CORNER_COOLDOWN_MS;tick(0x0F);
  assert(!strcmp(stopped,"TRANSIT_GARAGE_LINES_MISSING"));
  reset();corner(0);garage(task4_line_position(1,0,0));garage(task4_line_position(1,0,1));
  set_progress(10000);for(unsigned i=0;i<3;i++)tick(0xFF);
  assert(!strcmp(stopped,"UNEXPECTED_EXTRA_GARAGE_LINE"));
  for(uint8_t edge=1;edge<=3;edge++){
    assert(task4_line_position(edge,0,0)<task4_line_position(edge,1,0));
    assert(task4_line_position(edge,0,1)>task4_line_position(edge,1,1));
  }
  memset(&parking_plan,0xA5,sizeof(parking_plan));
  assert(!task4_make_plan("RED","CIRCLE","SIDE",&parking_plan) && !parking_plan.corners);
  assert(!task4_make_plan("BLUE","CIRCLE","REVERSE",&parking_plan));
  assert(!task4_make_plan("RED","HEXAGON","REVERSE",&parking_plan));
  puts("PASS NAV: all 8 garage-type layouts x 3 target edges = 24 routes; six color/shape branches; size-dependent first-line windows; per-edge line counts; gap/missing/extra/corrupt-plan stops");
  return 0;
}
