#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../menu_app.h"
#include "../keys.h"
#include "../menu_command.h"
#include "../native.h"
#include "../gimbal_pwm.h"
#include "../vision_uart6.h"
#include "../../2026-10-17_发挥一六分支视觉联动/task4_plan.h"
static MenuApp app;
static uint32_t now,pose_time;
static uint8_t ready,busy,screen,pose_ok,pose_stuck,uart_fault,start_ok;
static NativeId current;
static unsigned starts,selects,force_stops,reboots,pose_calls;
static uint8_t pose_task;
static Task4Plan plan;
volatile uint32_t vision_rx_bytes,vision_rx_errors,vision_rx_overflows,vision_tx_bytes;
volatile uint32_t gimbal_status[8];
void completion_select(NativeId id);
uint8_t completion_done(void);
uint32_t app_now(void){return now;}
void app_log(const char *line){(void)line;}
void app_force_stop(void){force_stops++;}
void app_reboot(void){reboots++;}
uint8_t app_screen_ready(void){return screen;}
void native_select(NativeId id){assert(!busy);current=id;selects++;completion_select(id);}
void native_tick(void){}
void native_imu_irq(void){}
uint8_t native_ready(void){return ready;}
uint8_t native_busy(void){return busy;}
uint8_t native_done(void){return completion_done();}
uint8_t native_start(void){starts++;busy=start_ok;return start_ok;}
void native_stop(void){busy=0;}
uint8_t native_plan(const char *color,const char *shape,const char *action)
{return task4_make_plan(color,shape,action,&plan);}
void route_clock_init(void){}
void vision_uart6_init(void){}
void vision_uart6_reset(void){uart_fault=0;}
void vision_uart6_poll(MvClient *client,uint32_t time){(void)client;(void)time;}
uint8_t vision_uart6_send(const char *data,uint16_t length){(void)data;vision_tx_bytes+=length;return 1;}
uint8_t vision_uart6_fault(void){return uart_fault;}
uint8_t gimbal_pwm_init(void){return pose_ok;}
uint8_t gimbal_begin_task(uint8_t task,uint32_t time){pose_task=task;pose_time=time;pose_calls++;return pose_ok;}
uint8_t gimbal_is_settled(uint32_t time){return !pose_stuck && time-pose_time>=1000U;}
static void tick(uint32_t time){now=time;menu_app_tick(&app,now);}
static void reset(void)
{
  now=0;ready=screen=pose_ok=start_ok=1;busy=pose_stuck=uart_fault=0;
  starts=selects=force_stops=reboots=pose_calls=0;
  vision_rx_errors=vision_rx_overflows=vision_rx_bytes=vision_tx_bytes=0;
  memset(&plan,0,sizeof(plan));completion_select(NATIVE_COUNT);menu_app_init(&app);
}
static void feed(const char *body,uint32_t time)
{
  char wire[193];uint8_t sum=0;for(const char *p=body;*p;p++)sum^=(uint8_t)*p;
  int n=snprintf(wire,sizeof(wire),"$%s*%02X\r\n",body,sum);assert(n>0 && n<=192);
  now=time;for(int i=0;i<n;i++)mv_feed(&app.vision,(uint8_t)wire[i],time);
}
static void begin_visual(uint8_t selected)
{
  reset();app.selected=selected;now=100;menu_app_start(&app,now);
  assert(app.phase==APP_POSE && pose_task==(selected==4U?3U:4U) && starts==0);
  feed("MV,1,READY,1234ABCD,0,IDLE,41,0,0",110);
  feed("MV,1,ACK,1234ABCD,42,0,IDLE",120);
  tick(1099);assert(app.phase==APP_POSE && app.vision.task==0 && !busy);
  tick(1100);assert(app.phase==APP_VISION && app.vision.seq==43 && starts==0);
}
static void result_roman(unsigned index)
{
  const char *symbols[]={"I","III","V","VII"};char body[160];
  snprintf(body,sizeof(body),"MV,1,RESULT,1234ABCD,43,3,ROMAN,%s,%u,%c,93",symbols[index],1+index*2,'A'+index);
  feed(body,1200);tick(1200);
}
static void result_card(unsigned index)
{
  const char *shapes[]={"CIRCLE","TRIANGLE","SQUARE"};char body[160];
  snprintf(body,sizeof(body),"MV,1,RESULT,1234ABCD,43,4,SHAPE,%s,%s,%s,93",
    index&1?"YELLOW":"RED",shapes[index/2],index&1?"SIDE":"REVERSE");
  feed(body,1200);tick(1200);
}
static void check_keys(void)
{
  MenuKey key={0};
  assert(menu_key_tick(&key,0,0,0,0)==KEY_NONE);
  assert(menu_key_tick(&key,0,0,0,2000)==KEY_NONE); /* held at boot */
  (void)menu_key_tick(&key,0,1,0,2001);(void)menu_key_tick(&key,0,1,0,2032);
  (void)menu_key_tick(&key,0,0,0,2100);(void)menu_key_tick(&key,0,0,0,2131);
  (void)menu_key_tick(&key,0,1,0,2200);
  assert(menu_key_tick(&key,0,1,0,2231)==KEY_NEXT);
  /* PA15 held for seconds still only selects, never starts. */
  (void)menu_key_tick(&key,0,0,0,2300);(void)menu_key_tick(&key,0,0,0,2331);
  (void)menu_key_tick(&key,0,1,0,5500);
  assert(menu_key_tick(&key,0,1,0,5531)==KEY_NEXT);
  memset(&key,0,sizeof(key));(void)menu_key_tick(&key,1,1,0,1);(void)menu_key_tick(&key,1,1,0,32);
  (void)menu_key_tick(&key,1,0,0,100);(void)menu_key_tick(&key,1,0,0,131);
  (void)menu_key_tick(&key,1,1,0,200);assert(menu_key_tick(&key,1,1,0,231)==KEY_START);
  for(unsigned index=0;index<2;index++){
    memset(&key,0,sizeof(key));(void)menu_key_tick(&key,index,1,0,1);(void)menu_key_tick(&key,index,1,0,32);
    (void)menu_key_tick(&key,index,0,1,100);assert(menu_key_tick(&key,index,0,1,131)==KEY_STOP);
    (void)menu_key_tick(&key,index,1,1,200);assert(menu_key_tick(&key,index,1,1,231)==KEY_NONE);
  }
  MenuCommand command={0};unsigned actions=0;
  const char *noise="GgTT\n@MENU:BAD\n@MENU:RUN";
  for(const char *p=noise;*p;p++)actions+=menu_command_feed(&command,(uint8_t)*p);
  assert(actions==0 && menu_command_feed(&command,'\n')==1);
  for(const char *p="@MENU:NEXT";*p;p++)assert(menu_command_feed(&command,(uint8_t)*p)==0);
  assert(menu_command_feed(&command,'\n')==2);
}
int main(void)
{
  check_keys();reset();assert(app.phase==APP_MENU && app.selected==0 && !starts);
  const char *labels[]={"1-1","1-2","2-1","2-2","3","4","5","6-1","6-2","6-3","6-4"};
  assert(ALL_MENU_COUNT==11U && !menu_app_laser_on(&app));
  for(unsigned i=1;i<=22;i++){
    menu_app_next(&app);assert(app.selected==i%11 && !starts);
    assert(!strcmp(menu_app_label(app.selected),labels[i%11]));
    assert(menu_app_laser_on(&app)==(app.selected>=7U));
  }
  for(unsigned i=0;i<11;i++){
    for(unsigned phase=APP_MENU;phase<=APP_FINISH;phase++){
      app.selected=i;app.phase=(AppPhase)phase;app.message[0]=0;
      assert(menu_app_laser_on(&app)==(i>=7U && phase==APP_MENU));
    }
  }
  app.selected=11;app.phase=APP_MENU;assert(!menu_app_laser_on(&app));
  app.selected=255;assert(!menu_app_laser_on(&app));
  const unsigned selected[]={0,1,2,3,6};
  const NativeId ids[]={NATIVE_11,NATIVE_12,NATIVE_21,NATIVE_22,NATIVE_5};
  const char *done[]={"EVENT,STOP,100,B_REACHED,turns=0","EVENT,STOP,100,A_REACHED_ONE_LAP,turns=4",
    "EVENT,TASK2_1_DONE,garage_entry_only","EVENT,SIDE_REFERENCE_DONE,entry_only","EVENT,EXT2_ENTRY_DONE,UNTESTED,motors_off"};
  for(unsigned i=0;i<5;i++){
    reset();app.selected=selected[i];now=100;menu_app_start(&app,now);
    assert(current==ids[i] && app.phase==APP_PREFLIGHT && !starts && !pose_calls);
    menu_app_next(&app);assert(app.selected==selected[i]);menu_app_start(&app,now);assert(selects==1);
    tick(101);tick(300);assert(!starts);tick(301);assert(starts==1 && app.phase==APP_RUNNING);
    native_event(done[i]);busy=0;tick(400);assert(app.phase==APP_FINISH && app.success);
    tick(1899);assert(!reboots);tick(1900);assert(reboots==1);tick(2500);assert(reboots==1 && starts==1);
  }
  for(unsigned i=7;i<11;i++){
    reset();app.selected=i;assert(menu_app_laser_on(&app));menu_app_start(&app,0);
    assert(app.phase==APP_FINISH && !selects && !starts && !pose_calls && !strcmp(app.message,"RESERVED"));
    assert(menu_app_laser_on(&app));tick(1500);assert(reboots==1 && !starts);
    menu_app_init(&app);assert(!menu_app_laser_on(&app));
  }
  reset();screen=0;menu_app_start(&app,0);assert(app.phase==APP_FINISH && !starts && !selects);
  reset();menu_app_start(&app,0);ready=0;tick(3000);assert(app.phase==APP_FINISH && !starts);
  reset();menu_app_start(&app,0);tick(1);start_ok=0;tick(201);assert(app.phase==APP_FINISH && !busy);
  reset();menu_app_start(&app,0);tick(1);tick(201);menu_app_stop(&app,"KEY_STOP",202);assert(app.phase==APP_FINISH && !busy && !app.success);
  reset();menu_app_start(&app,0);tick(1);tick(201);native_event("EVENT,STOP,500,PARKING_LINE_LOST,turns=0");busy=0;tick(300);
  assert(app.phase==APP_FINISH && !app.success && strstr(native_last_event(),"PARKING_LINE_LOST"));
  for(unsigned i=0;i<4;i++){
    begin_visual(4);result_roman(i);assert(current==NATIVE_3A+i && app.phase==APP_PREFLIGHT && !starts);
    tick(1399);assert(!starts);tick(1400);assert(starts==1 && app.phase==APP_RUNNING);
    result_roman(i);assert(starts==1); /* duplicate result cannot restart */
  }
  for(unsigned i=0;i<6;i++){
    begin_visual(5);result_card(i);assert(plan.branch_id==i && app.phase==APP_PREFLIGHT && !starts);
    tick(1400);assert(app.phase==APP_RUNNING && starts==1);
  }
  begin_visual(4);feed("MV,1,FAIL,1234ABCD,43,3,TIMEOUT",1200);tick(1200);assert(app.phase==APP_FINISH && !starts);
  begin_visual(5);feed("MV,1,FAIL,1234ABCD,43,4,TIMEOUT",1200);tick(1200);assert(app.phase==APP_FINISH && !starts);
  begin_visual(5);menu_app_stop(&app,"REMOTE_X",1150);result_card(0);assert(app.phase==APP_FINISH && !starts);
  begin_visual(4);result_roman(0);tick(1400);vision_rx_errors++;tick(1410);assert(app.phase==APP_FINISH && !busy);
  begin_visual(5);result_card(0);tick(1400);vision_rx_overflows++;tick(1410);assert(app.phase==APP_FINISH && !busy);
  begin_visual(5);result_card(0);tick(1400);feed("MV,1,READY,87654321,0,IDLE,0,0,0",1410);tick(1410);assert(app.phase==APP_FINISH && !busy);
  begin_visual(4);result_roman(0);tick(1400);tick(10000);assert(app.phase==APP_FINISH && !busy);
  reset();app.selected=5;pose_ok=0;menu_app_start(&app,0);assert(app.phase==APP_FINISH && !starts);
  reset();app.selected=4;menu_app_start(&app,0);pose_stuck=1;tick(5001);assert(app.phase==APP_FINISH && !starts);
  for(unsigned i=0;i<NATIVE_COUNT;i++){
    completion_select((NativeId)i);native_event("EVENT,STOP,100,IMU_STALE,turns=0");assert(!completion_done());
    native_event("EVENT,STOP,100,CD_FOLLOW_DISTANCE_LIMIT,turns=0");assert(!completion_done());
    native_event("EVENT,STOP,100,PARKING_LINE_LOST,turns=0");assert(!completion_done());
  }
  puts("PASS MENU: 11 entries, PE0 on only for 6-1/6-2/6-3/6-4 selection, wrap/reset/invalid-state laser off, two keys (PA15 next only / PE6 start), held/bounce/stop guards, full-frame commands, 5 direct dispatches, four real-protocol Roman targets, six real-protocol card branches, servo-before-SET, 200ms preflight, no duplicate restart, link/failure/cancel stops, completion-vs-protection, four reserved entries no motion, stopped reset-to-menu");
  return 0;
}
