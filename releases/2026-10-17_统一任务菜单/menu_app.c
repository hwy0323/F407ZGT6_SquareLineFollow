#include "menu_app.h"
#include "native.h"
#include "gimbal_pwm.h"
#include "vision_uart6.h"
#include <stdio.h>
#include <string.h>
static const char *const labels[ALL_MENU_COUNT]={"1-1","1-2","2-1","2-2","3","4","5","6-1","6-2","6-3","6-4"};
const char *menu_app_label(uint8_t selected){return selected<ALL_MENU_COUNT?labels[selected]:"-";}
uint8_t menu_app_laser_on(const MenuApp *a)
{
  /* 只看当前选中的预留项；其他任务及异常状态均关闭激光笔。 */
  return a->selected>=7U && a->selected<ALL_MENU_COUNT &&
    (a->phase==APP_MENU || (a->phase==APP_FINISH && !strcmp(a->message,"RESERVED")));
}
void menu_app_init(MenuApp *a){memset(a,0,sizeof(*a));a->dirty=1;app_force_stop();}
void menu_app_next(MenuApp *a)
{
  if(a->phase!=APP_MENU)return;
  a->selected=(uint8_t)((a->selected+1U)%ALL_MENU_COUNT);a->dirty=1;
}
static void finish(MenuApp *a,uint8_t success,const char *reason,uint32_t now)
{
  app_force_stop();native_stop();
  if(a->visual)mv_cancel(&a->vision,now);
  a->phase=APP_FINISH;a->success=success;a->since=app_now();a->dirty=1;
  snprintf(a->message,sizeof(a->message),"%s",reason);
  char line[150];snprintf(line,sizeof(line),"MENU,END,%s,%s,%s\r\n",menu_app_label(a->selected),success?"DONE":"STOP",reason);app_log(line);
}
void menu_app_stop(MenuApp *a,const char *reason,uint32_t now)
{
  app_force_stop();
  if(a->phase!=APP_MENU && a->phase!=APP_FINISH)finish(a,0,reason,now);
}
void menu_app_start(MenuApp *a,uint32_t now)
{
  if(a->phase!=APP_MENU)return;
  if(a->selected>=ALL_MENU_COUNT){finish(a,0,"INVALID_TASK",now);return;}
  if(a->selected>=7U){finish(a,0,"RESERVED",now);return;}
  if(!app_screen_ready()){finish(a,0,"OLED_NOT_READY",now);return;}
  app_force_stop();
  NativeId id=(NativeId)a->selected;
  if(a->selected==4U)id=NATIVE_3A;
  if(a->selected==5U)id=NATIVE_4;
  if(a->selected==6U)id=NATIVE_5;
  native_select(id);now=app_now();
  a->visual=a->selected==4U || a->selected==5U;
  a->since=now;a->stable=0;a->dirty=1;
  if(a->visual){
    vision_uart6_init();vision_uart6_reset();mv_init(&a->vision,vision_uart6_send,app_now());
    a->rx_errors=vision_rx_errors;a->rx_overflows=vision_rx_overflows;
    uint8_t task=a->selected==4U?3U:4U;
    if(!gimbal_begin_task(task,app_now())){finish(a,0,"GIMBAL_FAILED",app_now());return;}
    a->since=app_now();a->phase=APP_POSE;app_log("MENU,SERVO_BEFORE_VISION,MOTORS_OFF\r\n");
  }else a->phase=APP_PREFLIGHT;
}
void menu_app_tick(MenuApp *a,uint32_t now)
{
  if(a->phase==APP_MENU){app_force_stop();return;}
  if(a->visual){
    mv_poll(&a->vision,now);vision_uart6_poll(&a->vision,now);
    if(a->phase!=APP_FINISH && (vision_uart6_fault() ||
       vision_rx_errors!=a->rx_errors || vision_rx_overflows!=a->rx_overflows)){
      finish(a,0,"VISION_UART_ERROR",now);return;
    }
  }
  if(a->phase==APP_FINISH){
    app_force_stop();
    if(now-a->since>=1500U && !a->rebooted){a->rebooted=1;app_reboot();}
    return;
  }
  if(a->phase==APP_POSE){
    if(gimbal_is_settled(now)){
      uint8_t ok=a->selected==4U?mv_request_roman(&a->vision,now):mv_request_shape(&a->vision,now);
      if(!ok){finish(a,0,"VISION_REQUEST_REFUSED",now);return;}
      a->phase=APP_VISION;a->dirty=1;
    }else if(now-a->since>GIMBAL_PREPARE_TIMEOUT_MS){finish(a,0,"GIMBAL_TIMEOUT",now);return;}
  }
  if(a->phase==APP_VISION && now-a->since>40000U){finish(a,0,"VISION_TIMEOUT",now);return;}
  if(a->visual){
    MvEvent event;
    if(mv_take_event(&a->vision,&event)){
      if(event.kind==MV_FAILED || event.kind==MV_LOST){finish(a,0,event.reason,now);return;}
      if(a->phase==APP_VISION && a->selected==4U && event.kind==MV_ROMAN && event.task==3U){
        uint8_t index=event.number==1U?0:event.number==3U?1:event.number==5U?2:event.number==7U?3:255;
        if(index>3U || event.target!=(char)('A'+index)){finish(a,0,"INVALID_ROMAN",now);return;}
        a->result=event;native_select((NativeId)(NATIVE_3A+index));
        a->phase=APP_PREFLIGHT;a->since=app_now();a->stable=0;a->dirty=1;
      }else if(a->phase==APP_VISION && a->selected==5U && event.kind==MV_SHAPE && event.task==4U){
        if(!native_plan(event.color,event.shape,event.action)){finish(a,0,"INVALID_PARKING_PLAN",now);return;}
        a->result=event;a->phase=APP_PREFLIGHT;a->since=app_now();a->stable=0;a->dirty=1;
      }
    }
    if((a->phase==APP_PREFLIGHT || a->phase==APP_RUNNING) && !mv_link_ok(&a->vision,now)){
      finish(a,0,"VISION_LINK_UNSAFE",now);return;
    }
  }
  native_tick();now=app_now();
  if(a->phase==APP_PREFLIGHT){
    if(native_ready()){
      if(!a->stable){a->stable=1;a->stable_since=now;}
      if(now-a->stable_since>=200U){
        if(native_start()){
          a->phase=APP_RUNNING;a->run_since=app_now();
          /* RUN 画面已在预检时显示；行驶期间不整屏刷新 OLED。 */
          app_log("MENU,RUN_STARTED\r\n");
        }else finish(a,0,"START_REFUSED",now);
      }
    }else a->stable=0;
    if(a->phase==APP_PREFLIGHT && now-a->since>=3000U)finish(a,0,"NEED_LINE_IMU_LASER_KEY_RELEASED",now);
  }else if(a->phase==APP_RUNNING){
    if(!native_busy())finish(a,native_done(),native_done()?"NATIVE_DONE":"NATIVE_PROTECTION_STOP",now);
    else if(now-a->run_since>90000U)finish(a,0,"MENU_RUN_TIMEOUT",now);
  }
  if(a->phase!=APP_RUNNING)app_force_stop();
}
