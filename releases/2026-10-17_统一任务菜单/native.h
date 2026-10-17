#ifndef MENU_NATIVE_H
#define MENU_NATIVE_H
#include <stdint.h>
typedef enum { NATIVE_11, NATIVE_12, NATIVE_21, NATIVE_22,
               NATIVE_3A, NATIVE_3B, NATIVE_3C, NATIVE_3D,
               NATIVE_4, NATIVE_5, NATIVE_COUNT } NativeId;
void native_select(NativeId id);
void native_tick(void);
void native_imu_irq(void);
uint8_t native_ready(void);
uint8_t native_start(void);
uint8_t native_busy(void);
uint8_t native_done(void);
void native_stop(void);
uint8_t native_plan(const char *color,const char *shape,const char *action);
void native_event(const char *text);
const char *native_last_event(void);
void route_clock_init(void);
#endif
