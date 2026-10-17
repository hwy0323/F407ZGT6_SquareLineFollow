#ifndef PARKING_CONTROLLER_H
#define PARKING_CONTROLLER_H
#include "task4_plan.h"
void parking_init(void);
void parking_clock_init(void);
void parking_tick(void);
void parking_imu_irq(void);
uint8_t parking_configure(const char *color, const char *shape, const char *action);
uint8_t parking_preflight(void);
uint8_t parking_busy(void);
uint8_t parking_start(void);
uint8_t parking_done(void);
void parking_stop(void);
const Task4Plan *parking_current_plan(void);
#endif
