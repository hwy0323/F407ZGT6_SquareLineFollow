#ifndef ROUTE_RECORDER_H
#define ROUTE_RECORDER_H

#include <stdint.h>

/* Offline route teaching uses only short PA15 presses. */
void RouteRecorder_Init(void);
void RouteRecorder_Handle_Short_Press(uint32_t now,
                                      int32_t left_front,
                                      int32_t left_rear,
                                      int32_t right_rear);
void RouteRecorder_Process(uint32_t now);
uint8_t RouteRecorder_LED_Is_On(void);

#endif /* ROUTE_RECORDER_H */
