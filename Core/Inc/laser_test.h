#ifndef LASER_TEST_H
#define LASER_TEST_H

#include "stm32f4xx_hal.h"

/*
 * These variables are intentionally global and volatile.  While the test
 * firmware is running, inspect them through ST-Link in CubeIDE's
 * Live Expressions view.
 */
extern volatile uint16_t laser_distance_mm;
extern volatile uint16_t laser_model_id;
extern volatile uint8_t laser_status;
extern volatile uint8_t laser_range_status;
extern volatile uint8_t laser_detected_address;
extern volatile uint8_t laser_bus_pins;
extern volatile uint32_t laser_sample_count;
extern volatile int32_t laser_api_error;

void LaserTest_Init(I2C_HandleTypeDef *i2c);
void LaserTest_Process(void);

#endif
