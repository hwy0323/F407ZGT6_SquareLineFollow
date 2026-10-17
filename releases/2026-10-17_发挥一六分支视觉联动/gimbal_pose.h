#ifndef GIMBAL_POSE_H
#define GIMBAL_POSE_H
#include <stdint.h>

/* Keep the old upper-servo pulse range; lower servo has a different travel. */
#define GIMBAL_UPPER_RANGE_DEG 180U
#define GIMBAL_LOWER_RANGE_DEG 270U
#define GIMBAL_MIN_PULSE_US 500U
#define GIMBAL_MAX_PULSE_US 2500U
#define GIMBAL_SETTLE_MS 1000U
#define GIMBAL_PREPARE_TIMEOUT_MS 5000U
typedef struct {
  uint16_t upper_deg,lower_deg,upper_us,lower_us;
} GimbalPose;
uint16_t gimbal_pulse_us(uint16_t angle,uint16_t full_range);
uint8_t gimbal_pose_for_task(uint8_t task,GimbalPose *pose);
#endif
