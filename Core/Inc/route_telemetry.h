#ifndef ROUTE_TELEMETRY_H
#define ROUTE_TELEMETRY_H

#include "main.h"

uint8_t RouteTelemetry_Init(void);
void RouteTelemetry_SendReady(void);
void RouteTelemetry_SendStart(uint32_t now_ms);
void RouteTelemetry_SendStop(uint32_t elapsed_ms, const char *reason);
void RouteTelemetry_SendSample(uint32_t elapsed_ms,
                               uint8_t sensor_raw,
                               uint8_t sensor_used,
                               int16_t sensor_error,
                               int32_t encoder_lf,
                               int32_t encoder_rf,
                               int32_t encoder_lr,
                               int32_t encoder_rr,
                               int16_t left_command,
                               int16_t right_command,
                               uint8_t state,
                               uint8_t turns);
uint8_t RouteTelemetry_ReadByte(uint8_t *value);

#endif
