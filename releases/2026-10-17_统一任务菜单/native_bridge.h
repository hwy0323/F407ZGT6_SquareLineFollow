#ifndef MENU_NATIVE_BRIDGE_H
#define MENU_NATIVE_BRIDGE_H
#include "main.h"
#include "native.h"
/* 只旁听原串口事件，随后原样发送；不改成功版本的运动控制。 */
HAL_StatusTypeDef menu_native_transmit(UART_HandleTypeDef *uart,uint8_t *data,
                                      uint16_t size,uint32_t timeout);
#endif
