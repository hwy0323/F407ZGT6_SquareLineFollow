#ifndef VISION_UART6_H
#define VISION_UART6_H
#include "mv_client.h"
void vision_uart6_init(void);
void vision_uart6_poll(MvClient *client, uint32_t now);
uint8_t vision_uart6_send(const char *data, uint16_t length);
uint8_t vision_uart6_fault(void);
void vision_uart6_reset(void);
/* Main-loop trace only; never called from IRQ, never echoed to USART6. */
void vision_trace(const char *direction,const char *data,uint16_t length);
extern volatile uint32_t vision_rx_bytes,vision_rx_errors,vision_rx_overflows;
extern volatile uint32_t vision_tx_bytes;
#endif
