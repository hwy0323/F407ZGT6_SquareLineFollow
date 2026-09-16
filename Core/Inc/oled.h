#ifndef OLED_H
#define OLED_H

#include <stdint.h>

/* Standard 128 x 64 I2C SSD1306 OLED on I2C1: PB8=SCL, PB9=SDA. */
uint8_t OLED_Init(void);
uint8_t OLED_Is_Ready(void);
void OLED_Clear(void);
void OLED_Draw_String(uint8_t row, uint8_t column, const char *text);
void OLED_Draw_Big_String(uint8_t x, uint8_t y, uint8_t scale, const char *text);
void OLED_Refresh(void);

#endif /* OLED_H */
