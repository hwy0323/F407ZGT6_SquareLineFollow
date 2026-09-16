#include "main.h"
#include "oled.h"

#define OLED_WIDTH 128U
#define OLED_PAGES 8U
#define OLED_I2C_TIMEOUT_MS 20U

static I2C_HandleTypeDef hi2c1;
static uint8_t oled_buffer[OLED_WIDTH * OLED_PAGES];
static uint16_t oled_address = 0U;
static uint8_t oled_ready = 0U;

/* 5 x 7 font: only the uppercase characters used by the menu are needed. */
static const uint8_t font_digits[10][5] = {
  {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00},
  {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4B, 0x31},
  {0x18, 0x14, 0x12, 0x7F, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
  {0x3C, 0x4A, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
  {0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1E}
};

static const uint8_t font_upper[26][5] = {
  {0x7E, 0x11, 0x11, 0x11, 0x7E}, {0x7F, 0x49, 0x49, 0x49, 0x36},
  {0x3E, 0x41, 0x41, 0x41, 0x22}, {0x7F, 0x41, 0x41, 0x22, 0x1C},
  {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 0x09, 0x09, 0x09, 0x01},
  {0x3E, 0x41, 0x49, 0x49, 0x7A}, {0x7F, 0x08, 0x08, 0x08, 0x7F},
  {0x00, 0x41, 0x7F, 0x41, 0x00}, {0x20, 0x40, 0x41, 0x3F, 0x01},
  {0x7F, 0x08, 0x14, 0x22, 0x41}, {0x7F, 0x40, 0x40, 0x40, 0x40},
  {0x7F, 0x02, 0x0C, 0x02, 0x7F}, {0x7F, 0x04, 0x08, 0x10, 0x7F},
  {0x3E, 0x41, 0x41, 0x41, 0x3E}, {0x7F, 0x09, 0x09, 0x09, 0x06},
  {0x3E, 0x41, 0x51, 0x21, 0x5E}, {0x7F, 0x09, 0x19, 0x29, 0x46},
  {0x46, 0x49, 0x49, 0x49, 0x31}, {0x01, 0x01, 0x7F, 0x01, 0x01},
  {0x3F, 0x40, 0x40, 0x40, 0x3F}, {0x1F, 0x20, 0x40, 0x20, 0x1F},
  {0x3F, 0x40, 0x38, 0x40, 0x3F}, {0x63, 0x14, 0x08, 0x14, 0x63},
  {0x07, 0x08, 0x70, 0x08, 0x07}, {0x61, 0x51, 0x49, 0x45, 0x43}
};

static void OLED_Write_Command(uint8_t command)
{
  uint8_t data[2] = {0x00U, command};
  (void)HAL_I2C_Master_Transmit(&hi2c1, oled_address, data, 2U,
                                OLED_I2C_TIMEOUT_MS);
}

static const uint8_t *OLED_Get_Glyph(char character)
{
  static const uint8_t blank[5] = {0U, 0U, 0U, 0U, 0U};
  static const uint8_t colon[5] = {0U, 0x36U, 0x36U, 0U, 0U};
  static const uint8_t dash[5] = {0x08U, 0x08U, 0x08U, 0x08U, 0x08U};
  static const uint8_t arrow[5] = {0x08U, 0x1CU, 0x3EU, 0x1CU, 0x08U};

  if (character >= 'A' && character <= 'Z') return font_upper[character - 'A'];
  if (character >= '0' && character <= '9') return font_digits[character - '0'];
  if (character == ':') return colon;
  if (character == '-') return dash;
  if (character == '>') return arrow;
  return blank;
}

uint8_t OLED_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &gpio);

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000U;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0U;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0U;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) return 0U;

  if (HAL_I2C_IsDeviceReady(&hi2c1, 0x3CU << 1U, 2U,
                            OLED_I2C_TIMEOUT_MS) == HAL_OK) {
    oled_address = 0x3CU << 1U;
  } else if (HAL_I2C_IsDeviceReady(&hi2c1, 0x3DU << 1U, 2U,
                                   OLED_I2C_TIMEOUT_MS) == HAL_OK) {
    oled_address = 0x3DU << 1U;
  } else {
    return 0U;
  }

  OLED_Write_Command(0xAEU); /* Display off. */
  OLED_Write_Command(0x20U); OLED_Write_Command(0x00U); /* Horizontal mode. */
  OLED_Write_Command(0xB0U); OLED_Write_Command(0xC8U);
  OLED_Write_Command(0x00U); OLED_Write_Command(0x10U);
  OLED_Write_Command(0x40U); OLED_Write_Command(0x81U); OLED_Write_Command(0x7FU);
  OLED_Write_Command(0xA1U); OLED_Write_Command(0xA6U);
  OLED_Write_Command(0xA8U); OLED_Write_Command(0x3FU);
  OLED_Write_Command(0xA4U); OLED_Write_Command(0xD3U); OLED_Write_Command(0x00U);
  OLED_Write_Command(0xD5U); OLED_Write_Command(0x80U);
  OLED_Write_Command(0xD9U); OLED_Write_Command(0xF1U);
  OLED_Write_Command(0xDAU); OLED_Write_Command(0x12U);
  OLED_Write_Command(0xDBU); OLED_Write_Command(0x40U);
  OLED_Write_Command(0x8DU); OLED_Write_Command(0x14U);
  OLED_Write_Command(0xAFU); /* Display on. */

  oled_ready = 1U;
  OLED_Clear();
  OLED_Refresh();
  return 1U;
}

uint8_t OLED_Is_Ready(void)
{
  return oled_ready;
}

void OLED_Clear(void)
{
  uint16_t index;
  for (index = 0U; index < sizeof(oled_buffer); index++) oled_buffer[index] = 0U;
}

void OLED_Draw_String(uint8_t row, uint8_t column, const char *text)
{
  uint16_t buffer_index;

  if (row >= OLED_PAGES || column >= 21U) return;
  buffer_index = (uint16_t)row * OLED_WIDTH + (uint16_t)column * 6U;

  while (*text != '\0' && buffer_index + 5U < ((uint16_t)row + 1U) * OLED_WIDTH) {
    const uint8_t *glyph = OLED_Get_Glyph(*text++);
    uint8_t pixel_column;
    for (pixel_column = 0U; pixel_column < 5U; pixel_column++) {
      oled_buffer[buffer_index++] = glyph[pixel_column];
    }
    oled_buffer[buffer_index++] = 0U;
  }
}

static void OLED_Draw_Pixel(uint8_t x, uint8_t y)
{
  if (x < OLED_WIDTH && y < OLED_PAGES * 8U) {
    oled_buffer[(uint16_t)(y / 8U) * OLED_WIDTH + x] |=
        (uint8_t)(1U << (y % 8U));
  }
}

void OLED_Draw_Big_String(uint8_t x, uint8_t y, uint8_t scale, const char *text)
{
  while (*text != '\0') {
    const uint8_t *glyph = OLED_Get_Glyph(*text++);
    uint8_t glyph_column;

    for (glyph_column = 0U; glyph_column < 5U; glyph_column++) {
      uint8_t glyph_row;
      for (glyph_row = 0U; glyph_row < 7U; glyph_row++) {
        if ((glyph[glyph_column] & (1U << glyph_row)) != 0U) {
          uint8_t dx;
          uint8_t dy;
          for (dx = 0U; dx < scale; dx++) {
            for (dy = 0U; dy < scale; dy++) {
              OLED_Draw_Pixel((uint8_t)(x + glyph_column * scale + dx),
                              (uint8_t)(y + glyph_row * scale + dy));
            }
          }
        }
      }
    }
    x = (uint8_t)(x + 6U * scale);
  }
}

void OLED_Refresh(void)
{
  uint8_t page;
  uint8_t data[OLED_WIDTH + 1U];

  if (!oled_ready) return;
  data[0] = 0x40U;
  for (page = 0U; page < OLED_PAGES; page++) {
    uint16_t column;
    OLED_Write_Command((uint8_t)(0xB0U + page));
    OLED_Write_Command(0x00U);
    OLED_Write_Command(0x10U);
    for (column = 0U; column < OLED_WIDTH; column++) {
      data[column + 1U] = oled_buffer[(uint16_t)page * OLED_WIDTH + column];
    }
    (void)HAL_I2C_Master_Transmit(&hi2c1, oled_address, data, sizeof(data),
                                  OLED_I2C_TIMEOUT_MS);
  }
}
