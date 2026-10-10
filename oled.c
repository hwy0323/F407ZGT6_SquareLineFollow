#include "main.h"
#include "oled.h"

#define OLED_WIDTH 128U
#define OLED_PAGES 8U
#define OLED_I2C_TIMEOUT_MS 20U

static I2C_HandleTypeDef hi2c1;
static uint8_t oled_buffer[OLED_WIDTH * OLED_PAGES];
static uint16_t oled_address = 0U;
static uint8_t oled_ready = 0U;
static HAL_StatusTypeDef oled_last_status = HAL_OK;
static uint32_t oled_last_error, oled_refresh_ok, oled_refresh_fail;

static HAL_StatusTypeDef OLED_Transmit(uint8_t *data, uint16_t length)
{
  oled_last_status = HAL_I2C_Master_Transmit(&hi2c1, oled_address, data, length, OLED_I2C_TIMEOUT_MS);
  oled_last_error = HAL_I2C_GetError(&hi2c1);
  if (oled_last_status != HAL_OK) oled_ready = 0U;
  return oled_last_status;
}

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
  (void)OLED_Transmit(data, 2U);
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
  oled_ready = 0U;
  oled_address = 0U;

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_NOPULL; /* OLED module supplies external I2C pull-ups. */
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
  oled_last_status = HAL_I2C_Init(&hi2c1);
  oled_last_error = HAL_I2C_GetError(&hi2c1);
  if (oled_last_status != HAL_OK) return 0U;

  oled_last_status = HAL_I2C_IsDeviceReady(&hi2c1, 0x3CU << 1U, 2U, OLED_I2C_TIMEOUT_MS);
  if (oled_last_status == HAL_OK) oled_address = 0x3CU << 1U;
  else {
    oled_last_status = HAL_I2C_IsDeviceReady(&hi2c1, 0x3DU << 1U, 2U, OLED_I2C_TIMEOUT_MS);
    if (oled_last_status == HAL_OK) oled_address = 0x3DU << 1U;
  }
  oled_last_error = HAL_I2C_GetError(&hi2c1);
  if (!oled_address) return 0U;

  /* Page mode matches B0/column commands in OLED_Refresh. */
  static const uint8_t commands[] = {
    0xAEU,0x20U,0x02U,0xB0U,0xC8U,0x00U,0x10U,0x40U,0x81U,0x7FU,
    0xA1U,0xA6U,0xA8U,0x3FU,0xA4U,0xD3U,0x00U,0xD5U,0x80U,
    0xD9U,0xF1U,0xDAU,0x12U,0xDBU,0x40U,0x8DU,0x14U,0xAFU
  };
  for (uint8_t i = 0U; i < sizeof(commands); ++i) {
    OLED_Write_Command(commands[i]);
    if (oled_last_status != HAL_OK) return 0U;
  }

  oled_ready = 1U;
  OLED_Clear();
  OLED_Refresh();
  return oled_ready;
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

static void OLED_Draw_Pixel(int16_t x, int16_t y)
{
  if (x >= 0 && x < (int16_t)OLED_WIDTH && y >= 0 && y < (int16_t)(OLED_PAGES * 8U)) {
    oled_buffer[(uint16_t)(y / 8U) * OLED_WIDTH + x] |=
        (uint8_t)(1U << (y % 8U));
  }
}

void OLED_Draw_Big_String(int16_t x, int16_t y, uint8_t scale, const char *text)
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
              OLED_Draw_Pixel((int16_t)(x + glyph_column * scale + dx),
                              (int16_t)(y + glyph_row * scale + dy));
            }
          }
        }
      }
    }
    x = (int16_t)(x + 6U * scale);
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
    if (!oled_ready) { ++oled_refresh_fail; return; }
    for (column = 0U; column < OLED_WIDTH; column++) {
      data[column + 1U] = oled_buffer[(uint16_t)page * OLED_WIDTH + column];
    }
    if (OLED_Transmit(data, sizeof(data)) != HAL_OK) { ++oled_refresh_fail; return; }
  }
  ++oled_refresh_ok;
}

void OLED_Read_Status(uint32_t status[6])
{
  status[0] = oled_address >> 1U;
  status[1] = oled_ready;
  status[2] = (uint32_t)oled_last_status;
  status[3] = oled_last_error;
  status[4] = oled_refresh_ok;
  status[5] = oled_refresh_fail;
}

void OLED_Draw_Rect(int16_t x, int16_t y, int16_t width, int16_t height)
{
  if (width < 1 || height < 1) return;
  for (int16_t dx = 0; dx < width; ++dx) {
    OLED_Draw_Pixel((int16_t)(x + dx), y);
    OLED_Draw_Pixel((int16_t)(x + dx), (int16_t)(y + height - 1));
  }
  for (int16_t dy = 0; dy < height; ++dy) {
    OLED_Draw_Pixel(x, (int16_t)(y + dy));
    OLED_Draw_Pixel((int16_t)(x + width - 1), (int16_t)(y + dy));
  }
}
