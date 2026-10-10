#include "main.h"
#include "oled.h"
#include "menu_ui.h"
#include <stdio.h>

/* Read through ST-Link; this program does not require a USB UART. */
volatile uint32_t menu_test_status[20] = {0x4D454E55U};
static MenuUi menu;
typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
  uint8_t raw, stable, armed, ready;
  uint32_t change_ms, count;
} KeyInput;
static KeyInput keys[2] = {
  {GPIOA, GPIO_PIN_15, 1U, 1U, 0U, 0U, 0U, 0U},
  {GPIOE, GPIO_PIN_6, 1U, 1U, 0U, 0U, 0U, 0U}
};

void SysTick_Handler(void) { HAL_IncTick(); }
void USART1_IRQHandler(void) { }
void USART3_IRQHandler(void) { }
void UART4_IRQHandler(void) { }

void Error_Handler(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
  menu_test_status[1] = 0xEEU;
  while (1) { }
}

static void stationary_gpio_init(void)
{
  GPIO_InitTypeDef pin = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, 0x00FFU, GPIO_PIN_RESET);
  pin.Pin = GPIO_PIN_0;
  pin.Mode = GPIO_MODE_OUTPUT_PP;
  pin.Pull = GPIO_NOPULL;
  pin.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &pin);
  pin.Pin = 0x00FFU;
  HAL_GPIO_Init(GPIOD, &pin);
  pin.Pin = GPIO_PIN_15;
  pin.Mode = GPIO_MODE_INPUT;
  pin.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &pin);
  pin.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOE, &pin);
}

static void clock_init(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = 8U;
  osc.PLL.PLLN = 336U;
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = 7U;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV4;
  clk.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}


static uint8_t key_tick(uint32_t now)
{
  uint8_t released = 0U;
  for (uint8_t i = 0U; i < 2U; ++i) {
    KeyInput *key = &keys[i];
    uint8_t raw = HAL_GPIO_ReadPin(key->port, key->pin) == GPIO_PIN_SET;
    if (raw != key->raw) { key->raw = raw; key->change_ms = now; }
    if (now - key->change_ms < 30U) continue;
    if (!key->ready) {
      key->stable = raw;
      if (raw) key->ready = 1U;
      continue;
    }
    if (raw == key->stable) continue;
    key->stable = raw;
    if (!raw) key->armed = menu.view != MENU_PREVIEW;
    else if (key->armed) {
      key->armed = 0U;
      if (menu.view != MENU_PREVIEW) { ++key->count; released |= (uint8_t)(1U << i); }
    }
  }
  return released;
}

static void update_diagnostics(uint32_t now)
{
  uint32_t status[6];
  OLED_Read_Status(status);
  menu_test_status[2] = now;
  for (uint8_t i = 0U; i < 6U; ++i) menu_test_status[4U + i] = status[i];
  menu_test_status[10] = menu.selected;
  menu_test_status[11] = menu.view;
  menu_test_status[12] = keys[0].count;
  menu_test_status[13] = keys[1].count;
  menu_test_status[14] = menu.queued_steps;
  menu_test_status[15] = menu.preview_count;
  menu_test_status[16] = menu.completion_count;
  menu_test_status[17] = keys[0].raw;
  menu_test_status[18] = keys[1].raw;
  menu_test_status[19] = (keys[0].ready ? 1U : 0U) | (keys[1].ready ? 2U : 0U);
}

int main(void)
{
  uint32_t last_draw_ms = 0U, last_retry_ms = 0U;
  HAL_Init();
  stationary_gpio_init();
  menu_test_status[1] = 1U;
  clock_init();
  menu_test_status[1] = 2U;
  MenuUi_Init(&menu);
  HAL_Delay(100U);
  ++menu_test_status[3];
  (void)OLED_Init();
  menu_test_status[1] = 3U;
  while (1) {
    uint32_t now = HAL_GetTick();
    uint8_t released = key_tick(now);
    if (released & 2U) {
      MenuUi_Confirm(&menu, now);
      keys[0].armed = keys[1].armed = 0U;
    }
    else if (released & 1U) MenuUi_Next(&menu, now);
    MenuUi_Tick(&menu, HAL_GetTick());
    update_diagnostics(HAL_GetTick());
    now = HAL_GetTick();
    if (!OLED_Is_Ready() && now - last_retry_ms >= 1000U) {
      last_retry_ms = now;
      ++menu_test_status[3];
      (void)OLED_Init();
      menu.dirty = 1U;
    }
    now = HAL_GetTick();
    if (OLED_Is_Ready() && (menu.dirty ||
          (menu.view == MENU_SCROLL && now - last_draw_ms >= 40U) ||
          now - last_draw_ms >= 1000U)) {
      menu.dirty = 0U;
      MenuUi_Draw(&menu, now);
      last_draw_ms = now;
      update_diagnostics(HAL_GetTick());
    }
    HAL_Delay(1U);
  }
}
