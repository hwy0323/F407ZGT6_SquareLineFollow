/* Original C controller is included unchanged. Legacy main and IRQs are
 * renamed so all four proven routes can live in ONE flash image. */
#define main legacy_C_main
#define SysTick_Handler legacy_C_systick
#define Error_Handler legacy_C_error
#define USART1_IRQHandler legacy_C_uart1
#define USART3_IRQHandler legacy_C_uart3
#define UART4_IRQHandler route_C_imu_irq
#include "../2026-10-17_双层版本/3-C/main.c"
#undef main
#undef SysTick_Handler
#undef Error_Handler
#undef USART1_IRQHandler
#undef USART3_IRQHandler
#undef UART4_IRQHandler

static uint32_t last_control, last_report, last_motion;
static uint32_t last_sensor_sample, last_imu_start;
static const uint8_t imu_full[7] = {0xAA,0x55,0x60,0x0B,0x01,0x00,0x6C};
static const uint8_t imu_start[7] = {0xAA,0x55,0x60,0x0A,0x01,0x01,0x6C};


void route_C_init(void)
{
  gpio_init();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_UART4_CLK_ENABLE();
  encoder_init(&enc_a, TIM4);
  encoder_init(&enc_b, TIM3);
  encoder_init(&enc_c, TIM8);
  encoder_init(&enc_d, TIM2);
  uart_init(&dap_uart, USART1);
  uart_init(&imu_uart, UART4);
#ifdef PIVOT_MULTI
  SET_BIT(USART1->CR1, USART_CR1_RXNEIE);
  HAL_NVIC_SetPriority(USART1_IRQn, 7U, 0U);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
#endif
  SET_BIT(UART4->CR1, USART_CR1_RXNEIE);
  HAL_NVIC_SetPriority(UART4_IRQn, 6U, 0U);
  HAL_NVIC_EnableIRQ(UART4_IRQn);
  pwm_init();
  stop_motors();
  (void)HAL_UART_Transmit(&imu_uart, (uint8_t *)imu_full, 7U, 20U);
  (void)HAL_UART_Transmit(&imu_uart, (uint8_t *)imu_start, 7U, 20U);
  last_imu_start = HAL_GetTick();
}

void route_C_tick(void)
{
    uint32_t now = HAL_GetTick();
#ifdef PIVOT_TEST
#ifdef GARAGE_A_AUTO
    if (now - last_sensor_sample >= 5U) {
      sensor_sample();
      last_sensor_sample = now;
    }
    if (auto_phase == 2U &&
        now - auto_pause_start_ms >= AUTO_GARAGE_PAUSE_MS) {
      begin_post_exit_follow(now);
    }
    if (auto_phase == 3U && !route_active && !pivot_active)
      auto_phase = 4U;
#else
#ifdef ROUTE_REPLAY
    route_read_key(now);
#endif
#endif
    if (imu_frames == 0U && now - last_imu_start >= 2000U) {
      (void)HAL_UART_Transmit(&imu_uart, (uint8_t *)imu_full, 7U, 20U);
      (void)HAL_UART_Transmit(&imu_uart, (uint8_t *)imu_start, 7U, 20U);
      last_imu_start = now;
    }
    if (now - last_control >= CONTROL_MS) {
      last_control = now;
#ifdef GARAGE_A_AUTO
      if (auto_phase == 1U && state != STOPPED) {
        if (post_exit_tracking) post_exit_control(now);
        else control(now);
      }
#endif
      pivot_control(now);
#ifdef ROUTE_REPLAY
      route_tick(now);
#endif
    }
#ifdef GARAGE_A_AUTO
    if (auto_phase == 1U && now - last_motion >= 100U) {
      last_motion = now;
      check_encoder_motion(now);
    }
#endif
    if (now - last_report >= REPORT_MS) {
      last_report = now;
#ifdef GARAGE_A_AUTO
      report(now);
      if (auto_phase == 3U) pivot_report(now);
#else
      pivot_report(now);
#endif
    }
#else
    read_key(now);
    read_dap_command();
    if (now - last_sensor_sample >= 5U) {
      sensor_sample();
      last_sensor_sample = now;
    }
    if (imu_frames == 0U && now - last_imu_start >= 2000U) {
      (void)HAL_UART_Transmit(&imu_uart, (uint8_t *)imu_start, 7U, 20U);
      last_imu_start = now;
    }
    if (now - last_control >= CONTROL_MS) {
      last_control = now;
      control(now);
    }
    if (now - last_motion >= 100U) {
      last_motion = now;
      check_encoder_motion(now);
    }
    if (now - last_report >= REPORT_MS) {
      last_report = now;
      report(now);
      if (state == STOPPED)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,
                          ((now / 500U) & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
#endif
}

uint8_t route_C_preflight(void)
{
  uint32_t now = HAL_GetTick();
  return gray_frames && sample_age_ms(now, gray_last_ms) <= 100U &&
         gray_value == 0xFFU && imu_frames &&
         sample_age_ms(now, imu_last_ms) <= 150U &&
         HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET;
}

uint8_t route_C_busy(void)
{
  return state != STOPPED || pivot_active || route_active || auto_phase == 2U;
}

uint8_t route_C_start(void)
{
  start_from_garage(HAL_GetTick());
  return route_C_busy();
}

void route_C_stop(void)
{
  state = STOPPED;
  pivot_active = route_active = route_waiting = 0U;
  auto_phase = 4U;
  stop_motors();
}
