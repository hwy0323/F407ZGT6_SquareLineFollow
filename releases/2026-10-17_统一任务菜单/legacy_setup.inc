/* 所有旧任务使用同一接线。每次复位后只运行一个任务，结束复位回菜单。 */
static uint32_t menu_sample,menu_control,menu_report,menu_motion,menu_imu;
static void menu_legacy_init(void)
{
  static const uint8_t full[7]={0xAA,0x55,0x60,0x0B,0x01,0x00,0x6C};
  static const uint8_t start[7]={0xAA,0x55,0x60,0x0A,0x01,0x01,0x6C};
  gpio_init();
  __HAL_RCC_TIM2_CLK_ENABLE();__HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();__HAL_RCC_TIM8_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();__HAL_RCC_UART4_CLK_ENABLE();
  encoder_init(&enc_a,TIM4);encoder_init(&enc_b,TIM3);
  encoder_init(&enc_c,TIM8);encoder_init(&enc_d,TIM2);
  uart_init(&dap_uart,USART1);uart_init(&imu_uart,UART4);
  SET_BIT(USART1->CR1,USART_CR1_RXNEIE);
  HAL_NVIC_SetPriority(USART1_IRQn,7U,0U);HAL_NVIC_EnableIRQ(USART1_IRQn);
  SET_BIT(UART4->CR1,USART_CR1_RXNEIE);
  HAL_NVIC_SetPriority(UART4_IRQn,6U,0U);HAL_NVIC_EnableIRQ(UART4_IRQn);
  pwm_init();stop_motors();
  (void)HAL_UART_Transmit(&imu_uart,(uint8_t*)full,7,20);
  (void)HAL_UART_Transmit(&imu_uart,(uint8_t*)start,7,20);
  menu_imu=HAL_GetTick();
}
static void menu_legacy_sample(uint32_t now)
{
  if(now-menu_sample>=5U){sensor_sample();menu_sample=now;}
  if(!imu_frames && now-menu_imu>=2000U){
    static const uint8_t start[7]={0xAA,0x55,0x60,0x0A,0x01,0x01,0x6C};
    (void)HAL_UART_Transmit(&imu_uart,(uint8_t*)start,7,20);menu_imu=now;
  }
}
static uint8_t menu_legacy_ready(void)
{
  uint32_t now=HAL_GetTick();
  return gray_frames && sample_age_ms(now,gray_last_ms)<=100U &&
    (gray_value&0x18U) && active_count(gray_value)<=3U &&
    imu_frames && sample_age_ms(now,imu_last_ms)<=100U &&
    HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==GPIO_PIN_SET;
}
