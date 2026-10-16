/* Hardware, encoder and IMU routines copied from the PB15 task-4 base.
 * See README for source provenance. No route logic is included here. */
void SysTick_Handler(void) { HAL_IncTick(); }

static void stop_motors(void)
{
  __HAL_TIM_SET_COMPARE(&pwm_timer, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&pwm_timer, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&pwm_timer, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&pwm_timer, TIM_CHANNEL_4, 0U);
  HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, 0x00FFU, GPIO_PIN_RESET);
  left_command = 0;
  right_command = 0;
}

void Error_Handler(void)
{
  /* May run before the PWM timer has been initialized. */
  HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);
  while (1) { }
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

static void gpio_init(void)
{
  GPIO_InitTypeDef pin = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);
  pin.Pin = MOTOR_STBY_PIN;
  pin.Mode = GPIO_MODE_OUTPUT_PP;
  pin.Pull = GPIO_PULLDOWN;
  pin.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_STBY_PORT, &pin);
  pin.Pull = GPIO_NOPULL;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  pin.Pin = GPIO_PIN_13;
  pin.Mode = GPIO_MODE_OUTPUT_PP;
  pin.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &pin);
  HAL_GPIO_WritePin(GPIOD, 0x00FFU, GPIO_PIN_RESET);
  pin.Pin = 0x00FFU;
  pin.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &pin);

  pin.Mode = GPIO_MODE_INPUT;
  pin.Pull = GPIO_PULLUP;
  pin.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOA, &pin);

  pin.Mode = GPIO_MODE_AF_PP;
  pin.Pull = GPIO_PULLUP;
  pin.Speed = GPIO_SPEED_FREQ_HIGH;
  pin.Alternate = GPIO_AF1_TIM2;
  pin.Pin = GPIO_PIN_0 | GPIO_PIN_1;  /* D right-front */
  HAL_GPIO_Init(GPIOA, &pin);
  pin.Alternate = GPIO_AF2_TIM3;
  pin.Pin = GPIO_PIN_6 | GPIO_PIN_7;  /* B left-rear */
  HAL_GPIO_Init(GPIOA, &pin);
  pin.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &pin);        /* A left-front */
  pin.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(GPIOC, &pin);        /* C right-rear */

  pin.Pull = GPIO_NOPULL;
  pin.Alternate = GPIO_AF1_TIM1;
  pin.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
  HAL_GPIO_Init(GPIOE, &pin);

  pin.Pull = GPIO_PULLUP;
  pin.Alternate = GPIO_AF7_USART1;
  pin.Pin = GPIO_PIN_9 | GPIO_PIN_10; /* DAP PA9/PA10 */
  HAL_GPIO_Init(GPIOA, &pin);
  pin.Alternate = GPIO_AF8_UART4;
  pin.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  HAL_GPIO_Init(GPIOC, &pin);         /* IMU PC10/PC11 */

  HAL_GPIO_WritePin(SENSOR_ADDRESS_PORT,
                    SENSOR_AD0_PIN | SENSOR_AD1_PIN | SENSOR_AD2_PIN,
                    GPIO_PIN_RESET);
  pin.Pin = SENSOR_AD0_PIN | SENSOR_AD1_PIN | SENSOR_AD2_PIN;
  pin.Mode = GPIO_MODE_OUTPUT_PP;
  pin.Pull = GPIO_NOPULL;
  pin.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENSOR_ADDRESS_PORT, &pin);
  pin.Pin = SENSOR_OUT_PIN;
  pin.Mode = GPIO_MODE_INPUT;
  pin.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SENSOR_OUT_PORT, &pin);
}

static void uart_init(UART_HandleTypeDef *uart, USART_TypeDef *device)
{
  uart->Instance = device;
  uart->Init.BaudRate = 115200U;
  uart->Init.WordLength = UART_WORDLENGTH_8B;
  uart->Init.StopBits = UART_STOPBITS_1;
  uart->Init.Parity = UART_PARITY_NONE;
  uart->Init.Mode = UART_MODE_TX_RX;
  uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  uart->Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(uart) != HAL_OK) Error_Handler();
}

static void encoder_init(TIM_HandleTypeDef *timer, TIM_TypeDef *device)
{
  TIM_Encoder_InitTypeDef config = {0};
  timer->Instance = device;
  timer->Init.Prescaler = 0U;
  timer->Init.CounterMode = TIM_COUNTERMODE_UP;
  timer->Init.Period = 0xFFFFU;
  timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  config.EncoderMode = TIM_ENCODERMODE_TI12;
  config.IC1Polarity = TIM_ICPOLARITY_RISING;
  config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  config.IC1Filter = 8U;
  config.IC2Polarity = TIM_ICPOLARITY_RISING;
  config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  config.IC2Filter = 8U;
  if (HAL_TIM_Encoder_Init(timer, &config) != HAL_OK) Error_Handler();
  __HAL_TIM_SET_COUNTER(timer, 0U);
  if (HAL_TIM_Encoder_Start(timer, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
}

static void pwm_init(void)
{
  TIM_OC_InitTypeDef channel = {0};
  __HAL_RCC_TIM1_CLK_ENABLE();
  pwm_timer.Instance = TIM1;
  pwm_timer.Init.Prescaler = 0U;
  pwm_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
  pwm_timer.Init.Period = PWM_PERIOD;
  pwm_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&pwm_timer) != HAL_OK) Error_Handler();
  channel.OCMode = TIM_OCMODE_PWM1;
  channel.Pulse = 0U;
  channel.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&pwm_timer, &channel, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_ConfigChannel(&pwm_timer, &channel, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_ConfigChannel(&pwm_timer, &channel, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIM_PWM_ConfigChannel(&pwm_timer, &channel, TIM_CHANNEL_4) != HAL_OK)
    Error_Handler();
  if (HAL_TIM_PWM_Start(&pwm_timer, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_PWM_Start(&pwm_timer, TIM_CHANNEL_2) != HAL_OK ||
      HAL_TIM_PWM_Start(&pwm_timer, TIM_CHANNEL_3) != HAL_OK ||
      HAL_TIM_PWM_Start(&pwm_timer, TIM_CHANNEL_4) != HAL_OK)
    Error_Handler();
}

static int16_t clamp(int16_t value, int16_t limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

/* A sensor IRQ/sample may advance its timestamp after the main-loop snapshot.
 * Treat that small negative age as fresh instead of unsigned wraparound.
 */
static uint32_t sample_age_ms(uint32_t now, uint32_t sample_ms)
{
  int32_t age = (int32_t)(now - sample_ms);
  return age < 0 ? 0U : (uint32_t)age;
}

static void one_motor(uint16_t in1, uint16_t in2, uint32_t channel,
                      int16_t command, uint8_t forward_is_01)
{
  uint16_t duty;
  command = clamp(command, 1000);
  duty = (uint16_t)(((uint32_t)(command < 0 ? -command : command) * PWM_PERIOD) / 1000U);
  if (command == 0) HAL_GPIO_WritePin(GPIOD, in1 | in2, GPIO_PIN_RESET);
  else {
    uint8_t first_high = ((command > 0) != (forward_is_01 != 0U));
    HAL_GPIO_WritePin(GPIOD, in1, first_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, in2, first_high ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }
  __HAL_TIM_SET_COMPARE(&pwm_timer, channel, duty);
}

static void drive(int16_t left, int16_t right)
{
  left_command = left;
  right_command = right;
  one_motor(GPIO_PIN_0, GPIO_PIN_1, TIM_CHANNEL_1, left, 1U);  /* A LF */
  one_motor(GPIO_PIN_2, GPIO_PIN_3, TIM_CHANNEL_2, left, 0U);  /* B LR */
  one_motor(GPIO_PIN_4, GPIO_PIN_5, TIM_CHANNEL_3, right, 0U); /* C RR */
  one_motor(GPIO_PIN_6, GPIO_PIN_7, TIM_CHANNEL_4, right, 1U); /* D RF */
  HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN,
                    (left || right) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void send_text(const char *text)
{
  (void)HAL_UART_Transmit(&dap_uart, (uint8_t *)text,
                          (uint16_t)strlen(text), 60U);
}

static uint8_t active_count(uint8_t bits)
{
  uint8_t count = 0U;
  while (bits) { count += bits & 1U; bits >>= 1U; }
  return count;
}

static int16_t line_error(uint8_t bits)
{
  const int8_t weights[8] = {7, 5, 3, 1, -1, -3, -5, -7};
  int16_t sum = 0;
  uint8_t count = 0U;
  /* Either middle sensor alone, or both together, means straight ahead. */
  if (bits == 0x08U || bits == 0x10U || bits == 0x18U) return 0;
  for (uint8_t i = 0U; i < 8U; i++)
    if (bits & (1U << i)) { sum += weights[i]; count++; }
  return count ? sum / count : 0;
}

static int16_t enc_count(TIM_HandleTypeDef *timer)
{
  return (int16_t)__HAL_TIM_GET_COUNTER(timer);
}

static void odom_update(void)
{
  int16_t counts[4] = {enc_count(&enc_a), enc_count(&enc_b),
                       enc_count(&enc_c), enc_count(&enc_d)};
  for (uint8_t i = 0U; i < 4U; i++) {
    odom_total[i] += (int16_t)(counts[i] - odom_previous[i]);
    odom_previous[i] = counts[i];
  }
}

/* START is the first sighting of a corner, before the braking and pivot.
 * DONE is after line reacquisition. The four signed counts are cumulative
 * since KEY start, so timer wrap does not lose the corner positions.
 */
static void record_corner_event(const char *phase, uint8_t number,
                                Direction direction, uint32_t now)
{
  char line[180];
  uint8_t lap = number ? (uint8_t)((number - 1U) / 4U + 1U) : 0U;
  uint8_t corner = number ? (uint8_t)((number - 1U) % 4U + 1U) : 0U;
  char turn = direction == LEFT_TURN ? 'L' :
              direction == RIGHT_TURN ? 'R' : '-';
  odom_update();
  (void)snprintf(line, sizeof(line),
                 "EVENT,CORNER,%s,%u,%u,%u,%c,%lu,%02X,%u,%ld,%ld,%ld,%ld\r\n",
                 phase, (unsigned int)number, (unsigned int)lap,
                 (unsigned int)corner, turn,
                 (unsigned long)(now - run_start_ms),
                 (unsigned int)gray_value, (unsigned int)imu_yaw_x100,
                 (long)odom_total[0], (long)odom_total[1],
                 (long)odom_total[2], (long)odom_total[3]);
  send_text(line);
}

static void reset_encoders(void)
{
  __HAL_TIM_SET_COUNTER(&enc_a, 0U);
  __HAL_TIM_SET_COUNTER(&enc_b, 0U);
  __HAL_TIM_SET_COUNTER(&enc_c, 0U);
  __HAL_TIM_SET_COUNTER(&enc_d, 0U);
  memset(previous_counts, 0, sizeof(previous_counts));
  memset(odom_previous, 0, sizeof(odom_previous));
  memset(odom_total, 0, sizeof(odom_total));
}

static void report(uint32_t now)
{
  char line[220];
  odom_update();
  uint32_t sensor_age = gray_frames ? sample_age_ms(now, gray_last_ms) : 9999U;
  uint32_t imu_age = imu_frames ? sample_age_ms(now, imu_last_ms) : 9999U;
  (void)snprintf(line, sizeof(line),
    "D,%lu,%u,%02X,%lu,%lu,%d,%d,%d,%d,%d,%d,%u,%u,%d,%d,%d,%lu,%lu\r\n",
    (unsigned long)(state == STOPPED ? 0U : now - run_start_ms),
    (unsigned int)state, (unsigned int)gray_value,
    (unsigned long)sensor_age, (unsigned long)gray_errors,
    (int)enc_count(&enc_a), (int)enc_count(&enc_b),
    (int)enc_count(&enc_c), (int)enc_count(&enc_d),
    (int)left_command, (int)right_command,
    (unsigned int)turns, (unsigned int)imu_yaw_x100,
    (int)imu_pitch_x100, (int)imu_roll_x100, (int)imu_gyro_z_raw,
    (unsigned long)imu_age, (unsigned long)imu_frames);
  send_text(line);
}

static void sensor_delay_us(uint32_t microseconds)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = microseconds * (SystemCoreClock / 1000000U);
  while (DWT->CYCCNT - start < cycles) { }
}

static void sensor_sample(void)
{
  uint8_t bits = 0U;
  for (uint8_t channel = 0U; channel < 8U; channel++) {
    HAL_GPIO_WritePin(SENSOR_ADDRESS_PORT, SENSOR_AD0_PIN,
                      (channel & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SENSOR_ADDRESS_PORT, SENSOR_AD1_PIN,
                      (channel & 2U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SENSOR_ADDRESS_PORT, SENSOR_AD2_PIN,
                      (channel & 4U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    sensor_delay_us(SENSOR_SETTLE_US);
    if (HAL_GPIO_ReadPin(SENSOR_OUT_PORT, SENSOR_OUT_PIN) == SENSOR_BLACK_LEVEL)
      bits |= (uint8_t)(1U << (7U - channel));
  }
  gray_value = bits;
  gray_frames++;
  gray_last_ms = HAL_GetTick();
}

static void imu_parse(uint8_t byte)
{
  if (imu_state == 0U) { imu_state = (byte == 0xAAU) ? 1U : 0U; return; }
  if (imu_state == 1U) {
    imu_state = (byte == 0x55U) ? 2U : ((byte == 0xAAU) ? 1U : 0U);
    return;
  }
  if (imu_state == 2U) {
    imu_frame[0] = byte; imu_sum = byte; imu_index = 1U; imu_state = 3U;
    return;
  }
  if (imu_state == 3U) {
    imu_frame[imu_index++] = byte; imu_sum += byte; imu_state = 4U;
    return;
  }
  if (imu_state == 4U) {
    imu_length = byte;
    imu_frame[imu_index++] = byte;
    imu_sum += byte;
    imu_state = (byte > 24U) ? 0U : (byte == 0U ? 6U : 5U);
    return;
  }
  if (imu_state == 5U) {
    imu_frame[imu_index++] = byte; imu_sum += byte;
    if (imu_index == (uint8_t)(3U + imu_length)) imu_state = 6U;
    return;
  }
  if (byte == imu_sum && imu_frame[0] == 0x60U &&
      imu_frame[1] == 0x01U && imu_length == 18U) {
    imu_gyro_z_raw = (int16_t)((uint16_t)imu_frame[13] |
                    ((uint16_t)imu_frame[14] << 8U));
    imu_pitch_x100 = (int16_t)((uint16_t)imu_frame[15] |
                     ((uint16_t)imu_frame[16] << 8U));
    imu_roll_x100 = (int16_t)((uint16_t)imu_frame[17] |
                    ((uint16_t)imu_frame[18] << 8U));
    imu_yaw_x100 = (uint16_t)imu_frame[19] |
                   ((uint16_t)imu_frame[20] << 8U);
    imu_frames++;
    imu_last_ms = HAL_GetTick();
  }
  imu_state = 0U;
}

void UART4_IRQHandler(void)
{
  uint32_t status = UART4->SR;
  if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
    (void)UART4->DR; imu_state = 0U;
  } else if (status & USART_SR_RXNE) imu_parse((uint8_t)UART4->DR);
}
