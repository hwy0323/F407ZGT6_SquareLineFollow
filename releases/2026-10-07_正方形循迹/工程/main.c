#include "main.h"
#include <stdio.h>
#include <string.h>
#ifdef ROUTE_REPLAY
#ifndef PIVOT_MULTI
#error ROUTE_REPLAY requires PIVOT_MULTI
#endif
#include "route_data.h"
#endif

/* CD4051 sensor: channel 0 is leftmost, channel 7 is rightmost.
 * Keep the existing controller's bit 7 = leftmost convention.
 */
#define PWM_PERIOD 8399U
#define SENSOR_ADDRESS_PORT GPIOB
#define SENSOR_OUT_PORT GPIOC
#define SENSOR_AD0_PIN GPIO_PIN_12
#define SENSOR_AD1_PIN GPIO_PIN_13
#define SENSOR_AD2_PIN GPIO_PIN_14
#define SENSOR_OUT_PIN GPIO_PIN_1
#define SENSOR_BLACK_LEVEL GPIO_PIN_SET
#define SENSOR_SETTLE_US 50U
/* Keep the square-follow corner and short line-loss recovery behavior. */
#define SEGMENT_RECORD_MODE 0U
#define REVERSE_RECORD_MODE 0U
#define FORWARD_REVERSE_REFERENCE_MODE 0U
#define S_CURVE_RECORD_MODE 0U
#define FORWARD_REFERENCE_MAX_RUN_MS 12000U
#define S_CURVE_SPEED 155
#define S_CURVE_KP 20
#define S_CURVE_MAX_CORRECTION 100
#define S_CURVE_LOST_SPEED 85
#define REVERSE_LEFT_SPEED 120
#define REVERSE_RIGHT_SPEED 185
#define REVERSE_LOST_SPEED 110
#define REVERSE_LOST_STOP_MS 300U
#define REVERSE_MAX_RUN_MS 8000U
#define REVERSE_MAX_COUNTS 4000
#define REVERSE_MAX_YAW_X100 5500U
#define SEGMENT_LOSS_CONFIRM_MS 500U
#define SEGMENT_LOSS_MAX_YAW_X100 2000U
#define DRIVE_SPEED 200
#define CORNER_SPEED 170
#define PIVOT_SPEED 220
#define LOST_SPEED 120
#define LOST_HOLD_MS 60U
#define LOST_STRAIGHT_MS 220U
#define LOST_SWEEP_SWITCH_MS 420U
#define LOST_SWEEP_SPEED 105
#define LOST_SWEEP_BIAS 55
#define LOST_MAX_YAW_X100 1200U
#define LOST_MAX_ENCODER_COUNTS 500
#define KP 14
#define MAX_CORRECTION 105
#define CORNER_CONFIRM_MS 30U
#define CORNER_EDGE_WINDOW_MS 150U
#define CORNER_BRAKE_MS 120U
#define CORNER_COOLDOWN_MS 300U
#define TURN_MIN_ANGLE_X100 3000U
#define TURN_MAX_ANGLE_X100 12500U
#define TURN_RECOVER_MS 100U
#define CONTROL_MS 10U
#define REPORT_MS 50U
#define SENSOR_TIMEOUT_MS 200U
#define LOST_GRACE_MS 650U
#define TURN_TIMEOUT_MS 1500U
#define MAX_RUN_MS 120000U
#define CORNER_TARGET_COUNT 4U
#define CORNER_POSITION_WINDOW 900L
#define PARKING_PAUSE_MS 100U
#define PARKING_PASS_SPEED 140
#define PARKING_PASS_MIN_COUNTS 120L
#define PARKING_PASS_MAX_COUNTS 700L
#define PARKING_PASS_MAX_MS 1700U
#define PARKING_PASS_MAX_YAW_X100 1500U
#define PARKING_PASS_LINE_LOST_MS 120U
#define PARKING_EVENT_LIMIT 6U
#ifdef STOP_AFTER_ONE_CORNER
#define ONE_CORNER_APPROACH_TIMEOUT_MS 6000U
#endif

typedef enum { STOPPED, FOLLOW, ENTER_CORNER, PIVOT, RECOVER,
               WAIT_FINISH_KEY, PARKING_PAUSE, PARKING_PASS } State;
typedef enum { NO_TURN, LEFT_TURN, RIGHT_TURN } Direction;

static TIM_HandleTypeDef pwm_timer, enc_a, enc_b, enc_c, enc_d;
static UART_HandleTypeDef dap_uart, imu_uart;
static volatile uint8_t gray_value;
static volatile uint32_t gray_frames, gray_last_ms, gray_errors;
static volatile uint16_t imu_yaw_x100;
static volatile int16_t imu_pitch_x100, imu_roll_x100, imu_gyro_z_raw;
static volatile uint32_t imu_frames, imu_last_ms;
static volatile uint8_t imu_state, imu_index, imu_length, imu_sum;
static volatile uint8_t imu_frame[27];
static State state = STOPPED;
static Direction turn_direction = NO_TURN, corner_candidate = NO_TURN;
static uint8_t center_hits, left_old_line, turns;
#ifdef GIT_VERIFIED_CORNER
static uint8_t git_corner_hits;
static uint8_t parking_events, parking_clean_hits;
static uint32_t parking_line_lost_ms;
static int32_t parking_pass_start_count;
static uint16_t parking_pass_start_yaw_x100;
#endif
static int16_t left_command, right_command, last_correction;
static int16_t held_left_command, held_right_command;
static uint32_t run_start_ms, state_start_ms, line_lost_ms;
static uint32_t corner_first_ms, last_turn_complete_ms, last_center_ms;
static uint16_t turn_start_yaw_x100, line_lost_yaw_x100;
static uint16_t reverse_start_yaw_x100;
static int16_t line_lost_count_a, line_lost_count_d;
static int8_t line_search_direction;
static uint32_t last_encoder_progress_ms;
static int16_t previous_counts[4];
static int16_t odom_previous[4];
static int32_t odom_total[4];
static uint8_t line_lost;

static void report(uint32_t now);

void SysTick_Handler(void) { HAL_IncTick(); }

static void stop_motors(void)
{
  __HAL_TIM_SET_COMPARE(&pwm_timer, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&pwm_timer, TIM_CHANNEL_2, 0U);
  __HAL_TIM_SET_COMPARE(&pwm_timer, TIM_CHANNEL_3, 0U);
  __HAL_TIM_SET_COMPARE(&pwm_timer, TIM_CHANNEL_4, 0U);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, 0x00FFU, GPIO_PIN_RESET);
  left_command = 0;
  right_command = 0;
}

void Error_Handler(void)
{
  /* May run before the PWM timer has been initialized. */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
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

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  pin.Pin = GPIO_PIN_0 | GPIO_PIN_13;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0,
                    (left || right) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void send_text(const char *text)
{
  (void)HAL_UART_Transmit(&dap_uart, (uint8_t *)text,
                          (uint16_t)strlen(text), 60U);
}

static void stop_run(const char *reason)
{
  char line[80];
  if (state == STOPPED) return;
  stop_motors();
  state = STOPPED;
  (void)snprintf(line, sizeof(line), "EVENT,STOP,%lu,%s,turns=%u\r\n",
                 (unsigned long)(HAL_GetTick() - run_start_ms), reason, turns);
  send_text(line);
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

static Direction detect_corner(uint8_t bits)
{
  uint8_t left = active_count(bits & 0xF0U);
  uint8_t right = active_count(bits & 0x0FU);
  /* A wide sensor pitch may show only two far-side probes at a square bend.
   * A single off-centre probe is always treated as a small correction.
   */
  if (right <= 1U &&
      (left >= 3U || ((bits & 0xC0U) == 0xC0U &&
                      (bits & 0x18U) == 0U && right == 0U)))
    return LEFT_TURN;
  if (left <= 1U &&
      (right >= 3U || ((bits & 0x03U) == 0x03U &&
                      (bits & 0x18U) == 0U && left == 0U)))
    return RIGHT_TURN;
  return NO_TURN;
}

static uint8_t strong_corner(uint8_t bits, Direction direction)
{
  if (direction == LEFT_TURN)
    return active_count(bits & 0xF0U) >= 3U &&
           active_count(bits & 0x0FU) <= 1U;
  if (direction == RIGHT_TURN)
    return active_count(bits & 0x0FU) >= 3U &&
           active_count(bits & 0xF0U) <= 1U;
  return 0U;
}

static uint8_t outer_tip(uint8_t bits, Direction direction)
{
  if (direction == LEFT_TURN) return bits == 0x80U || bits == 0xC0U;
  if (direction == RIGHT_TURN) return bits == 0x01U || bits == 0x03U;
  return 0U;
}

static uint16_t yaw_change_x100(uint16_t start_yaw)
{
  int32_t delta = (int32_t)imu_yaw_x100 - (int32_t)start_yaw;
  if (delta > 18000) delta -= 36000;
  if (delta < -18000) delta += 36000;
  return (uint16_t)(delta < 0 ? -delta : delta);
}

static uint16_t turn_angle_x100(void)
{
  return yaw_change_x100(turn_start_yaw_x100);
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

static void start_run(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t bits = gray_value;
  if (gray_frames == 0U ||
      sample_age_ms(now, gray_last_ms) > SENSOR_TIMEOUT_MS ||
      (bits & 0x18U) == 0U || active_count(bits) > 3U) {
    send_text("EVENT,REFUSE_START,center_line_or_sensor_missing\r\n");
    return;
  }
  if (imu_frames == 0U || sample_age_ms(now, imu_last_ms) > 300U) {
    send_text("EVENT,REFUSE_START,imu_missing\r\n");
    return;
  }
  reset_encoders();
  reverse_start_yaw_x100 = imu_yaw_x100;
  turns = center_hits = left_old_line = line_lost = 0U;
#ifdef GIT_VERIFIED_CORNER
  git_corner_hits = 0U;
  parking_events = parking_clean_hits = 0U;
  parking_line_lost_ms = 0U;
  parking_pass_start_count = 0L;
  parking_pass_start_yaw_x100 = imu_yaw_x100;
#endif
  last_correction = 0;
  held_left_command = held_right_command = 0;
  corner_candidate = turn_direction = NO_TURN;
  last_turn_complete_ms = last_center_ms = now;
  state = FOLLOW;
  run_start_ms = state_start_ms = last_encoder_progress_ms = now;
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
#ifdef GIT_VERIFIED_CORNER
  send_text("EVENT,START,four_corner_garage_line_test\r\n");
  record_corner_event("ORIGIN", 0U, NO_TURN, now);
#else
  send_text("EVENT,START,four_corner_garage_line_test\r\n");
  record_corner_event("ORIGIN", 0U, NO_TURN, now);
#endif
}

static void control_reverse(uint32_t now, uint8_t bits)
{
  int32_t counts[4] = {enc_count(&enc_a), enc_count(&enc_b),
                       enc_count(&enc_c), enc_count(&enc_d)};

  if (now - run_start_ms >= REVERSE_MAX_RUN_MS) {
    report(now); stop_run("REVERSE_TIME_LIMIT"); return;
  }
  if (yaw_change_x100(reverse_start_yaw_x100) > REVERSE_MAX_YAW_X100) {
    report(now); stop_run("REVERSE_YAW_LIMIT"); return;
  }
  for (uint8_t i = 0U; i < 4U; i++) {
    if (counts[i] < 0) counts[i] = -counts[i];
    if (counts[i] > REVERSE_MAX_COUNTS) {
      report(now); stop_run("REVERSE_DISTANCE_LIMIT"); return;
    }
  }
  if (active_count(bits & 0xF0U) >= 3U &&
      active_count(bits & 0x0FU) >= 3U) {
    report(now); stop_run("AMBIGUOUS_BLACK"); return;
  }

  if (bits == 0U) {
    if (!line_lost) {
      line_lost = 1U;
      line_lost_ms = now;
      held_left_command = left_command;
      held_right_command = right_command;
    }
    if (now - line_lost_ms >= REVERSE_LOST_STOP_MS) {
      stop_motors(); report(now); stop_run("LINE_LOST_SEGMENT"); return;
    }
    if (now - line_lost_ms <= LOST_HOLD_MS)
      drive(held_left_command, held_right_command);
    else
      drive(-REVERSE_LOST_SPEED, -REVERSE_LOST_SPEED);
    return;
  }

  line_lost = 0U;
  /* The front-mounted line sensor trails while reversing. For this fixed
   * left-rear adjustment, command the right wheels faster in reverse so
   * the rear of the car moves left. Use gray data for end-of-line only.
   */
  drive(-REVERSE_LEFT_SPEED, -REVERSE_RIGHT_SPEED);
}

static void begin_corner(Direction direction, uint32_t now)
{
  stop_motors();
  if (turns >= CORNER_TARGET_COUNT) {
    stop_run("TURN_COUNT_LIMIT");
    return;
  }
  record_corner_event("START", (uint8_t)(turns + 1U), direction, now);
  turn_direction = direction;
  state = ENTER_CORNER;
  state_start_ms = now;
  send_text(direction == LEFT_TURN ? "EVENT,CORNER_LEFT\r\n" :
                                    "EVENT,CORNER_RIGHT\r\n");
}

#ifdef GIT_VERIFIED_CORNER
/* Keep the proven square-corner motion, with the car's current pins.
 * The run starts before the BC garage tape and ends after four corners.
 */
static int16_t git_line_error(uint8_t bits)
{
  const int8_t weights[8] = {7, 5, 3, 1, -1, -3, -5, -7};
  int16_t sum = 0;
  uint8_t count = 0U;
  for (uint8_t i = 0U; i < 8U; i++)
    if (bits & (1U << i)) { sum += weights[i]; count++; }
  return count ? sum / count : 0;
}

static Direction git_detect_corner(uint8_t bits)
{
  uint8_t left = active_count(bits & 0xF0U);
  uint8_t right = active_count(bits & 0x0FU);
  if (left == 4U && right <= 1U) return LEFT_TURN;
  if (right == 4U && left <= 1U) return RIGHT_TURN;
  return NO_TURN;
}

/* A encoder was negative on all eight corners in the verified two-lap run.
 * These are the measured corner START positions, not inferred lap lengths.
 */
static const int32_t true_corner_a_counts[CORNER_TARGET_COUNT] = {
  5100L, 11299L, 17567L, 23793L
};

static int32_t forward_a_counts(void)
{
  odom_update();
  return -odom_total[0];
}

static uint8_t near_true_corner(int32_t progress)
{
  int32_t target = true_corner_a_counts[turns];
  return progress >= target - CORNER_POSITION_WINDOW &&
         progress <= target + CORNER_POSITION_WINDOW;
}

/* A wide black feature in the middle of an edge cannot be a true corner.
 * This also covers extra tape on the otherwise plain CD and AB edges.
 */
static uint8_t in_straight_middle(int32_t progress)
{
  int32_t start = turns == 0U ? 200L :
                  true_corner_a_counts[turns - 1U] +
                    CORNER_POSITION_WINDOW;
  int32_t end = true_corner_a_counts[turns] -
                CORNER_POSITION_WINDOW;
  return progress > start && progress < end;
}

static void parking_event(const char *phase, uint32_t now,
                          uint8_t bits, int32_t progress)
{
  char line[120];
  (void)snprintf(line, sizeof(line),
                 "EVENT,PARKING_LINE,%s,%u,%lu,%02X,%ld,%u\r\n",
                 phase, (unsigned int)parking_events,
                 (unsigned long)(now - run_start_ms),
                 (unsigned int)bits, (long)progress,
                 (unsigned int)imu_yaw_x100);
  send_text(line);
}

static void git_verified_control(uint32_t now, uint8_t bits)
{
  Direction detected;
  int16_t correction;
  int32_t progress;
  uint8_t wide_black;
  uint8_t parking_zone;
  uint8_t left_black, right_black;
  if (state == PARKING_PAUSE) {
    stop_motors();
    if (now - state_start_ms >= PARKING_PAUSE_MS) {
      parking_pass_start_count = forward_a_counts();
      parking_pass_start_yaw_x100 = imu_yaw_x100;
      parking_line_lost_ms = 0U;
      parking_clean_hits = 0U;
      last_encoder_progress_ms = now;
      state_start_ms = now;
      state = PARKING_PASS;
      parking_event("RESUME", now, bits, parking_pass_start_count);
    }
    return;
  }
  if (state == PARKING_PASS) {
    progress = forward_a_counts();
    if (now - state_start_ms > PARKING_PASS_MAX_MS ||
        progress - parking_pass_start_count > PARKING_PASS_MAX_COUNTS ||
        yaw_change_x100(parking_pass_start_yaw_x100) >
          PARKING_PASS_MAX_YAW_X100) {
      stop_run("PARKING_PASS_LIMIT"); return;
    }
    if (bits == 0U) {
      if (parking_line_lost_ms == 0U) parking_line_lost_ms = now;
      if (now - parking_line_lost_ms > PARKING_PASS_LINE_LOST_MS) {
        stop_run("PARKING_LINE_LOST"); return;
      }
    } else parking_line_lost_ms = 0U;
    /* Keep a small amount of line correction after crossing the tape.
     * Blind equal-speed driving previously let the line slide off one side.
     */
    correction = bits ? clamp((int16_t)(git_line_error(bits) * 14), 60) : 0;
    drive(PARKING_PASS_SPEED + correction,
          PARKING_PASS_SPEED - correction);
    if ((bits & 0x18U) != 0U && active_count(bits) <= 2U &&
        progress - parking_pass_start_count >= PARKING_PASS_MIN_COUNTS) {
      if (++parking_clean_hits >= 10U) {
        state = FOLLOW;
        last_turn_complete_ms = now;
        corner_candidate = NO_TURN;
        git_corner_hits = 0U;
        parking_event("CLEARED", now, bits, progress);
      }
    } else parking_clean_hits = 0U;
    return;
  }
  if (state == FOLLOW) {
    progress = forward_a_counts();
    if (progress < -200L) { stop_run("ODOM_DIRECTION"); return; }
    if (progress > true_corner_a_counts[turns] +
                   CORNER_POSITION_WINDOW) {
      stop_run("EXPECTED_CORNER_MISSING"); return;
    }
    if (bits == 0U) {
      if (!line_lost) { line_lost = 1U; line_lost_ms = now; }
      if (now - line_lost_ms > 120U) {
        stop_run("GIT_LINE_LOST"); return;
      }
      drive(110, 110);
      return;
    }
    line_lost = 0U;
    left_black = active_count(bits & 0xF0U);
    right_black = active_count(bits & 0x0FU);
    wide_black = left_black >= 3U && right_black >= 3U;
    parking_zone = in_straight_middle(progress);
    detected = now - last_turn_complete_ms >= CORNER_COOLDOWN_MS ?
               git_detect_corner(bits) : NO_TURN;
    if (detected != NO_TURN || wide_black ||
        (parking_zone && (left_black >= 3U || right_black >= 3U))) {
      if (near_true_corner(progress)) {
        if (detected != RIGHT_TURN) {
          stop_run("UNEXPECTED_CORNER_PATTERN"); return;
        }
        if (corner_candidate == RIGHT_TURN) git_corner_hits++;
        else { corner_candidate = RIGHT_TURN; git_corner_hits = 1U; }
        if (git_corner_hits >= 3U) {
          record_corner_event("START", (uint8_t)(turns + 1U), detected, now);
          turn_direction = detected;
          state_start_ms = now;
          state = ENTER_CORNER;
          git_corner_hits = 0U;
          send_text("EVENT,CORNER_RIGHT\r\n");
        }
        return;
      }
      if (parking_zone) {
        if (corner_candidate == LEFT_TURN) git_corner_hits++;
        else { corner_candidate = LEFT_TURN; git_corner_hits = 1U; }
        if (git_corner_hits >= 3U) {
          if (++parking_events > PARKING_EVENT_LIMIT) {
            stop_run("PARKING_EVENT_LIMIT"); return;
          }
          stop_motors();
          state_start_ms = now;
          state = PARKING_PAUSE;
          git_corner_hits = 0U;
          parking_event("PAUSE", now, bits, progress);
        }
        return;
      }
      stop_run("UNEXPECTED_BLACK_FEATURE"); return;
    }
    corner_candidate = NO_TURN;
    git_corner_hits = 0U;
    correction = clamp((int16_t)(git_line_error(bits) * 22), 100);
    drive(220 + correction, 220 - correction);
  } else if (state == ENTER_CORNER) {
    drive(180, 180);
    if (now - state_start_ms >= 100U) {
      state_start_ms = now;
      center_hits = 0U;
      state = PIVOT;
    }
  } else if (state == PIVOT) {
    if (turn_direction == LEFT_TURN) drive(-220, 220);
    else drive(220, -220);
    if (now - state_start_ms > 1200U) {
      stop_run("GIT_TURN_TIMEOUT"); return;
    }
    if (now - state_start_ms >= 120U &&
        (bits & 0x18U) != 0U && active_count(bits) <= 2U) {
      if (++center_hits >= 3U) {
        state_start_ms = now;
        state = RECOVER;
      }
    } else center_hits = 0U;
  } else if (state == RECOVER) {
    drive(180, 180);
    if (now - state_start_ms >= 100U) {
      turns++;
      if (turns >= CORNER_TARGET_COUNT) stop_motors();
      record_corner_event("DONE", turns, turn_direction, now);
      if (turns >= CORNER_TARGET_COUNT) {
        report(now);
        stop_run("FOUR_CORNERS_COMPLETE");
      } else {
        state = FOLLOW;
        last_turn_complete_ms = now;
        corner_candidate = NO_TURN;
      }
    }
  }
}
#endif

static void control(uint32_t now)
{
  uint8_t bits = gray_value;
  uint8_t count = active_count(bits);
  int16_t correction;
  Direction corner;
  if (state == STOPPED) return;
  if (state == WAIT_FINISH_KEY) { stop_motors(); return; }
  if (gray_frames == 0U ||
      sample_age_ms(now, gray_last_ms) > SENSOR_TIMEOUT_MS) {
    stop_run("SENSOR_STALE"); return;
  }
  if (imu_frames == 0U || sample_age_ms(now, imu_last_ms) > 300U) {
    stop_run("IMU_STALE"); return;
  }
  if (now - run_start_ms > MAX_RUN_MS) { stop_run("RUN_TIMEOUT"); return; }
#ifdef GIT_VERIFIED_CORNER
  git_verified_control(now, bits);
  return;
#endif
#ifdef STOP_AFTER_ONE_CORNER
  if (state == FOLLOW && turns == 0U &&
      now - run_start_ms > ONE_CORNER_APPROACH_TIMEOUT_MS) {
    stop_run("ONE_CORNER_NOT_FOUND"); return;
  }
#endif

  if ((FORWARD_REVERSE_REFERENCE_MODE || S_CURVE_RECORD_MODE) &&
      now - run_start_ms > FORWARD_REFERENCE_MAX_RUN_MS) {
    report(now); stop_run("REFERENCE_TIME_LIMIT"); return;
  }

  if (REVERSE_RECORD_MODE) {
    control_reverse(now, bits);
    return;
  }

  if (state == FOLLOW) {
    if (bits & 0x18U) last_center_ms = now;
    if (active_count(bits & 0xF0U) >= 3U &&
        active_count(bits & 0x0FU) >= 3U) {
      stop_run("AMBIGUOUS_BLACK"); return;
    }
    corner = (!FORWARD_REVERSE_REFERENCE_MODE && !S_CURVE_RECORD_MODE &&
              now - last_turn_complete_ms >= CORNER_COOLDOWN_MS &&
              now - last_center_ms <= 250U)
                 ? detect_corner(bits) : NO_TURN;
    if (corner != NO_TURN) {
      if (corner != corner_candidate) {
        corner_candidate = corner;
        corner_first_ms = now;
      }
      if (strong_corner(bits, corner) ||
          now - corner_first_ms >= CORNER_CONFIRM_MS) {
        begin_corner(corner, now);
      } else {
        correction = clamp((int16_t)(line_error(bits) * KP), 60);
        drive(150 + correction, 150 - correction);
      }
      return;
    }
    if (corner_candidate != NO_TURN &&
        now - corner_first_ms <= CORNER_EDGE_WINDOW_MS &&
        (bits == 0U || outer_tip(bits, corner_candidate))) {
      begin_corner(corner_candidate, now);
      return;
    }
    corner_candidate = NO_TURN;
    if (bits == 0U) {
      if (!line_lost) {
        line_lost = 1U;
        line_lost_ms = now;
        line_lost_yaw_x100 = imu_yaw_x100;
        line_lost_count_a = enc_count(&enc_a);
        line_lost_count_d = enc_count(&enc_d);
        line_search_direction = last_correction < 0 ? -1 : 1;
        held_left_command = left_command;
        held_right_command = right_command;
      }
      int32_t moved_a = (int32_t)enc_count(&enc_a) - line_lost_count_a;
      int32_t moved_d = (int32_t)enc_count(&enc_d) - line_lost_count_d;
      if (moved_a < 0) moved_a = -moved_a;
      if (moved_d < 0) moved_d = -moved_d;
      uint32_t loss_limit = SEGMENT_RECORD_MODE ?
                            SEGMENT_LOSS_CONFIRM_MS : LOST_GRACE_MS;
      uint16_t yaw_limit = SEGMENT_RECORD_MODE ?
                           SEGMENT_LOSS_MAX_YAW_X100 : LOST_MAX_YAW_X100;
      if (now - line_lost_ms >= loss_limit ||
          yaw_change_x100(line_lost_yaw_x100) > yaw_limit ||
          moved_a > LOST_MAX_ENCODER_COUNTS ||
          moved_d > LOST_MAX_ENCODER_COUNTS) {
        if (SEGMENT_RECORD_MODE) {
          /* Save the final encoder and IMU readings before STOP. */
          stop_motors();
          report(now);
        }
        stop_run(SEGMENT_RECORD_MODE ? "LINE_LOST_SEGMENT" : "LINE_LOST");
        return;
      }
      if (now - line_lost_ms <= LOST_HOLD_MS) {
        drive(held_left_command, held_right_command);
        return;
      }
      if (SEGMENT_RECORD_MODE) {
        correction = clamp(last_correction, 35);
        if (S_CURVE_RECORD_MODE)
          drive(S_CURVE_LOST_SPEED + correction,
                S_CURVE_LOST_SPEED - correction);
        else
          drive(LOST_SPEED + correction, LOST_SPEED - correction);
        return;
      }
      if (now - line_lost_ms <= LOST_STRAIGHT_MS) {
        correction = clamp(last_correction, 35);
        drive(LOST_SPEED + correction, LOST_SPEED - correction);
      } else {
        int16_t bias = LOST_SWEEP_BIAS * line_search_direction;
        if (now - line_lost_ms > LOST_SWEEP_SWITCH_MS) bias = -bias;
        drive(LOST_SWEEP_SPEED + bias, LOST_SWEEP_SPEED - bias);
      }
      return;
    }
    line_lost = 0U;
    correction = S_CURVE_RECORD_MODE ?
                 clamp((int16_t)(line_error(bits) * S_CURVE_KP),
                       S_CURVE_MAX_CORRECTION) :
                 clamp((int16_t)(line_error(bits) * KP), MAX_CORRECTION);
    last_correction = correction;
    if (S_CURVE_RECORD_MODE)
      drive(S_CURVE_SPEED + correction, S_CURVE_SPEED - correction);
    else
      drive(DRIVE_SPEED + correction, DRIVE_SPEED - correction);
  } else if (state == ENTER_CORNER) {
    /* Brake at the detected corner instead of driving past the junction. */
    stop_motors();
    if (now - state_start_ms >= CORNER_BRAKE_MS) {
      state = PIVOT;
      state_start_ms = now;
      turn_start_yaw_x100 = imu_yaw_x100;
      center_hits = left_old_line = 0U;
    }
  } else if (state == PIVOT) {
    if (turn_direction == LEFT_TURN) drive(-PIVOT_SPEED, PIVOT_SPEED);
    else drive(PIVOT_SPEED, -PIVOT_SPEED);
    if (now - state_start_ms > TURN_TIMEOUT_MS) {
      stop_run("TURN_TIMEOUT"); return;
    }
    if (turn_angle_x100() > TURN_MAX_ANGLE_X100) {
      stop_run("TURN_ANGLE_LIMIT"); return;
    }
    if ((bits & 0x18U) == 0U) left_old_line = 1U;
    if (left_old_line && now - state_start_ms >= 120U &&
        turn_angle_x100() >= TURN_MIN_ANGLE_X100 &&
        (bits & 0x18U) != 0U && count <= 2U) {
      if (++center_hits >= 3U) {
        state = RECOVER;
        state_start_ms = now;
      }
    } else center_hits = 0U;
  } else if (state == RECOVER) {
    drive(CORNER_SPEED, CORNER_SPEED);
    if (now - state_start_ms >= TURN_RECOVER_MS) {
      turns++;
      if (turns >= CORNER_TARGET_COUNT) stop_motors();
      record_corner_event("DONE", turns, turn_direction, now);
      if (turns >= CORNER_TARGET_COUNT) {
        report(now);
        stop_run("FOUR_CORNERS_COMPLETE");
        return;
      }
#ifdef STOP_AFTER_ONE_CORNER
      report(now);
      stop_run("ONE_CORNER_COMPLETE");
      return;
#endif
      state = FOLLOW;
      last_turn_complete_ms = now;
      corner_candidate = NO_TURN;
    }
  }
}

static void check_encoder_motion(uint32_t now)
{
  int16_t values[4] = {enc_count(&enc_a), enc_count(&enc_b),
                       enc_count(&enc_c), enc_count(&enc_d)};
  uint8_t changed = 0U;
  for (uint8_t i = 0U; i < 4U; i++) {
    if (values[i] != previous_counts[i]) changed = 1U;
    previous_counts[i] = values[i];
  }
  if (changed) last_encoder_progress_ms = now;
  if (state != STOPPED && state != WAIT_FINISH_KEY &&
      state != PARKING_PAUSE &&
      now - last_encoder_progress_ms > 1000U)
    stop_run("ENCODER_STALL");
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

/* Kept for the shared startup vector; USART3 is not used by this build. */
void USART3_IRQHandler(void) { }
#ifndef PIVOT_MULTI
void USART1_IRQHandler(void) { }
#endif

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

static void read_key(uint32_t now)
{
  static uint8_t old_raw = 1U, stable = 1U, stop_consumed = 0U;
  static uint32_t change_ms;
  uint8_t raw = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET;
  if (raw != old_raw) { old_raw = raw; change_ms = now; }
  if (raw == stable || now - change_ms < 25U) return;
  stable = raw;
  if (!stable) {
    if (state != STOPPED) {
      stop_run(state == WAIT_FINISH_KEY ? "KEY_FINISH" : "KEY");
      stop_consumed = 1U;
    }
  } else if (stop_consumed) stop_consumed = 0U;
  else if (state == STOPPED) start_run();
}

static void read_dap_command(void)
{
  uint32_t status = USART1->SR;
  if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
    (void)USART1->DR;
  } else if (status & USART_SR_RXNE) {
    uint8_t command = (uint8_t)USART1->DR;
    if ((command == 'X' || command == 'x') && state != STOPPED)
      stop_run("REMOTE_X");
    else if ((command == 'G' || command == 'g') && state == STOPPED)
      start_run();
  }
}

#ifdef PIVOT_TEST
/* Bench test only: one low-speed pivot, commanded over DAP. */
#ifndef PIVOT_TARGET_X100
#define PIVOT_TARGET_X100 9000
#endif
#ifdef PIVOT_TEST_LEFT
#define PIVOT_SIGN (-1)
#else
#define PIVOT_SIGN 1
#endif
static uint8_t pivot_active, pivot_finished, pivot_phase, pivot_pulses;
static uint16_t pivot_start_yaw;
static uint32_t pivot_start_ms, pivot_phase_ms, pivot_progress_ms;
static int32_t pivot_last_progress;
static int32_t pivot_target_x100 = PIVOT_TARGET_X100;
static int8_t pivot_sign = PIVOT_SIGN;
#ifdef PIVOT_MULTI
static char pivot_command[8];
static uint8_t pivot_command_length;
static uint32_t pivot_last_stop_ms, pivot_step_id;
static int16_t pivot_encoder_start[4];
static uint8_t segment_linear;
static int8_t linear_direction;
static int32_t linear_target_mm, linear_target_counts, linear_last_progress;
static volatile uint8_t pivot_rx_buffer[16], pivot_rx_head, pivot_rx_tail;
#ifdef ROUTE_REPLAY
static uint8_t route_active, route_waiting, route_index;
static uint32_t route_start_ms;
static void route_after_step(const char *reason);
#endif

void USART1_IRQHandler(void)
{
  uint32_t status = USART1->SR;
  uint8_t byte = (uint8_t)USART1->DR;
  if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) return;
  if (status & USART_SR_RXNE) {
    uint8_t next = (uint8_t)((pivot_rx_head + 1U) & 15U);
    if (next != pivot_rx_tail) {
      pivot_rx_buffer[pivot_rx_head] = byte;
      pivot_rx_head = next;
    }
  }
}
#endif

static int32_t pivot_delta_x100(void)
{
  int32_t delta = (int32_t)imu_yaw_x100 - (int32_t)pivot_start_yaw;
  if (delta > 18000) delta -= 36000;
  if (delta < -18000) delta += 36000;
  return delta;
}

static int32_t pivot_progress_x100(void)
{
  return pivot_sign * pivot_delta_x100();
}

static void pivot_drive(int16_t speed)
{
  if (pivot_sign < 0) drive(-speed, speed); /* Left turn. */
  else drive(speed, -speed);               /* Right turn. */
}

#ifdef PIVOT_MULTI
static int32_t linear_progress_counts(void)
{
  int32_t values[4] = {
    (int16_t)(enc_count(&enc_a) - pivot_encoder_start[0]),
    (int16_t)(enc_count(&enc_b) - pivot_encoder_start[1]),
    (int16_t)(enc_count(&enc_c) - pivot_encoder_start[2]),
    (int16_t)(enc_count(&enc_d) - pivot_encoder_start[3])
  };
  for (uint8_t i = 0U; i < 4U; i++)
    if (values[i] < 0) values[i] = -values[i];
  for (uint8_t i = 0U; i < 4U; i++)
    for (uint8_t j = (uint8_t)(i + 1U); j < 4U; j++)
      if (values[j] < values[i]) {
        int32_t temp = values[i]; values[i] = values[j]; values[j] = temp;
      }
  return (values[1] + values[2]) / 2;
}

static void linear_drive(int16_t speed)
{
  int32_t correction = pivot_delta_x100() / 10;
  if (correction > 45) correction = 45;
  if (correction < -45) correction = -45;
  drive((int16_t)(linear_direction * speed - correction),
        (int16_t)(linear_direction * speed + correction));
}
#endif

static void pivot_stop(const char *reason)
{
  char line[200];
  stop_motors();
  pivot_active = 0U;
  pivot_finished = 1U; /* No second motion without a reset. */
#ifdef PIVOT_MULTI
  pivot_last_stop_ms = HAL_GetTick();
  (void)snprintf(line, sizeof(line),
                 "EVENT,STEP_END,%lu,%s,%c,%ld,%u,%u,%ld,%lu,%ld,%d,%d,%d,%d\r\n",
                 (unsigned long)pivot_step_id, reason,
                 segment_linear ? (linear_direction > 0 ? 'F' : 'B') :
                                  (pivot_sign < 0 ? 'L' : 'R'),
                 (long)(segment_linear ? linear_target_mm : pivot_target_x100),
                 (unsigned int)pivot_start_yaw,
                 (unsigned int)imu_yaw_x100, (long)pivot_delta_x100(),
                 (unsigned long)(HAL_GetTick() - pivot_start_ms),
                 (long)(segment_linear ? linear_progress_counts() :
                                        pivot_progress_x100()),
                 (int)(int16_t)(enc_count(&enc_a) - pivot_encoder_start[0]),
                 (int)(int16_t)(enc_count(&enc_b) - pivot_encoder_start[1]),
                 (int)(int16_t)(enc_count(&enc_c) - pivot_encoder_start[2]),
                 (int)(int16_t)(enc_count(&enc_d) - pivot_encoder_start[3]));
#else
  (void)snprintf(line, sizeof(line),
                 "EVENT,PIVOT_STOP,%s,yaw_start=%u,yaw_end=%u,delta_x100=%ld\r\n",
                 reason, (unsigned int)pivot_start_yaw,
                 (unsigned int)imu_yaw_x100, (long)pivot_delta_x100());
#endif
  send_text(line);
#ifdef ROUTE_REPLAY
  route_after_step(reason);
#endif
}

static void pivot_start(uint32_t now)
{
  char line[80];
  if (pivot_active
#ifndef PIVOT_MULTI
      || pivot_finished
#endif
      ) {
    send_text("EVENT,REFUSE,already_run\r\n"); return;
  }
#ifdef PIVOT_MULTI
  if (pivot_last_stop_ms && now - pivot_last_stop_ms < 800U) {
    send_text("EVENT,REFUSE,cooldown\r\n"); return;
  }
#endif
  if (imu_frames == 0U || sample_age_ms(now, imu_last_ms) > 150U) {
    send_text("EVENT,REFUSE,imu_missing\r\n"); return;
  }
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) {
    send_text("EVENT,REFUSE,key_pressed\r\n"); return;
  }
  pivot_start_yaw = imu_yaw_x100;
#ifdef PIVOT_MULTI
  segment_linear = 0U;
#endif
  pivot_start_ms = pivot_phase_ms = pivot_progress_ms = now;
  pivot_last_progress = 0;
  pivot_phase = pivot_pulses = 0U;
#ifdef PIVOT_MULTI
  pivot_step_id++;
  pivot_encoder_start[0] = enc_count(&enc_a);
  pivot_encoder_start[1] = enc_count(&enc_b);
  pivot_encoder_start[2] = enc_count(&enc_c);
  pivot_encoder_start[3] = enc_count(&enc_d);
#endif
  pivot_active = 1U;
#ifdef PIVOT_MULTI
  (void)snprintf(line, sizeof(line),
                 "EVENT,STEP_START,%lu,%c,%ld,%u\r\n",
                 (unsigned long)pivot_step_id,
                 pivot_sign < 0 ? 'L' : 'R',
                 (long)pivot_target_x100, (unsigned int)pivot_start_yaw);
#else
  (void)snprintf(line, sizeof(line),
                 "EVENT,PIVOT_START,target_delta_x100=%ld\r\n",
                 (long)(pivot_sign * pivot_target_x100));
#endif
  send_text(line);
  pivot_drive(200);
}

#ifdef PIVOT_MULTI
static void linear_start(uint32_t now, int8_t direction, int32_t distance_mm)
{
  char line[90];
  if (pivot_active) { send_text("EVENT,REFUSE,busy\r\n"); return; }
  if (pivot_last_stop_ms && now - pivot_last_stop_ms < 800U) {
    send_text("EVENT,REFUSE,cooldown\r\n"); return;
  }
  if (imu_frames == 0U || sample_age_ms(now, imu_last_ms) > 150U) {
    send_text("EVENT,REFUSE,imu_missing\r\n"); return;
  }
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) {
    send_text("EVENT,REFUSE,key_pressed\r\n"); return;
  }
  segment_linear = 1U;
  linear_direction = direction;
  linear_target_mm = distance_mm;
  linear_target_counts = (distance_mm * 573 + 50) / 100; /* 5.73 counts/mm. */
  pivot_start_yaw = imu_yaw_x100;
  pivot_start_ms = pivot_phase_ms = pivot_progress_ms = now;
  pivot_phase = pivot_pulses = 0U;
  linear_last_progress = 0;
  pivot_step_id++;
  pivot_encoder_start[0] = enc_count(&enc_a);
  pivot_encoder_start[1] = enc_count(&enc_b);
  pivot_encoder_start[2] = enc_count(&enc_c);
  pivot_encoder_start[3] = enc_count(&enc_d);
  pivot_active = 1U;
  (void)snprintf(line, sizeof(line),
                 "EVENT,STEP_START,%lu,%c,%ld,%u,target_counts=%ld\r\n",
                 (unsigned long)pivot_step_id,
                 direction > 0 ? 'F' : 'B', (long)distance_mm,
                 (unsigned int)pivot_start_yaw, (long)linear_target_counts);
  send_text(line);
  linear_drive(180);
}
#ifdef ROUTE_REPLAY
static void route_abort(const char *reason)
{
  char line[100];
  if (!route_active) return;
  stop_motors();
  route_active = route_waiting = 0U;
  (void)snprintf(line, sizeof(line),
                 "EVENT,ROUTE_ABORT,%u,%s\r\n",
                 (unsigned int)(route_index + 1U), reason);
  send_text(line);
}

static void route_after_step(const char *reason)
{
  char line[80];
  if (!route_active) return;
  if (strcmp(reason, "DONE") != 0 &&
      strcmp(reason, "OVERSHOOT") != 0 &&
      strcmp(reason, "DISTANCE_OVERSHOOT") != 0) {
    route_abort(reason);
    return;
  }
  if (route_index == 6U)
    send_text("EVENT,ROUTE_PARKED,7\r\n");
  if ((size_t)(route_index + 1U) == ROUTE_STEP_COUNT) {
    route_active = route_waiting = 0U;
    (void)snprintf(line, sizeof(line),
                   "EVENT,ROUTE_DONE,%lu,%u\r\n",
                   (unsigned long)(HAL_GetTick() - route_start_ms),
                   (unsigned int)ROUTE_STEP_COUNT);
    send_text(line);
    return;
  }
  route_index++;
  route_waiting = 1U;
  (void)snprintf(line, sizeof(line),
                 "EVENT,ROUTE_NEXT,%u\r\n",
                 (unsigned int)(route_index + 1U));
  send_text(line);
}

static void route_start_step(uint32_t now)
{
  const RouteStep *step = &route_steps[route_index];
  char line[90];
  route_waiting = 0U;
  (void)snprintf(line, sizeof(line),
                 "EVENT,ROUTE_STEP,%u,source=%u,%c,%u\r\n",
                 (unsigned int)(route_index + 1U),
                 (unsigned int)step->source_step,
                 step->direction, (unsigned int)step->value);
  send_text(line);
  if (step->direction == 'F' || step->direction == 'B') {
    linear_start(now, step->direction == 'F' ? 1 : -1, step->value);
  } else {
    pivot_sign = step->direction == 'L' ? -1 : 1;
    pivot_target_x100 = (int32_t)step->value * 100;
    pivot_start(now);
  }
  if (!pivot_active) route_abort("STEP_START_FAILED");
}

static void route_begin(uint32_t now)
{
  char line[80];
  if (route_active || pivot_active) return;
  if (imu_frames == 0U || sample_age_ms(now, imu_last_ms) > 150U) {
    send_text("EVENT,REFUSE,imu_missing\r\n"); return;
  }
  if (pivot_last_stop_ms && now - pivot_last_stop_ms < 800U) {
    send_text("EVENT,REFUSE,cooldown\r\n"); return;
  }
  route_active = 1U;
  route_waiting = route_index = 0U;
  route_start_ms = now;
  (void)snprintf(line, sizeof(line),
                 "EVENT,ROUTE_START,%u,no_gray\r\n",
                 (unsigned int)ROUTE_STEP_COUNT);
  send_text(line);
  route_start_step(now);
}

static void route_read_key(uint32_t now)
{
  static uint8_t old_raw = 1U, stable = 1U, stop_consumed;
  static uint32_t change_ms;
  uint8_t raw = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET;
  if (raw != old_raw) {
    old_raw = raw;
    change_ms = now;
    if (!raw && route_active) {
      stop_consumed = 1U;
      if (pivot_active) pivot_stop("KEY");
      else route_abort("KEY");
    }
  }
  if (raw == stable || now - change_ms < 25U) return;
  stable = raw;
  if (!stable) {
    if (route_active && !pivot_active) route_abort("KEY");
  } else if (stop_consumed) stop_consumed = 0U;
  else route_begin(now);
}

static void route_tick(uint32_t now)
{
  if (!route_active) return;
  if (now - route_start_ms > 60000U) {
    if (pivot_active) pivot_stop("ROUTE_TIMEOUT");
    else route_abort("ROUTE_TIMEOUT");
    return;
  }
  if (route_waiting && now - pivot_last_stop_ms >= 800U) {
    if (imu_frames == 0U || sample_age_ms(now, imu_last_ms) > 150U)
      route_abort("IMU_STALE");
    else route_start_step(now);
  }
}
#endif
#endif

static void pivot_read_dap(uint32_t now)
{
#ifdef PIVOT_MULTI
  if (pivot_rx_tail == pivot_rx_head) return;
  uint8_t command = pivot_rx_buffer[pivot_rx_tail];
  pivot_rx_tail = (uint8_t)((pivot_rx_tail + 1U) & 15U);
#else
  uint32_t status = USART1->SR;
  if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
    (void)USART1->DR; return;
  }
  if (!(status & USART_SR_RXNE)) return;
  uint8_t command = (uint8_t)USART1->DR;
#endif
  if (command == 'X' || command == 'x') {
#ifdef PIVOT_MULTI
    pivot_command_length = 0U;
#endif
    if (pivot_active) pivot_stop("REMOTE_X");
#ifdef ROUTE_REPLAY
    else if (route_active) route_abort("REMOTE_X");
#endif
    else send_text("ACK,X,already_stopped\r\n");
  }
#ifdef PIVOT_MULTI
  else if (command == '\r') return;
  else if (command == '\n') {
    pivot_command[pivot_command_length] = '\0';
    if (pivot_command_length == 1U &&
        (pivot_command[0] == 'Q' || pivot_command[0] == 'q')) {
      send_text("ACK,Q,uart_rx_ok\r\n");
#ifdef ROUTE_REPLAY
    } else if (pivot_command_length) {
      send_text("EVENT,REFUSE,route_mode\r\n");
    }
#else
    } else if (pivot_command_length == 4U &&
               (pivot_command[0] == 'L' || pivot_command[0] == 'R' ||
                pivot_command[0] == 'F' || pivot_command[0] == 'B') &&
               pivot_command[1] >= '0' && pivot_command[1] <= '9' &&
               pivot_command[2] >= '0' && pivot_command[2] <= '9' &&
               pivot_command[3] >= '0' && pivot_command[3] <= '9') {
      int32_t angle = (pivot_command[1] - '0') * 100 +
                      (pivot_command[2] - '0') * 10 +
                      (pivot_command[3] - '0');
      if (pivot_active) send_text("EVENT,REFUSE,busy\r\n");
      else if (pivot_command[0] == 'F' || pivot_command[0] == 'B') {
        if (angle < 10 || angle > 500)
          send_text("EVENT,REFUSE,distance_range_10_to_500_mm\r\n");
        else linear_start(now, pivot_command[0] == 'F' ? 1 : -1, angle);
      } else if (angle < 5 || angle > 120)
        send_text("EVENT,REFUSE,angle_range_5_to_120\r\n");
      else {
        pivot_sign = pivot_command[0] == 'L' ? -1 : 1;
        pivot_target_x100 = angle * 100;
        pivot_start(now);
      }
    } else if (pivot_command_length) send_text("EVENT,REFUSE,bad_command\r\n");
#endif
    pivot_command_length = 0U;
  } else if (pivot_command_length < sizeof(pivot_command) - 1U) {
    pivot_command[pivot_command_length++] = (char)command;
  } else {
    pivot_command_length = 0U;
    send_text("EVENT,REFUSE,command_too_long\r\n");
  }
#else
  else if (command == 'Q' || command == 'q') {
    send_text("ACK,Q,uart_rx_ok\r\n");
#ifdef PIVOT_TEST_LEFT
  } else if (command == 'L' || command == 'l') pivot_start(now);
#else
  } else if (command == 'R' || command == 'r') pivot_start(now);
#endif
#endif
}

#ifdef PIVOT_MULTI
static void linear_control(uint32_t now)
{
  int32_t distance = linear_progress_counts();
  int32_t yaw_error = pivot_delta_x100();
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) {
    pivot_stop("KEY"); return;
  }
  if (sample_age_ms(now, imu_last_ms) > 150U) { pivot_stop("IMU_STALE"); return; }
  if (now - pivot_start_ms > 8000U) { pivot_stop("TIMEOUT"); return; }
  if (yaw_error > 800 || yaw_error < -800) {
    pivot_stop("HEADING_ERROR"); return;
  }
  if (distance > linear_target_counts + 80) {
    pivot_stop("DISTANCE_OVERSHOOT"); return;
  }
  if (now - pivot_progress_ms >= 600U) {
    if (pivot_phase < 2U && distance - linear_last_progress < 8) {
      pivot_stop("ENCODER_STALL"); return;
    }
    linear_last_progress = distance;
    pivot_progress_ms = now;
  }
  if (pivot_phase == 0U) {
    if (distance >= linear_target_counts - 170) {
      pivot_phase = 1U; linear_drive(135);
    } else linear_drive(180);
  } else if (pivot_phase == 1U) {
    if (distance >= linear_target_counts - 45) {
      stop_motors(); pivot_phase = 2U; pivot_phase_ms = now;
    } else linear_drive(135);
  } else if (pivot_phase == 2U && now - pivot_phase_ms >= 250U) {
    if (distance >= linear_target_counts - 20 || pivot_pulses >= 3U)
      pivot_stop(distance > linear_target_counts + 30 ? "DISTANCE_OVERSHOOT" :
                                                     "DONE");
    else {
      linear_drive(135); pivot_pulses++;
      pivot_phase = 3U; pivot_phase_ms = now;
    }
  } else if (pivot_phase == 3U && now - pivot_phase_ms >= 80U) {
    stop_motors(); pivot_phase = 2U; pivot_phase_ms = now;
  }
}
#endif

static void pivot_control(uint32_t now)
{
  if (!pivot_active) return;
#ifdef PIVOT_MULTI
  if (segment_linear) { linear_control(now); return; }
#endif
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) {
    pivot_stop("KEY"); return;
  }
  if (sample_age_ms(now, imu_last_ms) > 150U) { pivot_stop("IMU_STALE"); return; }
  if (now - pivot_start_ms > 6000U) { pivot_stop("TIMEOUT"); return; }
  int32_t progress = pivot_progress_x100();
  if (progress < -300 || progress > pivot_target_x100 + 1000) {
    pivot_stop("WRONG_DIRECTION_OR_OVERSHOOT"); return;
  }
  if (now - pivot_progress_ms >= 800U) {
    if (pivot_phase < 2U && progress - pivot_last_progress < 50) {
      pivot_stop("NO_YAW_PROGRESS"); return;
    }
    pivot_last_progress = progress;
    pivot_progress_ms = now;
  }
  if (pivot_phase == 0U) {
    if (progress >= pivot_target_x100 - 2000) {
      pivot_phase = 1U; pivot_drive(135);
    }
  } else if (pivot_phase == 1U) {
    if (progress >= pivot_target_x100 - 400) {
      stop_motors(); pivot_phase = 2U; pivot_phase_ms = now;
    }
  } else if (pivot_phase == 2U && now - pivot_phase_ms >= 250U) {
    if (progress >= pivot_target_x100 - 200 || pivot_pulses >= 3U) {
      pivot_stop(progress > pivot_target_x100 + 200 ? "OVERSHOOT" : "DONE");
    } else {
      pivot_drive(135); pivot_pulses++;
      pivot_phase = 3U; pivot_phase_ms = now;
    }
  } else if (pivot_phase == 3U && now - pivot_phase_ms >= 70U) {
    stop_motors(); pivot_phase = 2U; pivot_phase_ms = now;
  }
}

static void pivot_report(uint32_t now)
{
  char line[200];
#ifdef PIVOT_MULTI
  (void)snprintf(line, sizeof(line),
                 "S,%lu,%u,%lu,%c,%ld,%ld,%u,%u,%ld,%d,%d,%d,%d,%d,%d,%lu,%lu\r\n",
                 (unsigned long)(pivot_active ? now - pivot_start_ms : 0U),
                 (unsigned int)pivot_active, (unsigned long)pivot_step_id,
                 segment_linear ? (linear_direction > 0 ? 'F' : 'B') :
                                  (pivot_sign < 0 ? 'L' : 'R'),
                 (long)(segment_linear ? linear_target_mm : pivot_target_x100),
                 (long)(pivot_active ?
                        (segment_linear ? linear_progress_counts() :
                                          pivot_progress_x100()) : 0),
                 (unsigned int)pivot_start_yaw, (unsigned int)imu_yaw_x100,
                 (long)pivot_delta_x100(), (int)left_command,
                 (int)right_command,
                 (int)(int16_t)(enc_count(&enc_a) - pivot_encoder_start[0]),
                 (int)(int16_t)(enc_count(&enc_b) - pivot_encoder_start[1]),
                 (int)(int16_t)(enc_count(&enc_c) - pivot_encoder_start[2]),
                 (int)(int16_t)(enc_count(&enc_d) - pivot_encoder_start[3]),
                 (unsigned long)(imu_frames ? sample_age_ms(now, imu_last_ms) : 9999U),
                 (unsigned long)imu_frames);
#else
  (void)snprintf(line, sizeof(line),
                 "P,%lu,%u,%u,%u,%u,%ld,%d,%d,%lu,%lu\r\n",
                 (unsigned long)(pivot_active ? now - pivot_start_ms : 0U),
                 (unsigned int)pivot_active, (unsigned int)pivot_phase,
                 (unsigned int)pivot_start_yaw, (unsigned int)imu_yaw_x100,
                 (long)pivot_delta_x100(), (int)left_command,
                 (int)right_command,
                 (unsigned long)(imu_frames ? sample_age_ms(now, imu_last_ms) : 9999U),
                 (unsigned long)imu_frames);
#endif
  send_text(line);
}
#endif

int main(void)
{
  uint32_t last_control = 0U, last_report = 0U, last_motion = 0U;
  uint32_t last_sensor_sample = 0U, last_imu_start = 0U;
  static const uint8_t imu_full[7] = {0xAA,0x55,0x60,0x0B,0x01,0x00,0x6C};
  static const uint8_t imu_start[7] = {0xAA,0x55,0x60,0x0A,0x01,0x01,0x6C};
  HAL_Init();
  clock_init();
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
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
#ifdef PIVOT_TEST
#ifdef PIVOT_MULTI
#ifdef ROUTE_REPLAY
  send_text("READY,MANUAL_ROUTE_REPLAY,KEY_once=start_17_steps,X=stop,no_gray\r\n");
#else
  send_text("READY,SEGMENT_CONTROL,Q=query,L030=left30,R030=right30,F150=forward150mm,B150=reverse150mm,X=stop,KEY=stop\r\n");
#endif
  send_text("HEADER,EVENT,STEP_END,id,reason,dir,target_value,start_yaw,end_y,delta_x100,ms,progress,enc_A,enc_B,enc_C,enc_D\r\n");
#else
  {
    char line[100];
#ifdef PIVOT_TEST_LEFT
    (void)snprintf(line, sizeof(line),
                   "READY,PIVOT_LEFT_%u,Q=query,L=start_once,X=stop,KEY=stop\r\n",
                   (unsigned int)(PIVOT_TARGET_X100 / 100));
#else
    (void)snprintf(line, sizeof(line),
                   "READY,PIVOT_RIGHT_%u,Q=query,R=start_once,X=stop,KEY=stop\r\n",
                   (unsigned int)(PIVOT_TARGET_X100 / 100));
#endif
    send_text(line);
  }
#endif
#ifdef PIVOT_MULTI
  send_text("HEADER,S,ms,active,step_id,dir,target_value,progress,start_yaw,yaw,delta_yaw,left,right,enc_A,enc_B,enc_C,enc_D,imu_age,imu_frames\r\n");
#else
  send_text("HEADER,P,elapsed_ms,active,phase,start_yaw_x100,yaw_x100,delta_x100,left,right,imu_age,imu_frames\r\n");
#endif
#else
#ifdef STOP_AFTER_ONE_CORNER
#ifdef GIT_VERIFIED_CORNER
  send_text("READY,GIT_VERIFIED_ONE_CORNER_TAG_D8B2F7E,G=start_if_centered,KEY=start_or_stop,X=stop\r\n");
#else
  send_text("READY,ONE_CORNER_STOP,G=start_if_centered,KEY=start_or_stop,X=stop\r\n");
#endif
#else
  send_text(REVERSE_RECORD_MODE ?
            "READY,REVERSE_LEFT_ADJUST_RECORD,KEY_start_and_stop; X=STOP\r\n" :
            S_CURVE_RECORD_MODE ?
            "READY,S_CURVE_FORWARD_RECORD,KEY_start_and_stop; X=STOP\r\n" :
            FORWARD_REVERSE_REFERENCE_MODE ?
            "READY,FORWARD_REVERSE_REFERENCE,KEY_start_and_stop; X=STOP\r\n" :
            "READY,TWO_LAP_PARKING_LINE_GATE,KEY_start,8_corners_auto_stop,X=stop\r\n");
#endif
  send_text("HEADER,D,ms,state,gray,gray_age,gray_error,A,B,C,D,left,right,turns,yaw_x100,pitch_x100,roll_x100,gyro_z_raw,imu_age,imu_frames\r\n");
#endif
  (void)HAL_UART_Transmit(&imu_uart, (uint8_t *)imu_full, 7U, 20U);
  (void)HAL_UART_Transmit(&imu_uart, (uint8_t *)imu_start, 7U, 20U);
  last_imu_start = HAL_GetTick();
  while (1) {
    uint32_t now = HAL_GetTick();
#ifdef PIVOT_TEST
#ifdef ROUTE_REPLAY
    route_read_key(now);
#endif
    pivot_read_dap(now);
    if (imu_frames == 0U && now - last_imu_start >= 2000U) {
      (void)HAL_UART_Transmit(&imu_uart, (uint8_t *)imu_full, 7U, 20U);
      (void)HAL_UART_Transmit(&imu_uart, (uint8_t *)imu_start, 7U, 20U);
      last_imu_start = now;
    }
    if (now - last_control >= CONTROL_MS) {
      last_control = now;
      pivot_control(now);
#ifdef ROUTE_REPLAY
      route_tick(now);
#endif
    }
    if (now - last_report >= REPORT_MS) {
      last_report = now;
      pivot_report(now);
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
}
