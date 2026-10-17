#include "main.h"

/* Common TB6612 standby: high = enabled, low = stopped. */
#define MOTOR_STBY_PORT GPIOB
#define MOTOR_STBY_PIN GPIO_PIN_15
#include <stdio.h>
#include <string.h>
#include "common/map_features.h"
#include "entry_plan.h"
#include "parking_controller.h"
Task4Plan parking_plan;
static uint8_t parking_lines_on_edge[4],entry_finished;
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
#define MAX_RUN_MS 45000U
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
static uint8_t reference_run_used;
static uint8_t auto_phase,entry_confirmed;
static uint32_t auto_pause_start_ms;
static uint8_t parking_feature_hits;
static int32_t parking_corner_progress;
static uint8_t pivot_active;
static uint16_t pivot_start_yaw;
static uint32_t pivot_last_stop_ms,pivot_start_ms;
static int16_t pivot_encoder_start[4];

static void report(uint32_t now);
static void combined_abort(const char *reason);
static uint8_t transit_lines_complete(void);

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

static void stop_run(const char *reason)
{
  char line[80];
  if (state == STOPPED) return;
  stop_motors();
  state = STOPPED;
  if(auto_phase==1U)auto_phase=4U;
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
  if(!task4_plan_valid(&parking_plan)){send_text("EVENT,REFUSE_START,invalid_plan\r\n");return;}
  if(auto_phase!=0U || pivot_active){send_text("EVENT,REFUSE_START,busy\r\n");return;}
  if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==GPIO_PIN_RESET){send_text("EVENT,REFUSE_START,key_pressed\r\n");return;}
  if (reference_run_used) { send_text("EVENT,REFUSE_START,once_per_reset\r\n"); return; }
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
  reference_run_used = 1U;
  auto_phase=1U;entry_confirmed=0U;
  memset(parking_lines_on_edge,0,sizeof(parking_lines_on_edge));entry_finished=0U;
  parking_feature_hits=0U;parking_corner_progress=0L;
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
  send_text("EVENT,START," ENTRY_NAME ",VISION_SELECTED\r\n");
  record_corner_event("ORIGIN", 0U, NO_TURN, now);
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
#include "common/navigation.inc"
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
/* USART3 is not used; the application owns the vector entry. */
#ifndef PIVOT_MULTI
/* USART1 receive handling belongs to the vision application in main.c. */
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

void parking_imu_irq(void)
{
  uint32_t status = UART4->SR;
  if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
    (void)UART4->DR; imu_state = 0U;
  } else if (status & USART_SR_RXNE) imu_parse((uint8_t)UART4->DR);
}


#include "common/entry_motion.inc"
#include "common/entry_workflow.inc"

static uint32_t last_sample,last_control,last_report,last_motion,last_imu;
static const uint8_t imu_full[7]={0xAA,0x55,0x60,0x0B,0x01,0x00,0x6C};
static const uint8_t imu_start[7]={0xAA,0x55,0x60,0x0A,0x01,0x01,0x6C};
void parking_clock_init(void){clock_init();}
void parking_init(void)
{
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
  (void)HAL_UART_Transmit(&imu_uart,(uint8_t*)imu_full,7U,20U);
  (void)HAL_UART_Transmit(&imu_uart,(uint8_t*)imu_start,7U,20U);
  last_imu=HAL_GetTick();
}
void parking_tick(void)
{
    uint32_t now=HAL_GetTick();
    if(now-last_sample>=5U){sensor_sample();last_sample=now;}
    if(!imu_frames && now-last_imu>=2000U){
      (void)HAL_UART_Transmit(&imu_uart,(uint8_t*)imu_full,7U,20U);
      (void)HAL_UART_Transmit(&imu_uart,(uint8_t*)imu_start,7U,20U);
      last_imu=now;
    }
    if(auto_phase>0U && auto_phase<4U && now-run_start_ms>90000U)
      combined_abort("JOB_TIMEOUT");
    if(now-last_control>=10U){
      last_control=now;
      if(auto_phase==1U && state!=STOPPED)control(now);
      reference_control(now);combined_tick(now);
    }
    if(auto_phase==1U && now-last_motion>=100U){
      check_encoder_motion(now);last_motion=now;
    }
    if(now-last_report>=50U){
      last_report=now;
      if(auto_phase<3U || reference_next==0U)report(now);else reference_report(now);
    }
    if(auto_phase==0U || auto_phase==4U)stop_motors();
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,
      ((now/(auto_phase==0U || auto_phase==4U?500U:100U))&1U)?GPIO_PIN_SET:GPIO_PIN_RESET);
}
uint8_t parking_configure(const char *color,const char *shape,const char *action)
{
  if(reference_run_used || state!=STOPPED || pivot_active || auto_phase==2U || auto_phase==3U)return 0U;
  if(!task4_make_plan(color,shape,action,&parking_plan))return 0U;
  reference_next=reference_running=reference_phase=reference_halted=0U;
  pivot_last_stop_ms=0U;auto_phase=0U;entry_finished=0U;
  return 1U;
}
const Task4Plan *parking_current_plan(void){return &parking_plan;}
uint8_t parking_preflight(void)
{
  uint32_t now=HAL_GetTick();
  return task4_plan_valid(&parking_plan) && gray_frames && sample_age_ms(now,gray_last_ms)<=100U &&
    (gray_value&0x18U) && active_count(gray_value)<=3U && imu_frames &&
    sample_age_ms(now,imu_last_ms)<=150U && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==GPIO_PIN_SET;
}
uint8_t parking_busy(void){return state!=STOPPED || pivot_active || auto_phase==2U || auto_phase==3U;}
uint8_t parking_start(void){if(!parking_preflight())return 0U;start_run();return parking_busy();}
uint8_t parking_done(void){return entry_finished;}
void parking_stop(void)
{
  stop_motors();state=STOPPED;pivot_active=0U;reference_halted=1U;auto_phase=4U;
}
