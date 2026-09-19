#include "main.h"
#include "menu.h"
#include "menu_display.h"
#include "route_recorder.h"
#include <stdio.h>
#include <string.h>

/* CD4051 gray sensor: AD0...AD2 select one channel, OUT returns its level. */
#define SENSOR_PORT GPIOB
#define SENSOR_AD0_PIN GPIO_PIN_0
#define SENSOR_AD1_PIN GPIO_PIN_1
#define SENSOR_AD2_PIN GPIO_PIN_2
#define SENSOR_OUT_PIN GPIO_PIN_10
/* The actual track is detected when the module indicator is on. */
#define BLACK_LINE_LEVEL GPIO_PIN_SET
#define SENSOR_SWITCH_DELAY_US 50U
#define SENSOR_LEFT_HALF_MASK 0x0FU
#define SENSOR_RIGHT_HALF_MASK 0xF0U
#define SENSOR_CENTER_MASK 0x18U
/* CH7 is physically the second probe from the right and is stuck active. */
#define SENSOR_FAULTY_MASK (1U << 6)
#define SENSOR_RIGHT_VALID_COUNT 3U

/* TIM1 runs four PWM outputs at about 20 kHz. */
#define MOTOR_PWM_PERIOD 8399U

/* Speeds use a 0...1000 scale. Tune these after the first low-speed run. */
#define FOLLOW_SPEED 220
#define FOLLOW_KP 22
#define FOLLOW_MAX_CORRECTION 100
#define CORNER_ENTER_SPEED 180
#define TURN_PIVOT_SPEED 220
#define CORNER_ENTER_TIME_MS 100U
#define TURN_MIN_TIME_MS 120U
#define TURN_TIMEOUT_MS 1200U
#define TURN_RECOVER_TIME_MS 100U
#define CORNER_CONFIRM_COUNT 3U
#define CENTER_CONFIRM_COUNT 3U
#define REACQUIRE_MAX_ACTIVE 2U
#define LAP_TURN_LIMIT 4U
#define CONTROL_PERIOD_MS 5U
/* Button menu: a short press changes task; a 3 s hold confirms it. */
#define KEY_SHORT_PRESS_MAX_MS 1000U
#define KEY_LONG_PRESS_MS 3000U
#define MENU_LED_FLASH_MS 160U

/* Side parking demo: drive straight first, then run the parking motion. */
#define PARK_SLOT_ON_RIGHT 1U
#define PARK_FOLLOW_SPEED 160
#define PARK_DRIVE_SPEED 150
#define PARK_INNER_SPEED 110
#define PARK_OUTER_SPEED 190
#define PARK_STRAIGHT_AHEAD_COUNTS 220U
#define PARK_TURN_IN_COUNTS 340U
#define PARK_STRAIGHTEN_COUNTS 340U
#define PARK_FINAL_ADJUST_COUNTS 85U
#define PARK_ENCODER_STALL_MS 300U
#define PARK_STEP_TIMEOUT_MS 5000U

/* Task 3: follow A->B->BC, then replay the taught BC parking movement. */
#define REVERSE_PARK_TURN_COUNT 1U
#define REVERSE_PARK_FOLLOW_SPEED 180
#define REVERSE_PARK_ALIGN_LEFT_SPEED 125
#define REVERSE_PARK_ALIGN_RIGHT_SPEED 165
#define REVERSE_PARK_BACK_LEFT_SPEED 165
#define REVERSE_PARK_BACK_RIGHT_SPEED 140
/* Verified from yesterday's BC teaching record; used if Flash cannot be read. */
#define REVERSE_PARK_FALLBACK_APPROACH_COUNTS 237U
#define REVERSE_PARK_FALLBACK_ALIGN_COUNTS 2485U
#define REVERSE_PARK_FALLBACK_BACK_IN_COUNTS 2699U

/* T4 UART motor and encoder test: 115200 bps. */
#define DIAGNOSTIC_SPEED 180
#define DIAGNOSTIC_REPORT_MS 200U
#define DIAGNOSTIC_AUTO_START 0U

/* This direct-key build has no menu: one short PA15 press starts reverse parking. */
#define SIDE_PARKING_DEMO_ON_KEY 0U
#define ROUTE_RECORDER_ON_KEY 0U
#define REVERSE_PARK_DEMO_ON_KEY 1U

typedef enum {
  CAR_STOPPED,
  CAR_FOLLOWING,
  CAR_ENTERING_CORNER,
  CAR_TURNING,
  CAR_RECOVERING,
  CAR_PARK_STRAIGHT_AHEAD,
  CAR_PARK_REVERSE_TURN_IN,
  CAR_PARK_REVERSE_STRAIGHTEN,
  CAR_PARK_FINAL_ADJUST,
  CAR_REVERSE_PARK_APPROACH,
  CAR_REVERSE_PARK_ALIGN,
  CAR_REVERSE_PARK_BACK_IN,
  CAR_DIAGNOSTIC
} CarState;

typedef enum {
  TASK_NORMAL_LINE,
  TASK_SIDE_PARKING,
  TASK_REVERSE_PARKING,
  TASK_DIAGNOSTIC
} TaskMode;

typedef enum {
  KEY_EVENT_NONE,
  KEY_EVENT_SHORT,
  KEY_EVENT_LONG
} KeyEvent;

typedef enum {
  CORNER_NONE,
  CORNER_LEFT,
  CORNER_RIGHT
} CornerDirection;

static TIM_HandleTypeDef htim1;
static TIM_HandleTypeDef htim2;
static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim4;
static TIM_HandleTypeDef htim8;
static UART_HandleTypeDef huart1;
static CarState car_state = CAR_STOPPED;
static TaskMode task_mode = TASK_NORMAL_LINE;
static CornerDirection turn_direction = CORNER_NONE;
static CornerDirection candidate_corner = CORNER_NONE;
static uint8_t corner_hits = 0U;
static uint8_t center_hits = 0U;
static uint8_t completed_turns = 0U;
static uint32_t state_start_tick = 0U;
static uint32_t parking_last_progress = 0U;
static uint32_t parking_progress_tick = 0U;
static uint8_t menu_led_toggles_remaining = 0U;
static uint8_t menu_led_is_on = 0U;
static uint32_t menu_led_tick = 0U;
static uint8_t ignore_stop_key_event = 0U;
static uint32_t diagnostic_report_tick = 0U;
static uint32_t reverse_park_approach_counts = 0U;
static uint32_t reverse_park_align_counts = 0U;
static uint32_t reverse_park_back_in_counts = 0U;

static void Car_Stop(void);
static void Status_LED(uint8_t on);
static void Diagnostic_Send(const char *text);

/* The three taught routes are retained in Flash sector 11 by route_recorder.c. */
#define ROUTE_FLASH_ADDRESS 0x080E0000U
#define ROUTE_STORE_MAGIC 0x52544D31U
#define ROUTE_SLOT_MAGIC 0x52544D32U
#define ROUTE_MAX_POINTS 11U

typedef struct {
  int32_t left_front;
  int32_t left_rear;
  int32_t right_rear;
} RecordedRoutePoint;

typedef struct {
  uint32_t valid;
  uint32_t point_count;
  RecordedRoutePoint point[ROUTE_MAX_POINTS];
  uint32_t checksum;
} RecordedRouteSlot;

typedef struct {
  uint32_t magic;
  uint32_t version;
  RecordedRouteSlot slot[3];
} RecordedRouteStore;

static uint8_t Key_Is_Pressed(void)
{
  return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) ? 1U : 0U;
}

/*
 * Before the OLED is installed, PC13 gives a simple menu indication:
 * Task N is shown as N LED flashes. OLED code can instead display the
 * values exported by menu.h; it does not need to change motor code.
 */
static void Menu_Indicate_Task(MenuTaskId task, uint32_t now)
{
  menu_led_toggles_remaining = (uint8_t)((uint8_t)task * 2U);
  menu_led_is_on = 0U;
  menu_led_tick = now - MENU_LED_FLASH_MS;
  Status_LED(0U);
}

static void Menu_Process_Indicator(uint32_t now)
{
  if (menu_led_toggles_remaining == 0U) return;
  if (now - menu_led_tick < MENU_LED_FLASH_MS) return;

  menu_led_tick = now;
  menu_led_is_on = menu_led_is_on ? 0U : 1U;
  Status_LED(menu_led_is_on);
  menu_led_toggles_remaining--;

  if (menu_led_toggles_remaining == 0U) {
    Status_LED(0U);
  }
}

static void Error_Stop(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                              GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
                    GPIO_PIN_RESET);
  while (1) { }
}

static void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = 8;
  osc.PLL.PLLN = 336;
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Stop();

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV4;
  clk.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) Error_Stop();
  SystemCoreClockUpdate();
}

static void GPIO_Init_All(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* PC13 LED is active low. PC0 is the common TB6612 STBY signal. */
  HAL_GPIO_WritePin(GPIOC, LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
  gpio.Pin = LED_Pin | GPIO_PIN_0;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio);

  /* Motor directions: A=PD0/PD1, B=PD2/PD3, C=PD4/PD5, D=PD6/PD7. */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                              GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
                    GPIO_PIN_RESET);
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
             GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &gpio);

  /* PB0...PB2 select CD4051 channels. PB10 reads the direct OUT signal. */
  HAL_GPIO_WritePin(SENSOR_PORT, SENSOR_AD0_PIN | SENSOR_AD1_PIN | SENSOR_AD2_PIN,
                    GPIO_PIN_RESET);
  gpio.Pin = SENSOR_AD0_PIN | SENSOR_AD1_PIN | SENSOR_AD2_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SENSOR_PORT, &gpio);

  gpio.Pin = SENSOR_OUT_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SENSOR_PORT, &gpio);

  /* Four GT50 Hall encoders, one timer for each wheel. */
  /* Right front: TIM3 CH1/CH2 = PB4/PB5. */
  gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &gpio);

  /* Right rear: TIM2 CH1/CH2 = PA0/PA1. */
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* Left rear: TIM8 CH1/CH2 = PC6/PC7. */
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(GPIOC, &gpio);

  /* PWM: A=PE9, B=PE11, C=PE13, D=PE14, all from TIM1. */
  gpio.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &gpio);

  /* Board KEY is PA15, low when pressed. */
  gpio.Pin = GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* ST-Link virtual COM port: PA9=USART1_TX, PA10=USART1_RX. */
  gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &gpio);
}

static void PWM_Init_All(void)
{
  TIM_OC_InitTypeDef pwm = {0};

  __HAL_RCC_TIM1_CLK_ENABLE();
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = MOTOR_PWM_PERIOD;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Stop();

  pwm.OCMode = TIM_OCMODE_PWM1;
  pwm.Pulse = 0;
  pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
  pwm.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &pwm, TIM_CHANNEL_1) != HAL_OK) Error_Stop();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &pwm, TIM_CHANNEL_2) != HAL_OK) Error_Stop();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &pwm, TIM_CHANNEL_3) != HAL_OK) Error_Stop();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &pwm, TIM_CHANNEL_4) != HAL_OK) Error_Stop();

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Stop();
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) Error_Stop();
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) Error_Stop();
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK) Error_Stop();
}

static void Encoder_Init_All(void)
{
  TIM_Encoder_InitTypeDef encoder = {0};

  encoder.EncoderMode = TIM_ENCODERMODE_TI12;
  encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
  encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  encoder.IC1Prescaler = TIM_ICPSC_DIV1;
  encoder.IC1Filter = 8U;
  encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
  encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  encoder.IC2Prescaler = TIM_ICPSC_DIV1;
  encoder.IC2Filter = 8U;

  __HAL_RCC_TIM2_CLK_ENABLE();
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0U;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  /* Use a signed 16-bit range like the other wheel counters. */
  htim2.Init.Period = 0xFFFFU;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Encoder_Init(&htim2, &encoder) != HAL_OK) Error_Stop();
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK) Error_Stop();

  __HAL_RCC_TIM3_CLK_ENABLE();
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0U;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFFU;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Encoder_Init(&htim3, &encoder) != HAL_OK) Error_Stop();
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) Error_Stop();

  __HAL_RCC_TIM4_CLK_ENABLE();
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0U;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFFU;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Encoder_Init(&htim4, &encoder) != HAL_OK) Error_Stop();
  if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK) Error_Stop();

  __HAL_RCC_TIM8_CLK_ENABLE();
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0U;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0xFFFFU;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Encoder_Init(&htim8, &encoder) != HAL_OK) Error_Stop();
  if (HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL) != HAL_OK) Error_Stop();
}

static void UART1_Init(void)
{
  __HAL_RCC_USART1_CLK_ENABLE();
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200U;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Stop();
}

static void Microsecond_Delay_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void Status_LED(uint8_t on)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static int16_t Limit_Speed(int16_t speed)
{
  if (speed > 1000) return 1000;
  if (speed < -1000) return -1000;
  return speed;
}

static uint16_t Speed_To_PWM(int16_t speed)
{
  uint16_t absolute_speed;

  if (speed < 0) absolute_speed = (uint16_t)(-speed);
  else absolute_speed = (uint16_t)speed;
  if (absolute_speed > 1000U) absolute_speed = 1000U;
  return (uint16_t)((absolute_speed * MOTOR_PWM_PERIOD) / 1000U);
}

static void Motor_Set_One(uint16_t in1, uint16_t in2, uint32_t channel, int16_t speed)
{
  speed = Limit_Speed(speed);

  /* The verified chassis forward direction is IN1 low and IN2 high. */
  if (speed > 0) {
    HAL_GPIO_WritePin(GPIOD, in1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, in2, GPIO_PIN_SET);
  } else if (speed < 0) {
    HAL_GPIO_WritePin(GPIOD, in1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, in2, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(GPIOD, in1 | in2, GPIO_PIN_RESET);
  }
  __HAL_TIM_SET_COMPARE(&htim1, channel, Speed_To_PWM(speed));
}

static void Motor_Set_Left_Right(int16_t left_speed, int16_t right_speed)
{
  if (left_speed == 0 && right_speed == 0) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
  }

  Motor_Set_One(GPIO_PIN_0, GPIO_PIN_1, TIM_CHANNEL_1, left_speed);   /* A: left front */
  Motor_Set_One(GPIO_PIN_4, GPIO_PIN_5, TIM_CHANNEL_3, left_speed);   /* C: left rear */
  Motor_Set_One(GPIO_PIN_2, GPIO_PIN_3, TIM_CHANNEL_2, right_speed);  /* B: right front */
  Motor_Set_One(GPIO_PIN_6, GPIO_PIN_7, TIM_CHANNEL_4, right_speed);  /* D: right rear */
}

static void Motor_Stop_All(void)
{
  Motor_Set_Left_Right(0, 0);
}

static void Motor_Set_Test_One(uint8_t motor, int16_t speed)
{
  Motor_Stop_All();
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

  if (motor == 1U) {
    Motor_Set_One(GPIO_PIN_0, GPIO_PIN_1, TIM_CHANNEL_1, speed); /* Left front */
  } else if (motor == 2U) {
    Motor_Set_One(GPIO_PIN_2, GPIO_PIN_3, TIM_CHANNEL_2, speed); /* Right front */
  } else if (motor == 3U) {
    Motor_Set_One(GPIO_PIN_4, GPIO_PIN_5, TIM_CHANNEL_3, speed); /* Left rear */
  } else if (motor == 4U) {
    Motor_Set_One(GPIO_PIN_6, GPIO_PIN_7, TIM_CHANNEL_4, speed); /* Right rear */
  }
}

static uint32_t Encoder_Absolute_Count(TIM_HandleTypeDef *timer)
{
  int16_t count = (int16_t)__HAL_TIM_GET_COUNTER(timer);
  return (count < 0) ? (uint32_t)(-count) : (uint32_t)count;
}

static int32_t Encoder_Signed_Count(TIM_HandleTypeDef *timer)
{
  return (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(timer);
}

static uint32_t Encoder_Left_Count(void)
{
  uint32_t front = Encoder_Absolute_Count(&htim4);
  uint32_t rear = Encoder_Absolute_Count(&htim8);
  return (front + rear) / 2U;
}

static uint32_t Encoder_Right_Count(void)
{
  uint32_t front = Encoder_Absolute_Count(&htim3);
  uint32_t rear = Encoder_Absolute_Count(&htim2);
  return (front + rear) / 2U;
}

static uint32_t Encoder_Travel_Count(void)
{
  uint32_t left_front = Encoder_Absolute_Count(&htim4);
  uint32_t left_rear = Encoder_Absolute_Count(&htim8);
  uint32_t right_rear = Encoder_Absolute_Count(&htim2);

  /* Right-front E2 has no valid signal in the current chassis wiring. */
  return (left_front + left_rear + right_rear) / 3U;
}

static void Encoder_Reset_Distance(void)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0U);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_SET_COUNTER(&htim4, 0U);
  __HAL_TIM_SET_COUNTER(&htim8, 0U);
}

static uint32_t Count_Difference(int32_t first, int32_t second)
{
  return (first >= second) ? (uint32_t)(first - second) :
                             (uint32_t)(second - first);
}

static uint32_t Recorded_Segment_Count(const RecordedRouteSlot *slot,
                                       uint8_t first_point, uint8_t last_point)
{
  const RecordedRoutePoint *first = &slot->point[first_point];
  const RecordedRoutePoint *last = &slot->point[last_point];

  return (Count_Difference(first->left_front, last->left_front) +
          Count_Difference(first->left_rear, last->left_rear) +
          Count_Difference(first->right_rear, last->right_rear)) / 3U;
}

static void Reverse_Parking_Load_Record(void)
{
  const RecordedRouteStore *store = (const RecordedRouteStore *)ROUTE_FLASH_ADDRESS;
  const RecordedRouteSlot *slot = &store->slot[1]; /* Slot 2: BC reverse parking. */

  if (store->magic == ROUTE_STORE_MAGIC && store->version == 1U &&
      slot->valid == ROUTE_SLOT_MAGIC && slot->point_count >= 6U) {
    /* P2->P3: BC approach; P3->P4: forward arc; P4->P5: reverse arc. */
    reverse_park_approach_counts = Recorded_Segment_Count(slot, 2U, 3U);
    reverse_park_align_counts = Recorded_Segment_Count(slot, 3U, 4U);
    reverse_park_back_in_counts = Recorded_Segment_Count(slot, 4U, 5U);
  }

  /* Do not silently refuse to start when a recorded Flash word is invalid. */
  if (reverse_park_approach_counts == 0U || reverse_park_align_counts == 0U ||
      reverse_park_back_in_counts == 0U) {
    reverse_park_approach_counts = REVERSE_PARK_FALLBACK_APPROACH_COUNTS;
    reverse_park_align_counts = REVERSE_PARK_FALLBACK_ALIGN_COUNTS;
    reverse_park_back_in_counts = REVERSE_PARK_FALLBACK_BACK_IN_COUNTS;
  }
}

static void Parking_Set_State(CarState next_state, uint32_t now)
{
  car_state = next_state;
  state_start_tick = now;
  parking_last_progress = 0U;
  parking_progress_tick = now;
  Encoder_Reset_Distance();
}

static uint8_t Parking_Target_Reached(uint32_t target_count, CarState next_state,
                                      uint32_t now)
{
  uint32_t progress = Encoder_Travel_Count();

  if (progress >= target_count) {
    Parking_Set_State(next_state, now);
    return 1U;
  }

  if (progress != parking_last_progress) {
    parking_last_progress = progress;
    parking_progress_tick = now;
  }

  if (now - parking_progress_tick > PARK_ENCODER_STALL_MS ||
      now - state_start_tick > PARK_STEP_TIMEOUT_MS) {
    Car_Stop();
    return 1U;
  }
  return 0U;
}

static void Delay_Us(uint32_t microseconds)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = microseconds * (SystemCoreClock / 1000000U);

  while ((DWT->CYCCNT - start) < cycles) { }
}

static void Sensor_Select_Channel(uint8_t channel)
{
  HAL_GPIO_WritePin(SENSOR_PORT, SENSOR_AD0_PIN,
                    (channel & 0x01U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SENSOR_PORT, SENSOR_AD1_PIN,
                    (channel & 0x02U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SENSOR_PORT, SENSOR_AD2_PIN,
                    (channel & 0x04U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t Sensor_Read(void)
{
  uint8_t sensor = 0U;
  uint8_t channel;

  for (channel = 0U; channel < 8U; channel++) {
    Sensor_Select_Channel(channel);
    Delay_Us(SENSOR_SWITCH_DELAY_US);
    if (HAL_GPIO_ReadPin(SENSOR_PORT, SENSOR_OUT_PIN) == BLACK_LINE_LEVEL) {
      sensor |= (uint8_t)(1U << channel);
    }
  }
  /* Keep the failed CH7 from affecting error, corner, and lost-line logic. */
  return (uint8_t)(sensor & (uint8_t)~SENSOR_FAULTY_MASK);
}

static uint8_t Sensor_Count(uint8_t sensor)
{
  uint8_t count = 0U;
  uint8_t bit;

  for (bit = 0U; bit < 8U; bit++) {
    if (sensor & (1U << bit)) count++;
  }
  return count;
}

static int16_t Sensor_Error(uint8_t sensor)
{
  static const int8_t weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
  int16_t sum = 0;
  uint8_t count = 0U;
  uint8_t bit;

  for (bit = 0U; bit < 8U; bit++) {
    if (sensor & (1U << bit)) {
      sum += weight[bit];
      count++;
    }
  }
  if (count == 0U) return 0;
  return (int16_t)(sum / (int16_t)count);
}

static CornerDirection Detect_Corner(uint8_t sensor)
{
  uint8_t left_count = Sensor_Count(sensor & SENSOR_LEFT_HALF_MASK);
  uint8_t right_count = Sensor_Count(sensor & SENSOR_RIGHT_HALF_MASK);

  if (left_count == 4U && right_count <= 1U) return CORNER_LEFT;
  /* CH7 is masked, so the right half has only three working probes. */
  if (right_count == SENSOR_RIGHT_VALID_COUNT && left_count <= 1U) {
    return CORNER_RIGHT;
  }
  return CORNER_NONE;
}

static uint8_t Is_Ambiguous_Black_Area(uint8_t sensor)
{
  uint8_t left_count = Sensor_Count(sensor & SENSOR_LEFT_HALF_MASK);
  uint8_t right_count = Sensor_Count(sensor & SENSOR_RIGHT_HALF_MASK);
  return (left_count >= 3U && right_count >= 3U);
}

static uint8_t Is_Line_Reacquired(uint8_t sensor)
{
  return ((sensor & SENSOR_CENTER_MASK) != 0U &&
          Sensor_Count(sensor) <= REACQUIRE_MAX_ACTIVE);
}

static void Car_Stop(void)
{
  Motor_Stop_All();
  car_state = CAR_STOPPED;
  turn_direction = CORNER_NONE;
  candidate_corner = CORNER_NONE;
  corner_hits = 0U;
  center_hits = 0U;
  Status_LED(0U);
  Menu_Select_First();
  MenuDisplay_Show_Stopped(Menu_GetSelectedTask());
}

static void Car_Start(TaskMode mode)
{
  if (mode == TASK_REVERSE_PARKING) Reverse_Parking_Load_Record();

  task_mode = mode;
  turn_direction = CORNER_NONE;
  candidate_corner = CORNER_NONE;
  corner_hits = 0U;
  center_hits = 0U;
  completed_turns = 0U;
  Encoder_Reset_Distance();
  if (task_mode == TASK_SIDE_PARKING) {
    Parking_Set_State(CAR_PARK_STRAIGHT_AHEAD, HAL_GetTick());
  } else if (task_mode == TASK_REVERSE_PARKING) {
    /* Follow the track through B first; BC parking starts after that corner. */
    car_state = CAR_FOLLOWING;
  } else if (task_mode == TASK_DIAGNOSTIC) {
    car_state = CAR_DIAGNOSTIC;
    diagnostic_report_tick = HAL_GetTick();
    Diagnostic_Send("T4 READY. Type h for commands. Wheels start stopped.\\r\\n");
  } else {
    car_state = CAR_FOLLOWING;
  }
  menu_led_toggles_remaining = 0U;
  Status_LED(1U);
  if (task_mode == TASK_SIDE_PARKING) {
    MenuDisplay_Show_Running(MENU_TASK_2_SIDE_PARKING);
  } else if (task_mode == TASK_REVERSE_PARKING) {
    MenuDisplay_Show_Running(MENU_TASK_3_REVERSE_PARKING);
  } else if (task_mode == TASK_DIAGNOSTIC) {
    MenuDisplay_Show_Running(MENU_TASK_4_ENCODER_TEST);
  } else {
    MenuDisplay_Show_Running(MENU_TASK_1_SQUARE_LINE);
  }
}

static void Follow_Line(uint8_t sensor, uint32_t now)
{
  CornerDirection detected_corner;
  int16_t correction;

  if (sensor == 0U || Is_Ambiguous_Black_Area(sensor)) {
    Car_Stop();
    return;
  }

  detected_corner = Detect_Corner(sensor);
  if (detected_corner != CORNER_NONE) {
    if (detected_corner == candidate_corner) corner_hits++;
    else {
      candidate_corner = detected_corner;
      corner_hits = 1U;
    }

    if (corner_hits >= CORNER_CONFIRM_COUNT) {
      turn_direction = detected_corner;
      state_start_tick = now;
      car_state = CAR_ENTERING_CORNER;
      corner_hits = 0U;
    }
    return;
  }

  candidate_corner = CORNER_NONE;
  corner_hits = 0U;
  correction = (int16_t)(Sensor_Error(sensor) * FOLLOW_KP);
  if (correction > FOLLOW_MAX_CORRECTION) correction = FOLLOW_MAX_CORRECTION;
  if (correction < -FOLLOW_MAX_CORRECTION) correction = -FOLLOW_MAX_CORRECTION;

  Motor_Set_Left_Right((int16_t)(FOLLOW_SPEED + correction),
                       (int16_t)(FOLLOW_SPEED - correction));
}

static void Process_Corner(uint8_t sensor, uint32_t now)
{
  if (car_state == CAR_ENTERING_CORNER) {
    Motor_Set_Left_Right(CORNER_ENTER_SPEED, CORNER_ENTER_SPEED);
    if (now - state_start_tick >= CORNER_ENTER_TIME_MS) {
      state_start_tick = now;
      center_hits = 0U;
      car_state = CAR_TURNING;
    }
    return;
  }

  if (car_state == CAR_TURNING) {
    if (turn_direction == CORNER_LEFT) {
      Motor_Set_Left_Right(-TURN_PIVOT_SPEED, TURN_PIVOT_SPEED);
    } else {
      Motor_Set_Left_Right(TURN_PIVOT_SPEED, -TURN_PIVOT_SPEED);
    }

    if (now - state_start_tick > TURN_TIMEOUT_MS) {
      Car_Stop();
      return;
    }

    if (now - state_start_tick >= TURN_MIN_TIME_MS && Is_Line_Reacquired(sensor)) {
      center_hits++;
      if (center_hits >= CENTER_CONFIRM_COUNT) {
        state_start_tick = now;
        car_state = CAR_RECOVERING;
      }
    } else {
      center_hits = 0U;
    }
    return;
  }

  if (car_state == CAR_RECOVERING) {
    Motor_Set_Left_Right(CORNER_ENTER_SPEED, CORNER_ENTER_SPEED);
    if (now - state_start_tick >= TURN_RECOVER_TIME_MS) {
      completed_turns++;
      if (task_mode == TASK_REVERSE_PARKING &&
          completed_turns >= REVERSE_PARK_TURN_COUNT) {
        Parking_Set_State(CAR_REVERSE_PARK_APPROACH, now);
      } else if (LAP_TURN_LIMIT != 0U && completed_turns >= LAP_TURN_LIMIT) {
        Car_Stop();
      } else {
        car_state = CAR_FOLLOWING;
      }
    }
  }
}

static void Parking_Reverse_Turn_Into_Slot(void)
{
  if (PARK_SLOT_ON_RIGHT) {
    Motor_Set_Left_Right(-PARK_INNER_SPEED, -PARK_OUTER_SPEED);
  } else {
    Motor_Set_Left_Right(-PARK_OUTER_SPEED, -PARK_INNER_SPEED);
  }
}

static void Parking_Reverse_Straighten(void)
{
  if (PARK_SLOT_ON_RIGHT) {
    Motor_Set_Left_Right(-PARK_OUTER_SPEED, -PARK_INNER_SPEED);
  } else {
    Motor_Set_Left_Right(-PARK_INNER_SPEED, -PARK_OUTER_SPEED);
  }
}

static void Process_Side_Parking(uint8_t sensor, uint32_t now)
{
  (void)sensor;

  if (car_state == CAR_PARK_STRAIGHT_AHEAD) {
    Motor_Set_Left_Right(PARK_DRIVE_SPEED, PARK_DRIVE_SPEED);
    (void)Parking_Target_Reached(PARK_STRAIGHT_AHEAD_COUNTS,
                                 CAR_PARK_REVERSE_TURN_IN, now);
    return;
  }

  if (car_state == CAR_PARK_REVERSE_TURN_IN) {
    Parking_Reverse_Turn_Into_Slot();
    (void)Parking_Target_Reached(PARK_TURN_IN_COUNTS,
                                 CAR_PARK_REVERSE_STRAIGHTEN, now);
    return;
  }

  if (car_state == CAR_PARK_REVERSE_STRAIGHTEN) {
    Parking_Reverse_Straighten();
    (void)Parking_Target_Reached(PARK_STRAIGHTEN_COUNTS,
                                 CAR_PARK_FINAL_ADJUST, now);
    return;
  }

  if (car_state == CAR_PARK_FINAL_ADJUST) {
    Motor_Set_Left_Right(PARK_DRIVE_SPEED, PARK_DRIVE_SPEED);
    if (Parking_Target_Reached(PARK_FINAL_ADJUST_COUNTS, CAR_STOPPED, now)) {
      Car_Stop();
    }
  }
}

static void Process_Reverse_Parking(uint8_t sensor, uint32_t now)
{
  if (car_state == CAR_REVERSE_PARK_APPROACH) {
    int16_t correction;

    /* Keep using the gray sensor on BC, but do not accept another corner. */
    if (sensor == 0U || Is_Ambiguous_Black_Area(sensor)) {
      Car_Stop();
      return;
    }
    correction = (int16_t)(Sensor_Error(sensor) * FOLLOW_KP);
    if (correction > FOLLOW_MAX_CORRECTION) correction = FOLLOW_MAX_CORRECTION;
    if (correction < -FOLLOW_MAX_CORRECTION) correction = -FOLLOW_MAX_CORRECTION;
    Motor_Set_Left_Right((int16_t)(REVERSE_PARK_FOLLOW_SPEED + correction),
                         (int16_t)(REVERSE_PARK_FOLLOW_SPEED - correction));
    (void)Parking_Target_Reached(reverse_park_approach_counts,
                                 CAR_REVERSE_PARK_ALIGN, now);
    return;
  }

  if (car_state == CAR_REVERSE_PARK_ALIGN) {
    /* P3->P4: forward left arc, right side travels farther. */
    Motor_Set_Left_Right(REVERSE_PARK_ALIGN_LEFT_SPEED,
                         REVERSE_PARK_ALIGN_RIGHT_SPEED);
    (void)Parking_Target_Reached(reverse_park_align_counts,
                                 CAR_REVERSE_PARK_BACK_IN, now);
    return;
  }

  if (car_state == CAR_REVERSE_PARK_BACK_IN) {
    /* P4->P5: low-speed reverse right arc into the garage. */
    Motor_Set_Left_Right(-REVERSE_PARK_BACK_LEFT_SPEED,
                         -REVERSE_PARK_BACK_RIGHT_SPEED);
    if (Parking_Target_Reached(reverse_park_back_in_counts, CAR_STOPPED, now)) {
      Car_Stop();
    }
  }
}

static void Diagnostic_Send(const char *text)
{
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)text,
                          (uint16_t)strlen(text), 20U);
}

static void Diagnostic_Report(void)
{
  char text[96];
  uint32_t left_front = Encoder_Absolute_Count(&htim4);
  uint32_t right_front = Encoder_Absolute_Count(&htim3);
  uint32_t left_rear = Encoder_Absolute_Count(&htim8);
  uint32_t right_rear = Encoder_Absolute_Count(&htim2);

  (void)snprintf(text, sizeof(text), "ENC LF=%lu RF=%lu LR=%lu RR=%lu AVG=%lu\\r\\n",
                 (unsigned long)left_front, (unsigned long)right_front,
                 (unsigned long)left_rear, (unsigned long)right_rear,
                 (unsigned long)Encoder_Travel_Count());
  Diagnostic_Send(text);
}

static void Diagnostic_Handle_Command(uint8_t command)
{
  if (command == '1') Motor_Set_Test_One(1U, DIAGNOSTIC_SPEED);
  else if (command == '2') Motor_Set_Test_One(2U, DIAGNOSTIC_SPEED);
  else if (command == '3') Motor_Set_Test_One(3U, DIAGNOSTIC_SPEED);
  else if (command == '4') Motor_Set_Test_One(4U, DIAGNOSTIC_SPEED);
  else if (command == 'A') Motor_Set_Test_One(1U, -DIAGNOSTIC_SPEED);
  else if (command == 'B') Motor_Set_Test_One(2U, -DIAGNOSTIC_SPEED);
  else if (command == 'C') Motor_Set_Test_One(3U, -DIAGNOSTIC_SPEED);
  else if (command == 'D') Motor_Set_Test_One(4U, -DIAGNOSTIC_SPEED);
  else if (command == 'f') Motor_Set_Left_Right(DIAGNOSTIC_SPEED, DIAGNOSTIC_SPEED);
  else if (command == 'b') Motor_Set_Left_Right(-DIAGNOSTIC_SPEED, -DIAGNOSTIC_SPEED);
  else if (command == 's') Motor_Stop_All();
  else if (command == 'r') Encoder_Reset_Distance();
  else if (command == 'p') Diagnostic_Report();
  else if (command == 'h') {
    Diagnostic_Send("CMD 1-4=forward A-D=reverse f=all forward b=all reverse s=stop r=reset p=print q=exit\\r\\n");
  } else if (command == 'q') {
    Car_Stop();
  }
}

static void Process_Diagnostic(uint32_t now)
{
  uint8_t command;

  if (HAL_UART_Receive(&huart1, &command, 1U, 0U) == HAL_OK) {
    Diagnostic_Handle_Command(command);
  }

  if (now - diagnostic_report_tick >= DIAGNOSTIC_REPORT_MS) {
    diagnostic_report_tick = now;
    Diagnostic_Report();
  }
}

static void Car_Process(uint32_t now)
{
  if (task_mode == TASK_DIAGNOSTIC) {
    Process_Diagnostic(now);
    return;
  }

  uint8_t sensor = Sensor_Read();

  if (car_state == CAR_FOLLOWING) Follow_Line(sensor, now);
  else if (car_state == CAR_ENTERING_CORNER ||
           car_state == CAR_TURNING ||
           car_state == CAR_RECOVERING) Process_Corner(sensor, now);
  else if (task_mode == TASK_SIDE_PARKING) Process_Side_Parking(sensor, now);
  else if (task_mode == TASK_REVERSE_PARKING) Process_Reverse_Parking(sensor, now);
}

static KeyEvent Key_Read_Event(void)
{
  static uint8_t pressed = 0U;
  static uint8_t long_reported = 0U;
  static uint32_t pressed_tick = 0U;
  uint8_t key_is_pressed = Key_Is_Pressed();

  if (key_is_pressed && !pressed) {
    HAL_Delay(20U);
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) {
      pressed = 1U;
      long_reported = 0U;
      pressed_tick = HAL_GetTick();
    }
    return KEY_EVENT_NONE;
  }

  if (key_is_pressed && pressed && !long_reported &&
      HAL_GetTick() - pressed_tick >= KEY_LONG_PRESS_MS) {
    long_reported = 1U;
    return KEY_EVENT_LONG;
  }

  if (!key_is_pressed && pressed) {
    uint32_t pressed_ms = HAL_GetTick() - pressed_tick;
    pressed = 0U;
    if (!long_reported && pressed_ms < KEY_SHORT_PRESS_MAX_MS) {
      return KEY_EVENT_SHORT;
    }
  }
  return KEY_EVENT_NONE;
}

int main(void)
{
  uint32_t control_tick = 0U;

  HAL_Init();
  SystemClock_Config();
  Microsecond_Delay_Init();
  GPIO_Init_All();
  PWM_Init_All();
  Encoder_Init_All();
  UART1_Init();
  Car_Stop();
  Menu_Init();
  MenuDisplay_Init();
#if ROUTE_RECORDER_ON_KEY
  RouteRecorder_Init();
  Status_LED(RouteRecorder_LED_Is_On());
#else
#if DIAGNOSTIC_AUTO_START
  /* Dedicated bench-test firmware: T4 starts automatically but motors stay stopped. */
  Car_Start(TASK_DIAGNOSTIC);
#else
  Menu_Indicate_Task(Menu_GetSelectedTask(), HAL_GetTick());
  MenuDisplay_Show_Browse(Menu_GetSelectedTask());
#endif
#endif

  while (1) {
    uint32_t now = HAL_GetTick();

    /* A running car stops as soon as PA15 is pressed. */
    if (car_state != CAR_STOPPED && Key_Is_Pressed()) {
      Car_Stop();
      ignore_stop_key_event = 1U;
      Menu_Indicate_Task(Menu_GetSelectedTask(), now);
    }

    KeyEvent key_event = Key_Read_Event();
#if ROUTE_RECORDER_ON_KEY
    if (key_event == KEY_EVENT_SHORT) {
      if (RouteRecorder_Is_Armed()) {
        Encoder_Reset_Distance();
      }
      RouteRecorder_Handle_Short_Press(now,
                                       Encoder_Signed_Count(&htim4),
                                       Encoder_Signed_Count(&htim8),
                                       Encoder_Signed_Count(&htim2));
    }
    RouteRecorder_Process(now);
    Status_LED(RouteRecorder_LED_Is_On());
#else
    if (key_event != KEY_EVENT_NONE) {
      if (ignore_stop_key_event) {
        /* The release after an emergency stop must not change selection. */
        ignore_stop_key_event = 0U;
#if SIDE_PARKING_DEMO_ON_KEY
      } else if (key_event == KEY_EVENT_SHORT || key_event == KEY_EVENT_LONG) {
        Car_Start(TASK_SIDE_PARKING);
#endif
      }
#if REVERSE_PARK_DEMO_ON_KEY
      else if (key_event == KEY_EVENT_SHORT) {
        Car_Start(TASK_REVERSE_PARKING);
      }
#else
      else {
        if (key_event == KEY_EVENT_SHORT) {
          Menu_Select_Next();
          Menu_Indicate_Task(Menu_GetSelectedTask(), now);
          MenuDisplay_Show_Browse(Menu_GetSelectedTask());
        } else if (Menu_GetSelectedTask() == MENU_TASK_1_SQUARE_LINE) {
          Car_Start(TASK_NORMAL_LINE);
        } else if (Menu_GetSelectedTask() == MENU_TASK_2_SIDE_PARKING) {
          Car_Start(TASK_SIDE_PARKING);
        } else if (Menu_GetSelectedTask() == MENU_TASK_3_REVERSE_PARKING) {
          Car_Start(TASK_REVERSE_PARKING);
        } else if (Menu_GetSelectedTask() == MENU_TASK_4_ENCODER_TEST) {
          Car_Start(TASK_DIAGNOSTIC);
        } else {
          /* Reserved tasks do not move the car until their code exists. */
          Menu_Indicate_Task(Menu_GetSelectedTask(), now);
        }
      }
#endif
    }
#endif

#if ROUTE_RECORDER_ON_KEY
    Motor_Stop_All();
#else
    if (car_state != CAR_STOPPED && now - control_tick >= CONTROL_PERIOD_MS) {
      control_tick = now;
      Car_Process(now);
    } else if (car_state == CAR_STOPPED) {
      Menu_Process_Indicator(now);
    }
#endif
    HAL_Delay(1U);
  }
}

void Error_Handler(void)
{
  Error_Stop();
}
