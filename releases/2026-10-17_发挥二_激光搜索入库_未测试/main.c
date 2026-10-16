#include "main.h"
#include <stdio.h>
#include <string.h>
#include "entry_plan.h"
#include "laser_test.h"

#define MOTOR_STBY_PORT GPIOB
#define MOTOR_STBY_PIN GPIO_PIN_15
#define PWM_PERIOD 8399U
#define SENSOR_ADDRESS_PORT GPIOB
#define SENSOR_OUT_PORT GPIOC
#define SENSOR_AD0_PIN GPIO_PIN_12
#define SENSOR_AD1_PIN GPIO_PIN_13
#define SENSOR_AD2_PIN GPIO_PIN_14
#define SENSOR_OUT_PIN GPIO_PIN_1
#define SENSOR_BLACK_LEVEL GPIO_PIN_SET
#define SENSOR_SETTLE_US 50U

typedef enum { STOPPED,FOLLOW,ENTER_CORNER,PIVOT,RECOVER } State;
typedef enum { NO_TURN,LEFT_TURN,RIGHT_TURN } Direction;
typedef enum { IDLE,SEARCH,LINE_PAUSE,PROBE_FORWARD,MEASURE,PROBE_BACK,
               ENTRY,DONE,FAILED } Phase;
static TIM_HandleTypeDef pwm_timer,enc_a,enc_b,enc_c,enc_d;
static UART_HandleTypeDef dap_uart,imu_uart;
static I2C_HandleTypeDef laser_i2c;
static volatile uint8_t gray_value;
static volatile uint32_t gray_frames,gray_last_ms,gray_errors;
static volatile uint16_t imu_yaw_x100;
static volatile int16_t imu_pitch_x100,imu_roll_x100,imu_gyro_z_raw;
static volatile uint32_t imu_frames,imu_last_ms;
static volatile uint8_t imu_state,imu_index,imu_length,imu_sum,imu_frame[27];
static State state=STOPPED;
static Phase auto_phase=IDLE;
static uint8_t turns,center_hits,feature_hits,reference_run_used,line_lost;
static TrackFeature feature_candidate;
static Direction turn_direction;
static int16_t left_command,right_command,previous_counts[4],odom_previous[4];
static int32_t odom_total[4];
static uint32_t run_start_ms,state_start_ms,last_turn_complete_ms,line_lost_ms;
static uint32_t phase_ms,watch_ms;
static int32_t watch_position;
static uint8_t pivot_active;
static uint16_t pivot_start_yaw;
static uint32_t pivot_last_stop_ms,pivot_start_ms;
static int16_t pivot_encoder_start[4];
static GarageType garage_type;
static int16_t garage_origin[3];
static uint16_t garage_yaw,range_samples[RANGE_SAMPLE_COUNT],range_median;
static uint8_t range_used,range_discard;
static uint32_t range_seen,last_range_poll;
static void combined_abort(const char *why);
static void start_run(void);

#include "hardware.inc"
#include "entry_motion.inc"
#include "search_workflow.inc"
#include "navigation.inc"
#include "commands.inc"

static void laser_init(void)
{
  GPIO_InitTypeDef p={0};
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
  p.Pin=GPIO_PIN_8;p.Mode=GPIO_MODE_OUTPUT_PP;p.Pull=GPIO_NOPULL;
  p.Speed=GPIO_SPEED_FREQ_LOW;HAL_GPIO_Init(GPIOC,&p);
  p.Mode=GPIO_MODE_AF_OD;p.Pull=GPIO_PULLUP;p.Speed=GPIO_SPEED_FREQ_HIGH;
  p.Alternate=GPIO_AF4_I2C3;p.Pin=GPIO_PIN_8;HAL_GPIO_Init(GPIOA,&p);
  p.Pin=GPIO_PIN_9;HAL_GPIO_Init(GPIOC,&p);
  __HAL_RCC_I2C3_CLK_ENABLE();
  laser_i2c.Instance=I2C3;laser_i2c.Init.ClockSpeed=100000U;
  laser_i2c.Init.DutyCycle=I2C_DUTYCYCLE_2;
  laser_i2c.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
  laser_i2c.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
  laser_i2c.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE;
  laser_i2c.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
  if(HAL_I2C_Init(&laser_i2c)!=HAL_OK)Error_Handler();
  /* Blocking vendor initialization is only called with STBY low. */
  LaserTest_Init(&laser_i2c);
}

#include "start.inc"

void USART3_IRQHandler(void) { } /* Not enabled in this firmware. */

int main(void)
{
  static const uint8_t imu_full[7]={0xAA,0x55,0x60,0x0B,0x01,0x00,0x6C};
  static const uint8_t imu_start[7]={0xAA,0x55,0x60,0x0A,0x01,0x01,0x6C};
  uint32_t last_sample=0,last_control=0,last_report=0,last_imu=0;
  HAL_Init();clock_init();gpio_init();
  CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT=0;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
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
  pwm_init();stop_motors();laser_init();
  send_text("READY," ENTRY_NAME ",BOOT_STOPPED,KEY=start_stop,START=" SEARCH_START_FRAME "\\n,X=stop,once_per_reset\r\n");
  char boot[140];
  snprintf(boot,sizeof(boot),"LASER_INIT,status=%u,id=%04X,offset_mm=%d,estimated=1\r\n",
           laser_status,laser_model_id,LASER_RIGHT_OFFSET_MM);send_text(boot);
  (void)HAL_UART_Transmit(&imu_uart,(uint8_t*)imu_full,7,20);
  (void)HAL_UART_Transmit(&imu_uart,(uint8_t*)imu_start,7,20);last_imu=HAL_GetTick();
  search_read_key(last_imu);
  while(1){
    uint32_t now=HAL_GetTick();
    search_read_key(now);search_read_dap();
    if(now-last_sample>=5U){sensor_sample();odom_update();last_sample=now;}
    if(!imu_frames && now-last_imu>=2000U){
      (void)HAL_UART_Transmit(&imu_uart,(uint8_t*)imu_full,7,20);
      (void)HAL_UART_Transmit(&imu_uart,(uint8_t*)imu_start,7,20);last_imu=now;
    }
    if(now-last_control>=10U){
      last_control=now;
      /* Check stop conditions BEFORE allowing any motor command. */
      search_safety(now);
      if(auto_phase==SEARCH)search_navigation(now);
      if(auto_phase>IDLE && auto_phase<DONE){reference_control(now);search_tick(now);}
    }
    if(now-last_report>=100U){
      last_report=now;report(now);
      if(reference_next)reference_report(now);
    }
    if(auto_phase==IDLE || auto_phase>=DONE)stop_motors();
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,
      ((now/(auto_phase==IDLE || auto_phase>=DONE?500U:100U))&1U)?GPIO_PIN_SET:GPIO_PIN_RESET);
  }
}
