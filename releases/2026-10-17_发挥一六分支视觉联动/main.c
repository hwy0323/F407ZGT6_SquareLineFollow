#ifdef TASK4_HOST_TEST
#include "tests/fake_app_hal.h"
#else
#include "main.h"
#endif
#include "parking_controller.h"
#include "vision_uart6.h"
#include "gimbal_pwm.h"
#include "start_command.h"
#include <stdio.h>
#include <string.h>

typedef enum { IDLE, POSITIONING, IDENTIFYING, PREFLIGHT, DRIVING, FINISHED } Mission;
static MvClient vision;
static Mission mission;
static uint8_t route_used;
static volatile uint8_t remote_start, remote_stop;
static volatile uint8_t dap_rx[128], dap_head, dap_tail, dap_fault;
static StartCommand start_command;
static uint32_t pose_started, last_link_report;
volatile uint32_t task4_status[12];
volatile MvEvent vision_last_result;
static uint32_t preflight_start, preflight_stable;
static uint8_t preflight_tracking;

void SysTick_Handler(void) { HAL_IncTick(); }
void Error_Handler(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef pin = {0};
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  pin.Pin = GPIO_PIN_15;
  pin.Mode = GPIO_MODE_OUTPUT_PP;
  pin.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &pin);
  while (1) { }
}

static void log_text(const char *text)
{
  /* USART1 is the existing DAP logger. It never carries MV/1 commands. */
  for (; *text; ++text) {
    uint32_t start = HAL_GetTick();
    while (!(USART1->SR & USART_SR_TXE))
      if (HAL_GetTick() - start >= 5U) return;
    USART1->DR = (uint8_t)*text;
  }
}

void vision_trace(const char *direction,const char *data,uint16_t length)
{
  char clean[MV_MAX_FRAME+1U],line[240];uint16_t used=0U;
  for(uint16_t i=0U;i<length && used<MV_MAX_FRAME;i++){
    uint8_t b=(uint8_t)data[i];
    if(b!='\r' && b!='\n')clean[used++]=(b>=32U && b<=126U)?(char)b:'.';
  }
  clean[used]=0;
  /* Keep runtime heartbeat tracing from adding motor-control jitter. */
  if(mission==DRIVING)return;
  (void)snprintf(line,sizeof(line),"V6_%s,%lu,%s\r\n",direction,(unsigned long)HAL_GetTick(),clean);
  log_text(line);
}

void USART1_IRQHandler(void)
{
  uint32_t status = USART1->SR;
  if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
    (void)USART1->DR; dap_fault=1U; return;
  }
  if (status & USART_SR_RXNE) {
    uint8_t value = (uint8_t)USART1->DR;
    if(value=='X' || value=='x')remote_stop=1U;
    else {
      uint8_t next=(uint8_t)((dap_head+1U)&127U);
      if(next==dap_tail)dap_fault=1U;
      else {dap_rx[dap_head]=value;dap_head=next;}
    }
  }
}

static void read_dap(void)
{
  if(dap_fault){
    dap_fault=0U;dap_tail=dap_head;memset(&start_command,0,sizeof(start_command));
    if(mission!=IDLE && mission!=FINISHED)remote_stop=1U;
    return;
  }
  while(dap_tail!=dap_head){
    uint8_t byte=dap_rx[dap_tail];dap_tail=(uint8_t)((dap_tail+1U)&127U);
    if(start_command_feed(&start_command,byte))remote_start=1U;
  }
}

void USART3_IRQHandler(void) { }
void UART4_IRQHandler(void)
{
  parking_imu_irq();
}

static void route_tick(void)
{
  parking_tick();
}

static uint8_t route_ready(void)
{
  return parking_preflight();
}

static uint8_t route_busy(void)
{
  return parking_busy();
}

static uint8_t route_start(void)
{
  return parking_start();
}

static void route_stop(void)
{
  /* Drop the hardware enable FIRST, independent of route's software state. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  parking_stop();
}

static void request(uint32_t now)
{
  if (route_used) {
    log_text("TASK4,REFUSE,RESET_BEFORE_NEXT_RUN\r\n");
    return;
  }
  if (mission != IDLE) return;
  route_stop();
  if(vision.phase==MV_OFFLINE || vision_uart6_fault()){
    vision_uart6_reset();mv_init(&vision,vision_uart6_send,now);
  }
  if(!gimbal_begin_task(4U,now)){
    log_text("TASK4,REFUSE,GIMBAL_FAILED\r\n");return;
  }
  pose_started=now;mission=POSITIONING;
  log_text("TASK4,SERVO,upper=150,lower=75,wait_ms=1000,MOTORS_OFF\r\n");
}

static void cancel(uint32_t now, const char *reason)
{
  route_stop();
  remote_start=0U;memset(&start_command,0,sizeof(start_command));
  mv_cancel(&vision, now);
  mission = route_used ? FINISHED : IDLE;
  log_text(reason);
}

static void read_key(uint32_t now)
{
  static uint8_t previous = 1U, stable = 1U, armed, consumed;
  static uint32_t changed;
  uint8_t raw = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET;
  if (raw != previous) { previous = raw; changed = now; }
  if (now - changed < 25U) return;
  if (raw && !armed) { armed = 1U; stable = 1U; return; }
  if (!armed || raw == stable) return;
  stable = raw;
  if (!stable && mission != IDLE && mission != FINISHED) {
    consumed = 1U;
    cancel(now, "TASK4,STOP,KEY\r\n");
  } else if (stable) {
    if (consumed) consumed = 0U;
    else request(now);
  }
}

static void app_tick(uint32_t now)
{
    read_key(now);read_dap();
    if (remote_stop) {
      remote_stop = remote_start = 0U;
      cancel(now, "TASK4,STOP,REMOTE_X_OR_UART_ERROR\r\n");
    } else if (remote_start) { remote_start = 0U; request(now); }
    mv_poll(&vision, now);
    vision_uart6_poll(&vision, now);
    if(mission==POSITIONING){
      if(gimbal_is_settled(now)){
        if(mv_request_shape(&vision,now)){
          mission=IDENTIFYING;
          log_text("TASK4,REQUEST,SHAPE,WAIT_RESULT_MOTORS_OFF\r\n");
        }else cancel(now,"TASK4,STOP,VISION_REQUEST_REFUSED\r\n");
      }else if(now-pose_started>GIMBAL_PREPARE_TIMEOUT_MS)
        cancel(now,"TASK4,STOP,GIMBAL_SETTLE_TIMEOUT\r\n");
    }
    MvEvent event;
    if (mv_take_event(&vision, &event)) {
      if (event.kind == MV_SHAPE && event.task==4U && mission == IDENTIFYING) {
        if(!parking_configure(event.color,event.shape,event.action))
          cancel(now,"TASK4,STOP,INVALID_PLAN\r\n");
        else {
          char line[220];vision_last_result=event;
          const Task4Plan *plan=parking_current_plan();
          (void)snprintf(line,sizeof(line),
            "TASK4,VISION_RESULT,%08lX,%lu,%s,%s,%s,edge=%s,corners=%u,branch=%u,line=%ld,score=%u\r\n",
            (unsigned long)event.sid,(unsigned long)event.seq,event.color,event.shape,event.action,
            plan->edge,plan->corners,plan->branch_id,(long)plan->first_line_center,event.score);
          log_text(line);now=HAL_GetTick();
          preflight_start=now;preflight_tracking=0U;mission=PREFLIGHT;
        }
      } else if (event.kind == MV_FAILED || event.kind == MV_LOST) {
        char line[90];
        (void)snprintf(line, sizeof(line), "TASK4,VISION_STOP,%s\r\n", event.reason);
        cancel(now, line);
      }
    }
    if ((mission == PREFLIGHT || mission == DRIVING) &&
        (!mv_link_ok(&vision, now) || vision_uart6_fault()))
      cancel(now, "TASK4,STOP,VISION_LINK_UNSAFE\r\n");
    route_tick();
    now = HAL_GetTick();
    if (mission == PREFLIGHT) {
      if (route_ready()) {
        if (!preflight_tracking) { preflight_stable = now; preflight_tracking = 1U; }
        if (now - preflight_stable >= 200U) {
          route_used = 1U;
          if (route_start()) {mission = DRIVING;log_text("TASK4,ROUTE_START\r\n");}
          else cancel(now, "TASK4,STOP,ROUTE_START_REFUSED\r\n");
        }
      } else preflight_tracking = 0U;
      if (mission == PREFLIGHT && now - preflight_start >= 3000U)
        cancel(now, "TASK4,STOP,NEED_A_CENTER_FRESH_IMU_KEY_RELEASED\r\n");
    } else if (mission == DRIVING && !route_busy()) {
      route_stop();mission = FINISHED;
      log_text(parking_done()?"TASK4,ENTRY_FINISHED,VERIFY_PHYSICAL_PARKING\r\n":
                             "TASK4,PROTECTION_STOP,SEE_NATIVE_STOP_REASON\r\n");
    }
    const Task4Plan *plan=parking_current_plan();
    task4_status[0]=0x54345632U;task4_status[1]=mission;task4_status[2]=plan->branch_id;
    task4_status[3]=vision.phase;task4_status[4]=vision.sid;task4_status[5]=vision.seq;
    task4_status[6]=mv_link_ok(&vision,now);task4_status[7]=vision_rx_bytes;
    task4_status[8]=vision_tx_bytes;task4_status[9]=vision.bad_frames;
    task4_status[10]=route_used;task4_status[11]=vision_rx_errors;
    if(now-last_link_report>=1000U){
      char line[190];last_link_report=now;
      (void)snprintf(line,sizeof(line),"TASK4,LINK,%lu,mission=%u,edge=%s,phase=%u,ok=%u,rx=%lu,tx=%lu,bad=%lu\r\n",
        (unsigned long)now,(unsigned)mission,plan->edge[0]?plan->edge:"--",(unsigned)vision.phase,
        (unsigned)mv_link_ok(&vision,now),(unsigned long)vision_rx_bytes,
        (unsigned long)vision_tx_bytes,(unsigned long)vision.bad_frames);
      log_text(line);
    }
    if (mission != DRIVING) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (now / 500U) & 1U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

int main(void)
{
  HAL_Init();
  parking_clock_init();
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  parking_init(); /* Wheel motors stay off while waiting at A. */
  if(!gimbal_pwm_init())Error_Handler();
  vision_uart6_init();
  mv_init(&vision, vision_uart6_send, HAL_GetTick());
  log_text("READY,TASK4_SIX_BRANCH_VISION_PB15,BOOT_STOPPED,START=" TASK4_START_FRAME "\\n,X=stop\r\n");
  while(1)app_tick(HAL_GetTick());
}
