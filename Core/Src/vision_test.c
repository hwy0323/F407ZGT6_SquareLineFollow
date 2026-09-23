#include "main.h"
#include "oled.h"
#include "vision_test.h"
#include <stdio.h>
#include <string.h>

#define VISION_FRAME_MAX_LENGTH 192U
#define VISION_RX_RING_SIZE 256U
#define VISION_HELLO_INTERVAL_MS 1000U
#define VISION_SET_RETRY_MS 1000U
#define VISION_SET_RETRY_LIMIT 3U
#define VISION_RESULT_TIMEOUT_MS 33000U
#define VISION_RESTART_DELAY_MS 500U

typedef enum {
  VISION_LINK_WAIT,
  VISION_CANCEL_WAIT,
  VISION_RESULT_WAIT,
  VISION_RESTART_WAIT,
  VISION_FATAL
} VisionTestState;

static UART_HandleTypeDef vision_uart;
static volatile uint8_t vision_rx_ring[VISION_RX_RING_SIZE];
static volatile uint16_t vision_rx_head = 0U;
static volatile uint16_t vision_rx_tail = 0U;
static volatile uint8_t vision_rx_overflow = 0U;

static char vision_frame[VISION_FRAME_MAX_LENGTH + 1U];
static uint16_t vision_frame_length = 0U;
static uint8_t vision_frame_collecting = 0U;

static VisionTestState vision_state = VISION_LINK_WAIT;
static char vision_sid[9] = {0};
static uint32_t vision_sequence = 0U;
static uint32_t vision_last_terminal_sequence = 0U;
static uint32_t vision_last_hello_tick = 0U;
static uint32_t vision_last_set_tick = 0U;
static uint32_t vision_deadline_tick = 0U;
static uint32_t vision_restart_tick = 0U;
static uint8_t vision_set_retries = 0U;
static uint8_t vision_set_ack_seen = 0U;
static uint8_t vision_have_result = 0U;

static void Vision_Display_Text(const char *text, uint8_t scale)
{
  uint16_t width;
  uint8_t x;
  uint8_t y;

  if (!OLED_Is_Ready()) return;
  width = (uint16_t)strlen(text) * 6U * scale;
  x = (width < 128U) ? (uint8_t)((128U - width) / 2U) : 0U;
  y = (uint8_t)((64U - 7U * scale) / 2U);
  OLED_Clear();
  OLED_Draw_Big_String(x, y, scale, text);
  OLED_Refresh();
}

static void Vision_Display_Number(char number)
{
  char text[2] = {number, '\0'};
  Vision_Display_Text(text, 8U);
}

static uint8_t Vision_Hex_Value(char character, uint8_t *value)
{
  if (character >= '0' && character <= '9') {
    *value = (uint8_t)(character - '0');
    return 1U;
  }
  if (character >= 'A' && character <= 'F') {
    *value = (uint8_t)(character - 'A' + 10);
    return 1U;
  }
  return 0U;
}

static uint8_t Vision_Parse_U32(const char *text, uint32_t *value)
{
  uint32_t result = 0U;

  if (*text == '\0') return 0U;
  while (*text != '\0') {
    uint32_t digit;
    if (*text < '0' || *text > '9') return 0U;
    digit = (uint32_t)(*text - '0');
    if (result > (0xFFFFFFFFU - digit) / 10U) return 0U;
    result = result * 10U + digit;
    text++;
  }
  *value = result;
  return 1U;
}

static uint8_t Vision_SID_Is_Valid(const char *sid)
{
  uint8_t index;
  uint8_t nonzero = 0U;

  if (strlen(sid) != 8U) return 0U;
  for (index = 0U; index < 8U; index++) {
    if (!((sid[index] >= '0' && sid[index] <= '9') ||
          (sid[index] >= 'A' && sid[index] <= 'F'))) return 0U;
    if (sid[index] != '0') nonzero = 1U;
  }
  return nonzero;
}

static uint8_t Vision_Send_Body(const char *body)
{
  char frame[VISION_FRAME_MAX_LENGTH + 1U];
  uint8_t checksum = 0U;
  uint16_t index;
  int length;

  for (index = 0U; body[index] != '\0'; index++) {
    checksum ^= (uint8_t)body[index];
  }
  length = snprintf(frame, sizeof(frame), "$%s*%02X\r\n", body, checksum);
  if (length <= 0 || length > (int)VISION_FRAME_MAX_LENGTH) return 0U;
  return HAL_UART_Transmit(&vision_uart, (uint8_t *)frame,
                           (uint16_t)length, 100U) == HAL_OK ? 1U : 0U;
}

static void Vision_Send_Hello(void)
{
  (void)Vision_Send_Body("MV,1,HELLO,00000000,0");
}

static void Vision_Send_Ack(uint32_t sequence)
{
  char body[64];
  (void)snprintf(body, sizeof(body), "MV,1,ACK,%s,%lu",
                 vision_sid, (unsigned long)sequence);
  (void)Vision_Send_Body(body);
}

static void Vision_Send_Set(uint32_t task, uint32_t timeout_ms)
{
  char body[80];
  (void)snprintf(body, sizeof(body), "MV,1,SET,%s,%lu,%lu,%lu",
                 vision_sid, (unsigned long)vision_sequence,
                 (unsigned long)task, (unsigned long)timeout_ms);
  (void)Vision_Send_Body(body);
}

static uint8_t Vision_Next_Sequence(void)
{
  if (vision_sequence == 0xFFFFFFFFU) {
    vision_state = VISION_FATAL;
    Vision_Display_Text("ERR", 5U);
    return 0U;
  }
  vision_sequence++;
  return 1U;
}

static void Vision_Start_Roman(uint32_t now_ms)
{
  if (!Vision_Next_Sequence()) return;
  Vision_Send_Set(3U, 30000U);
  vision_state = VISION_RESULT_WAIT;
  vision_set_ack_seen = 0U;
  vision_set_retries = 0U;
  vision_last_set_tick = now_ms;
  vision_deadline_tick = now_ms + VISION_RESULT_TIMEOUT_MS;
  if (!vision_have_result) Vision_Display_Text("WAIT", 4U);
}

static void Vision_Start_Cancel(uint32_t now_ms)
{
  if (!Vision_Next_Sequence()) return;
  Vision_Send_Set(0U, 0U);
  vision_state = VISION_CANCEL_WAIT;
  vision_set_retries = 0U;
  vision_last_set_tick = now_ms;
}

static uint8_t Vision_Time_Reached(uint32_t now_ms, uint32_t target_ms)
{
  return ((int32_t)(now_ms - target_ms) >= 0) ? 1U : 0U;
}

static uint8_t Vision_Result_Number(char **field, uint8_t count,
                                    char *number)
{
  uint32_t score;

  if (count != 11U || strcmp(field[5], "3") != 0 ||
      strcmp(field[6], "ROMAN") != 0 ||
      !Vision_Parse_U32(field[10], &score) || score > 100U) return 0U;

  if (strcmp(field[7], "I") == 0 && strcmp(field[8], "1") == 0 &&
      strcmp(field[9], "A") == 0) *number = '1';
  else if (strcmp(field[7], "III") == 0 && strcmp(field[8], "3") == 0 &&
           strcmp(field[9], "B") == 0) *number = '3';
  else if (strcmp(field[7], "V") == 0 && strcmp(field[8], "5") == 0 &&
           strcmp(field[9], "C") == 0) *number = '5';
  else if (strcmp(field[7], "VII") == 0 && strcmp(field[8], "7") == 0 &&
           strcmp(field[9], "D") == 0) *number = '7';
  else return 0U;
  return 1U;
}

static void Vision_Handle_Ready(char **field, uint8_t count, uint32_t now_ms)
{
  uint32_t last_sequence;

  if (count != 9U || strcmp(field[4], "0") != 0 ||
      !Vision_SID_Is_Valid(field[3]) ||
      !Vision_Parse_U32(field[6], &last_sequence)) return;

  if (vision_state != VISION_LINK_WAIT &&
      strcmp(vision_sid, field[3]) == 0) return;

  memcpy(vision_sid, field[3], 9U);
  vision_sequence = last_sequence;
  vision_last_terminal_sequence = 0U;
  vision_have_result = 0U;
  Vision_Start_Cancel(now_ms);
}

static void Vision_Handle_Ack(char **field, uint8_t count, uint32_t now_ms)
{
  uint32_t sequence;
  uint32_t task;

  if (count != 7U || strcmp(field[3], vision_sid) != 0 ||
      !Vision_Parse_U32(field[4], &sequence) || sequence != vision_sequence ||
      !Vision_Parse_U32(field[5], &task)) return;

  if (vision_state == VISION_CANCEL_WAIT && task == 0U) {
    Vision_Start_Roman(now_ms);
  } else if (vision_state == VISION_RESULT_WAIT && task == 3U) {
    vision_set_ack_seen = 1U;
  }
}

static void Vision_Handle_Result(char **field, uint8_t count, uint32_t now_ms)
{
  uint32_t sequence;
  char number;

  if (count < 5U || strcmp(field[3], vision_sid) != 0 ||
      !Vision_Parse_U32(field[4], &sequence)) return;

  if (sequence == vision_last_terminal_sequence) {
    Vision_Send_Ack(sequence);
    return;
  }
  if (vision_state != VISION_RESULT_WAIT || sequence != vision_sequence ||
      !Vision_Result_Number(field, count, &number)) return;

  Vision_Send_Ack(sequence);
  vision_last_terminal_sequence = sequence;
  vision_have_result = 1U;
  Vision_Display_Number(number);
  vision_restart_tick = now_ms + VISION_RESTART_DELAY_MS;
  vision_state = VISION_RESTART_WAIT;
}

static void Vision_Handle_Fail(char **field, uint8_t count, uint32_t now_ms)
{
  uint32_t sequence;
  uint32_t task;

  if (count != 7U || strcmp(field[3], vision_sid) != 0 ||
      !Vision_Parse_U32(field[4], &sequence) ||
      !Vision_Parse_U32(field[5], &task) || task != 3U) return;

  if (sequence == vision_last_terminal_sequence) {
    Vision_Send_Ack(sequence);
    return;
  }
  if (vision_state != VISION_RESULT_WAIT || sequence != vision_sequence) return;

  Vision_Send_Ack(sequence);
  vision_last_terminal_sequence = sequence;
  Vision_Display_Text("FAIL", 4U);
  vision_restart_tick = now_ms + VISION_RESTART_DELAY_MS;
  vision_state = VISION_RESTART_WAIT;
}

static void Vision_Handle_Frame(char *frame, uint16_t length, uint32_t now_ms)
{
  char *field[12];
  char *star;
  char *cursor;
  uint8_t field_count = 1U;
  uint8_t checksum = 0U;
  uint8_t high;
  uint8_t low;

  if (length < 8U || frame[0] != '$' || frame[length - 2U] != '\r' ||
      frame[length - 1U] != '\n') return;
  star = strrchr(frame, '*');
  if (star == NULL || (uint16_t)(star - frame) + 5U != length ||
      !Vision_Hex_Value(star[1], &high) || !Vision_Hex_Value(star[2], &low)) return;

  for (cursor = frame + 1; cursor < star; cursor++) checksum ^= (uint8_t)*cursor;
  if (checksum != (uint8_t)((high << 4U) | low)) return;

  *star = '\0';
  field[0] = frame + 1;
  for (cursor = frame + 1; *cursor != '\0'; cursor++) {
    if (*cursor == ',') {
      if (field_count >= 12U) return;
      *cursor = '\0';
      field[field_count++] = cursor + 1;
    }
  }
  if (field_count < 5U || strcmp(field[0], "MV") != 0 ||
      strcmp(field[1], "1") != 0) return;

  if (strcmp(field[2], "READY") == 0) {
    Vision_Handle_Ready(field, field_count, now_ms);
  } else if (strcmp(field[2], "ACK") == 0) {
    Vision_Handle_Ack(field, field_count, now_ms);
  } else if (strcmp(field[2], "RESULT") == 0) {
    Vision_Handle_Result(field, field_count, now_ms);
  } else if (strcmp(field[2], "FAIL") == 0) {
    Vision_Handle_Fail(field, field_count, now_ms);
  }
}

static uint8_t Vision_Rx_Pop(uint8_t *byte)
{
  if (vision_rx_tail == vision_rx_head) return 0U;
  *byte = vision_rx_ring[vision_rx_tail];
  vision_rx_tail = (uint16_t)((vision_rx_tail + 1U) &
                              (VISION_RX_RING_SIZE - 1U));
  return 1U;
}

static void Vision_Process_Byte(uint8_t byte, uint32_t now_ms)
{
  if (byte == '$') {
    vision_frame_collecting = 1U;
    vision_frame_length = 0U;
    vision_frame[vision_frame_length++] = '$';
    return;
  }
  if (!vision_frame_collecting) return;
  if (vision_frame_length >= VISION_FRAME_MAX_LENGTH) {
    vision_frame_collecting = 0U;
    vision_frame_length = 0U;
    return;
  }

  vision_frame[vision_frame_length++] = (char)byte;
  if (byte == '\n') {
    vision_frame[vision_frame_length] = '\0';
    Vision_Handle_Frame(vision_frame, vision_frame_length, now_ms);
    vision_frame_collecting = 0U;
    vision_frame_length = 0U;
  }
}

uint8_t VisionTest_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &gpio);

  vision_uart.Instance = USART1;
  vision_uart.Init.BaudRate = 115200U;
  vision_uart.Init.WordLength = UART_WORDLENGTH_8B;
  vision_uart.Init.StopBits = UART_STOPBITS_1;
  vision_uart.Init.Parity = UART_PARITY_NONE;
  vision_uart.Init.Mode = UART_MODE_TX_RX;
  vision_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  vision_uart.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&vision_uart) != HAL_OK) return 0U;

  HAL_NVIC_SetPriority(USART1_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  __HAL_UART_ENABLE_IT(&vision_uart, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&vision_uart, UART_IT_ERR);

  (void)OLED_Init();
  Vision_Display_Text("LINK", 4U);
  vision_last_hello_tick = HAL_GetTick() - VISION_HELLO_INTERVAL_MS;
  return 1U;
}

void VisionTest_Process(uint32_t now_ms)
{
  uint8_t byte;

  while (Vision_Rx_Pop(&byte)) Vision_Process_Byte(byte, now_ms);

  if (vision_rx_overflow) {
    vision_rx_overflow = 0U;
    vision_frame_collecting = 0U;
    vision_frame_length = 0U;
  }

  if (vision_state == VISION_LINK_WAIT) {
    if (now_ms - vision_last_hello_tick >= VISION_HELLO_INTERVAL_MS) {
      vision_last_hello_tick = now_ms;
      Vision_Send_Hello();
    }
  } else if (vision_state == VISION_CANCEL_WAIT) {
    if (now_ms - vision_last_set_tick >= VISION_SET_RETRY_MS) {
      vision_last_set_tick = now_ms;
      Vision_Send_Set(0U, 0U);
    }
  } else if (vision_state == VISION_RESULT_WAIT) {
    if (!vision_set_ack_seen && vision_set_retries < VISION_SET_RETRY_LIMIT &&
        now_ms - vision_last_set_tick >= VISION_SET_RETRY_MS) {
      vision_last_set_tick = now_ms;
      vision_set_retries++;
      Vision_Send_Set(3U, 30000U);
    }
    if (Vision_Time_Reached(now_ms, vision_deadline_tick)) {
      Vision_Display_Text("FAIL", 4U);
      Vision_Start_Cancel(now_ms);
    }
  } else if (vision_state == VISION_RESTART_WAIT &&
             Vision_Time_Reached(now_ms, vision_restart_tick)) {
    Vision_Start_Roman(now_ms);
  }
}

void USART1_IRQHandler(void)
{
  uint32_t status = USART1->SR;

  if ((status & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) != 0U) {
    volatile uint32_t discard = USART1->DR;
    (void)discard;
    vision_rx_overflow = 1U;
  } else if ((status & USART_SR_RXNE) != 0U) {
    uint8_t byte = (uint8_t)USART1->DR;
    uint16_t next = (uint16_t)((vision_rx_head + 1U) &
                               (VISION_RX_RING_SIZE - 1U));
    if (next != vision_rx_tail) {
      vision_rx_ring[vision_rx_head] = byte;
      vision_rx_head = next;
    } else {
      vision_rx_overflow = 1U;
    }
  }
}
