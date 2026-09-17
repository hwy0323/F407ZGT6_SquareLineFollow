#include "route_recorder.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* Sector 11 starts at 0x080E0000 on the 1 MB STM32F407ZG flash. */
#define ROUTE_FLASH_ADDRESS 0x080E0000U
#define ROUTE_STORE_MAGIC 0x52544D31U
#define ROUTE_SLOT_MAGIC 0x52544D32U
#define ROUTE_STORE_VERSION 1U
#define ROUTE_SLOT_COUNT 3U
#define ROUTE_MAX_POINTS 11U

/* This firmware records one route only. Change this for the next teaching run. */
#define FIXED_RECORD_SLOT 1U

#define SELECT_FINISH_MS 1500U
#define CONFIRM_BLINK_MS 180U
#define COMPLETE_BLINK_MS 120U
#define COMPLETE_HOLD_MS 2500U

typedef enum {
  RECORDER_SELECT_ROUTE,
  RECORDER_ARMED,
  RECORDER_RECORDING,
  RECORDER_COMPLETE
} RecorderState;

typedef struct {
  int32_t left_front;
  int32_t left_rear;
  int32_t right_rear;
} RoutePoint;

typedef struct {
  uint32_t valid;
  uint32_t point_count;
  RoutePoint point[ROUTE_MAX_POINTS];
  uint32_t checksum;
} RouteSlot;

typedef struct {
  uint32_t magic;
  uint32_t version;
  RouteSlot slot[ROUTE_SLOT_COUNT];
} RouteStore;

static RouteStore route_store;
static RecorderState recorder_state;
static uint8_t selection_clicks;
static uint8_t selected_slot;
static uint8_t required_points;
static uint8_t recorded_points;
static uint8_t led_is_on;
static uint8_t blink_toggles;
static uint32_t selection_tick;
static uint32_t blink_tick;
static uint32_t complete_tick;

static uint32_t Slot_Checksum(const RouteSlot *slot)
{
  const uint32_t *word = (const uint32_t *)slot;
  uint32_t checksum = 0x5A17C3E9U;
  uint32_t index;

  for (index = 0U; index < (sizeof(RouteSlot) / sizeof(uint32_t)) - 1U; index++) {
    checksum = (checksum << 5U) | (checksum >> 27U);
    checksum ^= word[index];
  }
  return checksum;
}

static uint8_t Slot_Is_Valid(const RouteSlot *slot)
{
  if (slot->valid != ROUTE_SLOT_MAGIC) return 0U;
  if (slot->point_count == 0U || slot->point_count > ROUTE_MAX_POINTS) return 0U;
  return (slot->checksum == Slot_Checksum(slot)) ? 1U : 0U;
}

static uint8_t Points_For_Slot(uint8_t slot)
{
  if (slot == 1U) return 5U;   /* Square: A, B, C, D, A. */
  if (slot == 2U) return 6U;   /* BC reverse parking. */
  if (slot == 3U) return 11U;  /* DA side parking. */
  return 0U;
}

static void Start_Blink(uint8_t flashes, uint32_t now, uint32_t interval)
{
  blink_toggles = (uint8_t)(flashes * 2U);
  blink_tick = now - interval;
  led_is_on = 0U;
}

static uint8_t Flash_Save_All(void)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t sector_error = 0U;
  uint32_t address;
  uint32_t index;
  const uint32_t *word = (const uint32_t *)&route_store;

  HAL_FLASH_Unlock();

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Sector = FLASH_SECTOR_11;
  erase.NbSectors = 1U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
    HAL_FLASH_Lock();
    return 0U;
  }

  address = ROUTE_FLASH_ADDRESS;
  for (index = 0U; index < sizeof(RouteStore) / sizeof(uint32_t); index++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, word[index]) != HAL_OK) {
      HAL_FLASH_Lock();
      return 0U;
    }
    address += sizeof(uint32_t);
  }

  HAL_FLASH_Lock();
  return 1U;
}

static void Finish_Recording(uint32_t now)
{
  RouteSlot *slot = &route_store.slot[selected_slot - 1U];

  slot->valid = ROUTE_SLOT_MAGIC;
  slot->point_count = recorded_points;
  slot->checksum = Slot_Checksum(slot);

  recorder_state = RECORDER_COMPLETE;
  complete_tick = now;
  if (Flash_Save_All()) Start_Blink(5U, now, COMPLETE_BLINK_MS);
  else Start_Blink(2U, now, COMPLETE_BLINK_MS);
}

void RouteRecorder_Init(void)
{
  const RouteStore *flash_store = (const RouteStore *)ROUTE_FLASH_ADDRESS;
  uint32_t index;

  memset(&route_store, 0, sizeof(route_store));
  if (flash_store->magic == ROUTE_STORE_MAGIC &&
      flash_store->version == ROUTE_STORE_VERSION) {
    memcpy(&route_store, flash_store, sizeof(route_store));
  } else {
    route_store.magic = ROUTE_STORE_MAGIC;
    route_store.version = ROUTE_STORE_VERSION;
  }

  for (index = 0U; index < ROUTE_SLOT_COUNT; index++) {
    if (!Slot_Is_Valid(&route_store.slot[index])) {
      memset(&route_store.slot[index], 0, sizeof(RouteSlot));
    }
  }

  recorder_state = RECORDER_SELECT_ROUTE;
  selection_clicks = 0U;
  selected_slot = FIXED_RECORD_SLOT;
  required_points = Points_For_Slot(FIXED_RECORD_SLOT);
  recorded_points = 0U;
  led_is_on = 0U;
  blink_toggles = 0U;

  /* No menu: after boot, the first short press records the start point. */
  recorder_state = RECORDER_ARMED;
  Start_Blink(FIXED_RECORD_SLOT, HAL_GetTick(), CONFIRM_BLINK_MS);
}

void RouteRecorder_Handle_Short_Press(uint32_t now,
                                      int32_t left_front,
                                      int32_t left_rear,
                                      int32_t right_rear)
{
  RouteSlot *slot;

  if (recorder_state == RECORDER_SELECT_ROUTE) {
    if (selection_clicks < ROUTE_SLOT_COUNT) selection_clicks++;
    selection_tick = now;
    return;
  }

  if (recorder_state == RECORDER_ARMED) {
    slot = &route_store.slot[selected_slot - 1U];
    memset(slot, 0, sizeof(*slot));
    recorded_points = 0U;
    recorder_state = RECORDER_RECORDING;
    blink_toggles = 0U;
    led_is_on = 1U;
  }

  if (recorder_state != RECORDER_RECORDING) return;
  if (recorded_points >= required_points) return;

  slot = &route_store.slot[selected_slot - 1U];
  slot->point[recorded_points].left_front = left_front;
  slot->point[recorded_points].left_rear = left_rear;
  slot->point[recorded_points].right_rear = right_rear;
  recorded_points++;

  if (recorded_points >= required_points) Finish_Recording(now);
}

void RouteRecorder_Process(uint32_t now)
{
  uint32_t interval;

  if (recorder_state == RECORDER_SELECT_ROUTE && selection_clicks != 0U &&
      now - selection_tick >= SELECT_FINISH_MS) {
    selected_slot = selection_clicks;
    required_points = Points_For_Slot(selected_slot);
    recorder_state = RECORDER_ARMED;
    Start_Blink(selected_slot, now, CONFIRM_BLINK_MS);
  }

  interval = (recorder_state == RECORDER_COMPLETE) ? COMPLETE_BLINK_MS : CONFIRM_BLINK_MS;
  if (blink_toggles != 0U && now - blink_tick >= interval) {
    blink_tick = now;
    led_is_on = led_is_on ? 0U : 1U;
    blink_toggles--;
    if (blink_toggles == 0U) led_is_on = 0U;
  }
}

uint8_t RouteRecorder_LED_Is_On(void)
{
  return led_is_on;
}
