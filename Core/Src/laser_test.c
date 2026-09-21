#include "laser_test.h"
#include "vl53l0x_api.h"

#define VL53L0X_ADDRESS 0x29U
#define VL53L0X_MODEL_ID_REGISTER 0xC0U
#define VL53L0X_EXPECTED_MODEL_ID 0xEEAAU

/* Values shown through ST-Link Live Expressions. */
volatile uint16_t laser_distance_mm = 0U;
volatile uint16_t laser_model_id = 0U;
volatile uint8_t laser_status = 0U;
volatile uint8_t laser_range_status = 0xFFU;
volatile uint8_t laser_detected_address = 0U;
volatile uint8_t laser_bus_pins = 0U;
volatile uint32_t laser_sample_count = 0U;
volatile int32_t laser_api_error = 0;

static VL53L0X_Dev_t laser_device;
static uint8_t laser_running = 0U;

extern void VL53L0X_Port_Set_I2C(I2C_HandleTypeDef *i2c);

static uint8_t Laser_Call(VL53L0X_Error result)
{
  laser_api_error = (int32_t)result;
  return (result == VL53L0X_ERROR_NONE) ? 1U : 0U;
}

void LaserTest_Init(I2C_HandleTypeDef *i2c)
{
  uint8_t id[2];
  uint32_t ref_spad_count;
  uint8_t aperture_spads;
  uint8_t vhv_settings;
  uint8_t phase_cal;
  uint8_t address;

  laser_status = 0U;
  laser_running = 0U;
  VL53L0X_Port_Set_I2C(i2c);

  /* XSH gives this one sensor a clean power-on reset. */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_Delay(30U);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(30U);

  /* Bit0=SCL, bit1=SDA. Both should normally read high on an idle I2C bus. */
  laser_bus_pins = 0U;
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) laser_bus_pins |= 1U;
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET) laser_bus_pins |= 2U;

  if (HAL_I2C_IsDeviceReady(i2c, VL53L0X_ADDRESS << 1, 3U, 100U) != HAL_OK) {
    /* Keep the first acknowledged 7-bit address for wiring diagnosis. */
    for (address = 1U; address < 0x7FU; address++) {
      if (HAL_I2C_IsDeviceReady(i2c, address << 1, 1U, 5U) == HAL_OK) {
        laser_detected_address = address;
        break;
      }
    }
    laser_status = 1U; /* I2C address 0x29 did not acknowledge. */
    return;
  }

  if (HAL_I2C_Mem_Read(i2c, VL53L0X_ADDRESS << 1, VL53L0X_MODEL_ID_REGISTER,
                       I2C_MEMADD_SIZE_8BIT, id, 2U, 100U) != HAL_OK) {
    laser_status = 1U;
    return;
  }
  laser_model_id = ((uint16_t)id[0] << 8) | id[1];
  if (laser_model_id != VL53L0X_EXPECTED_MODEL_ID) {
    laser_status = 2U; /* Acknowledged, but is not the expected VL53L0X. */
    return;
  }

  laser_device.I2cDevAddr = VL53L0X_ADDRESS;
  laser_device.comms_type = I2C;
  laser_device.comms_speed_khz = 100U;

  if (!Laser_Call(VL53L0X_DataInit(&laser_device)) ||
      !Laser_Call(VL53L0X_StaticInit(&laser_device)) ||
      !Laser_Call(VL53L0X_PerformRefCalibration(&laser_device, &vhv_settings, &phase_cal)) ||
      !Laser_Call(VL53L0X_PerformRefSpadManagement(&laser_device, &ref_spad_count, &aperture_spads)) ||
      !Laser_Call(VL53L0X_SetDeviceMode(&laser_device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING)) ||
      !Laser_Call(VL53L0X_StartMeasurement(&laser_device))) {
    laser_status = 3U; /* API initialization failure; see laser_api_error. */
    return;
  }

  laser_status = 4U; /* Running. */
  laser_running = 1U;
}

void LaserTest_Process(void)
{
  VL53L0X_RangingMeasurementData_t data;
  uint8_t ready = 0U;

  if (laser_running == 0U) return;
  if (!Laser_Call(VL53L0X_GetMeasurementDataReady(&laser_device, &ready))) {
    laser_status = 5U;
    laser_running = 0U;
    return;
  }
  if (ready == 0U) return;

  if (!Laser_Call(VL53L0X_GetRangingMeasurementData(&laser_device, &data)) ||
      !Laser_Call(VL53L0X_ClearInterruptMask(&laser_device,
                   VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY))) {
    laser_status = 5U;
    laser_running = 0U;
    return;
  }

  laser_distance_mm = data.RangeMilliMeter;
  laser_range_status = data.RangeStatus;
  laser_sample_count++;
}
