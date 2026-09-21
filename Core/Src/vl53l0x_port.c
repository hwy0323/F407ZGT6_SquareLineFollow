#include "main.h"
#include "vl53l0x_platform.h"

/* PA8/PC9 use the STM32 hardware I2C3 peripheral. */
static I2C_HandleTypeDef *vl53_i2c = NULL;

void VL53L0X_Port_Set_I2C(I2C_HandleTypeDef *i2c)
{
  vl53_i2c = i2c;
}

int32_t VL53L0X_comms_initialise(uint8_t comms_type, uint16_t comms_speed_khz)
{
  (void)comms_type;
  (void)comms_speed_khz;
  return (vl53_i2c != NULL) ? 0 : 1;
}

int32_t VL53L0X_comms_close(void) { return 0; }

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t *data, int32_t count)
{
  if (vl53_i2c == NULL || count < 1) return 1;
  return (HAL_I2C_Mem_Write(vl53_i2c, (uint16_t)(address << 1), index,
                            I2C_MEMADD_SIZE_8BIT, data, (uint16_t)count,
                            100U) == HAL_OK) ? 0 : 1;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *data, int32_t count)
{
  if (vl53_i2c == NULL || count < 1) return 1;
  return (HAL_I2C_Mem_Read(vl53_i2c, (uint16_t)(address << 1), index,
                           I2C_MEMADD_SIZE_8BIT, data, (uint16_t)count,
                           100U) == HAL_OK) ? 0 : 1;
}

int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
  return VL53L0X_write_multi(address, index, &data, 1);
}

int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data)
{
  uint8_t buffer[2] = {(uint8_t)(data >> 8), (uint8_t)data};
  return VL53L0X_write_multi(address, index, buffer, 2);
}

int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data)
{
  uint8_t buffer[4] = {(uint8_t)(data >> 24), (uint8_t)(data >> 16),
                       (uint8_t)(data >> 8), (uint8_t)data};
  return VL53L0X_write_multi(address, index, buffer, 4);
}

int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *data)
{
  return VL53L0X_read_multi(address, index, data, 1);
}

int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *data)
{
  uint8_t buffer[2];
  if (VL53L0X_read_multi(address, index, buffer, 2) != 0) return 1;
  *data = ((uint16_t)buffer[0] << 8) | buffer[1];
  return 0;
}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *data)
{
  uint8_t buffer[4];
  if (VL53L0X_read_multi(address, index, buffer, 4) != 0) return 1;
  *data = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) |
          ((uint32_t)buffer[2] << 8) | buffer[3];
  return 0;
}

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV dev)
{
  (void)dev;
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV dev)
{
  (void)dev;
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV dev, uint8_t index, uint8_t *data, uint32_t count)
{
  return (VL53L0X_write_multi(dev->I2cDevAddr, index, data, (int32_t)count) == 0) ?
         VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV dev, uint8_t index, uint8_t *data, uint32_t count)
{
  return (VL53L0X_read_multi(dev->I2cDevAddr, index, data, (int32_t)count) == 0) ?
         VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV dev, uint8_t index, uint8_t data)
{
  return (VL53L0X_write_byte(dev->I2cDevAddr, index, data) == 0) ?
         VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV dev, uint8_t index, uint16_t data)
{
  return (VL53L0X_write_word(dev->I2cDevAddr, index, data) == 0) ?
         VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV dev, uint8_t index, uint32_t data)
{
  return (VL53L0X_write_dword(dev->I2cDevAddr, index, data) == 0) ?
         VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV dev, uint8_t index, uint8_t and_data, uint8_t or_data)
{
  uint8_t data;
  VL53L0X_Error status = VL53L0X_RdByte(dev, index, &data);
  if (status != VL53L0X_ERROR_NONE) return status;
  return VL53L0X_WrByte(dev, index, (uint8_t)((data & and_data) | or_data));
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV dev, uint8_t index, uint8_t *data)
{
  return (VL53L0X_read_byte(dev->I2cDevAddr, index, data) == 0) ?
         VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV dev, uint8_t index, uint16_t *data)
{
  return (VL53L0X_read_word(dev->I2cDevAddr, index, data) == 0) ?
         VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV dev, uint8_t index, uint32_t *data)
{
  return (VL53L0X_read_dword(dev->I2cDevAddr, index, data) == 0) ?
         VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV dev)
{
  (void)dev;
  HAL_Delay(1U);
  return VL53L0X_ERROR_NONE;
}

int32_t VL53L0X_platform_wait_us(int32_t wait_us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = (uint32_t)wait_us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < cycles) { }
  return 0;
}

int32_t VL53L0X_wait_ms(int32_t wait_ms) { HAL_Delay((uint32_t)wait_ms); return 0; }
int32_t VL53L0X_set_gpio(uint8_t level) { (void)level; return 0; }
int32_t VL53L0X_get_gpio(uint8_t *level) { *level = 0U; return 0; }
int32_t VL53L0X_release_gpio(void) { return 0; }
int32_t VL53L0X_cycle_power(void) { return 0; }
int32_t VL53L0X_get_timer_frequency(int32_t *frequency) { *frequency = 1000; return 0; }
int32_t VL53L0X_get_timer_value(int32_t *value) { *value = (int32_t)HAL_GetTick(); return 0; }
