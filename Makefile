TOOL_ROOT := F:/cbide/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344/tools/bin
CC := $(TOOL_ROOT)/arm-none-eabi-gcc.exe
OBJCOPY := $(TOOL_ROOT)/arm-none-eabi-objcopy.exe
SIZE := $(TOOL_ROOT)/arm-none-eabi-size.exe
TARGET := Debug/F407ZGT6_SquareLineFollow
INCLUDES := -ICore/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Include -IDrivers/CMSIS/Device/ST/STM32F4xx/Include
CFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -DSTM32F407xx -DUSE_HAL_DRIVER $(INCLUDES) -O0 -g3 -Wall -Wextra -ffunction-sections -fdata-sections -std=gnu11
LDFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard --specs=nano.specs --specs=nosys.specs -TSTM32F407ZGTx_FLASH.ld -Wl,--gc-sections -Wl,-Map=$(TARGET).map -Wl,--start-group -lc -lm -Wl,--end-group
SOURCES := Core/Src/main.c Core/Src/menu.c Core/Src/oled.c Core/Src/menu_display.c Core/Src/route_recorder.c Core/Src/laser_test.c Core/Src/vl53l0x_port.c Core/Src/vl53l0x_api.c Core/Src/vl53l0x_api_calibration.c Core/Src/vl53l0x_api_core.c Core/Src/vl53l0x_api_ranging.c Core/Src/vl53l0x_api_strings.c Core/Src/system_stm32f4xx.c Core/Src/stm32f4xx_it.c Core/Src/startup_stm32f407xx.c Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
OBJECTS := $(addprefix Debug/,$(notdir $(SOURCES:.c=.o)))
vpath %.c Core/Src Drivers/STM32F4xx_HAL_Driver/Src
all: $(TARGET).elf $(TARGET).bin
$(TARGET).elf: $(OBJECTS) | Debug
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@
$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@
Debug:
	@if not exist Debug mkdir Debug
Debug/%.o: %.c | Debug
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@
clean:
	@if exist Debug\*.o del /Q Debug\*.o
	@if exist Debug\*.d del /Q Debug\*.d
	@if exist Debug\*.elf del /Q Debug\*.elf
	@if exist Debug\*.bin del /Q Debug\*.bin
	@if exist Debug\*.map del /Q Debug\*.map
-include $(OBJECTS:.o=.d)
.PHONY: all clean Debug
