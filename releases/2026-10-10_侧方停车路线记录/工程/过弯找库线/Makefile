ROOT := ../../../../
TOOL_ROOT := F:/cbide/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344/tools/bin
CC := $(TOOL_ROOT)/arm-none-eabi-gcc.exe
OBJCOPY := $(TOOL_ROOT)/arm-none-eabi-objcopy.exe
SIZE := $(TOOL_ROOT)/arm-none-eabi-size.exe
TARGET := build/F407ZGT6_OneCorner_To_ParkingStart
INCLUDES := -I$(ROOT)/Core/Inc -I$(ROOT)/Drivers/STM32F4xx_HAL_Driver/Inc -I$(ROOT)/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I$(ROOT)/Drivers/CMSIS/Include -I$(ROOT)/Drivers/CMSIS/Device/ST/STM32F4xx/Include
CFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -DSTM32F407xx -DUSE_HAL_DRIVER -DGIT_VERIFIED_CORNER $(INCLUDES) -O0 -g3 -Wall -Wextra -ffunction-sections -fdata-sections -std=gnu11
LDFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard --specs=nano.specs --specs=nosys.specs -T$(ROOT)/STM32F407ZGTx_FLASH.ld -Wl,--gc-sections -Wl,-Map=$(TARGET).map -Wl,--start-group -lc -lm -Wl,--end-group
SOURCES := main.c startup_stm32f407xx.c system_stm32f4xx.c stm32f4xx_hal.c stm32f4xx_hal_cortex.c stm32f4xx_hal_gpio.c stm32f4xx_hal_pwr.c stm32f4xx_hal_pwr_ex.c stm32f4xx_hal_rcc.c stm32f4xx_hal_rcc_ex.c stm32f4xx_hal_tim.c stm32f4xx_hal_tim_ex.c stm32f4xx_hal_uart.c
OBJECTS := $(addprefix build/,$(SOURCES:.c=.o))
vpath %.c . $(ROOT)/Core/Src $(ROOT)/Drivers/STM32F4xx_HAL_Driver/Src
all: $(TARGET).elf $(TARGET).bin
$(TARGET).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@
$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@
build/%.o: %.c
	@if not exist build mkdir build
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@
-include $(OBJECTS:.o=.d)
.PHONY: all
