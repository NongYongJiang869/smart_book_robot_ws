TARGET := app

BUILD_DIR := build

MKDIR_BUILD := mkdir -p "$(BUILD_DIR)"
RMDIR_BUILD := rm -rf "$(BUILD_DIR)"

LIB_ROOT := stdlib
SPL_DIR := $(LIB_ROOT)/STM32F10x_StdPeriph_Driver
CMSIS_DEVICE_DIR := $(LIB_ROOT)/CMSIS/CM3/DeviceSupport/ST/STM32F10x
CMSIS_CORE_DIR := $(LIB_ROOT)/CMSIS/CM3/CoreSupport

CC := arm-none-eabi-gcc
AS := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size

CPU_FLAGS := -mcpu=cortex-m3 -mthumb
DEFS := -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER

INCLUDES := \
	-Iinc \
	-I$(SPL_DIR)/inc \
	-I$(CMSIS_DEVICE_DIR) \
	-I$(CMSIS_CORE_DIR)

CFLAGS := $(CPU_FLAGS) $(DEFS) $(INCLUDES) -std=c11 -Wall -Wextra -Og -g3 -ffunction-sections -fdata-sections
ASFLAGS := $(CPU_FLAGS) -x assembler-with-cpp -g3

LDSCRIPT := ld/stm32f103c8t6.ld
LDFLAGS := $(CPU_FLAGS) -T$(LDSCRIPT) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs -lm

PROJECT_C_SOURCES := \
	src/main.c \
	src/motor.c \
	src/bsp_delay.c \
	src/bsp_usart.c \
	src/encoder.c \
	src/protocol.c \
	src/mpu6050.c \
	src/stm32f10x_it.c \
	src/system_stm32f10x.c

SPL_C_SOURCES := \
	$(SPL_DIR)/src/stm32f10x_gpio.c \
	$(SPL_DIR)/src/stm32f10x_rcc.c \
	$(SPL_DIR)/src/stm32f10x_tim.c \
	$(SPL_DIR)/src/stm32f10x_usart.c \
	$(SPL_DIR)/src/stm32f10x_i2c.c \
	$(SPL_DIR)/src/misc.c

ASM_SOURCES := startup/startup_stm32f10x_md.s

C_SOURCES := $(PROJECT_C_SOURCES) $(SPL_C_SOURCES)
C_OBJECTS := $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
ASM_OBJECTS := $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
OBJECTS := $(C_OBJECTS) $(ASM_OBJECTS)

ELF := $(BUILD_DIR)/$(TARGET).elf
HEX := $(BUILD_DIR)/$(TARGET).hex
BIN := $(BUILD_DIR)/$(TARGET).bin

.PHONY: all clean flash erase size compiledb

all: $(ELF) $(HEX) $(BIN)

$(ELF): $(OBJECTS) | $(BUILD_DIR)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(HEX): $(ELF)
	$(OBJCOPY) -O ihex $< $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR):
	$(MKDIR_BUILD)

$(BUILD_DIR)/%.o: src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: $(SPL_DIR)/src/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: startup/%.s | $(BUILD_DIR)
	$(AS) $(ASFLAGS) -c $< -o $@

OPENOCD ?= openocd
OPENOCD_CFG ?= tools/openocd/openocd.cfg

flash: $(ELF)
	$(OPENOCD) -f $(OPENOCD_CFG) -c "program $(ELF) verify reset exit"

erase:
	$(OPENOCD) -f $(OPENOCD_CFG) -c "init; reset halt; stm32f1x mass_erase 0; reset run; shutdown"

size: $(ELF)
	$(SIZE) $(ELF)

clean:
	$(RMDIR_BUILD)

compiledb:
	compiledb make all
