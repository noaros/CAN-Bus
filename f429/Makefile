# NUCLEO-F429ZI firmware
#
#   make           build build/f429.elf (debug)
#   make DEBUG=0   optimized build
#   make flash     flash over the ST-LINK (SWD) and reset
#   make clean

TARGET   := f429
BUILD    := build
DEBUG    ?= 1

LIB      := ../lib
HAL      := $(LIB)/stm32f4xx-hal-driver
DEV      := $(LIB)/cmsis-device-f4

PREFIX   := arm-none-eabi-
CC       := $(PREFIX)gcc
OBJCOPY  := $(PREFIX)objcopy
SIZE     := $(PREFIX)size
FLASHER  := STM32_Programmer_CLI

# Sources
C_SRCS := \
	Src/main.c \
	Src/stm32f4xx_it.c \
	$(DEV)/Source/Templates/system_stm32f4xx.c \
	$(addprefix $(HAL)/Src/stm32f4xx_hal, \
		.c _can.c _cortex.c _dma.c _dma_ex.c _exti.c _flash.c _flash_ex.c \
		_gpio.c _pwr.c _pwr_ex.c _rcc.c _rcc_ex.c _uart.c)

ASM_SRCS := $(DEV)/Source/Templates/gcc/startup_stm32f429xx.s

INCLUDES := -IInc -I$(HAL)/Inc -I$(DEV)/Include -I$(LIB)/cmsis_core/Include
DEFINES  := -DSTM32F429xx -DUSE_HAL_DRIVER
LDSCRIPT := STM32F429ZITx_FLASH.ld

# Flags
MCU     := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

ifeq ($(DEBUG),1)
OPT     := -Og -g3
else
OPT     := -O2
endif

CFLAGS  := $(MCU) $(DEFINES) $(INCLUDES) $(OPT) -std=c11 -Wall -Wextra \
           -ffunction-sections -fdata-sections -MMD -MP
ASFLAGS := $(MCU) -x assembler-with-cpp
LDFLAGS := $(MCU) -T$(LDSCRIPT) --specs=nano.specs --specs=nosys.specs \
           -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map \
           -Wl,--print-memory-usage -Wl,--no-warn-rwx-segments

# Object files mirror the source tree under build/ ("../" becomes "_/")
obj = $(BUILD)/$(subst ../,_/,$(1))
OBJS := $(foreach s,$(C_SRCS),$(call obj,$(s:.c=.o))) \
        $(foreach s,$(ASM_SRCS),$(call obj,$(s:.s=.o)))

.PHONY: all flash clean

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin

$(BUILD)/$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/_/%.o: ../%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/_/%.o: ../%.s
	@mkdir -p $(dir $@)
	$(CC) $(ASFLAGS) -c $< -o $@

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

flash: $(BUILD)/$(TARGET).elf
	$(FLASHER) -c port=SWD -w $< -rst

clean:
	rm -rf $(BUILD)

-include $(OBJS:.o=.d)
