# NUCLEO-H723ZG firmware
#
#   make           build build/h723.elf (debug)
#   make DEBUG=0   optimized build
#   make FILTER=all  control build: accept-all filter instead of the other
#                  board's ID only (see README, step 7)
#   make flash     flash over the ST-LINK (SWD) and reset; with several
#                  ST-LINKs, picks the one whose target has DEVICE_ID
#   make clean

TARGET   := h723
BUILD    := build
DEBUG    ?= 1
FILTER   ?= id

LIB      := ../lib
HAL      := $(LIB)/stm32h7xx-hal-driver
DEV      := $(LIB)/cmsis-device-h7

PREFIX   := arm-none-eabi-
CC       := $(PREFIX)gcc
OBJCOPY  := $(PREFIX)objcopy
SIZE     := $(PREFIX)size
DEVICE_ID := 0x483

# Sources
C_SRCS := \
	Src/main.c \
	Src/stm32h7xx_it.c \
	$(DEV)/Source/Templates/system_stm32h7xx.c \
	$(addprefix $(HAL)/Src/stm32h7xx_hal, \
		.c _adc.c _adc_ex.c _cortex.c _dma.c _dma_ex.c _exti.c _fdcan.c _flash.c _flash_ex.c \
		_gpio.c _pwr.c _pwr_ex.c _rcc.c _rcc_ex.c _uart.c _uart_ex.c)

ASM_SRCS := $(DEV)/Source/Templates/gcc/startup_stm32h723xx.s

INCLUDES := -IInc -I../common -I$(HAL)/Inc -I$(DEV)/Include -I$(LIB)/cmsis_core/Include
DEFINES  := -DSTM32H723xx -DUSE_HAL_DRIVER -DUSE_PWR_LDO_SUPPLY
LDSCRIPT := STM32H723ZGTx_FLASH.ld

# Flags
MCU     := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard

ifeq ($(FILTER),all)
DEFINES  += -DP1_FILTER_ACCEPT_ALL
endif

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

# main.c depends on FILTER: a stamp file named after the current value makes
# it rebuild when FILTER changes
$(BUILD)/Src/main.o: $(BUILD)/filter.$(FILTER)
$(BUILD)/filter.$(FILTER):
	@mkdir -p $(BUILD)
	@rm -f $(BUILD)/filter.*
	@touch $@

flash: $(BUILD)/$(TARGET).elf
	../tools/flash.sh $(DEVICE_ID) $<

clean:
	rm -rf $(BUILD)

-include $(OBJS:.o=.d)
