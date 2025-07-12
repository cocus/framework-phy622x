##############################################################################
PROJECT_NAME ?= phy622x
PROJECT_DEFS ?=
FREERTOS ?= 1
OFFLOAD_GPIO ?= 1
BTSTACK ?= 0
##############################################################################
COM_PORT = COM32
COM_SPEED = 250000
COM_STARTBAUD = 115200
UART_LOG_BPS = 115200
##############################################################################
# Source
SRC_PATH = ./examples/simple_no_platformio
SRC_PRJ = main.c

INCLUDES = -I$(SRC_PATH)

SRCS = $(addprefix $(SRC_PATH)/, $(SRC_PRJ))
##############################################################################
DEFINES = -D__GCC
DEFINES += $(PROJECT_DEFS)

DEFINES += -DARMCM0
DEFINES += -DPHY_MCU_TYPE=MCU_BUMBEE_M0
DEFINES += -DDEBUG_INFO=3
DEFINES += -DUSE_ROM_GPIO
##############################################################################
BIN_DIR = ./bin
OBJ_DIR = ./build
PYTHON = python3
GCC_PATH =
CC = $(GCC_PATH)arm-none-eabi-gcc
OBJCOPY = $(GCC_PATH)arm-none-eabi-objcopy
OBJDUMP = $(GCC_PATH)arm-none-eabi-objdump
SIZE = $(GCC_PATH)arm-none-eabi-size
READELF = $(GCC_PATH)arm-none-eabi-readelf
##############################################################################
ARCH_FLAGS := -mcpu=cortex-m0 -mthumb -mthumb-interwork
OPT_CFLAGS ?= -Os
DEB_CFLAGS ?= -g3 -ggdb
##############################################################################
ASFLAGS	   := $(ARCH_FLAGS) $(OPT_CFLAGS) $(DEB_CFLAGS)
CFLAGS     := $(ARCH_FLAGS) $(OPT_CFLAGS) $(DEB_CFLAGS)

CFLAGS     += -W -Wall --std=gnu99
CFLAGS     += -fno-diagnostics-show-caret
CFLAGS     += -fdata-sections -ffunction-sections
CFLAGS     += -funsigned-char -funsigned-bitfields
CFLAGS     += -fms-extensions
CFLAGS     += -specs=nosys.specs
CFLAGS     += -Wl,--gc-sections

LDSCRIPT   ?= bsp/phy6222.ld

ADD_OPT =

LDFLAGS    := $(ARCH_FLAGS)
LDFLAGS    += --static -nostartfiles -nostdlib
LDFLAGS    += -Wl,--gc-sections
LDFLAGS    += -Wl,--script=$(LDSCRIPT)
# LDFLAGS    += -Wl,--no-warn-rwx-segments
LDFLAGS    += -Wl,--just-symbols=bsp/symbols/phy6222_rom.gcc
LDFLAGS    += -Wl,-Map=$(OBJ_DIR)/$(PROJECT_NAME).map
LDFLAGS    += -Wl,--print-memory-usage
LIBS       += -Wl,--start-group -lgcc -lnosys -Wl,--end-group

##############################################################################

INCLUDES += -Ibsp/ -Ibsp/CMSIS/include -Ibsp/CMSIS/device/phyplus/ -Ibsp/log

##############################################################################

STARTUP_ASM = bsp/CMSIS/device/phyplus/phy6222_start.s
SRCS += bsp/CMSIS/device/phyplus/phy6222_cstart.c
SRCS += bsp/osal_nuker.c

###########################################
# LIBs
SRCS += bsp/jump_table.c \
	bsp/ble/rf/patch.c \
	bsp/ble/controller/rf_phy_driver.c \
	bsp/driver/adc/adc.c \
	bsp/driver/aes/aes.c \
	bsp/driver/clock/clock.c \
	bsp/driver/dma/dma.c \
	bsp/driver/flash/flash.c \
	bsp/driver/gpio/gpio.c \
	bsp/driver/i2c/i2c.c \
	bsp/driver/pwm/pwm.c \
	bsp/driver/pwrmgr/pwrmgr.c \
	bsp/driver/spi/spi.c \
	bsp/driver/uart/uart.c \
	bsp/log/log.c \

ifeq ($(FREERTOS),1)
DEFINES += -DENABLE_FREERTOS
FREERTOS_PATH = ./freertos

SRCS_FREERTOS = portable/GCC/ARM_CM0/port.c
SRCS_FREERTOS += portable/GCC/ARM_CM0/portasm.c
SRCS_FREERTOS += portable/MemMang/heap_4.c

SRCS_FREERTOS += queue.c
SRCS_FREERTOS += list.c
SRCS_FREERTOS += tasks.c
SRCS_FREERTOS += timers.c
SRCS_FREERTOS += event_groups.c
SRCS_FREERTOS += croutine.c

INCLUDES += -I$(FREERTOS_PATH)
INCLUDES += -I$(FREERTOS_PATH)/include
INCLUDES += -I$(FREERTOS_PATH)/portable/GCC/ARM_CM0

SRCS += $(addprefix $(FREERTOS_PATH)/, $(SRCS_FREERTOS))
endif

##############################################################################

CFLAGS  += $(DEFINES) $(INCLUDES)

SRC_O = $(SRCS:%.c=%.o) $(STARTUP_ASM:%.s=%.o)
OBJS = $(patsubst %, $(OBJ_DIR)/%, $(patsubst ./%, %, $(SRC_O)))

DEPENDENCY_LIST = $(OBJS:%o=%d)

##############################################################################
.PHONY: all directory clean size flash erase_and_flash

all: directory $(GATT_OBJ) $(SRC_O) $(OBJ_DIR)/$(PROJECT_NAME).elf $(OBJ_DIR)/$(PROJECT_NAME).hex $(BIN_OTA) $(OBJ_DIR)/$(PROJECT_NAME).asm size

%.elf %.map: $(SRC_O) $(LDSCRIPT)
	@echo LD: $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

%.hex: %.elf
	@echo OBJCOPY: $@
	@$(OBJCOPY) -O ihex $^ $@

%.bin: %.hex
	@echo Make: $@
	@$(PYTHON) ./phy62x2_ota.py $(ADD_OPT) $(OBJ_DIR)/$(PROJECT_NAME).hex

%.asm: %.elf
	@echo OBJDUMP: $@
	@$(OBJDUMP) -s -S $^ >$@

%.o : %.c
	@echo CC: $<
	@mkdir -p $(OBJ_DIR)/$(dir $@)
	@$(CC) $(CFLAGS) $(INCFLAGS) -c $< -o $(OBJ_DIR)/$@
	@$(CC) -MM $(CFLAGS) $(INCFLAGS) $< -MT $@ -MF $(OBJ_DIR)/$(patsubst %.o,%.d,$@)

%.o : %.s
	@echo CC: $<
	@mkdir -p $(OBJ_DIR)/$(dir $@)
	@$(CC) $(CFLAGS) $(INCFLAGS) -c $< -o $(OBJ_DIR)/$@
	@$(CC) -MM $(CFLAGS) $(INCFLAGS) $< -MT $@ -MF $(OBJ_DIR)/$(patsubst %.o,%.d,$@)

flash: $(OBJ_DIR)/$(PROJECT_NAME).hex
	@$(PYTHON) ./rdwr_phy62x2.py -p$(COM_PORT) -b $(COM_SPEED) --startbaud $(COM_STARTBAUD) -r wh $(OBJ_DIR)/$(PROJECT_NAME).hex

terminal:
	@$(PYTHON) ./miniterm.py $(COM_PORT) $(UART_LOG_BPS) --rts 1 --rtstoggle 100 --exit-char 3 --rtsexit 0

terminal_flash: flash size
	@$(PYTHON) ./miniterm.py $(COM_PORT) $(UART_LOG_BPS) --rts 1 --rtstoggle 100 --exit-char 3 --rtsexit 0

identify:
	@$(PYTHON) ./rdwr_phy62x2.py -p$(COM_PORT) -b $(COM_SPEED) i

erase_and_flash:
	@$(PYTHON) ./rdwr_phy62x2.py -p$(COM_PORT) -b $(COM_SPEED) -e -r wh $(OBJ_DIR)/$(PROJECT_NAME).hex

reset:
	@$(PYTHON) ./rdwr_phy62x2.py -p$(COM_PORT) -r i

dump:
	@$(PYTHON) ./rdwr_phy62x2.py -p$(COM_PORT) -b $(COM_SPEED) --startbaud $(COM_STARTBAUD) rc 0x11000000 0x00080000 ff_phy6222_512k.bin

directory:
	@mkdir -p $(OBJ_DIR)

size: $(OBJ_DIR)/$(PROJECT_NAME).elf
	@echo size:
	@$(SIZE) -t $^
	@$(READELF) -l $^
	@echo

clean:
	@echo clean
	@-rm -rf $(OBJ_DIR)

-include $(DEPENDENCY_LIST)

VPATH:=$(OBJ_DIR) $(SDK_PATH)
