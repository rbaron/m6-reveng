
TARGET=ble-b-parasite-tracker

CC = $(TC32_HOME)/bin/tc32-elf-gcc
LD = $(TC32_HOME)/bin/tc32-elf-ld
CP = $(TC32_HOME)/bin/tc32-elf-objcopy

CCFLAGS = -Wall -std=gnu99 -DMCU_STARTUP_5316 -D__TL_LIB_5316__ -I $(SDK)/ -ffunction-sections -fdata-sections

LDFLAGS = --gc-sections -T $(SDK)/boot.link

LIB = $(SDK)/proj_lib/liblt_5316.a

BUILD_DIR = _build

COMMON_SRC = \
	../common/uart.c \
	../common/tft.c
COMMON_OBJS = $(addprefix $(BUILD_DIR)/common/, $(notdir $(COMMON_SRC:%.c=%.o)))

DRIVERS_SRC = \
	$(SDK)/drivers/5316/gpio.c \
	$(SDK)/drivers/5316/analog.c \
	$(SDK)/drivers/5316/clock.c \
	$(SDK)/drivers/5316/bsp.c \
	$(SDK)/drivers/5316/flash.c \
	$(SDK)/drivers/5316/spi.c \
	$(SDK)/drivers/5316/timer.c \
	$(SDK)/drivers/5316/uart.c \
	$(SDK)/drivers/5316/adc.c
DRIVERS_OBJS = $(addprefix $(BUILD_DIR)/drivers/, $(notdir $(DRIVERS_SRC:%.c=%.o)))

SDK_COMMON_SRC = \
	$(SDK)/common/string.c \
	$(SDK)/common/utility.c \
	$(SDK)/common/printf.c
SDK_COMMON_OBJS = $(addprefix $(BUILD_DIR)/sdk-common/, $(notdir $(SDK_COMMON_SRC:%.c=%.o)))

VENDORS_SRC = \
	$(SDK)/vendor/common/blt_common.c
VENDORS_OBJS = $(addprefix $(BUILD_DIR)/vendor/common/, $(notdir $(VENDORS_SRC:%.c=%.o)))

STARTUP_SRC = $(SDK)/boot/5316/cstartup_5316.S
STARTUP_OBJ = $(addprefix $(BUILD_DIR)/asm/, $(notdir $(STARTUP_SRC:%.S=%.o)))

DIVMOD_SRC = $(SDK)/div_mod.S
DIVMOD_OBJ = $(addprefix $(BUILD_DIR)/asm/, $(notdir $(DIVMOD_SRC:%.S=%.o)))

SRC = \
	main.c \
	app.c \
	app_att.c
OBJS = $(addprefix $(BUILD_DIR)/, $(SRC:%.c=%.o))

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET)
	$(CP) -O binary $< $@

$(BUILD_DIR)/$(TARGET): $(OBJS) $(DRIVERS_OBJS) $(COMMON_OBJS) $(VENDORS_OBJS) $(SDK_COMMON_OBJS) $(STARTUP_OBJ) $(DIVMOD_OBJ) $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS) $(DRIVERS_OBJS) $(COMMON_OBJS) $(VENDORS_OBJS) $(SDK_COMMON_OBJS) $(STARTUP_OBJ) $(DIVMOD_OBJ) $(LIB)

# Drivers.
$(BUILD_DIR)/drivers/%.o: $(SDK)/drivers/5316/%.c
	mkdir -p $(BUILD_DIR)/drivers
	$(CC) -c $(CCFLAGS) -o $@ $<

# Common.
$(BUILD_DIR)/common/%.o: ../common/%.c
	mkdir -p $(BUILD_DIR)/common
	$(CC) -c $(CCFLAGS) -o $@ $<

# SDK common stuff.
$(BUILD_DIR)/sdk-common/%.o: $(SDK)/common/%.c
	mkdir -p $(BUILD_DIR)/sdk-common
	$(CC) -c $(CCFLAGS) -o $@ $<

# Vendor stuff.
$(BUILD_DIR)/vendor/common/%.o: $(SDK)/vendor/common/%.c
	mkdir -p $(BUILD_DIR)/vendor/common
	$(CC) -c $(CCFLAGS) -o $@ $<

# Startup.
$(BUILD_DIR)/asm/cstartup_5316.o : $(STARTUP_SRC)
	mkdir -p $(BUILD_DIR)/asm
	$(CC) -c $(CCFLAGS) $< -o $@

# div_mod.
$(BUILD_DIR)/asm/div_mod.o : $(DIVMOD_SRC)
	mkdir -p $(BUILD_DIR)/asm
	$(CC) -c $(CCFLAGS) $< -o $@

# App sources.
$(BUILD_DIR)/%.o: %.c
	mkdir -p $(BUILD_DIR)
	$(CC) -c $(CCFLAGS) -o $@ $<

.PHONE: all
all: $(DRIVERS_OBJS) $(STARTUP_OBJ)
	@echo $(DRIVERS_OBJ)

.PHONY: clean
clean:
	rm -rf $(BUILD_DIR)
