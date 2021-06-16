
TARGET=blinky

CC = $(TC32_HOME)/bin/tc32-elf-gcc
LD = $(TC32_HOME)/bin/tc32-elf-ld
CP = $(TC32_HOME)/bin/tc32-elf-objcopy

CCFLAGS = -Wall -std=gnu99 -DMCU_STARTUP_5316 -I $(SDK)/ -ffunction-sections -fdata-sections

LDFLAGS = --gc-sections -T $(SDK)/boot.link

LIB = $(SDK)/proj_lib/liblt_5316.a

BUILD_DIR = _build

DRIVERS_SRC = \
	$(SDK)/drivers/5316/gpio.c \
	$(SDK)/drivers/5316/analog.c \
	$(SDK)/drivers/5316/clock.c \
	$(SDK)/drivers/5316/bsp.c \
	$(SDK)/drivers/5316/adc.c
DRIVERS_OBJS = $(addprefix $(BUILD_DIR)/drivers/, $(notdir $(DRIVERS_SRC:%.c=%.o)))

STARTUP_SRC = $(SDK)/boot/5316/cstartup_5316.S
STARTUP_OBJ = $(addprefix $(BUILD_DIR)/asm/, $(notdir $(STARTUP_SRC:%.S=%.o)))

SRC = main.c
OBJS = $(addprefix $(BUILD_DIR)/, $(SRC:%.c=%.o))

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET)
	$(CP) -O binary $< $@

$(BUILD_DIR)/$(TARGET): $(OBJ) $(DRIVERS_OBJS) $(STARTUP_OBJ) $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS) $(DRIVERS_OBJS) $(STARTUP_OBJ) $(LIB)

# Drivers.
$(BUILD_DIR)/drivers/%.o: $(SDK)/drivers/5316/%.c
	mkdir -p $(BUILD_DIR)/drivers
	$(CC) -c $(CCFLAGS) -o $@ $<

# Startup.
$(BUILD_DIR)/asm/cstartup_5316.o : $(STARTUP_SRC)
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

# Manually:
# tc32-elf-gcc -DMCU_STARTUP_5316 -I $SDK/ -c -ffunction-sections -fdata-sections -std=gnu99 $SDK/boot/5316/cstartup_5316.S main.c $SDK/drivers/5316/gpio.c $SDK/drivers/5316/analog.c $SDK/drivers/5316/clock.c $SDK/drivers/5316/bsp.c $SDK/div_mod.S $SDK/drivers/5316/adc.c
# tc32-elf-ld --gc-sections -L $SDK/proj_li -T $SDK/boot.link -o prog main.o cstartup_5316.o gpio.o analog.o clock.o $SDK/proj_lib/liblt_5316.a bsp.o div_mod.o adc.o