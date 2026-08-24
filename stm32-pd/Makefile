TARGET  := pd-adc-uart
BUILD   := build

CC      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE    := arm-none-eabi-size

SRC     := src/main.c src/startup.c
OBJ     := $(SRC:%.c=$(BUILD)/%.o)

ARCH    := -mcpu=cortex-m4 -mthumb -mfloat-abi=soft
CFLAGS  := $(ARCH) -std=c11 -Os -g3 -Wall -Wextra -ffreestanding \
           -ffunction-sections -fdata-sections -fno-common
LDFLAGS := $(ARCH) -T stm32f407vg.ld -nostdlib -Wl,--gc-sections \
           -Wl,-Map=$(BUILD)/$(TARGET).map

all: $(BUILD)/$(TARGET).bin

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJ)
	$(CC) $(LDFLAGS) $^ -lgcc -o $@
	@echo
	$(SIZE) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

# Cadangkan firmware yang SEKARANG ada di board sebelum ditimpa.
backup:
	@mkdir -p backup
	st-flash read backup/firmware-lama.bin 0x08000000 0x100000

flash: $(BUILD)/$(TARGET).bin
	st-flash --reset write $< 0x08000000

# Kembalikan firmware PD yang lama, persis seperti sebelum ditimpa.
restore:
	st-flash --reset write backup/firmware-lama.bin 0x08000000

erase:
	st-flash erase

clean:
	rm -rf $(BUILD)

.PHONY: all backup flash restore erase clean
