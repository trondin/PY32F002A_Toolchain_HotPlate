
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

CFLAGS = -mcpu=cortex-m0 -mthumb -Os \
	-ffunction-sections -fdata-sections \
	-I./lib/Puya_PY32F0xx_DFP/Drivers/CMSIS/Device/PY32F0xx/Include \
	-I./lib/Puya_PY32F0xx_DFP/CMSIS/Core/Include \
	-DPY32F030x6

LDFLAGS = -T src/py32f030.ld -Wl,-Map=main.map -Wl,--gc-sections -specs=nano.specs

# List of all source files
SOURCES = \
	src/main.c \
	src/startup.S \
	./lib/Puya_PY32F0xx_DFP/Drivers/CMSIS/Device/PY32F0xx/Source/system_py32f0xx.c \
	src/syscalls.c \
	src/uart.c \
	src/lcd.c \
	src/scr_power.c \
	src/ntc_lut.c \
	src/eeprom_emul.c \
	src/pid.c \
	src/systick.c

# Generate list of object files
OBJECTS = $(SOURCES:.c=.o)
OBJECTS := $(OBJECTS:.S=.o)

all: main.elf main.hex main.bin

main.elf: $(OBJECTS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^
	$(SIZE) $@  # Display program size and memory usage

main.hex: main.elf
	$(OBJCOPY) -O ihex $< $@

main.bin: main.elf
	$(OBJCOPY) -O binary $< $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.S
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) *.o *.elf *.map
	
