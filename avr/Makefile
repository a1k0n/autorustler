# for atmega328p, set fuses:
# default settings:
# E:07, H:D9, L:FF
# E: BOD disabled (FIXME)
# H: reset enabled, watchdog disabled, debugwire disabled, do not preserve
#    eeprom on erase, disable bootloader
# L: external osc >8MHz
# 
# change L to 0xF8 for external low-power oscillator

CC = avr-gcc
CFLAGS = -O2 -DF_CPU=20000000 -mmcu=atmega328p

OBJECTS = main.c

all:: main.hex


flash: main.hex
	sudo avrdude -p atmega328p -c linuxspi -P /dev/spidev0.0 -U flash:w:main.hex

.c.o:
	$(CC) $(CFLAGS) -c $< -o $@

main.hex: main.bin
	avr-objcopy -j .text -j .data -O ihex main.bin main.hex
	avr-size main.hex

main.bin: $(OBJECTS)
	$(CC) $(CFLAGS) -o $@ $(OBJECTS) $(LDFLAGS)

disasm: main.bin
	avr-objdump -d main.bin

fuses:
	sudo avrdude -p atmega328p -c linuxspi -P /dev/spidev0.0 \
	  -U hfuse:w:0xD9:m -U lfuse:w:0xFF:m -U efuse:w:0x05:m
