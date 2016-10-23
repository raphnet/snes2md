CC=avr-gcc
AS=$(CC)
LD=$(CC)
CPU=atmega8
CFLAGS=-Wall -mmcu=$(CPU) -DF_CPU=16000000L -Os -g -Werror
LDFLAGS=-mmcu=$(CPU) -Wl,-Map=snes2md.map -Wl,--section-start=.boot=0x1800
AVRDUDE=avrdude -p m8 -P usb -c avrispmkII

OBJS=main.o snes.o

all: snes2md.hex

clean:
	rm -f snes2md.elf snes2md.hex snes2md.map $(OBJS)

snes2md.elf: $(OBJS)
	$(LD) $(OBJS) $(LDFLAGS) -o snes2md.elf

snes2md.hex: snes2md.elf
	avr-objcopy -j .data -j .text -j .boot -O ihex snes2md.elf snes2md.hex
	avr-size snes2md.elf

LFUSE=0x9F
HFUSE=0xC9

fuse:
	$(AVRDUDE) -Uhfuse:w:$(HFUSE):m -Ulfuse:w:$(LFUSE):m -B 20.0

flash: snes2md.hex
	$(AVRDUDE) -Uflash:w:snes2md.hex -B 1.0

erase:
	$(AVRDUDE) -e -B 20.0

reset:
	$(AVRDUDE) -B 1.0

%.o: %.S
	$(CC) $(CFLAGS) -c $<
