CC=avr-gcc
AS=$(CC)
LD=$(CC)
CPU=atmega168
CFLAGS=-Wall -mmcu=$(CPU) -DF_CPU=16000000L -Os -g -Werror
LDFLAGS=-mmcu=$(CPU) -Wl,-Map=snes2md.m168.map -Wl,--section-start=.boot=0x1800 
AVRDUDE=avrdude -p m8 -P usb -c avrispmkII

OBJS=main.o snes.o

all: snes2md.m168.hex

clean:
	rm -f snes2md.m168.elf snes2md.m168.hex snes2md.m168.map $(OBJS)

snes2md.m168.elf: $(OBJS)
	$(LD) $(OBJS) $(LDFLAGS) -o snes2md.m168.elf

snes2md.m168.hex: snes2md.m168.elf
	avr-objcopy -j .data -j .text -j .boot -O ihex snes2md.m168.elf snes2md.m168.hex
	avr-size snes2md.m168.elf

EFUSE=0x01
LFUSE=0xD7
HFUSE=0xDC

fuse:
	$(AVRDUDE) -Uhfuse:w:$(HFUSE):m -Ulfuse:w:$(LFUSE):m -Uefuse:w:$(EFUSE):m -B 20.0

flash: snes2md.m168.hex
	$(AVRDUDE) -Uflash:w:snes2md.m168.hex -B 1.0

erase:
	$(AVRDUDE) -e -B 20.0

reset:
	$(AVRDUDE) -B 1.0

%.o: %.S
	$(CC) $(CFLAGS) -c $<
