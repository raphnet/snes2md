CC=avr-gcc
AS=$(CC)
LD=$(CC)
CPU=atmega8
CFLAGS=-Wall -mmcu=$(CPU) -DF_CPU=16000000L -Os
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

# BOOTSZ1 and BOOTSZ0 are 0. Therefore:
#  
#  - Application falsh section 0x000 - 0xBFF
#  - Boot loader flash section 0xC00 - 0xFFF

fuse:
	$(AVRDUDE) -Uhfuse:w:0xc9:m -Ulfuse:w:0x9f:m -B 20.0

flash: snes2md.hex
	$(AVRDUDE) -Uflash:w:snes2md.hex -B 1.0

%.o: %.S
	$(CC) $(CFLAGS) -c $<
