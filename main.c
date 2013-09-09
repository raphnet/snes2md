#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "snes.h"

static void hwinit(void)
{
	/* PORTB
	 *
	 * Bit	Function	Dir		Level/pu
	 * 0	-			out		0
	 * 1	-			out		0
	 * 2	-			out		0
	 * 3	SNES LATCH	out		0
	 * 4	SNES DAT	In		1
	 * 5	SNES CLK	out		1
	 * 6	XTAL1
	 * 7	XTAL2
	 */
	DDRB = 0x2F;
	PORTB = 0x30;

	/* PORTC
	 *
	 * Bit	Function	Dir		Level/pull-up
	 * 0	START/C		In		0
	 * 1	A/B			In		0
	 * 2	0/RT/MODE	In		0
	 * 3	0/LF/X		In		0
	 * 4	DN/DN/Y		In		0
	 * 5	UP/UP/Z		In		0
	 */
	DDRC = 0xFF;
	PORTC = 0xFF;

	/* PORTD
	 *
	 * Bit	Function	Dir		Level/pull-up
	 * 0	-			Out		0  \___ tied together on multiuse PCB2
	 * 1	-			Out		0  /
	 * 2	SELECT		In		1
	 * 3	-			Out		0
	 * 4	VCC			In		0  <---- Shorted to GND on multiuse PCB2
	 * 5	-			Out		0
	 * 6	-			Out		0
	 * 7	-			Out		0
	 */
	DDRD = 0xEB;
	PORTD = 0x04;

}

static volatile uint8_t mddata[8];

void fastint(void) __attribute__((naked)) __attribute__((section(".boot")));

void fastint(void)
{
#if 0
	asm volatile(
		"	nop\nnop\n					\n" // VECTOR 1 : RESET

		"	in r16, 0x23 	; OCR2		\n"
		"	out 0x15, r16	; PORTC		\n"
		
		"	in r16, 0x3F	; SREG		\n"

		"	push r16					\n"
		"	push r30					\n"
		"	push r31					\n"



		"	out 0x1D, r16 ; EEDR		\n" // 1 cycle
		"	sbis 0x10, 2	; PIND2		\n"
		"	in r16, 0x09	; UBRLL		\n"
		"	in r16, 0x1D 	; EEDR		\n"
		"	reti						\n"
	:: "z"(mddata));
#endif


#if 1
asm volatile(
		"	nop\nnop\n					\n" // VECTOR 1 : RESET
	
		"	in __zero_reg__, 0x26 	; ICR1L		\n" 
		"	out 0x15, __zero_reg__	; PORTC		\n"

		// Now, let's prepare for the next transition...	
		
		"	in __zero_reg__, 0x09	; UBRLL		\n"
		"	sbis 0x10, 2	; PIND2		\n"
		"	in __zero_reg__, 0x23 	; OCR2		\n"
		"	out 0x26, __zero_reg__	; ICR1L		\n"
	
		"	clr __zero_reg__			\n"	
		"	reti						\n"
	::);
#endif

#if 0
	asm volatile(
		"	nop\nnop\n					\n" // VECTOR 1 : RESET
		"	out 0x1D, r16 ; EEDR		\n" // 1 cycle
		"	in r16, 0x23 	; OCR2		\n"
		"	sbis 0x10, 2	; PIND2		\n"
		"	in r16, 0x09	; UBRLL		\n"
		"	out 0x15, r16	; PORTC		\n"
		"	in r16, 0x1D 	; EEDR		\n"
		"	reti						\n"
	::);
#endif
}


EMPTY_INTERRUPT(INT0_vect);

struct snes_md_map {
	uint16_t snes_btn;
	uint8_t s0,s1,sx;
};

struct snes_md_map default_map[] = {
	// SNES       		S0    S1    SX
	{ SNES_BTN_A,		0x02, 0x00, 0x00 },
	{ SNES_BTN_B, 		0x00, 0x02, 0x00 },
	{ SNES_BTN_X,		0x00, 0x01, 0x00 },
	{ SNES_BTN_Y,		0x00, 0x00, 0x08 },
	{ SNES_BTN_L,		0x00, 0x00, 0x10 },
	{ SNES_BTN_R,		0x00, 0x00, 0x20 },
	{ SNES_BTN_START,	0x01, 0x00, 0x00 },
	{ SNES_BTN_SELECT,	0x00, 0x00, 0x04 },	
	{ SNES_BTN_DPAD_UP,	0x20, 0x20, 0x00 },
	{ SNES_BTN_DPAD_DOWN,0x10,0x10, 0x00 },
	{ SNES_BTN_DPAD_LEFT,0x00,0x08, 0x00 },
	{ SNES_BTN_DPAD_RIGHT,0x00, 0x04,0x00 },
	{ 0xffff,			0x0c, 0x00, 0x00 }, // always 0 bits
	{ 0, }, /* SNES btns == 0 termination. */
};

int main(void)
{
	Gamepad *snespad;
	gamepad_data last_data;
	uint8_t next_S0_PC, next_S1_PC, next_Sx_PC;

	hwinit();

	snespad = snesGetGamepad();

	// setup SELECT external interrupt
	//
	
	// Move the vector to the bootloader section where we have direct code for
	// INT0.	
	GICR = (1<<IVCE);
	GICR = (1<<IVSEL);

	//
	MCUCR |= (1<<ISC00); // Any change generates an interrupt
	GICR |= (1<<INT0);


	sei();

	while(1)
	{
		struct snes_md_map *map;

		snespad->update();
		snespad->getReport(&last_data);
		
		next_S0_PC = next_S1_PC = next_Sx_PC = 0;

		map = default_map;

		while (map->snes_btn) {
			if (last_data.snes.buttons & map->snes_btn || 
				map->snes_btn==0xffff)
			{
				next_S0_PC |= map->s0;
				next_S1_PC |= map->s1;
				next_Sx_PC |= map->sx;
			}
			map++;
		}

		UBRRL = mddata[0] = next_S0_PC ^ 0xff;
		OCR2 = mddata[1] = next_S1_PC ^ 0xff;

		_delay_ms(4);
	}

	return 0;
}

