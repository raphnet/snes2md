#include <stdint.h>
#include <string.h>
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

volatile uint8_t mddata[8] = { 0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff };
volatile uint8_t dat_pos;
volatile uint8_t polled;

void fastint(void) __attribute__((naked)) __attribute__((section(".boot")));

void fastint(void)
{
asm volatile(
		"	nop\nnop\n					\n" // VECTOR 1 : RESET
	
		"	in __zero_reg__, 0x26 	; ICR1L		\n" 
		"	out 0x15, __zero_reg__	; PORTC		\n"

		// Now, let's prepare for the next transition...	

		"	push r16					\n"
		"	push r30					\n"
		"	push r31					\n"

		"	in r16, __SREG__			\n"
		"	push r16					\n"
		
		// notify main loop
		"	ldi r16, 1					\n"
		"	sts polled, r16				\n"

		// manage pointer
		"	lds r16, dat_pos			\n"	// previous index (now on PORT)
		"	inc r16						\n" // next index
		"	andi r16, 0x07				\n"
		"	sts dat_pos, r16			\n" // save next index

		"	ldi r30, lo8(mddata)		\n" // get mdata base adress into Z
		"	ldi r31, hi8(mddata)		\n"
	
		"	add r30, r16				\n" // Add current index to Z
		"	ldi r16, 0					\n"
		"	adc r31, r16				\n"
		
		"	ld r16, Z					\n"
		"	out 0x26, r16			;	 ICR1L		\n" // next value

		"	clr __zero_reg__			\n" // clear zero reg

		"	pop r16						\n"
		"	out __SREG__, r16			\n"
		
		"	pop r31						\n" // r31
		"	pop r30						\n" // r30
		"	pop r16						\n" // r16
		"	reti						\n"
	::);
}


EMPTY_INTERRUPT(INT0_vect);

struct snes_md_map {
	uint16_t snes_btn;

	uint8_t s[3]; // [0] SELECT LOW STATE, [1] SELECT HIGH STATE, [2] 3RD HIGH STATE
};


#define GEN_BTN_A			{0x02, 0x00, 0x00 }
#define GEN_BTN_B			{0x00, 0x02, 0x00 }
#define GEN_BTN_C			{0x00, 0x01, 0x00 }
#define GEN_BTN_X			{0x00, 0x00, 0x08 }
#define GEN_BTN_Y			{0x00, 0x00, 0x10 }
#define GEN_BTN_Z			{0x00, 0x00, 0x20 }
#define GEN_BTN_START		{0x01, 0x00, 0x00 }
#define GEN_BTN_MODE		{0x00, 0x00, 0x04 }
#define GEN_BTN_DPAD_UP		{0x20, 0x20, 0x00 }
#define GEN_BTN_DPAD_DOWN	{0x10, 0x10, 0x00 }
#define GEN_BTN_DPAD_LEFT	{0x00, 0x08, 0x00 }
#define GEN_BTN_DPAD_RIGHT	{0x00, 0x04, 0x00 }

struct snes_md_map default_map[] = {

	{ SNES_BTN_A,			GEN_BTN_A },
	{ SNES_BTN_B, 			GEN_BTN_B },
	{ SNES_BTN_X,			GEN_BTN_C },
	{ SNES_BTN_Y,			GEN_BTN_X },
	{ SNES_BTN_L,			GEN_BTN_Y },
	{ SNES_BTN_R,			GEN_BTN_Z },
	{ SNES_BTN_START,		GEN_BTN_START },
	{ SNES_BTN_SELECT,		GEN_BTN_MODE },	
	{ SNES_BTN_DPAD_UP,		GEN_BTN_DPAD_UP },
	{ SNES_BTN_DPAD_DOWN,	GEN_BTN_DPAD_DOWN },
	{ SNES_BTN_DPAD_LEFT,	GEN_BTN_DPAD_LEFT },
	{ SNES_BTN_DPAD_RIGHT,	GEN_BTN_DPAD_RIGHT },
	{ 0, }, /* SNES btns == 0 termination. */
};

int main(void)
{
	uint8_t next_data[8];
	Gamepad *snespad;

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
	MCUCR &= ~(1<<ISC01);
	GICR |= (1<<INT0);


	sei();

	while(1)
	{
		struct snes_md_map *map;
		uint8_t sel_low_dat, sel_high_dat, sel_x_dat;
		gamepad_data last_data;
		int i;

		polled = 0;
		while (!polled) { }

		_delay_ms(1.5);
		for (i=0; i<sizeof(next_data); i++) {
			mddata[i] = next_data[i] ^ 0xff;
		}
		if (PIND & (1<<PIND2)) {
			dat_pos = 1;
			PORTC = mddata[0];
		} else {
			dat_pos = 0;
			PORTC = mddata[1];
		}
		ICR1L = mddata[dat_pos];

		snespad->update();
		snespad->getReport(&last_data);

		sel_low_dat = 0;
		sel_high_dat = 0;
		sel_x_dat = 0;
		map = default_map;
		while (map->snes_btn) {
			if ((last_data.snes.buttons & map->snes_btn))
			{
				sel_low_dat |= map->s[0];
				sel_high_dat |= map->s[1];
				sel_x_dat |= map->s[2];
			}
			map++;
		}
	
		next_data[0] = sel_high_dat;	
		next_data[1] = sel_low_dat | 0x0c; // Force right/left to 0 for detection
		next_data[2] = sel_high_dat;
		next_data[3] = sel_low_dat | 0x0c; // Force right/left to 0 for detection
		next_data[4] = sel_high_dat;
		next_data[5] = sel_low_dat | 0x3f; // FORCE UP/Dn/lef/right to 0 for detection.
		next_data[6] = sel_x_dat;
		next_data[7] = sel_high_dat & 0x03; // Keep Up/Dn/Left/Right high for detection.
	}

	return 0;
}

