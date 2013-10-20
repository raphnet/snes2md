/* Snes controller to Genesis/Megadrive adapter
 * Copyright (C) 2013 Raphël Assénat
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The author may be contacted at raph@raphnet.net
 */
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
	 * 0	JP COM		out		0
	 * 1	JP1			in		1
	 * 2	JP2			in		1
	 * 3	SNES LATCH	out		0
	 * 4	SNES DAT	In		1
	 * 5	SNES CLK	out		1
	 * 6	XTAL1
	 * 7	XTAL2
	 */
	DDRB = 0x29;
	PORTB = 0x36;

	/* PORTC
	 *
	 * Bit	Function	Dir		Level/pull-up
	 * 0	START/C		In		1
	 * 1	A/B			In		1
	 * 2	0/RT/MODE	In		1
	 * 3	0/LF/X		In		1
	 * 4	DN/DN/Y		In		1
	 * 5	UP/UP/Z		In		1
	 */
	DDRC = 0x00;
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

volatile uint8_t mddata[8] = { 0xff,0xf3,0xff,0xf3,0xff,0xc3,0xff,0xff };
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
		"	andi r16, 0x07				\n" // 0-7
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

#define ATARI_BTN_DPAD_UP		{0x20, 0x20, 0x20 }
#define ATARI_BTN_DPAD_DOWN		{0x10, 0x10, 0x10 }
#define ATARI_BTN_DPAD_LEFT		{0x08, 0x08, 0x08 }
#define ATARI_BTN_DPAD_RIGHT	{0x04, 0x04, 0x04 }
#define ATARI_BTN_FIRE			{0x02, 0x02, 0x02 }

struct snes_md_map md_default_map[] = {

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

struct snes_md_map md_alt_map1[] = {

	{ SNES_BTN_A,			GEN_BTN_B },
	{ SNES_BTN_B, 			GEN_BTN_A },
	{ SNES_BTN_X,			GEN_BTN_X },
	{ SNES_BTN_Y,			GEN_BTN_C },
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

struct snes_md_map md_alt_map2[] = {

	{ SNES_BTN_A,			GEN_BTN_C },
	{ SNES_BTN_B, 			GEN_BTN_B },
	{ SNES_BTN_X,			GEN_BTN_Y },
	{ SNES_BTN_Y,			GEN_BTN_A },
	{ SNES_BTN_L,			GEN_BTN_X },
	{ SNES_BTN_R,			GEN_BTN_Z },
	{ SNES_BTN_START,		GEN_BTN_START },
	{ SNES_BTN_SELECT,		GEN_BTN_MODE },	
	{ SNES_BTN_DPAD_UP,		GEN_BTN_DPAD_UP },
	{ SNES_BTN_DPAD_DOWN,	GEN_BTN_DPAD_DOWN },
	{ SNES_BTN_DPAD_LEFT,	GEN_BTN_DPAD_LEFT },
	{ SNES_BTN_DPAD_RIGHT,	GEN_BTN_DPAD_RIGHT },
	{ 0, }, /* SNES btns == 0 termination. */
};

struct snes_md_map atari_default_map[] = {
	{ SNES_BTN_DPAD_UP,		ATARI_BTN_DPAD_UP },
	{ SNES_BTN_DPAD_DOWN,	ATARI_BTN_DPAD_DOWN },
	{ SNES_BTN_DPAD_LEFT,	ATARI_BTN_DPAD_LEFT },
	{ SNES_BTN_DPAD_RIGHT,	ATARI_BTN_DPAD_RIGHT },
	{ SNES_BTN_A,			ATARI_BTN_FIRE },
	{ SNES_BTN_B,			ATARI_BTN_FIRE },
	{ SNES_BTN_X,			ATARI_BTN_FIRE },
	{ SNES_BTN_Y,			ATARI_BTN_FIRE },
	{ 0, }, /* SNES btns == 0 termination. */
};

struct snes_md_map atari_alt_map1[] = {
	{ SNES_BTN_DPAD_DOWN,	ATARI_BTN_DPAD_DOWN },
	{ SNES_BTN_DPAD_LEFT,	ATARI_BTN_DPAD_LEFT },
	{ SNES_BTN_DPAD_RIGHT,	ATARI_BTN_DPAD_RIGHT },

	{ SNES_BTN_A,			ATARI_BTN_FIRE },
	{ SNES_BTN_B,			ATARI_BTN_DPAD_UP },
	{ 0, }, /* SNES btns == 0 termination. */
};

struct snes_md_map atari_alt_map2[] = {
	{ SNES_BTN_DPAD_DOWN,	ATARI_BTN_DPAD_DOWN },
	{ SNES_BTN_DPAD_LEFT,	ATARI_BTN_DPAD_LEFT },
	{ SNES_BTN_DPAD_RIGHT,	ATARI_BTN_DPAD_RIGHT },

	{ SNES_BTN_B,			ATARI_BTN_FIRE },
	{ SNES_BTN_A,			ATARI_BTN_DPAD_UP },
	{ 0, }, /* SNES btns == 0 termination. */
};


#define MD_MAP_DEFAULT			0
#define MD_MAP_ALT1				1
#define MD_MAP_ALT2				2
#define ATARI_MAP_DEFAULT		3
#define ATARI_MAP_ALT1			4
#define ATARI_MAP_ALT2			5
struct snes_md_map *maps[6] = {
	md_default_map,
	md_alt_map1,
	md_alt_map2,
	atari_default_map,
	atari_alt_map1,
	atari_alt_map2,
};

int main(void)
{
	gamepad_data last_data;
	uint8_t next_data[8];
	Gamepad *snespad;
	uint8_t cur_map_id;
	char atari_mode;

	hwinit();

	if (PIND & (1<<PIND2)) {
		dat_pos = 1;
		PORTC = mddata[0];
	} else {
		dat_pos = 0;
	}

	dat_pos = 1;

	_delay_ms(50);

	/* If PB1 and/or PB2 are shorted to GND (or PB0 which is 
	 * configured as an output), run in Atari-style mode. */
	switch (PINB & 0x06) {
		case 0x06:
			atari_mode = 0;
			break;
		default:
			atari_mode = 1;				
	}

	snespad = snesGetGamepad();
	snespad->update();
	snespad->getReport(&last_data);

	if (!atari_mode) {
		// Push-pull drive for Genesis
		DDRC = 0xFF;
		PORTC = 0xFf;

		switch (last_data.snes.buttons)
		{
			default:
			case SNES_BTN_A:
				cur_map_id = MD_MAP_DEFAULT;	
				break;
			case SNES_BTN_B:
				cur_map_id = MD_MAP_ALT1;	
				break;
			case SNES_BTN_Y:
				cur_map_id = MD_MAP_ALT2;	
				break;
		}
	} else {
		// Simulated open-collector/switch drive for Atari
		DDRC = 0x00;
		PORTC = 0x00;
		switch (last_data.snes.buttons)
		{
			default:
				cur_map_id = ATARI_MAP_DEFAULT;	
				break;
			case SNES_BTN_B:
				cur_map_id = ATARI_MAP_ALT1;	
				break;
			case SNES_BTN_A:
				cur_map_id = ATARI_MAP_ALT2;
				break;
		}
	}

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

	if (!atari_mode) {
		// Interrupts are only used by the Genesis mode
		sei();
	}

	while(1)
	{
		struct snes_md_map *map;
		uint8_t sel_low_dat, sel_high_dat, sel_x_dat;
		int i;

		if (!atari_mode)
		{
			// Genesis mode
			polled = 0;
			while (!polled) { }
			_delay_ms(1.5);

			// Timeout from the 6button mode
			memcpy((void*)mddata, next_data, 8);
			if (PIND & (1<<PIND2)) {
				dat_pos = 1;
				PORTC = mddata[0];
			} else {
				dat_pos = 0;
			}

			ICR1L = mddata[dat_pos];
		}
		else {
			// Atari mode
			PORTC = 0x00;
			DDRC = next_data[0] ^ 0xff;
			_delay_ms(15);
		}
		
		snespad->update();
		snespad->getReport(&last_data);

		sel_low_dat = 0;
		sel_high_dat = 0;
		sel_x_dat = 0;

		map = maps[cur_map_id];
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
		next_data[5] = sel_low_dat | 0x3c; // FORCE UP/Dn/lef/right to 0 for detection.
		next_data[6] = sel_x_dat;
		next_data[7] = sel_high_dat & 0x03; // Keep Up/Dn/Left/Right high for detection.

		for (i=0; i<8; i++) {
			next_data[i] ^= 0xff;
		}
	}

	return 0;
}
