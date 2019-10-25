/* Snes controller to Genesis/Megadrive adapter
 * Copyright (C) 2013-2016 Raphël Assénat
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
	 * 2	SELECT		In		1* see note
	 * 3	-			Out		0
	 * 4	VCC			In		0  <---- Shorted to GND on multiuse PCB2
	 * 5	-			Out		0
	 * 6	-			Out		0
	 * 7	-			Out		0
	 */
	DDRD = 0xEB;
//	PORTD = 0x04; // see note
	PORTD = 0x00;

	// Note: Games like GoldenAxe II let the SELECT line float for a moment. With
	// the internal pull-up, the signal rises slowly and sometimes is seen as a
	// logic 1, but just barely. Depending on which buttons are pressed on the
	// controller, the ramping up of the SELECT signal is not always smooth, and
	// this is seen as a series of transition by the adapter which updates the
	// buttons.
	//
	//
	//       No buttons pressed            Buttons pressed
	// 1       ___        _             ___          _
	//        |   |      | |           |   |    , , | |
	//        |   |   /| | |           |   |   /|/| | |
	// 0 _____|   |__/ |_| |____   ____|   |__/   |_| |___
	//
	//              ^^^^                      ^^^^
	//
	// Disabling the internal pullup seems to help, but ideally an external pull-down
	// resistor should be added. Goldenaxe II is known to misbehave with 6 button
	// controllers. This is solved by forcing 3 button mode (by holding MODE at power
	// up).
	//
	// Interestingly, the adapter does not need to be in compatibility mode for this
	// game. Maybe the behaviour shown above is what cause problems with real 6
	// button controllers?
}

volatile uint8_t mddata[8] = { 0xff,0xf3,0xff,0xf3,0xff,0xc3,0xff,0xff };
volatile uint8_t dat_pos;
volatile uint8_t polled;

void fastint(void) __attribute__((naked)) __attribute__((section(".boot")));

void fastint(void)
{
asm volatile(
		"	nop\nnop\n					\n" // VECTOR 1 : RESET

#if defined(__AVR_ATmega168__)
		// PORTC address is different.
		// And we can only use the first 64 registers
		// with in/out instructions. So ICR1L was not available.
		"	in __zero_reg__, 0x27 	; OCR0A		\n"
		"	out 0x08, __zero_reg__	; PORTC		\n"
#elif defined(__AVR_ATmega8__)
		"	in __zero_reg__, 0x26 	; ICR1L		\n"
		"	out 0x15, __zero_reg__	; PORTC		\n"
#else
#error MCU not supported yet
#endif

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

#if defined(__AVR_ATmega168__)
		"	out 0x27, r16			;	 OCR0A		\n" // next value
#elif defined(__AVR_ATmega8__)
		"	out 0x26, r16			;	 ICR1L		\n" // next value
#else
#error MCU not supported yet
#endif

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

#define OPT_TURBO			1	/* This button is turbo */
#define OPT_TURBO_SEL_SPEED	2	/* This button cycles through turbo speeds */

struct snes_md_map {
	uint16_t snes_btn;
	uint8_t s[3]; // [0] SELECT LOW STATE, [1] SELECT HIGH STATE, [2] 3RD HIGH STATE
	char opts;
};

#define DB9_NULL			{0x00, 0x00, 0x00 }

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
#define ATARI_BTN_FIRE2			{0x01, 0x01, 0x01 }


struct snes_md_map md_snes1[] = {

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

struct snes_md_map md_snes2[] = {

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

struct snes_md_map md_snes3[] = {

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

struct snes_md_map md_snes4[] = {
	{ SNES_BTN_X,			GEN_BTN_A },
	{ SNES_BTN_Y, 			GEN_BTN_B },
	{ SNES_BTN_B,			GEN_BTN_C },
	{ SNES_BTN_A,			GEN_BTN_X },
	{ SNES_BTN_R,			GEN_BTN_Y },
	{ SNES_BTN_L,			GEN_BTN_Z },
	{ SNES_BTN_START,		GEN_BTN_START },
	{ SNES_BTN_SELECT,		GEN_BTN_MODE },
	{ SNES_BTN_DPAD_UP,		GEN_BTN_DPAD_UP },
	{ SNES_BTN_DPAD_DOWN,	GEN_BTN_DPAD_DOWN },
	{ SNES_BTN_DPAD_LEFT,	GEN_BTN_DPAD_LEFT },
	{ SNES_BTN_DPAD_RIGHT,	GEN_BTN_DPAD_RIGHT },
	{ 0, }, /* SNES btns == 0 termination. */
};

struct snes_md_map md_snes5[] = {
	{ SNES_BTN_A,			GEN_BTN_A },
	{ SNES_BTN_B, 			GEN_BTN_B },
	{ SNES_BTN_X,			GEN_BTN_C },
	{ SNES_BTN_Y,			GEN_BTN_X },
	{ SNES_BTN_L,			GEN_BTN_Y },
	{ SNES_BTN_R,			GEN_BTN_Z },
	{ SNES_BTN_START,		GEN_BTN_START },
	{ SNES_BTN_SELECT,		GEN_BTN_MODE },
	{ SNES_BTN_DPAD_DOWN,	GEN_BTN_DPAD_UP },
	{ SNES_BTN_DPAD_UP,		GEN_BTN_DPAD_DOWN },
	{ SNES_BTN_DPAD_RIGHT,	GEN_BTN_DPAD_LEFT },
	{ SNES_BTN_DPAD_LEFT,	GEN_BTN_DPAD_RIGHT },
	{ 0, }, /* SNES btns == 0 termination. */
};

struct snes_md_map md_snes6[] = {
	{ SNES_BTN_A,			GEN_BTN_B },
	{ SNES_BTN_B, 			GEN_BTN_A },
	{ SNES_BTN_X,			GEN_BTN_Y },
	{ SNES_BTN_Y,			GEN_BTN_X },
	{ SNES_BTN_L,			GEN_BTN_Z },
	{ SNES_BTN_R,			GEN_BTN_C },
	{ SNES_BTN_START,		GEN_BTN_START },
	{ SNES_BTN_SELECT,		GEN_BTN_MODE },
	{ SNES_BTN_DPAD_UP,		GEN_BTN_DPAD_UP },
	{ SNES_BTN_DPAD_DOWN,	GEN_BTN_DPAD_DOWN },
	{ SNES_BTN_DPAD_LEFT,	GEN_BTN_DPAD_LEFT },
	{ SNES_BTN_DPAD_RIGHT,	GEN_BTN_DPAD_RIGHT },
	{ 0, }, /* SNES btns == 0 termination. */
};


struct snes_md_map atari_style1_map[] = {
	{ SNES_BTN_DPAD_UP,		ATARI_BTN_DPAD_UP },
	{ SNES_BTN_DPAD_DOWN,	ATARI_BTN_DPAD_DOWN },
	{ SNES_BTN_DPAD_LEFT,	ATARI_BTN_DPAD_LEFT },
	{ SNES_BTN_DPAD_RIGHT,	ATARI_BTN_DPAD_RIGHT },
	{ SNES_BTN_A | SNES_BTN_B,		ATARI_BTN_FIRE },

	{ SNES_BTN_Y | SNES_BTN_R,		ATARI_BTN_FIRE,	OPT_TURBO },
	{ SNES_BTN_X | SNES_BTN_L,		ATARI_BTN_FIRE2, },
	{ SNES_BTN_SELECT,		DB9_NULL,	OPT_TURBO_SEL_SPEED },
	{ 0, }, /* SNES btns == 0 termination. */
};

struct snes_md_map atari_style2_map[] = {
	{ SNES_BTN_DPAD_DOWN,	ATARI_BTN_DPAD_DOWN },
	{ SNES_BTN_DPAD_LEFT,	ATARI_BTN_DPAD_LEFT },
	{ SNES_BTN_DPAD_RIGHT,	ATARI_BTN_DPAD_RIGHT },
	{ SNES_BTN_B,			ATARI_BTN_DPAD_UP },
	{ SNES_BTN_A,			ATARI_BTN_FIRE },

	{ SNES_BTN_Y | SNES_BTN_R,			ATARI_BTN_FIRE,	OPT_TURBO },
	{ SNES_BTN_X | SNES_BTN_L,			ATARI_BTN_FIRE2, },
	{ SNES_BTN_SELECT,		DB9_NULL,	OPT_TURBO_SEL_SPEED },
	{ 0, }, /* SNES btns == 0 termination. */
};

struct snes_md_map atari_style3_map[] = {
	{ SNES_BTN_DPAD_DOWN,	ATARI_BTN_DPAD_DOWN },
	{ SNES_BTN_DPAD_LEFT,	ATARI_BTN_DPAD_LEFT },
	{ SNES_BTN_DPAD_RIGHT,	ATARI_BTN_DPAD_RIGHT },
	{ SNES_BTN_A,			ATARI_BTN_DPAD_UP },
	{ SNES_BTN_B,			ATARI_BTN_FIRE },

	{ SNES_BTN_X | SNES_BTN_L,			ATARI_BTN_FIRE2	 },
	{ SNES_BTN_Y | SNES_BTN_R,			ATARI_BTN_FIRE,	OPT_TURBO },
	{ SNES_BTN_SELECT,		DB9_NULL,	OPT_TURBO_SEL_SPEED },
	{ 0, }, /* SNES btns == 0 termination. */
};

struct snes_md_map atari_style4_map[] = {
	{ SNES_BTN_DPAD_UP,		ATARI_BTN_DPAD_DOWN },
	{ SNES_BTN_DPAD_DOWN,	ATARI_BTN_DPAD_UP },
	{ SNES_BTN_DPAD_RIGHT,	ATARI_BTN_DPAD_LEFT },
	{ SNES_BTN_DPAD_LEFT,	ATARI_BTN_DPAD_RIGHT },
	{ SNES_BTN_A,			ATARI_BTN_DPAD_UP },
	{ SNES_BTN_B,			ATARI_BTN_FIRE },

	{ SNES_BTN_X | SNES_BTN_L,	ATARI_BTN_FIRE2, },
	{ SNES_BTN_Y | SNES_BTN_R,	ATARI_BTN_FIRE,	OPT_TURBO },
	{ SNES_BTN_SELECT,		DB9_NULL,	OPT_TURBO_SEL_SPEED },
	{ 0, }, /* SNES btns == 0 termination. */
};



#define MD_MAP_SNES1			0
#define MD_MAP_SNES2			1
#define MD_MAP_SNES3			2
#define MD_MAP_SNES4			3
#define MD_MAP_SNES5			4
#define ATARI_MAP_STYLE1		5
#define ATARI_MAP_STYLE2		6
#define ATARI_MAP_STYLE3		7
#define ATARI_MAP_STYLE4		8
#define MD_MAP_SNES6			9
struct snes_md_map *maps[10] = {
	md_snes1,
	md_snes2,
	md_snes3,
	md_snes4,
	md_snes5,
	atari_style1_map,
	atari_style2_map,
	atari_style3_map,
	atari_style4_map,
	md_snes6,
};

#define TURBO_SPEED_30		0 // 60 / 2
#define TURBO_SPEED_25		1 // 50 / 2
#define TURBO_SPEED_20		2 // 60 / 3
#define TURBO_SPEED_16		3 // 50 / 3
#define TURBO_SPEED_15		4 // 60 / 4
#define TURBO_SPEED_12_5	5 // 50 / 4

static uint8_t turbo_speed = TURBO_SPEED_25;

static void turboCycleSpeed(uint8_t btn_state)
{
	static uint8_t last_state;

	if (btn_state && !last_state) {
		turbo_speed++;
		if (turbo_speed > TURBO_SPEED_12_5) {
			turbo_speed = TURBO_SPEED_30;
		}
	}

	last_state = btn_state;
}

static uint8_t turbo_lock_on = 0;

static void turboToggleLockOn(uint8_t btn_state)
{
	static uint8_t last_state;

	if (btn_state && !last_state) {
		turbo_lock_on = !turbo_lock_on;
	}

	last_state = btn_state;
}

static uint8_t turboGetTop(void)
{
	switch (turbo_speed)
	{
			// divide by 2
		default:
		case TURBO_SPEED_30:
		case TURBO_SPEED_25:
			return 2;

			// divide by 3
		case TURBO_SPEED_20:
		case TURBO_SPEED_16:
			return 3;

			// divide by 4
		case TURBO_SPEED_15:
		case TURBO_SPEED_12_5:
			return 4;
	}
}

static void turboSleep(void)
{
	switch (turbo_speed)
	{
			// 120Hz base
		default:
		case TURBO_SPEED_30:
		case TURBO_SPEED_20:
		case TURBO_SPEED_15:
//			_delay_us(8333);
			// This delay is trimmed to account for overhead and obtain 29.97Hz.
			_delay_us(8208);
			break;

			// 100Hz base
		case TURBO_SPEED_25:
		case TURBO_SPEED_16:
		case TURBO_SPEED_12_5:
			//_delay_us(10000); // 24.66
			_delay_us(9860); // 25 Hz
			break;
	}
}

static char turboPoll(void)
{
	static uint8_t c=0;

	c = !c;

	return c;
}

static uint8_t turbo_state=0;

static void turboDo(void)
{
	static char turbo_count = 0;

	if (turbo_count <= 0) {
		turbo_count = turboGetTop();
		turbo_state = !turbo_state;
	}
	turbo_count--;
}

int main(void)
{
	gamepad_data last_data;
	uint8_t next_data[8];
	Gamepad *snespad;
	uint8_t cur_map_id;
	char atari_mode;
	char ignore_buttons = 1;
	char tribtn_compat = 0;
	char genesis_polling = 0;

	hwinit();

	if (PIND & (1<<PIND2)) {
		dat_pos = 1;
		PORTC = mddata[0];
	} else {
		dat_pos = 0;
	}

	dat_pos = 1;

	_delay_ms(20);

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

		switch (last_data.snes.buttons & (~SNES_BTN_SELECT))
		{
			default:
			case SNES_BTN_A:
				cur_map_id = MD_MAP_SNES1;
				break;
			case SNES_BTN_B:
				cur_map_id = MD_MAP_SNES2;
				break;
			case SNES_BTN_Y:
				cur_map_id = MD_MAP_SNES3;
				break;
			case SNES_BTN_X:
				cur_map_id = MD_MAP_SNES4;
				break;
			case SNES_BTN_L:
				cur_map_id = MD_MAP_SNES5;
				break;
			case SNES_BTN_R:
				cur_map_id = MD_MAP_SNES6;
				break;
		}

		// If select is down, enable 3 button compatibility mode
		if (last_data.snes.buttons & SNES_BTN_SELECT) {
			tribtn_compat = 1;
		}

	} else {
		// Simulated open-collector/switch drive for Atari
		DDRC = 0x00;
		PORTC = 0x00;
		switch (last_data.snes.buttons)
		{
			default:
				cur_map_id = ATARI_MAP_STYLE1;
				break;
			case SNES_BTN_B:
				cur_map_id = ATARI_MAP_STYLE2;
				break;
			case SNES_BTN_A:
				cur_map_id = ATARI_MAP_STYLE3;
				break;
			case SNES_BTN_X:
				cur_map_id = ATARI_MAP_STYLE4;
		}
	}

	// setup SELECT external interrupt
	//
#if defined(__AVR_ATmega8__)
	// Move the vector to the bootloader section where we have direct code for
	// INT0.
	GICR = (1<<IVCE);
	GICR = (1<<IVSEL);

	//
	MCUCR |= (1<<ISC00); // Any change generates an interrupt
	MCUCR &= ~(1<<ISC01);
	GICR |= (1<<INT0);
#elif defined(__AVR_ATmega168__)
	/* Move the vector to the bootloader section where we have direct code for INT0. */
	MCUCR = (1<<IVCE);
	MCUCR = (1<<IVSEL);

	/* Any changes triggers INT0 */
	EICRA = (1<<ISC00);
	EIMSK = (1<<INT0);
#else
#error MCU not supported
#endif

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
			if (!genesis_polling) {
				_delay_ms(15);
				snespad->update();
				snespad->getReport(&last_data);

				memcpy((void*)mddata, next_data, 8);
				if (PIND & (1<<PIND2)) {
					dat_pos = 1;
					PORTC = mddata[0];
				} else {
					PORTC = mddata[1];
					dat_pos = 0;
				}

#if defined(__AVR_ATmega168__)
				OCR0A = mddata[dat_pos];
#elif defined(__AVR_ATmega8__)
				ICR1L = mddata[dat_pos];
#else
#error MCU not supported yet
#endif
				if (polled) {
					genesis_polling = 1;
				}
			}
			else {
				char c = 0;
				// Genesis mode
				polled = 0;
				while (!polled) {
					c++;
					_delay_ms(1);
					if (c > 100) {
						// After 100ms, fall back to self-timed mode (for SMS games)
						genesis_polling = 0;
						break;
					}
				}
				_delay_ms(1.5);

				// Timeout from the 6button mode
				memcpy((void*)mddata, next_data, 8);
				if (PIND & (1<<PIND2)) {
					dat_pos = 1;
					PORTC = mddata[0];
				} else {
					PORTC = mddata[1];
					dat_pos = 0;
				}

#if defined(__AVR_ATmega168__)
				OCR0A = mddata[dat_pos];
#elif defined(__AVR_ATmega8__)
				ICR1L = mddata[dat_pos];
#else
#error MCU not supported yet
#endif


				snespad->update();
				snespad->getReport(&last_data);
			}
		}
		else {
			// Atari mode
			PORTC = 0x00;
			DDRC = next_data[0] ^ 0xff;
			turboSleep();

			// make sure the controller is polled at
			// 50/60 hz regardless of the loop
			// timing for turbo which is higher.
			if (turboPoll()) {
				snespad->update();
				snespad->getReport(&last_data);
			}

			turboToggleLockOn(last_data.snes.buttons & SNES_BTN_START);
		}

		turboDo();

		sel_low_dat = 0;
		sel_high_dat = 0;
		sel_x_dat = 0;

		map = maps[cur_map_id];
		while (map->snes_btn) {
			// To prevent buttons pressed at powerup (for mappings) from confusing
			// controller detection, buttons are reported as idle until we first
			// see a 'no button pressed' state.
			if (ignore_buttons && (0 == (last_data.snes.buttons & SNES_BTN_ALL))) {
				ignore_buttons = 0;
			}

			if (map->opts&OPT_TURBO) {
				if (turbo_lock_on) {
					if (turbo_state && !ignore_buttons) {
						sel_low_dat |= map->s[0];
						sel_high_dat |= map->s[1];
						sel_x_dat |= map->s[2];
					}
				}
			}

			if ((last_data.snes.buttons & map->snes_btn))
			{
				if (!(map->opts&OPT_TURBO) || turbo_state) {
					if (!ignore_buttons) {
						sel_low_dat |= map->s[0];
						sel_high_dat |= map->s[1];
						sel_x_dat |= map->s[2];
					}
				}
			}

			if ((map->opts&OPT_TURBO_SEL_SPEED)) {
				turboCycleSpeed(last_data.snes.buttons & map->snes_btn);
			}

			map++;
		}

		if (tribtn_compat) {
			next_data[0] = sel_high_dat;
			next_data[1] = sel_low_dat | 0x0c;
			next_data[2] = sel_high_dat;
			next_data[3] = sel_low_dat | 0x0c;
			next_data[4] = sel_high_dat;
			next_data[5] = sel_low_dat | 0x0c;
			next_data[6] = sel_high_dat;
			next_data[7] = sel_low_dat | 0x0c;
		}
		else {
			next_data[0] = sel_high_dat;
			next_data[1] = sel_low_dat | 0x0c; // Force right/left to 0 for detection
			next_data[2] = sel_high_dat;
			next_data[3] = sel_low_dat | 0x0c; // Force right/left to 0 for detection
			next_data[4] = sel_high_dat;
			next_data[5] = sel_low_dat | 0x3c; // FORCE UP/Dn/lef/right to 0 for detection.
			next_data[6] = sel_x_dat;
			next_data[7] = sel_high_dat & 0x03; // Keep Up/Dn/Left/Right high for detection.
		}

		for (i=0; i<8; i++) {
			next_data[i] ^= 0xff;
		}
	}

	return 0;
}
