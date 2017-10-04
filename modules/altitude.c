/**
    modules/altitude.c: altitude module

	Copyright (C) 2017 Ralf Horstmann <ralf@ackstorm.de>

    http://github.com/ra1fh/openchronos-ng-elf

    This file is part of openchronos-ng-elf.

    openchronos-ng-elf is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    openchronos-ng-elf is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
**/

#include "messagebus.h"
#include "menu.h"

/* drivers */
#include "drivers/rtca.h"
#include "drivers/display.h"
#include "drivers/vti_ps.h"

extern uint8_t ps_ok;
static uint8_t altitude_dots;

static void altitude_event(enum sys_message msg)
{
	uint32_t pressure;
	uint8_t digits[10] = { 0 };
	int i;

	altitude_dots = !altitude_dots;
	display_symbol(0, LCD_SYMB_ARROW_UP,   altitude_dots ? SEG_ON  : SEG_OFF);
	display_symbol(0, LCD_SYMB_ARROW_DOWN, altitude_dots ? SEG_OFF : SEG_ON );
	display_symbol(0, LCD_SEG_L2_DP, SEG_ON);

	pressure = ps_get_pa();
	for (i = 0; i < sizeof(digits); ++i) {
		digits[i] = pressure % 10;
		pressure  = pressure / 10;
	}

	i = 0;
	_printf(0, LCD_SEG_L2_1_0, "%02u", digits[1 + i] * 10 + digits[0 + i]);
	_printf(0, LCD_SEG_L2_3_2, "%02u", digits[3 + i] * 10 + digits[2 + i]);
	_printf(0, LCD_SEG_L2_5_4, "%02u", digits[5 + i] * 10 + digits[4 + i]);
}

/* update screens with fake event */
static inline void update_screen()
{
    altitude_event(SYS_MSG_RTC_SECOND);
}

/************************ menu callbacks **********************************/
static void altitude_activated()
{
    /* create one screens */
    lcd_screens_create(1); // 0: log, 1: show, 2: diff

	display_chars(0,  LCD_SEG_L1_3_0, "ALTI", SEG_SET);

    sys_messagebus_register(&altitude_event, SYS_MSG_PS_INT);

	if (ps_ok) {
		display_chars(0,  LCD_SEG_L2_4_0, "OK   ", SEG_SET);
	} else {
		display_chars(0,  LCD_SEG_L2_4_0, "ERR 0", SEG_SET);
	}

	/* enable ps */
	ps_start();
}

static void altitude_deactivated()
{
    sys_messagebus_unregister_all(&altitude_event);

	/* disable ps */
	ps_stop();

    /* destroy virtual screens */
    lcd_screens_destroy();

    /* clean up screen */
    display_clear(0, 1);
    display_clear(0, 2);
}

static void num_pressed()
{
}

static void num_long_pressed()
{
}

static void up_pressed()
{
}

static void down_pressed()
{
}

void mod_altitude_init()
{
    menu_add_entry("ALTI",
					&up_pressed,           	 /* up         */
					&down_pressed,         	 /* down       */
					&num_pressed,          	 /* num        */
					NULL,                  	 /* star long  */
					&num_long_pressed,     	 /* num long   */
					NULL,                  	 /* up + down  */
					&altitude_activated,     /* activate   */
					&altitude_deactivated);  /* deactivate */
}
