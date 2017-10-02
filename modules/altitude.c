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

static void altitude_event(enum sys_message msg)
{
    if (msg & SYS_MSG_RTC_SECOND) {
        display_symbol(0, LCD_SEG_L1_COL, ((rtca_time.sec & 0x01) ? SEG_ON : SEG_OFF));
    }
}

/* update screens with fake event */
static inline void update_screen()
{
    altitude_event(SYS_MSG_RTC_SECOND);
}

/************************ menu callbacks **********************************/
static void altitude_activated()
{
    sys_messagebus_register(&altitude_event, SYS_MSG_RTC_SECOND);

    /* create one screens */
    lcd_screens_create(1); // 0: log, 1: show, 2: diff

	display_chars(0,  LCD_SEG_L1_3_0, "ALTI", SEG_SET);

	/* enable ps */
	ps_start();

	if (ps_ok) {
		display_chars(0,  LCD_SEG_L2_4_0, "OK   ", SEG_SET);
	} else {
		display_chars(0,  LCD_SEG_L2_4_0, "ERR 0", SEG_SET);
	}

    /* update screens with fake event */
    update_screen();
}

static void altitude_deactivated()
{
    sys_messagebus_unregister_all(&altitude_event);

    /* destroy virtual screens */
    lcd_screens_destroy();

    /* clean up screen */
    display_symbol(0, LCD_SEG_L1_COL, SEG_OFF);
    display_symbol(0, LCD_SYMB_PM, SEG_OFF);
	display_symbol(0, LCD_SYMB_ARROW_UP, SEG_OFF);
	display_symbol(0, LCD_SYMB_ARROW_DOWN, SEG_OFF);
	display_symbol(0, LCD_SYMB_MAX, SEG_OFF);

    display_clear(0, 1);
    display_clear(0, 2);

	/* disable ps */
	ps_stop();
}

/* Num button press callback */
static void num_pressed()
{
	uint32_t altitude;

	/* read altitude */
	if (ps_ok) {
		PS_INT_IFG &= ~PS_INT_PIN;
		PS_INT_IE |= PS_INT_PIN;

		while ((PS_INT_IN & PS_INT_PIN) == 0) ;

		altitude = ps_get_pa();

		_printf(0, LCD_SEG_L2_4_0, "%5u", altitude / 10);

	} else {
		display_chars(0,  LCD_SEG_L2_4_0, "ERR 1", SEG_SET);
	}
}

static void num_long_pressed()
{
}

static void up_pressed()
{
    update_screen();
}

static void down_pressed()
{
    update_screen();
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
