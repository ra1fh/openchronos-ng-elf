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

#include <math.h>

#include "messagebus.h"
#include "menu.h"

/* drivers */
#include "drivers/rtca.h"
#include "drivers/display.h"
#include "drivers/vti_ps.h"

extern uint8_t ps_ok;
static uint8_t altitude_dots = 0;
static uint8_t altitude_screen = 0;
static uint32_t altitude_qnh_cur = 1013;
static uint32_t altitude_qnh_tmp = 1013;

static void altitude_print_line2(int display, uint32_t val, uint32_t scale)
{
	uint32_t digits[10] = { 0 };
	uint32_t v = val;
	const char *fmt = "%2u";
	int i;
	
	for (i = 0; i < (sizeof(digits) / sizeof(digits[0])); ++i) {
		digits[i] = v % 10;
		v  = v / 10;
	}

	if (val >= 10000) {
		_printf(display, LCD_SEG_L2_5_4, fmt, digits[5 + scale] * 10 + digits[4 + scale]);
		fmt = "%02u";
	} else {
		display_chars(display, LCD_SEG_L2_5_4, "  ", SEG_SET);
	}
	if (val >= 100) {
		_printf(display, LCD_SEG_L2_3_2, fmt, digits[3 + scale] * 10 + digits[2 + scale]);
		fmt = "%02u";
	} else {
		display_chars(display, LCD_SEG_L2_3_2, "  ", SEG_SET);
	}
	_printf(display, LCD_SEG_L2_1_0, fmt, digits[1 + scale] * 10 + digits[0 + scale]);
}

static void altitude_event(enum sys_message msg)
{
	uint32_t p_meas;
	uint32_t p_disp;
	uint32_t alt;
	double qnh = altitude_qnh_cur;

	altitude_dots = !altitude_dots;
	display_symbol(0, LCD_SYMB_ARROW_UP,   altitude_dots ? SEG_ON  : SEG_OFF);
	display_symbol(0, LCD_SYMB_ARROW_DOWN, altitude_dots ? SEG_OFF : SEG_ON );
	display_symbol(1, LCD_SYMB_ARROW_UP,   altitude_dots ? SEG_ON  : SEG_OFF);
	display_symbol(1, LCD_SYMB_ARROW_DOWN, altitude_dots ? SEG_OFF : SEG_ON );
	display_symbol(2, LCD_SYMB_ARROW_UP,   altitude_dots ? SEG_ON  : SEG_OFF);
	display_symbol(2, LCD_SYMB_ARROW_DOWN, altitude_dots ? SEG_OFF : SEG_ON );

	p_meas = p_disp = ps_get_pa();

	alt = 145442.2 * (1.0 - pow(((double)p_meas) / qnh / 100.0, 0.19));
	altitude_print_line2(0, alt, 0);

	alt = 44330.8 * (1.0 - pow(((double)p_meas) / qnh / 100.0, 0.19));
	altitude_print_line2(1, alt, 0);

	altitude_print_line2(2, p_meas, 0);
}

/************************ menu callbacks **********************************/
static void altitude_activated()
{
	/* 0: Pressure [hPA]
     * 1: Altitude [ft]
	 * 2: Altitude [m]
	 * 3: QNH Setting
	 */
    lcd_screens_create(4);

	display_chars(0,  LCD_SEG_L1_3_0, "ALTI", SEG_SET);
	display_symbol(0, LCD_UNIT_L1_FT, SEG_ON);
	
	display_chars(1,  LCD_SEG_L1_3_0, "ALTI", SEG_SET);
	display_symbol(1, LCD_UNIT_L1_M, SEG_ON);

	display_chars(2,  LCD_SEG_L1_3_0, " HPA", SEG_SET);
	display_symbol(2, LCD_SEG_L2_DP, SEG_ON);

	display_chars(3,  LCD_SEG_L1_3_0, " QNH", SEG_SET);
	_printf(3, LCD_SEG_L2_3_0, "%4u", altitude_qnh_cur);
	
    sys_messagebus_register(&altitude_event, SYS_MSG_PS_INT);

	if (ps_ok) {
		display_chars(0,  LCD_SEG_L2_4_0, "OK   ", SEG_SET);
		display_chars(1,  LCD_SEG_L2_4_0, "OK   ", SEG_SET);
		display_chars(2,  LCD_SEG_L2_4_0, "OK   ", SEG_SET);
	} else {
		display_chars(0,  LCD_SEG_L2_4_0, "ERR 0", SEG_SET);
		display_chars(1,  LCD_SEG_L2_4_0, "ERR 0", SEG_SET);
		display_chars(2,  LCD_SEG_L2_4_0, "ERR 0", SEG_SET);
	}

	/* enable ps */
	ps_start();

	altitude_screen = 0;
	lcd_screen_activate(altitude_screen);
}

static void altitude_deactivated()
{
    sys_messagebus_unregister_all(&altitude_event);

	/* disable ps */
	ps_stop();

    /* destroy virtual screens */
    lcd_screens_destroy();

    /* clean up screen */
    display_clear(0, 0);
}

static void num_pressed()
{
	altitude_screen += 1;
	if (altitude_screen > 2)
		altitude_screen = 0;
	lcd_screen_activate(altitude_screen);
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

/************************ edit mode **********************************/

static void altitude_print_qnh(void)
{
	_printf(3, LCD_SEG_L2_3_0, "%4u", altitude_qnh_tmp);
}

static void altitude_edit_sel(void)
{
    display_chars(3, LCD_SEG_L2_3_0, NULL, BLINK_ON);
}

static void altitude_edit_dsel(void)
{
    display_chars(3, LCD_SEG_L2_3_0, NULL, BLINK_OFF);
}

static void altitude_edit_set(int8_t step)
{
	altitude_qnh_tmp += step;
	if (altitude_qnh_tmp < 900)
		altitude_qnh_tmp = 900;
	if (altitude_qnh_tmp > 1100)
		altitude_qnh_tmp = 1100;
    altitude_print_qnh();
}

static struct menu_editmode_item altitude_edit_items[] = {
    {&altitude_edit_sel, &altitude_edit_dsel, &altitude_edit_set},
    { NULL },
};

static void altitude_edit_save(void)
{
	altitude_qnh_cur = altitude_qnh_tmp;
	lcd_screen_activate(altitude_screen);
}

static void star_long_pressed()
{
    altitude_print_qnh();
	lcd_screen_activate(3);
	altitude_qnh_tmp = altitude_qnh_cur;
    menu_editmode_start(&altitude_edit_save, altitude_edit_items);
}

/************************ init **********************************/
void mod_altitude_init()
{
    menu_add_entry("ALTI",
					&up_pressed,           	 /* up         */
					&down_pressed,         	 /* down       */
					&num_pressed,          	 /* num        */
					&star_long_pressed,		 /* star long  */
					&num_long_pressed,     	 /* num long   */
					NULL,                  	 /* up + down  */
					&altitude_activated,     /* activate   */
					&altitude_deactivated);  /* deactivate */
}
