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

#define USE_SERIES 0
#define USE_LIBM 0
#define USE_LUT 1

extern uint8_t ps_ok;
static uint8_t altitude_dots = 0;
static uint8_t altitude_screen = 0;
static uint16_t altitude_qnh_cur = 1013;
static uint16_t altitude_qnh_tmp = 1013;

#if USE_SERIES

/**********************************************************************
 Altitude Calculation

 ISA standard atmosphere:

        altitude = (T0 / -a) * (1 - (p/p0)^(-a * R / g))
 With:
        T0 = 288.15 K
         a = -0.065 K/m
        p0 = 101325 Pa
         R =    287 J/kgK
         g =   9.82 m/s^2
        p0 = QNH

 Computing the constants results in:

        altitude = 44330m * (1 - (p/p0)^0.19)

 To calculate x^0.19, we use a binomial series:

        (1+x)^a = Sum(k=0, inf)(a over k) * (x)^k

     => x^a     = Sum(k=0, inf)(a over k) * (x-1)^k

     => x^0.19  = Sum(k=0, inf)(0.19 over k) * (x-1)^k

     => x^0.19  = (0.19 over 0) * (x-1)^0
                + (0.19 over 1) * (x-1)^1
                + (0.19 over 2) * (x-1)^2 ...

     => x^0.19  = 1.00000
                + 0.19000 * (x-1)
                - 0.07695 * (x-1)^2 ...

 The coefficients (0.19 over k) can be precomputed:
***********************************************************************/

static const float coefficients[] = {
     1.000000,
     0.190000,
    -0.076950,
     0.046427,
    -0.032615,
     0.024852,
    -0.019923,
     0.016536,
    -0.014077,
     0.012215,
};

/*
 The series converges quickly for low altitudes (p ~ QNH, x ~ 1) with
 only 2 iteration steps needed. At x = 0.5 (~18500ft) 9 iterations are
 needed and the error is approx. 3ft. Beyond that, the error gets
 larger.
*/

static float altitude_calc(float p_meas, float qnh) {
    const float term = 0.00005;
    const float u = (p_meas / qnh) - 1;
    float sum = 0;
    float prev = 0;
    float prod = 1;
    int i;

    for (i = 0; i < sizeof(coefficients) / sizeof(coefficients[0]); i++) {
        prev = sum;
        sum += coefficients[i] * prod;
        prod *= u;
        if (sum - prev < term && sum - prev > -term)
            break;
    }
    return (1.0 - sum);
}

/**********************************************************************/

static void altitude_event(enum sys_message msg)
{
    uint32_t p_meas;
    int32_t alt;
    float altf;

    altitude_dots = !altitude_dots;
    display_symbol(0, LCD_SYMB_ARROW_UP,   altitude_dots ? SEG_ON  : SEG_OFF);
    display_symbol(0, LCD_SYMB_ARROW_DOWN, altitude_dots ? SEG_OFF : SEG_ON );
    display_symbol(1, LCD_SYMB_ARROW_UP,   altitude_dots ? SEG_ON  : SEG_OFF);
    display_symbol(1, LCD_SYMB_ARROW_DOWN, altitude_dots ? SEG_OFF : SEG_ON );
    display_symbol(2, LCD_SYMB_ARROW_UP,   altitude_dots ? SEG_ON  : SEG_OFF);
    display_symbol(2, LCD_SYMB_ARROW_DOWN, altitude_dots ? SEG_OFF : SEG_ON );

    p_meas = ps_get_pa();

    altf = altitude_calc(p_meas * 1.0, altitude_qnh_cur * 100.0);
    if (altf > 0) {
        alt = 145442.2 * altf;
        _printf(0, LCD_SEG_L2_5_0, "%6u", alt);
        alt = 44330.8 * altf;
        _printf(1, LCD_SEG_L2_5_0, "%6u", alt);
    } else {
        display_chars(0, LCD_SEG_L2_4_0, "UNDER", SEG_SET);
        display_chars(1, LCD_SEG_L2_4_0, "UNDER", SEG_SET);
    }

    _printf(2, LCD_SEG_L2_5_0, "%6u", p_meas);
}
#endif

#if USE_LUT

#include "altitude.h"

static const int32_t alt_scale = 10000;
static const int32_t alt_scale_meter = 32808;

static int32_t altitude_calc(int16_t p, int16_t qnh) {
    int16_t pindex = (p - p_low) / p_step;
    int16_t qindex = (qnh - qnh_low) / qnh_step;
    int16_t pbase = p_low + pindex * p_step;
    int16_t qbase = qnh_low + qindex * qnh_step;
    int32_t abase = alt_scale * altitude[qindex][pindex];
    int32_t atopp = alt_scale * altitude[qindex][pindex+1];
    int32_t atopq = alt_scale * altitude[qindex+1][pindex];
    int32_t pgrad = (atopp - abase) / p_step;
    int32_t qgrad = (atopq - abase) / qnh_step;
    int32_t alt;

    alt = abase + (p - pbase) * pgrad + (qnh - qbase) * qgrad;
    return alt;
}

static void altitude_event(enum sys_message msg)
{
    uint32_t p_meas;
    int32_t alt;

    altitude_dots = !altitude_dots;
    display_symbol(0, LCD_SYMB_ARROW_UP,   altitude_dots ? SEG_ON  : SEG_OFF);
    display_symbol(0, LCD_SYMB_ARROW_DOWN, altitude_dots ? SEG_OFF : SEG_ON );
    display_symbol(1, LCD_SYMB_ARROW_UP,   altitude_dots ? SEG_ON  : SEG_OFF);
    display_symbol(1, LCD_SYMB_ARROW_DOWN, altitude_dots ? SEG_OFF : SEG_ON );
    display_symbol(2, LCD_SYMB_ARROW_UP,   altitude_dots ? SEG_ON  : SEG_OFF);
    display_symbol(2, LCD_SYMB_ARROW_DOWN, altitude_dots ? SEG_OFF : SEG_ON );

    p_meas = ps_get_pa();
    alt = altitude_calc(p_meas / 10, altitude_qnh_cur);
    if (alt > 0) {
        _printf(0, LCD_SEG_L2_5_0, "%6u", alt / alt_scale);
        _printf(1, LCD_SEG_L2_5_0, "%6u", alt / alt_scale_meter);
    } else {
        display_chars(0, LCD_SEG_L2_4_0, "UNDER", SEG_SET);
        display_chars(1, LCD_SEG_L2_4_0, "UNDER", SEG_SET);
    }

    _printf(2, LCD_SEG_L2_5_0, "%6u", p_meas);
}

#endif

#if USE_LIBM

#include <math.h>

static float altitude_calc(float p, float qnh) {
    return 44300.0 * (1.0 - powf((float) p / (float) qnh, 0.19));
}

static void altitude_event(enum sys_message msg)
{
    uint32_t p_meas;
    uint32_t alti;
    float altf;

    altitude_dots = !altitude_dots;
    display_symbol(0, LCD_SYMB_ARROW_UP,   altitude_dots ? SEG_ON  : SEG_OFF);
    display_symbol(0, LCD_SYMB_ARROW_DOWN, altitude_dots ? SEG_OFF : SEG_ON );
    display_symbol(1, LCD_SYMB_ARROW_UP,   altitude_dots ? SEG_ON  : SEG_OFF);
    display_symbol(1, LCD_SYMB_ARROW_DOWN, altitude_dots ? SEG_OFF : SEG_ON );
    display_symbol(2, LCD_SYMB_ARROW_UP,   altitude_dots ? SEG_ON  : SEG_OFF);
    display_symbol(2, LCD_SYMB_ARROW_DOWN, altitude_dots ? SEG_OFF : SEG_ON );

    p_meas = ps_get_pa();
    altf = altitude_calc(p_meas * 1.0, altitude_qnh_cur * 100.0);
    if (altf > 0) {
        alti = altf;
        _printf(1, LCD_SEG_L2_5_0, "%6u", alti);
        altf = altf * 3.28084;
        alti = altf;
        _printf(0, LCD_SEG_L2_5_0, "%6u", alti);
    } else {
        display_chars(0, LCD_SEG_L2_4_0, "UNDER", SEG_SET);
        display_chars(1, LCD_SEG_L2_4_0, "UNDER", SEG_SET);
    }

    _printf(2, LCD_SEG_L2_5_0, "%6u", p_meas);
}

#endif




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
    menu_editmode_start(&altitude_edit_save, NULL, altitude_edit_items);
}

/************************ init **********************************/
void mod_altitude_init()
{
    menu_add_entry("ALTI",
					NULL,           		 /* up         */
					NULL,         			 /* down       */
					&num_pressed,          	 /* num        */
					&star_long_pressed,		 /* star long  */
					NULL,     				 /* num long   */
					NULL,                  	 /* up + down  */
					&altitude_activated,     /* activate   */
					&altitude_deactivated);  /* deactivate */
}
