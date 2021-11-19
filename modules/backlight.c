/**
    modules/backlight.c: backlight module

	Copyright (C) 2021 Ralf Horstmann <ralf@ackstorm.de>

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
#include "drivers/ports.h"

static uint8_t backlight_sec = 0;

static void backlight_timer(enum sys_message msg)
{
	if (--backlight_sec == 0) {
		sys_messagebus_unregister(&backlight_timer, SYS_MSG_RTC_SECOND);
		P2OUT &= ~PORTS_BTN_BL_PIN;
		P2DIR &= ~PORTS_BTN_BL_PIN;
	}
}

static void backlight_button(enum sys_message msg)
{
	if (backlight_sec == 0 && ports_button_pressed_peek(PORTS_BTN_BL_PIN, 0)) {
		sys_messagebus_register(&backlight_timer, SYS_MSG_RTC_SECOND);
		P2OUT |= PORTS_BTN_BL_PIN;
		P2DIR |= PORTS_BTN_BL_PIN;
		backlight_sec = 5;
	}
}

void mod_backlight_init()
{
	sys_messagebus_register(&backlight_button, SYS_MSG_BUTTON);
}
