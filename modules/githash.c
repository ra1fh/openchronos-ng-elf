/**
    modules/githash.c: display githash of source tree

	Copyright (C) 2019 Ralf Horstmann <ralf@ackstorm.de>

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

#include "menu.h"
#include "drivers/display.h"

static const char* githash = GITHASH;

static void githash_display(void)
{
	display_char(0, LCD_SEG_L1_2, githash[0], SEG_SET);
	display_char(0, LCD_SEG_L1_1, githash[1], SEG_SET);
	display_char(0, LCD_SEG_L1_0, githash[2], SEG_SET);
	display_char(0, LCD_SEG_L2_3, githash[3], SEG_SET);
	display_char(0, LCD_SEG_L2_2, githash[4], SEG_SET);
	display_char(0, LCD_SEG_L2_1, githash[5], SEG_SET);
	display_char(0, LCD_SEG_L2_0, githash[6], SEG_SET);
}

static void githash_activate(void)
{
    /* refresh display */
    githash_display();
}

static void githash_deactivate(void)
{
    display_clear(0, 0);
}

void mod_githash_init(void)
{
    menu_add_entry("GIT",
                   NULL,
                   NULL,
                   NULL,
                   NULL,
                   NULL,
                   NULL,
                   &githash_activate,
                   &githash_deactivate);
}
