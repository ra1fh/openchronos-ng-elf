/**
    drivers/ps.c: pressure interface functions for BMP and VTI

    Copyright (C) 2021 Ralf Horstmann <ralf@ackstorm.de>

    https://github.com/ra1fh/openchronos-ng-elf

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

#ifndef PS_H_
#define PS_H_

extern void ps_init(void);
extern void ps_start(void);
extern void ps_stop(void);
extern void ps_store_last_interrupt(void);
extern uint8_t ps_get_last_interrupt(void);
extern void ps_handle_interrupt(void);
extern uint32_t ps_get_pa(void);
extern uint8_t ps_ok_get(void);

#endif /* PS_H_ */
