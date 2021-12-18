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

#include "openchronos.h"

#include "ps.h"
#include "ps_lib.h"
#include "bmp_ps.h"
#include "vti_ps.h"
#include "timer.h"

volatile uint8_t ps_last_interrupt;

// Global flag for proper pressure sensor operation
static uint8_t ps_ok;
// Global flag set if Bosch sensors are used
static uint8_t bmp_used;
// Global flag set if VTI sensors are used
static uint8_t vti_used;

// **********************************************************************
// @fn          ps_init
// @brief       Init pressure sensors
// @param       none
// @return      none
// **********************************************************************
void ps_init(void)
{
	bmp_used = 0;
	vti_used = 0;
	ps_ok = 0;

#ifdef CONFIG_PRESSURE_BUILD_BOSCH_PS
    if (bmp_ps_init()) {
		ps_ok = 1;
        bmp_used = 1;
		return;
	}
#endif
#ifdef CONFIG_PRESSURE_BUILD_VTI_PS
	if (vti_ps_init()) {
		ps_ok = 1;
		vti_used = 1;
		return;
	}
#endif
}

// **********************************************************************
// @fn          ps_start
// @brief       Start pressure sensor I/O
// @param       none
// @return      none
// **********************************************************************
void ps_start(void)
{
	if (bmp_used) {
#ifdef CONFIG_PRESSURE_BUILD_BOSCH_PS
		bmp_ps_start();
#endif
	} else {
#ifdef CONFIG_PRESSURE_BUILD_VTI_PS
		vti_ps_start();
#endif
	}
}

// **********************************************************************
// @fn          ps_stop
// @brief       Stop pressure sensor I/O
// @param       none
// @return      none
// **********************************************************************
void ps_stop(void)
{
	if (bmp_used) {
#ifdef CONFIG_PRESSURE_BUILD_BOSCH_PS
		bmp_ps_stop();
#endif
	} else {
#ifdef CONFIG_PRESSURE_BUILD_VTI_PS
		vti_ps_stop();
#endif
	}
}

// **********************************************************************
// @fn          ps_check_last_interrupt
// @brief       Check for interrupt from ps and store it
// @param       none
// @return      uint8_t     last interrupt value
// **********************************************************************
void ps_store_last_interrupt(void)
{
    /* Check if pressure interrupt flag */
    if ((P2IFG & PS_INT_PIN) == PS_INT_PIN)
        ps_last_interrupt = 1;
}

// **********************************************************************
// @fn          ps_get_last_interrupt
// @brief       Get last interrupt flag and clear it
// @param       none
// @return      uint8_t     last interrupt value
// **********************************************************************
uint8_t ps_get_last_interrupt(void)
{
	if (ps_last_interrupt) {
        ps_last_interrupt = 0;
		return 1;
	} else {
		return 0;
	}
}

uint8_t ps_handle_interrupt(void) {
	if (bmp_used) {
#ifdef CONFIG_PRESSURE_BUILD_BOSCH_PS
		return bmp_ps_handle_interrupt();
#endif
	} else {
#ifdef CONFIG_PRESSURE_BUILD_VTI_PS
		return vti_ps_handle_interrupt();
#endif
	}
	return 0;
}

// **********************************************************************
// @fn          ps_get_pa
// @brief       Stop pressure sensor I/O
// @param       none
// @return      uint32_t     pressure in Pa
// **********************************************************************
uint32_t ps_get_pa(void) {
	if (bmp_used) {
#ifdef CONFIG_PRESSURE_BUILD_BOSCH_PS
		return bmp_ps_get_pa();
#endif
	} else {
#ifdef CONFIG_PRESSURE_BUILD_VTI_PS
		return vti_ps_get_pa();
#endif
	}
	return 0;
}

// **********************************************************************
// @fn          ps_ok_get
// @brief       Get ps_ok state
// @param       none
// @return      uint8_t     ps_ok state
// **********************************************************************
uint8_t ps_ok_get(void)
{
	return ps_ok;
}
