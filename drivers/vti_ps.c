// *************************************************************************************************
//
//  Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
//
//
//    Redistribution and use in source and binary forms, with or without
//    modification, are permitted provided that the following conditions
//    are met:
//
//      Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//      Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the
//      distribution.
//
//      Neither the name of Texas Instruments Incorporated nor the names of
//      its contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
//
//    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//    OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *************************************************************************************************
// VTI SCP1000-D0x pressure sensor driver functions
// *************************************************************************************************

#include "openchronos.h"

#include "ps.h"
#include "ps_lib.h"
#include "vti_ps.h"
#include "timer.h"

uint32_t vti_ps_pa;
uint8_t vti_ps_last_command;

uint16_t vti_ps_read_register(uint8_t address, uint8_t mode);
uint8_t vti_ps_write_register(uint8_t address, uint8_t data);

// **********************************************************************
// @fn          vti_ps_init
// @brief       Init pressure sensor I/O
// @param       none
// @return      1=success 0=failure
// **********************************************************************
uint8_t vti_ps_init(void)
{
    volatile uint8_t status, eeprom;
    __attribute__((unused)) volatile uint8_t success;

	vti_ps_last_command = 0;
    ps_init_io();

    // Reset pressure sensor -> powerdown sensor
    success = vti_ps_write_register(0x06, 0x01);

    // 100msec delay
    timer0_delay(100, LPM3_bits);

    // Check if STATUS register BIT0 is cleared
    status = vti_ps_read_register(0x07, PS_I2C_8BIT_ACCESS);
    if (((status & BIT0) == 0) && (status != 0))
    {
        // Check EEPROM checksum in DATARD8 register
        eeprom = vti_ps_read_register(0x7F, PS_I2C_8BIT_ACCESS);
        if (eeprom == 0x01)
            return 1;
    }
	return 0;
}

// **********************************************************************
// @fn          vti_ps_measure
// @brief       Init pressure sensor registers and start sampling
// @param       none
// @return      none
// **********************************************************************
void vti_ps_measure(void)
{
	if (vti_ps_last_command != 0)
		return;

    // read pending data that may block further measurements. this can
    // happen when data arrives after vti_ps_stop has been called.
    vti_ps_read_register(0x7F, PS_I2C_8BIT_ACCESS);
    vti_ps_read_register(0x80, PS_I2C_16BIT_ACCESS);

    // Enable DRDY IRQ on rising edge
    PS_INT_IFG &= ~PS_INT_PIN;
    PS_INT_IE |= PS_INT_PIN;

    // Start sampling triggered mode
    vti_ps_write_register(0x03, 0x0C);
	vti_ps_last_command = 0x0C;
}

// **********************************************************************
// @fn          vti_ps_write_register
// @brief       Write a byte to the pressure sensor
// @param       uint8_t address              Register address
//              uint8_t data                 Data to write
// @return      uint8_t
// **********************************************************************
uint8_t vti_ps_write_register(uint8_t address, uint8_t data)
{
    return ps_write_register(VTI_SCP1000_I2C_ADDR << 1, address, data);
}

// **********************************************************************
// @fn          vti_ps_read_register
// @brief       Read a byte from the pressure sensor
// @param       uint8_t address   Register address
//              uint8_t mode      PS_TWI_8BIT_ACCESS, PS_TWI_16BIT_ACCESS
// @return      uint16_t          Register content
// **********************************************************************
uint16_t vti_ps_read_register(uint8_t address, uint8_t mode)
{
    return ps_read_register(VTI_SCP1000_I2C_ADDR << 1, address, mode);
}

// **********************************************************************
// @fn          vti_ps_get_pa
// @brief       Read out pressure in Pa. Range is 30000 .. 120000 Pa.
// @param       none
// @return      uint32_t             15-bit pressure sensor value (Pa)
// **********************************************************************
uint32_t vti_ps_get_pa(void)
{
	return vti_ps_pa;
}

// **********************************************************************
// @fn          vti_ps_get_pa
// @brief       Read out pressure in Pa. Range is 30000 .. 120000 Pa.
// @param       none
// @return      uint8_t    1=new ps value  0=no ps value
// **********************************************************************
uint8_t vti_ps_handle_interrupt(void)
{
    volatile uint32_t data = 0;

    // Get 3 MSB from DATARD8 register
    data = vti_ps_read_register(0x7F, PS_I2C_8BIT_ACCESS);
    data = ((data & 0x07) << 8) << 8;

    // Get 16 LSB from DATARD16 register
    data |= vti_ps_read_register(0x80, PS_I2C_16BIT_ACCESS);

    // Convert decimal value to Pa
    data = (data >> 2);

    vti_ps_pa = data;

    // Disable DRDY IRQ
    PS_INT_IE  &= ~PS_INT_PIN;
    PS_INT_IFG &= ~PS_INT_PIN;

    // Put sensor to standby (may not be necessary in triggered mode)
    vti_ps_write_register(0x03, 0x00);
	vti_ps_last_command = 0;
	return 1;
}

// **********************************************************************
// @fn          vti_ps_get_temp
// @brief       Read out temperature.
// @param       none
// @return      uint16_t        13-bit temperature value in xx.x K format
// **********************************************************************
uint16_t vti_ps_get_temp(void)
{
    volatile uint16_t data = 0;
    uint16_t temp = 0;
    uint8_t is_negative = 0;
    uint16_t kelvin;

    // Get 13 bit from TEMPOUT register
    data = vti_ps_read_register(0x81, PS_I2C_16BIT_ACCESS);

    // Convert negative temperatures
    if ((data >> 13) & 0x1)
    {
        // Sign extend temperature
        data |= 0xC000;
        // Convert two's complement
        data = ~data;
        data += 1;
        is_negative = 1;
    }

    temp = data / 2;

    // Convert from °C to K
    if (is_negative)
        kelvin = 2732 - temp;
    else
        kelvin = temp + 2732;

    return (kelvin);
}
