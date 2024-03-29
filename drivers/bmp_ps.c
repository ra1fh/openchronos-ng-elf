/*
 *  Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Bosch BMP085 pressure sensor driver functions
 */

#include "openchronos.h"

#include "ps.h"
#include "ps_lib.h"
#include "bmp_ps.h"
#include "timer.h"

// Variable to hold BMP085 calibration data
bmp_085_calibration_param_t bmp_cal_param;
// Paramater used by temperature and pressure measurement
long bmp_param_b5;

uint32_t bmp_ps_pa;
uint16_t bmp_ps_temp;
uint32_t bmp_ps_err;

uint8_t bmp_ps_last_command;

// **********************************************************************
// @fn          bmp_ps_init
// @brief       Init pressure sensor I/O
// @param       none
// @return      1=success 0=failure
// **********************************************************************
uint8_t bmp_ps_init(void)
{
    volatile uint8_t status;

	bmp_ps_last_command = 0;
	bmp_ps_err = 0;
    ps_init_io();

    // Read ChipID to check if communication is working
    status = bmp_ps_read_register(BMP_085_CHIP_ID_REG, PS_I2C_8BIT_ACCESS);
    if (status == BMP_085_CHIP_ID)
    {
        bmp_ps_get_cal_param();
        return 1;
    }
	return 0;
}


// **********************************************************************
// @fn          bmp_ps_get_cal_param
// @brief       read out calibration parameters from BMP085 memory to
//              cal_param
// @param       none
// @return      none
// **********************************************************************
void bmp_ps_get_cal_param(void)
{
    /*parameters AC1-AC6*/
    bmp_cal_param.ac1 = bmp_ps_read_register(BMP_085_PROM_START_ADDR + 0, PS_I2C_16BIT_ACCESS);
    bmp_cal_param.ac2 = bmp_ps_read_register(BMP_085_PROM_START_ADDR + 2, PS_I2C_16BIT_ACCESS);
    bmp_cal_param.ac3 = bmp_ps_read_register(BMP_085_PROM_START_ADDR + 4, PS_I2C_16BIT_ACCESS);
    bmp_cal_param.ac4 = bmp_ps_read_register(BMP_085_PROM_START_ADDR + 6, PS_I2C_16BIT_ACCESS);
    bmp_cal_param.ac5 = bmp_ps_read_register(BMP_085_PROM_START_ADDR + 8, PS_I2C_16BIT_ACCESS);
    bmp_cal_param.ac6 = bmp_ps_read_register(BMP_085_PROM_START_ADDR + 10,PS_I2C_16BIT_ACCESS);

    /*parameters B1,B2*/
    bmp_cal_param.b1 = bmp_ps_read_register(BMP_085_PROM_START_ADDR + 12, PS_I2C_16BIT_ACCESS);
    bmp_cal_param.b2 = bmp_ps_read_register(BMP_085_PROM_START_ADDR + 14, PS_I2C_16BIT_ACCESS);

    /*parameters MB,MC,MD*/
    bmp_cal_param.mb = bmp_ps_read_register(BMP_085_PROM_START_ADDR + 16, PS_I2C_16BIT_ACCESS);
    bmp_cal_param.mc = bmp_ps_read_register(BMP_085_PROM_START_ADDR + 18, PS_I2C_16BIT_ACCESS);
    bmp_cal_param.md = bmp_ps_read_register(BMP_085_PROM_START_ADDR + 20, PS_I2C_16BIT_ACCESS);
}

// **********************************************************************
// @fn          bmp_ps_measure
// @brief       Init pressure sensor registers and start sampling
// @param       none
// @return      none
// **********************************************************************
void bmp_ps_measure(void)
{
	if (bmp_ps_last_command != 0)
		return;

    PS_INT_IFG &= ~PS_INT_PIN;
    PS_INT_IE |= PS_INT_PIN;

    // Start sampling temperature
	bmp_ps_last_command = BMP_085_T_MEASURE;
    bmp_ps_write_register(BMP_085_CTRL_MEAS_REG, BMP_085_T_MEASURE);
}

// **********************************************************************
// @fn          bmp_ps_write_register
// @brief       Write a byte to the pressure sensor
// @param       uint8_t address              Register address
//              uint8_t data                 Data to write
// @return      uint8_t
// **********************************************************************
uint8_t bmp_ps_write_register(uint8_t address, uint8_t data)
{
    return ps_write_register(BMP_085_I2C_ADDR << 1, address, data);
}

// **********************************************************************
// @fn          bmp_ps_read_register
// @brief       Read a byte from the pressure sensor
// @param       uint8_t address  Register address
//              uint8_t mode     PS_I2C_8BIT_ACCESS, PS_I2C_16BIT_ACCESS
// @return      uint16_t         Register content
// **********************************************************************
uint16_t bmp_ps_read_register(uint8_t address, uint8_t mode)
{
    return ps_read_register(BMP_085_I2C_ADDR << 1, address, mode);
}

// **********************************************************************
// @fn          bmp_ps_get_pa
// @brief       Read out pressure in Pa. Range is 30000 .. 120000 Pa.
// @param       none
// @return      uint32_t      15-bit pressure sensor value (Pa)
// **********************************************************************
uint32_t bmp_ps_get_pa(void)
{
	return bmp_ps_pa;
}

// **********************************************************************
// @fn          bmp_ps_get_temp
// @brief       Read out temperature.
// @param       none
// @return      uint16_t      13-bit temperature value in xx.x K format
// **********************************************************************
uint16_t bmp_ps_get_temp(void)
{
	return bmp_ps_temp;
}

// **********************************************************************
// @fn          bmp_ps_read_pressure
// @brief       Read out pressure
// @param       none
// @return      none
// **********************************************************************
void bmp_ps_read_pressure(void)
{
    uint16_t up;            // uncompensated pressure
    int32_t pressure, x1, x2, x3, b3, b6;
    uint32_t result, b4, b7;

    // Add Compensation and convert decimal value to Pa

    b6 = bmp_param_b5 - 4000;
    //*****calculate B3************
    x1 = (b6 * b6) >> 12;
    x1 = (bmp_cal_param.b2 * x1) / 2048;

    x2 = (bmp_cal_param.ac2 * b6) / 2048;

    x3 = x1 + x2;

    b3 = (((((long) bmp_cal_param.ac1) * 4 + x3)) + 2) / 4;

    //*****calculate B4************
    x1 = (bmp_cal_param.ac3 * b6) / 8192;
    x2 = (bmp_cal_param.b1 * ((b6 * b6) >> 12)) / 65536;
    x3 = ((x1 + x2) + 2) / 4;
    b4 = (bmp_cal_param.ac4 * (uint32_t) (x3 + 32768)) / 32768;

    // Get MSB from ADC_OUT_MSB_REG
    up = bmp_ps_read_register(BMP_085_ADC_OUT_MSB_REG, PS_I2C_16BIT_ACCESS);

    b7 = ((uint32_t)(up - b3) * 50000);
    if (b7 < 0x80000000)
    {
      pressure = (b7 * 2) / b4;
    }
    else
    {
      pressure = (b7 / b4) * 2;
    }

    x1 = pressure / 256;
    x1 *= x1;
    x1 = (x1 * BMP_SMD500_PARAM_MG) / 65536;
    x2 = (pressure * BMP_SMD500_PARAM_MH) / 65536;
    result = pressure + (x1 + x2 + BMP_SMD500_PARAM_MI) / 16;   // pressure in Pa

	// store result
	bmp_ps_pa = result;
}

// **********************************************************************
// @fn          bmp_ps_read_temp
// @brief       Read out temperature.
// @param       none
// @return      none
// **********************************************************************
uint16_t bmp_ps_read_temp(void)
{
    uint16_t ut;
    int32_t x1, x2;
    int16_t temperature;
    uint16_t kelvin;

    // Get temp bits from ADC_OUT registers
    ut = bmp_ps_read_register(BMP_085_ADC_OUT_MSB_REG, PS_I2C_16BIT_ACCESS);

    // Add Compensation and convert decimal value to 0.1 �C
    x1 = (((long) ut - (long) bmp_cal_param.ac6) * (long) bmp_cal_param.ac5) / 32768;
    x2 = ((long) bmp_cal_param.mc * 2048) / (x1 + bmp_cal_param.md);
    bmp_param_b5 = x1 + x2;

    temperature = ((bmp_param_b5 + 8) / 16);  // temperature in 0.1�C

    // Convert from �C to K
    if (temperature < 0)
        kelvin = 2732 - temperature;
    else
        kelvin = temperature + 2732;

    return (kelvin);
}

// **********************************************************************
// @fn          bmp_handle_interrupt
// @brief       handle last interrupt
// @param       none
// @return      1=new ps value 0=no ps value
// **********************************************************************
uint8_t bmp_ps_handle_interrupt(void)
{
	switch (bmp_ps_last_command) {
	case BMP_085_P_MEASURE:
		PS_INT_IE  &= ~PS_INT_PIN;
		PS_INT_IFG &= ~PS_INT_PIN;
		bmp_ps_read_pressure();
		bmp_ps_last_command = 0;
		return 1;
		break;
	case BMP_085_T_MEASURE:
		bmp_ps_read_temp();
		bmp_ps_last_command = BMP_085_P_MEASURE;
		bmp_ps_write_register(BMP_085_CTRL_MEAS_REG, BMP_085_P_MEASURE);
		return 0;
		break;
	}
	// should not happen
	bmp_ps_err++;
	return 0;
}
