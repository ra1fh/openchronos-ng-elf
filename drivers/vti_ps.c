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


// *************************************************************************************************
// Include section

// system
#include "openchronos.h"

// driver
#include "vti_ps.h"
#include "timer.h"

// *************************************************************************************************
// Prototypes section
uint16_t ps_read_register(uint8_t address, uint8_t mode);
uint8_t ps_write_register(uint8_t address, uint8_t data);
uint8_t ps_twi_read(uint8_t ack);
void twi_delay(void);

// *************************************************************************************************
// Defines section


// *************************************************************************************************
// Global Variable section


// VTI pressure (hPa) to altitude (m) conversion tables
const int16_t h0[17] =
    { -153, 0, 111, 540, 989, 1457, 1949, 2466, 3012, 3591, 4206, 4865, 5574, 6344, 7185, 8117, 9164 };
const uint16_t p0[17] =
    { 1031, 1013, 1000, 950, 900, 850, 800, 750, 700, 650, 600, 550, 500, 450, 400, 350, 300 };
float p[17];

// Global flag for proper pressure sensor operation
uint8_t ps_ok;


// *************************************************************************************************
// Extern section


// *************************************************************************************************
// @fn          ps_init
// @brief       Init pressure sensor I/O
// @param       none
// @return      none
// *************************************************************************************************
void ps_init(void)
{
    volatile uint8_t status, eeprom;
    __attribute__((unused)) volatile uint8_t success;

    PS_INT_DIR &= ~PS_INT_PIN;              // DRDY is input
    PS_INT_IES &= ~PS_INT_PIN;              // Interrupt on DRDY rising edge
    PS_TWI_OUT |= PS_SCL_PIN + PS_SDA_PIN;  // SCL and SDA are outputs by default
    PS_TWI_DIR |= PS_SCL_PIN + PS_SDA_PIN;  // SCL and SDA are outputs by default

    // Reset global ps_ok flag
    ps_ok = 0;

    // 100msec delay to allow VDD stabilisation
    timer0_delay(100, LPM3_bits);

    // Reset pressure sensor -> powerdown sensor
    success = ps_write_register(0x06, 0x01);

    // 100msec delay
    timer0_delay(100, LPM3_bits);

    // Check if STATUS register BIT0 is cleared
    status = ps_read_register(0x07, PS_TWI_8BIT_ACCESS);

    if (((status & BIT0) == 0) && (status != 0)) {
        // Check EEPROM checksum in DATARD8 register
        eeprom = ps_read_register(0x7F, PS_TWI_8BIT_ACCESS);

        if (eeprom == 0x01) ps_ok = 1;
        else                ps_ok = 0;
    }
}


// *************************************************************************************************
// @fn          ps_start
// @brief       Init pressure sensor registers and start sampling
// @param       none
// @return      uint8_t     1=Sensor started, 0=Sensor did not start
// *************************************************************************************************
void ps_start(void)
{
	// Enable DRDY IRQ on rising edge
	PS_INT_IFG &= ~PS_INT_PIN;
	PS_INT_IE |= PS_INT_PIN;

    // Start sampling data in ultra low power mode
    ps_write_register(0x03, 0x0B);

	// 200ms needed to have a working interrupt
	timer0_delay(200, LPM3_bits);
}



// *************************************************************************************************
// @fn          ps_stop
// @brief       Power down pressure sensor
// @param       none
// @return      none
// *************************************************************************************************
void ps_stop(void)
{
 	// Disable DRDY IRQ
	PS_INT_IE  &= ~PS_INT_PIN;
	PS_INT_IFG &= ~PS_INT_PIN;
	
	// Put sensor to standby
    ps_write_register(0x03, 0x00);

	// 200ms needed ? FIXME
	timer0_delay(200, LPM3_bits);
}



// *************************************************************************************************
// @fn          ps_twi_sda
// @brief       Control SDA line
// @param       uint8_t condition       PS_TWI_SEND_START, PS_TWI_SEND_RESTART, PS_TWI_SEND_STOP
//                                      PS_TWI_CHECK_ACK
// @return      uint8_t                 1=ACK, 0=NACK
// *************************************************************************************************
uint8_t ps_twi_sda(uint8_t condition)
{
    uint8_t sda = 0;

    if (condition == PS_TWI_SEND_START) {
        PS_TWI_SDA_OUT;         // SDA is output
        PS_TWI_SCL_HI;
        twi_delay();
        PS_TWI_SDA_LO;
        twi_delay();
        PS_TWI_SCL_LO;          // SCL 1-0 transition while SDA=0
        twi_delay();
    } else if (condition == PS_TWI_SEND_RESTART) {
        PS_TWI_SDA_OUT;         // SDA is output
        PS_TWI_SCL_LO;
        PS_TWI_SDA_HI;
        twi_delay();
        PS_TWI_SCL_HI;
        twi_delay();
        PS_TWI_SDA_LO;
        twi_delay();
        PS_TWI_SCL_LO;
        twi_delay();
    } else if (condition == PS_TWI_SEND_STOP) {
        PS_TWI_SDA_OUT;         // SDA is output
        PS_TWI_SDA_LO;
        twi_delay();
        PS_TWI_SCL_LO;
        twi_delay();
        PS_TWI_SCL_HI;
        twi_delay();
        PS_TWI_SDA_HI;          // SDA 0-1 transition while SCL=1
        twi_delay();
    } else if (condition == PS_TWI_CHECK_ACK) {
        PS_TWI_SDA_IN;          // SDA is input
        PS_TWI_SCL_LO;
        twi_delay();
        PS_TWI_SCL_HI;
        twi_delay();
        sda = PS_TWI_IN & PS_SDA_PIN;
        PS_TWI_SCL_LO;
    }

    // Return value will only be evaluated when checking device ACK
    return (sda == 0);
}



// *************************************************************************************************
// @fn          twi_delay
// @brief       Delay between TWI signal edges.
// @param       none
// @return      none
// *************************************************************************************************
void twi_delay(void)
{
    asm("   nop");
}


// *************************************************************************************************
// @fn          ps_twi_write
// @brief       Clock out bits through SDA.
// @param       uint8_t data        Byte to send
// @return      none
// *************************************************************************************************
void ps_twi_write(uint8_t data)
{
    uint8_t i, mask;

    // Set mask byte to 10000000b
    mask = BIT0 << 7;

    PS_TWI_SDA_OUT;     // SDA is output

    for (i = 8; i > 0; i--) {
        PS_TWI_SCL_LO;  // SCL=0

        if ((data & mask) == mask) {
            PS_TWI_SDA_HI; // SDA=1
        } else {
            PS_TWI_SDA_LO; // SDA=0
        }

        mask = mask >> 1;
        twi_delay();
        PS_TWI_SCL_HI;  // SCL=1
        twi_delay();
    }

    PS_TWI_SCL_LO;      // SCL=0
    PS_TWI_SDA_IN;      // SDA is input
}


// *************************************************************************************************
// @fn          ps_twi_read
// @brief       Read bits from SDA
// @param       uint8_t ack     1=Send ACK after read, 0=Send NACK after read
// @return      uint8_t         Bits read
// *************************************************************************************************
uint8_t ps_twi_read(uint8_t ack)
{
    uint8_t i;
    uint8_t data = 0;

    PS_TWI_SDA_IN;      // SDA is input

    for (i = 0; i < 8; i++) {
        PS_TWI_SCL_LO;          // SCL=0
        twi_delay();
        PS_TWI_SCL_HI;          // SCL=0
        twi_delay();

        // Shift captured bits to left
        data = data << 1;

        // Capture new bit
        if ((PS_TWI_IN & PS_SDA_PIN) == PS_SDA_PIN) data |= BIT0;
    }

    PS_TWI_SDA_OUT;         // SDA is output

    // 1 aditional clock phase to generate master ACK
    PS_TWI_SCL_LO;          // SCL=0

    if (ack == 1)   PS_TWI_SDA_LO       // Send ack -> continue read
        else            PS_TWI_SDA_HI       // Send nack -> stop read
            twi_delay();

    PS_TWI_SCL_HI;          // SCL=0
    twi_delay();
    PS_TWI_SCL_LO;

    return (data);
}



// *************************************************************************************************
// @fn          as_write_register
// @brief       Write a byte to the pressure sensor
// @param       uint8_t address         Register address
//              uint8_t data            Data to write
// @return      uint8_t
// *************************************************************************************************
uint8_t ps_write_register(uint8_t address, uint8_t data)
{
    volatile uint8_t success;

    ps_twi_sda(PS_TWI_SEND_START);          // Generate start condition

    ps_twi_write((0x11 << 1) | PS_TWI_WRITE);   // Send 7bit device address 0x11 + write bit '0'
    success = ps_twi_sda(PS_TWI_CHECK_ACK); // Check ACK from device

    if (!success) return (0);

    ps_twi_write(address);                  // Send 8bit register address
    success = ps_twi_sda(PS_TWI_CHECK_ACK); // Check ACK from device

    if (!success) return (0);

    ps_twi_write(data);                     // Send 8bit data to register
    success = ps_twi_sda(PS_TWI_CHECK_ACK); // Check ACK from device
// Slave does not send this ACK
// if (!success) return (0);

    ps_twi_sda(PS_TWI_SEND_STOP);               // Generate stop condition

    return (1);
}


// *************************************************************************************************
// @fn          ps_read_register
// @brief       Read a byte from the pressure sensor
// @param       uint8_t address     Register address
//              uint8_t mode        PS_TWI_8BIT_ACCESS, PS_TWI_16BIT_ACCESS
// @return      uint16_t            Register content
// *************************************************************************************************
uint16_t ps_read_register(uint8_t address, uint8_t mode)
{
    uint8_t success;
    uint16_t data = 0;

    ps_twi_sda(PS_TWI_SEND_START);          // Generate start condition

    ps_twi_write((0x11 << 1) | PS_TWI_WRITE);   // Send 7bit device address 0x11 + write bit '0'
    success = ps_twi_sda(PS_TWI_CHECK_ACK); // Check ACK from device

    if (!success) return (0);

    ps_twi_write(address);                  // Send 8bit register address
    success = ps_twi_sda(PS_TWI_CHECK_ACK); // Check ACK from device

    if (!success) return (0);

    ps_twi_sda(PS_TWI_SEND_RESTART);            // Generate restart condition

    ps_twi_write((0x11 << 1) | PS_TWI_READ);    // Send 7bit device address 0x11 + read bit '1'
    success = ps_twi_sda(PS_TWI_CHECK_ACK); // Check ACK from device

    if (!success) return (0);

    if (mode == PS_TWI_16BIT_ACCESS) {
        data =  ps_twi_read(1) << 8;            // Read MSB 8bit data from register
        data |= ps_twi_read(0);             // Read LSB 8bit data from register
    } else {
        data = ps_twi_read(0);              // Read 8bit data from register
    }

    ps_twi_sda(PS_TWI_SEND_STOP);               // Generate stop condition

    return (data);
}



// *************************************************************************************************
// @fn          ps_get_pa
// @brief       Read out pressure. Format is Pa. Range is 30000 .. 120000 Pa.
// @param       none
// @return      uint32_t        15-bit pressure sensor value (Pa)
// *************************************************************************************************
uint32_t ps_get_pa(void)
{
    uint32_t data = 0;

    // Get 3 MSB from DATARD8 register
    data = ps_read_register(0x7F, PS_TWI_8BIT_ACCESS);
    data = ((data & 0x07) << 8) << 8;

    // Get 16 LSB from DATARD16 register
    data |= ps_read_register(0x80, PS_TWI_16BIT_ACCESS);

    // Convert decimal value to Pa
    data = (data >> 2);

    return (data);
}


// *************************************************************************************************
// @fn          ps_get_temp
// @brief       Read out temperature.
// @param       none
// @return      uint16_t        13-bit temperature value in xx.x�K format
// *************************************************************************************************
uint16_t ps_get_temp(void)
{
    volatile uint16_t data = 0;
    uint16_t temp = 0;
    uint8_t is_negative = 0;
    uint16_t kelvin;

    // Get 13 bit from TEMPOUT register
    data = ps_read_register(0x81, PS_TWI_16BIT_ACCESS);

    // Convert negative temperatures
    if ((data >> 13) & 0x1) {
        // Sign extend temperature
        data |= 0xC000;
        // Convert two's complement
        data = ~data;
        data += 1;
        is_negative = 1;
    }

    temp = data / 2;

    // Convert from �C to �K
    if (is_negative)    kelvin = 2732 - temp;
    else                kelvin = temp + 2732;

    return (kelvin);
}

// *************************************************************************************************
// @fn          init_pressure_table
// @brief       Init pressure table with constants
// @param       uint32_t        p       Pressure (Pa)
// @return      uint16_t                Altitude (m)
// *************************************************************************************************
void init_pressure_table(void)
{
    uint8_t i;

    for (i = 0; i < 17; i++)
		p[i] = p0[i];
}

// *************************************************************************************************
// @fn          update_pressure_table
// @brief       Calculate pressure table for reference altitude.
//              Implemented straight from VTI reference code.
// @param       int16_t     href    Reference height
//              uint32_t        p_meas  Pressure (Pa)
//              uint16_t        t_meas  Temperature (10*�K)
// @return      none
// *************************************************************************************************
void update_pressure_table(int16_t href, uint32_t p_meas, uint16_t t_meas)
{
    const float Invt00 = 0.003470415;
    const float coefp  = 0.00006;
    volatile float p_fact;
    volatile float p_noll;
    volatile float hnoll;
    volatile float h_low = 0;
    volatile float t0;
    uint8_t i;

    // Typecast arguments
    volatile float fl_href      = href;
    volatile float fl_p_meas    = (float)p_meas / 100;  // Convert from Pa to hPa
    volatile float fl_t_meas    = (float)t_meas / 10;       // Convert from 10�K to 1�K

    t0 = fl_t_meas + (0.0065 * fl_href);

    hnoll  = fl_href / (t0 * Invt00);

    for (i = 0; i <= 15; i++) {
        if (h0[i] > hnoll) break;

        h_low = h0[i];
    }

    p_noll = (float)(hnoll - h_low) *
		(1 - (hnoll - (float)h0[i]) * coefp) *
		((float)p0[i] - (float)p0[i - 1]) /
		((float)h0[i] - h_low) +
		(float)p0[i - 1];

    // Calculate multiplicator
    p_fact = fl_p_meas / p_noll;

    // Apply correction factor to pressure table
    for (i = 0; i <= 16; i++) {
        p[i] = p0[i] * p_fact;
    }
}

// *************************************************************************************************
// @fn          conv_pa_to_meter
// @brief       Convert pressure (Pa) to altitude (m) using a conversion table
//              Implemented straight from VTI reference code.
// @param       uint32_t        p_meas  Pressure (Pa)
//              uint16_t        t_meas  Temperature (10*�K)
// @return      int16_t             Altitude (m)
// *************************************************************************************************
int16_t conv_pa_to_meter(uint32_t p_meas, uint16_t t_meas)
{
    const float coef2  = 0.0007;
    const float Invt00 = 0.003470415;
    volatile float hnoll;
    volatile float t0;
    volatile float p_low;
    volatile float fl_h;
    volatile int16_t h;
    uint8_t i;

    // Typecast arguments
    volatile float fl_p_meas = (float)p_meas / 100; // Convert from Pa to hPa
    volatile float fl_t_meas = (float)t_meas / 10;      // Convert from 10�K to 1�K

    for (i = 0; i <= 16; i++) {
        if (p[i] < fl_p_meas) break;

        p_low = p[i];
    }

    if (i == 0) {
        hnoll = (float)(fl_p_meas - p[0]) /
			(p[1] - p[0]) *
			((float)(h0[1] - h0[0]));
    } else if (i < 15) {
        hnoll = (float)(fl_p_meas - p_low) *
			(1 - (fl_p_meas - p[i]) * coef2) /
			(p[i] - p_low) *
			((float)(h0[i] - h0[i - 1])) +
			h0[i - 1];
    } else if (i == 15) {
        hnoll = (float)(fl_p_meas - p_low) /
			(p[i] - p_low) *
			((float)(h0[i] - h0[i - 1])) +
			h0[i - 1];
    } else { // i==16
        hnoll = (float)(fl_p_meas - p[16]) /
			(p[16] - p[15]) *
			((float)(h0[16] - h0[15])) +
			h0[16];
    }

    // Compensate temperature error
    t0 = fl_t_meas / (1 - hnoll * Invt00 * 0.0065);
    fl_h = Invt00 * t0 * hnoll;
    h = (uint16_t)fl_h;

    return (h);
}
