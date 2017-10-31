/**********************************************************************/
/** Math Functions from https://github.com/jackokring/miChronos.git

    (C)2012, 2013 Jacko edited to turn into a slide rule LINE2

    Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the
    distribution.

    Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include "openchronos.h"
#include "drivers/wdt.h"

#include "mathutils.h"

float square(float x)
{
	return x * x;
}

float inv(float x)
{
	x = irt(x);
	return square(x);
}

/* use initial estimate and y'=y*(3-x*y*y)/2 with iterations */
float irt(float x)
{
	// Watchdog triggers after 16 seconds when not cleared
	// So place here just in case any code uses intense calculaton.
#ifdef USE_WATCHDOG
	WDTCTL = WDTPW + WDTIS__512K + WDTSSEL__ACLK;
#else
	wdt_stop
#endif
	uint32_t i;
	float x2;
	const float threehalfs = 1.5F;
	x2 = x * 0.5F;
	i  = * ( long * ) &x;
	i  = 0x5f3759df - ( i >> 1 );
	x  = * ( float * ) &i;
	for(i = 0; i < 4; i++)
		x  *= ( threehalfs - ( x2 * square(x) ) );   //iteration
	return x;
}

//OSAF FN (flags and function produced)
//0000
//0001 expm1
//0010
//0011 expm1(ix)
//0100
//0101 sinh
//0110
//0111 sin
//1000
//1001 qfn
//1010
//1011
//1100 log with right input transform (is atanh)
//1101
//1110 atan
//1111

float eq(float x, int8_t over, int8_t sq, int8_t alt, int8_t fact) { //base e exponential and Q+
	float acc = 0;
	float lacc;
	float mul = x;
	float harm = 1;
	uint16_t start = 1;
	if(sq != 0) x = square(x);
	x = (alt != 0 ? -x : x);
	do {
		lacc = acc;
		acc += mul * (over == 0 ? 1.0F : harm);
		start += sq + 1;
		harm = inv((float)start);
		mul *= x * (fact == 0 ? 1.0F : harm * (sq == 0 ? 1.0F : inv(start - 1)));
        } while(lacc != acc && start < 200);//term limit
	return acc;
}

float ln(float x) { //base e
	x = irt(irt(irt(x)));//symetry and double triple roots
	return -eq((x-1.0F) * inv(x+1.0F), 1, 1, 0, 0) * 16.0F;
}

float ex(float x) {
	return eq(x, 0, 0, 0, 1) + 1.0F;
}
