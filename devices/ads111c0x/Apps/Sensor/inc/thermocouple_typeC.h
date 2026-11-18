/*
 * Copyright (c) 2026, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef THERMOCOUPLE_TYPEC_H_
#define THERMOCOUPLE_TYPEC_H_

// Omega.com
/*-----------------------------------------------------------------------------------------*/
/************************************************************************************
 *
 *	@brief Type C Thermocouple Coefficients
 *	
 *	Temperature Range (°C)
 *	cold: 0 to 2315
 *
 * 	Equation: E = SUM[i=0 to n] { c(i) * t^(i) } + a0 * exp(a1 (t - a2)^2)
 *
 ************************************************************************************
 */
const float typeC_T2mV_cold[] = {
         0.0000000000000000e+00,
         1.3387722982319094e-02,
         1.2252598548103214e-05,
        -1.0489145155399067e-08,
		 3.6006582486412798e-12,
		-4.9446064258560002e-16,
};

TC_TempRanges typeC_T2mV_tempRanges[] = {
	{ 
		.coeffs    = typeC_T2mV_cold,
		.min       = 0,  
		.max       = 2315.0f,
		.numCoeffs = sizeof(typeC_T2mV_cold)/sizeof(typeC_T2mV_cold[0])
	}
};

thermoPolyCoefficientSet typeC_T2mV = {
	.type          = C,
	.numTempRanges = sizeof(typeC_T2mV_tempRanges)/sizeof(typeC_T2mV_tempRanges[0]),
	.tempRanges    = typeC_T2mV_tempRanges
};

/*-----------------------------------------------------------------------------------------*/

#endif // THERMOCOUPLE_TYPEC_H_
