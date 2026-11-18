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

/*-----------------------------------------------------------------------------------------*/

#ifndef THERMOCOUPLE_TYPED_H_
#define THERMOCOUPLE_TYPED_H_

/************************************************************************************
 *
 *	@brief Type D Thermocouple Coefficients
 *	
 *	Temperature Range (°C)
 *	cold: 0 to 783
 *	warm: 783 to 2320
 *
 * 	Equation: E = SUM[i=0 to n] { c(i) * t^(i) } + a0 * exp(a1 (t - a2)^2)
 *
 ************************************************************************************
 */
const float typeD_T2mV_cold[] = {
         0.0000000e+00,
         9.5685256e-03,
         2.0592621e-05,
        -1.8464573e-08,
         7.9498033e-12,
        -1.4240735e-15
};

const float typeD_T2mV_warm[] = {
         0.0000000e+00,
         9.9109462e-03,
         1.8666488e-05,
        -1.4935266e-08,
         5.3743821e-12,
        -7.9026726e-16,
};

TC_TempRanges typeD_T2mV_tempRanges[] = {
	{ 
		.coeffs    = typeD_T2mV_cold,
		.min       = 0,  
		.max       = 783.0f,
		.numCoeffs = sizeof(typeC_T2mV_cold)/sizeof(typeC_T2mV_cold[0])
	},
	{
		.coeffs    = typeD_T2mV_warm,
		.min       = 783.0f,
		.max       = 2320.0f,
		.numCoeffs = sizeof(typeD_T2mV_warm)/sizeof(typeD_T2mV_warm[0])
	}
};

thermoPolyCoefficientSet typeD_T2mV = {
	.type          = D,
	.numTempRanges = sizeof(typeD_T2mV_tempRanges)/sizeof(typeD_T2mV_tempRanges[0]),
	.tempRanges    = typeD_T2mV_tempRanges
};

/*-----------------------------------------------------------------------------------------*/

#endif // THERMOCOUPLE_TYPED_H_