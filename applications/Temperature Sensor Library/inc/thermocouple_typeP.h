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

#ifndef THERMOCOUPLE_TYPEP_H_
#define THERMOCOUPLE_TYPEP_H_

/************************************************************************************
 *
 *	@brief Type P Thermocouple Coefficients
 *	
 *	Temperature Range (°C)
 *	cold: 0 to 746.6
 *	warm:  746.6 to 1395
 *
 * 	Equation: E = SUM[i=0 to n] { c(i) * t^(i) } + a0 * exp(a1 (t - a2)^2)
 *
 ************************************************************************************
 */
const float typeP_T2mV_cold[] = {
         0.0000000e+00,
         2.9819716e-02,
         3.5175152e-05,
        -3.4878428e-08,
         1.4851327e-11,
        -3.6375467e-15
};

const float typeP_T2mV_warm[] = {
        -8.9621838e+00,
         8.5377200e-02,
        -1.0570233e-04,
         1.5424937e-07,
        -1.2855115e-10,
         5.4438760e-14,
        -9.3211269e-18
};

TC_TempRanges typeP_T2mV_tempRanges[] = {
	{ 
		.coeffs    = typeP_T2mV_cold,
		.min       = 0,  
		.max       = 746.6f,
		.numCoeffs = sizeof(typeP_T2mV_cold)/sizeof(typeP_T2mV_cold[0])
	},
	{
		.coeffs    = typeP_T2mV_warm,
		.min       = 746.6f,
		.max       = 1395.0f,
		.numCoeffs = sizeof(typeP_T2mV_warm)/sizeof(typeP_T2mV_warm[0])
	}
};

thermoPolyCoefficientSet typeP_T2mV = {
	.type          = P,
	.numTempRanges = sizeof(typeP_T2mV_tempRanges)/sizeof(typeP_T2mV_tempRanges[0]),
	.tempRanges    = typeP_T2mV_tempRanges
};

/*-----------------------------------------------------------------------------------------*/

#endif // THERMOCOUPLE_TYPEP_H_