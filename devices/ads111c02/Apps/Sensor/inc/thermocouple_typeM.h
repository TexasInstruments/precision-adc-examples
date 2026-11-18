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

#ifndef THERMOCOUPLE_TYPEM_H_
#define THERMOCOUPLE_TYPEM_H_


/************************************************************************************
 *
 *	@brief Type M Thermocouple Coefficients
 *	
 *	Temperature Range (°C)
 *	cold: -50 to 370.8
 *	warm:  370.8 to 1410
 *
 * 	Equation: E = SUM[i=0 to n] { c(i) * t^(i) } + a0 * exp(a1 (t - a2)^2)
 *
 ************************************************************************************
 */
const float typeM_T2mV_cold[] = {
         0.000000000e+00,
         3.690092195e-02,
         4.408522682e-05,
        -3.142898226e-08,
        -1.025216130e-10,
         1.846977453e-13,
        -9.738054601e-17,
        -3.394387900e-19,
};

const float typeM_mV2T_warm[] = {
        -1.145582129e+01,
         2.059913943e-01,
        -8.846963426e-04,
         2.650568429e-06,
        -4.958763813e-09,
         6.145877457e-12,
        -5.041679909e-15,
         2.627522669e-18,
        -7.864442961e-22,
         1.027600874e-25,
};

TC_TempRanges typeM_T2mV_tempRanges[] = {
	{ 
		.coeffs    = typeM_T2mV_cold,
		.min       = 0,  
		.max       = 746.6f,
		.numCoeffs = sizeof(typeM_T2mV_cold)/sizeof(typeM_T2mV_cold[0])
	},
	{
		.coeffs    = typeM_mV2T_warm,
		.min       = 746.6f,
		.max       = 1395.0f,
		.numCoeffs = sizeof(typeM_mV2T_warm)/sizeof(typeM_mV2T_warm[0])
	}
};

thermoPolyCoefficientSet typeM_T2mV = {
	.type          = M,
	.numTempRanges = sizeof(typeM_T2mV_tempRanges)/sizeof(typeM_T2mV_tempRanges[0]),
	.tempRanges    = typeM_T2mV_tempRanges
};

/*-----------------------------------------------------------------------------------------*/


#endif THERMOCOUPLE_TYPEM_H_
