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

#ifndef THERMOCOUPLE_TYPEG_H_
#define THERMOCOUPLE_TYPEG_H_

// ASTM E 1751-00
/*-----------------------------------------------------------------------------------------*/
/************************************************************************************
 *
 *	@brief Type G Thermocouple Coefficients
 *	
 *	Temperature Range (°C)
 *	cold: 0 to 630.615
 *	warm:  630.615 to 2315
 *
 * 	Equation: E = SUM[i=0 to n] { c(i) * t^(i) } + a0 * exp(a1 (t - a2)^2)
 *
 ************************************************************************************
 */
const float typeG_T2mV_cold[] = {
         0.0000000e+00,
         1.2792201e-03,
         2.1634754e-05,
        -1.1393234e-08,
         4.3850022e-12,
        -1.7089202e-15
};

const float typeG_mV2T_warm[] = {
        -1.1064412e+00,
         9.4962455e-03,
        -3.6467516e-06,
         3.1141330e-08,
        -3.8615222e-11,
         2.4455012e-14,
        -8.9888053e-18,
         1.8120237e-21,
        -1.5534591e-25,
};

TC_TempRanges typeG_T2mV_tempRanges[] = {
	{ 
		.coeffs    = typeG_T2mV_cold,
		.min       = 0,  
		.max       = 630.615f,
		.numCoeffs = sizeof(typeG_T2mV_cold)/sizeof(typeG_T2mV_cold[0])
	},
	{
		.coeffs    = typeG_mV2T_warm,
		.min       = 630.615f,
		.max       = 2315.0f,
		.numCoeffs = sizeof(typeG_mV2T_warm)/sizeof(typeG_mV2T_warm[0])
	}
};

thermoPolyCoefficientSet typeG_T2mV = {
	.type          = G,
	.numTempRanges = sizeof(typeG_T2mV_tempRanges)/sizeof(typeG_T2mV_tempRanges[0]),
	.tempRanges    = typeG_T2mV_tempRanges
};

/*-----------------------------------------------------------------------------------------*/


#endif //THERMOCOUPLE_TYPEG_H_
