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

#ifndef THERMOCOUPLE_TYPEJ_H_
#define THERMOCOUPLE_TYPEJ_H_


/************************************************************************************
 *
 *  @brief Type J Thermocouple Coefficients
 *  
 *  Temperature Range (°C)
 *  cold: -210.000 to  760.000
 *  warm:  760.000 to 1200.000
 *
 *  Equation: E (mV) = SUM[i=0 to n] { c(i) * t^(i) }
 *
 ************************************************************************************
 */
const float typeJ_T2mV_cold[] = {
	 0.000000000000e+00,
	 5.038118781500e-02,
	 3.047583693000e-05,
	-8.568106572000e-08,
	 1.322819529500e-10,
	-1.705295833700e-13,
	 2.094809069700e-16,
	-1.253839533600e-19,
	 1.563172569700e-23,
};

const float typeJ_T2mV_warm[] = {
	 2.964562568100e+02,
	-1.497612778600e+00,
	 3.178710392400e-03,
	-3.184768670100e-06,
	 1.572081900400e-09,
	-3.069136905600e-13,
};

TC_TempRanges typeJ_T2mV_tempRanges[] = {
	{
		.coeffs    = typeJ_T2mV_cold,
		.min       =  -210.000,
		.max       =   760.000,
		.numCoeffs = sizeof(typeJ_T2mV_cold)/sizeof(typeJ_T2mV_cold[0])
	},
	{
		.coeffs    = typeJ_T2mV_warm,
		.min       =   760.000,
		.max       =  1200.000,
		.numCoeffs = sizeof(typeJ_T2mV_warm)/sizeof(typeJ_T2mV_warm[0])
	},
};

thermoPolyCoefficientSet typeJ_T2mV = {
		.type          = J,
		.numTempRanges = sizeof(typeJ_T2mV_tempRanges)/sizeof(typeJ_T2mV_tempRanges[0]),
		.tempRanges    = typeJ_T2mV_tempRanges
};

/************************************************************************************
 *
 *  @brief Type J Thermocouple Inverse Coefficients
 *  
 *         Voltage Range (mV)
 *  cold:   -8.095 to    0.000
 *  warm:    0.000 to   42.919
 *   hot:   42.919 to   69.553
 *
 *  Equation: T = d(0) + d(1)*E + d(2)*E^2 + ... + d(n)*E^n
 *
 ************************************************************************************
 */
const float typeJ_mV2T_cold[] = {
	 0.000000000000e+00,
	 1.952826800000e+01,
	-1.228618500000e+00,
	-1.075217800000e+00,
	-5.908693300000e-01,
	-1.725671300000e-01,
	-2.813151300000e-02,
	-2.396337000000e-03,
	-8.382332100000e-05,
};

const float typeJ_mV2T_warm[] = {
	 0.000000000000e+00,
	 1.978425000000e+01,
	-2.001204000000e-01,
	 1.036969000000e-02,
	-2.549687000000e-04,
	 3.585153000000e-06,
	-5.344285000000e-08,
	 5.099890000000e-10,
};

const float typeJ_mV2T_hot[] = {
	-3.113581870000e+03,
	 3.005436840000e+02,
	-9.947732300000e+00,
	 1.702766300000e-01,
	-1.430334680000e-03,
	 4.738860840000e-06,
};

TC_TempRanges typeJ_mV2T_tempRanges[] = {
	{
	.coeffs    = typeJ_mV2T_cold,
		.min       = -8.095e+00,
		.max       =  0.000e+00,
		.numCoeffs = sizeof(typeJ_mV2T_cold)/sizeof(typeJ_mV2T_cold[0])
	},
	{
	.coeffs    = typeJ_mV2T_warm,
		.min       =  0.000e+00,
		.max       =  4.292e+01,
		.numCoeffs = sizeof(typeJ_mV2T_warm)/sizeof(typeJ_mV2T_warm[0])
	},
	{
	.coeffs    = typeJ_mV2T_hot,
		.min       =  4.292e+01,
		.max       =  6.955e+01,
		.numCoeffs = sizeof(typeJ_mV2T_hot)/sizeof(typeJ_mV2T_hot[0])
	},
};

thermoPolyCoefficientSet typeJ_mV2T = {
		.type          = J,
		.numTempRanges = sizeof(typeJ_mV2T_tempRanges)/sizeof(typeJ_mV2T_tempRanges[0]),
		.tempRanges    = typeJ_mV2T_tempRanges
};

/*-----------------------------------------------------------------------------------------*/

#endif //THERMOCOUPLE_TYPEJ_H_


