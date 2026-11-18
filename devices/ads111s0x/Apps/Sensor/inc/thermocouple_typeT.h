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

#ifndef THERMOCOUPLE_TYPET_H_
#define THERMOCOUPLE_TYPET_H_

/************************************************************************************
 *
 *  @brief Type T Thermocouple Coefficients
 *  
 *  Temperature Range (°C)
 *  cold: -270.000 to    0.000
 *  warm:    0.000 to  400.000
 *
 *  Equation: E (mV) = SUM[i=0 to n] { c(i) * t^(i) }
 *
 ************************************************************************************
 */
const float typeT_T2mV_cold[] = {
	 0.000000000000e+00,
	 3.874810636400e-02,
	 4.419443434700e-05,
	 1.184432310500e-07,
	 2.003297355400e-08,
	 9.013801955900e-10,
	 2.265115659300e-11,
	 3.607115420500e-13,
	 3.849393988300e-15,
	 2.821352192500e-17,
	 1.425159477900e-19,
	 4.876866228600e-22,
	 1.079553927000e-24,
	 1.394502706200e-27,
	 7.979515392700e-31,
};

const float typeT_T2mV_warm[] = {
	 0.000000000000e+00,
	 3.874810636400e-02,
	 3.329222788000e-05,
	 2.061824340400e-07,
	-2.188225684600e-09,
	 1.099688092800e-11,
	-3.081575877200e-14,
	 4.547913529000e-17,
	-2.751290167300e-20,
};

TC_TempRanges typeT_T2mV_tempRanges[] = {
	{
		.coeffs    = typeT_T2mV_cold,
		.min       =  -270.000,
		.max       =     0.000,
		.numCoeffs = sizeof(typeT_T2mV_cold)/sizeof(typeT_T2mV_cold[0])
	},
	{
		.coeffs    = typeT_T2mV_warm,
		.min       =     0.000,
		.max       =   400.000,
		.numCoeffs = sizeof(typeT_T2mV_warm)/sizeof(typeT_T2mV_warm[0])
	},
};

thermoPolyCoefficientSet typeT_T2mV = {
		.type          = T,
		.numTempRanges = sizeof(typeT_T2mV_tempRanges)/sizeof(typeT_T2mV_tempRanges[0]),
		.tempRanges    = typeT_T2mV_tempRanges
};

/************************************************************************************
 *
 *  @brief Type T Thermocouple Inverse Coefficients
 *  
 *         Voltage Range (mV)
 *  cold:   -5.603 to    0.000
 *  warm:    0.000 to   20.872
 *
 *  Equation: T = d(0) + d(1)*E + d(2)*E^2 + ... + d(n)*E^n
 *
 ************************************************************************************
 */
const float typeT_mV2T_cold[] = {
	 0.000000000000e+00,
	 2.594919200000e+01,
	-2.131696700000e-01,
	 7.901869200000e-01,
	 4.252777700000e-01,
	 1.330447300000e-01,
	 2.024144600000e-02,
	 1.266817100000e-03,
};

const float typeT_mV2T_warm[] = {
	 0.000000000000e+00,
	 2.592800000000e+01,
	-7.602961000000e-01,
	 4.637791000000e-02,
	-2.165394000000e-03,
	 6.048144000000e-05,
	-7.293422000000e-07,
};

TC_TempRanges typeT_mV2T_tempRanges[] = {
	{
	.coeffs    = typeT_mV2T_cold,
		.min       = -5.603e+00,
		.max       =  0.000e+00,
		.numCoeffs = sizeof(typeT_mV2T_cold)/sizeof(typeT_mV2T_cold[0])
	},
	{
	.coeffs    = typeT_mV2T_warm,
		.min       =  0.000e+00,
		.max       =  2.087e+01,
		.numCoeffs = sizeof(typeT_mV2T_warm)/sizeof(typeT_mV2T_warm[0])
	},
};

thermoPolyCoefficientSet typeT_mV2T = {
		.type          = T,
		.numTempRanges = sizeof(typeT_mV2T_tempRanges)/sizeof(typeT_mV2T_tempRanges[0]),
		.tempRanges    = typeT_mV2T_tempRanges
};

/*-----------------------------------------------------------------------------------------*/

#endif //THERMOCOUPLE_TYPET_H_