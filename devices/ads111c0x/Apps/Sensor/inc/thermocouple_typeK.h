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

#ifndef THERMOCOUPLE_TYPEK_H_
#define THERMOCOUPLE_TYPEK_H_

/************************************************************************************
 *
 *  @brief Type K Thermocouple Coefficients
 *  
 *  Temperature Range (°C)
 *  cold: -270.000 to    0.000
 *  warm:    0.000 to 1372.000
 *
 *  Equation: E (mV) = SUM[i=0 to n] { c(i) * t^(i) }  + a0 * exp(a1 (t - a2)^2)
 *
 ************************************************************************************
 */
const float typeK_T2mV_cold[] = {
	 0.000000000000e+00,
	 3.945012802500e-02,
	 2.362237359800e-05,
	-3.285890678400e-07,
	-4.990482877700e-09,
	-6.750905917300e-11,
	-5.741032742800e-13,
	-3.108887289400e-15,
	-1.045160936500e-17,
	-1.988926687800e-20,
	-1.632269748600e-23,
};

const float typeK_T2mV_warm[] = {
	-1.760041368600e-02,
	 3.892120497500e-02,
	 1.855877003200e-05,
	-9.945759287400e-08,
	 3.184094571900e-10,
	-5.607284488900e-13,
	 5.607505905900e-16,
	-3.202072000300e-19,
	 9.715114715200e-23,
	-1.210472127500e-26,
};

const float typeK_T2mV_alpha[] = {
	 1.185976000000e-01,
	-1.183432000000e-04,
	 1.269686000000e+02,
};

TC_TempRanges typeK_T2mV_tempRanges[] = {
	{
		.coeffs    = typeK_T2mV_cold,
		.min       =  -270.000,
		.max       =     0.000,
		.numCoeffs = sizeof(typeK_T2mV_cold)/sizeof(typeK_T2mV_cold[0])
	},
	{
		.coeffs    = typeK_T2mV_warm,
		.min       =     0.000,
		.max       =  1372.000,
		.numCoeffs = sizeof(typeK_T2mV_warm)/sizeof(typeK_T2mV_warm[0])
	},
};

thermoPolyCoefficientSet typeK_T2mV = {
		.type          = K,
		.numTempRanges = sizeof(typeK_T2mV_tempRanges)/sizeof(typeK_T2mV_tempRanges[0]),
		.tempRanges    = typeK_T2mV_tempRanges
};

/************************************************************************************
 *
 *  @brief Type K Thermocouple Inverse Coefficients
 *  
 *         Voltage Range (mV)
 *  cold:   -5.891 to    0.000
 *  warm:    0.000 to   20.644
 *   hot:   20.644 to   54.886
 *
 *  Equation: T = d(0) + d(1)*E + d(2)*E^2 + ... + d(n)*E^n
 *
 ************************************************************************************
 */
const float typeK_mV2T_cold[] = {
	 0.000000000000e+00,
	 2.517346200000e+01,
	-1.166287800000e+00,
	-1.083363800000e+00,
	-8.977354000000e-01,
	-3.734237700000e-01,
	-8.663264300000e-02,
	-1.045059800000e-02,
	-5.192057700000e-04,
};

const float typeK_mV2T_warm[] = {
	 0.000000000000e+00,
	 2.508355000000e+01,
	 7.860106000000e-02,
	-2.503131000000e-01,
	 8.315270000000e-02,
	-1.228034000000e-02,
	 9.804036000000e-04,
	-4.413030000000e-05,
	 1.057734000000e-06,
	-1.052755000000e-08,
};

const float typeK_mV2T_hot[] = {
	-1.318058000000e+02,
	 4.830222000000e+01,
	-1.646031000000e+00,
	 5.464731000000e-02,
	-9.650715000000e-04,
	 8.802193000000e-06,
	-3.110810000000e-08,
};

TC_TempRanges typeK_mV2T_tempRanges[] = {
	{
	.coeffs    = typeK_mV2T_cold,
		.min       = -5.891e+00,
		.max       =  0.000e+00,
		.numCoeffs = sizeof(typeK_mV2T_cold)/sizeof(typeK_mV2T_cold[0])
	},
	{
	.coeffs    = typeK_mV2T_warm,
		.min       =  0.000e+00,
		.max       =  2.064e+01,
		.numCoeffs = sizeof(typeK_mV2T_warm)/sizeof(typeK_mV2T_warm[0])
	},
	{
	.coeffs    = typeK_mV2T_hot,
		.min       =  2.064e+01,
		.max       =  5.489e+01,
		.numCoeffs = sizeof(typeK_mV2T_hot)/sizeof(typeK_mV2T_hot[0])
	},
};

thermoPolyCoefficientSet typeK_mV2T = {
		.type          = K,
		.numTempRanges = sizeof(typeK_mV2T_tempRanges)/sizeof(typeK_mV2T_tempRanges[0]),
		.tempRanges    = typeK_mV2T_tempRanges
};

/*-----------------------------------------------------------------------------------------*/

#endif //THERMOCOUPLE_TYPEK_H_