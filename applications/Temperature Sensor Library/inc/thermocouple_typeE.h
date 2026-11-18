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

#ifndef THERMOCOUPLE_TYPEE_H_
#define THERMOCOUPLE_TYPEE_H_


/************************************************************************************
 *
 *  @brief Type E Thermocouple Coefficients
 *  
 *  Temperature Range (°C)
 *  cold: -270.000 to    0.000
 *  warm:    0.000 to 1000.000
 *
 *  Equation: E (mV) = SUM[i=0 to n] { c(i) * t^(i) }
 *
 ************************************************************************************
 */
const float typeE_T2mV_cold[] = {
	 0.000000000000e+00,
	 5.866550870800e-02,
	 4.541097712400e-05,
	-7.799804868600e-07,
	-2.580016084300e-08,
	-5.945258305700e-10,
	-9.321405866700e-12,
	-1.028760553400e-13,
	-8.037012362100e-16,
	-4.397949739100e-18,
	-1.641477635500e-20,
	-3.967361951600e-23,
	-5.582732872100e-26,
	-3.465784201300e-29,
};

const float typeE_T2mV_warm[] = {
	 0.000000000000e+00,
	 5.866550871000e-02,
	 4.503227558200e-05,
	 2.890840721200e-08,
	-3.305689665200e-10,
	 6.502440327000e-13,
	-1.919749550400e-16,
	-1.253660049700e-18,
	 2.148921756900e-21,
	-1.438804178200e-24,
	 3.596089948100e-28,
};

TC_TempRanges typeE_T2mV_tempRanges[] = {
	{
		.coeffs    = typeE_T2mV_cold,
		.min       =  -270.000,
		.max       =     0.000,
		.numCoeffs = sizeof(typeE_T2mV_cold)/sizeof(typeE_T2mV_cold[0])
	},
	{
		.coeffs    = typeE_T2mV_warm,
		.min       =     0.000,
		.max       =  1000.000,
		.numCoeffs = sizeof(typeE_T2mV_warm)/sizeof(typeE_T2mV_warm[0])
	},
};

thermoPolyCoefficientSet typeE_T2mV = {
		.type          = E,
		.numTempRanges = sizeof(typeE_T2mV_tempRanges)/sizeof(typeE_T2mV_tempRanges[0]),
		.tempRanges    = typeE_T2mV_tempRanges
};

/************************************************************************************
 *
 *  @brief Type E Thermocouple Inverse Coefficients
 *  
 *         Voltage Range (mV)
 *  cold:   -8.825 to    0.000
 *  warm:    0.000 to   76.373
 *
 *  Equation: T = d(0) + d(1)*E + d(2)*E^2 + ... + d(n)*E^n
 *
 ************************************************************************************
 */
const float typeE_mV2T_cold[] = {
	 0.000000000000e+00,
	 1.697728800000e+01,
	-4.351497000000e-01,
	-1.585969700000e-01,
	-9.250287100000e-02,
	-2.608431400000e-02,
	-4.136019900000e-03,
	-3.403403000000e-04,
	-1.156489000000e-05,
};

const float typeE_mV2T_warm[] = {
	 0.000000000000e+00,
	 1.705703500000e+01,
	-2.330175900000e-01,
	 6.543558500000e-03,
	-7.356274900000e-05,
	-1.789600100000e-06,
	 8.403616500000e-08,
	-1.373587900000e-09,
	 1.062982300000e-11,
	-3.244708700000e-14,
};

TC_TempRanges typeE_mV2T_tempRanges[] = {
	{
	.coeffs    = typeE_mV2T_cold,
		.min       = -8.825e+00,
		.max       =  0.000e+00,
		.numCoeffs = sizeof(typeE_mV2T_cold)/sizeof(typeE_mV2T_cold[0])
	},
	{
	.coeffs    = typeE_mV2T_warm,
		.min       =  0.000e+00,
		.max       =  7.637e+01,
		.numCoeffs = sizeof(typeE_mV2T_warm)/sizeof(typeE_mV2T_warm[0])
	},
};

thermoPolyCoefficientSet typeE_mV2T = {
		.type          = E,
		.numTempRanges = sizeof(typeE_mV2T_tempRanges)/sizeof(typeE_mV2T_tempRanges[0]),
		.tempRanges    = typeE_mV2T_tempRanges
};


/*-----------------------------------------------------------------------------------------*/

#endif // THERMOCOUPLE_TYPEE_H_