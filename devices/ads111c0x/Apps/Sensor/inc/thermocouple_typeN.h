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

#ifndef THERMOCOUPLE_TYPEN_H_
#define THERMOCOUPLE_TYPEN_H_


/************************************************************************************
 *
 *  @brief Type N Thermocouple Coefficients
 *  
 *  Temperature Range (°C)
 *  cold: -270.000 to    0.000
 *  warm:    0.000 to 1300.000
 *
 *  Equation: E (mV) = SUM[i=0 to n] { c(i) * t^(i) }
 *
 ************************************************************************************
 */
const float typeN_T2mV_cold[] = {
	 0.000000000000e+00,
	 2.615910596200e-02,
	 1.095748422800e-05,
	-9.384111155400e-08,
	-4.641203975900e-11,
	-2.630335771600e-12,
	-2.265343800300e-14,
	-7.608930079100e-17,
	-9.341966783500e-20,
};

const float typeN_T2mV_warm[] = {
	 0.000000000000e+00,
	 2.592939460100e-02,
	 1.571014188000e-05,
	 4.382562723700e-08,
	-2.526116979400e-10,
	 6.431181933900e-13,
	-1.006347151900e-15,
	 9.974533899200e-19,
	-6.086324560700e-22,
	 2.084922933900e-25,
	-3.068219615100e-29,
};

TC_TempRanges typeN_T2mV_tempRanges[] = {
	{
		.coeffs    = typeN_T2mV_cold,
		.min       =  -270.000,
		.max       =     0.000,
		.numCoeffs = sizeof(typeN_T2mV_cold)/sizeof(typeN_T2mV_cold[0])
	},
	{
		.coeffs    = typeN_T2mV_warm,
		.min       =     0.000,
		.max       =  1300.000,
		.numCoeffs = sizeof(typeN_T2mV_warm)/sizeof(typeN_T2mV_warm[0])
	},
};

thermoPolyCoefficientSet typeN_T2mV = {
		.type          = N,
		.numTempRanges = sizeof(typeN_T2mV_tempRanges)/sizeof(typeN_T2mV_tempRanges[0]),
		.tempRanges    = typeN_T2mV_tempRanges
};

/************************************************************************************
 *
 *  @brief Type N Thermocouple Inverse Coefficients
 *  
 *         Voltage Range (mV)
 *  cold:   -3.990 to    0.000
 *  warm:    0.000 to   20.613
 *   hot:   20.613 to   47.513
 *
 *  Equation: T = d(0) + d(1)*E + d(2)*E^2 + ... + d(n)*E^n
 *
 ************************************************************************************
 */
const float typeN_mV2T_cold[] = {
	 0.000000000000e+00,
	 3.843684700000e+01,
	 1.101048500000e+00,
	 5.222931200000e+00,
	 7.206052500000e+00,
	 5.848858600000e+00,
	 2.775491600000e+00,
	 7.707516600000e-01,
	 1.158266500000e-01,
	 7.313886800000e-03,
};

const float typeN_mV2T_warm[] = {
	 0.000000000000e+00,
	 3.868960000000e+01,
	-1.082670000000e+00,
	 4.702050000000e-02,
	-2.121690000000e-06,
	-1.172720000000e-04,
	 5.392800000000e-06,
	-7.981560000000e-08,
};

const float typeN_mV2T_hot[] = {
	 1.972485000000e+01,
	 3.300943000000e+01,
	-3.915159000000e-01,
	 9.855391000000e-03,
	-1.274371000000e-04,
	 7.767022000000e-07,
};

TC_TempRanges typeN_mV2T_tempRanges[] = {
	{
	.coeffs    = typeN_mV2T_cold,
		.min       = -3.990e+00,
		.max       =  0.000e+00,
		.numCoeffs = sizeof(typeN_mV2T_cold)/sizeof(typeN_mV2T_cold[0])
	},
	{
	.coeffs    = typeN_mV2T_warm,
		.min       =  0.000e+00,
		.max       =  2.061e+01,
		.numCoeffs = sizeof(typeN_mV2T_warm)/sizeof(typeN_mV2T_warm[0])
	},
	{
	.coeffs    = typeN_mV2T_hot,
		.min       =  2.061e+01,
		.max       =  4.751e+01,
		.numCoeffs = sizeof(typeN_mV2T_hot)/sizeof(typeN_mV2T_hot[0])
	},
};

thermoPolyCoefficientSet typeN_mV2T = {
		.type          = N,
		.numTempRanges = sizeof(typeN_mV2T_tempRanges)/sizeof(typeN_mV2T_tempRanges[0]),
		.tempRanges    = typeN_mV2T_tempRanges
};

/*-----------------------------------------------------------------------------------------*/

#endif //THERMOCOUPLE_TYPEN_H_