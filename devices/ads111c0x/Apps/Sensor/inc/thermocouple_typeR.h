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
#ifndef THERMOCOUPLE_TYPER_H_
#define THERMOCOUPLE_TYPER_H_

/************************************************************************************
 *
 *  @brief Type R Thermocouple Coefficients
 *  
 *  Temperature Range (°C)
 *  cold:  -50.000 to 1064.180
 *  warm: 1064.180 to 1664.500
 *  hot: 1664.500 to 1768.100
 *
 *  Equation: E (mV) = SUM[i=0 to n] { c(i) * t^(i) }
 *
 ************************************************************************************
 */
const float typeR_T2mV_cold[] = {
	 0.000000000000e+00,
	 5.289617297650e-03,
	 1.391665897820e-05,
	-2.388556930170e-08,
	 3.569160010630e-11,
	-4.623476662980e-14,
	 5.007774410340e-17,
	-3.731058861910e-20,
	 1.577164823670e-23,
	-2.810386252510e-27,
};

const float typeR_T2mV_warm[] = {
	 2.951579253160e+00,
	-2.520612513320e-03,
	 1.595645018650e-05,
	-7.640859475760e-09,
	 2.053052910240e-12,
	-2.933596681730e-16,
};

const float typeR_T2mV_hot[] = {
	 1.522321182090e+02,
	-2.688198885450e-01,
	 1.712802804710e-04,
	-3.458957064530e-08,
	-9.346339710460e-15,
};

TC_TempRanges typeR_T2mV_tempRanges[] = {
	{
		.coeffs    = typeR_T2mV_cold,
		.min       =   -50.000,
		.max       =  1064.180,
		.numCoeffs = sizeof(typeR_T2mV_cold)/sizeof(typeR_T2mV_cold[0])
	},
	{
		.coeffs    = typeR_T2mV_warm,
		.min       =  1064.180,
		.max       =  1664.500,
		.numCoeffs = sizeof(typeR_T2mV_warm)/sizeof(typeR_T2mV_warm[0])
	},
	{
	.coeffs    = typeR_T2mV_hot,
		.min       =  1664.500,
		.max       =  1768.100,
		.numCoeffs = sizeof(typeR_T2mV_hot)/sizeof(typeR_T2mV_hot[0])
},
};

thermoPolyCoefficientSet typeR_T2mV = {
		.type          = R,
		.numTempRanges = sizeof(typeR_T2mV_tempRanges)/sizeof(typeR_T2mV_tempRanges[0]),
		.tempRanges    = typeR_T2mV_tempRanges
};

/************************************************************************************
 *
 *  @brief Type R Thermocouple Inverse Coefficients
 *  
 *         Voltage Range (mV)
 *  cold:   -0.226 to    1.923
 *  warm:    1.923 to   13.228
 *   hot:   11.361 to   19.739
 *   hot:   19.739 to   21.103
 *
 *  Equation: T = d(0) + d(1)*E + d(2)*E^2 + ... + d(n)*E^n
 *
 ************************************************************************************
 */
const float typeR_mV2T_cold[] = {
	 0.000000000000e+00,
	 1.889138000000e+02,
	-9.383529000000e+01,
	 1.306861900000e+02,
	-2.270358000000e+02,
	 3.514565900000e+02,
	-3.895390000000e+02,
	 2.823947100000e+02,
	-1.260728100000e+02,
	 3.135361100000e+01,
	-3.318776900000e+00,
};

const float typeR_mV2T_warm[] = {
	 1.334584505000e+01,
	 1.472644573000e+02,
	-1.844024844000e+01,
	 4.031129726000e+00,
	-6.249428360000e-01,
	 6.468412046000e-02,
	-4.458750426000e-03,
	 1.994710149000e-04,
	-5.313401790000e-06,
	 6.481976217000e-08,
};

const float typeR_mV2T_hot[] = {
	-8.199599416000e+01,
	 1.553962042000e+02,
	-8.342197663000e+00,
	 4.279433549000e-01,
	-1.191577910000e-02,
	 1.492290091000e-04,
};

const float typeR_mV2T_veryHot[] = {
	 3.406177836000e+04,
	-7.023729171000e+03,
	 5.582903813000e+02,
	-1.952394635000e+01,
	 2.560740231000e-01,
};

TC_TempRanges typeR_mV2T_tempRanges[] = {
	{
	.coeffs    = typeR_mV2T_cold,
		.min       = -2.260e-01,
		.max       =  1.923e+00,
		.numCoeffs = sizeof(typeR_mV2T_cold)/sizeof(typeR_mV2T_cold[0])
	},
	{
	.coeffs    = typeR_mV2T_warm,
		.min       =  1.923e+00,
		.max       =  1.323e+01,
		.numCoeffs = sizeof(typeR_mV2T_warm)/sizeof(typeR_mV2T_warm[0])
	},
	{
	.coeffs    = typeR_mV2T_hot,
		.min       =  1.136e+01,
		.max       =  1.974e+01,
		.numCoeffs = sizeof(typeR_mV2T_hot)/sizeof(typeR_mV2T_hot[0])
	},
	{
	.coeffs    = typeR_mV2T_veryHot,
		.min       =  1.974e+01,
		.max       =  2.110e+01,
		.numCoeffs = sizeof(typeR_mV2T_hot)/sizeof(typeR_mV2T_hot[0])
	},
};

thermoPolyCoefficientSet typeR_mV2T = {
		.type          = R,
		.numTempRanges = sizeof(typeR_mV2T_tempRanges)/sizeof(typeR_mV2T_tempRanges[0]),
		.tempRanges    = typeR_mV2T_tempRanges
};

/*-----------------------------------------------------------------------------------------*/

#endif //THERMOCOUPLE_TYPER_H_