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

#ifndef THERMOCOUPLE_TYPEB_H_
#define THERMOCOUPLE_TYPEB_H_

/************************************************************************************
 *
 *  @brief Type B Thermocouple Coefficients
 *  
 *  Temperature Range (°C)
 *  cold:    0.000 to  630.615
 *  warm:  630.615 to 1820.000
 *
 *  Equation: E (mV) = SUM[i=0 to n] { c(i) * t^(i) }
 *
 ************************************************************************************
 */
const float typeB_T2mV_cold[] = {
	 0.000000000000e+00,
	-2.465081834600e-04,
	 5.904042117100e-06,
	-1.325793163600e-09,
	 1.566829190100e-12,
	-1.694452924000e-15,
	 6.299034709400e-19,
};

const float typeB_T2mV_warm[] = {
	-3.893816862100e+00,
	 2.857174747000e-02,
	-8.488510478500e-05,
	 1.578528016400e-07,
	-1.683534486400e-10,
	 1.110979401300e-13,
	-4.451543103300e-17,
	 9.897564082100e-21,
	-9.379133028900e-25,
};

TC_TempRanges typeB_T2mV_tempRanges[] = {
	{
		.coeffs    = typeB_T2mV_cold,
		.min       =     0.000,
		.max       =   630.615,
		.numCoeffs = sizeof(typeB_T2mV_cold)/sizeof(typeB_T2mV_cold[0])
	},
	{
		.coeffs    = typeB_T2mV_warm,
		.min       =   630.615,
		.max       =  1820.000,
		.numCoeffs = sizeof(typeB_T2mV_warm)/sizeof(typeB_T2mV_warm[0])
	},
};

thermoPolyCoefficientSet typeB_T2mV = {
		.type          = B,
		.numTempRanges = sizeof(typeB_T2mV_tempRanges)/sizeof(typeB_T2mV_tempRanges[0]),
		.tempRanges    = typeB_T2mV_tempRanges
};

/************************************************************************************
 *
 *  @brief Type B Thermocouple Inverse Coefficients
 *  
 *         Voltage Range (mV)
 *  cold:    0.291 to    2.431
 *  warm:    2.431 to   13.820
 *
 *  Equation: T = d(0) + d(1)*E + d(2)*E^2 + ... + d(n)*E^n
 *
 ************************************************************************************
 */
const float typeB_mV2T_cold[] = {
	 9.842332100000e+01,
	 6.997150000000e+02,
	-8.476530400000e+02,
	 1.005264400000e+03,
	-8.334595200000e+02,
	 4.550854200000e+02,
	-1.552303700000e+02,
	 2.988675000000e+01,
	-2.474286000000e+00,
};

const float typeB_mV2T_warm[] = {
	 2.131507100000e+02,
	 2.851050400000e+02,
	-5.274288700000e+01,
	 9.916080400000e+00,
	-1.296530300000e+00,
	 1.119587000000e-01,
	-6.062519900000e-03,
	 1.866169600000e-04,
	-2.487858500000e-06,
};

TC_TempRanges typeB_mV2T_tempRanges[] = {
	{
	.coeffs    = typeB_mV2T_cold,
		.min       =  2.910e-01,
		.max       =  2.431e+00,
		.numCoeffs = sizeof(typeB_mV2T_cold)/sizeof(typeB_mV2T_cold[0])
	},
	{
	.coeffs    = typeB_mV2T_warm,
		.min       =  2.431e+00,
		.max       =  1.382e+01,
		.numCoeffs = sizeof(typeB_mV2T_warm)/sizeof(typeB_mV2T_warm[0])
	},
};

thermoPolyCoefficientSet typeB_mV2T = {
		.type          = B,
		.numTempRanges = sizeof(typeB_mV2T_tempRanges)/sizeof(typeB_mV2T_tempRanges[0]),
		.tempRanges    = typeB_mV2T_tempRanges
};

/*-----------------------------------------------------------------------------------------*/

#endif // THERMOCOUPLE_TYPEB_H_