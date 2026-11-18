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

#ifndef THERMOCOUPLE_TYPES_H_
#define THERMOCOUPLE_TYPES_H_


/************************************************************************************
 *
 *  @brief Type S Thermocouple Coefficients
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
const float typeS_T2mV_cold[] = {
	 0.000000000000e+00,
	 5.403133086310e-03,
	 1.259342897400e-05,
	-2.324779686890e-08,
	 3.220288230360e-11,
	-3.314651963890e-14,
	 2.557442517860e-17,
	-1.250688713930e-20,
	 2.714431761450e-24,
};

const float typeS_T2mV_warm[] = {
	 1.329004440850e+00,
	 3.345093113440e-03,
	 6.548051928180e-06,
	-1.648562592090e-09,
	 1.299896051740e-14,
};

const float typeS_T2mV_hot[] = {
	 1.466282326360e+02,
	-2.584305167520e-01,
	 1.636935746410e-04,
	-3.304390469870e-08,
	-9.432236906120e-15,
};

TC_TempRanges typeS_T2mV_tempRanges[] = {
	{
		.coeffs    = typeS_T2mV_cold,
		.min       =   -50.000,
		.max       =  1064.180,
		.numCoeffs = sizeof(typeS_T2mV_cold)/sizeof(typeS_T2mV_cold[0])
	},
	{
		.coeffs    = typeS_T2mV_warm,
		.min       =  1064.180,
		.max       =  1664.500,
		.numCoeffs = sizeof(typeS_T2mV_warm)/sizeof(typeS_T2mV_warm[0])
	},
	{
	.coeffs    = typeS_T2mV_hot,
		.min       =  1664.500,
		.max       =  1768.100,
		.numCoeffs = sizeof(typeS_T2mV_hot)/sizeof(typeS_T2mV_hot[0])
},
};

thermoPolyCoefficientSet typeS_T2mV = {
		.type          = S,
		.numTempRanges = sizeof(typeS_T2mV_tempRanges)/sizeof(typeS_T2mV_tempRanges[0]),
		.tempRanges    = typeS_T2mV_tempRanges
};

/************************************************************************************
 *
 *  @brief Type S Thermocouple Inverse Coefficients
 *  
 *         Voltage Range (mV)
 *  cold:   -0.235 to    1.874
 *  warm:    1.874 to   11.950
 *   hot:   10.332 to   17.536
 *   hot:   17.536 to   18.693
 *
 *  Equation: T = d(0) + d(1)*E + d(2)*E^2 + ... + d(n)*E^n
 *
 ************************************************************************************
 */
const float typeS_mV2T_cold[] = {
	 0.000000000000e+00,
	 1.849494600000e+02,
	-8.005040620000e+01,
	 1.022374300000e+02,
	-1.522485920000e+02,
	 1.888213430000e+02,
	-1.590859410000e+02,
	 8.230278800000e+01,
	-2.341819440000e+01,
	 2.797862600000e+00,
};

const float typeS_mV2T_warm[] = {
	 1.291507177000e+01,
	 1.466298863000e+02,
	-1.534713402000e+01,
	 3.145945973000e+00,
	-4.163257839000e-01,
	 3.187963771000e-02,
	-1.291637500000e-03,
	 2.183475087000e-05,
	-1.447379511000e-07,
	 8.211272125000e-09,
};

const float typeS_mV2T_hot[] = {
	-8.087801117000e+01,
	 1.621573104000e+02,
	-8.536869453000e+00,
	 4.719686976000e-01,
	-1.441693666000e-02,
	 2.081618890000e-04,
};

const float typeS_mV2T_veryHot[] = {
	 5.333875126000e+04,
	-1.235892298000e+04,
	 1.092657613000e+03,
	-4.265693686000e+01,
	 6.247205420000e-01,
};

TC_TempRanges typeS_mV2T_tempRanges[] = {
	{
	.coeffs    = typeS_mV2T_cold,
		.min       = -2.350e-01,
		.max       =  1.874e+00,
		.numCoeffs = sizeof(typeS_mV2T_cold)/sizeof(typeS_mV2T_cold[0])
	},
	{
	.coeffs    = typeS_mV2T_warm,
		.min       =  1.874e+00,
		.max       =  1.195e+01,
		.numCoeffs = sizeof(typeS_mV2T_warm)/sizeof(typeS_mV2T_warm[0])
	},
	{
	.coeffs    = typeS_mV2T_hot,
		.min       =  1.033e+01,
		.max       =  1.754e+01,
		.numCoeffs = sizeof(typeS_mV2T_hot)/sizeof(typeS_mV2T_hot[0])
	},
	{
	.coeffs    = typeS_mV2T_veryHot,
		.min       =  1.754e+01,
		.max       =  1.869e+01,
		.numCoeffs = sizeof(typeS_mV2T_hot)/sizeof(typeS_mV2T_hot[0])
	},
};

thermoPolyCoefficientSet typeS_mV2T = {
		.type          = S,
		.numTempRanges = sizeof(typeS_mV2T_tempRanges)/sizeof(typeS_mV2T_tempRanges[0]),
		.tempRanges    = typeS_mV2T_tempRanges
};

/*-----------------------------------------------------------------------------------------*/

#endif //THERMOCOUPLE_TYPES_H_
