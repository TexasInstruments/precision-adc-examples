/**
 * @file thermocouple.h
 * 
 *  @brief Thermocouple conversion API and coefficient data structures for
 *         	Type B, E, J, K, N, R, S, T
 * Source: 	Type B, E, J, K, N, R, S, T
 * 			http://srdata.nist.gov/its90/download/allcoeff.tab
 *
 * 			Type G, M, P
 * 			http://www.astm.org/cgi-bin/resolver.cgi?E1751E1751M
 *
 * 			Type C, D
 * 			http://www.omega.com/temperature/z/pdf/z202.pdf
 
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


/**
 * @defgroup group_Thermocouple Thermocouple
 * @brief Thermocouple voltage/temperature conversion routines.
 *
 * @{
 */

//*****************************************************************************
//
// #DEFINES
//
//*****************************************************************************

// choose which method of code-to-temp translation to be used
#define POLYNOMIAL          // use polynomial calculation ( requires more compute time) 
//#define LUT               // use a Loop Up Table ( requires more memory )

//*****************************************************************************/


/**
 * @brief Defines types of Thermocouples
 */
typedef enum TCtypeDef {
	B,	///< Type B
	E,	///< Type E
	J,	///< Type J
	K,	///< Type K
	N,	///< Type N
	R,	///< Type R
	S,	///< Type S
	T,	///< Type T
#ifdef POLYNOMIAL
	C,	///< Type C (polynomial method only)
	D,	///< Type D (polynomial method only)
	G,	///< Type G (polynomial method only)
	M,	///< Type M (polynomial method only)
	P	///< Type P (polynomial method only)
#endif
} TC_Type;


/**
 * @brief Defines Thermocouple temperature ranges
 */
typedef struct TCTempRangeDef {
    const float min;		///< Lower bound of the range (degC or mV)
    const float max;		///< Upper bound of the range (degC or mV)
    const int   numCoeffs;	///< Number of polynomial coefficients
    const float *coeffs;	///< Polynomial coefficients, c(0) first
} TC_TempRanges;

/**
 * @brief Defines Thermocouple coefficient set for table lookup
 */
typedef struct thermoTableCoefficientSetDef
{
	TC_Type 	 type;		///< Thermocouple type
	int 		 min;		///< Lowest temperature in the table (degC)
	int 		 max;		///< Highest temperature in the table (degC)
	int 		 size;		///< Number of entries in the table
	const float* table;		///< Thermoelectric voltages (mV), one per degC from min
} thermoTableCoefficientSet;

/**
 * @brief Defines Thermocouple coefficient set for polynomial calculation
 */
typedef struct thermoPolyCoefficientSetDef
{
	TC_Type 	 	type;			///< Thermocouple type
	const int 	 	numTempRanges;	///< Number of entries in tempRanges
	TC_TempRanges 	*tempRanges;	///< Array of coefficient ranges
} thermoPolyCoefficientSet;


//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************
#ifdef POLYNOMIAL
#include <math.h>

/**
 * @brief Converts a thermoelectric voltage to temperature using polynomial coefficients.
 *
 * @param[in] tcSet Polynomial coefficient set for the thermocouple type.
 * @param[in] mV    Thermoelectric voltage (mV).
 *
 * @return Temperature (degC).
 */
float Thermocouple_mV2Temp( thermoPolyCoefficientSet *tcSet, float mV );

/**
 * @brief Converts a temperature to thermoelectric voltage using polynomial coefficients.
 *
 * @param[in] tcSet Polynomial coefficient set for the thermocouple type.
 * @param[in] temp  Temperature (degC).
 *
 * @return Thermoelectric voltage (mV).
 */
float Thermocouple_Temp2mV( thermoPolyCoefficientSet *tcSet, float temp );
#endif
#ifdef LUT

/**
 * @brief Converts a thermoelectric voltage to temperature using look-up tables.
 *
 * @param[in] tcSet Table coefficient set for the thermocouple type.
 * @param[in] mV    Thermoelectric voltage (mV).
 *
 * @return Temperature (degC).
 */
float Thermocouple_mV2Temp( thermoTableCoefficientSet *tcSet, float mV);

/**
 * @brief Converts a temperature to thermoelectric voltage using look-up tables.
 *
 * @param[in] tcSet Table coefficient set for the thermocouple type.
 * @param[in] temp  Temperature (degC).
 *
 * @return Thermoelectric voltage (mV).
 */
float Thermocouple_Temp2mV( thermoTableCoefficientSet *tcSet, float temp);
#endif

/**
 * @brief Converts a thermocouple voltage to temperature with cold junction compensation.
 *
 * Calculates the thermoelectric voltage of the cold junction, adds it to the
 * measured voltage, and converts the sum to temperature.
 *
 * @param[in] TCtype Thermocouple type.
 * @param[in] TCmV   Measured thermoelectric voltage (mV).
 * @param[in] CJtemp Cold junction temperature (degC).
 *
 * @return Thermocouple temperature (degC).
 */
float Thermocouple_Linearization_and_CJC( TC_Type TCtype, float TCmV, float CJtemp );

/** @} */

