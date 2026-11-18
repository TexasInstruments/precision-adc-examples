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

/**
 * @file thermocouple.c
 *
 * @brief Thermocouple Support Routines to Convert converted ADC data to Temperature
 *           Supports Type B, E, J, K, N, R, S, T Thermocouples
 * Source: http://srdata.nist.gov/its90/download/allcoeff.tab
 */

#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include "inc/thermocouple.h"

#ifdef LUT
	#include "inc/thermocouple_tables.h"
#endif

#ifdef POLYNOMIAL
#include "/inc/thermocouple_typeB.h"
#include "/inc/thermocouple_typeE.h"
#include "/inc/thermocouple_typeJ.h"
#include "/inc/thermocouple_typeK.h"
#include "/inc/thermocouple_typeN.h"
#include "/inc/thermocouple_typeR.h"
#include "/inc/thermocouple_typeS.h"
#include "/inc/thermocouple_typeT.h"
#endif

/**
 * @addtogroup group_Thermocouple
 *
 * @{
 */

/************************************************************************************//**
 *
 * @brief Thermocouple_Linearization_and_CJC()
 *        Converts Thermocouple voltage to temperature through interpolation to closest 
 *                 temperature value and applies cold junction compensation
 * 
 * @param[in] TCtype 	Thermocouple Type (B, E, J, K, N, R, S, T)
 * @param[in] TCmV   	Thermoelectric voltage of Thermocouple (mV)
 * @param[in] CJtemp   	Cold Junction Temperature (°C)
 *   	   
 * @return Thermocouple Temperature (°C)
 */
float Thermocouple_Linearization_and_CJC( TC_Type TCtype, float TCmV, float CJtemp )
{
	float CJmV = 0.0f;
	float mV = 0.0f;
	float temp = 0.0f;
#ifdef POLYNOMIAL
    thermoPolyCoefficientSet *tcSet = NULL;
	switch (TCtype)	{
		case B: tcSet = &typeB_T2mV;
			break;
		case E: tcSet = &typeE_T2mV; 
			break;
		case J: tcSet = &typeJ_T2mV; 
			break;
		case K: tcSet = &typeK_T2mV; 
			break;
		case N: tcSet = &typeN_T2mV; 
			break;
		case R: tcSet = &typeR_T2mV; 
			break;
		case S: tcSet = &typeS_T2mV; 
			break;
		case T: tcSet = &typeT_T2mV; 
			break;
		default:
    	    return NAN;
	}
#else

    thermoTableCoefficientSet *tcSet = NULL;
    switch( TCtype ) {
        case B:
            tcSet = &typeB_mV2T;
            break; 
        case E:
            tcSet = &typeE_mV2T;
            break;
        case J:
            tcSet = &typeJ_mV2T;
            break;
        case K:
            tcSet  = &typeK_mV2T;
            break;
        case N:
            tcSet = &typeN_mV2T;
            break;
        case R:
            tcSet = &typeR_mV2T;
            break;
        case S:
            tcSet = &typeS_mV2T;
            break;
        case T:
            tcSet = &typeT_mV2T;
            break;
        default:
            return NAN;
    }
#endif



	// Compute Thermoelectric Voltage of Thermocouple's cold junction temperature
  	CJmV = Thermocouple_Temp2mV( tcSet, CJtemp );

  	// Total Thermoelectric Voltage
  	mV = TCmV + CJmV;
    
	switch( TCtype ) {
        case B:
            tcSet = &typeB_mV2T;
            break; 
        case E:
            tcSet = &typeE_mV2T;
            break;
        case J:
            tcSet = &typeJ_mV2T;
            break;
        case K:
            tcSet  = &typeK_mV2T;
            break;
        case N:
            tcSet = &typeN_mV2T;
            break;
        case R:
            tcSet = &typeR_mV2T;
            break;
        case S:
            tcSet = &typeS_mV2T;
            break;
        case T:
            tcSet = &typeT_mV2T;
            break;
        default:
            return NAN;
    }



  	// Interpolate Temperature
  	temp = Thermocouple_mV2Temp( tcSet, mV );

  	return( temp );
}



#ifdef POLYNOMIAL 				// Polynomial expansion
/************************************************************************************//**
 *
 * @brief Calc_Polynomial()
 *        Computes the polynomial 
 *         y = SUM[i=0 to n] { c(i) * x^(i) }
 *
 * @param[in] x          Temperature (C) or Thermoelectric Voltage (mV)
 * @param[in] coeffs     Coefficient Array
 * @param[in] coeffCount number of coefficients
 *
 * @return y Thermoelectric Voltage (mV) or Temperature (C)
 *
 */
static float Calc_Polynomial( float x, const float* coeffs, int coeffCount )
{
	int i;
	float accum   = coeffs[0];				// c(0) * x^0 = c(0)
	float x2power = x;						// x^1 = temp

	for(i = 1; i < coeffCount; i++) {
		accum   += coeffs[i] * x2power;	// accum += c(i) * x^(i)
		x2power *= x;			       	// x^(i+1) = x  * x^(i);
	}
	return accum;
}

/************************************************************************************//**
 *
 * @brief Match_Range()
 *        Finds the coefficient range that contains the given value
 *
 * @param[in] value Temperature (C) or Thermoelectric Voltage (mV) to look up
 * @param[in] tcSet pointer to Thermocouple Coefficient Set
 *
 * @return Pointer to the matching range, or NULL if the value is outside all ranges
 *
 */
static TC_TempRanges * Match_Range( float value, thermoPolyCoefficientSet *tcSet )
{
	int i;
	TC_TempRanges *tcRange = NULL;
	for(i = 0; i < tcSet->numTempRanges; i++) {
		if ( (value >= tcSet->tempRanges[i].min) && (value <= tcSet->tempRanges[i].max)) {
			tcRange = &(tcSet->tempRanges[i]);
			return tcRange;
		}
	}
	return tcRange;
}


/************************************************************************************//**
 *
 * @brief Thermocouple_Temp2mV()
 *        Converts Temperature (C) into Thermoelectric Voltage (mV) for 
 *              specific type of thermocouple using polynomial formula
 *
 * @param[in] tcSet Thermocouple Type Set
 * @param[in] temp  Temperature (C)
 *
 * @return Thermoelectric Voltage (mV)
 *
 */
float Thermocouple_Temp2mV( thermoPolyCoefficientSet *tcSet, float temp )
{
	TC_TempRanges *tcRange       = NULL;
	float result                = 0.0f;

	if(temp == NAN) return NAN;

	tcRange = Match_Range(temp, tcSet);
	if ( tcRange == NULL )
		return NAN;
	result  += Calc_Polynomial(temp, tcRange->coeffs, tcRange->numCoeffs);
	return result;
}

/************************************************************************************//**
 *
 * @brief Thermocouple_mV2Temp()
 *        Converts Thermoelectric Voltage (mV) into Temperature for 
 *              specific type of thermocouple using polynomial formula
 *
 * @param[in] tcSet Thermocouple Type Set
 * @param[in] mV    Thermoelectric Voltage (mV)
 *
 * @return Temperature (C)
 *
 */
float Thermocouple_mV2Temp( thermoPolyCoefficientSet *tcSet, float mV )
{
	TC_TempRanges *tcRange       = NULL;
	float result                = 0.0f;

	tcRange = Match_Range(mV, tcSet);
	if ( tcRange == NULL )
		return NAN;
	result  = Calc_Polynomial(mV, tcRange->coeffs, tcRange->numCoeffs);
	return result;
}

/* ----------------------------------------------------------------------------------- */
#else							// Look-up Tables (LUT)			
/************************************************************************************//**
 *
 * @brief Thermocouple_Temp2mV()
 *        Convert Temperature (C) into Thermoelectric Voltage (mV) for 
 *              specific type of thermocouple using linear interpolated
 *              look-up tables
 *
 * @param[in] tcSet Thermocouple Type Set
 * @param[in] temp  Temperature (C)
 *
 * @return Thermoelectric Voltage (mV), or NAN if the temperature is outside the table
 *
 */
float Thermocouple_Temp2mV( thermoTableCoefficientSet *tcSet, float temp)
{
	int tcIndex;

	// if previous math gave NAN, return NAN.
	if ( temp == NAN ) return NAN;

	tcIndex = ((int) temp) - tcSet->min;	

	if ( tcIndex < 0 ) 
		return NAN;							// Temp below table min, return NAN
	else if ( tcIndex < 1 ) 
		return tcSet->min;					// Temp at min, return min
	else if ( tcIndex > (tcSet->max - tcSet->min) ) 
		return NAN;							// Temp above table max, return NAN
	else if ( tcIndex > (tcSet->max - tcSet->min - 1) ) 
		return tcSet->max;					// Temp at table max, return max

	// Linear Interpolation
	// mVx = m(tx - t0) + mV0; m = (mV1-mV0)/(temp1-temp0) = (mV1-mV0)/1
	// tx = mV0 + (tx - t0)*(mV1 - mV0)
	if ( (tcSet->table[tcIndex+1] - tcSet->table[tcIndex]) == 0 ) {  	// Both index are the same, choose upper one
		return( tcSet->table[tcIndex]  );
	} else {
		return( tcSet->table[tcIndex] + (temp - (tcSet->min + tcIndex)) * (tcSet->table[tcIndex+1] - tcSet->table[tcIndex]));
	}
}


/************************************************************************************//**
 *
 * @brief Binary_Search()
 *        Binary search of a Table
 *              
 * @param[in] table Thermoelectric Voltage Table
 * @param[in] value Thermoelectric Voltage value (mV)
 * @param[in] min   Minimum value in Table
 * @param[in] max   Maximum value in Table
 *         
 * @return index into Table with match
 *
 */
static int Binary_Search(const float *table, float value, int min, int max)
{
	int midPt = (min + max)/2;

	if ( (midPt == max) || (midPt == min) )
		return( midPt );
	else {
	    if ( value < table[midPt])
	        return Binary_Search( table, value, min, midPt - 1 );
	    else
	        return Binary_Search(table, value, midPt + 1, max );

	}
}

/************************************************************************************//**
 *
 * @brief Thermocouple_mV2Temp()
 *        Convert Thermoelectric Voltage (mV) into Temperature for 
 *              specific type of thermocouple using linear interpolated
 *              look-up tables
 *              
 * @param[in] tcSet Thermocouple Type Set
 * @param[in] mV    Thermoelectric Voltage (mV)
 *
 * @return Temperature (C)
 *
 */
float Thermocouple_mV2Temp( thermoTableCoefficientSet *tcSet, float mV)
{
	int tcIndex;

	// if previous math gave NAN, return NAN.
	if ( mV == NAN ) return NAN;

	// test to see if the mv value is within range.
	// interpolation method requires one above and one below
	// the 'found' value.
	if ( mV < tcSet->table[0] ) 
		return NAN;
	else if ( mV <= tcSet->table[1] )
		return tcSet->min;
	else if ( mV > tcSet->table[tcSet->max - tcSet->min] )
		return NAN;
	if ( mV > tcSet->table[tcSet->max - tcSet->min - 1] )
		return tcSet->max;

	tcIndex = Binary_Search( tcSet->table, mV, 0, tcSet->size - 1 );

	// Linear Interpolation
	// tx = m(mVx - mV0) + t0; m = (t1-t0)/(mV1-mV0) = 1/(mV1-mV0)
	// tx = t0 + (mVx - mV0)/(mV1 - mV0)
	if ( (tcSet->table[tcIndex+1] - tcSet->table[tcIndex]) == 0 ) {  	// Both index are the same, choose upper one
		return( tcIndex + tcSet->min  );
	} else {
		return( tcIndex + tcSet->min + (mV - tcSet->table[tcIndex])/(tcSet->table[tcIndex+1] - tcSet->table[tcIndex]) );
	}
}
#endif
/** @} // group group_Thermocouple
 *
 */
