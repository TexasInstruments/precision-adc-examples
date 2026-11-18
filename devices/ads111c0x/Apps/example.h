/**
 * \copyright Copyright (C) 2026 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef EXAMPLE_H_
#define EXAMPLE_H_
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"



//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

/**
 * @brief Runs the ADC example application.
 *
 * Starts up the ADC, configures it for single-shot conversions on AIN0 (single-ended, referenced to GND),
 * then loops forever: converts, scales the result to millivolts, reads the
 * internal temperature as the cold junction temperature, and calculates the
 * thermocouple temperature. This function does not return.
 */
void applicationExample(void);

//*****************************************************************************
//
// Typedefs
//
//*****************************************************************************

/**
 * @brief Describes the ADC Characteristics structure
 */
typedef struct ADCcharDef {
	uint8_t		 resolution;		///< ADC bit resolution (8, 10, 12, 14, 16, 18, 24, 32)
	float_t	 	 pgaGain;			///< ADC PGA linear gain setting (1x, 2x, 4x, 8x, 16x, etc.)
	int32_t 	 offsetCalCoeff;	///< ADC Offset Calibration Coefficient (ideal ADC has offsetCalCoeff = 0)
	int32_t 	 gainCalCoeff;		///< ADC Gain Calibration Coefficient (ideal ADC has gainCalCoeff = 1)
	uint32_t 	 sampleRate;		///< ADC sample rate in samples/sec
	float	 	 Vref;				///< ADC Voltage reference
	int16_t	 	 adcValue;			///< ADC conversion result provided as sign extended 2's complement (first one)
} ADCchar_Set;


#endif /* EXAMPLE_H_ */
