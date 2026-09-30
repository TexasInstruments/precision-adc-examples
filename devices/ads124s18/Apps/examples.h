/**
 * @file examples.h
 * @brief Example implementations demonstrating ADS124S18 ADC usage patterns.
 *
 * This header provides two example configurations for the ADS124S18 precision
 * ADC: a simple continuous conversion mode and a sequencer-based multi-channel
 * scanning mode. These examples show different use cases from basic single-channel
 * acquisition to complex multi-step configurations with integrated calibration.
 *
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

#ifndef EXAMPLES_H_
#define EXAMPLES_H_
#include <stdbool.h>
#include <stdint.h>

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

/**
 * @brief Simple continuous ADC conversion example.
 *
 * Demonstrates basic ADS124S18 operation with continuous single-channel conversion.
 * The example performs the following steps:
 * - Issues device reset and waits for completion
 * - Performs offset calibration using internal short on AIN0 with OSR 8192
 * - Configures differential measurement between AIN0 (positive) and AIN1 (negative)
 * - Sets filter to OSR 512 for optimal signal acquisition
 * - Continuously collects ADC_readings[numSamples] conversions
 * - Enters low-power standby between acquisition cycles
 *
 * Configuration:
 * - Reference: 2.5V internal reference (REF_VAL_2)
 * - Speed Mode: 01 (256 kHz fmod)
 * - Input: External signal on AIN0/AIN1 differential pair
 * - Filter: SINC4 filter with OSR 512
 * - Number of samples per cycle: Defined by numSamples macro (64 default)
 *
 * Cycle timing: ~20ms per acquisition cycle (20ms powerdown delay between cycles)
 *
 * @note This example supports SPI_CRC, STATUS_HEADER, and REGMAP_CRC options.
 *       Adjust numSamples, step0_settings, and delay times as needed.
 */
void simpleADC(void);

/**
 * @brief Multi-channel sequencer ADC example.
 *
 * Demonstrates ADS124S18 operation in sequencer mode as a 4-channel scanner.
 * The example performs the following steps:
 * - Initializes CRC and resets the device
 * - Loads register configuration from ADS124S18_RegisterDetails.h
 * - Continuously executes a pre-configured sequence of 4 steps
 * - Each step measures a single-ended input (AIN0 through AIN3)
 * - Collects all conversions into ADC_readings[] array
 * - Pauses 200ms between complete sequence cycles
 *
 * Sequence behavior:
 * - Total conversions per cycle: numSeqSamples (32 default)
 * - After sequence completion, ADC stops and re-initializes
 * - Each step typically converts 8 times (4 steps × 8 conversions = 32 samples)
 *
 * Configuration:
 * - Uses register settings from ADS124S18_RegisterDetails.h for each step
 * - Supports SPI_CRC, STATUS_HEADER, and REGMAP_CRC options
 *
 * Adjustable parameters:
 * - numSeqSamples: Total number of conversions per sequence (must match
 *   configured number of steps × conversions per step)
 * - Step configuration: Define input channels, gains, filters, and delays
 *   in ADS124S18_RegisterDetails.h
 *
 * @note If adjusting the number of steps or NUM_CONVERSIONS per step,
 *       update the numSeqSamples variable accordingly.
 */
void sequencerExample(void);

#endif /* EXAMPLES_H_ */
