/**
 * @file examples.c
 * @brief Example implementations for ADS124S18 ADC configurations and data acquisition.
 *
 * This file provides two complete application examples for the ADS124S18
 * precision ADC interfaced with MSPM0 microcontroller:
 *
 * 1. **Simple ADC Example**: Continuous single-channel measurement with
 *    offset calibration, suitable for simple measurement applications.
 *
 * 2. **Sequencer Example**: Multi-channel scanning using the built-in sequencer,
 *    ideal for monitoring multiple input channels in a programmed sequence.
 *
 * Key global variables:
 * - ADC_readings[]: Ring buffer storing conversion results and status
 * - g_collectData: Control flag for continuous data acquisition
 * - ADC_STATUS: Status register snapshot for error checking
 *
 * \copyright Copyright (c) 2024-2026, Texas Instruments Incorporated
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

#include "Driver/ads124s18.h"
#include "Driver/ads124s18_page0.h"
#include "Driver/ads124s18_page1.h"
#include "Driver/ads124s18_pages.h"
#include "Driver/hal.h"
#include "Driver/crc.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

REG_IO ADC_Register;                //!< Register read/write result with STATUS and CRC validation
#define numSamples  64              //!< ADC samples per acquisition cycle 
ADC_IO ADC_readings[numSamples];    //!< Buffer: stores 24-bit conversions, STATUS flags, and CRC validity
STATUS_IO ADC_STATUS;               //!< Current ADC status flags: STATUS_MSB and STATUS_LSB error indicators
bool g_collectData = true;          //!< Control flag: set false to stop acquisition 

//**********************************************************************************
//  SIMPLE ADC EXAMPLE
//
//  Configuration: Single-channel continuous conversion with offset calibration
//  Inputs: Differential measurement (AIN0+, AIN1-)
//  Sequencer: Disabled - continuous conversion mode
//  Reference: 2.5V internal
//  Filter: SINC4 with configurable OSR
//
//  Execution sequence:
//  1. SPI reset and register initialization
//  2. Offset calibration:
//     - Configure inputs for internal short (AIN0 to AIN0)
//     - Set high OSR (8192) for accurate offset measurement
//     - Store calibration results in OFFSET_CAL registers
//  3. Reconfigure for measurement:
//     - Set inputs to AIN0 (positive) / AIN1 (negative)
//     - Lower OSR to 512 
//  4. Main acquisition loop:
//     - Collect numSamples conversions
//     - Enter low-power standby 
//     - Resume and repeat
//
//
//  Adjustable parameters:
//  - numSamples: Samples per cycle (modify #define at top of file)
//  - STEPX_FLTR_CFG_ADDRESS: Change OSR for speed/accuracy tradeoff
//  - delay_ms(20): Adjust standby duration between cycles
//  - REF_VAL_2, SPEED_MODE_SPEEDMODE1: Modify reference and clock speed
//
//  Options supported:
//  - SPI_CRC: Validates SPI communication integrity
//  - STATUS_ENABLED: Includes STATUS_MSB/LSB in every transaction
//  - REGMAP_CRC: Computes CRC over entire register map
//
//**********************************************************************************

/**
 * @brief Simple continuous ADC conversion example.
 *
 * Initializes the ADS124S18 for single-channel continuous conversion with
 * integrated offset calibration. This example demonstrates basic ADC operation
 * with minimal sequencer configuration.
 *
 * Execution flow:
 * 1. Reset device via SPI command
 * 2. Perform offset calibration:
 *    - Internally short AIN0 to measure offset
 *    - Use OSR 8192 for high-resolution offset measurement
 *    - Apply calibration to STEP_0 registers
 * 3. Reconfigure for differential measurement (AIN0 / AIN1)
 * 4. Enable register map CRC checking
 * 5. Continuously acquire numSamples, then enter low-power mode
 *
 * @return void
 *
 * @note Data is stored in global ADC_readings[] array (64-sample buffer).
 *       The acquisition continues while g_collectData = true.
 */
void simpleADC(void)
{
    // issue an SPI reset command and wait for reset.
    resetDevice();          

    //*****Offset Calibration Phase: Measure zero-point reference on internal short*****//
    // Configure reference and clock: 2.5V internal, 256kHz fmod
    writeSingleRegister(GENERAL_SETTINGS, ADC_CFG_ADDRESS, (REF_VAL_2 | SPEED_MODE_SPEEDMODE1));

    // Internal short calibration: connect AIN0 to itself to measure offset
    // This removes DC errors from the signal path
    writeSingleRegister(STEP_0, STEPX_AINP_CFG_ADDRESS, STEPX_AINP_AIN0);
    writeSingleRegister(STEP_0, STEPX_AINN_CFG_ADDRESS, STEPX_AINN_AIN0);

    // High-resolution OSR (8192) ensures accurate offset measurement
    // SINC4 filter provides excellent noise rejection at cost of lower speed
    writeSingleRegister(STEP_0, STEPX_FLTR_CFG_ADDRESS, STEPX_FLTR_OSR_OSR8192_SINC4);

    startAdcConversion();
    delay_ms(30);           // Wait for ADC output to stabilize (SINC4 filter settling time)
    ADC_IO ADC_Offset_reading = readData();
    stopAdcConversion();

    // Extract 24-bit offset measurement and decompose into 3 bytes
    // The ADC_reading format is [MSB, MidSB, LSB] with optional STATUS/CRC prepended
    uint8_t OFFSET_CAL_MSB = (ADC_Offset_reading.ADC_reading & 0x00FF0000) >> 16;
    uint8_t OFFSET_CAL_MidSB = (ADC_Offset_reading.ADC_reading & 0x0000FF00) >> 8;
    uint8_t OFFSET_CAL_LSB = (ADC_Offset_reading.ADC_reading & 0x000000FF);

    // Write offset calibration: Stored in STEP_0 for use during normal measurement
    // These registers are subtracted from future conversions by the ADC
    writeSingleRegister(STEP_0, STEPX_OFFSET_CAL_MSB_ADDRESS, OFFSET_CAL_MSB);
    writeSingleRegister(STEP_0, STEPX_OFFSET_CAL_ISB_ADDRESS, OFFSET_CAL_MidSB);
    writeSingleRegister(STEP_0, STEPX_OFFSET_CAL_LSB_ADDRESS, OFFSET_CAL_LSB);
    
    //*****Measurement Configuration Phase: Set up for differential signal acquisition*****//
    // Configure differential input pair: AIN0 (positive), AIN1 (negative)
    // This measures the voltage difference between external signal sources
    writeSingleRegister(STEP_0, STEPX_AINP_CFG_ADDRESS, STEPX_AINP_AIN0);
    writeSingleRegister(STEP_0, STEPX_AINN_CFG_ADDRESS, STEPX_AINN_AIN1);

    // Lower OSR from 8192 to 512 for faster sampling rate
    // OSR 512 provides good balance of speed (250+ SPS) and noise rejection
    // Adjust this value based on your application's throughput requirements
    writeSingleRegister(STEP_0, STEPX_FLTR_CFG_ADDRESS, STEPX_FLTR_OSR_OSR512_SINC4);

    // Enable register map CRC: Validates that all ADC configuration registers
    // remain uncorrupted. CRC mismatch indicates SPI communication error or noise.
    enableRegisterMapCrc(true);

    //*****Main Acquisition Loop*****//
    while (g_collectData)
    {
        // Read and check ADC status registers for errors before acquisition
        ADC_STATUS = checkStatus();

        // Start continuous conversions
        startAdcConversion();

        // Read numSamples conversions into the buffer
        // Each readData() waits for DRDY signal and returns 24-bit result + STATUS + optional CRC
        for(uint8_t i = 0; i < numSamples; i++)
        {
            ADC_readings[i] = readData();
        }

        // Stop conversions and enter standby for power efficiency
        stopAdcConversion();
        powerDownMode();

        // 20ms standby interval between acquisition cycles
        // Adjust this delay based on your power budget and throughput requirements
        delay_ms(20);

        // Restore to active mode for next acquisition cycle
        activeMode();
    }
}

//**********************************************************************************
//  SEQUENCER ADC EXAMPLE
//
//  Configuration: Multi-channel scanning via built-in sequencer
//  Inputs: Single-ended measurements on 4 channels (AIN0, AIN1, AIN2, AIN3)
//  Sequencer: Enabled - programmed step sequence
//  Number of steps: 4 (one per input channel)
//  Conversions per step: 8 
//
//  Execution sequence:
//  1. SPI reset and register initialization
//  2. Load complete register configuration from ADS124S18_RegisterDetails.h
//     (this file contains all sequencer step definitions and settings)
//  3. Main loop: Continuously execute programmed sequence
//     - Each sequence cycle measures all 4 channels (8 conversions per channel)
//     - Total samples per cycle: numSeqSamples (32 default)
//     - ADC automatically transitions between steps
//  4. Pause 200ms between sequence cycles
//
//
//  Register Configuration:
//  The sequencer configuration is defined in ADS124S18_RegisterDetails.h:
//  - Each step can have independent:
//    * Input channel assignment (AIN0-AIN15)
//    * Gain setting
//    * Filter type and OSR
//    * IDAC excitation current (optional)
//    * Delay between conversions
//
//  Adjustable parameters:
//  - numSeqSamples: Total conversions per sequence cycle
//  - Step configuration: Modify ADS124S18_sequencer_config[] in _RegisterDetails.h
//  - delay_ms(200): Interval between sequence cycles
//  - Channel mapping: Configure STEPX_AINP_CFG in each step
//
//  Options supported:
//  - SPI_CRC: Validates SPI communication integrity
//  - STATUS_ENABLED: Includes STATUS_MSB/LSB in transactions
//  - REGMAP_CRC: Validates register map integrity
//
//  Notes:
//  - If changing the number of steps or conversions per step, update
//    numSeqSamples to match (steps × conversions/step)
//  - Register configuration persists between cycles
//
//**********************************************************************************

/**
 * @brief Multi-channel ADC sequencer example.
 *
 * Initializes the ADS124S18 for multi-channel scanning using the hardware
 * sequencer. This example demonstrates automated sequential measurement of
 * multiple input channels with minimal software overhead.
 *
 * Execution flow:
 * 1. Initialize CRC engine
 * 2. Reset device via SPI command
 * 3. Load complete ADC configuration from ADS124S18_RegisterDetails.h
 * 4. Continuously execute programmed sequence:
 *    - Hardware automatically steps through 4 measurement configurations
 *    - Each step measures one input channel
 *    - After final step, ADC halts and sequence re-initializes
 * 5. Pause 200ms between complete sequence cycles
 *
 * Sequencer benefits over continuous mode:
  * - Lower software overhead (hardware manages sequencing)
 * - Flexible per-step configuration (gain, filter, delay, channels)
 * - Synchronized multi-channel sampling within a sequence
 *
 * @return void
 *
 * @note Configuration is stored in ADS124S18_RegisterDetails.h. To modify
 *       channel assignments, gains, or filter settings, edit the step
 *       configuration definitions in that file. If adjusting the number of
 *       steps or conversions per step, also update numSeqSamples accordingly.
 *
 * @note Data is stored in global ADC_readings[] array. The acquisition
 *       continues while g_collectData = true.
 */
void sequencerExample(void)
{
    // Total conversions per sequence cycle
    // For 4-step configuration with 8 conversions per step: 4 × 8 = 32
    // Adjust if changing number of sequencer steps or conversions per step in _RegisterDetails.h
    uint16_t numSeqSamples = 32;

    initCRC();
    resetDevice();          // Issue SPI reset command and wait for reset completion
    initADC();              // Load all sequencer and measurement register settings from _RegisterDetails.h

    //*****Main Acquisition Loop*****//
    while (g_collectData)
    {
        // Start the sequencer: hardware automatically cycles through configured steps
        startAdcConversion();

        // Read all samples collected during the sequence execution
        // Each readData() waits for DRDY and retrieves the conversion result
        for(uint8_t i = 0; i < numSeqSamples; i++)
        {
            ADC_readings[i] = readData();
        }

        // 200ms pause between sequence cycles allows time for result processing
        // Adjust based on your application's data rate requirements
        delay_ms(200);
    }
}

