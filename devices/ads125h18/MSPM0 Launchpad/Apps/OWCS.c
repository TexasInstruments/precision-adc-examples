/**
 * @file OWCS.c
 *
 * @brief This file contains all communication and device setup for OWCS.
 * @warning This software utilizes TI Drivers
 *
 * @copyright Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
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

#include <stdlib.h>
#include <string.h>
#include "OWCS.h"
#include "Driver/ads125h18.h"

///////////////////////////////////////////////////////////////////////////////
// See the OWCS Application Note: https://www.ti.com/lit/an/sdaa378/sdaa378.pdf
///////////////////////////////////////////////////////////////////////////////

/* Device ID: 0b10=V12, 0b00=V20, other=V40 */
#define OWCS_DEVICE_ID 0b00     

/**
 * @brief Demonstrate complete OWCS measurement workflow
 *
 * Calls all OWCS functions in sequence: validates configuration, collects data,
 * and calculates rsource values for measurement demonstration purposes.
 */
void OWCSexample(void)
{
    
    // Define step arrays for different measurement types
    // Input step#, not page# e.g. for step 0, input "0" even though step 0 is on page 1
    // monitor_steps[] can be an empty array. Input_steps[] and owcs_steps[] arrays should be entered exactly as in RegisterDetails.h
    uint8_t input_steps[] = {4, 7, 10, 11, 13};
    uint8_t monitor_steps[] = {16, 21, 22};
    uint8_t owcs_steps[] = {25, 26, 27, 28};    // always use four steps for OWCS -> first pair is for single-ended 
                                                // measurements only, first and second pairs are for differential measurements
    
    /* Create UserStepGroups struct */
    uint8_t num_input_steps = sizeof(input_steps) / sizeof(input_steps[0]);
    uint8_t num_owcs_steps = sizeof(owcs_steps) / sizeof(owcs_steps[0]);
    uint8_t num_monitor_steps = sizeof(monitor_steps) / sizeof(monitor_steps[0]);

    uint8_t ain_index = 0;
    
    UserStepGroups user_steps = {
        .input_steps = input_steps,
        .num_input_steps = num_input_steps,
        .owcs_steps = owcs_steps,
        .num_owcs_steps = num_owcs_steps,
        .monitor_steps = monitor_steps,
        .num_monitor_steps = num_monitor_steps
    };    
 
    /* Validate steps and retrieve configuration */
    resetDevice();                  // issue an SPI reset command and wait for reset. 
    initADC();                      // load the regmap config from _RegisterDetails.h file
    
    StepResult *step_result = retrieveStepConfig(&user_steps);
    if (!step_result) {
        return;
    }

    /* collect and process the OWCS data */
    while (1) {
        OWCSProcessResult OWCS_result = processOWCSMeasurements(step_result, user_steps, ain_index);
        
        // Reset the ain_index if we have reached the end of the input_ain_cfg array
        ain_index = (ain_index + 1) % user_steps.num_input_steps;
        
        /* USER TODO: Process rsource values (result->rsource_values[0], result->rsource_values[1]) */
        /* USER TODO: Process delta values (result->delta_values[0], result->delta_values[1]) */
        /* USER TODO: Process raw ADC data (result->all_sequences_data, result->total_samples) */
 
        free(OWCS_result.all_sequences_data);
    }
}

/**
 * @brief Collect and process OWCS (Open Wire Current Sources) measurements
 * @param step_result Pointer to step result containing conversion info
 * @param user_steps User step configuration
 * @param ain_index Current element in the analog input channel array on which to perform OWCS
 * @return OWCSProcessResult structure with pointers to data and metadata
 * @note Caller is responsible for freeing returned data arrays
 */
OWCSProcessResult processOWCSMeasurements(const StepResult *step_result, const UserStepGroups user_steps, uint8_t ain_index)
{
    // Initialize all fields to 0/NULL/false
    OWCSProcessResult result = {0};  
    
    uint32_t owcs_data_offset = 0;
    uint8_t num_owcs_pairs = 1;
        

        
    // Update channels on first and second OWCS steps to next value every time
    // add one to "page" because page 0 is the global page so step0 is actually page 1, etc.
    writeSingleRegister(user_steps.owcs_steps[0] + 1, 0x00, step_result->input_ain_cfg[ain_index]);
    writeSingleRegister(user_steps.owcs_steps[1] + 1, 0x00, step_result->input_ain_cfg[ain_index]);

    // calculate the total conversions
    // if the input channel is differential then we also need to add the third and fourth OWCS conversions to the total
    uint16_t total_conversions = step_result->input_conversions + step_result->monitor_conversions + step_result->owcs_conversions[0] + step_result->owcs_conversions[1];
    
    if (step_result->owcs_config_types[ain_index] == DIFFERENTIAL){
        total_conversions = total_conversions + step_result->owcs_conversions[2] + step_result->owcs_conversions[3];

        // update channels on the third and fourth OWCS channels if DIFFERENTIAL
        // add one to "page" because page 0 is the global page so step0 is actually page 1, etc.
        // add one to "data" because we are only storing the AINP value for a differential measurement
        //      for example, if AIN = AIN8-AIN9, we only store AIN8, but we know the other channel must be AIN9
        writeSingleRegister(user_steps.owcs_steps[2] + 1, 0x00, (step_result->input_ain_cfg[ain_index] + 1));
        writeSingleRegister(user_steps.owcs_steps[3] + 1, 0x00, (step_result->input_ain_cfg[ain_index] + 1));
        
        num_owcs_pairs = 2;       
    } 

    /* Allocate buffer for all sequences */
    uint32_t *all_sequences_data = (uint32_t *)malloc(total_conversions * sizeof(uint32_t));
    if (all_sequences_data == NULL) {
        return result;  // success = false
    }
    
    /* Collect all ADC data directly into allocated buffer */
    if (!collectADCData(total_conversions, all_sequences_data)) {
        free(all_sequences_data);
        return result;
    }
      
    // /* Calculate offset to OWCS data (skip input and monitor data) */
    owcs_data_offset = step_result->input_conversions + step_result->monitor_conversions;

    
    /* Process each OWCS pair */
    for (uint8_t pair_idx = 0; pair_idx < num_owcs_pairs; pair_idx++) {
        uint8_t disabled_idx = pair_idx * 2;
        uint8_t enabled_idx = pair_idx * 2 + 1;
        
        /* Get conversion counts for this pair */
        uint16_t owcs_disabled_samples = step_result->owcs_conversions[disabled_idx];
        uint16_t owcs_enabled_samples = step_result->owcs_conversions[enabled_idx];
        
        /* Calculate averages for disabled and enabled OWCS */
        float owcs_disabled_avg = calculateStepAverage(&all_sequences_data[owcs_data_offset], owcs_disabled_samples);
        owcs_data_offset += owcs_disabled_samples;
        
        float owcs_enabled_avg = calculateStepAverage(&all_sequences_data[owcs_data_offset], owcs_enabled_samples);
        owcs_data_offset += owcs_enabled_samples;
        
        /* Calculate OWCS delta */
        float delta = (owcs_enabled_avg - owcs_disabled_avg) / VREF;
        
        /* Calculate Rsource using the stored config type */
        ConfigType config = step_result->owcs_config_types[ain_index];
        float rsource = OWCSgetRsource(delta, config, OWCS_DEVICE_ID);
        
        /* Store rsource value */
        result.rsource_values[pair_idx] = rsource;
        result.delta_values[pair_idx] = delta;
    }
    
    /* Populate result structure */
    result.all_sequences_data = all_sequences_data;
    result.total_samples = total_conversions;
    
    return result;
}

/**
 * @brief Validate user-provided steps and retrieve their configuration
 *
 * Returns AIN_CFG values for input channels and their config type (single-ended or differential).
 * Uses static allocation (MAX_INPUT_AIN_CFG_SIZE max elements)
 * 
 * Returns total number of monitoring and input channel conversions
 * Returns an array of the number of conversions for each OWCS step
 *
 * @param user_steps Pointer to UserStepGroups struct containing arrays of step indices
 *
 * @return Pointer to static ValidationResult struct with input_ain_cfg array and total_conversions
 */
StepResult* retrieveStepConfig(UserStepGroups *user_steps)
{
    if (!user_steps || user_steps->num_input_steps == 0) {
        return NULL;
    }

    static StepResult result_static;
    StepResult *result = &result_static;

    //uint16_t total_conversions = 0;
    uint16_t input_conversions = 0;
    uint16_t monitor_conversions = 0;
    uint16_t array_index = 0;

    /* Process input channels: read all analog channel configurations, determine if single-ended or differential, and calculate total conversions */
    for (uint8_t i = 0; i < user_steps->num_input_steps; i++) {
        uint8_t step = user_steps->input_steps[i];

        /* Read AIN_CFG register and decode channel configuration */
      //  REG_IO ain_cfg.Register_data = 
        uint8_t cfg_value = registerMap[step + 1][ STEPX_AIN_CFG_ADDRESS];

        /* Store channel value and configuration type */
        if (array_index < MAX_INPUT_AIN_CFG_SIZE) {
            if (cfg_value <= 15) {
                /* Single-ended: store channel value directly */
                result->input_ain_cfg[array_index] = cfg_value;
                result->owcs_config_types[array_index++] = SINGLE_ENDED;
            } else {
                /* Differential: extract lower 4 bits and multiply by 2 to get channel pair start */
                uint8_t channel = (cfg_value & 0x0F) * 2;
                result->input_ain_cfg[array_index] = channel;
                result->owcs_config_types[array_index++] = DIFFERENTIAL;
            }
        }

        /* Extract NUM_CONV field (lower 4 bits) from ADC_REF_CFG and lookup actual conversion count */
        uint8_t num_conv_field = (registerMap[step + 1][STEPX_ADC_REF_CFG_ADDRESS] & STEPX_NUM_CONV_MASK) ;
        if (num_conv_field < sizeof(num_conversions_table) / sizeof(num_conversions_table[0])) {
            uint16_t conv_count = num_conversions_table[num_conv_field];
            input_conversions += conv_count;
        }
    }

    result->input_conversions = input_conversions;

    /* Process monitor channels for conversion count only */
    for (uint8_t i = 0; i < user_steps->num_monitor_steps; i++) {
        uint8_t step = user_steps->monitor_steps[i];

        /* Extract and accumulate conversion count for this monitor step */
        uint8_t num_conv_field = (registerMap[step + 1][STEPX_ADC_REF_CFG_ADDRESS] & STEPX_NUM_CONV_MASK );
        if (num_conv_field < sizeof(num_conversions_table) / sizeof(num_conversions_table[0])) {
            uint16_t conv_count = num_conversions_table[num_conv_field];
            monitor_conversions += conv_count;
        }
    }

    /* Process OWCS (Open Wire Current Sources) steps and return conversion count for each OWCS step  */
    for (uint8_t i = 0; i < user_steps->num_owcs_steps; i++) {
        uint8_t step = user_steps->owcs_steps[i];

        /* Extract and accumulate conversion count for this OWCS step */
        uint8_t num_conv_field = (registerMap[step + 1] [STEPX_ADC_REF_CFG_ADDRESS] & STEPX_NUM_CONV_MASK);
        if (num_conv_field < sizeof(num_conversions_table) / sizeof(num_conversions_table[0])) {
            uint16_t conv_count = num_conversions_table[num_conv_field];
            result->owcs_conversions[i] = conv_count;
        }
    }

    /* Return result with expanded input channel values and total conversion count across all channels */
    result->monitor_conversions = monitor_conversions;
    return result;
}

/**
 * @brief Collect ADC data from all steps in a sequence
 *
 * @param num_samples   Total number of samples to collect
 * @param buffer        Pointer to uint32_t buffer to store ADC reading values
 *
 * @return true if operation succeeded, false otherwise
 */
bool collectADCData(uint16_t num_samples, uint32_t *buffer)
{
    /* Validate input parameters */
    if (!buffer || num_samples == 0) {
        return false;
    }

    /* Begin ADC conversion sequence */
    startAdcConversion();
    waitForDRDYinterrupt(100);
    
    /* Collect data for all samples */
    for (uint16_t i = 0; i < num_samples; i++) {
        buffer[i] = readFIFO().ADC_reading;
    }

    /* Stop conversion after all samples are collected */
    stopAdcConversion();

    return true;
}

/**
 * @brief Calculate average of ADC readings
 *
 * @param readings      Array of ADC reading values
 * @param num_readings  Number of readings to average
 *
 * @return Average of all ADC readings as a float
 */
float calculateStepAverage(const uint32_t *readings, uint16_t num_readings)
{
    if (!readings || num_readings == 0) {
        return 0.0f;
    }

    double sum = 0.0;
    uint8_t num_bits = RESOLUTION_IS_16_BIT ? 16 : 24;
    
    for (uint16_t i = 0; i < num_readings; i++) {
        // Convert from two's complement to signed
        int32_t signed_value;
        
        // Check if sign bit is set
        if (readings[i] & (1u << (num_bits - 1))) {
            // Negative number - sign extend
            signed_value = (int32_t)(readings[i] | (~0u << num_bits));
        } else {
            // Positive number
            signed_value = (int32_t)readings[i];
        }
        
        sum += (double)signed_value;
    }

    return (float)(sum / num_readings);
}

/**
 * @brief Gets the Rsource value corresponding to the calculated delta value (OWCS on - OWCS off / VREF)
 *
 * @param value The previously calculated delta value
 * @param config The configuration type (SINGLE_ENDED or DIFFERENTIAL)
 * @param device_ID The device identifier (V12, V20, or V40)
 * @return The interpolated Rsource value in kOhms
 */
float OWCSgetRsource(float value, ConfigType config, uint8_t device_ID)
{
    uint8_t col_idx = 0;
    float A, B, C, max_rsource;
    
    /* Check which device version is being used */
    if (device_ID == 0b10) {
        col_idx = 0;    // V12
    } else if (device_ID == 0b00) {
        col_idx = 1;    // V20
    } else {
        col_idx = 2;    // V40
    }
    
    /* Retrieve variables for single-ended input calculations */
    if (config == SINGLE_ENDED) {
        A = SingleEnded_variables[0][col_idx];
        B = SingleEnded_variables[1][col_idx];
        C = SingleEnded_variables[2][col_idx];
        max_rsource = SingleEnded_variables[3][col_idx];
       
        /* Check if there is a divider / 0 error such that the input impedance is infinite */
        if (value >= C / 2.0f) {
            return max_rsource;
        }

        // Calculate the rsource value in ohms for single-ended inputs
        // return 0.0f if the result is negative 
        float result_se = (value * A - B) / (C - 2 * value);
        return (result_se < 0) ? 0.0f : result_se;
    
    /* Retrieve variables for differential inputs calculations */
    } else {
        A = Differential_variables[0][col_idx];
        B = Differential_variables[1][col_idx];
        C = Differential_variables[2][col_idx];
        max_rsource = Differential_variables[3][col_idx];
    
        /* Check if there is a divider / 0 error such that the input impedance is infinite */
        if (value >= C / 4.0f) {
            return max_rsource;
        }

        // Calculate the rsource value in ohms for differential inputs
        // return 0.0f if the result is negative                    
        float result_diff = (value * A - B) / (C - 4 * value);
        return (result_diff < 0) ? 0.0f : result_diff;
    }
}