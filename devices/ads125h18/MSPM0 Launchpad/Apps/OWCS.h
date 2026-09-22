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

#ifndef OWCS_H_
#define OWCS_H_

#include <stdint.h>
#include <stdbool.h>
#include "Driver/ads125h18.h"
#include "Driver/hal.h"

#define MAX_INPUT_AIN_CFG_SIZE 48                       /* Maximum size for ValidationResult input_ain_cfg array */
#define VREF (RESOLUTION_IS_24_BIT ? 8388608 : 32768)   /* VREF for OWCS calculations */

//**********************************************************************************
//
// Typedefs
//
//**********************************************************************************

typedef enum {
    SINGLE_ENDED,
    DIFFERENTIAL
} ConfigType;

typedef struct {
    uint8_t input_ain_cfg[MAX_INPUT_AIN_CFG_SIZE];          /* Array of channel values for input channels */
    uint16_t input_conversions;                             /* total conversions for input channels */
    uint16_t monitor_conversions;                           /* total conversions for monitor channels */
    uint16_t owcs_conversions[4];                           /* Conversion count for each OWCS channel */
    ConfigType owcs_config_types[MAX_INPUT_AIN_CFG_SIZE];   /* Configuration type (SINGLE_ENDED or DIFFERENTIAL) for each OWCS channel */
} StepResult;

typedef struct {
    uint8_t *input_steps;        /* Array of input channel step indices */
    uint8_t num_input_steps;     /* Number of input channel steps */
    uint8_t *owcs_steps;         /* Array of OWCS channel step indices */
    uint8_t num_owcs_steps;      /* Number of OWCS channel steps */
    uint8_t *monitor_steps;      /* Array of monitor channel step indices */
    uint8_t num_monitor_steps;   /* Number of monitor channel steps */
} UserStepGroups;

typedef struct {
    uint32_t *all_sequences_data;   // All ADC data (input + monitor + OWCS)
    float rsource_values[2];        // Two Rsource values (one per OWCS pair)
    float delta_values[2];          // Two delta values (one per OWCS pair)
    uint32_t total_samples;         // Total number of samples in all_sequences_data
} OWCSProcessResult;

/* Mapping of NUM_CONV register values to actual conversion counts */
static const uint16_t num_conversions_table[] = {
    1, 2, 3, 4, 6, 8, 10, 12, 14, 16, 24, 32, 64, 128, 256, 512
};

//**********************************************************************************
//
// Rsource and delta tables
//
//**********************************************************************************

/* Single-ended variables (Rows: A, B, C, MaxRsource) */
static float SingleEnded_variables[4][3] = {
    /* V12          V20             V40 */
    {2625000.0f,    2500000.0f,     2375000.0f},   /* A_SE */
    {421875.0f,     562500.0f,      562500.0f},    /* B_SE */
    {0.375f,        0.5f,           0.5f},         /* C_SE */
    {10e9f,         10e9f,          10e9f}         /* MaxRsource */
};

/* Differential variables (Rows: A, B, C, MaxRsource) */
static float Differential_variables[4][3] = {
    /* V12          V20             V40 */
    {10500000.0f,   10000000.0f,    9500000.0f},   /* A_Diff */
    {1828125.0f,    2375000.0f,     2312500.0f},   /* B_Diff */
    {0.75f,         1.0f,           1.0f},         /* C_Diff */
    {10e9f,         10e9f,          10e9f}         /* MaxRsource */
};

//**********************************************************************************
//
// Function prototypes
//
//**********************************************************************************

OWCSProcessResult processOWCSMeasurements(const StepResult *step_result, const UserStepGroups user_steps, uint8_t ain_index);
void initializeADC(void);
bool collectADCData(uint16_t num_samples, uint32_t *buffer);
float calculateStepAverage(const uint32_t *readings, uint16_t num_readings);
StepResult* retrieveStepConfig(UserStepGroups *user_steps);
void OWCSexample(void);
float OWCSgetRsource(float value, ConfigType config, uint8_t device_ID);

#endif /* OWCS_H_ */
