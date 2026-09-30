/**
 * @file ads124s18_pages.h
 * @brief ADS124S18 memory page definitions and organization.
 *
 * Defines the 25 memory pages in the ADS124S18 register map. The ADC organizes
 * its 64-byte register space into pages to manage the large number of
 * configuration options for multi-channel sequencing.
 *
 * Register Map Organization:
 *
 * Page 0 (GENERAL_SETTINGS):
 * - Device configuration and status
 * - 64 registers (addresses 0x00-0x3F)
 * - Controls: power, conversions, reference, clock, SPI format, etc.
 * - Status: device ID, supply status, ADC status, digital status
 * - Used for global ADC setup and error monitoring
 *
 * Pages 1-24 (STEP_0 through STEP_23):
 * - Sequencer step configurations
 * - Each page: 24 registers (addresses 0x00-0x10)
 * - Each step defines: input channel, gain, filter, delay, calibration, etc.
 * - Used for multi-channel scanning sequences
 * - Only accessed when using sequencer mode
 *
 * Page Switching:
 * The ADC maintains a page pointer (GENERAL_SETTINGS.PAGE_POINTER, address 0x3F).
 * Before reading/writing to a step page, update this pointer:
 *   writeSingleRegister(GENERAL_SETTINGS, PAGE_POINTER_ADDRESS, STEP_X);
 * Then access that step's registers using STEP_X as the page parameter.
 *
 * Shadow Register Architecture:
 * All pages are mirrored in the driver's software registerMap array:
 *   registerMap[page][address]
 * This enables fast local access and automatic page switching without
 * redundant SPI transactions.
 *
 * Typical Usage:
 * - Configure GENERAL_SETTINGS (power, reference, SPI options)
 * - For sequencer mode: configure STEP_0 through STEP_N
 * - Driver automatically handles PAGE_POINTER switching
 *
 * @copyright Copyright (C) 2024-2026 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef ADS124S18_PAGES_H_
#define ADS124S18_PAGES_H_

#include <stdint.h>
#include <stdbool.h>


//!< Total number of memory pages in ADS124S18 register map
#define NUM_PAGES                           ((uint8_t) 25)

//**********************************************************************************
//
// Memory Page Definitions
//
// Page 0: Global ADC configuration and status (64 registers)
// Pages 1-24: Sequencer step configurations (24 registers each)
//
//**********************************************************************************

//!< Page 0: General settings - global ADC config, status, control
#define GENERAL_SETTINGS                    ((uint8_t) 0x00)

//!< Pages 1-24: Sequencer step configurations (one per step)
//!< Each step defines: input channels, gain, filter, calibration, delays
#define STEP_0                              ((uint8_t) 0x01)
#define STEP_1                              ((uint8_t) 0x02)
#define STEP_2                              ((uint8_t) 0x03)
#define STEP_3                              ((uint8_t) 0x04)
#define STEP_4                              ((uint8_t) 0x05)
#define STEP_5                              ((uint8_t) 0x06)
#define STEP_6                              ((uint8_t) 0x07)
#define STEP_7                              ((uint8_t) 0x08)
#define STEP_8                              ((uint8_t) 0x09)
#define STEP_9                              ((uint8_t) 0x0A)
#define STEP_10                             ((uint8_t) 0x0B)
#define STEP_11                             ((uint8_t) 0x0C)
#define STEP_12                             ((uint8_t) 0x0D)
#define STEP_13                             ((uint8_t) 0x0E)
#define STEP_14                             ((uint8_t) 0x0F)
#define STEP_15                             ((uint8_t) 0x10)
#define STEP_16                             ((uint8_t) 0x11)
#define STEP_17                             ((uint8_t) 0x12)
#define STEP_18                             ((uint8_t) 0x13)
#define STEP_19                             ((uint8_t) 0x14)
#define STEP_20                             ((uint8_t) 0x15)
#define STEP_21                             ((uint8_t) 0x16)
#define STEP_22                             ((uint8_t) 0x17)
#define STEP_23                             ((uint8_t) 0x18)

#endif /* ADS124S18_PAGES_H_ */