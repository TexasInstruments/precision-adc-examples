/**
 *
 * \copyright Copyright (C) 2025 Texas Instruments Incorporated - http://www.ti.com/
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

#include "driverlib.h"
#include "device.h"


// Macro to change code based on number of data lanes used. Can be 1 or 4
#define DATA_LANES_SELECT (4)


// Macro for ADC data output array
#define MAX_ENTRIES 128

//
// SPI settings
//
#define mySPI0_BASE SPID_BASE

// SPI MACRO to change SPI speed
#define mySPI0_BITRATE 33000000
// 1-lane mode has been tested upto mySPI0_BITRATE = 50MSPS and CONVST = 900KHz
// 4-lane mode has been tested upto mySPI0_BITRATE = 33MSPS and CONVST = 2MHz
//  CONVST variable name is ADS9327_EXT_ADC1_SAMPLING_FREQUENCY_KHZ


#define mySPI0_DATAWIDTH 8

#define mySPI0_SPIPTE_GPIO_4LANE 94
#define mySPI0_SPIPTE_PIN_CONFIG_4LANE 0x00881C0FU

// Device clocking conditions
//==============================================================================
//
#define ADS9327_SYSCLK_HZ        ((float32_t)200 * 1000000)
#define ADS9327_SYSCLK_NS        ((float32_t)1000000000 / ADS9327_SYSCLK_HZ)
#define ADS9327_EPWM_HZ          ((float32_t)100 * 1000000)
#define ADS9327_EPWM_NS          ((float32_t)1000000000 / ADS9327_EPWM_HZ)
#define ADS9327_INTOSC_HZ        ((float32_t) 25 * 1000000)

#define ADS9327_EPWM_EPWMCLK_DIV          EPWM_CLOCK_DIVIDER_1
#define ADS9327_EPWM_HSCLK_DIV            EPWM_HSCLOCK_DIVIDER_1

#if(ADS9327_EPWM_HSCLK_DIV == EPWM_HSCLOCK_DIVIDER_1)
    #define ADS9327_EPWM_TOTAL_CLKDIV     ((uint16_t)0x1 << ADS9327_EPWM_EPWMCLK_DIV)
#else
    #define ADS9327_EPWM_TOTAL_CLKDIV     (((uint16_t)0x1 << ADS9327_EPWM_EPWMCLK_DIV) * (ADS9327_EPWM_HSCLK_DIV << 1))
#endif

#define ADS9327_EPWM_PERIOD_TICKS     ((uint32_t)(ADS9327_EPWM_HZ / ADS9327_EPWM_SWITCHING_FREQUENCY_HZ / ADS9327_EPWM_TOTAL_CLKDIV))
#define ADS9327_EPWM_TBPRD            ((uint16_t)ADS9327_EPWM_PERIOD_TICKS - 1U)
#define ADS9327_EPWM_PERIOD_SEC       ((uint32_t)ADS9327_EPWM_PERIOD_TICKS / ADS9327_EPWM_HZ)

//
// EPWM configuration for external ADC CONVSTA/B signal
//
#define ADS9327_EXT_ADC1_CONVST_EPWM_BASE                 EPWM1_BASE
#define ADS9327_EXT_ADC1_CONVST_EPWM_NUM                  ((uint16_t)1)
#define ADS9327_EXT_ADC1_CONVST_EPWM_GPIO_NUM               ((uint16_t)0)
#define ADS9327_EXT_ADC1_CONVST_PIN_CONFIG_EPWM    GPIO_0_EPWM1_A
#define ADS9327_EXT_ADC1_CONVST_PIN_CONFIG_GPIO    GPIO_0_GPIO0

//
// Configure external ADC clock frequency
// Hardware change is required to change the ADC sample rate
// See the ADS8588S datasheet for more information
//

#define ADS9327_EXT_ADC1_SAMPLING_FREQUENCY_KHZ           (2000)
#define ADS9327_EXT_ADC1_CONVST_ONTIME_NS                 ((uint32_t)(10)) 

#define ADS9327_EXT_ADC1_CONVST_FREQUENCY_HZ              ((float32_t)ADS9327_EXT_ADC1_SAMPLING_FREQUENCY_KHZ * 1000)
#define ADS9327_EXT_ADC1_CONVST_EPWM_PERIOD_TICKS         ((uint32_t)(ADS9327_EPWM_HZ/ADS9327_EXT_ADC1_CONVST_FREQUENCY_HZ))
#define ADS9327_EXT_ADC1_CONVST_EPWM_PERIOD               ((uint16_t)(ADS9327_EXT_ADC1_CONVST_EPWM_PERIOD_TICKS - 1))
#define ADS9327_EXT_ADC1_CONVST_ONTIME_TICKS              ((uint16_t)(ADS9327_EXT_ADC1_CONVST_ONTIME_NS*ADS9327_EPWM_HZ/1000000000))
