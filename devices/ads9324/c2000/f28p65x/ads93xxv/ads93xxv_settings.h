/*
 * @brief: User-customized content for the project
 *
 * @copyright Copyright (C) 2025-2026 Texas Instruments Incorporated - http://www.ti.com/
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
 */

#ifndef ADS93XXV_SETTINGS_H_
#define ADS93XXV_SETTINGS_H_

#ifdef __cplusplus

extern "C" {
#endif

#include "driverlib.h"
#include "device.h"


//
// Set the BUILD_NUM to 1 or 2 or 3
//
#define ADS93XXV_BUILD_NUM (2)


#define ADS93XXV_16B_SPI_FIFO (1)
#define ADS93XXV_16B_DMA (2)
#define ADS93XXV_24B_DMA (3)


#define ADS93XXV_TEST_PATT_EN (true)

//
// ADC channels
//

#define ADS93XXV_CHANNELS 16

#define ADS93XXV_16B_FSR_CODE (float32_t)(65536.0)


// Device clocking conditions
//==============================================================================
//
#define ADS93XXV_SYSCLK_HZ        ((float32_t)200 * 1000000)
#define ADS93XXV_SYSCLK_NS        ((float32_t)1000000000 / ADS93XXV_SYSCLK_HZ)
#define ADS93XXV_EPWM_HZ          ((float32_t)200 * 1000000)
#define ADS93XXV_EPWM_NS          ((float32_t)1000000000 / ADS93XXV_EPWM_HZ)
#define ADS93XXV_INTOSC_HZ        ((float32_t) 25 * 1000000)

#define ADS93XXV_EPWM_EPWMCLK_DIV          EPWM_CLOCK_DIVIDER_1
#define ADS93XXV_EPWM_HSCLK_DIV            EPWM_HSCLOCK_DIVIDER_1

#if(ADS93XXV_EPWM_HSCLK_DIV == EPWM_HSCLOCK_DIVIDER_1)
    #define ADS93XXV_EPWM_TOTAL_CLKDIV     ((uint16_t)0x1 << ADS93XXV_EPWM_EPWMCLK_DIV)
#else
    #define ADS93XXV_EPWM_TOTAL_CLKDIV     (((uint16_t)0x1 << ADS93XXV_EPWM_EPWMCLK_DIV) * (ADS93XXV_EPWM_HSCLK_DIV << 1))
#endif

//
// Configure external ADC CONVST frequency
//
#define ADS93XXV_CONVST_EPWM_BASE                 EPWM4_BASE
#define ADS93XXV_CONVST_EPWM_NUM                  ((uint16_t)4)
#define ADS93XXV_CONVST_EPWM_GPIO_NUM               ((uint16_t)6)
#define ADS93XXV_CONVST_PIN_CONFIG_EPWM    GPIO_6_EPWM4_A
#define ADS93XXV_CONVST_PIN_CONFIG_GPIO    GPIO_6_GPIO6


#define ADS93XXV_SAMPLING_FREQUENCY_KHZ           (200)


#define ADS93XXV_CONVST_FREQUENCY_HZ              ((float32_t)ADS93XXV_SAMPLING_FREQUENCY_KHZ * 1000)
#define ADS93XXV_CONVST_EPWM_PERIOD_TICKS         ((uint32_t)(ADS93XXV_EPWM_HZ/ADS93XXV_CONVST_FREQUENCY_HZ))
#define ADS93XXV_CONVST_EPWM_PERIOD               ((uint16_t)(ADS93XXV_CONVST_EPWM_PERIOD_TICKS - 1))

#define ADS93XXV_CONVST_ONTIME_NS                 ((uint32_t)(1000000/ADS93XXV_SAMPLING_FREQUENCY_KHZ/2))
#define ADS93XXV_CONVST_ONTIME_TICKS              ((uint16_t)(ADS93XXV_CONVST_ONTIME_NS*ADS93XXV_EPWM_HZ/1000000000))


#define ADS93XXV_OUTPUT_DATA_RATE_KHZ           (ADS93XXV_SAMPLING_FREQUENCY_KHZ/ADS93XXV_OSR)
//
// SPI settings
//
#define ADS93XXV_SPI_CLOCK_FREQ_HZ 25000000

#define ADS93XXV_SPI_DATAWIDTH_BIT 8

#define ADS93XXV_SPI_BASE       SPIA_BASE
#define ADS93XXV_INT_SPI_RX     INT_SPIA_RX
#define ADS93XXV_ISR_SPI_RXFIFO ISR2
#define ADS93XXV_SPI_RXFIFO_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP6

//
// External ADC - SPI GPIO configuration
//
#define ADS93XXV_SDIN_GPIO_NUM (16U)
#define ADS93XXV_SDIN_PIN_CONFIG GPIO_16_SPIA_PICO

#define ADS93XXV_SDOUT_GPIO_NUM (17U)
#define ADS93XXV_SDOUT_PIN_CONFIG GPIO_17_SPIA_POCI

#define ADS93XXV_SCLK_GPIO_NUM (18U)
#define ADS93XXV_SCLK_PIN_CONFIG GPIO_18_SPIA_CLK

#define ADS93XXV_CS_GPIO_NUM (35U)
#define ADS93XXV_CS_PIN_CONFIG_GPIO GPIO_35_SPIA_PTE

#define ADS93XXV_RST_GPIO_NUM (33U)
#define ADS93XXV_RST_PIN_CONFIG_GPIO GPIO_33_GPIO33

#define BT4CH_EXT_SPI_NULL_VALUE ((uint16_t)0x0000)

#define ADS93XXV_DRDY_GPIO_NUM (32U)
#define ADS93XXV_DRDY_PIN_CONFIG_GPIO GPIO_32_GPIO32

//
// External ADC - BUSY signal interrupt configuration
//
#define ADS93XXV_INT_DRDY INT_XINT1
#define ADS93XXV_HAL_INT_DRDY_ISR ISR1
#define ADS93XXV_INT_DRDY_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP1

//
// GPIO for debug
//
#define ADS93XXV_HAL_GPIO_NUM_GPIO0 48
#define ADS93XXV_HAL_GPIO_PIN_CONFIG_GPIO0 GPIO_48_GPIO48
//
// GPIO31 - GPIO Settings
//
#define ADS93XXV_HAL_GPIO_NUM_GPIO1 31
#define ADS93XXV_HAL_GPIO_PIN_CONFIG_GPIO1 GPIO_31_GPIO31


//
// Configure DMA for Transmit
//
#if(ADS93XXV_BUILD_NUM==ADS93XXV_16B_DMA)
#define ADS93XXV_HAL_SPI_TX_DMA_BURSTSIZE 8U
#define ADS93XXV_HAL_SPI_TX_DMA_TRANSFERSIZE 2U
#define ADS93XXV_HAL_SPI_RX_DMA_BURSTSIZE 8U
#define ADS93XXV_HAL_SPI_RX_DMA_TRANSFERSIZE 2U
#elif(ADS93XXV_BUILD_NUM==ADS93XXV_24B_DMA)
#define ADS93XXV_HAL_SPI_TX_DMA_BURSTSIZE 8U
#define ADS93XXV_HAL_SPI_TX_DMA_TRANSFERSIZE 6U
#define ADS93XXV_HAL_SPI_RX_DMA_BURSTSIZE 8U
#define ADS93XXV_HAL_SPI_RX_DMA_TRANSFERSIZE 6U
#endif

#define ADS93XXV_HAL_SPI_TX_DMA_TRIGGER DMA_TRIGGER_SPIATX
#define ADS93XXV_HAL_SPI_TX_DMA_ADDRESS ((uint16_t *)(SPIA_BASE + SPI_O_TXBUF))

#define ADS93XXV_HAL_SPI_TX_DMA_BASE DMA_CH5_BASE

#define ADS93XXV_HAL_SPI_TX_DMA_SRC_WRAPSIZE 65535U
#define ADS93XXV_HAL_SPI_TX_DMA_DEST_WRAPSIZE 65535U

//
// Configure DMA for Receive
//
#define ADS93XXV_HAL_SPI_RX_DMA_TRIGGER DMA_TRIGGER_SPIARX
#define ADS93XXV_HAL_SPI_RX_DMA_ADDRESS ((uint16_t *)(ADS93XXV_SPI_BASE + SPI_O_RXBUF))

#define ADS93XXV_HAL_SPI_RX_DMA_BASE DMA_CH6_BASE

#define ADS93XXV_HAL_SPI_RX_DMA_SRC_WRAPSIZE 65535U
#define ADS93XXV_HAL_SPI_RX_DMA_DEST_WRAPSIZE 65535U

#define ADS93XXV_INT_HAL_RX_DMA INT_DMA_CH6
#define ADS93XXV_INT_HAL_RX_DMA_INTERRUPT_ACK_GROUP INTERRUPT_ACK_GROUP7

#define ADS93XXV_INT_RX_DMA_ISR ISR3

//
// Define the factor for internal ADC sampling
// Internal ADC sampling rate = Switching Frequency / ADS93XXV_EPWM_ADC_TRIGGER_PRESCALE
// ADS93XXV_EPWM_ADC_TRIGGER_PRESCALE is used to EPWM ADC trigger prescale value
//
#define ADS93XXV_EPWM_ADC_TRIGGER_PRESCALE (15U)

#ifdef __cplusplus
}
#endif                                  /* extern "C" */

#endif /* BT4CH_USER_SETTINGS_H_ */

