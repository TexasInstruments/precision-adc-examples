/**
 * @file ads1118.h
 *
 * @brief This header file contains all register map definitions for the ADS1118 device family.
 * @warning This software utilizes TI Drivers
 *
 * @copyright Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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
#ifndef ADS1118_H
#define ADS1118_H

//****************************************************************************
//
// Standard Libraries
//
//****************************************************************************
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
//****************************************************************************
//
// Custom Libraries
//
//****************************************************************************
#include "hal.h"

//****************************************************************************
//
// Global variables
//
//****************************************************************************
extern const char *adcRegisterNames[];

//****************************************************************************
//
// Function prototypes
//
//****************************************************************************
void adcStartup(void);
int16_t readData(void);
uint16_t readSingleRegister(uint8_t address);
uint16_t writeSingleRegister(uint8_t address, uint16_t data);
uint16_t startAdcConversion(void);
uint16_t stopAdcConversion(void);
float ads1118_measure_internal_temperature_example(void);

// Getter functions
uint16_t    getRegisterValue(uint8_t address);

// Helper functions
uint8_t     upperByte(uint16_t uint16_Word);
uint8_t     lowerByte(uint16_t uint16_Word);
uint16_t    combineBytes(uint8_t upperByte, uint8_t lowerByte);
int32_t     signExtend(const uint8_t dataBytes[]);

//****************************************************************************
//
// Register macros
//
//****************************************************************************

#define WLENGTH     1

//**********************************************************************************
//
// Device commands
//
//**********************************************************************************

//****************************************************************************
//
// Constants
//
//****************************************************************************
/* The ADS1118 does not have addressable registers, but a numbered register concept
 * is used to maintain synchronization between the device and the firmware.
 * Register 0 can be considered the Conversion register and Register 1 can be
 * considered the Configuration register.
 *
 */
#define NUM_REGISTERS                           ((uint8_t) 2)
/* Maximum register address or address of the last register in the regmap */
#define MAX_REGISTER_ADDRESS                    ((uint8_t) 1)

//****************************************************************************
//
// Register definitions
//
//****************************************************************************

/* NOTE: Whenever possible, macro names (defined below) were derived from
 * datasheet defined names; however, updates to documentation or readability
 * may cause mismatches between names defined here in example code from those
 * shown in the device datasheet.
 */


/* Register 0x00 (CONVERSION) definition
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |  Bit 15  |  Bit 14  |  Bit 13  |  Bit 12  |  Bit 11  |  Bit 10  |   Bit 9  |   Bit 8  |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                    CONV[15:0]                                                                                   |
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CONVERSION register address */
    #define CONVERSION_ADDRESS                                              ((uint16_t) 0x00)

    /* CONVERSION default (reset) value */
    #define CONVERSION_DEFAULT                                              ((uint16_t) 0x0000)

    /* CONVERSION register field masks */
    #define CONVERSION_CONV_MASK                                            ((uint16_t) 0xFFFF)


/* Register 0x01 (CONFIG) definition
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |  Bit 15  |  Bit 14  |  Bit 13  |  Bit 12  |  Bit 11  |  Bit 10  |   Bit 9  |   Bit 8  |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |    SS    |            MUX[2:0]            |            PGA[2:0]            |   MODE   |             DR[2:0]            |  TS_MODE |PULL_UP_EN|       NOP[1:0]      | RESERVED |
 * ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CONFIG register address */
    #define CONFIG_ADDRESS                                                  ((uint16_t) 0x01)

    /* CONFIG default (reset) value */
    #define CONFIG_DEFAULT                                                  ((uint16_t) 0x8583)

    /* CONFIG register field masks */
    #define CONFIG_SS_MASK                                                  ((uint16_t) 0x8000)
    #define CONFIG_MUX_MASK                                                 ((uint16_t) 0x7000)
    #define CONFIG_PGA_MASK                                                 ((uint16_t) 0x0E00)
    #define CONFIG_MODE_MASK                                                ((uint16_t) 0x0100)
    #define CONFIG_DR_MASK                                                  ((uint16_t) 0x00E0)
    #define CONFIG_TS_MODE_MASK                                             ((uint16_t) 0x0010)
    #define CONFIG_PULL_UP_EN_MASK                                          ((uint16_t) 0x0008)
    #define CONFIG_NOP_MASK                                                 ((uint16_t) 0x0006)
    #define CONFIG_RESERVED_MASK                                            ((uint16_t) 0x0001)

    /* SS field values */
    #define CONFIG_SS_NA                                                    ((uint16_t) 0x0000)
    #define CONFIG_SS_CONV_START                                            ((uint16_t) 0x8000)

    /* MUX field values */
    #define CONFIG_MUX_AIN0_AIN1                                            ((uint16_t) 0x0000)
    #define CONFIG_MUX_AIN0_AIN3                                            ((uint16_t) 0x1000)
    #define CONFIG_MUX_AIN1_AIN3                                            ((uint16_t) 0x2000)
    #define CONFIG_MUX_AIN2_AIN3                                            ((uint16_t) 0x3000)
    #define CONFIG_MUX_AIN0_GND                                             ((uint16_t) 0x4000)
    #define CONFIG_MUX_AIN1_GND                                             ((uint16_t) 0x5000)
    #define CONFIG_MUX_AIN2_GND                                             ((uint16_t) 0x6000)
    #define CONFIG_MUX_AIN3_GND                                             ((uint16_t) 0x7000)

    /* PGA field values */
    #define CONFIG_PGA_6p144V                                               ((uint16_t) 0x0000)
    #define CONFIG_PGA_4p096V                                               ((uint16_t) 0x0200)
    #define CONFIG_PGA_2p048V                                               ((uint16_t) 0x0400)
    #define CONFIG_PGA_1p024V                                               ((uint16_t) 0x0600)
    #define CONFIG_PGA_0p512V                                               ((uint16_t) 0x0800)
    #define CONFIG_PGA_0p256V                                               ((uint16_t) 0x0A00)

    /* MODE field values */
    #define CONFIG_MODE_CONT                                                ((uint16_t) 0x0000)
    #define CONFIG_MODE_SS                                                  ((uint16_t) 0x0100)

#ifdef ADS1018
    /* DR field values */
    #define CONFIG_DR_128SPS                                                ((uint16_t) 0x0000)
    #define CONFIG_DR_250SPS                                                ((uint16_t) 0x0020)
    #define CONFIG_DR_490SPS                                                ((uint16_t) 0x0040)
    #define CONFIG_DR_920SPS                                                ((uint16_t) 0x0060)
    #define CONFIG_DR_1600SPS                                               ((uint16_t) 0x0080)
    #define CONFIG_DR_2400SPS                                               ((uint16_t) 0x00A0)
    #define CONFIG_DR_3300_1SPS                                             ((uint16_t) 0x00C0)
    #define CONFIG_DR_3300_2SPS                                             ((uint16_t) 0x00E0)
#else
    /* DR field values */
    #define CONFIG_DR_8SPS                                                  ((uint16_t) 0x0000)
    #define CONFIG_DR_16SPS                                                 ((uint16_t) 0x0020)
    #define CONFIG_DR_32SPS                                                 ((uint16_t) 0x0040)
    #define CONFIG_DR_64SPS                                                 ((uint16_t) 0x0060)
    #define CONFIG_DR_128SPS                                                ((uint16_t) 0x0080)
    #define CONFIG_DR_250SPS                                                ((uint16_t) 0x00A0)
    #define CONFIG_DR_475SPS                                                ((uint16_t) 0x00C0)
    #define CONFIG_DR_860SPS                                                ((uint16_t) 0x00E0)
#endif

    /* TS_MODE field values */
    #define TS_MODE_ADC                                                     ((uint16_t) 0x0000)
    #define TS_MODE_TS                                                      ((uint16_t) 0x0010)

    /* PULL_UP_EN field values */
    #define PULL_UP_EN_DISABLE                                              ((uint16_t) 0x0000)
    #define PULL_UP_EN_ENABLE                                               ((uint16_t) 0x0080)

    /* NOP field values */
    #define NOP_INV_0                                                       ((uint16_t) 0x0000)
    #define NOP_VALID                                                       ((uint16_t) 0x0002)
    #define NOP_INV_2                                                       ((uint16_t) 0x0004)
    #define NOP_INV_3                                                       ((uint16_t) 0x0006)

    /* RESERVED field values */
#define RESERVED_VALUE                                                      ((uint16_t) 0x0001)

#endif
