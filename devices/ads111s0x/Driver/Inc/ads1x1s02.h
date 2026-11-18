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
 * @file ads1x1s02.h
 * @brief Register map definitions for the ADS111S02 device family.
 * @copyright Copyright (C) 2026 Texas Instruments Incorporated - http://www.ti.com/
 */
#ifndef ADS111S02_H
#define ADS111S02_H

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "/Driver/hal.h"
#include "Driver/crc.h"


// ========== CONSTANTS ==========
#define NUM_REGISTERS                                           ((uint8_t)  2)
#define MAX_REGISTER_ADDRESS                                    ((uint8_t)  1)
#define WLENGTH                                                 1

// ============VARIABLES ==========
extern const char *registerNames[];
extern uint16_t registerMap[NUM_REGISTERS];

// ========== FUNCTION PROTOTYPES ==========

/**
 * @brief Starts up the ADC driver (SPI variant).
 *
 * Waits 50 ms for the device to power up, loads the local register map with
 * the default values, then reads the CONFIG register from the device.
 */
void     adcStartup(void);

/**
 * @brief Waits for a new conversion and returns the result.
 *
 * Waits (up to 1000 ms) for nDRDY to signal a new conversion, then reads the
 * conversion register.
 *
 * @return Signed 16-bit conversion result.
 */
int16_t  readData(void);

/**
 * @brief Reads the CONFIG register over SPI.
 *
 * @return 16-bit CONFIG register contents.
 */
uint16_t readConfigRegister(void);

/**
 * @brief Reads the conversion register over SPI.
 *
 * @return 16-bit conversion register contents.
 */
uint16_t readConversionRegister(void);

/**
 * @brief Writes a 16-bit value to a register over SPI.
 *
 * @param[in] reg  Register address (must be <= MAX_REGISTER_ADDRESS).
 * @param[in] data 16-bit value to write.
 *
 * @return The 16-bit word clocked back from the device during the write.
 */
uint16_t writeSingleRegister(uint8_t reg, uint16_t data);

/**
 * @brief Starts a conversion by setting the single-shot start bit (SS) in the CONFIG register.
 *
 * @return Value written to the CONFIG register.
 */
uint16_t startAdcConversion(void);

/**
 * @brief Stops conversions by selecting single-shot conversion mode.
 *
 * @return Value written to the CONFIG register.
 */
uint16_t stopAdcConversion(void);

/**
 * @brief Measures the internal temperature sensor.
 *
 * Temporarily selects the internal temperature sensor, performs a conversion
 * and deselects the sensor again. The result has a resolution of
 * 0.03125 degC/LSB (16-bit device) or 0.125 degC/LSB (12-bit device).
 *
 * @return Internal temperature in degrees Celsius.
 */
float    measureInternalTemperature(void);

/**
 * @brief Returns the locally cached value of a register.
 *
 * No communication with the device takes place.
 *
 * @param[in] reg Register address (must be <= MAX_REGISTER_ADDRESS).
 *
 * @return Cached 16-bit register value.
 */
uint16_t getRegisterValue(uint8_t reg);

/**
 * @brief Returns the upper byte (bits 15:8) of a 16-bit word.
 *
 * @param[in] uint16_Word 16-bit input word.
 *
 * @return Upper byte of the word.
 */
uint8_t  upperByte(uint16_t uint16_Word);

/**
 * @brief Returns the lower byte (bits 7:0) of a 16-bit word.
 *
 * @param[in] uint16_Word 16-bit input word.
 *
 * @return Lower byte of the word.
 */
uint8_t  lowerByte(uint16_t uint16_Word);

/**
 * @brief Combines two bytes into a 16-bit word.
 *
 * @param[in] upperByte Most significant byte.
 * @param[in] lowerByte Least significant byte.
 *
 * @return 16-bit word (upperByte << 8 | lowerByte).
 */
uint16_t combineBytes(uint8_t upperByte, uint8_t lowerByte);

/**
 * @brief Sign extends a big-endian 16-bit two's complement value to 32 bits.
 *
 * @param[in] dataBytes Array holding the MSB at index 0 and the LSB at index 1.
 *
 * @return Sign-extended 32-bit value.
 */
int32_t  signExtend(const uint8_t dataBytes[]);


// ========== REGISTER 0x00 - CONVERSION ==========
// -----------------------------------------------------------------------------------------------------------------
// |  Bit 15 ... Bit 0 : CONV[15:0]                                                                               |
// -----------------------------------------------------------------------------------------------------------------
#define CONVERSION_ADDRESS                                      ((uint16_t) 0x00)
#define CONVERSION_DEFAULT                                      ((uint16_t) 0x0000)
#define CONVERSION_CONV_MASK                                    ((uint16_t) 0xFFFF)


/* Register 0x0001 (CONFIG) definition
 * |--------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |
 * |--------------------------------------------------------------------------------------------------------
 * |     SS     |               MUX[2:0]               |               PGA[2:0]               |    MODE    |
 * |--------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------|
 * |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------|
 * |         DR[1:0]         | RESOLUTION |     TS     | PULL_UP_EN |      WRITE_EN[1:0]      |  RESERVED  |
 * --------------------------------------------------------------------------------------------------------|
 */

    /* CONFIG register */
    #define CONFIG_ADDRESS                                        ((uint16_t) 0x0001)
    #define CONFIG_DEFAULT                                        ((uint16_t) 0x052B)

    /* SS field */
    #define CONFIG_SS_MASK                                        ((uint16_t) 0x8000)
    #define CONFIG_SS_BITOFFSET                                   (15)
    #define CONFIG_SS_NO_EFFECT                                   ((uint16_t) 0x0000)    // DEFAULT
    #define CONFIG_SS_START_SINGLE_CONVERSION                     ((uint16_t) 0x8000)

    /* MUX field */
    #define CONFIG_MUX_MASK                                       ((uint16_t) 0x7000)
    #define CONFIG_MUX_BITOFFSET                                  (12)
    #define CONFIG_MUX_AINP_AIN0_AINN_AIN1                       ((uint16_t) 0x0000)    // DEFAULT
    #define CONFIG_MUX_AINP_AIN0_AINN_AIN3                       ((uint16_t) 0x1000)
    #define CONFIG_MUX_AINP_AIN1_AINN_AIN3                       ((uint16_t) 0x2000)
    #define CONFIG_MUX_AINP_AIN2_AINN_AIN3                       ((uint16_t) 0x3000)
    #define CONFIG_MUX_AINP_AIN0_AINN_GND                        ((uint16_t) 0x4000)
    #define CONFIG_MUX_AINP_AIN1_AINN_GND                        ((uint16_t) 0x5000)
    #define CONFIG_MUX_AINP_AIN2_AINN_GND                        ((uint16_t) 0x6000)
    #define CONFIG_MUX_AINP_AIN3_AINN_GND                        ((uint16_t) 0x7000)

    /* PGA field */
    #define CONFIG_PGA_MASK                                       ((uint16_t) 0x0E00)
    #define CONFIG_PGA_BITOFFSET                                  (9)
    #define CONFIG_PGA_GAIN_1_6                                   ((uint16_t) 0x0000)
    #define CONFIG_PGA_GAIN_1_4                                   ((uint16_t) 0x0200)
    #define CONFIG_PGA_GAIN_1_2                                   ((uint16_t) 0x0400)    // DEFAULT
    #define CONFIG_PGA_GAIN_1                                     ((uint16_t) 0x0600)
    #define CONFIG_PGA_GAIN_2                                     ((uint16_t) 0x0800)
    #define CONFIG_PGA_GAIN_4                                     ((uint16_t) 0x0A00)
    #define CONFIG_PGA_GAIN_8                                     ((uint16_t) 0x0C00)
    #define CONFIG_PGA_GAIN_16                                    ((uint16_t) 0x0E00)

    /* MODE field */
    #define CONFIG_MODE_MASK                                      ((uint16_t) 0x0100)
    #define CONFIG_MODE_BITOFFSET                                 (8)
    #define CONFIG_MODE_CONTINUOUS_CONVERSION_MODE                ((uint16_t) 0x0000)
    #define CONFIG_MODE_SINGLE_SHOT_CONVERSION_MODE               ((uint16_t) 0x0100)    // DEFAULT

    /* DR field */
    #define CONFIG_DR_MASK                                        ((uint16_t) 0x00C0)
    #define CONFIG_DR_BITOFFSET                                   (6)
    #define CONFIG_DR_10SPS                                       ((uint16_t) 0x0000)    // DEFAULT
    #define CONFIG_DR_100SPS                                      ((uint16_t) 0x0040)
    #define CONFIG_DR_1KSPS                                       ((uint16_t) 0x0080)
    #define CONFIG_DR_4KSPS                                       ((uint16_t) 0x00C0)

    /* RESOLUTION field */
    #define CONFIG_RESOLUTION_MASK                                ((uint16_t) 0x0020)
    #define CONFIG_RESOLUTION_BITOFFSET                           (5)
    #define CONFIG_RESOLUTION_12BIT                               ((uint16_t) 0x0000)
    #define CONFIG_RESOLUTION_16BIT                               ((uint16_t) 0x0020)    // DEFAULT

    /* TS field */
    #define CONFIG_TS_MASK                                        ((uint16_t) 0x0010)
    #define CONFIG_TS_BITOFFSET                                   (4)
    #define CONFIG_TS_MUX_SELECTED_FOR_MEASUREMENT                ((uint16_t) 0x0000)    // DEFAULT
    #define CONFIG_TS_TEMPERATURE_SENSOR_SELECTED_FOR_MEASUREMENT ((uint16_t) 0x0010)

    /* PULL_UP_EN field */
    #define CONFIG_PULL_UP_EN_MASK                                ((uint16_t) 0x0008)
    #define CONFIG_PULL_UP_EN_BITOFFSET                           (3)
    #define CONFIG_PULL_UP_EN_DISABLED                            ((uint16_t) 0x0000)
    #define CONFIG_PULL_UP_EN_ENABLED                             ((uint16_t) 0x0008)    // DEFAULT

    /* WRITE_EN field */
    #define CONFIG_WRITE_EN_MASK                                  ((uint16_t) 0x0006)
    #define CONFIG_WRITE_EN_BITOFFSET                             (1)
    #define CONFIG_WRITE_EN_INVALID_DATA                          ((uint16_t) 0x0000)
    #define CONFIG_WRITE_EN_VALID_DATA                            ((uint16_t) 0x0002)    // DEFAULT
    #define CONFIG_WRITE_EN_INVALID_DATA_A                        ((uint16_t) 0x0004)
    #define CONFIG_WRITE_EN_INVALID_DATA_B                        ((uint16_t) 0x0006)


// ========== REGISTER MACROS ==========

#define MASKED_REG_DATA(addr, mask)     (getRegisterValue(addr) & (mask))

#define RESOLUTION_IS_16_BIT    ((bool) MASKED_REG_DATA(CONFIG_ADDRESS, CONFIG_RESOLUTION_MASK))

#endif /* ADS111S02_H_ */
