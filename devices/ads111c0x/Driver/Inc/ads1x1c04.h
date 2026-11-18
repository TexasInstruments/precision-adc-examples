/**
 * @file ads1x1c04.h
 * @brief Register map definitions for the ADS111C04 device family.
 * @copyright Copyright (C) 2026 Texas Instruments Incorporated - http://www.ti.com/
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
#ifndef ADS111C04_H_
#define ADS111C04_H_

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "/Driver/hal.h"
#include "Driver/crc.h"



// ========== I2C ADDRESSES ==========
#define ADDR_0                                                  ((uint8_t)  0x48)
#define ADDR_1                                                  ((uint8_t)  0x49)
#define ADDR_2                                                  ((uint8_t)  0x4A)
#define ADDR_3                                                  ((uint8_t)  0x4B)

// ========== CONSTANTS ==========
#define NUM_REGISTERS                                           ((uint8_t)  5)
#define MAX_REGISTER_ADDRESS                                    ((uint8_t)  4)
#define WLENGTH                                                 1

// ============VARIABLES ==========
extern const char *registerNames[];
extern uint16_t registerMap[NUM_REGISTERS];

// ========== FUNCTION PROTOTYPES ==========

/**
 * @brief Starts up the ADC driver (I2C variant).
 *
 * Waits 50 ms for the device to power up, loads the local register map with
 * the default values, then reads the CONFIG register from the device to
 * synchronize the local copy.
 */
void     adcStartup(void);

/**
 * @brief Reads the CONFIG register over I2C.
 *
 * The result is also stored in the local register map.
 *
 * @return 16-bit CONFIG register contents.
 */
uint16_t readConfigRegister(void);

/**
 * @brief Reads a 16-bit register over I2C.
 *
 * Writes the register address to the address pointer, then reads two bytes.
 * The result is also stored in the local register map.
 *
 * @param[in] reg Register address (must be <= MAX_REGISTER_ADDRESS).
 *
 * @return 16-bit register contents.
 */
uint16_t readSingleRegister(uint8_t reg);

/**
 * @brief Reads the conversion result from the device.
 *
 * Sets the address pointer to the conversion register, then reads two bytes.
 * The result is also stored in the local register map.
 *
 * @return 16-bit conversion register contents.
 */
uint16_t  readConversionRegister(void);

/**
 * @brief Writes a 16-bit value to a register over I2C.
 *
 * @param[in] reg  Register address (must be <= MAX_REGISTER_ADDRESS).
 * @param[in] data 16-bit value to write.
 *
 * @return true (the write is not read back for confirmation).
 */
bool     writeSingleRegister(uint8_t reg, uint16_t data);

/**
 * @brief Measures the internal temperature sensor.
 *
 * Temporarily selects the internal temperature sensor, starts a conversion
 * and reads the result, then deselects the sensor again. The result has a
 * resolution of 0.03125 degC/LSB (16-bit device) or 0.125 degC/LSB
 * (12-bit device).
 *
 * @return Internal temperature in degrees Celsius.
 */
float    measureInternalTemperature(void);

/**
 * @brief Starts a conversion and returns the result.
 *
 * Waits (up to 1000 ms) for nDRDY to signal a new conversion, then reads the
 * conversion register. The conversion must already have been started, for
 * example with startAdcConversion().
 *
 * @return Signed 16-bit conversion result.
 */
int16_t  readData(void);

/**
 * @brief Starts a conversion by setting the OS bit in the CONFIG register.
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

// ============Helper functions ===========

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

//**********************************************************************************
//
// Register definitions
//
//**********************************************************************************


/* Register 0x007F (ADDRESS_POINTER) definition
 * |----------------------------------------------------------------------------------------
 * |  Bit 15  |  Bit 14  |  Bit 13  |  Bit 12  |  Bit 11  |  Bit 10  |   Bit 9  |   Bit 8  |
 * |----------------------------------------------------------------------------------------
 * |          |          |          |          |          |          |          |          |
 * |----------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------|
 * |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * ----------------------------------------------------------------------------------------|
 * |                     RESERVED[4:0]                    |             P[2:0]             |
 * ----------------------------------------------------------------------------------------|
 */

    /* ADDRESS_POINTER register */
    #define ADDRESS_POINTER_ADDRESS                               ((uint16_t) 0x007F)
    #define ADDRESS_POINTER_DEFAULT                               ((uint16_t) 0x0000)

    /* P field */
    #define ADDRESS_POINTER_P_MASK                                ((uint16_t) 0x0007)
    #define ADDRESS_POINTER_P_BITOFFSET                           (0)
    #define ADDRESS_POINTER_P_CONVERSION_DATA_REGISTER            ((uint16_t) 0x0000)    // DEFAULT
    #define ADDRESS_POINTER_P_CONFIGURATION_REGISTER              ((uint16_t) 0x0001)
    #define ADDRESS_POINTER_P_ID_REGISTER                         ((uint16_t) 0x0004)


/* Register 0x0000 (DATA) definition
 * |----------------------------------------------------------------------------------------
 * |  Bit 15  |  Bit 14  |  Bit 13  |  Bit 12  |  Bit 11  |  Bit 10  |   Bit 9  |   Bit 8  |
 * |----------------------------------------------------------------------------------------
 * |                                 CONVERSION_DATA[15:8]
 * |----------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------|
 * |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * ----------------------------------------------------------------------------------------|
 *                                    CONVERSION_DATA[7:0]
 * ----------------------------------------------------------------------------------------|
 */

    /* DATA register */
    #define DATA_ADDRESS                                          ((uint16_t) 0x0000)
    #define DATA_DEFAULT                                          ((uint16_t) 0x0000)

    /* CONVERSION_DATA field */
    #define DATA_CONVERSION_DATA_MASK                             ((uint16_t) 0xFFFF)
    #define DATA_CONVERSION_DATA_BITOFFSET                        (0)


/* Register 0x0001 (CONFIG) definition
 * |----------------------------------------------------------------------------------------
 * |  Bit 15  |  Bit 14  |  Bit 13  |  Bit 12  |  Bit 11  |  Bit 10  |   Bit 9  |   Bit 8  |
 * |----------------------------------------------------------------------------------------
 * |    OS    |                       MUX[4:0]                       |       PGA[2:1]
 * |----------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------|
 * |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * ----------------------------------------------------------------------------------------|
 *   PGA[0:0] |   MODE   |       DR[1:0]       |    TS    |          RESERVED[2:0]         |
 * ----------------------------------------------------------------------------------------|
 */

    /* CONFIG register */
    #define CONFIG_ADDRESS                                        ((uint16_t) 0x0001)
    #define CONFIG_DEFAULT                                        ((uint16_t) 0x8140)

    /* OS field */
    #define CONFIG_OS_MASK                                        ((uint16_t) 0x8000)
    #define CONFIG_OS_BITOFFSET                                   (15)

    /* MUX field */
    #define CONFIG_MUX_MASK                                       ((uint16_t) 0x7C00)
    #define CONFIG_MUX_BITOFFSET                                  (10)
    #define CONFIG_MUX_AINP_AIN0_AINN_GND                         ((uint16_t) 0x0000)    // DEFAULT
    #define CONFIG_MUX_AINP_AIN1_AINN_GND                         ((uint16_t) 0x0400)
    #define CONFIG_MUX_AINP_AIN2_AINN_GND                         ((uint16_t) 0x0800)
    #define CONFIG_MUX_AINP_AIN3_AINN_GND                         ((uint16_t) 0x0C00)
    #define CONFIG_MUX_AINP_AIN4_AINN_GND                         ((uint16_t) 0x1000)
    #define CONFIG_MUX_AINP_AIN5_AINN_GND                         ((uint16_t) 0x1400)
    #define CONFIG_MUX_AINP_AIN6_AINN_GND                         ((uint16_t) 0x1800)
    #define CONFIG_MUX_AINP_AIN7_AINN_GND                         ((uint16_t) 0x1C00)
    #define CONFIG_MUX_AINP_AIN0_AINN_AIN1                        ((uint16_t) 0x2000)
    #define CONFIG_MUX_AINP_AIN1_AINN_AIN2                        ((uint16_t) 0x2400)
    #define CONFIG_MUX_AINP_AIN2_AINN_AIN3                        ((uint16_t) 0x2800)
    #define CONFIG_MUX_AINP_AIN3_AINN_AIN4                        ((uint16_t) 0x2C00)
    #define CONFIG_MUX_AINP_AIN4_AINN_AIN5                        ((uint16_t) 0x3000)
    #define CONFIG_MUX_AINP_AIN5_AINN_AIN6                        ((uint16_t) 0x3400)
    #define CONFIG_MUX_AINP_AIN6_AINN_AIN7                        ((uint16_t) 0x3800)
    #define CONFIG_MUX_AINP_AIN0_AINN_AIN7                        ((uint16_t) 0x3C00)
    #define CONFIG_MUX_AINP_AIN1_AINN_AIN7                        ((uint16_t) 0x4000)
    #define CONFIG_MUX_AINP_AIN2_AINN_AIN7                        ((uint16_t) 0x4400)
    #define CONFIG_MUX_AINP_AIN3_AINN_AIN7                        ((uint16_t) 0x4800)
    #define CONFIG_MUX_AINP_AIN4_AINN_AIN7                        ((uint16_t) 0x4C00)
    #define CONFIG_MUX_AINP_AIN5_AINN_AIN7                        ((uint16_t) 0x5000)
    #define CONFIG_MUX_AINP_AIN0_AINN_AIN4                        ((uint16_t) 0x5400)
    #define CONFIG_MUX_AINP_AIN1_AINN_AIN4                        ((uint16_t) 0x5800)
    #define CONFIG_MUX_AINP_AIN2_AINN_AIN4                        ((uint16_t) 0x5C00)
    #define CONFIG_MUX_AINP_AIN5_AINN_AIN4                        ((uint16_t) 0x6000)
    #define CONFIG_MUX_AINP_AIN6_AINN_AIN4                        ((uint16_t) 0x6400)
    #define CONFIG_MUX_AINP_AIN7_AINN_AIN4                        ((uint16_t) 0x6800)
    #define CONFIG_MUX_AINP_AIN0_AINN_AIN3                        ((uint16_t) 0x6C00)
    #define CONFIG_MUX_AINP_AIN1_AINN_AIN3                        ((uint16_t) 0x7000)
    #define CONFIG_MUX_AINP_AIN5_AINN_AIN3                        ((uint16_t) 0x7400)
    #define CONFIG_MUX_AINP_VDD_AINN_GND                          ((uint16_t) 0x7800)
    #define CONFIG_MUX_SHORTED_INPUTS                             ((uint16_t) 0x7C00)

    /* PGA field */
    #define CONFIG_PGA_MASK                                       ((uint16_t) 0x0380)
    #define CONFIG_PGA_BITOFFSET                                  (7)
    #define CONFIG_PGA_GAIN_1_6                                   ((uint16_t) 0x0000)
    #define CONFIG_PGA_GAIN_1_4                                   ((uint16_t) 0x0080)
    #define CONFIG_PGA_GAIN_1_2                                   ((uint16_t) 0x0100)    // DEFAULT
    #define CONFIG_PGA_GAIN_1                                     ((uint16_t) 0x0180)
    #define CONFIG_PGA_GAIN_2                                     ((uint16_t) 0x0200)
    #define CONFIG_PGA_GAIN_4                                     ((uint16_t) 0x0280)
    #define CONFIG_PGA_GAIN_8                                     ((uint16_t) 0x0300)
    #define CONFIG_PGA_GAIN_16                                    ((uint16_t) 0x0380)

    /* MODE field */
    #define CONFIG_MODE_MASK                                      ((uint16_t) 0x0040)
    #define CONFIG_MODE_BITOFFSET                                 (6)
    #define CONFIG_MODE_CONTINUOUS_CONVERSION_MODE                ((uint16_t) 0x0000)
    #define CONFIG_MODE_SINGLE_SHOT_CONVERSION_MODE               ((uint16_t) 0x0040)    // DEFAULT

    /* DR field */
    #define CONFIG_DR_MASK                                        ((uint16_t) 0x0030)
    #define CONFIG_DR_BITOFFSET                                   (4)
    #define CONFIG_DR_10SPS                                       ((uint16_t) 0x0000)    // DEFAULT
    #define CONFIG_DR_100SPS                                      ((uint16_t) 0x0010)
    #define CONFIG_DR_1KSPS                                       ((uint16_t) 0x0020)
    #define CONFIG_DR_4KSPS                                       ((uint16_t) 0x0030)

    /* TS field */
    #define CONFIG_TS_MASK                                        ((uint16_t) 0x0008)
    #define CONFIG_TS_BITOFFSET                                   (3)
    #define CONFIG_TS_MUX_SELECTED_FOR_MEASUREMENT                ((uint16_t) 0x0000)    // DEFAULT
    #define CONFIG_TS_TEMPERATURE_SENSOR_SELECTED_FOR_MEASUREMENT ((uint16_t) 0x0008)


/* Register 0x0004 (ID) definition
 * |----------------------------------------------------------------------------------------
 * |  Bit 15  |  Bit 14  |  Bit 13  |  Bit 12  |  Bit 11  |  Bit 10  |   Bit 9  |   Bit 8  |
 * |----------------------------------------------------------------------------------------
 * |                                     RESERVED[13:6]
 * |----------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------|
 * |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * ----------------------------------------------------------------------------------------|
 *                            RESERVED[5:0]                          |   RESOLUTION[1:0]   |
 * ----------------------------------------------------------------------------------------|
 */

    /* ID register */
    #define ID_ADDRESS                                            ((uint16_t) 0x0004)
    #define ID_DEFAULT                                            ((uint16_t) 0x0002)

    /* RESOLUTION field */
    #define ID_RESOLUTION_MASK                                    ((uint16_t) 0x0003)
    #define ID_RESOLUTION_BITOFFSET                               (0)
    #define ID_RESOLUTION_12BIT                                   ((uint16_t) 0x0001)
    #define ID_RESOLUTION_16BIT                                   ((uint16_t) 0x0002)    // DEFAULT

// ========== REGISTER MACROS ==========

#define MASKED_REG_DATA(addr, mask)     (getRegisterValue(addr) & (mask))

#define RESOLUTION_IS_16_BIT    ((bool) MASKED_REG_DATA(ID_ADDRESS, ID_RESOLUTION_MASK))



#endif /* ADS111C04_H_ */
