/*
 * @file ads122c04.h
 *
 * @brief ADS122C04 Descriptor
 *
 * @copyright Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
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

#ifndef ADS122C04_H_
#define ADS122C04_H_

/*****************************************************************************
 *
 * Standard Libraries
 *
 ****************************************************************************/
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
/*****************************************************************************
 *
 * Custom Libraries
 *
 ****************************************************************************/
#include "hal.h"
/*****************************************************************************
 *
 * Global variables
 *
 ****************************************************************************/
#define ADS122C04

extern const char *adcRegisterNames[];
extern uint8_t ADC_RegisterMap[];
/*****************************************************************************
 *
 * Helpful Macros
 *
 ****************************************************************************/
/** Returns true if CRC enable bit is set */
#define CRC_ENABLED    ((bool) (ADC_RegisterMap[CONFIG2_ADDRESS] & CONFIG2_CRC_CRC16))

/** Returns true if inverted data enable bit is set */
#define INVERTED_ENABLED    ((bool) (ADC_RegisterMap[CONFIG2_ADDRESS] & CONFIG2_CRC_INVERTED))

/** Returns true if conversion counter enable bit is set */
#define COUNT_ENABLED    ((bool) (ADC_RegisterMap[CONFIG2_ADDRESS] & CONFIG2_DCNT_ENABLED))

/*****************************************************************************
 *
 * Function prototypes
 *
 ****************************************************************************/
uint8_t registerRead(uint8_t regnum);
uint8_t registerWrite(uint8_t regnum, uint8_t data);
uint8_t sendCommand(uint8_t op_code);
int32_t dataRead(uint8_t *dCount, uint8_t *dInvCount, uint32_t *dCRC);
void adcStartup(void);
bool resetDevice(bool bAction);
uint8_t sendStartSync(void);
uint8_t sendPowerdown(void);
uint8_t startContConversion(void);
uint8_t startSingleShotConversion(void);
bool checkDRDYbitForEOC(const uint32_t timeout_ms);
float checkInternalTemperature(bool dFormat);
float convertCodeToTemperature(int32_t dCodes);

// Getter functions
uint8_t getRegisterValue(uint8_t address);

// Helper functions
uint16_t combineBytes(uint8_t upperByte, uint8_t lowerByte);
int32_t signExtend(const uint8_t dataBytes[]);

/*****************************************************************************
 *
 * Register macros
 *
 ****************************************************************************/

/*****************************************************************************
 *
 * Device commands
 *
 ****************************************************************************/
#define CMD_POWERDOWN 0x02
#define CMD_START_SYNC 0x08
#define CMD_RESET 0x06
#define CMD_RDATA 0x10
#define CMD_RREG 0x20
#define CMD_WREG 0x40

/*****************************************************************************
 *
 * Constants
 *
 ****************************************************************************/
#define ADS122C04_ADDRESS 0x40

/* Adjust data components as needed relative to ADC */
/* Lengths of conversion data components */
#ifdef ADS122C04
#define DATA_LENGTH         3
#define INV_LENGTH          3
#define WORD_LENGTH_24BIT   3
#define BIT_RESOLUTION      24
#define TEMP_MASK   0xFFFC00
#else
#define DATA_LENGTH         2
#define INV_LENGTH          2
#define WORD_LENGTH_16BIT_TRUNCATED 2
#define BIT_RESOLUTION      16
#define TEMP_MASK   0xFFFC
#endif
#define TEMP_BITRES         14
#define COUNT_LENGTH        1
#define CRC_LENGTH          2
#define REGISTER_BYTE_LENGTH 1
#define COMMAND_BYTE_LENGTH 1
#define CHECK_DRDY_LOOP     13 // One loop is approximately 72us...1ms/72us = 13

/*****************************************************************************
 *
 * Device operating modes
 *
 ****************************************************************************/
#define DATA_MODE_NORMAL    0x00
#define DATA_MODE_DCNT      0x40
#define DATA_MODE_CRC       0x20
#define DATA_MODE_INV       0x10


#define NUM_REGISTERS                           ((uint8_t) 4)
/** Maximum register address or address of the last register in the regmap */
#define MAX_REGISTER_ADDRESS                    ((uint8_t) 3)


/*****************************************************************************
 *
 * Register definitions
 *
 ****************************************************************************/

/* Register 0x00 (CONFIG0) definition
 * |-------------------------------------------------------------------------------------------------------|
 * |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * |-------------------------------------------------------------------------------------------------------|
 * |                      MUX[3:0]                     |               GAIN[2:0]              | PGA_BYPASS |
 * |-------------------------------------------------------------------------------------------------------|
 */
    /* CONFIG0 register */
    #define CONFIG0_ADDRESS                                                 ((uint8_t) 0x00)
    #define CONFIG0_DEFAULT                                                 ((uint8_t) 0x00)

    /* MUX field */
    #define CONFIG0_MUX_MASK                                                ((uint8_t) 0xF0)
    #define CONFIG0_MUX_AIN0_AIN1                                           ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG0_MUX_AIN0_AIN2                                           ((uint8_t) 0x10)
    #define CONFIG0_MUX_AIN0_AIN3                                           ((uint8_t) 0x20)
    #define CONFIG0_MUX_AIN1_AIN0                                           ((uint8_t) 0x30)
    #define CONFIG0_MUX_AIN1_AIN2                                           ((uint8_t) 0x40)
    #define CONFIG0_MUX_AIN1_AIN3                                           ((uint8_t) 0x50)
    #define CONFIG0_MUX_AIN2_AIN3                                           ((uint8_t) 0x60)
    #define CONFIG0_MUX_AIN3_AIN2                                           ((uint8_t) 0x70)
    #define CONFIG0_MUX_AIN0_AVSS                                           ((uint8_t) 0x80)
    #define CONFIG0_MUX_AIN1_AVSS                                           ((uint8_t) 0x90)
    #define CONFIG0_MUX_AIN2_AVSS                                           ((uint8_t) 0xA0)
    #define CONFIG0_MUX_AIN3_AVSS                                           ((uint8_t) 0xB0)
    #define CONFIG0_MUX_VREF_DIV4                                           ((uint8_t) 0xC0)
    #define CONFIG0_MUX_ASUPPLY_DIV4                                        ((uint8_t) 0xD0)
    #define CONFIG0_MUX_INPUT_SHORT                                         ((uint8_t) 0xE0)

    /* GAIN field */
    #define CONFIG0_GAIN_MASK                                               ((uint8_t) 0x0E)
    #define CONFIG0_GAIN_1                                                  ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG0_GAIN_2                                                  ((uint8_t) 0x02)
    #define CONFIG0_GAIN_4                                                  ((uint8_t) 0x04)
    #define CONFIG0_GAIN_8                                                  ((uint8_t) 0x06)
    #define CONFIG0_GAIN_16                                                 ((uint8_t) 0x08)
    #define CONFIG0_GAIN_32                                                 ((uint8_t) 0x0A)
    #define CONFIG0_GAIN_64                                                 ((uint8_t) 0x0C)
    #define CONFIG0_GAIN_128                                                ((uint8_t) 0x0E)

    /* PGA_BYPASS field */
    #define CONFIG0_PGA_BYPASS_MASK                                         ((uint8_t) 0x01)
    #define CONFIG0_PGA_BYPASS_NO                                           ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG0_PGA_BYPASS_YES_DISABLED                                 ((uint8_t) 0x01)

/* Register 0x01 (CONFIG1) definition
 * |-------------------------------------------------------------------------------------------------------|
 * |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * |-------------------------------------------------------------------------------------------------------|
 * |                DR[2:0]               |    MODE    |     CM     |        VREF[1:0]        |     TS     |
 * |-------------------------------------------------------------------------------------------------------|
 */
    /* CONFIG1 register */
    #define CONFIG1_ADDRESS                                                 ((uint8_t) 0x01)
    #define CONFIG1_DEFAULT                                                 ((uint8_t) 0x00)

    /* DR field */
    #define CONFIG1_DR_MASK                                                 ((uint8_t) 0xE0)
    #define CONFIG1_DR_20_SPS                                               ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG1_DR_45_SPS                                               ((uint8_t) 0x20)
    #define CONFIG1_DR_90_SPS                                               ((uint8_t) 0x40)
    #define CONFIG1_DR_175_SPS                                              ((uint8_t) 0x60)
    #define CONFIG1_DR_330_SPS                                              ((uint8_t) 0x80)
    #define CONFIG1_DR_600_SPS                                              ((uint8_t) 0xA0)
    #define CONFIG1_DR_1000_SPS                                             ((uint8_t) 0xC0)
//    #define CONFIG1_DR_RESERVED                                               ((uint8_t) 0xE0)

    /* MODE field */
    #define CONFIG1_MODE_MASK                                               ((uint8_t) 0x10)
    #define CONFIG1_MODE_NORMAL                                             ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG1_MODE_TURBO                                              ((uint8_t) 0x10)

    /* CM field */
    #define CONFIG1_CM_MASK                                                 ((uint8_t) 0x08)
    #define CONFIG1_CM_SINGLE_SHOT                                          ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG1_CM_CONTINUOUS                                           ((uint8_t) 0x08)

    /* VREF field */
    #define CONFIG1_VREF_MASK                                               ((uint8_t) 0x06)
    #define CONFIG1_VREF_INT                                                ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG1_VREF_EXT                                                ((uint8_t) 0x02)
    #define CONFIG1_VREF_ASUPPLY                                            ((uint8_t) 0x04)
//    #define CONFIG1_VREF_ASUPPLY                                          ((uint8_t) 0x06)

    /* TS field */
    #define CONFIG1_TS_MASK                                                 ((uint8_t) 0x01)
    #define CONFIG1_TS_DISABLED                                             ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG1_TS_ENABLED                                              ((uint8_t) 0x01)

/* Register 0x02 (CONFIG2) definition
 * |-------------------------------------------------------------------------------------------------------|
 * |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * |-------------------------------------------------------------------------------------------------------|
 * |    DRDY    |    DCNT    |         CRC[1:0]        |     BCS    |               IDAC[2:0]              |
 * |-------------------------------------------------------------------------------------------------------|
 */
    /* CONFIG2 register */
    #define CONFIG2_ADDRESS                                                 ((uint8_t) 0x02)
    #define CONFIG2_DEFAULT                                                 ((uint8_t) 0x00)

    /* DRDY field */
    #define CONFIG2_DRDY_MASK                                               ((uint8_t) 0x80)
    #define CONFIG2_DRDY_OLD                                                ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG2_DRDY_NEW                                                ((uint8_t) 0x80)

    /* DCNT field */
    #define CONFIG2_DCNT_MASK                                               ((uint8_t) 0x40)
    #define CONFIG2_DCNT_DISABLED                                           ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG2_DCNT_ENABLED                                            ((uint8_t) 0x40)

    /* CRC field */
    #define CONFIG2_CRC_MASK                                                ((uint8_t) 0x30)
    #define CONFIG2_CRC_DISABLED                                            ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG2_CRC_INVERTED                                            ((uint8_t) 0x10)
    #define CONFIG2_CRC_CRC16                                               ((uint8_t) 0x20)
//    #define CONFIG2_CRC_RESERVED                                          ((uint8_t) 0x30)

    /* BCS field */
    #define CONFIG2_BCS_MASK                                                ((uint8_t) 0x08)
    #define CONFIG2_BCS_DISABLED                                            ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG2_BCS_ENABLED                                             ((uint8_t) 0x08)

    /* IDAC field */
    #define CONFIG2_IDAC_MASK                                               ((uint8_t) 0x07)
    #define CONFIG2_IDAC_OFF                                                ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG2_IDAC_10u                                                ((uint8_t) 0x01)
    #define CONFIG2_IDAC_50u                                                ((uint8_t) 0x02)
    #define CONFIG2_IDAC_100u                                               ((uint8_t) 0x03)
    #define CONFIG2_IDAC_250u                                               ((uint8_t) 0x04)
    #define CONFIG2_IDAC_500u                                               ((uint8_t) 0x05)
    #define CONFIG2_IDAC_1000u                                              ((uint8_t) 0x06)
    #define CONFIG2_IDAC_1500u                                              ((uint8_t) 0x07)

/* Register 0x03 (CONFIG3) definition
 * |-------------------------------------------------------------------------------------------------------|
 * |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * |-------------------------------------------------------------------------------------------------------|
 * |              I1MUX[2:0]              |              I2MUX[2:0]              |      RESERVED[1:0]      |
 * |-------------------------------------------------------------------------------------------------------|
 */
    /* CONFIG3 register */
    #define CONFIG3_ADDRESS                                                 ((uint8_t) 0x03)
    #define CONFIG3_DEFAULT                                                 ((uint8_t) 0x00)

    /* I1MUX field */
    #define CONFIG3_I1MUX_MASK                                              ((uint8_t) 0xE0)
    #define CONFIG3_I1MUX_DISABLED                                          ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG3_I1MUX_AIN0                                              ((uint8_t) 0x20)
    #define CONFIG3_I1MUX_AIN1                                              ((uint8_t) 0x40)
    #define CONFIG3_I1MUX_AIN2                                              ((uint8_t) 0x60)
    #define CONFIG3_I1MUX_AIN3                                              ((uint8_t) 0x80)
    #define CONFIG3_I1MUX_REFP                                              ((uint8_t) 0xA0)
    #define CONFIG3_I1MUX_REFN                                              ((uint8_t) 0xC0)
//    #define CONFIG3_I1MUX_RESERVED                                            ((uint8_t) 0xE0)

    /* I2MUX field */
    #define CONFIG3_I2MUX_MASK                                              ((uint8_t) 0x1C)
    #define CONFIG3_I2MUX_DISABLED                                          ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG3_I2MUX_AIN0                                              ((uint8_t) 0x04)
    #define CONFIG3_I2MUX_AIN1                                              ((uint8_t) 0x08)
    #define CONFIG3_I2MUX_AIN2                                              ((uint8_t) 0x0C)
    #define CONFIG3_I2MUX_AIN3                                              ((uint8_t) 0x10)
    #define CONFIG3_I2MUX_REFP                                              ((uint8_t) 0x14)
    #define CONFIG3_I2MUX_REFN                                              ((uint8_t) 0x18)
//    #define CONFIG3_I2MUX_RESERVED                                            ((uint8_t) 0x1C)

#endif /* ADS122C04_H_ */
