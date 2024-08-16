/**
 * \brief This header file contains all register map definitions for the ADS131B04-Q1.
 *
 * \copyright Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef ADS131B04_Q1_H_
#define ADS131B04_Q1_H_

// Standard libraries
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

// Custom libraries
#include "hal.h"


//****************************************************************************
//
// Select the device variant to use...
//
//****************************************************************************

#define CHANNEL_COUNT (4)   // ADS131B04-Q1 -> 4 Channels

#if ((CHANNEL_COUNT < 1) || (CHANNEL_COUNT > 4))
    #error Invalid channel count configured in 'ads131b04-q1.h'.
#endif



//****************************************************************************
//
// Select the desired MODE register settings...
// NOTE: These settings will be enforced and not modifiable during runtime!
//
//****************************************************************************

/* Pick one (and only one) mode to use... */
//#define WORD_LENGTH_16BIT_TRUNCATED
#define WORD_LENGTH_24BIT
//#define WORD_LENGTH_32BIT_SIGN_EXTEND
//#define WORD_LENGTH_32BIT_ZERO_PADDED

/* Enable this define statement to use CRC on DIN */
#define ENABLE_CRC_IN

/* Select CRC type */
#define CRC_CCITT
//#define CRC_ANSI

/* Disable assertions when not in the CCS "Debug" configuration */
#ifndef _DEBUG
    #define NDEBUG
#endif


//
// Validation
//

// Throw an error if no WORD_LENGTH mode was selected above
#if !defined WORD_LENGTH_16BIT_TRUNCATED &&  \
    !defined WORD_LENGTH_24BIT && \
    !defined WORD_LENGTH_32BIT_SIGN_EXTEND && \
    !defined WORD_LENGTH_32BIT_ZERO_PADDED
#error Must define at least one WORD_LENGTH mode
#endif

// Throw an error if none or both CRC types are selected
#if !defined CRC_CCITT && !defined CRC_ANSI
#error Must define at least one CRC type
#endif
#if defined CRC_CCITT && defined CRC_ANSI
#error Must define only one CRC type
#endif



//****************************************************************************
//
// Constants
//
//****************************************************************************

#define NUM_REGISTERS                           ((uint8_t) 63)

#ifdef EXAMPLE_CODE
#else

#define NUM_WRITABLE_REGISTERS  ((uint8_t) (5 * CHANNEL_COUNT) + 9)     // These macros are currently not used
#define MAX_REGISTER_ADDRESS    ((uint8_t) REGMAP_CRC_ADDRESS + 1)



//****************************************************************************
//
// Global variables
//
//****************************************************************************

// Register names, used by EVM firmware
extern const char *adcRegisterNames[];

#endif  // EXAMPLE_CODE


//****************************************************************************
//
// SPI command opcodes
//
//****************************************************************************

#define OPCODE_NULL                             ((uint16_t) 0x0000)
#define OPCODE_RESET                            ((uint16_t) 0x0011)
#define OPCODE_STANDBY                          ((uint16_t) 0x0022)
#define OPCODE_WAKEUP                           ((uint16_t) 0x0033)
#define OPCODE_LOCK                             ((uint16_t) 0x0555)
#define OPCODE_UNLOCK                           ((uint16_t) 0x0655)
#define OPCODE_RREG                             ((uint16_t) 0xA000)
#define OPCODE_WREG                             ((uint16_t) 0x6000)



//****************************************************************************
//
// Register definitions
//
//****************************************************************************

/* NOTE: Whenever possible, macro names (defined below) were derived from
 * data sheet defined names; however, updates to documentation may cause
 * mismatches between names defined here in example code from those shown
 * in the device data sheet.
 */


/* Register 0x00 (ID) definition - READ ONLY
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                    RESERVED0[3:0]                 |                    CHANCNT[3:0]                   |                                               RESERVED1[7:0]                                          |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 8 through 11 are fixed on the ADS131B04-Q1. These bits will always read 0100b.
 */

    /* ID register address & default value */
    #define ID_ADDRESS                                                      ((uint8_t)  0x00)
    #define ID_DEFAULT                                                      ((uint16_t) 0x4000 | (CHANNEL_COUNT << 8))  // NOTE: May change with future device revisions!

    /* RESERVED field mask */
    #define ID_RESERVED0_MASK                                               ((uint16_t) 0xF000)
    #define ID_RESERVED1_MASK                                               ((uint16_t) 0x00FF)

    /* CHANCNT field mask & values */
    #define ID_CHANCNT_MASK                                                 ((uint16_t) 0x0F00)
    #define ID_CHANCNT_4CH                                                  ((uint16_t) 0x0004 << 8)    // DEFAULT



/* Register 0x01 (STATUS) definition - READ ONLY
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |    LOCK    |  F_RESYNC  |   REG_MAP  |   CRC_ERR  |  CRC_TYPE  |    RESET   |       WLENGTH[1:0]      |                   RESERVED[3:0]                   |    DRDY3   |    DRDY2   |    DRDY1   |    DRDY0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 4 through 7 are reserved on the ADS131B04-Q1. These bits will always read 0000b.
 */

    /* STATUS register address & default value */
    #define STATUS_ADDRESS                                                  ((uint8_t)  0x01)
    #define STATUS_DEFAULT                                                  ((uint16_t) 0x0500)

    /* LOCK field mask & values */
    #define STATUS_LOCK_MASK                                                ((uint16_t) 0x8000)
    #define STATUS_LOCK_UNLOCKED                                            ((uint16_t) 0x0000 << 15)   // DEFAULT
    #define STATUS_LOCK_LOCKED                                              ((uint16_t) 0x0001 << 15)

    /* F_RESYNC field mask & values */
    #define STATUS_F_RESYNC_MASK                                            ((uint16_t) 0x4000)
    #define STATUS_F_RESYNC_NO_FAULT                                        ((uint16_t) 0x0000 << 14)   // DEFAULT
    #define STATUS_F_RESYNC_FAULT                                           ((uint16_t) 0x0001 << 14)

    /* REG_MAP field mask & values */
    #define STATUS_REG_MAP_MASK                                             ((uint16_t) 0x2000)
    #define STATUS_REG_MAP_NO_CHANGE_CRC                                    ((uint16_t) 0x0000 << 13)   // DEFAULT
    #define STATUS_REG_MAP_CHANGED_CRC                                      ((uint16_t) 0x0001 << 13)

    /* CRC_ERR field mask & values */
    #define STATUS_CRC_ERR_MASK                                             ((uint16_t) 0x1000)
    #define STATUS_CRC_ERR_NO_CRC_ERROR                                     ((uint16_t) 0x0000 << 12)   // DEFAULT
    #define STATUS_CRC_ERR_INPUT_CRC_ERROR                                  ((uint16_t) 0x0001 << 12)

    /* CRC_TYPE field mask & values */
    #define STATUS_CRC_TYPE_MASK                                            ((uint16_t) 0x0800)
    #define STATUS_CRC_TYPE_16BIT_CCITT                                     ((uint16_t) 0x0000 << 11)   // DEFAULT
    #define STATUS_CRC_TYPE_16BIT_ANSI                                      ((uint16_t) 0x0001 << 11)

    /* RESET field mask & values */
    #define STATUS_RESET_MASK                                               ((uint16_t) 0x0400)
    #define STATUS_RESET_NO_RESET                                           ((uint16_t) 0x0000 << 10)
    #define STATUS_RESET_RESET_OCCURRED                                     ((uint16_t) 0x0001 << 10)   // DEFAULT

    /* WLENGTH field mask & values */
    #define STATUS_WLENGTH_MASK                                             ((uint16_t) 0x0300)
    #define STATUS_WLENGTH_16BIT                                            ((uint16_t) 0x0000 << 8)
    #define STATUS_WLENGTH_24BIT                                            ((uint16_t) 0x0001 << 8)    // DEFAULT
    #define STATUS_WLENGTH_32BIT_LSB_ZEROES                                 ((uint16_t) 0x0002 << 8)
    #define STATUS_WLENGTH_32BIT_MSB_SIGN_EXT                               ((uint16_t) 0x0003 << 8)

    /* RESERVED field mask */
    #define STATUS_RESERVED_MASK                                            ((uint16_t) 0x00F0)

    /* DRDY3 field mask & values */
    #define STATUS_DRDY3_MASK                                               ((uint16_t) 0x0008)
    #define STATUS_DRDY3_NO_NEW_DATA                                        ((uint16_t) 0x0000 << 3)
    #define STATUS_DRDY3_NEW_DATA                                           ((uint16_t) 0x0001 << 3)

    /* DRDY2 field mask & values */
    #define STATUS_DRDY2_MASK                                               ((uint16_t) 0x0004)
    #define STATUS_DRDY2_NO_NEW_DATA                                        ((uint16_t) 0x0000 << 2)
    #define STATUS_DRDY2_NEW_DATA                                           ((uint16_t) 0x0001 << 2)

    /* DRDY1 field mask & values */
    #define STATUS_DRDY1_MASK                                               ((uint16_t) 0x0002)
    #define STATUS_DRDY1_NO_NEW_DATA                                        ((uint16_t) 0x0000 << 1)
    #define STATUS_DRDY1_NEW_DATA                                           ((uint16_t) 0x0001 << 1)

    /* DRDY0 field mask & values */
    #define STATUS_DRDY0_MASK                                               ((uint16_t) 0x0001)
    #define STATUS_DRDY0_NO_NEW_DATA                                        ((uint16_t) 0x0000 << 0)
    #define STATUS_DRDY0_NEW_DATA                                           ((uint16_t) 0x0001 << 0)



/* Register 0x02 (MODE) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      RESERVED0[1:0]     | REG_CRC_EN |  RX_CRC_EN |  CRC_TYPE  |    RESET   |       WLENGTH[1:0]      |             RESERVED1[2:0]           |   TIMEOUT  |      RESERVED2[1:0]     |  DRDY_HiZ  | RESERVED3  |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 14 through 15 are reserved on the ADS131B04-Q1. Always write 00b to these bits.
 *  NOTE 2: Bits 5 through 7 are reserved on the ADS131B04-Q1. Always write 000b to these bits.
 *  NOTE 3: Bits 2 through 3 are reserved on the ADS131B04-Q1. Always write 00b to these bits.
 *  NOTE 4: Bit 0 is reserved on the ADS131B04-Q1. Always write 0b to this bit.
 */

    /* MODE register address & default value */
    #define MODE_ADDRESS                                                    ((uint8_t)  0x02)
    #define MODE_DEFAULT                                                    ((uint16_t) 0x0510)

    /* RESERVED0 field mask */
    #define MODE_RESERVED0_MASK                                             ((uint16_t) 0xC000)

    /* REG_CRC_EN field mask & values */
    #define MODE_REG_CRC_EN_MASK                                            ((uint16_t) 0x2000)
    #define MODE_REG_CRC_EN_DISABLED                                        ((uint16_t) 0x0000 << 13)   // DEFAULT
    #define MODE_REG_CRC_EN_ENABLED                                         ((uint16_t) 0x0001 << 13)

    /* RX_CRC_EN field mask & values */
    #define MODE_RX_CRC_EN_MASK                                             ((uint16_t) 0x1000)
    #define MODE_RX_CRC_EN_DISABLED                                         ((uint16_t) 0x0000 << 12)   // DEFAULT
    #define MODE_RX_CRC_EN_ENABLED                                          ((uint16_t) 0x0001 << 12)

    /* CRC_TYPE field mask & values */
    #define MODE_CRC_TYPE_MASK                                              ((uint16_t) 0x0800)
    #define MODE_CRC_TYPE_16BIT_CCITT                                       ((uint16_t) 0x0000 << 11)   // DEFAULT
    #define MODE_CRC_TYPE_16BIT_ANSI                                        ((uint16_t) 0x0001 << 11)

    /* RESET field mask & values */
    #define MODE_RESET_MASK                                                 ((uint16_t) 0x0400)
    #define MODE_RESET_NO_RESET                                             ((uint16_t) 0x0000 << 10)
    #define MODE_RESET_RESET_OCCURRED                                       ((uint16_t) 0x0001 << 10)   // DEFAULT

    /* WLENGTH field mask & values */
    #define MODE_WLENGTH_MASK                                               ((uint16_t) 0x0300)
    #define MODE_WLENGTH_16BIT                                              ((uint16_t) 0x0000 << 8)
    #define MODE_WLENGTH_24BIT                                              ((uint16_t) 0x0001 << 8)    // DEFAULT
    #define MODE_WLENGTH_32BIT_LSB_ZEROES                                   ((uint16_t) 0x0002 << 8)
    #define MODE_WLENGTH_32BIT_MSB_SIGN_EXT                                 ((uint16_t) 0x0003 << 8)

    /* RESERVED1 field mask */
    #define MODE_RESERVED1_MASK                                             ((uint16_t) 0x00E0)

    /* TIMEOUT field mask & values */
    #define MODE_TIMEOUT_MASK                                               ((uint16_t) 0x0010)
    #define MODE_TIMEOUT_DISABLED                                           ((uint16_t) 0x0000 << 4)
    #define MODE_TIMEOUT_ENABLED                                            ((uint16_t) 0x0001 << 4)    // DEFAULT

    /* RESERVED2 field mask */
    #define MODE_RESERVED2_MASK                                             ((uint16_t) 0x000C)

    /* DRDY_HiZ field mask & values */
    #define MODE_DRDY_HiZ_MASK                                              ((uint16_t) 0x0002)
    #define MODE_DRDY_HiZ_LOGIC_HIGH                                        ((uint16_t) 0x0000 << 1)    // DEFAULT
    #define MODE_DRDY_HiZ_HIGH_IMPEDANCE                                    ((uint16_t) 0x0001 << 1)

    /* RESERVED3 field mask */
    #define MODE_RESERVED3_MASK                                             ((uint16_t) 0x0001)



/* Register 0x03 (CLOCK) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                  RESERVED0[3:0]                   |   CH3_EN   |   CH2_EN   |   CH1_EN   |   CH0_EN   |   CLK_SEL  |     RESERVED1[1:0]      |               OSR[2:0]               |         PWR[1:0]        |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 12 through 15 are reserved on the ADS131B04-Q1. These bits will always read 0000b.
 *  NOTE 2: Bits 5 through 6 are reserved on the ADS131B04-Q1. Always write 00b to these bits.
 */

    /* CLOCK register address & default value */
    #define CLOCK_ADDRESS                                                   ((uint8_t)  0x03)
    #define CLOCK_DEFAULT                                                   ((uint16_t) 0x0F0E)

    /* RESERVED0 field mask */
    #define CLOCK_RESERVED0_MASK                                             ((uint16_t) 0xE000)

    /* CH3_EN field mask & values */
    #define CLOCK_CH3_EN_MASK                                               ((uint16_t) 0x0800)
    #define CLOCK_CH3_EN_DISABLED                                           ((uint16_t) 0x0000 << 11)
    #define CLOCK_CH3_EN_ENABLED                                            ((uint16_t) 0x0001 << 11)   // DEFAULT

    /* CH2_EN field mask & values */
    #define CLOCK_CH2_EN_MASK                                               ((uint16_t) 0x0400)
    #define CLOCK_CH2_EN_DISABLED                                           ((uint16_t) 0x0000 << 10)
    #define CLOCK_CH2_EN_ENABLED                                            ((uint16_t) 0x0001 << 10)   // DEFAULT

    /* CH1_EN field mask & values */
    #define CLOCK_CH1_EN_MASK                                               ((uint16_t) 0x0200)
    #define CLOCK_CH1_EN_DISABLED                                           ((uint16_t) 0x0000 << 9)
    #define CLOCK_CH1_EN_ENABLED                                            ((uint16_t) 0x0001 << 9)    // DEFAULT

    /* CH0_EN field mask & values */
    #define CLOCK_CH0_EN_MASK                                               ((uint16_t) 0x0100)
    #define CLOCK_CH0_EN_DISABLED                                           ((uint16_t) 0x0000 << 8)
    #define CLOCK_CH0_EN_ENABLED                                            ((uint16_t) 0x0001 << 8)    // DEFAULT

    /* CLKSEL field mask */
    #define CLOCK_CLK_SEL_MASK                                              ((uint16_t) 0x0080)
    #define CLOCK_CLK_SEL_INTERNAL                                          ((uint16_t) 0x0000 << 7)
    #define CLOCK_CLK_SEL_EXTERNAL                                          ((uint16_t) 0x0001 << 7)    // DEFAULT

    /* RESERVED1 field mask */
    #define CLOCK_RESERVED1_MASK                                            ((uint16_t) 0x0060)

    /* OSR field mask & values */
    #define CLOCK_OSR_MASK                                                  ((uint16_t) 0x001C)
    #define CLOCK_OSR_128                                                   ((uint16_t) 0x0000 << 2)
    #define CLOCK_OSR_256                                                   ((uint16_t) 0x0001 << 2)
    #define CLOCK_OSR_512                                                   ((uint16_t) 0x0002 << 2)
    #define CLOCK_OSR_1024                                                  ((uint16_t) 0x0003 << 2)    // DEFAULT
    #define CLOCK_OSR_2048                                                  ((uint16_t) 0x0004 << 2)
    #define CLOCK_OSR_4096                                                  ((uint16_t) 0x0005 << 2)
    #define CLOCK_OSR_8192                                                  ((uint16_t) 0x0006 << 2)
    #define CLOCK_OSR_16384                                                 ((uint16_t) 0x0007 << 2)

    /* PWR field mask & values */
    #define CLOCK_PWR_MASK                                                  ((uint16_t) 0x0003)
    #define CLOCK_PWR_VLP                                                   ((uint16_t) 0x0000 << 0)
    #define CLOCK_PWR_LP                                                    ((uint16_t) 0x0001 << 0)
    #define CLOCK_PWR_HR                                                    ((uint16_t) 0x0002 << 0)    // DEFAULT, Alternative value: ((uint16_t) 0x0003 << 2)



/* Register 0x04 (GAIN) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |  RESERVED0 |             PGAGAIN3[2:0]            |  RESERVED1 |             PGAGAIN2[2:0]            |  RESERVED2 |             PGAGAIN1[2:0]            |  RESERVED3 |             PGAGAIN0[2:0]            |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bit 15 is reserved on the ADS131B04-Q1. Always write 0b to this bit.
 *  NOTE 2: Bit 11 is reserved on the ADS131B04-Q1. Always write 0b to this bit.
 *  NOTE 3: Bit 7 is reserved on the ADS131B04-Q1. Always write 0b to this bit.
 *  NOTE 4: Bit 3 is reserved on the ADS131B04-Q1. Always write 0b to this bit.
 */

    /* GAIN register address & default value */
    #define GAIN_ADDRESS                                                   ((uint8_t)  0x04)
    #define GAIN_DEFAULT                                                   ((uint16_t) 0x0000)

    /* RESERVED0 field mask & values */
    #define GAIN_RESERVED0_MASK                                            ((uint16_t) 0x8000)

    /* PGAGAIN3 field mask & values */
    #define GAIN_PGAGAIN3_MASK                                             ((uint16_t) 0x7000)
    #define GAIN_PGAGAIN3_1                                                ((uint16_t) 0x0000 << 12)    // DEFAULT
    #define GAIN_PGAGAIN3_2                                                ((uint16_t) 0x0001 << 12)
    #define GAIN_PGAGAIN3_4                                                ((uint16_t) 0x0002 << 12)
    #define GAIN_PGAGAIN3_8                                                ((uint16_t) 0x0003 << 12)
    #define GAIN_PGAGAIN3_16                                               ((uint16_t) 0x0004 << 12)
    #define GAIN_PGAGAIN3_32                                               ((uint16_t) 0x0005 << 12)
    #define GAIN_PGAGAIN3_64                                               ((uint16_t) 0x0006 << 12)
    #define GAIN_PGAGAIN3_128                                              ((uint16_t) 0x0007 << 12)

    /* RESERVED1 field mask & values */
    #define GAIN_RESERVED1_MASK                                            ((uint16_t) 0x0800)

    /* PGAGAIN2 field mask & values */
    #define GAIN_PGAGAIN2_MASK                                             ((uint16_t) 0x0700)
    #define GAIN_PGAGAIN2_1                                                ((uint16_t) 0x0000 << 8)     // DEFAULT
    #define GAIN_PGAGAIN2_2                                                ((uint16_t) 0x0001 << 8)
    #define GAIN_PGAGAIN2_4                                                ((uint16_t) 0x0002 << 8)
    #define GAIN_PGAGAIN2_8                                                ((uint16_t) 0x0003 << 8)
    #define GAIN_PGAGAIN2_16                                               ((uint16_t) 0x0004 << 8)
    #define GAIN_PGAGAIN2_32                                               ((uint16_t) 0x0005 << 8)
    #define GAIN_PGAGAIN2_64                                               ((uint16_t) 0x0006 << 8)
    #define GAIN_PGAGAIN2_128                                              ((uint16_t) 0x0007 << 8)

    /* RESERVED2 field mask & values */
    #define GAIN_RESERVED2_MASK                                            ((uint16_t) 0x0080)

    /* PGAGAIN field mask & values */
    #define GAIN_PGAGAIN1_MASK                                             ((uint16_t) 0x0070)
    #define GAIN_PGAGAIN1_1                                                ((uint16_t) 0x0000 << 4)     // DEFAULT
    #define GAIN_PGAGAIN1_2                                                ((uint16_t) 0x0001 << 4)
    #define GAIN_PGAGAIN1_4                                                ((uint16_t) 0x0002 << 4)
    #define GAIN_PGAGAIN1_8                                                ((uint16_t) 0x0003 << 4)
    #define GAIN_PGAGAIN1_16                                               ((uint16_t) 0x0004 << 4)
    #define GAIN_PGAGAIN1_32                                               ((uint16_t) 0x0005 << 4)
    #define GAIN_PGAGAIN1_64                                               ((uint16_t) 0x0006 << 4)
    #define GAIN_PGAGAIN1_128                                              ((uint16_t) 0x0007 << 4)

    /* RESERVED3 field mask & values */
    #define GAIN_RESERVED3_MASK                                            ((uint16_t) 0x0008)

    /* PGAGAIN0 field mask & values */
    #define GAIN_PGAGAIN0_MASK                                             ((uint16_t) 0x0007)
    #define GAIN_PGAGAIN0_1                                                ((uint16_t) 0x0000 << 0)     // DEFAULT
    #define GAIN_PGAGAIN0_2                                                ((uint16_t) 0x0001 << 0)
    #define GAIN_PGAGAIN0_4                                                ((uint16_t) 0x0002 << 0)
    #define GAIN_PGAGAIN0_8                                                ((uint16_t) 0x0003 << 0)
    #define GAIN_PGAGAIN0_16                                               ((uint16_t) 0x0004 << 0)
    #define GAIN_PGAGAIN0_32                                               ((uint16_t) 0x0005 << 0)
    #define GAIN_PGAGAIN0_64                                               ((uint16_t) 0x0006 << 0)
    #define GAIN_PGAGAIN0_128                                              ((uint16_t) 0x0007 << 0)



/* Register 0x05 (RESERVED) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                                RESERVED[15:0]                                                                                                 |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: This register is reserved on the ADS131B04-Q1. Always write 0x0000 to this register.
 */

    /* RESERVED_05h register address & default value */
    #define RESERVED_05H_ADDRESS                                            ((uint8_t)  0x05)
    #define RESERVED_05H_DEFAULT                                            ((uint16_t) 0x0000)



/* Register 0x06 (CFG) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |            RESERVED0[2:0]            |                    GC_DLY[3:0]                    |    GC_EN   |                                            RESERVED1[7:0]                                             |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 13 through 15 are reserved on the ADS131B04-Q1. Always write 000b to these bits.
 *  NOTE 2: Bits 0 through 7 are reserved on the ADS131B04-Q1. Always write 0x00 to these bits.
 */

    /* CFG register address & default value */
    #define CFG_ADDRESS                                                     ((uint8_t)  0x06)
    #define CFG_DEFAULT                                                     ((uint16_t) 0x0600)

    /* RESERVED0 field mask & values */
    #define CFG_RESERVED0_MASK                                              ((uint16_t) 0xE000)

    /* GC_DLY field mask & values */
    #define CFG_GC_DLY_MASK                                                 ((uint16_t) 0x1E00)
    #define CFG_GC_DLY_2                                                    ((uint16_t) 0x0000 << 9)
    #define CFG_GC_DLY_4                                                    ((uint16_t) 0x0001 << 9)
    #define CFG_GC_DLY_8                                                    ((uint16_t) 0x0002 << 9)
    #define CFG_GC_DLY_16                                                   ((uint16_t) 0x0003 << 9)    // DEFAULT
    #define CFG_GC_DLY_32                                                   ((uint16_t) 0x0004 << 9)
    #define CFG_GC_DLY_64                                                   ((uint16_t) 0x0005 << 9)
    #define CFG_GC_DLY_128                                                  ((uint16_t) 0x0006 << 9)
    #define CFG_GC_DLY_256                                                  ((uint16_t) 0x0007 << 9)
    #define CFG_GC_DLY_512                                                  ((uint16_t) 0x0008 << 9)
    #define CFG_GC_DLY_1024                                                 ((uint16_t) 0x0009 << 9)
    #define CFG_GC_DLY_2048                                                 ((uint16_t) 0x000A << 9)
    #define CFG_GC_DLY_4096                                                 ((uint16_t) 0x000B << 9)
    #define CFG_GC_DLY_8192                                                 ((uint16_t) 0x000C << 9)
    #define CFG_GC_DLY_16484                                                ((uint16_t) 0x000D << 9)
    #define CFG_GC_DLY_32768                                                ((uint16_t) 0x000E << 9)
    #define CFG_GC_DLY_65536                                                ((uint16_t) 0x000F << 9)

    /* GC_EN field mask & values */
    #define CFG_GC_EN_MASK                                                  ((uint16_t) 0x0100)
    #define CFG_GC_EN_DISABLED                                              ((uint16_t) 0x0000 << 8)    // DEFAULT
    #define CFG_GC_EN_ENABLED                                               ((uint16_t) 0x0001 << 8)

    /* RESERVED1 field mask & values */
    #define CFG_RESERVED1_MASK                                              ((uint16_t) 0x00FF)



/* Register 0x07 (RESERVED) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                                RESERVED[15:0]                                                                                                 |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: This register is reserved on the ADS131B04-Q1. Always write 0x0000 to this register.
 */

    /* RESERVED register address & default value */
    #define RESERVED_07H_ADDRESS                                            ((uint8_t)  0x07)
    #define RESERVED_07H_DEFAULT                                            ((uint16_t) 0x0000)



/* Register 0x08 (RESERVED) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                            RESERVED0[15:8]                                            |                  RESERVED1[7:4]                   |                  RESERVED2[3:0]                   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 8 through 15 are reserved on the ADS131B04-Q1. Always write 0x00 to these bits.
 *  NOTE 2: Bits 4 through 7 are reserved on the ADS131B04-Q1. These bits will always read 0000b.
 *  NOTE 3: Bits 0 through 3 are reserved on the ADS131B04-Q1. Always write 0000b to these bits.
 */

    /* RESERVED register address & default value */
    #define RESERVED_08H_ADDRESS                                            ((uint8_t)  0x08)
    #define RESERVED_08H_DEFAULT                                            ((uint16_t) 0x0000)



/* Register 0x09 (CH0_CFG) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                        RESERVED0[15:6]                                                          |         RESERVED1[5:3]               |RESERVED2[2]|        MUX0[1:0]        |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 6 through 15 are reserved on the ADS131B04-Q1. Always write 0000000000b to these bits.
 *  NOTE 2: Bits 3 through 5 are reserved on the ADS131B04-Q1. These bits will always read 000b.
 *  NOTE 3: Bit 2 is reserved on the ADS131B04-Q1. Always write 0b to this bit.
 */

    /* CH0_CFG register address & default value */
    #define CH0_CFG_ADDRESS                                                 ((uint8_t)  0x09)
    #define CH0_CFG_DEFAULT                                                 ((uint16_t) 0x0000)

    /* RESERVED0 field mask & values */
    #define CH0_CFG_RESERVED0_MASK                                          ((uint16_t) 0xFFC0)

    /* RESERVED1 field mask & values */
    #define CH0_CFG_RESERVED1_MASK                                          ((uint16_t) 0x0038)

    /* RESERVED2 field mask & values */
    #define CH0_CFG_RESERVED2_MASK                                          ((uint16_t) 0x0004)

    /* MUX0 field mask & values */
    #define CH0_CFG_MUX0_MASK                                               ((uint16_t) 0x0003)
    #define CH0_CFG_MUX0_AIN0P_AIN0N                                        ((uint16_t) 0x0000 << 0)    // DEFAULT
    #define CH0_CFG_MUX0_ADC_INPUT_SHORT                                    ((uint16_t) 0x0001 << 0)
    #define CH0_CFG_MUX0_POSITIVE_DC_TEST                                   ((uint16_t) 0x0002 << 0)
    #define CH0_CFG_MUX0_NEGATIVE_DC_TEST                                   ((uint16_t) 0x0003 << 0)



/* Register 0x0A (CH0_OCAL_MSB) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                                OCAL0_MSB[15:0]                                                                                                |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CH0_OCAL_MSB register address & default value */
    #define CH0_OCAL_MSB_ADDRESS                                            ((uint8_t)  0x0A)
    #define CH0_OCAL_MSB_DEFAULT                                            ((uint16_t) 0x0000)

    /* OCAL0_MSB field mask & values */
    #define CH0_OCAL_MSB_OCAL0_MSB_MASK                                     ((uint16_t) 0xFFFF)



/* Register 0x0B (CH0_OCAL_LSB) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                             OCAL0_LSB[7:0]                                            |                                             RESERVED0[7:0]                                            |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 0 through 7 are reserved on the ADS131B04-Q1. These bits will always read 0x00.
 */

    /* CH0_OCAL_LSB register address & default value */
    #define CH0_OCAL_LSB_ADDRESS                                            ((uint8_t)  0x0B)
    #define CH0_OCAL_LSB_DEFAULT                                            ((uint16_t) 0x0000)

    /* OCAL0_LSB field mask & values */
    #define CH0_OCAL_LSB_OCAL0_LSB_MASK                                     ((uint16_t) 0xFF00)

    /* RESERVED0 field mask & values */
    #define CH0_OCAL_LSB_RESERVED0_MASK                                     ((uint16_t) 0x00FF)



/* Register 0x0C (CH0_GCAL_MSB) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                                GCAL0_MSB[15:0]                                                                                                |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CH0_GCAL_MSB register address & default value */
    #define CH0_GCAL_MSB_ADDRESS                                            ((uint8_t)  0x0C)
    #define CH0_GCAL_MSB_DEFAULT                                            ((uint16_t) 0x8000)

    /* GCAL0_MSB field mask & values */
    #define CH0_GCAL_MSB_GCAL0_MSB_MASK                                     ((uint16_t) 0xFFFF)



/* Register 0x0D (CH0_GCAL_LSB) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                             GCAL0_LSB[7:0]                                            |                                             RESERVED0[7:0]                                            |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 0 through 7 are reserved on the ADS131B04-Q1. These bits will always read 0x00.
 */

    /* CH0_GCAL_LSB register address & default value */
    #define CH0_GCAL_LSB_ADDRESS                                            ((uint8_t)  0x0D)
    #define CH0_GCAL_LSB_DEFAULT                                            ((uint16_t) 0x0000)

    /* GCAL0_LSB field mask & values */
    #define CH0_GCAL_LSB_GCAL0_LSB_MASK                                     ((uint16_t) 0xFF00)

    /* RESERVED0 field mask & values */
    #define CH0_GCAL_LSB_RESERVED0_MASK                                     ((uint16_t) 0x00FF)



/* Register 0x0E (CH1_CFG) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                        RESERVED0[15:6]                                                          |         RESERVED1[5:3]               |RESERVED2[2]|        MUX1[1:0]        |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 6 through 15 are reserved on the ADS131B04-Q1. Always write 0000000000b to these bits.
 *  NOTE 2: Bits 3 through 5 are reserved on the ADS131B04-Q1. These bits will always read 000b.
 *  NOTE 3: Bit 2 is reserved on the ADS131B04-Q1. Always write 0b to this bit.
 */

    /* CH1_CFG register address & default value */
    #define CH1_CFG_ADDRESS                                                 ((uint8_t)  0x0E)
    #define CH1_CFG_DEFAULT                                                 ((uint16_t) 0x0000)

    /* RESERVED0 field mask & values */
    #define CH1_CFG_RESERVED0_MASK                                          ((uint16_t) 0xFFC0)

    /* RESERVED1 field mask & values */
    #define CH1_CFG_RESERVED1_MASK                                          ((uint16_t) 0x0038)

    /* RESERVED2 field mask & values */
    #define CH1_CFG_RESERVED2_MASK                                          ((uint16_t) 0x0004)

    /* MUX1 field mask & values */
    #define CH1_CFG_MUX1_MASK                                               ((uint16_t) 0x0003)
    #define CH1_CFG_MUX1_AIN1P_AIN1N                                        ((uint16_t) 0x0000 << 0)    // DEFAULT
    #define CH1_CFG_MUX1_ADC_INPUT_SHORT                                    ((uint16_t) 0x0001 << 0)
    #define CH1_CFG_MUX1_POSITIVE_DC_TEST                                   ((uint16_t) 0x0002 << 0)
    #define CH1_CFG_MUX1_NEGATIVE_DC_TEST                                   ((uint16_t) 0x0003 << 0)



/* Register 0x0F (CH1_OCAL_MSB) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                                OCAL1_MSB[15:0]                                                                                                |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CH1_OCAL_MSB register address & default value */
    #define CH1_OCAL_MSB_ADDRESS                                            ((uint8_t)  0x0F)
    #define CH1_OCAL_MSB_DEFAULT                                            ((uint16_t) 0x0000)

    /* OCAL1_MSB field mask & values */
    #define CH1_OCAL_MSB_OCAL1_MSB_MASK                                     ((uint16_t) 0xFFFF)



/* Register 0x10 (CH1_OCAL_LSB) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                             OCAL1_LSB[7:0]                                            |                                             RESERVED0[7:0]                                            |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 0 through 7 are reserved on the ADS131B04-Q1. These bits will always read 0x00.
 */

    /* CH1_OCAL_LSB register address & default value */
    #define CH1_OCAL_LSB_ADDRESS                                            ((uint8_t)  0x10)
    #define CH1_OCAL_LSB_DEFAULT                                            ((uint16_t) 0x0000)

    /* OCAL1_LSB field mask & values */
    #define CH1_OCAL_LSB_OCAL1_LSB_MASK                                     ((uint16_t) 0xFF00)

    /* RESERVED0 field mask & values */
    #define CH1_OCAL_LSB_RESERVED0_MASK                                     ((uint16_t) 0x00FF)



/* Register 0x11 (CH1_GCAL_MSB) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                                GCAL1_MSB[15:0]                                                                                                |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CH0_GCAL_MSB register address & default value */
    #define CH1_GCAL_MSB_ADDRESS                                            ((uint8_t)  0x11)
    #define CH1_GCAL_MSB_DEFAULT                                            ((uint16_t) 0x8000)

    /* GCAL1_MSB field mask & values */
    #define CH1_GCAL_MSB_GCAL1_MSB_MASK                                     ((uint16_t) 0xFFFF)



/* Register 0x12 (CH1_GCAL_LSB) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                             GCAL1_LSB[7:0]                                            |                                             RESERVED0[7:0]                                            |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 0 through 7 are reserved on the ADS131B04-Q1. These bits will always read 0x00.
 */

    /* CH1_GCAL_LSB register address & default value */
    #define CH1_GCAL_LSB_ADDRESS                                            ((uint8_t)  0x12)
    #define CH1_GCAL_LSB_DEFAULT                                            ((uint16_t) 0x0000)

    /* GCAL1_LSB field mask & values */
    #define CH1_GCAL_LSB_GCAL1_LSB_MASK                                     ((uint16_t) 0xFF00)

    /* RESERVED0 field mask & values */
    #define CH1_GCAL_LSB_RESERVED0_MASK                                     ((uint16_t) 0x00FF)



/* Register 0x13 (CH2_CFG) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                        RESERVED0[15:6]                                                          |         RESERVED1[5:3]               |RESERVED2[2]|        MUX2[1:0]        |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 6 through 15 are reserved on the ADS131B04-Q1. Always write 0000000000b to these bits.
 *  NOTE 2: Bits 3 through 5 are reserved on the ADS131B04-Q1. These bits will always read 000b.
 *  NOTE 3: Bit 2 is reserved on the ADS131B04-Q1. Always write 0b to this bit.
 */

    /* CH2_CFG register address & default value */
    #define CH2_CFG_ADDRESS                                                 ((uint8_t)  0x13)
    #define CH2_CFG_DEFAULT                                                 ((uint16_t) 0x0000)


    /* RESERVED0 field mask & values */
    #define CH2_CFG_RESERVED0_MASK                                          ((uint16_t) 0xFFC0)

    /* RESERVED1 field mask & values */
    #define CH2_CFG_RESERVED1_MASK                                          ((uint16_t) 0x0038)

    /* RESERVED2 field mask & values */
    #define CH2_CFG_RESERVED2_MASK                                          ((uint16_t) 0x0004)

    /* MUX2 field mask & values */
    #define CH2_CFG_MUX2_MASK                                               ((uint16_t) 0x0003)
    #define CH2_CFG_MUX2_AIN2P_AIN2N                                        ((uint16_t) 0x0000 << 0)    // DEFAULT
    #define CH2_CFG_MUX2_ADC_INPUT_SHORT                                    ((uint16_t) 0x0001 << 0)
    #define CH2_CFG_MUX2_POSITIVE_DC_TEST                                   ((uint16_t) 0x0002 << 0)
    #define CH2_CFG_MUX2_NEGATIVE_DC_TEST                                   ((uint16_t) 0x0003 << 0)



/* Register 0x14 (CH2_OCAL_MSB) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                                OCAL2_MSB[15:0]                                                                                                |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CH2_OCAL_MSB register address & default value */
    #define CH2_OCAL_MSB_ADDRESS                                            ((uint8_t)  0x14)
    #define CH2_OCAL_MSB_DEFAULT                                            ((uint16_t) 0x0000)

    /* OCAL2_MSB field mask & values */
    #define CH2_OCAL_MSB_OCAL2_MSB_MASK                                     ((uint16_t) 0xFFFF)



/* Register 0x15 (CH2_OCAL_LSB) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                             OCAL2_LSB[7:0]                                            |                                             RESERVED0[7:0]                                            |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 0 through 7 are reserved on the ADS131B04-Q1. These bits will always read 0x00.
 */

    /* CH2_OCAL_LSB register address & default value */
    #define CH2_OCAL_LSB_ADDRESS                                            ((uint8_t)  0x15)
    #define CH2_OCAL_LSB_DEFAULT                                            ((uint16_t) 0x0000)

    /* OCAL2_LSB field mask & values */
    #define CH2_OCAL_LSB_OCAL2_LSB_MASK                                     ((uint16_t) 0xFF00)

    /* RESERVED0 field mask & values */
    #define CH2_OCAL_LSB_RESERVED0_MASK                                     ((uint16_t) 0x00FF)



/* Register 0x16 (CH2_GCAL_MSB) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                                GCAL2_MSB[15:0]                                                                                                |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CH2_GCAL_MSB register address & default value */
    #define CH2_GCAL_MSB_ADDRESS                                            ((uint8_t)  0x16)
    #define CH2_GCAL_MSB_DEFAULT                                            ((uint16_t) 0x8000)

    /* GCAL2_MSB field mask & values */
    #define CH2_GCAL_MSB_GCAL2_MSB_MASK                                     ((uint16_t) 0xFFFF)



/* Register 0x17 (CH2_GCAL_LSB) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                             GCAL2_LSB[7:0]                                            |                                             RESERVED0[7:0]                                            |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 0 through 7 are reserved on the ADS131B04-Q1. These bits will always read 0x00.
 */

    /* CH2_GCAL_LSB register address & default value */
    #define CH2_GCAL_LSB_ADDRESS                                            ((uint8_t)  0x17)
    #define CH2_GCAL_LSB_DEFAULT                                            ((uint16_t) 0x0000)

    /* GCAL2_LSB field mask & values */
    #define CH2_GCAL_LSB_GCAL2_LSB_MASK                                     ((uint16_t) 0xFF00)

    /* RESERVED0 field mask & values */
    #define CH2_GCAL_LSB_RESERVED0_MASK                                     ((uint16_t) 0x00FF)



/* Register 0x18 (CH3_CFG) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                        RESERVED0[15:6]                                                          |         RESERVED1[5:3]               |RESERVED2[2]|        MUX3[1:0]        |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 6 through 15 are reserved on the ADS131B04-Q1. Always write 0000000000b to these bits.
 *  NOTE 2: Bits 3 through 5 are reserved on the ADS131B04-Q1. These bits will always read 000b.
 *  NOTE 3: Bit 2 is reserved on the ADS131B04-Q1. Always write 0b to this bit.
 */

    /* CH3_CFG register address & default value */
    #define CH3_CFG_ADDRESS                                                 ((uint8_t)  0x18)
    #define CH3_CFG_DEFAULT                                                 ((uint16_t) 0x0000)

    /* RESERVED0 field mask & values */
    #define CH3_CFG_RESERVED0_MASK                                          ((uint16_t) 0xFFC0)

    /* RESERVED1 field mask & values */
    #define CH3_CFG_RESERVED1_MASK                                          ((uint16_t) 0x0038)

    /* RESERVED2 field mask & values */
    #define CH3_CFG_RESERVED2_MASK                                          ((uint16_t) 0x0004)

    /* MUX3 field mask & values */
    #define CH3_CFG_MUX3_MASK                                               ((uint16_t) 0x0003)
    #define CH3_CFG_MUX3_AIN3P_AIN3N                                        ((uint16_t) 0x0000 << 0)    // DEFAULT
    #define CH3_CFG_MUX3_ADC_INPUT_SHORT                                    ((uint16_t) 0x0001 << 0)
    #define CH3_CFG_MUX3_POSITIVE_DC_TEST                                   ((uint16_t) 0x0002 << 0)
    #define CH3_CFG_MUX3_NEGATIVE_DC_TEST                                   ((uint16_t) 0x0003 << 0)



/* Register 0x19 (CH3_OCAL_MSB) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                                OCAL3_MSB[15:0]                                                                                                |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CH3_OCAL_MSB register address & default value */
    #define CH3_OCAL_MSB_ADDRESS                                            ((uint8_t)  0x19)
    #define CH3_OCAL_MSB_DEFAULT                                            ((uint16_t) 0x0000)

    /* OCAL3_MSB field mask & values */
    #define CH3_OCAL_MSB_OCAL3_MSB_MASK                                     ((uint16_t) 0xFFFF)



/* Register 0x1A (CH3_OCAL_LSB) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                             OCAL3_LSB[7:0]                                            |                                             RESERVED0[7:0]                                            |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 0 through 7 are reserved on the ADS131B04-Q1. These bits will always read 0x00.
 */

    /* CH3_OCAL_LSB register address & default value */
    #define CH3_OCAL_LSB_ADDRESS                                            ((uint8_t)  0x1A)
    #define CH3_OCAL_LSB_DEFAULT                                            ((uint16_t) 0x0000)

    /* OCAL3_LSB field mask & values */
    #define CH3_OCAL_LSB_OCAL3_LSB_MASK                                     ((uint16_t) 0xFF00)

    /* RESERVED0 field mask & values */
    #define CH3_OCAL_LSB_RESERVED0_MASK                                     ((uint16_t) 0x00FF)



/* Register 0x1B (CH3_GCAL_MSB) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                                GCAL3_MSB[15:0]                                                                                                |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

    /* CH3_GCAL_MSB register address & default value */
    #define CH3_GCAL_MSB_ADDRESS                                            ((uint8_t)  0x1B)
    #define CH3_GCAL_MSB_DEFAULT                                            ((uint16_t) 0x8000)

    /* GCAL3_MSB field mask & values */
    #define CH3_GCAL_MSB_GCAL3_MSB_MASK                                     ((uint16_t) 0xFFFF)



/* Register 0x1C (CH3_GCAL_LSB) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                             GCAL3_LSB[7:0]                                            |                                             RESERVED0[7:0]                                            |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 *  NOTE 1: Bits 0 through 7 are reserved on the ADS131B04-Q1. These bits will always read 0x00.
 */

    /* CH3_GCAL_LSB register address & default value */
    #define CH3_GCAL_LSB_ADDRESS                                            ((uint8_t)  0x1C)
    #define CH3_GCAL_LSB_DEFAULT                                            ((uint16_t) 0x0000)

    /* GCAL3_LSB field mask & values */
    #define CH3_GCAL_LSB_GCAL3_LSB_MASK                                     ((uint16_t) 0xFF00)

    /* RESERVED0 field mask & values */
    #define CH3_GCAL_LSB_RESERVED0_MASK                                     ((uint16_t) 0x00FF)



/* Register 0x3E (REGMAP_CRC) definition
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   Bit 15   |   Bit 14   |   Bit 13   |   Bit 12   |   Bit 11   |   Bit 10   |    Bit 9   |    Bit 8   |    Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                                 REG_CRC[15:0]                                                                                                 |
 * -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */

    /* REGMAP_CRC register address & default value */
    #define REGMAP_CRC_ADDRESS                                              ((uint8_t)  0x3E)
    #define REGMAP_CRC_DEFAULT                                              ((uint16_t) 0x0000)

    /* REG_CRC field mask & values */
    #define REGMAP_CRC_REG_CRC_MASK                                         ((uint16_t) 0xFFFF)



//****************************************************************************
//
// Channel data structure
//
//****************************************************************************

typedef struct
{
    uint16_t response;
    uint16_t crc;
    int32_t channel0;

#if (CHANNEL_COUNT > 1)
    int32_t channel1;
#endif
#if (CHANNEL_COUNT > 2)
    int32_t channel2;
#endif
#if (CHANNEL_COUNT > 3)
    int32_t channel3;
#endif
} adc_channel_data;



//****************************************************************************
//
// Function prototypes
//
//****************************************************************************

void        adcStartup(void);
uint16_t    sendCommand(uint16_t op_code);
bool        readData(adc_channel_data *DataStruct);
uint16_t    readSingleRegister(uint8_t address);
void        writeSingleRegister(uint8_t address, uint16_t data);
bool        lockRegisters(void);
bool        unlockRegisters(void);
uint16_t    resetDevice(void);
void        restoreRegisterDefaults(void);
uint16_t    calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint16_t initialValue);
#ifdef EXAMPLE_CODE
#else
//TODO HIGH: Implement these functions...
//void        readMultipleRegisters(uint8_t startAddress, uint8_t count, uint16_t regDataArray[]);
//void        writeMultipleRegisters(uint8_t addr, uint8_t count, const uint16_t data[]);
#endif

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

/** Returns Number of Channels */
#define CHANCNT             ((uint8_t) ((getRegisterValue(ID_ADDRESS) & ID_CHANCNT_MASK) >> 8))

/** Revision ID bits */
#define REVISION_ID         ((uint8_t) ((getRegisterValue(ID_ADDRESS) & ID_REVID_MASK) >> 0))

/** Returns true if SPI interface is locked */
#define SPI_LOCKED          ((bool) (getRegisterValue(STATUS_ADDRESS) & STATUS_LOCK_LOCKED))

/** Returns SPI Communication Word Format*/
#define WLENGTH             ((uint8_t) ((getRegisterValue(MODE_ADDRESS) & STATUS_WLENGTH_MASK) >> 8))

/** Returns true if Register Map CRC byte enable bit is set */
#define REGMAP_CRC_ENABLED  ((bool) (getRegisterValue(MODE_ADDRESS) & MODE_REG_CRC_EN_ENABLED))

/** Returns true if SPI CRC byte enable bit is set */
#define SPI_CRC_ENABLED     ((bool) (getRegisterValue(MODE_ADDRESS) & MODE_RX_CRC_EN_ENABLED))

/** Returns false for CCITT and true for ANSI CRC type */
#define SPI_CRC_TYPE        ((bool) (getRegisterValue(MODE_ADDRESS) & MODE_CRC_TYPE_MASK))

/** Data rate register field setting */
#define OSR_INDEX           ((uint8_t) ((getRegisterValue(CLOCK_ADDRESS) & CLOCK_OSR_MASK) >> 2))

/** Data rate register field setting */
#define POWER_MODE          ((uint8_t) ((getRegisterValue(CLOCK_ADDRESS) & CLOCK_PWR_MASK) >> 0))

#endif /* ADS131B04_Q1_H_ */
