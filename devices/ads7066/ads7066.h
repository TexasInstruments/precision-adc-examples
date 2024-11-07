/**
 * \copyright Copyright (C) 2019-2020 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef ADS7066_H_
#define ADS7066_H_

// Standard libraries
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

// Custom libraries
#include "hal.h"


//****************************************************************************
//
// Macros & Constants
//
//****************************************************************************
#define NUM_REGISTERS                           ((uint8_t) 195)


//**********************************************************************************
//
// Device commands
//
//**********************************************************************************
#define OPCODE_NULL                             ((uint8_t) 0x00)
#define OPCODE_RREG                             ((uint8_t) 0x10)
#define OPCODE_WREG                             ((uint8_t) 0x08)
#define OPCODE_SETBIT                           ((uint8_t) 0x18)
#define OPCODE_CLRBIT                           ((uint8_t) 0x20)


//****************************************************************************
//
// Register definitions
//
//****************************************************************************

/* NOTE: Whenever possible, macro names (defined below) were derived from
 * datasheet defined names; however, updates to documentation may cause
 * mismatches between names defined here in this code from those shown
 * in the device datasheet.
 */

/* Register 0x00 (SYSTEM_STATUS) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |   RSVD    | SEQ_STATUS |           RESERVED[5:3]              | CRCERR_FUSE|  CRCERR_IN |     BOR    |
 * --------------------------------------------------------------------------------------------------------
 *  Bits 0 and 1 alone supports both read and write operations. Bits from 2 to 7 supports only read.
 */

    /* SYSTEM_STATUS register address & default value */
    #define SYSTEM_STATUS_ADDRESS                                           ((uint8_t) 0x00)
    #define SYSTEM_STATUS_DEFAULT                                           ((uint8_t) 0x81)

    /* RESERVED field mask */
    #define SYSTEM_STATUS_RESERVED_MASK                                     ((uint8_t) 0xB8)

    /* SEQ_STATUS field mask & values */
    #define SYSTEM_STATUS_SEQ_STATUS_MASK                                   ((uint8_t) 0x40)
    #define SYSTEM_STATUS_SEQ_STATUS_SEQ_STOPPED                            ((uint8_t) 0x00 << 6) // DEFAULT
    #define SYSTEM_STATUS_SEQ_STATUS_SEQ_INPROGRESS                         ((uint8_t) 0x01 << 6)

    /* CRCERR_FUSE field mask & values */
    #define SYSTEM_STATUS_CRCERR_FUSE_MASK                                  ((uint8_t) 0x04)
    #define SYSTEM_STATUS_CRCERR_FUSE_NO_ERR_ON_PWR_UP                      ((uint8_t) 0x00 << 2) // DEFAULT
    #define SYSTEM_STATUS_CRCERR_FUSE_ERR_ON_PWR_UP                         ((uint8_t) 0x01 << 2)

    /* CRCERR_IN field mask & values */
    #define SYSTEM_STATUS_CRCERR_IN_MASK                                    ((uint8_t) 0x02)
    #define SYSTEM_STATUS_CRCERR_IN_NO_CRC_ERR                              ((uint8_t) 0x00 << 1) // DEFAULT
    #define SYSTEM_STATUS_CRCERR_IN_INPUT_CRC_ERR                           ((uint8_t) 0x01 << 1)

    /* BOR field mask & values */
    #define SYSTEM_STATUS_BOR_MASK                                          ((uint8_t) 0x01)
    #define SYSTEM_STATUS_BOR_NOT_DETECTED                                  ((uint8_t) 0x00 << 0)
    #define SYSTEM_STATUS_BOR_DETECTED_OR_DEVICE_PWR_CYCLED                 ((uint8_t) 0x01 << 0) // DEFAULT


/* Register 0x01 (GENERAL_CFG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |  REF_EN   |   CRC_EN   |     RESERVED[5:4]       |    RANGE   |    CH_RST  |     CAL    |     RST    |
 * --------------------------------------------------------------------------------------------------------
 */

    /* GENERAL_CFG register address & default value */
    #define GENERAL_CFG_ADDRESS                                             ((uint8_t) 0x01)
    #define GENERAL_CFG_DEFAULT                                             ((uint8_t) 0x00)

    /* REF_EN field mask & values */
    #define GENERAL_CFG_REF_EN_MASK                                         ((uint8_t) 0x80)
    #define GENERAL_CFG_REF_EN_INTERNAL_REF_DISABLED                        ((uint8_t) 0x00 << 7) // DEFAULT
    #define GENERAL_CFG_REF_EN_INTERNAL_REF_ENABLED                         ((uint8_t) 0x01 << 7)

    /* CRC_EN field mask & values */
    #define GENERAL_CFG_CRC_EN_MASK                                         ((uint8_t) 0x40)
    #define GENERAL_CFG_CRC_EN_DISABLED                                     ((uint8_t) 0x00 << 6) // DEFAULT
    #define GENERAL_CFG_CRC_EN_ENABLED                                      ((uint8_t) 0x01 << 6)

    /* RESERVED field mask */
    #define GENERAL_CFG_RESERVED_MASK                                       ((uint8_t) 0x30)

    /* RANGE field mask & values */
    #define GENERAL_CFG_RANGE_MASK                                          ((uint8_t) 0x08)
    #define GENERAL_CFG_RANGE_ADC_INPUT_RANGE_ONE_VREF                      ((uint8_t) 0x00 << 3) // DEFAULT
    #define GENERAL_CFG_RANGE_ADC_INPUT_RANGE_TWO_VREF                      ((uint8_t) 0x01 << 3)

    /* CH_RST field mask & values */
    #define GENERAL_CFG_CH_RST_MASK                                         ((uint8_t) 0x04)
    #define GENERAL_CFG_CH_RST_NO_OPERATION                                 ((uint8_t) 0x00 << 2) // DEFAULT
    #define GENERAL_CFG_CH_RST_SET_ALL_CH_AS_ANALOG_INPUTS                  ((uint8_t) 0x01 << 2)

    /* CAL field mask & values */
    #define GENERAL_CFG_CAL_MASK                                            ((uint8_t) 0x02)
    #define GENERAL_CFG_CAL_NO_OPERATION                                    ((uint8_t) 0x00 << 1) // DEFAULT
    #define GENERAL_CFG_CAL_CALIBRATE_ADC_OFFSET                            ((uint8_t) 0x01 << 1)

    /* RST field mask & values */
    #define GENERAL_CFG_RST_MASK                                            ((uint8_t) 0x01)
    #define GENERAL_CFG_RST_NO_OPERATION                                    ((uint8_t) 0x00 << 0) // DEFAULT
    #define GENERAL_CFG_RST_RESET_ADC_OFFSET                                ((uint8_t) 0x01 << 0)


/* Register 0x02 (DATA_CFG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |  FIX_PAT  |  RESERVED  |   APPEND_STATUS[5:4]    |        RESERVED         |      CPOL_CPHA[1:0]     |
 * --------------------------------------------------------------------------------------------------------
 */

    /* DATA_CFG register address & default value */
    #define DATA_CFG_ADDRESS                                                ((uint8_t) 0x02)
    #define DATA_CFG_DEFAULT                                                ((uint8_t) 0x00)

    /* FIX_PAT field mask & values */
    #define DATA_CFG_FIX_PAT_MASK                                           ((uint8_t) 0x080)
    #define DATA_CFG_FIX_PAT_DISABLED                                       ((uint8_t) 0x00 << 7) // DEFAULT
    #define DATA_CFG_FIX_PAT_ENABLED                                        ((uint8_t) 0x01 << 7)

    /* RESERVED field mask */
    #define DATA_CFG_RESERVED_MASK                                          ((uint8_t) 0x4C)

    /* APPEND_STATUS field mask & values */
    #define DATA_CFG_APPEND_STATUS_MASK                                     ((uint8_t) 0x030)
    #define DATA_CFG_APPEND_STATUS_DISABLED                                 ((uint8_t) 0x00 << 4) // DEFAULT
    #define DATA_CFG_APPEND_STATUS_FOUR_BIT_CHID                            ((uint8_t) 0x01 << 4)
    #define DATA_CFG_APPEND_STATUS_FOUR_BIT_STATUS_FLAGS                    ((uint8_t) 0x02 << 4)
    #define DATA_CFG_APPEND_STATUS_RESERVED                                 ((uint8_t) 0x03 << 4)

    /* CPOL_CPHA field mask & values */
    #define DATA_CFG_CPOL_CPHA_MASK                                         ((uint8_t) 0x003)
    #define DATA_CFG_CPOL_CPHA_POLARITY0_PHASE0                             ((uint8_t) 0x00 << 0) // DEFAULT
    #define DATA_CFG_CPOL_CPHA_POLARITY0_PHASE1                             ((uint8_t) 0x01 << 0)
    #define DATA_CFG_CPOL_CPHA_POLARITY1_PHASE0                             ((uint8_t) 0x02 << 0)
    #define DATA_CFG_CPOL_CPHA_POLARITY1_PHASE1                             ((uint8_t) 0x03 << 0)


/* Register 0x03 (OSR_CFG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                         RESERVED                              |                OSR[2:0]              |
 * --------------------------------------------------------------------------------------------------------
 */

    /* OSR_CFG  register address & default value */
    #define OSR_CFG_ADDRESS                                                 ((uint8_t) 0x03)
    #define OSR_CFG_DEFAULT                                                 ((uint8_t) 0x00)

    /* RESERVED field mask */
    #define OSR_CFG_RESERVED_MASK                                           ((uint8_t) 0xF8)

    /* CPOL_CPHA field mask & values */
    #define OSR_CFG_OSR_MASK                                                ((uint8_t) 0x007)
    #define OSR_CFG_OSR_NO_AVERAGING                                        ((uint8_t) 0x00 << 0) // DEFAULT
    #define OSR_CFG_OSR_2                                                   ((uint8_t) 0x01 << 0)
    #define OSR_CFG_OSR_4                                                   ((uint8_t) 0x02 << 0)
    #define OSR_CFG_OSR_8                                                   ((uint8_t) 0x03 << 0)
    #define OSR_CFG_OSR_16                                                  ((uint8_t) 0x04 << 0)
    #define OSR_CFG_OSR_32                                                  ((uint8_t) 0x05 << 0)
    #define OSR_CFG_OSR_64                                                  ((uint8_t) 0x06 << 0)
    #define OSR_CFG_OSR_128                                                 ((uint8_t) 0x07 << 0)


/* Register 0x04 (OPMODE_CFG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |              RESERVED               |   OSC_SEL  |                    CLK_DIV[3:0]                   |
 * --------------------------------------------------------------------------------------------------------
 */

    /* OPMODE_CFG register address & default value */
    #define OPMODE_CFG_ADDRESS                                              ((uint8_t) 0x04)
    #define OPMODE_CFG_DEFAULT                                              ((uint8_t) 0x00)

    /* RESERVED field mask */
    #define OPMODE_CFG_RESERVED_MASK                                        ((uint8_t) 0xE0)

    /* OSC_SEL field mask & values */
    #define OPMODE_CFG_OSC_SEL_MASK                                         ((uint8_t) 0x10)
    #define OPMODE_CFG_OSC_SEL_HIGH_SPEED                                   ((uint8_t) 0x00 << 4) // DEFAULT
    #define OPMODE_CFG_OSC_SEL_LOW_SPEED                                    ((uint8_t) 0x01 << 4)

    /* CLK_DIV field mask & values */
    #define OPMODE_CFG_CLK_DIV_MASK                                         ((uint8_t) 0x0F)
    #define OPMODE_CFG_CLK_DIV_0                                            ((uint8_t) 0x00 << 0) // DEFAULT
    #define OPMODE_CFG_CLK_DIV_1                                            ((uint8_t) 0x01 << 0)
    #define OPMODE_CFG_CLK_DIV_2                                            ((uint8_t) 0x02 << 0)
    #define OPMODE_CFG_CLK_DIV_3                                            ((uint8_t) 0x03 << 0)
    #define OPMODE_CFG_CLK_DIV_4                                            ((uint8_t) 0x04 << 0)
    #define OPMODE_CFG_CLK_DIV_5                                            ((uint8_t) 0x05 << 0)
    #define OPMODE_CFG_CLK_DIV_6                                            ((uint8_t) 0x06 << 0)
    #define OPMODE_CFG_CLK_DIV_7                                            ((uint8_t) 0x07 << 0)
    #define OPMODE_CFG_CLK_DIV_8                                            ((uint8_t) 0x08 << 0)
    #define OPMODE_CFG_CLK_DIV_9                                            ((uint8_t) 0x09 << 0)
    #define OPMODE_CFG_CLK_DIV_10                                           ((uint8_t) 0x0A << 0)
    #define OPMODE_CFG_CLK_DIV_11                                           ((uint8_t) 0x0B << 0)
    #define OPMODE_CFG_CLK_DIV_12                                           ((uint8_t) 0x0C << 0)
    #define OPMODE_CFG_CLK_DIV_13                                           ((uint8_t) 0x0D << 0)
    #define OPMODE_CFG_CLK_DIV_14                                           ((uint8_t) 0x0E << 0)
    #define OPMODE_CFG_CLK_DIV_15                                           ((uint8_t) 0x0F << 0)


/* Register 0x05 (PIN_CFG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                             PIN_CFG[7:0]                                             |
 * --------------------------------------------------------------------------------------------------------
 */

    /* PIN_CFG  register address & default value */
    #define PIN_CFG_ADDRESS                                                 ((uint8_t) 0x05)
    #define PIN_CFG_DEFAULT                                                 ((uint8_t) 0x00)

    /* PIN_CFG field mask & values */
    #define PIN_CFG_PIN_CFG_MASK                                            ((uint8_t) 0x0FF)
    #define PIN_CFG_PIN_CFG_CH7_ANALOG_INPUT                                ((uint8_t) 0x00 << 7) // DEFAULT
    #define PIN_CFG_PIN_CFG_CH7_GPIO                                        ((uint8_t) 0x01 << 7)
    #define PIN_CFG_PIN_CFG_CH6_ANALOG_INPUT                                ((uint8_t) 0x00 << 6) // DEFAULT
    #define PIN_CFG_PIN_CFG_CH6_GPIO                                        ((uint8_t) 0x01 << 6)
    #define PIN_CFG_PIN_CFG_CH5_ANALOG_INPUT                                ((uint8_t) 0x00 << 5) // DEFAULT
    #define PIN_CFG_PIN_CFG_CH5_GPIO                                        ((uint8_t) 0x01 << 5)
    #define PIN_CFG_PIN_CFG_CH4_ANALOG_INPUT                                ((uint8_t) 0x00 << 4) // DEFAULT
    #define PIN_CFG_PIN_CFG_CH4_GPIO                                        ((uint8_t) 0x01 << 4)
    #define PIN_CFG_PIN_CFG_CH3_ANALOG_INPUT                                ((uint8_t) 0x00 << 3) // DEFAULT
    #define PIN_CFG_PIN_CFG_CH3_GPIO                                        ((uint8_t) 0x01 << 3)
    #define PIN_CFG_PIN_CFG_CH2_ANALOG_INPUT                                ((uint8_t) 0x00 << 2) // DEFAULT
    #define PIN_CFG_PIN_CFG_CH2_GPIO                                        ((uint8_t) 0x01 << 2)
    #define PIN_CFG_PIN_CFG_CH1_ANALOG_INPUT                                ((uint8_t) 0x00 << 1) // DEFAULT
    #define PIN_CFG_PIN_CFG_CH1_GPIO                                        ((uint8_t) 0x01 << 1)
    #define PIN_CFG_PIN_CFG_CH0_ANALOG_INPUT                                ((uint8_t) 0x00 << 0) // DEFAULT
    #define PIN_CFG_PIN_CFG_CH0_GPIO                                        ((uint8_t) 0x01 << 0)


/* Register 0x07 (GPIO_CFG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                            GPIO_CFG[7:0]                                             |
 * --------------------------------------------------------------------------------------------------------
 */
    /* GPIO_CFG register address & default value */
    #define GPIO_CFG_ADDRESS                                                ((uint8_t) 0x07)
    #define GPIO_CFG_DEFAULT                                                ((uint8_t) 0x00)

    /* GPIO_CFG field mask & values */
    #define GPIO_CFG_GPIO_CFG_MASK                                          ((uint8_t) 0x0FF)
    #define GPIO_CFG_GPIO_CFG_CH7_DIGITAL_INPUT                             ((uint8_t) 0x00 << 7) // DEFAULT
    #define GPIO_CFG_GPIO_CFG_CH7_DIGITAL_OUTPUT                            ((uint8_t) 0x01 << 7)
    #define GPIO_CFG_GPIO_CFG_CH6_DIGITAL_INPUT                             ((uint8_t) 0x00 << 6) // DEFAULT
    #define GPIO_CFG_GPIO_CFG_CH6_DIGITAL_OUTPUT                            ((uint8_t) 0x01 << 6)
    #define GPIO_CFG_GPIO_CFG_CH5_DIGITAL_INPUT                             ((uint8_t) 0x00 << 5) // DEFAULT
    #define GPIO_CFG_GPIO_CFG_CH5_DIGITAL_OUTPUT                            ((uint8_t) 0x01 << 5)
    #define GPIO_CFG_GPIO_CFG_CH4_DIGITAL_INPUT                             ((uint8_t) 0x00 << 4) // DEFAULT
    #define GPIO_CFG_GPIO_CFG_CH4_DIGITAL_OUTPUT                            ((uint8_t) 0x01 << 4)
    #define GPIO_CFG_GPIO_CFG_CH3_DIGITAL_INPUT                             ((uint8_t) 0x00 << 3) // DEFAULT
    #define GPIO_CFG_GPIO_CFG_CH3_DIGITAL_OUTPUT                            ((uint8_t) 0x01 << 3)
    #define GPIO_CFG_GPIO_CFG_CH2_DIGITAL_INPUT                             ((uint8_t) 0x00 << 2) // DEFAULT
    #define GPIO_CFG_GPIO_CFG_CH2_DIGITAL_OUTPUT                            ((uint8_t) 0x01 << 2)
    #define GPIO_CFG_GPIO_CFG_CH1_DIGITAL_INPUT                             ((uint8_t) 0x00 << 1) // DEFAULT
    #define GPIO_CFG_GPIO_CFG_CH1_DIGITAL_OUTPUT                            ((uint8_t) 0x01 << 1)
    #define GPIO_CFG_GPIO_CFG_CH0_DIGITAL_INPUT                             ((uint8_t) 0x00 << 0) // DEFAULT
    #define GPIO_CFG_GPIO_CFG_CH0_DIGITAL_OUTPUT                            ((uint8_t) 0x01 << 0)


/* Register 0x09 (GPO_DRIVE_CFG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                         GPO_DRIVE_CFG[7:0]                                           |
 * --------------------------------------------------------------------------------------------------------
 */

    /* GPO_DRIVE_CFG  register address & default value */
    #define GPO_DRIVE_CFG_ADDRESS                                           ((uint8_t) 0x09)
    #define GPO_DRIVE_CFG_DEFAULT                                           ((uint8_t) 0x00)

    /* GPO_DRIVE_CFG field mask & values */
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_MASK                                ((uint8_t) 0x0FF)
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH7_OPEN_DRAIN_OUTPUT               ((uint8_t) 0x00 << 7) // DEFAULT
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH7_PUSH_PULL_OUTPUT                ((uint8_t) 0x01 << 7)
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH6_OPEN_DRAIN_OUTPUT               ((uint8_t) 0x00 << 6) // DEFAULT
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH6_PUSH_PULL_OUTPUT                ((uint8_t) 0x01 << 6)
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH5_OPEN_DRAIN_OUTPUT               ((uint8_t) 0x00 << 5) // DEFAULT
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH5_PUSH_PULL_OUTPUT                ((uint8_t) 0x01 << 5)
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH4_OPEN_DRAIN_OUTPUT               ((uint8_t) 0x00 << 4) // DEFAULT
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH4_PUSH_PULL_OUTPUT                ((uint8_t) 0x01 << 4)
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH3_OPEN_DRAIN_OUTPUT               ((uint8_t) 0x00 << 3) // DEFAULT
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH3_PUSH_PULL_OUTPUT                ((uint8_t) 0x01 << 3)
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH2_OPEN_DRAIN_OUTPUT               ((uint8_t) 0x00 << 2) // DEFAULT
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH2_PUSH_PULL_OUTPUT                ((uint8_t) 0x01 << 2)
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH1_OPEN_DRAIN_OUTPUT               ((uint8_t) 0x00 << 1) // DEFAULT
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH1_PUSH_PULL_OUTPUT                ((uint8_t) 0x01 << 1)
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH0_OPEN_DRAIN_OUTPUT               ((uint8_t) 0x00 << 0) // DEFAULT
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH0_PUSH_PULL_OUTPUT                ((uint8_t) 0x01 << 0)


/* Register 0x0B (GPO_OUTPUT_VALUE) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                       GPO_OUTPUT_VALUE[7:0]                                          |
 * --------------------------------------------------------------------------------------------------------
 */
    /* GPO_OUTPUT_VALUE register address & default value */
    #define GPO_OUTPUT_VALUE_ADDRESS                                        ((uint8_t) 0x0B)
    #define GPO_OUTPUT_VALUE_DEFAULT                                        ((uint8_t) 0x00)

    /* GPO_OUTPUT_VALUE field mask & values */
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_MASK                          ((uint8_t) 0x0FF)
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH7_LOW                       ((uint8_t) 0x00 << 7) // DEFAULT
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH7_HIGH                      ((uint8_t) 0x01 << 7)
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH6_LOW                       ((uint8_t) 0x00 << 6) // DEFAULT
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH6_HIGH                      ((uint8_t) 0x01 << 6)
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH5_LOW                       ((uint8_t) 0x00 << 5) // DEFAULT
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH5_HIGH                      ((uint8_t) 0x01 << 5)
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH4_LOW                       ((uint8_t) 0x00 << 4) // DEFAULT
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH4_HIGH                      ((uint8_t) 0x01 << 4)
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH3_LOW                       ((uint8_t) 0x00 << 3) // DEFAULT
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH3_HIGH                      ((uint8_t) 0x01 << 3)
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH2_LOW                       ((uint8_t) 0x00 << 2) // DEFAULT
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH2_HIGH                      ((uint8_t) 0x01 << 2)
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH1_LOW                       ((uint8_t) 0x00 << 1) // DEFAULT
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH1_HIGH                      ((uint8_t) 0x01 << 1)
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH0_LOW                       ((uint8_t) 0x00 << 0) // DEFAULT
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH0_HIGH                      ((uint8_t) 0x01 << 0)


/* Register 0x0D (GPI_VALUE) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                            GPI_VALUE[7:0]                                            |
 * --------------------------------------------------------------------------------------------------------
 */

    /* GPI_VALUE register address & default value */
    #define GPI_VALUE_ADDRESS                                               ((uint8_t) 0x0D)
    #define GPI_VALUE_DEFAULT                                               ((uint8_t) 0x00)

    /* GPI_OUTPUT_VALUE field mask & values */
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_MASK                          ((uint8_t) 0x0FF)
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH7_LOW                       ((uint8_t) 0x00 << 7) // DEFAULT
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH7_HIGH                      ((uint8_t) 0x01 << 7)
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH6_LOW                       ((uint8_t) 0x00 << 6) // DEFAULT
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH6_HIGH                      ((uint8_t) 0x01 << 6)
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH5_LOW                       ((uint8_t) 0x00 << 5) // DEFAULT
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH5_HIGH                      ((uint8_t) 0x01 << 5)
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH4_LOW                       ((uint8_t) 0x00 << 4) // DEFAULT
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH4_HIGH                      ((uint8_t) 0x01 << 4)
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH3_LOW                       ((uint8_t) 0x00 << 3) // DEFAULT
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH3_HIGH                      ((uint8_t) 0x01 << 3)
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH2_LOW                       ((uint8_t) 0x00 << 2) // DEFAULT
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH2_HIGH                      ((uint8_t) 0x01 << 2)
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH1_LOW                       ((uint8_t) 0x00 << 1) // DEFAULT
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH1_HIGH                      ((uint8_t) 0x01 << 1)
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH0_LOW                       ((uint8_t) 0x00 << 0) // DEFAULT
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH0_HIGH                      ((uint8_t) 0x01 << 0)


/* Register 0x10 (SEQUENCE_CFG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |        RESERVED        |        SEQ_START        |        RESERVED         |      SEQ_MODE[1:0]      |
 * --------------------------------------------------------------------------------------------------------
 */

    /* SEQUENCE_CFG register address & default value */
    #define SEQUENCE_CFG_ADDRESS                                            ((uint8_t) 0x10)
    #define SEQUENCE_CFG_DEFAULT                                            ((uint8_t) 0x00)

    /* RESERVED field mask */
    #define SEQUENCE_CFG_RESERVED_MASK                                      ((uint8_t) 0xEC)

    /* SEQ_START field mask & values */
    #define SEQUENCE_CFG_SEQ_START_MASK                                     ((uint8_t) 0x010)
    #define SEQUENCE_CFG_SEQ_START_DISABLED                                 ((uint8_t) 0x00 << 4) // DEFAULT
    #define SEQUENCE_CFG_SEQ_START_ENABLED                                  ((uint8_t) 0x01 << 4)

    /* SEQ_MODE field mask & values */
    #define SEQUENCE_CFG_SEQ_MODE_MASK                                      ((uint8_t) 0x03)
    #define SEQUENCE_CFG_SEQ_MODE_MANUAL                                    ((uint8_t) 0x00 << 0) // DEFAULT
    #define SEQUENCE_CFG_SEQ_MODE_AUTO_SEQ                                  ((uint8_t) 0x01 << 0)
    #define SEQUENCE_CFG_SEQ_MODE_ON_THE_FLY                                ((uint8_t) 0x02 << 0)
    #define SEQUENCE_CFG_SEQ_MODE_RESERVED                                  ((uint8_t) 0x03 << 0)


/* Register 0x11 (CHANNEL_SEL) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                     RESERVED                     |                   MANUAL_CHID[3:0]                |
 * --------------------------------------------------------------------------------------------------------
 */
    /* CHANNEL_SEL register address & default value */
    #define CHANNEL_SEL_ADDRESS                                             ((uint8_t) 0x11)
    #define CHANNEL_SEL_DEFAULT                                             ((uint8_t) 0x00)

    /* RESERVED field mask */
    #define CHANNEL_SEL_RESERVED_MASK                                       ((uint8_t) 0xF0)

    /* MANUAL_CHID field mask & values */
    #define CHANNEL_SEL_MANUAL_CHID_MASK                                    ((uint8_t) 0x0F)
    #define CHANNEL_SEL_MANUAL_CHID_0                                       ((uint8_t) 0x00 << 0) // DEFAULT
    #define CHANNEL_SEL_MANUAL_CHID_1                                       ((uint8_t) 0x01 << 0)
    #define CHANNEL_SEL_MANUAL_CHID_2                                       ((uint8_t) 0x02 << 0)
    #define CHANNEL_SEL_MANUAL_CHID_3                                       ((uint8_t) 0x03 << 0)
    #define CHANNEL_SEL_MANUAL_CHID_4                                       ((uint8_t) 0x04 << 0)
    #define CHANNEL_SEL_MANUAL_CHID_5                                       ((uint8_t) 0x05 << 0)
    #define CHANNEL_SEL_MANUAL_CHID_6                                       ((uint8_t) 0x06 << 0)
    #define CHANNEL_SEL_MANUAL_CHID_7                                       ((uint8_t) 0x07 << 0)


/* Register 0x12 (AUTO_SEQ_CHSEL) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                         AUTO_SEQ_CHSEL[7:0]                                          |
 * --------------------------------------------------------------------------------------------------------
 */

    /* AUTO_SEQ_CHSEL register address & default value */
    #define AUTO_SEQ_CHSEL_ADDRESS                                          ((uint8_t) 0x12)
    #define AUTO_SEQ_CHSEL_DEFAULT                                          ((uint8_t) 0x00)

    /* AUTO_SEQ_CHSEL field mask & values */
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_MASK                              ((uint8_t) 0x0FF)
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH7_DISABLED                      ((uint8_t) 0x00 << 7) // DEFAULT
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH7_ENABLED                       ((uint8_t) 0x01 << 7)
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH6_DISABLED                      ((uint8_t) 0x00 << 6) // DEFAULT
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH6_ENABLED                       ((uint8_t) 0x01 << 6)
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH5_DISABLED                      ((uint8_t) 0x00 << 5) // DEFAULT
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH5_ENABLED                       ((uint8_t) 0x01 << 5)
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH4_DISABLED                      ((uint8_t) 0x00 << 4) // DEFAULT
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH4_ENABLED                       ((uint8_t) 0x01 << 4)
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH3_DISABLED                      ((uint8_t) 0x00 << 3) // DEFAULT
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH3_ENABLED                       ((uint8_t) 0x01 << 3)
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH2_DISABLED                      ((uint8_t) 0x00 << 2) // DEFAULT
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH2_ENABLED                       ((uint8_t) 0x01 << 2)
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH1_DISABLED                      ((uint8_t) 0x00 << 1) // DEFAULT
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH1_ENABLED                       ((uint8_t) 0x01 << 1)
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH0_DISABLED                      ((uint8_t) 0x00 << 0) // DEFAULT
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH0_ENABLED                       ((uint8_t) 0x01 << 0)


/* Register 0xBF (DIAGNOSTICS_KEY) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                              DIAG_KEY[7:0]                                           |
 * --------------------------------------------------------------------------------------------------------
 */

    /* DIAGNOSTICS_KEY register address & default value */
    #define DIAGNOSTICS_KEY_ADDRESS                                         ((uint8_t) 0xBF)
    #define DIAGNOSTICS_KEY_DEFAULT                                         ((uint8_t) 0x00)

    /* DIAG_KEY field mask & values */
    #define DIAGNOSTICS_KEY_DIAG_KEY_MASK                                   ((uint8_t) 0x010)
    #define DIAGNOSTICS_KEY_DIAG_KEY_REG_WR_ACCESS_DISABLED                 ((uint8_t) 0x00 << 0) // DEFAULT
    #define DIAGNOSTICS_KEY_DIAG_KEY_REG_WR_ACCESS_ENABLED                  ((uint8_t) 0x96 << 0)


/* Register 0xC0 (BIT_WALK) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                         RESERVED                                        |    DIAG_EN |
 * --------------------------------------------------------------------------------------------------------
 */

    /* BIT_WALK register address & default value */
    #define BIT_WALK_ADDRESS                                                ((uint8_t) 0xC0)
    #define BIT_WALK_DEFAULT                                                ((uint8_t) 0x00)

    /* RESERVED field mask */
    #define BIT_WALK_RESERVED_MASK                                          ((uint8_t) 0xFE)

    /* DIAG_EN field mask & values */
    #define BIT_WALK_DIAG_EN_MASK                                           ((uint8_t) 0x010)
    #define BIT_WALK_DIAG_EN_DISABLED                                       ((uint8_t) 0x00 << 0) // DEFAULT
    #define BIT_WALK_DIAG_EN_ENABLED                                        ((uint8_t) 0x01 << 0)


/* Register 0xC1 (BIT_SAMPLE_LSB) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                          BIT_SAMPLE_LSB[7:0]                                         |
 * --------------------------------------------------------------------------------------------------------
 */

    /* BIT_SAMPLE_LSB register address & default value */
    #define BIT_SAMPLE_LSB_ADDRESS                                          ((uint8_t) 0xC1)
    #define BIT_SAMPLE_LSB_DEFAULT                                          ((uint8_t) 0x00)


/* Register 0xC2 (BIT_SAMPLE_MSB) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                          BIT_SAMPLE_MSB[7:0]                                         |
 * --------------------------------------------------------------------------------------------------------
 */

    /* BIT_SAMPLE_MSB register address & default value */
    #define BIT_SAMPLE_MSB_ADDRESS                                          ((uint8_t) 0xC2)
    #define BIT_SAMPLE_MSB_DEFAULT                                          ((uint8_t) 0x00)


//****************************************************************************
//
// Register macros
//
//****************************************************************************

/* Returns true if SPI CRC enable bit is set */
#define SPI_CRC_ENABLED         ((bool) ((getRegisterValue(GENERAL_CFG_ADDRESS) \
                                            & GENERAL_CFG_CRC_EN_MASK) >> 6))

/* Returns true if SPI CRC error bit is set in device */
#define SPI_CRC_ERROR           ((bool) ((getRegisterValue(SYSTEM_STATUS_ADDRESS) \
                                            & SYSTEM_STATUS_CRCERR_IN_MASK) >> 1))

/* Returns true if Auto-Sequence capture mode is selected */
#define AUTOSEQ_MODE_ENABLED    ((bool) ((getRegisterValue(SEQUENCE_CFG_ADDRESS) \
                                            & SEQUENCE_CFG_SEQ_MODE_MASK) == SEQUENCE_CFG_SEQ_MODE_AUTO_SEQ))

/* Returns true if channel sequencing in auto-sequence mode is enabled */
#define AUTOSEQ_CH_SEQ_ENABLED  ((bool) ((getRegisterValue(SEQUENCE_CFG_ADDRESS) \
                                            & SEQUENCE_CFG_SEQ_START_MASK) >> 4))

/* Returns true if channel sequencing in auto-sequence mode is enabled */
#define AVERAGING_ENABLED       ((bool) (getRegisterValue(OSR_CFG_ADDRESS) & OSR_CFG_OSR_MASK))


//****************************************************************************
//
// Function prototypes
//
//****************************************************************************
void        initADS7066(void);
void        setChannelAsAnalogInput(uint8_t channelID);
void        startManualConversions(uint8_t channelID, uint32_t samplesPerSecond);
void        stopConversions(void);
int32_t     readData(uint8_t dataRx[]);
uint8_t     readSingleRegister(uint8_t address);
uint8_t     getRegisterValue(uint8_t address);
void        writeSingleRegister(uint8_t address, uint8_t data);
void        setRegisterBits(uint8_t address, uint8_t bitMask);
void        clearRegisterBits(uint8_t address, uint8_t bitMask);
uint8_t     calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint8_t initialValue);


#endif /* ADS7066_H_ */
