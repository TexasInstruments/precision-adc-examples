/*
 * \brief This header file contains all register map definitions for the ADS7038 device family.
 *
 * \copyright Copyright (C) 2019-2021 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef ADS7038_H_
#define ADS7038_H_

// Standard libraries
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

// Custom libraries
#include "hal.h"


//****************************************************************************
//
// Settings
//
//****************************************************************************

/** Disable assertions when not in the CCS "Debug" configuration */
#ifndef _DEBUG
    #define NDEBUG
#endif


//****************************************************************************
//
// Constants
//
//****************************************************************************

/** Initial seed value for CRC calculation */
#define CRC_INITIAL_SEED                        ((uint8_t) 0x00)


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
 * |   RSVD    | SEQ_STATUS |     RESERVED[5:3]       |  OSR_DONE  | CRCERR_FUSE|  CRCERR_IN |     BOR    |
 * --------------------------------------------------------------------------------------------------------
 *  Bits 0, 1 and 3 alone supports both read and write operations. Bits 2 and 4 to 7 supports only read.
 */

    /* SYSTEM_STATUS register address & default value */
    #define SYSTEM_STATUS_ADDRESS                                           ((uint8_t) 0x00)
    #define SYSTEM_STATUS_DEFAULT                                           ((uint8_t) 0x81)

    /* RESERVED field mask */
    #define SYSTEM_STATUS_RESERVED_MASK                                     ((uint8_t) 0xB0)

    /* SEQ_STATUS field mask & values */
    #define SYSTEM_STATUS_SEQ_STATUS_MASK                                   ((uint8_t) 0x40)
    #define SYSTEM_STATUS_SEQ_STATUS_SEQ_STOPPED                            ((uint8_t) 0x00 << 6) // DEFAULT
    #define SYSTEM_STATUS_SEQ_STATUS_SEQ_INPROGRESS                         ((uint8_t) 0x01 << 6)

    /* OSR_DONE field mask & values */
    #define SYSTEM_STATUS_OSR_DONE_MASK                                     ((uint8_t) 0x08)
    #define SYSTEM_STATUS_OSR_DONE_AVG_RSLT_NOT_RDY                         ((uint8_t) 0x00 << 3) // DEFAULT
    #define SYSTEM_STATUS_OSR_DONE_AVG_RSLT_RDY                             ((uint8_t) 0x01 << 3)

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
    #define SYSTEM_STATUS_BOR_DETECTED                                      ((uint8_t) 0x01 << 0) // DEFAULT


/* Register 0x01 (GENERAL_CFG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |   RSVD    |   CRC_EN   |  STATS_EN  |   DWC_EN   |    RSVD    |    CH_RST  |     CAL    |     RST    |
 * --------------------------------------------------------------------------------------------------------
 */

    /* GENERAL_CFG register address & default value */
    #define GENERAL_CFG_ADDRESS                                             ((uint8_t) 0x01)
    #define GENERAL_CFG_DEFAULT                                             ((uint8_t) 0x00)

    /* RESERVED field mask */
    #define GENERAL_CFG_RESERVED_MASK                                       ((uint8_t) 0x88)

    /* CRC_EN field mask & values */
    #define GENERAL_CFG_CRC_EN_MASK                                         ((uint8_t) 0x40)
    #define GENERAL_CFG_CRC_EN_DISABLED                                     ((uint8_t) 0x00 << 6) // DEFAULT
    #define GENERAL_CFG_CRC_EN_ENABLED                                      ((uint8_t) 0x01 << 6)

    /* STATS_EN field mask & values */
    #define GENERAL_CFG_STATS_EN_MASK                                       ((uint8_t) 0x20)
    #define GENERAL_CFG_STATS_EN_STATS_DISABLED                             ((uint8_t) 0x00 << 5) // DEFAULT
    #define GENERAL_CFG_STATS_EN_STATS_ENABLED                              ((uint8_t) 0x01 << 5)

    /* DWC_EN field mask & values */
    #define GENERAL_CFG_DWC_EN_MASK                                         ((uint8_t) 0x10)
    #define GENERAL_CFG_DWC_EN_WNDO_COMP_DISABLED                           ((uint8_t) 0x00 << 4) // DEFAULT
    #define GENERAL_CFG_DWC_EN_WNDO_COMP_ENABLED                            ((uint8_t) 0x01 << 4)

    /* CH_RST field mask & values */
    #define GENERAL_CFG_CH_RST_MASK                                         ((uint8_t) 0x04)
    #define GENERAL_CFG_CH_RST_NORM_OPERATION                               ((uint8_t) 0x00 << 2) // DEFAULT
    #define GENERAL_CFG_CH_RST_SET_ALL_CH_AS_ANALOG_INPUTS                  ((uint8_t) 0x01 << 2)

    /* CAL field mask & values */
    #define GENERAL_CFG_CAL_MASK                                            ((uint8_t) 0x02)
    #define GENERAL_CFG_CAL_NORM_OPERATION                                  ((uint8_t) 0x00 << 1) // DEFAULT
    #define GENERAL_CFG_CAL_CALIBRATE_ADC_OFFSET                            ((uint8_t) 0x01 << 1)

    /* RST field mask & values */
    #define GENERAL_CFG_RST_MASK                                            ((uint8_t) 0x01)
    #define GENERAL_CFG_RST_NO_OPERATION                                    ((uint8_t) 0x00 << 0) // DEFAULT
    #define GENERAL_CFG_RST_RESET_DEVICE_REGS                               ((uint8_t) 0x01 << 0)


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
    #define OSR_CFG_OSR_DISABLED                                            ((uint8_t) 0x00 << 0) // DEFAULT
    #define OSR_CFG_OSR_2                                                   ((uint8_t) 0x01 << 0)
    #define OSR_CFG_OSR_4                                                   ((uint8_t) 0x02 << 0)
    #define OSR_CFG_OSR_8                                                   ((uint8_t) 0x03 << 0)
    #define OSR_CFG_OSR_16                                                  ((uint8_t) 0x04 << 0)
    #define OSR_CFG_OSR_32                                                  ((uint8_t) 0x05 << 0)
    #define OSR_CFG_OSR_64                                                  ((uint8_t) 0x06 << 0)
    #define OSR_CFG_OSR_128                                                 ((uint8_t) 0x07 << 0)


/* Register 0x04 (OPMODE_CFG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7    |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |CONV_ON_ERR |        CONV_MODE        |   OSC_SEL  |                    CLK_DIV[3:0]                   |
 * --------------------------------------------------------------------------------------------------------
 */

    /* OPMODE_CFG register address & default value */
    #define OPMODE_CFG_ADDRESS                                          ((uint8_t) 0x04)
    #define OPMODE_CFG_DEFAULT                                          ((uint8_t) 0x00)

    /* CONV_ON_ERR field mask & values */
    #define OPMODE_CFG_CONV_ON_ERR_MASK                                 ((uint8_t) 0x80)
    #define OPMODE_CFG_CONV_ON_ERR_CONT_CONV_ON_ERR                     ((uint8_t) 0x00 << 7) // DEFAULT
    #define OPMODE_CFG_CONV_ON_ERR_STOP_CONV_ON_ERR                     ((uint8_t) 0x01 << 7)

    /* CONV_MODE field mask & values */
    #define OPMODE_CFG_CONV_MODE_MASK                                   ((uint8_t) 0x60)
    #define OPMODE_CFG_CONV_MODE_MANUAL_MODE                            ((uint8_t) 0x00 << 5) // DEFAULT
    #define OPMODE_CFG_CONV_MODE_AUTONOMOUS_MODE                        ((uint8_t) 0x01 << 5)

    /* OSC_SEL field mask & values */
    #define OPMODE_CFG_OSC_SEL_MASK                                     ((uint8_t) 0x10)
    #define OPMODE_CFG_OSC_SEL_HIGH_SPEED                               ((uint8_t) 0x00 << 4) // DEFAULT
    #define OPMODE_CFG_OSC_SEL_LOW_SPEED                                ((uint8_t) 0x01 << 4)

    /* CLK_DIV field mask & values */
    #define OPMODE_CFG_CLK_DIV_MASK                                     ((uint8_t) 0x0F)
    #define OPMODE_CFG_CLK_DIV_0                                        ((uint8_t) 0x00 << 0) // DEFAULT
    #define OPMODE_CFG_CLK_DIV_1                                        ((uint8_t) 0x01 << 0)
    #define OPMODE_CFG_CLK_DIV_2                                        ((uint8_t) 0x02 << 0)
    #define OPMODE_CFG_CLK_DIV_3                                        ((uint8_t) 0x03 << 0)
    #define OPMODE_CFG_CLK_DIV_4                                        ((uint8_t) 0x04 << 0)
    #define OPMODE_CFG_CLK_DIV_5                                        ((uint8_t) 0x05 << 0)
    #define OPMODE_CFG_CLK_DIV_6                                        ((uint8_t) 0x06 << 0)
    #define OPMODE_CFG_CLK_DIV_7                                        ((uint8_t) 0x07 << 0)
    #define OPMODE_CFG_CLK_DIV_8                                        ((uint8_t) 0x08 << 0)
    #define OPMODE_CFG_CLK_DIV_9                                        ((uint8_t) 0x09 << 0)
    #define OPMODE_CFG_CLK_DIV_10                                       ((uint8_t) 0x0A << 0)
    #define OPMODE_CFG_CLK_DIV_11                                       ((uint8_t) 0x0B << 0)
    #define OPMODE_CFG_CLK_DIV_12                                       ((uint8_t) 0x0C << 0)
    #define OPMODE_CFG_CLK_DIV_13                                       ((uint8_t) 0x0D << 0)
    #define OPMODE_CFG_CLK_DIV_14                                       ((uint8_t) 0x0E << 0)
    #define OPMODE_CFG_CLK_DIV_15                                       ((uint8_t) 0x0F << 0)


/* Register 0x05 (PIN_CFG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                             PIN_CFG[7:0]                                             |
 * --------------------------------------------------------------------------------------------------------
 */

    /* PIN_CFG  register address & default value */
    #define PIN_CFG_ADDRESS                                             ((uint8_t) 0x05)
    #define PIN_CFG_DEFAULT                                             ((uint8_t) 0x00)

    /* PIN_CFG field mask & values */
    #define PIN_CFG_PIN_CFG_MASK                                        ((uint8_t) 0xFF)
    #define PIN_CFG_PIN_CFG_CH7_ANALOG_INPUT                            ((uint8_t) 0x00 << 7) // DEFAULT
    #define PIN_CFG_PIN_CFG_CH7_GPIO                                    ((uint8_t) 0x01 << 7)
    #define PIN_CFG_PIN_CFG_CH6_ANALOG_INPUT                            ((uint8_t) 0x00 << 6) // DEFAULT
    #define PIN_CFG_PIN_CFG_CH6_GPIO                                    ((uint8_t) 0x01 << 6)
    #define PIN_CFG_PIN_CFG_CH5_ANALOG_INPUT                            ((uint8_t) 0x00 << 5) // DEFAULT
    #define PIN_CFG_PIN_CFG_CH5_GPIO                                    ((uint8_t) 0x01 << 5)
    #define PIN_CFG_PIN_CFG_CH4_ANALOG_INPUT                            ((uint8_t) 0x00 << 4) // DEFAULT
    #define PIN_CFG_PIN_CFG_CH4_GPIO                                    ((uint8_t) 0x01 << 4)
    #define PIN_CFG_PIN_CFG_CH3_ANALOG_INPUT                            ((uint8_t) 0x00 << 3) // DEFAULT
    #define PIN_CFG_PIN_CFG_CH3_GPIO                                    ((uint8_t) 0x01 << 3)
    #define PIN_CFG_PIN_CFG_CH2_ANALOG_INPUT                            ((uint8_t) 0x00 << 2) // DEFAULT
    #define PIN_CFG_PIN_CFG_CH2_GPIO                                    ((uint8_t) 0x01 << 2)
    #define PIN_CFG_PIN_CFG_CH1_ANALOG_INPUT                            ((uint8_t) 0x00 << 1) // DEFAULT
    #define PIN_CFG_PIN_CFG_CH1_GPIO                                    ((uint8_t) 0x01 << 1)
    #define PIN_CFG_PIN_CFG_CH0_ANALOG_INPUT                            ((uint8_t) 0x00 << 0) // DEFAULT
    #define PIN_CFG_PIN_CFG_CH0_GPIO                                    ((uint8_t) 0x01 << 0)


/* Register 0x07 (GPIO_CFG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                            GPIO_CFG[7:0]                                             |
 * --------------------------------------------------------------------------------------------------------
 */
    /* GPIO_CFG register address & default value */
    #define GPIO_CFG_ADDRESS                                            ((uint8_t) 0x07)
    #define GPIO_CFG_DEFAULT                                            ((uint8_t) 0x00)

    /* GPIO_CFG field mask & values */
    #define GPIO_CFG_GPIO_CFG_MASK                                      ((uint8_t) 0xFF)
    #define GPIO_CFG_GPIO_CFG_CH7_DIGITAL_INPUT                         ((uint8_t) 0x00 << 7) // DEFAULT
    #define GPIO_CFG_GPIO_CFG_CH7_DIGITAL_OUTPUT                        ((uint8_t) 0x01 << 7)
    #define GPIO_CFG_GPIO_CFG_CH6_DIGITAL_INPUT                         ((uint8_t) 0x00 << 6) // DEFAULT
    #define GPIO_CFG_GPIO_CFG_CH6_DIGITAL_OUTPUT                        ((uint8_t) 0x01 << 6)
    #define GPIO_CFG_GPIO_CFG_CH5_DIGITAL_INPUT                         ((uint8_t) 0x00 << 5) // DEFAULT
    #define GPIO_CFG_GPIO_CFG_CH5_DIGITAL_OUTPUT                        ((uint8_t) 0x01 << 5)
    #define GPIO_CFG_GPIO_CFG_CH4_DIGITAL_INPUT                         ((uint8_t) 0x00 << 4) // DEFAULT
    #define GPIO_CFG_GPIO_CFG_CH4_DIGITAL_OUTPUT                        ((uint8_t) 0x01 << 4)
    #define GPIO_CFG_GPIO_CFG_CH3_DIGITAL_INPUT                         ((uint8_t) 0x00 << 3) // DEFAULT
    #define GPIO_CFG_GPIO_CFG_CH3_DIGITAL_OUTPUT                        ((uint8_t) 0x01 << 3)
    #define GPIO_CFG_GPIO_CFG_CH2_DIGITAL_INPUT                         ((uint8_t) 0x00 << 2) // DEFAULT
    #define GPIO_CFG_GPIO_CFG_CH2_DIGITAL_OUTPUT                        ((uint8_t) 0x01 << 2)
    #define GPIO_CFG_GPIO_CFG_CH1_DIGITAL_INPUT                         ((uint8_t) 0x00 << 1) // DEFAULT
    #define GPIO_CFG_GPIO_CFG_CH1_DIGITAL_OUTPUT                        ((uint8_t) 0x01 << 1)
    #define GPIO_CFG_GPIO_CFG_CH0_DIGITAL_INPUT                         ((uint8_t) 0x00 << 0) // DEFAULT
    #define GPIO_CFG_GPIO_CFG_CH0_DIGITAL_OUTPUT                        ((uint8_t) 0x01 << 0)


/* Register 0x09 (GPO_DRIVE_CFG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                         GPO_DRIVE_CFG[7:0]                                           |
 * --------------------------------------------------------------------------------------------------------
 */

    /* GPO_DRIVE_CFG  register address & default value */
    #define GPO_DRIVE_CFG_ADDRESS                                       ((uint8_t) 0x09)
    #define GPO_DRIVE_CFG_DEFAULT                                       ((uint8_t) 0x00)

    /* GPO_DRIVE_CFG field mask & values */
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_MASK                            ((uint8_t) 0xFF)
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH7_OPEN_DRAIN_OUTPUT           ((uint8_t) 0x00 << 7) // DEFAULT
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH7_PUSH_PULL_OUTPUT            ((uint8_t) 0x01 << 7)
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH6_OPEN_DRAIN_OUTPUT           ((uint8_t) 0x00 << 6) // DEFAULT
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH6_PUSH_PULL_OUTPUT            ((uint8_t) 0x01 << 6)
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH5_OPEN_DRAIN_OUTPUT           ((uint8_t) 0x00 << 5) // DEFAULT
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH5_PUSH_PULL_OUTPUT            ((uint8_t) 0x01 << 5)
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH4_OPEN_DRAIN_OUTPUT           ((uint8_t) 0x00 << 4) // DEFAULT
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH4_PUSH_PULL_OUTPUT            ((uint8_t) 0x01 << 4)
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH3_OPEN_DRAIN_OUTPUT           ((uint8_t) 0x00 << 3) // DEFAULT
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH3_PUSH_PULL_OUTPUT            ((uint8_t) 0x01 << 3)
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH2_OPEN_DRAIN_OUTPUT           ((uint8_t) 0x00 << 2) // DEFAULT
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH2_PUSH_PULL_OUTPUT            ((uint8_t) 0x01 << 2)
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH1_OPEN_DRAIN_OUTPUT           ((uint8_t) 0x00 << 1) // DEFAULT
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH1_PUSH_PULL_OUTPUT            ((uint8_t) 0x01 << 1)
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH0_OPEN_DRAIN_OUTPUT           ((uint8_t) 0x00 << 0) // DEFAULT
    #define GPO_DRIVE_CFG_GPO_DRIVE_CFG_CH0_PUSH_PULL_OUTPUT            ((uint8_t) 0x01 << 0)


/* Register 0x0B (GPO_OUTPUT_VALUE) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                       GPO_OUTPUT_VALUE[7:0]                                          |
 * --------------------------------------------------------------------------------------------------------
 */
    /* GPO_OUTPUT_VALUE register address & default value */
    #define GPO_OUTPUT_VALUE_ADDRESS                                    ((uint8_t) 0x0B)
    #define GPO_OUTPUT_VALUE_DEFAULT                                    ((uint8_t) 0x00)

    /* GPO_OUTPUT_VALUE field mask & values */
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_MASK                      ((uint8_t) 0xFF)
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH7_LOW                   ((uint8_t) 0x00 << 7) // DEFAULT
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH7_HIGH                  ((uint8_t) 0x01 << 7)
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH6_LOW                   ((uint8_t) 0x00 << 6) // DEFAULT
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH6_HIGH                  ((uint8_t) 0x01 << 6)
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH5_LOW                   ((uint8_t) 0x00 << 5) // DEFAULT
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH5_HIGH                  ((uint8_t) 0x01 << 5)
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH4_LOW                   ((uint8_t) 0x00 << 4) // DEFAULT
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH4_HIGH                  ((uint8_t) 0x01 << 4)
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH3_LOW                   ((uint8_t) 0x00 << 3) // DEFAULT
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH3_HIGH                  ((uint8_t) 0x01 << 3)
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH2_LOW                   ((uint8_t) 0x00 << 2) // DEFAULT
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH2_HIGH                  ((uint8_t) 0x01 << 2)
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH1_LOW                   ((uint8_t) 0x00 << 1) // DEFAULT
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH1_HIGH                  ((uint8_t) 0x01 << 1)
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH0_LOW                   ((uint8_t) 0x00 << 0) // DEFAULT
    #define GPO_OUTPUT_VALUE_GPO_OUTPUT_VALUE_CH0_HIGH                  ((uint8_t) 0x01 << 0)


/* Register 0x0D (GPI_VALUE) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                            GPI_VALUE[7:0]                                            |
 * --------------------------------------------------------------------------------------------------------
 */

    /* GPI_VALUE register address & default value */
    #define GPI_VALUE_ADDRESS                                           ((uint8_t) 0x0D)
    #define GPI_VALUE_DEFAULT                                           ((uint8_t) 0x00)

    /* GPI_OUTPUT_VALUE field mask & values */
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_MASK                      ((uint8_t) 0xFF)
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH7_LOW                   ((uint8_t) 0x00 << 7) // DEFAULT
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH7_HIGH                  ((uint8_t) 0x01 << 7)
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH6_LOW                   ((uint8_t) 0x00 << 6) // DEFAULT
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH6_HIGH                  ((uint8_t) 0x01 << 6)
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH5_LOW                   ((uint8_t) 0x00 << 5) // DEFAULT
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH5_HIGH                  ((uint8_t) 0x01 << 5)
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH4_LOW                   ((uint8_t) 0x00 << 4) // DEFAULT
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH4_HIGH                  ((uint8_t) 0x01 << 4)
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH3_LOW                   ((uint8_t) 0x00 << 3) // DEFAULT
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH3_HIGH                  ((uint8_t) 0x01 << 3)
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH2_LOW                   ((uint8_t) 0x00 << 2) // DEFAULT
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH2_HIGH                  ((uint8_t) 0x01 << 2)
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH1_LOW                   ((uint8_t) 0x00 << 1) // DEFAULT
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH1_HIGH                  ((uint8_t) 0x01 << 1)
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH0_LOW                   ((uint8_t) 0x00 << 0) // DEFAULT
    #define GPI_OUTPUT_VALUE_GPI_OUTPUT_VALUE_CH0_HIGH                  ((uint8_t) 0x01 << 0)


/* Register 0x10 (SEQUENCE_CFG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |        RESERVED        |        SEQ_START        |        RESERVED         |      SEQ_MODE[1:0]      |
 * --------------------------------------------------------------------------------------------------------
 */

    /* SEQUENCE_CFG register address & default value */
    #define SEQUENCE_CFG_ADDRESS                                        ((uint8_t) 0x10)
    #define SEQUENCE_CFG_DEFAULT                                        ((uint8_t) 0x00)

    /* RESERVED field mask */
    #define SEQUENCE_CFG_RESERVED_MASK                                  ((uint8_t) 0xEC)

    /* SEQ_START field mask & values */
    #define SEQUENCE_CFG_SEQ_START_MASK                                 ((uint8_t) 0x10)
    #define SEQUENCE_CFG_SEQ_START_DISABLED                             ((uint8_t) 0x00 << 4) // DEFAULT
    #define SEQUENCE_CFG_SEQ_START_ENABLED                              ((uint8_t) 0x01 << 4)

    /* SEQ_MODE field mask & values */
    #define SEQUENCE_CFG_SEQ_MODE_MASK                                  ((uint8_t) 0x03)
    #define SEQUENCE_CFG_SEQ_MODE_MANUAL                                ((uint8_t) 0x00 << 0) // DEFAULT
    #define SEQUENCE_CFG_SEQ_MODE_AUTO_SEQ                              ((uint8_t) 0x01 << 0)
    #define SEQUENCE_CFG_SEQ_MODE_ON_THE_FLY                            ((uint8_t) 0x02 << 0)
    #define SEQUENCE_CFG_SEQ_MODE_RESERVED                              ((uint8_t) 0x03 << 0)


/* Register 0x11 (CHANNEL_SEL) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                     RESERVED                     |                   MANUAL_CHID[3:0]                |
 * --------------------------------------------------------------------------------------------------------
 */
    /* CHANNEL_SEL register address & default value */
    #define CHANNEL_SEL_ADDRESS                                         ((uint8_t) 0x11)
    #define CHANNEL_SEL_DEFAULT                                         ((uint8_t) 0x00)

    /* RESERVED field mask */
    #define CHANNEL_SEL_RESERVED_MASK                                   ((uint8_t) 0xF0)

    /* MANUAL_CHID field mask & values */
    #define CHANNEL_SEL_MANUAL_CHID_MASK                                ((uint8_t) 0x0F)
    #define CHANNEL_SEL_MANUAL_CHID_0                                   ((uint8_t) 0x00) // DEFAULT
    #define CHANNEL_SEL_MANUAL_CHID_1                                   ((uint8_t) 0x01)
    #define CHANNEL_SEL_MANUAL_CHID_2                                   ((uint8_t) 0x02)
    #define CHANNEL_SEL_MANUAL_CHID_3                                   ((uint8_t) 0x03)
    #define CHANNEL_SEL_MANUAL_CHID_4                                   ((uint8_t) 0x04)
    #define CHANNEL_SEL_MANUAL_CHID_5                                   ((uint8_t) 0x05)
    #define CHANNEL_SEL_MANUAL_CHID_6                                   ((uint8_t) 0x06)
    #define CHANNEL_SEL_MANUAL_CHID_7                                   ((uint8_t) 0x07)


/* Register 0x12 (AUTO_SEQ_CHSEL) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                         AUTO_SEQ_CHSEL[7:0]                                          |
 * --------------------------------------------------------------------------------------------------------
 */

    /* AUTO_SEQ_CHSEL register address & default value */
    #define AUTO_SEQ_CHSEL_ADDRESS                                      ((uint8_t) 0x12)
    #define AUTO_SEQ_CHSEL_DEFAULT                                      ((uint8_t) 0x00)

    /* AUTO_SEQ_CHSEL field mask & values */
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_MASK                          ((uint8_t) 0xFF)
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH7_DISABLED                  ((uint8_t) 0x00 << 7) // DEFAULT
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH7_ENABLED                   ((uint8_t) 0x01 << 7)
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH6_DISABLED                  ((uint8_t) 0x00 << 6) // DEFAULT
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH6_ENABLED                   ((uint8_t) 0x01 << 6)
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH5_DISABLED                  ((uint8_t) 0x00 << 5) // DEFAULT
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH5_ENABLED                   ((uint8_t) 0x01 << 5)
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH4_DISABLED                  ((uint8_t) 0x00 << 4) // DEFAULT
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH4_ENABLED                   ((uint8_t) 0x01 << 4)
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH3_DISABLED                  ((uint8_t) 0x00 << 3) // DEFAULT
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH3_ENABLED                   ((uint8_t) 0x01 << 3)
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH2_DISABLED                  ((uint8_t) 0x00 << 2) // DEFAULT
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH2_ENABLED                   ((uint8_t) 0x01 << 2)
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH1_DISABLED                  ((uint8_t) 0x00 << 1) // DEFAULT
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH1_ENABLED                   ((uint8_t) 0x01 << 1)
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH0_DISABLED                  ((uint8_t) 0x00 << 0) // DEFAULT
    #define AUTO_SEQ_CHSEL_AUTO_SEQ_CHSEL_CH0_ENABLED                   ((uint8_t) 0x01 << 0)

/* Register 0x14 (ALERT_CH_SEL) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                          ALERT_CH_SEL[7:0]                                           |
 * --------------------------------------------------------------------------------------------------------
 */

    /* ALERT_CH_SEL register address & default value */
    #define ALERT_CH_SEL_ADDRESS                                        ((uint8_t) 0x14)
    #define ALERT_CH_SEL_DEFAULT                                        ((uint8_t) 0x00)

    /* ALERT_CH_SEL field mask & values */
    #define ALERT_CH_SEL_ALERT_CH_SEL_MASK                              ((uint8_t) 0xFF)
    #define ALERT_CH_SEL_ALERT_CH_SEL_CH7_DISABLED                      ((uint8_t) 0x00 << 7) // DEFAULT
    #define ALERT_CH_SEL_ALERT_CH_SEL_CH7_ENABLED                       ((uint8_t) 0x01 << 7)
    #define ALERT_CH_SEL_ALERT_CH_SEL_CH6_DISABLED                      ((uint8_t) 0x00 << 6) // DEFAULT
    #define ALERT_CH_SEL_ALERT_CH_SEL_CH6_ENABLED                       ((uint8_t) 0x01 << 6)
    #define ALERT_CH_SEL_ALERT_CH_SEL_CH5_DISABLED                      ((uint8_t) 0x00 << 5) // DEFAULT
    #define ALERT_CH_SEL_ALERT_CH_SEL_CH5_ENABLED                       ((uint8_t) 0x01 << 5)
    #define ALERT_CH_SEL_ALERT_CH_SEL_CH4_DISABLED                      ((uint8_t) 0x00 << 4) // DEFAULT
    #define ALERT_CH_SEL_ALERT_CH_SEL_CH4_ENABLED                       ((uint8_t) 0x01 << 4)
    #define ALERT_CH_SEL_ALERT_CH_SEL_CH3_DISABLED                      ((uint8_t) 0x00 << 3) // DEFAULT
    #define ALERT_CH_SEL_ALERT_CH_SEL_CH3_ENABLED                       ((uint8_t) 0x01 << 3)
    #define ALERT_CH_SEL_ALERT_CH_SEL_CH2_DISABLED                      ((uint8_t) 0x00 << 2) // DEFAULT
    #define ALERT_CH_SEL_ALERT_CH_SEL_CH2_ENABLED                       ((uint8_t) 0x01 << 2)
    #define ALERT_CH_SEL_ALERT_CH_SEL_CH1_DISABLED                      ((uint8_t) 0x00 << 1) // DEFAULT
    #define ALERT_CH_SEL_ALERT_CH_SEL_CH1_ENABLED                       ((uint8_t) 0x01 << 1)
    #define ALERT_CH_SEL_ALERT_CH_SEL_CH0_DISABLED                      ((uint8_t) 0x00 << 0) // DEFAULT
    #define ALERT_CH_SEL_ALERT_CH_SEL_CH0_ENABLED                       ((uint8_t) 0x01 << 0)

/* Register 0x16 (ALERT_MAP) definition
 * ---------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0    |
 * --------------------------------------------------------------------------------------------------------
 * |                                         RESERVED                                        | ALERT_CRCIN |
 * ---------------------------------------------------------------------------------------------------------
 */
    /* ALERT_MAP register address & default value */
    #define ALERT_MAP_ADDRESS                                           ((uint8_t) 0x16)
    #define ALERT_MAP_DEFAULT                                           ((uint8_t) 0x00)

    /* RESERVED field mask */
    #define ALERT_MAP_RESERVED_MASK                                     ((uint8_t) 0xFE)

    /* ALERT_CRCIN field mask & values */
    #define ALERT_MAP_ALERT_CRCIN_MASK                                  ((uint8_t) 0x01)
    #define ALERT_MAP_ALERT_CRCIN_DISABLED                              ((uint8_t) 0x00) // DEFAULT
    #define ALERT_MAP_ALERT_CRCIN_ENABLED                               ((uint8_t) 0x01)

/* Register 0x17 (ALERT_PIN_CFG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                ALERT_PIN[3:0]                    |        RESERVED         |     ALERT_LOGIC[1:0]    |
 * --------------------------------------------------------------------------------------------------------
 */
    /* ALERT_PIN_CFG register address & default value */
    #define ALERT_PIN_CFG_ADDRESS                                       ((uint8_t) 0x17)
    #define ALERT_PIN_CFG_DEFAULT                                       ((uint8_t) 0x00)

    /* ALERT_PIN field mask & values */
    #define ALERT_PIN_CFG_ALERT_PIN_MASK                                ((uint8_t) 0xF0)
    #define ALERT_PIN_CFG_ALERT_PIN_POS                                 ((uint8_t) 0x04)
    #define ALERT_PIN_CFG_ALERT_PIN_CH_0                                ((uint8_t) 0x00 << 4) // DEFAULT
    #define ALERT_PIN_CFG_ALERT_PIN_CH_1                                ((uint8_t) 0x01 << 4)
    #define ALERT_PIN_CFG_ALERT_PIN_CH_2                                ((uint8_t) 0x02 << 4)
    #define ALERT_PIN_CFG_ALERT_PIN_CH_3                                ((uint8_t) 0x03 << 4)
    #define ALERT_PIN_CFG_ALERT_PIN_CH_4                                ((uint8_t) 0x04 << 4)
    #define ALERT_PIN_CFG_ALERT_PIN_CH_5                                ((uint8_t) 0x05 << 4)
    #define ALERT_PIN_CFG_ALERT_PIN_CH_6                                ((uint8_t) 0x06 << 4)
    #define ALERT_PIN_CFG_ALERT_PIN_CH_7                                ((uint8_t) 0x07 << 4)

    /* RESERVED field mask */
    #define ALERT_PIN_CFG_RESERVED_MASK                                 ((uint8_t) 0x0C)

    /* ALERT_LOGIC field mask & values */
    #define ALERT_PIN_CFG_ALERT_LOGIC_MASK                              ((uint8_t) 0x03)
    #define ALERT_PIN_CFG_ALERT_LOGIC_ACTIVE_LOW                        ((uint8_t) 0x00) // DEFAULT
    #define ALERT_PIN_CFG_ALERT_LOGIC_ACTIVE_HIGH                       ((uint8_t) 0x01)
    #define ALERT_PIN_CFG_ALERT_LOGIC_PULSED_LOW                        ((uint8_t) 0x02)
    #define ALERT_PIN_CFG_ALERT_LOGIC_PULSED_HIGH                       ((uint8_t) 0x03)

/* Register 0x18 (EVENT_FLAG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                           EVENT_FLAG[7:0]                                            |
 * --------------------------------------------------------------------------------------------------------
 */

    /* EVENT_FLAG register address & default value */
    #define EVENT_FLAG_ADDRESS                                          ((uint8_t) 0x18)
    #define EVENT_FLAG_DEFAULT                                          ((uint8_t) 0x00)

    /* EVENT_FLAG field mask & values */
    #define EVENT_FLAG_MASK                                             ((uint8_t) 0xFF)
    #define EVENT_FLAG_CH7_NO_EVENT                                     ((uint8_t) 0x00 << 7) // DEFAULT
    #define EVENT_FLAG_CH7_EVENT                                        ((uint8_t) 0x01 << 7)
    #define EVENT_FLAG_CH6_NO_EVENT                                     ((uint8_t) 0x00 << 6) // DEFAULT
    #define EVENT_FLAG_CH6_EVENT                                        ((uint8_t) 0x01 << 6)
    #define EVENT_FLAG_CH5_NO_EVENT                                     ((uint8_t) 0x00 << 5) // DEFAULT
    #define EVENT_FLAG_CH5_EVENT                                        ((uint8_t) 0x01 << 5)
    #define EVENT_FLAG_CH4_NO_EVENT                                     ((uint8_t) 0x00 << 4) // DEFAULT
    #define EVENT_FLAG_CH4_EVENT                                        ((uint8_t) 0x01 << 4)
    #define EVENT_FLAG_CH3_NO_EVENT                                     ((uint8_t) 0x00 << 3) // DEFAULT
    #define EVENT_FLAG_CH3_EVENT                                        ((uint8_t) 0x01 << 3)
    #define EVENT_FLAG_CH2_NO_EVENT                                     ((uint8_t) 0x00 << 2) // DEFAULT
    #define EVENT_FLAG_CH2_EVENT                                        ((uint8_t) 0x01 << 2)
    #define EVENT_FLAG_CH1_NO_EVENT                                     ((uint8_t) 0x00 << 1) // DEFAULT
    #define EVENT_FLAG_CH1_EVENT                                        ((uint8_t) 0x01 << 1)
    #define EVENT_FLAG_CH0_NO_EVENT                                     ((uint8_t) 0x00 << 0) // DEFAULT
    #define EVENT_FLAG_CH0_EVENT                                        ((uint8_t) 0x01 << 0)

/* Register 0x1A (EVENT_HIGH_FLAG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                        EVENT_HIGH_FLAG[7:0]                                          |
 * --------------------------------------------------------------------------------------------------------
 */

    /* EVENT_HIGH_FLAG register address & default value */
    #define EVENT_HIGH_FLAG_ADDRESS                                     ((uint8_t) 0x1A)
    #define EVENT_HIGH_FLAG_DEFAULT                                     ((uint8_t) 0x00)

    /* EVENT_HIGH_FLAG field mask & values */
    #define EVENT_HIGH_FLAG_MASK                                        ((uint8_t) 0xFF)
    #define EVENT_HIGH_FLAG_CH7_NO_EVENT                                ((uint8_t) 0x00 << 7) // DEFAULT
    #define EVENT_HIGH_FLAG_CH7_EVENT                                   ((uint8_t) 0x01 << 7)
    #define EVENT_HIGH_FLAG_CH6_NO_EVENT                                ((uint8_t) 0x00 << 6) // DEFAULT
    #define EVENT_HIGH_FLAG_CH6_EVENT                                   ((uint8_t) 0x01 << 6)
    #define EVENT_HIGH_FLAG_CH5_NO_EVENT                                ((uint8_t) 0x00 << 5) // DEFAULT
    #define EVENT_HIGH_FLAG_CH5_EVENT                                   ((uint8_t) 0x01 << 5)
    #define EVENT_HIGH_FLAG_CH4_NO_EVENT                                ((uint8_t) 0x00 << 4) // DEFAULT
    #define EVENT_HIGH_FLAG_CH4_EVENT                                   ((uint8_t) 0x01 << 4)
    #define EVENT_HIGH_FLAG_CH3_NO_EVENT                                ((uint8_t) 0x00 << 3) // DEFAULT
    #define EVENT_HIGH_FLAG_CH3_EVENT                                   ((uint8_t) 0x01 << 3)
    #define EVENT_HIGH_FLAG_CH2_NO_EVENT                                ((uint8_t) 0x00 << 2) // DEFAULT
    #define EVENT_HIGH_FLAG_CH2_EVENT                                   ((uint8_t) 0x01 << 2)
    #define EVENT_HIGH_FLAG_CH1_NO_EVENT                                ((uint8_t) 0x00 << 1) // DEFAULT
    #define EVENT_HIGH_FLAG_CH1_EVENT                                   ((uint8_t) 0x01 << 1)
    #define EVENT_HIGH_FLAG_CH0_NO_EVENT                                ((uint8_t) 0x00 << 0) // DEFAULT
    #define EVENT_HIGH_FLAG_CH0_EVENT                                   ((uint8_t) 0x01 << 0)

/* Register 0x1C (EVENT_LOW_FLAG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                        EVENT_LOW_FLAG[7:0]                                           |
 * --------------------------------------------------------------------------------------------------------
 */

    /* EVENT_LOW_FLAG register address & default value */
    #define EVENT_LOW_FLAG_ADDRESS                                      ((uint8_t) 0x1C)
    #define EVENT_LOW_FLAG_DEFAULT                                      ((uint8_t) 0x00)

    /* EVENT_LOW_FLAG field mask & values */
    #define EVENT_LOW_FLAG_MASK                                         ((uint8_t) 0xFF)
    #define EVENT_LOW_FLAG_CH7_NO_EVENT                                 ((uint8_t) 0x00 << 7) // DEFAULT
    #define EVENT_LOW_FLAG_CH7_EVENT                                    ((uint8_t) 0x01 << 7)
    #define EVENT_LOW_FLAG_CH6_NO_EVENT                                 ((uint8_t) 0x00 << 6) // DEFAULT
    #define EVENT_LOW_FLAG_CH6_EVENT                                    ((uint8_t) 0x01 << 6)
    #define EVENT_LOW_FLAG_CH5_NO_EVENT                                 ((uint8_t) 0x00 << 5) // DEFAULT
    #define EVENT_LOW_FLAG_CH5_EVENT                                    ((uint8_t) 0x01 << 5)
    #define EVENT_LOW_FLAG_CH4_NO_EVENT                                 ((uint8_t) 0x00 << 4) // DEFAULT
    #define EVENT_LOW_FLAG_CH4_EVENT                                    ((uint8_t) 0x01 << 4)
    #define EVENT_LOW_FLAG_CH3_NO_EVENT                                 ((uint8_t) 0x00 << 3) // DEFAULT
    #define EVENT_LOW_FLAG_CH3_EVENT                                    ((uint8_t) 0x01 << 3)
    #define EVENT_LOW_FLAG_CH2_NO_EVENT                                 ((uint8_t) 0x00 << 2) // DEFAULT
    #define EVENT_LOW_FLAG_CH2_EVENT                                    ((uint8_t) 0x01 << 2)
    #define EVENT_LOW_FLAG_CH1_NO_EVENT                                 ((uint8_t) 0x00 << 1) // DEFAULT
    #define EVENT_LOW_FLAG_CH1_EVENT                                    ((uint8_t) 0x01 << 1)
    #define EVENT_LOW_FLAG_CH0_NO_EVENT                                 ((uint8_t) 0x00 << 0) // DEFAULT
    #define EVENT_LOW_FLAG_CH0_EVENT                                    ((uint8_t) 0x01 << 0)

/* Register 0x1E (EVENT_RGN) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                            EVENT_RGN[7:0]                                            |
 * --------------------------------------------------------------------------------------------------------
 */

    /* EVENT_RGN register address & default value */
    #define EVENT_RGN_ADDRESS                                           ((uint8_t) 0x1E)
    #define EVENT_RGN_DEFAULT                                           ((uint8_t) 0x00)

    /* EVENT_RGN field mask & values */
    #define EVENT_RGN_MASK                                              ((uint8_t) 0xFF)
    #define EVENT_RGN_CH7_ALERT_OUT_OF_BAND                             ((uint8_t) 0x00 << 7) // DEFAULT
    #define EVENT_RGN_CH7_ALERT_IN_BAND                                 ((uint8_t) 0x01 << 7)
    #define EVENT_RGN_CH6_ALERT_OUT_OF_BAND                             ((uint8_t) 0x00 << 6) // DEFAULT
    #define EVENT_RGN_CH6_ALERT_IN_BAND                                 ((uint8_t) 0x01 << 6)
    #define EVENT_RGN_CH5_ALERT_OUT_OF_BAND                             ((uint8_t) 0x00 << 5) // DEFAULT
    #define EVENT_RGN_CH5_ALERT_IN_BAND                                 ((uint8_t) 0x01 << 5)
    #define EVENT_RGN_CH4_ALERT_OUT_OF_BAND                             ((uint8_t) 0x00 << 4) // DEFAULT
    #define EVENT_RGN_CH4_ALERT_IN_BAND                                 ((uint8_t) 0x01 << 4)
    #define EVENT_RGN_CH3_ALERT_OUT_OF_BAND                             ((uint8_t) 0x00 << 3) // DEFAULT
    #define EVENT_RGN_CH3_ALERT_IN_BAND                                 ((uint8_t) 0x01 << 3)
    #define EVENT_RGN_CH2_ALERT_OUT_OF_BAND                             ((uint8_t) 0x00 << 2) // DEFAULT
    #define EVENT_RGN_CH2_ALERT_IN_BAND                                 ((uint8_t) 0x01 << 2)
    #define EVENT_RGN_CH1_ALERT_OUT_OF_BAND                             ((uint8_t) 0x00 << 1) // DEFAULT
    #define EVENT_RGN_CH1_ALERT_IN_BAND                                 ((uint8_t) 0x01 << 1)
    #define EVENT_RGN_CH0_ALERT_OUT_OF_BAND                             ((uint8_t) 0x00 << 0) // DEFAULT
    #define EVENT_RGN_CH0_ALERT_IN_BAND                                 ((uint8_t) 0x01 << 0)

/* Registers 0x20, 0x24, 0x28, 0x2C, 0x30, 0x34, 0x38, 0x3C (HYSTERESIS_CHx) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |          HIGH_THRESHOLD_CHx_LSB[3:0]             |                HYSTERESIS_CHx[3:0]                |
 * --------------------------------------------------------------------------------------------------------
 */
    /* HYSTERESIS_CHx register address */
    #define HYSTERESIS_CH0_ADDRESS                                      ((uint8_t) 0x20)
    #define HYSTERESIS_CH1_ADDRESS                                      ((uint8_t) 0x24)
    #define HYSTERESIS_CH2_ADDRESS                                      ((uint8_t) 0x28)
    #define HYSTERESIS_CH3_ADDRESS                                      ((uint8_t) 0x2C)
    #define HYSTERESIS_CH4_ADDRESS                                      ((uint8_t) 0x30)
    #define HYSTERESIS_CH5_ADDRESS                                      ((uint8_t) 0x34)
    #define HYSTERESIS_CH6_ADDRESS                                      ((uint8_t) 0x38)
    #define HYSTERESIS_CH7_ADDRESS                                      ((uint8_t) 0x3C)

    /* HYSTERESIS_CHx registers default value */
    #define HYSTERESIS_CHx_DEFAULT                                      ((uint8_t) 0xF0)

    /* HIGH_THRESHOLD_CHx_LSB field mask  */
    #define HIGH_THRESHOLD_CHx_LSB_MASK                                 ((uint8_t) 0xF0)

    /* HYSTERESIS_CHx field mask */
    #define HYSTERESIS_CHx_MASK                                         ((uint8_t) 0x0F)

/* Registers 0x21, 0x25, 0x29, 0x2D, 0x31, 0x35, 0x39, 0x3D (HIGH_TH_CHx) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                    HIGH_THRESHOLD_CHx_MSB[7:0]                                       |
 * --------------------------------------------------------------------------------------------------------
 */
    /* HIGH_TH_CHx register address */
    #define HIGH_TH_CH0_ADDRESS                                         ((uint8_t) 0x21)
    #define HIGH_TH_CH1_ADDRESS                                         ((uint8_t) 0x25)
    #define HIGH_TH_CH2_ADDRESS                                         ((uint8_t) 0x29)
    #define HIGH_TH_CH3_ADDRESS                                         ((uint8_t) 0x2D)
    #define HIGH_TH_CH4_ADDRESS                                         ((uint8_t) 0x31)
    #define HIGH_TH_CH5_ADDRESS                                         ((uint8_t) 0x35)
    #define HIGH_TH_CH6_ADDRESS                                         ((uint8_t) 0x39)
    #define HIGH_TH_CH7_ADDRESS                                         ((uint8_t) 0x3D)

    /* HIGH_TH_CHx registers default value */
    #define HIGH_TH_CHx_DEFAULT                                         ((uint8_t) 0xFF)

    /* HIGH_THRESHOLD_CHx_MSB field mask */
    #define HIGH_THRESHOLD_CHx_MSB_MASK                                 ((uint8_t) 0xFF)

/* Registers 0x22, 0x26, 0x2A, 0x2E, 0x32, 0x36, 0x3A, 0x3E (EVENT_COUNT_CHx) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |           LOW_THRESHOLD_CHx_LSB[3:0]             |               EVENT_COUNT_CHx[3:0]                |
 * --------------------------------------------------------------------------------------------------------
 */
    /* EVENT_COUNT_CHx register address */
    #define EVENT_COUNT_CH0_ADDRESS                                     ((uint8_t) 0x22)
    #define EVENT_COUNT_CH1_ADDRESS                                     ((uint8_t) 0x26)
    #define EVENT_COUNT_CH2_ADDRESS                                     ((uint8_t) 0x2A)
    #define EVENT_COUNT_CH3_ADDRESS                                     ((uint8_t) 0x2E)
    #define EVENT_COUNT_CH4_ADDRESS                                     ((uint8_t) 0x32)
    #define EVENT_COUNT_CH5_ADDRESS                                     ((uint8_t) 0x36)
    #define EVENT_COUNT_CH6_ADDRESS                                     ((uint8_t) 0x3A)
    #define EVENT_COUNT_CH7_ADDRESS                                     ((uint8_t) 0x3E)

    /* EVENT_COUNT_CHx registers default value */
    #define EVENT_COUNT_CHx_DEFAULT                                     ((uint8_t) 0x00)

    /* LOW_THRESHOLD_CH_LSB field mask */
    #define LOW_THRESHOLD_CHx_LSB_MASK                                  ((uint8_t) 0xF0)

    /* EVENT_COUNT_CH field mask */
    #define EVENT_COUNT_CHx_MASK                                        ((uint8_t) 0x0F)

/* Registers 0x23, 0x27, 0x2B, 0x2F, 0x33, 0x37, 0x3B, 0x3F (LOW_TH_CHx) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                     LOW_THRESHOLD_CHx_MSB[7:0]                                       |
 * --------------------------------------------------------------------------------------------------------
 */
    /* LOW_TH_CHx register address */
    #define LOW_TH_CH0_ADDRESS                                          ((uint8_t) 0x23)
    #define LOW_TH_CH1_ADDRESS                                          ((uint8_t) 0x27)
    #define LOW_TH_CH2_ADDRESS                                          ((uint8_t) 0x2B)
    #define LOW_TH_CH3_ADDRESS                                          ((uint8_t) 0x2F)
    #define LOW_TH_CH4_ADDRESS                                          ((uint8_t) 0x33)
    #define LOW_TH_CH5_ADDRESS                                          ((uint8_t) 0x37)
    #define LOW_TH_CH6_ADDRESS                                          ((uint8_t) 0x3B)
    #define LOW_TH_CH7_ADDRESS                                          ((uint8_t) 0x3F)

    /* LOW_TH_CHx registers default value */
    #define LOW_TH_CHx_DEFAULT                                          ((uint8_t) 0x00)

    /* LOW_THRESHOLD_CH_MSB field mask */
    #define LOW_THRESHOLD_CHx_MSB_MASK                                  ((uint8_t) 0xFF)

/* Registers 0x60, 0x62, 0x64, 0x66, 0x68, 0x6A, 0x6C, 0x6E (MAX_CHx_LSB) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                       MAX_VALUE_CHx_LSB[7:0]                                         |
 * --------------------------------------------------------------------------------------------------------
 */
    /* MAX_CHx_LSB register address */
    #define MAX_CH0_LSB_ADDRESS                                         ((uint8_t) 0x60)
    #define MAX_CH1_LSB_ADDRESS                                         ((uint8_t) 0x62)
    #define MAX_CH2_LSB_ADDRESS                                         ((uint8_t) 0x64)
    #define MAX_CH3_LSB_ADDRESS                                         ((uint8_t) 0x66)
    #define MAX_CH4_LSB_ADDRESS                                         ((uint8_t) 0x68)
    #define MAX_CH5_LSB_ADDRESS                                         ((uint8_t) 0x6A)
    #define MAX_CH6_LSB_ADDRESS                                         ((uint8_t) 0x6C)
    #define MAX_CH7_LSB_ADDRESS                                         ((uint8_t) 0x6E)

    /* MAX_CHx_LSB registers default value */
    #define MAX_CHx_LSB_DEFAULT                                         ((uint8_t) 0x00)

    /* MAX_VALUE_CH_LSB field mask */
    #define MAX_VALUE_CHx_LSB_MASK                                      ((uint8_t) 0xFF)

/* Registers 0x61, 0x63, 0x65, 0x67, 0x69, 0x6B, 0x6D, 0x6F (MAX_CHx_MSB) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                       MAX_VALUE_CHx_MSB[7:0]                                         |
 * --------------------------------------------------------------------------------------------------------
 */
    /* MAX_CHx_MSB register address */
    #define MAX_CH0_MSB_ADDRESS                                         ((uint8_t) 0x61)
    #define MAX_CH1_MSB_ADDRESS                                         ((uint8_t) 0x63)
    #define MAX_CH2_MSB_ADDRESS                                         ((uint8_t) 0x65)
    #define MAX_CH3_MSB_ADDRESS                                         ((uint8_t) 0x67)
    #define MAX_CH4_MSB_ADDRESS                                         ((uint8_t) 0x69)
    #define MAX_CH5_MSB_ADDRESS                                         ((uint8_t) 0x6B)
    #define MAX_CH6_MSB_ADDRESS                                         ((uint8_t) 0x6D)
    #define MAX_CH7_MSB_ADDRESS                                         ((uint8_t) 0x6F)

    /* MAX_CHx_MSB registers default value */
    #define MAX_CHx_MSB_DEFAULT                                         ((uint8_t) 0x00)

    /* MAX_VALUE_CH_MSB field mask */
    #define MAX_VALUE_CHx_MSB_MASK                                      ((uint8_t) 0xFF)

/* Registers 0x80, 0x82, 0x84, 0x86, 0x88, 0x8A, 0x8C, 0x8E (MIN_CHx_LSB) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                       MIN_VALUE_CHx_LSB[7:0]                                         |
 * --------------------------------------------------------------------------------------------------------
 */
    /* MIN_CHx_LSB register address */
    #define MIN_CH0_LSB_ADDRESS                                         ((uint8_t) 0x80)
    #define MIN_CH1_LSB_ADDRESS                                         ((uint8_t) 0x82)
    #define MIN_CH2_LSB_ADDRESS                                         ((uint8_t) 0x84)
    #define MIN_CH3_LSB_ADDRESS                                         ((uint8_t) 0x86)
    #define MIN_CH4_LSB_ADDRESS                                         ((uint8_t) 0x88)
    #define MIN_CH5_LSB_ADDRESS                                         ((uint8_t) 0x8A)
    #define MIN_CH6_LSB_ADDRESS                                         ((uint8_t) 0x8C)
    #define MIN_CH7_LSB_ADDRESS                                         ((uint8_t) 0x8E)

    /* MIN_CHx_LSB registers default value */
    #define MIN_CHx_LSB_DEFAULT                                         ((uint8_t) 0xFF)

    /* MIN_VALUE_CH_LSB field mask */
    #define MIN_VALUE_CHx_LSB_MASK                                      ((uint8_t) 0xFF)

/* Registers 0x81, 0x83, 0x85, 0x87, 0x89, 0x8B, 0x8D, 0x8F (MIN_CHx_MSB) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                       MIN_VALUE_CHx_MSB[7:0]                                         |
 * --------------------------------------------------------------------------------------------------------
 */
    /* MIN_CHx_MSB register address */
    #define MIN_CH0_MSB_ADDRESS                                         ((uint8_t) 0x81)
    #define MIN_CH1_MSB_ADDRESS                                         ((uint8_t) 0x83)
    #define MIN_CH2_MSB_ADDRESS                                         ((uint8_t) 0x85)
    #define MIN_CH3_MSB_ADDRESS                                         ((uint8_t) 0x87)
    #define MIN_CH4_MSB_ADDRESS                                         ((uint8_t) 0x89)
    #define MIN_CH5_MSB_ADDRESS                                         ((uint8_t) 0x8B)
    #define MIN_CH6_MSB_ADDRESS                                         ((uint8_t) 0x8D)
    #define MIN_CH7_MSB_ADDRESS                                         ((uint8_t) 0x8F)

    /* MIN_CHx_MSB registers default value */
    #define MIN_CHx_MSB_DEFAULT                                         ((uint8_t) 0xFF)

    /* MIN_VALUE_CH_MSB field mask */
    #define MIN_VALUE_CHx_MSB_MASK                                      ((uint8_t) 0xFF)

/* Registers 0xA0, 0xA2, 0xA4, 0xA6, 0xA8, 0xAA, 0xAC, 0xAE (RECENT_CHx_LSB) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                       LAST_VALUE_CHx_LSB[7:0]                                        |
 * --------------------------------------------------------------------------------------------------------
 */
    /* RECENT_CHx_LSB register address */
    #define RECENT_CH0_LSB_ADDRESS                                      ((uint8_t) 0xA0)
    #define RECENT_CH1_LSB_ADDRESS                                      ((uint8_t) 0xA2)
    #define RECENT_CH2_LSB_ADDRESS                                      ((uint8_t) 0xA4)
    #define RECENT_CH3_LSB_ADDRESS                                      ((uint8_t) 0xA6)
    #define RECENT_CH4_LSB_ADDRESS                                      ((uint8_t) 0xA8)
    #define RECENT_CH5_LSB_ADDRESS                                      ((uint8_t) 0xAA)
    #define RECENT_CH6_LSB_ADDRESS                                      ((uint8_t) 0xAC)
    #define RECENT_CH7_LSB_ADDRESS                                      ((uint8_t) 0xAE)

    /* RECENT_CHx_LSB registers default value */
    #define RECENT_CHx_LSB_DEFAULT                                      ((uint8_t) 0x00)

    /* LAST_VALUE_CH_LSB field mask */
    #define LAST_VALUE_CHx_LSB_MASK                                     ((uint8_t) 0xFF)

/* Registers 0xA1, 0xA3, 0xA5, 0xA6, 0xA9, 0xAB, 0xAD, 0xAF (RECENT_CHx_MSB) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                       LAST_VALUE_CHx_MSB[7:0]                                        |
 * --------------------------------------------------------------------------------------------------------
 */
    /* RECENT_CHx_MSB register address */
    #define RECENT_CH0_MSB_ADDRESS                                      ((uint8_t) 0xA1)
    #define RECENT_CH1_MSB_ADDRESS                                      ((uint8_t) 0xA3)
    #define RECENT_CH2_MSB_ADDRESS                                      ((uint8_t) 0xA5)
    #define RECENT_CH3_MSB_ADDRESS                                      ((uint8_t) 0xA7)
    #define RECENT_CH4_MSB_ADDRESS                                      ((uint8_t) 0xA9)
    #define RECENT_CH5_MSB_ADDRESS                                      ((uint8_t) 0xAB)
    #define RECENT_CH6_MSB_ADDRESS                                      ((uint8_t) 0xAD)
    #define RECENT_CH7_MSB_ADDRESS                                      ((uint8_t) 0xAF)

    /* RECENT_CHx_MSB registers default value */
    #define RECENT_CHx_MSB_DEFAULT                                      ((uint8_t) 0x00)

    /* LAST_VALUE_CH_MSB field mask */
    #define LAST_VALUE_CHx_MSB_MASK                                     ((uint8_t) 0xFF)

/* Register (GPOx_TRIG_EVENT_SEL) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                      GPOx_TRIG_EVENT_SEL[7:0]                                        |
 * --------------------------------------------------------------------------------------------------------
 */
    /* GPOx_TRIG_EVENT_SEL register address */
    #define GPO0_TRIG_EVENT_SEL_ADDRESS                                 ((uint8_t) 0xC3)
    #define GPO1_TRIG_EVENT_SEL_ADDRESS                                 ((uint8_t) 0xC5)
    #define GPO2_TRIG_EVENT_SEL_ADDRESS                                 ((uint8_t) 0xC7)
    #define GPO3_TRIG_EVENT_SEL_ADDRESS                                 ((uint8_t) 0xC9)
    #define GPO4_TRIG_EVENT_SEL_ADDRESS                                 ((uint8_t) 0xCB)
    #define GPO5_TRIG_EVENT_SEL_ADDRESS                                 ((uint8_t) 0xCD)
    #define GPO6_TRIG_EVENT_SEL_ADDRESS                                 ((uint8_t) 0xCF)
    #define GPO7_TRIG_EVENT_SEL_ADDRESS                                 ((uint8_t) 0xD1)

    /* GPO_TRIG_EVENT_SEL register default value */
    #define GPOx_TRIG_EVENT_SEL_DEFAULT                                 ((uint8_t) 0x00)

    /* GPOx_TRIG_EVENT_SEL field mask & values */
    #define GPOx_TRIG_EVENT_SEL_MASK                                    ((uint8_t) 0xFF)
    #define GPOx_TRIG_EVENT_SEL_CH7_NO_TRIG                             ((uint8_t) 0x00 << 7) // DEFAULT
    #define GPOx_TRIG_EVENT_SEL_CH7_TRIG                                ((uint8_t) 0x01 << 7)
    #define GPOx_TRIG_EVENT_SEL_CH6_NO_TRIG                             ((uint8_t) 0x00 << 6) // DEFAULT
    #define GPOx_TRIG_EVENT_SEL_CH6_TRIG                                ((uint8_t) 0x01 << 6)
    #define GPOx_TRIG_EVENT_SEL_CH5_NO_TRIG                             ((uint8_t) 0x00 << 5) // DEFAULT
    #define GPOx_TRIG_EVENT_SEL_CH5_TRIG                                ((uint8_t) 0x01 << 5)
    #define GPOx_TRIG_EVENT_SEL_CH4_NO_TRIG                             ((uint8_t) 0x00 << 4) // DEFAULT
    #define GPOx_TRIG_EVENT_SEL_CH4_TRIG                                ((uint8_t) 0x01 << 4)
    #define GPOx_TRIG_EVENT_SEL_CH3_NO_TRIG                             ((uint8_t) 0x00 << 3) // DEFAULT
    #define GPOx_TRIG_EVENT_SEL_CH3_TRIG                                ((uint8_t) 0x01 << 3)
    #define GPOx_TRIG_EVENT_SEL_CH2_NO_TRIG                             ((uint8_t) 0x00 << 2) // DEFAULT
    #define GPOx_TRIG_EVENT_SEL_CH2_TRIG                                ((uint8_t) 0x01 << 2)
    #define GPOx_TRIG_EVENT_SEL_CH1_NO_TRIG                             ((uint8_t) 0x00 << 1) // DEFAULT
    #define GPOx_TRIG_EVENT_SEL_CH1_TRIG                                ((uint8_t) 0x01 << 1)
    #define GPOx_TRIG_EVENT_SEL_CH0_NO_TRIG                             ((uint8_t) 0x00 << 0) // DEFAULT
    #define GPOx_TRIG_EVENT_SEL_CH0_TRIG                                ((uint8_t) 0x01 << 0)

/* Register (GPO_TRIGGER_CFG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                     GPO_TRIGGER_UPDATE_EN[7:0]                                       |
 * --------------------------------------------------------------------------------------------------------
 */
    /* GPO_TRIGGER_CFG register address */
    #define GPO_TRIGGER_CFG_ADDRESS                                     ((uint8_t) 0xE9)

    /* GPO_TRIGGER_CFG register default value */
    #define GPO_TRIGGER_CFG_DEFAULT                                     ((uint8_t) 0x00)

    /* GPO_TRIGGER_UPDATE_EN field mask & values */
    #define GPO_TRIGGER_UPDATE_EN_MASK                                  ((uint8_t) 0xFF)
    #define GPO_TRIGGER_UPDATE_EN_CH7_ALERT_OUT_DISABLE                 ((uint8_t) 0x00 << 7) // DEFAULT
    #define GPO_TRIGGER_UPDATE_EN_CH7_ALERT_OUT_ENABLE                  ((uint8_t) 0x01 << 7)
    #define GPO_TRIGGER_UPDATE_EN_CH6_ALERT_OUT_DISABLE                 ((uint8_t) 0x00 << 6) // DEFAULT
    #define GPO_TRIGGER_UPDATE_EN_CH6_ALERT_OUT_ENABLE                  ((uint8_t) 0x01 << 6)
    #define GPO_TRIGGER_UPDATE_EN_CH5_ALERT_OUT_DISABLE                 ((uint8_t) 0x00 << 5) // DEFAULT
    #define GPO_TRIGGER_UPDATE_EN_CH5_ALERT_OUT_ENABLE                  ((uint8_t) 0x01 << 5)
    #define GPO_TRIGGER_UPDATE_EN_CH4_ALERT_OUT_DISABLE                 ((uint8_t) 0x00 << 4) // DEFAULT
    #define GPO_TRIGGER_UPDATE_EN_CH4_ALERT_OUT_ENABLE                  ((uint8_t) 0x01 << 4)
    #define GPO_TRIGGER_UPDATE_EN_CH3_ALERT_OUT_DISABLE                 ((uint8_t) 0x00 << 3) // DEFAULT
    #define GPO_TRIGGER_UPDATE_EN_CH3_ALERT_OUT_ENABLE                  ((uint8_t) 0x01 << 3)
    #define GPO_TRIGGER_UPDATE_EN_CH2_ALERT_OUT_DISABLE                 ((uint8_t) 0x00 << 2) // DEFAULT
    #define GPO_TRIGGER_UPDATE_EN_CH2_ALERT_OUT_ENABLE                  ((uint8_t) 0x01 << 2)
    #define GPO_TRIGGER_UPDATE_EN_CH1_ALERT_OUT_DISABLE                 ((uint8_t) 0x00 << 1) // DEFAULT
    #define GPO_TRIGGER_UPDATE_EN_CH1_ALERT_OUT_ENABLE                  ((uint8_t) 0x01 << 1)
    #define GPO_TRIGGER_UPDATE_EN_CH0_ALERT_OUT_DISABLE                 ((uint8_t) 0x00 << 0) // DEFAULT
    #define GPO_TRIGGER_UPDATE_EN_CH0_ALERT_OUT_ENABLE                  ((uint8_t) 0x01 << 0)

/* Register (GPO_VALUE_TRIG) definition
 * --------------------------------------------------------------------------------------------------------
 * |   Bit 7   |    Bit 6   |    Bit 5   |    Bit 4   |    Bit 3   |    Bit 2   |    Bit 1   |    Bit 0   |
 * --------------------------------------------------------------------------------------------------------
 * |                                     GPO_VALUE_ON_TRIGGER[[7:0]                                       |
 * --------------------------------------------------------------------------------------------------------
 */
    /* GPO_VALUE_TRIG register address */
    #define GPO_VALUE_TRIG_ADDRESS                                      ((uint8_t) 0xEB)

    /* GPO_VALUE_TRIG register default value */
    #define GPO_VALUE_TRIG_DEFAULT                                      ((uint8_t) 0x00)

    /* GPO_VALUE_ON_TRIGGER field mask & values */
    #define GPO_VALUE_ON_TRIGGER_MASK                                   ((uint8_t) 0xFF)
    #define GPO_VALUE_ON_TRIGGER_CH7_LOW                                ((uint8_t) 0x00 << 7) // DEFAULT
    #define GPO_VALUE_ON_TRIGGER_CH7_HIGH                               ((uint8_t) 0x01 << 7)
    #define GPO_VALUE_ON_TRIGGER_CH6_LOW                                ((uint8_t) 0x00 << 6) // DEFAULT
    #define GPO_VALUE_ON_TRIGGER_CH6_HIGH                               ((uint8_t) 0x01 << 6)
    #define GPO_VALUE_ON_TRIGGER_CH5_LOW                                ((uint8_t) 0x00 << 5) // DEFAULT
    #define GPO_VALUE_ON_TRIGGER_CH5_HIGH                               ((uint8_t) 0x01 << 5)
    #define GPO_VALUE_ON_TRIGGER_CH4_LOW                                ((uint8_t) 0x00 << 4) // DEFAULT
    #define GPO_VALUE_ON_TRIGGER_CH4_HIGH                               ((uint8_t) 0x01 << 4)
    #define GPO_VALUE_ON_TRIGGER_CH3_LOW                                ((uint8_t) 0x00 << 3) // DEFAULT
    #define GPO_VALUE_ON_TRIGGER_CH3_HIGH                               ((uint8_t) 0x01 << 3)
    #define GPO_VALUE_ON_TRIGGER_CH2_LOW                                ((uint8_t) 0x00 << 2) // DEFAULT
    #define GPO_VALUE_ON_TRIGGER_CH2_HIGH                               ((uint8_t) 0x01 << 2)
    #define GPO_VALUE_ON_TRIGGER_CH1_LOW                                ((uint8_t) 0x00 << 1) // DEFAULT
    #define GPO_VALUE_ON_TRIGGER_CH1_HIGH                               ((uint8_t) 0x01 << 1)
    #define GPO_VALUE_ON_TRIGGER_CH0_LOW                                ((uint8_t) 0x00 << 0) // DEFAULT
    #define GPO_VALUE_ON_TRIGGER_CH0_HIGH                               ((uint8_t) 0x01 << 0)


//****************************************************************************
//
// Register macros
//
//****************************************************************************

/* Maximum register address or address of the last register in the regmap */
#define MAX_REGISTER_ADDRESS    ((uint8_t) 0xEB)

/* Returns true if the CRC_EN enable bit is set */
#define SPI_CRC_ENABLED         ((bool) (getRegisterValue(GENERAL_CFG_ADDRESS) & GENERAL_CFG_CRC_EN_MASK))

/* Returns true if the CRCERR_IN error bit is set */
#define SPI_CRCERR_IN           ((bool) (getRegisterValue(SYSTEM_STATUS_ADDRESS) & SYSTEM_STATUS_CRCERR_IN_MASK))

/* Returns true if Auto-Sequence capture mode is selected */
#define AUTOSEQ_MODE_ENABLED    ((bool) ((getRegisterValue(SEQUENCE_CFG_ADDRESS) & SEQUENCE_CFG_SEQ_MODE_MASK) \
                                            == SEQUENCE_CFG_SEQ_MODE_AUTO_SEQ))

/* Returns true if channel sequencing in auto-sequence mode is enabled */
#define AUTOSEQ_CH_SEQ_ENABLED  ((bool) (getRegisterValue(SEQUENCE_CFG_ADDRESS) & SEQUENCE_CFG_SEQ_START_MASK))

/* Returns true if channel sequencing in auto-sequence mode is enabled */
#define AVERAGING_ENABLED       ((bool) (getRegisterValue(OSR_CFG_ADDRESS) & OSR_CFG_OSR_MASK))


//****************************************************************************
//
// Function prototypes
//
//****************************************************************************

void        initADS7038(void);
void        resetDevice();
void        startManualConversions(uint8_t channelID, uint32_t samplesPerSecond);
void        stopConversions(void);

int16_t     readData(uint8_t dataRx[]);
uint8_t     readSingleRegister(uint8_t address);
uint8_t     getRegisterValue(uint8_t address);

void        writeSingleRegister(uint8_t address, uint8_t data);
void        setRegisterBits(uint8_t address, uint8_t bitMask);
void        clearRegisterBits(uint8_t address, uint8_t bitMask);

/* Helper Functions */
uint8_t     calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint8_t initialValue);
void        setChannelAsAnalogInput(uint8_t channelID);


#endif /* ADS7038_H_ */
