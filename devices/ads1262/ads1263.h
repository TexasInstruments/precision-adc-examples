/**
 * \file ads1263.h
 *
 * \brief This header file contains all register map definitions for the ADS1262 and ADS1263.
 *
 * \note Macro naming conventions try to follow ADS1262 and ADS1263 data sheet naming definitions;
 *  however, future data sheet revisions may cause macro names to differ from the example code.
 *
 * \copyright Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef ADS1263_H_
#define ADS1263_H_

// C standard libraries
#include <assert.h>
#include <stdint.h>

// User libraries
#include "hal.h"

// Define options
#define ADS1263_ONLY_FEATURES		// Enables ADS1263-only register & command definitions



//*****************************************************************************
//
// Number of device registers
//
//*****************************************************************************

    #ifdef ADS1263_ONLY_FEATURES
        #define NUM_REGISTERS 					((uint8_t) 27)
    #else
        #define NUM_REGISTERS                   ((uint8_t) 21)
    #endif



//*****************************************************************************
//
// SPI Opcodes
//
//*****************************************************************************

    #define OPCODE_NOP                          ((uint8_t) 0x00)
    #define OPCODE_RESET                        ((uint8_t) 0x06)
    #define OPCODE_START1                       ((uint8_t) 0x08)
    #define OPCODE_STOP1                        ((uint8_t) 0x0B)
    #define OPCODE_RDATA1                       ((uint8_t) 0x12)
    #define OPCODE_SYOCAL1                      ((uint8_t) 0x16)
    #define OPCODE_SYGCAL1                      ((uint8_t) 0x17)
    #define OPCODE_SFOCAL1                      ((uint8_t) 0x19)
    #define OPCODE_RREG                         ((uint8_t) 0x20)
    #define OPCODE_WREG                         ((uint8_t) 0x40)

    /* Additional ADS1263 commands */
    #ifdef ADS1263_ONLY_FEATURES
        #define OPCODE_START2                   ((uint8_t) 0x0C)
        #define OPCODE_STOP2                    ((uint8_t) 0x0E)
        #define OPCODE_RDATA2                   ((uint8_t) 0x14)
        #define OPCODE_SYOCAL2                  ((uint8_t) 0x1B)
        #define OPCODE_SYGCAL2                  ((uint8_t) 0x1C)
        #define OPCODE_SFOCAL2                  ((uint8_t) 0x1E)
    #endif



//*****************************************************************************
//
// Status byte formatting
//
//*****************************************************************************

/* STATUS byte definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |   ADC2    |   ADC1    |  EXTCLK   |  REF_ALM  | PGAL_ALM  | PGAH_ALM  | PGAD_ALM  |   RESET   |
 * -------------------------------------------------------------------------------------------------
 */

    /* STATUS byte field masks */
	#define	STATUS_ADC1						    ((uint8_t) 0x40)			/* Indicates new ADC1 data */
	#define	STATUS_EXTCLK						((uint8_t) 0x20)			/* Indicates ADC clock source */
	#define	STATUS_REF_ALM						((uint8_t) 0x10)			/* Low Reference Alarm   - Only used with ADC1 */
	#define	STATUS_PGAL_ALM						((uint8_t) 0x08)			/* PGA Output Low Alarm  - Only used with ADC1 */
	#define	STATUS_PGAH_ALM						((uint8_t) 0x04)			/* PGA Output High Alarm - Only used with ADC1 */
	#define	STATUS_PGAD_ALM						((uint8_t) 0x02)			/* PGA Diff Output Alarm - Only used with ADC1 */
	#define	STATUS_RESET						((uint8_t) 0x01)			/* Indicates device reset (re-named to avoid conflict) */

    /* Additional ADS1263 status flag */
    #ifdef ADS1263_ONLY_FEATURES
        #define STATUS_ADC2                     ((uint8_t) 0x80)            /* Indicates new ADC2 data */
    #endif



//*****************************************************************************
//
// Register definitions
//
//*****************************************************************************

/* Register 0x00 (ID) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |         DEV_ID[2:0]         |                   REV_ID[4:0]                   |
 * ---------------------------------------------------------------------------------
 */

    /* ID register address */
    #define REG_ADDR_ID                         ((uint8_t) 0x00)

    /* ID register field masks */
    #define ID_DEV_MASK                         ((uint8_t) 0xE0)
    #define ID_REV_MASK                         ((uint8_t) 0x1F)

    /* Define DEV_ID (device) */
    #define ID_DEV_ADS1262                      ((uint8_t) 0x00)
    #define ID_DEV_ADS1263                      ((uint8_t) 0x20)

    /* Define REV_ID (revision) */
    /* Note: Revision ID can change without notification */
    #define ID_REV_A                            ((uint8_t) 0x00)
    #define ID_REV_B                            ((uint8_t) 0x01)
    #define ID_REV_C                            ((uint8_t) 0x02)



/* Register 0x01 (POWER) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |           RESERVED          |  RESET  |      RESERVED     |  VBIAS  |  INTREF |
 * ---------------------------------------------------------------------------------
 */

    /* POWER register address */
    #define REG_ADDR_POWER                      ((uint8_t) 0x01)

    /* POWER default (reset) value */
    #define POWER_DEFAULT                       ((uint8_t) 0x11)

    /* POWER register field masks */
    #define POWER_RESET_MASK                    ((uint8_t) 0x10)
    #define POWER_VBIAS_MASK                    ((uint8_t) 0x02)
    #define POWER_INTREF_MASK                   ((uint8_t) 0x01)



/* Register 0x02 (INTERFACE) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |               RESERVED                | TIMEOUT |  STATUS |     CRC[1:0]      |
 * ---------------------------------------------------------------------------------
 */

    /* INTERFACE register address */
    #define REG_ADDR_INTERFACE                  ((uint8_t) 0x02)

    /* INTERFACE default (reset) value */
    #define INTERFACE_DEFAULT                   ((uint8_t) 0x05)

    /* INTERFACE register field masks */
    #define INTERFACE_TIMEOUT_MASK              ((uint8_t) 0x08)
    #define INTERFACE_STATUS_MASK               ((uint8_t) 0x04)
    #define INTERFACE_CRC_MASK                  ((uint8_t) 0x03)

    /* CRC field values */
    #define INTERFACE_CRC_OFF                   ((uint8_t) 0x00)
    #define INTERFACE_CRC_CHKSUM                ((uint8_t) 0x01)
    #define INTERFACE_CRC_ON                    ((uint8_t) 0x02)



/* Register 0x03 (MODE0) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |  REFREV | RUNMODE |      CHOP[1:0]    |               DELAY[3:0]              |
 * ---------------------------------------------------------------------------------
 */

    /* MODE0 register address */
    #define REG_ADDR_MODE0                      ((uint8_t) 0x03)

    /* MODE0 default (reset) value */
    #define MODE0_DEFAULT                       ((uint8_t) 0x00)

    /* MODE0 register field masks */
    #define MODE0_REFREV_MASK                   ((uint8_t) 0x80)
    #define MODE0_RUNMODE_MASK                  ((uint8_t) 0x40)
    #define MODE0_CHOP_MASK                     ((uint8_t) 0x30)
    #define MODE0_DELAY_MASK                    ((uint8_t) 0x0F)

    /* CHOP field values */
    #define MODE0_CHOP_OFF                      ((uint8_t) 0x00)
    #define MODE0_CHOP_ON                       ((uint8_t) 0x10)
    #define MODE0_CHOP_IDAC                     ((uint8_t) 0x20)
    #define MODE0_CHOP_ON_IDAC                  ((uint8_t) 0x30)

    /* DELAY field values */
    #define MODE0_DELAY_0us                     ((uint8_t) 0x00)
    #define MODE0_DELAY_8_7us                   ((uint8_t) 0x01)
    #define MODE0_DELAY_17us                    ((uint8_t) 0x02)
    #define MODE0_DELAY_35us                    ((uint8_t) 0x03)
    #define MODE0_DELAY_69us                    ((uint8_t) 0x04)
    #define MODE0_DELAY_139us                   ((uint8_t) 0x05)
    #define MODE0_DELAY_278us                   ((uint8_t) 0x06)
    #define MODE0_DELAY_555us                   ((uint8_t) 0x07)
    #define MODE0_DELAY_1100us                  ((uint8_t) 0x08)
    #define MODE0_DELAY_2200us                  ((uint8_t) 0x09)
    #define MODE0_DELAY_4400us                  ((uint8_t) 0x0A)
    #define MODE0_DELAY_8800us                  ((uint8_t) 0x0B)



/* Register 0x04 (MODE1) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |         FILTER[2:0]         |  SBADC  |  SBPOL  |         SBMAG[3:0]          |
 * ---------------------------------------------------------------------------------
 */

    /* MODE1 register address */
    #define REG_ADDR_MODE1                      ((uint8_t) 0x04)

    /* MODE1 default (reset) value */
    #define MODE1_DEFAULT                       ((uint8_t) 0x80)

    /* MODE1 register field masks */
    #define MODE1_FILTER_MASK                   ((uint8_t) 0xE0)
    #define MODE1_SBADC_MASK                    ((uint8_t) 0x80)
    #define MODE1_SBPOL_MASK                    ((uint8_t) 0x08)
    #define MODE1_SBMAG_MASK                    ((uint8_t) 0x07)

    /* FILTER field values */
    #define MODE1_FILTER_SINC1                  ((uint8_t) 0x00)
    #define MODE1_FILTER_SINC2                  ((uint8_t) 0x20)
    #define MODE1_FILTER_SINC3                  ((uint8_t) 0x40)
    #define MODE1_FILTER_SINC4                  ((uint8_t) 0x60)
    #define MODE1_FILTER_FIR                    ((uint8_t) 0x80)

    /* SBMAG field values */
    #define MODE1_SBMAG_0uA                     ((uint8_t) 0x00)
    #define MODE1_SBMAG_0_5uA                   ((uint8_t) 0x01)
    #define MODE1_SBMAG_2uA                     ((uint8_t) 0x02)
    #define MODE1_SBMAG_10uA                    ((uint8_t) 0x03)
    #define MODE1_SBMAG_50uA                    ((uint8_t) 0x04)
    #define MODE1_SBMAG_200uA                   ((uint8_t) 0x05)
    #define MODE1_SBMAG_10MOhm                  ((uint8_t) 0x06)



/* Register 0x05 (MODE2) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * | BYPASS  |          GAIN[2:0]          |                DR[3:0]                |
 * ---------------------------------------------------------------------------------
 */

    /* MODE2 register address */
    #define REG_ADDR_MODE2                      ((uint8_t) 0x05)

    /* MODE2 default (reset) value */
    #define MODE2_DEFAULT                       ((uint8_t) 0x04)

    /* MODE2 register field masks */
    #define MODE2_BYPASS_MASK                   ((uint8_t) 0x80)
    #define MODE2_GAIN_MASK                     ((uint8_t) 0x70)
    #define MODE2_DR_MASK                       ((uint8_t) 0x0F)

    /* GAIN field values */
    #define MODE2_GAIN_1                        ((uint8_t) 0x00)
    #define MODE2_GAIN_2                        ((uint8_t) 0x10)
    #define MODE2_GAIN_4                        ((uint8_t) 0x20)
    #define MODE2_GAIN_8                        ((uint8_t) 0x30)
    #define MODE2_GAIN_16                       ((uint8_t) 0x40)
    #define MODE2_GAIN_32                       ((uint8_t) 0x50)

    /* DR field values */
    #define MODE2_DR_2_5_SPS                    ((uint8_t) 0x00)
    #define MODE2_DR_5_SPS                      ((uint8_t) 0x01)
    #define MODE2_DR_10_SPS                     ((uint8_t) 0x02)
    #define MODE2_DR_16_6_SPS                   ((uint8_t) 0x03)
    #define MODE2_DR_20_SPS                     ((uint8_t) 0x04)
    #define MODE2_DR_50_SPS                     ((uint8_t) 0x05)
    #define MODE2_DR_60_SPS                     ((uint8_t) 0x06)
    #define MODE2_DR_100_SPS                    ((uint8_t) 0x07)
    #define MODE2_DR_400_SPS                    ((uint8_t) 0x08)
    #define MODE2_DR_1200_SPS                   ((uint8_t) 0x09)
    #define MODE2_DR_2400_SPS                   ((uint8_t) 0x0A)
    #define MODE2_DR_4800_SPS                   ((uint8_t) 0x0B)
    #define MODE2_DR_7200_SPS                   ((uint8_t) 0x0C)
    #define MODE2_DR_14400_SPS                  ((uint8_t) 0x0D)
    #define MODE2_DR_19200_SPS                  ((uint8_t) 0x0E)
    #define MODE2_DR_38400_SPS                  ((uint8_t) 0x0F)



/* Register 0x06 (INPMUX) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                MUXP[2:0]              |                MUXN[3:0]              |
 * ---------------------------------------------------------------------------------
 */

    /* INPMUX register address */
    #define REG_ADDR_INPMUX                     ((uint8_t) 0x06)

    /* INPMUX default (reset) value */
    #define INPMUX_DEFAULT                      ((uint8_t) 0x01)

    /* INPMUX register field masks */
    #define INPMUX_MUXP_MASK                    ((uint8_t) 0xF0)
    #define INPMUX_MUXN_MASK                    ((uint8_t) 0x0F)

    /* MUXP field values */
    #define INPMUX_MUXP_AIN0                    ((uint8_t) 0x00)
    #define INPMUX_MUXP_AIN1                    ((uint8_t) 0x10)
    #define INPMUX_MUXP_AIN2                    ((uint8_t) 0x20)
    #define INPMUX_MUXP_AIN3                    ((uint8_t) 0x30)
    #define INPMUX_MUXP_AIN4                    ((uint8_t) 0x40)
    #define INPMUX_MUXP_AIN5                    ((uint8_t) 0x50)
    #define INPMUX_MUXP_AIN6                    ((uint8_t) 0x60)
    #define INPMUX_MUXP_AIN7                    ((uint8_t) 0x70)
    #define INPMUX_MUXP_AIN8                    ((uint8_t) 0x80)
    #define INPMUX_MUXP_AIN9                    ((uint8_t) 0x90)
    #define INPMUX_MUXP_AINCOM                  ((uint8_t) 0xA0)
    #define INPMUX_MUXP_TEMP_P                  ((uint8_t) 0xB0)
    #define INPMUX_MUXP_AVDD_P                  ((uint8_t) 0xC0)
    #define INPMUX_MUXP_DVDD_P                  ((uint8_t) 0xD0)
    #define INPMUX_MUXP_TDAC_P                  ((uint8_t) 0xE0)
    #define INPMUX_MUXP_OPEN                    ((uint8_t) 0xF0)

    /* MUXN field values */
    #define INPMUX_MUXN_AIN0                    ((uint8_t) 0x00)
    #define INPMUX_MUXN_AIN1                    ((uint8_t) 0x01)
    #define INPMUX_MUXN_AIN2                    ((uint8_t) 0x02)
    #define INPMUX_MUXN_AIN3                    ((uint8_t) 0x03)
    #define INPMUX_MUXN_AIN4                    ((uint8_t) 0x04)
    #define INPMUX_MUXN_AIN5                    ((uint8_t) 0x05)
    #define INPMUX_MUXN_AIN6                    ((uint8_t) 0x06)
    #define INPMUX_MUXN_AIN7                    ((uint8_t) 0x07)
    #define INPMUX_MUXN_AIN8                    ((uint8_t) 0x08)
    #define INPMUX_MUXN_AIN9                    ((uint8_t) 0x09)
    #define INPMUX_MUXN_AINCOM                  ((uint8_t) 0x0A)
    #define INPMUX_MUXN_TEMP_N                  ((uint8_t) 0x0B)
    #define INPMUX_MUXN_AVDD_N                  ((uint8_t) 0x0C)
    #define INPMUX_MUXN_DVDD_N                  ((uint8_t) 0x0D)
    #define INPMUX_MUXN_TDAC_N                  ((uint8_t) 0x0E)
    #define INPMUX_MUXN_OPEN                    ((uint8_t) 0x0F)



/* Register 0x07 (OFCAL0) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                    OFC[7:0]                                   |
 * ---------------------------------------------------------------------------------
 */

    /* INPMUX register address */
    #define REG_ADDR_OFCAL0                     ((uint8_t) 0x07)

    /* INPMUX default (reset) value */
    #define OFCAL0_DEFAULT                      ((uint8_t) 0x00)



/* Register 0x08 (OFCAL1) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                   OFC[15:8]                                   |
 * ---------------------------------------------------------------------------------
 */

    /* INPMUX register address */
    #define REG_ADDR_OFCAL1                     ((uint8_t) 0x08)

    /* INPMUX default (reset) value */
    #define OFCAL1_DEFAULT                      ((uint8_t) 0x00)



/* Register 0x09 (OFCAL2) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                   OFC[23:16]                                  |
 * ---------------------------------------------------------------------------------
 */

    /* INPMUX register address */
    #define REG_ADDR_OFCAL2                     ((uint8_t) 0x09)

    /* INPMUX default (reset) value */
    #define OFCAL2_DEFAULT                      ((uint8_t) 0x00)



/* Register 0x0A (FSCAL0) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                   FSCAL[7:0]                                  |
 * ---------------------------------------------------------------------------------
 */

    /* INPMUX register address */
    #define REG_ADDR_FSCAL0                     ((uint8_t) 0x0A)

    /* INPMUX default (reset) value */
    #define FSCAL0_DEFAULT                      ((uint8_t) 0x00)



/* Register 0x0B (FSCAL1) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                   FSCAL[15:8]                                 |
 * ---------------------------------------------------------------------------------
 */

    /* INPMUX register address */
    #define REG_ADDR_FSCAL1                     ((uint8_t) 0x0B)

    /* INPMUX default (reset) value */
    #define FSCAL1_DEFAULT                      ((uint8_t) 0x00)



/* Register 0x0C (FSCAL2) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                   FSCAL[23:16]                                |
 * ---------------------------------------------------------------------------------
 */

    /* INPMUX register address */
    #define REG_ADDR_FSCAL2                     ((uint8_t) 0x0C)

    /* INPMUX default (reset) value */
    #define FSCAL2_DEFAULT                      ((uint8_t) 0x40)



/* Register 0x0D (IDACMUX) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |               MUX2[3:0]               |               MUX1[3:0]               |
 * ---------------------------------------------------------------------------------
 */

    /* IDACMUX register address */
    #define REG_ADDR_IDACMUX                    ((uint8_t) 0x0D)

    /* IDACMUX default (reset) value */
    #define IDACMUX_DEFAULT                     ((uint8_t) 0xBB)

    /* IDACMUX register field masks */
    #define IDACMUX_MUX2_MASK                   ((uint8_t) 0xF0)
    #define IDACMUX_MUX1_MASK                   ((uint8_t) 0x0F)

    /* MUX2 field values */
    #define IDACMUX_MUX2_AIN0                   ((uint8_t) 0x00)
    #define IDACMUX_MUX2_AIN1                   ((uint8_t) 0x10)
    #define IDACMUX_MUX2_AIN2                   ((uint8_t) 0x20)
    #define IDACMUX_MUX2_AIN3                   ((uint8_t) 0x30)
    #define IDACMUX_MUX2_AIN4                   ((uint8_t) 0x40)
    #define IDACMUX_MUX2_AIN5                   ((uint8_t) 0x50)
    #define IDACMUX_MUX2_AIN6                   ((uint8_t) 0x60)
    #define IDACMUX_MUX2_AIN7                   ((uint8_t) 0x70)
    #define IDACMUX_MUX2_AIN8                   ((uint8_t) 0x80)
    #define IDACMUX_MUX2_AIN9                   ((uint8_t) 0x90)
    #define IDACMUX_MUX2_AINCOM                 ((uint8_t) 0xA0)
    #define IDACMUX_MUX2_NO_CONN                ((uint8_t) 0xB0)

    /* MUX1 field values */
    #define IDACMUX_MUX1_AIN0                   ((uint8_t) 0x00)
    #define IDACMUX_MUX1_AIN1                   ((uint8_t) 0x01)
    #define IDACMUX_MUX1_AIN2                   ((uint8_t) 0x02)
    #define IDACMUX_MUX1_AIN3                   ((uint8_t) 0x03)
    #define IDACMUX_MUX1_AIN4                   ((uint8_t) 0x04)
    #define IDACMUX_MUX1_AIN5                   ((uint8_t) 0x05)
    #define IDACMUX_MUX1_AIN6                   ((uint8_t) 0x06)
    #define IDACMUX_MUX1_AIN7                   ((uint8_t) 0x07)
    #define IDACMUX_MUX1_AIN8                   ((uint8_t) 0x08)
    #define IDACMUX_MUX1_AIN9                   ((uint8_t) 0x09)
    #define IDACMUX_MUX1_AINCOM                 ((uint8_t) 0x0A)
    #define IDACMUX_MUX1_NO_CONN                ((uint8_t) 0x0B)



/* Register 0x0E (IDACMAG) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |               MAG2[3:0]               |               MAG1[3:0]               |
 * ---------------------------------------------------------------------------------
 */

    /* IDACMAG register address */
    #define REG_ADDR_IDACMAG                    ((uint8_t) 0x0E)

    /* IDACMAG default (reset) value */
    #define IDACMAG_DEFAULT                     ((uint8_t) 0x00)

    /* IDACMAG register field masks */
    #define IDACMAG_MAG2_MASK                   ((uint8_t) 0xF0)
    #define IDACMAG_MAG1_MASK                   ((uint8_t) 0x0F)

    /* MUX2 field values */
    #define IDACMAG_MAG2_OFF                    ((uint8_t) 0x00)
    #define IDACMAG_MAG2_50uA                   ((uint8_t) 0x10)
    #define IDACMAG_MAG2_100uA                  ((uint8_t) 0x20)
    #define IDACMAG_MAG2_250uA                  ((uint8_t) 0x30)
    #define IDACMAG_MAG2_500uA                  ((uint8_t) 0x40)
    #define IDACMAG_MAG2_750uA                  ((uint8_t) 0x50)
    #define IDACMAG_MAG2_1000uA                 ((uint8_t) 0x60)
    #define IDACMAG_MAG2_1500uA                 ((uint8_t) 0x70)
    #define IDACMAG_MAG2_2000uA                 ((uint8_t) 0x80)
    #define IDACMAG_MAG2_2500uA                 ((uint8_t) 0x90)
    #define IDACMAG_MAG2_3000uA                 ((uint8_t) 0xA0)

    /* MUX1 field values */
    #define IDACMAG_MAG1_OFF                    ((uint8_t) 0x00)
    #define IDACMAG_MAG1_50uA                   ((uint8_t) 0x01)
    #define IDACMAG_MAG1_100uA                  ((uint8_t) 0x02)
    #define IDACMAG_MAG1_250uA                  ((uint8_t) 0x03)
    #define IDACMAG_MAG1_500uA                  ((uint8_t) 0x04)
    #define IDACMAG_MAG1_750uA                  ((uint8_t) 0x05)
    #define IDACMAG_MAG1_1000uA                 ((uint8_t) 0x06)
    #define IDACMAG_MAG1_1500uA                 ((uint8_t) 0x07)
    #define IDACMAG_MAG1_2000uA                 ((uint8_t) 0x08)
    #define IDACMAG_MAG1_2500uA                 ((uint8_t) 0x09)
    #define IDACMAG_MAG1_3000uA                 ((uint8_t) 0x0A)



/* Register 0x0F (REFMUX) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |     RESERVED      |          RMUXP[2:0]         |          RMUXN[2:0]         |
 * ---------------------------------------------------------------------------------
 */

    /* REFMUX register address */
    #define REG_ADDR_REFMUX                     ((uint8_t) 0x0F)

    /* REFMUX default (reset) value */
    #define REFMUX_DEFAULT                      ((uint8_t) 0x00)

    /* REFMUX register field masks */
    #define REFMUX_RMUXP_MASK                   ((uint8_t) 0x38)
    #define REFMUX_RMUXN_MASK                   ((uint8_t) 0x07)

    /* RMUXP field values */
    #define REFMUX_RMUXP_INT_REF_P              ((uint8_t) 0x00)
    #define REFMUX_RMUXP_EXT_AIN0               ((uint8_t) 0x08)
    #define REFMUX_RMUXP_EXT_AIN2               ((uint8_t) 0x10)
    #define REFMUX_RMUXP_EXT_AIN4               ((uint8_t) 0x18)
    #define REFMUX_RMUXP_INT_AVDD               ((uint8_t) 0x20)

    /* RMUXN field values */
    #define REFMUX_RMUXN_INT_REF_N              ((uint8_t) 0x00)
    #define REFMUX_RMUXN_EXT_AIN1               ((uint8_t) 0x01)
    #define REFMUX_RMUXN_EXT_AIN3               ((uint8_t) 0x02)
    #define REFMUX_RMUXN_EXT_AIN5               ((uint8_t) 0x03)
    #define REFMUX_RMUXN_INT_AVSS               ((uint8_t) 0x04)



/* Register 0x10 (TDACP) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |   OUTP  |      RESERVED     |                    MAGP[4:0]                    |
 * ---------------------------------------------------------------------------------
 */

    /* TDACP register address */
    #define REG_ADDR_TDACP                      ((uint8_t) 0x10)

    /* TDACP default (reset) value */
    #define TDACP_DEFAULT                       ((uint8_t) 0x00)

    /* TDACP register field masks */
    #define TDACP_OUTP_MASK                     ((uint8_t) 0x80)
    #define TDACP_MAGP_MASK                     ((uint8_t) 0x1F)

    /* MAGP field values */
    #define TDACP_MAGP_4_5_V                    ((uint8_t) 0x09)
    #define TDACP_MAGP_3_5_V                    ((uint8_t) 0x08)
    #define TDACP_MAGP_3_V                      ((uint8_t) 0x07)
    #define TDACP_MAGP_2_75_V                   ((uint8_t) 0x06)
    #define TDACP_MAGP_2_625_V                  ((uint8_t) 0x05)
    #define TDACP_MAGP_2_5625_V                 ((uint8_t) 0x04)
    #define TDACP_MAGP_2_53125_V                ((uint8_t) 0x03)
    #define TDACP_MAGP_2_515625_V               ((uint8_t) 0x02)
    #define TDACP_MAGP_2_5078125_V              ((uint8_t) 0x01)
    #define TDACP_MAGP_2_5_V                    ((uint8_t) 0x00)
    #define TDACP_MAGP_2_4921875_V              ((uint8_t) 0x11)
    #define TDACP_MAGP_2_484375_V               ((uint8_t) 0x12)
    #define TDACP_MAGP_2_46875_V                ((uint8_t) 0x13)
    #define TDACP_MAGP_2_4375_V                 ((uint8_t) 0x14)
    #define TDACP_MAGP_2_375_V                  ((uint8_t) 0x15)
    #define TDACP_MAGP_2_25_V                   ((uint8_t) 0x16)
    #define TDACP_MAGP_2_V                      ((uint8_t) 0x17)
    #define TDACP_MAGP_1_5_V                    ((uint8_t) 0x18)
    #define TDACP_MAGP_0_5_V                    ((uint8_t) 0x19)



/* Register 0x11 (TDACN) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |   OUTN  |      RESERVED     |                    MAGN[4:0]                    |
 * ---------------------------------------------------------------------------------
 */

    /* TDACN register address */
    #define REG_ADDR_TDACN                      ((uint8_t) 0x11)

    /* TDACN default (reset) value */
    #define TDACN_DEFAULT                       ((uint8_t) 0x00)

    /* TDACN register field masks */
    #define TDACN_OUTN_MASK                     ((uint8_t) 0x80)
    #define TDACN_MAGN_MASK                     ((uint8_t) 0x1F)

    /* MAGN field values */
    #define TDACN_MAGN_4_5_V                    ((uint8_t) 0x09)
    #define TDACN_MAGN_3_5_V                    ((uint8_t) 0x08)
    #define TDACN_MAGN_3_V                      ((uint8_t) 0x07)
    #define TDACN_MAGN_2_75_V                   ((uint8_t) 0x06)
    #define TDACN_MAGN_2_625_V                  ((uint8_t) 0x05)
    #define TDACN_MAGN_2_5625_V                 ((uint8_t) 0x04)
    #define TDACN_MAGN_2_53125_V                ((uint8_t) 0x03)
    #define TDACN_MAGN_2_515625_V               ((uint8_t) 0x02)
    #define TDACN_MAGN_2_5078125_V              ((uint8_t) 0x01)
    #define TDACN_MAGN_2_5_V                    ((uint8_t) 0x00)
    #define TDACN_MAGN_2_4921875_V              ((uint8_t) 0x11)
    #define TDACN_MAGN_2_484375_V               ((uint8_t) 0x12)
    #define TDACN_MAGN_2_46875_V                ((uint8_t) 0x13)
    #define TDACN_MAGN_2_4375_V                 ((uint8_t) 0x14)
    #define TDACN_MAGN_2_375_V                  ((uint8_t) 0x15)
    #define TDACN_MAGN_2_25_V                   ((uint8_t) 0x16)
    #define TDACN_MAGN_2_V                      ((uint8_t) 0x17)
    #define TDACN_MAGN_1_5_V                    ((uint8_t) 0x18)
    #define TDACN_MAGN_0_5_V                    ((uint8_t) 0x19)



/* Register 0x12 (GPIOCON) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                    CON[7:0]                                   |
 * ---------------------------------------------------------------------------------
 */

    /* GPIOCON register address */
    #define REG_ADDR_GPIOCON                    ((uint8_t) 0x12)

    /* GPIOCON default (reset) value */
    #define GPIOCON_DEFAULT                     ((uint8_t) 0x00)

    /* GPIOCON register Fields */
    #define GPIOCON_AINCOM_EN                   ((uint8_t) 0x80)
    #define GPIOCON_AIN9_EN                     ((uint8_t) 0x40)
    #define GPIOCON_AIN8_EN                     ((uint8_t) 0x20)
    #define GPIOCON_AIN7_EN                     ((uint8_t) 0x10)
    #define GPIOCON_AIN6_EN                     ((uint8_t) 0x08)
    #define GPIOCON_AIN5_EN                     ((uint8_t) 0x04)
    #define GPIOCON_AIN4_EN                     ((uint8_t) 0x02)
    #define GPIOCON_AIN3_EN                     ((uint8_t) 0x01)



/* Register 0x13 (GPIODIR) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                    DIR[7:0]                                   |
 * ---------------------------------------------------------------------------------
 */

    /* GPIODIR register address */
    #define REG_ADDR_GPIODIR                    ((uint8_t) 0x13)

    /* GPIODIR default (reset) value */
    #define GPIODIR_DEFAULT                     ((uint8_t) 0x00)

    /* GPIODIR register Fields */
    #define GPIODIR_AINCOM_INPUT                ((uint8_t) 0x80)
    #define GPIODIR_AIN9_INPUT                  ((uint8_t) 0x40)
    #define GPIODIR_AIN8_INPUT                  ((uint8_t) 0x20)
    #define GPIODIR_AIN7_INPUT                  ((uint8_t) 0x10)
    #define GPIODIR_AIN6_INPUT                  ((uint8_t) 0x08)
    #define GPIODIR_AIN5_INPUT                  ((uint8_t) 0x04)
    #define GPIODIR_AIN4_INPUT                  ((uint8_t) 0x02)
    #define GPIODIR_AIN3_INPUT                  ((uint8_t) 0x01)



/* Register 0x14 (GPIODAT) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                    DAT[7:0]                                   |
 * ---------------------------------------------------------------------------------
 */

    /* GPIODAT register address */
    #define REG_ADDR_GPIODAT                    ((uint8_t) 0x14)

    /* GPIODAT default (reset) value */
    #define GPIODAT_DEFAULT                     ((uint8_t) 0x00)

    /* GPIODAT register Fields */
    #define GPIODAT_AINCOM_HIGH                 ((uint8_t) 0x80)
    #define GPIODAT_AIN9_HIGH                   ((uint8_t) 0x40)
    #define GPIODAT_AIN8_HIGH                   ((uint8_t) 0x20)
    #define GPIODAT_AIN7_HIGH                   ((uint8_t) 0x10)
    #define GPIODAT_AIN6_HIGH                   ((uint8_t) 0x08)
    #define GPIODAT_AIN5_HIGH                   ((uint8_t) 0x04)
    #define GPIODAT_AIN4_HIGH                   ((uint8_t) 0x02)
    #define GPIODAT_AIN3_HIGH                   ((uint8_t) 0x01)



/* Additional ADS1263 Registers */
#ifdef ADS1263_ONLY_FEATURES


/* Register 0x15 (ADC2CFG) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |      DR2[1:0]     |          REF2[2:0]          |          GAIN2[2:0]         |
 * ---------------------------------------------------------------------------------
 */

    /* ADC2CFG register address */
    #define REG_ADDR_ADC2CFG                    ((uint8_t) 0x15)

    /* ADC2CFG default (reset) value */
    #define ADC2CFG_DEFAULT                     ((uint8_t) 0x00)

    /* ADC2CFG register field masks */
    #define ADC2CFG_DR2_MASK                    ((uint8_t) 0xC0)
    #define ADC2CFG_REF2_MASK                   ((uint8_t) 0x38)
    #define ADC2CFG_GAIN2_MASK                  ((uint8_t) 0x07)

    /* DR2 field values */
    #define ADC2CFG_DR2_10_SPS                  ((uint8_t) 0x00)
    #define ADC2CFG_DR2_100_SPS                 ((uint8_t) 0x40)
    #define ADC2CFG_DR2_400_SPS                 ((uint8_t) 0x80)
    #define ADC2CFG_DR2_800_SPS                 ((uint8_t) 0xC0)

    /* REF2 field values */
    #define ADC2CFG_REF2_INTP_INN               ((uint8_t) 0x00)
    #define ADC2CFG_REF2_AIN0_AIN1              ((uint8_t) 0x08)
    #define ADC2CFG_REF2_AIN2_AIN3              ((uint8_t) 0x10)
    #define ADC2CFG_REF2_AIN4_AIN5              ((uint8_t) 0x18)
    #define ADC2CFG_REF2_AVDD_AVSS              ((uint8_t) 0x20)

    /* GAIN2 field values */
    #define ADC2CFG_GAIN2_1                     ((uint8_t) 0x00)
    #define ADC2CFG_GAIN2_2                     ((uint8_t) 0x01)
    #define ADC2CFG_GAIN2_4                     ((uint8_t) 0x02)
    #define ADC2CFG_GAIN2_8                     ((uint8_t) 0x03)
    #define ADC2CFG_GAIN2_16                    ((uint8_t) 0x04)
    #define ADC2CFG_GAIN2_32                    ((uint8_t) 0x05)
    #define ADC2CFG_GAIN2_64                    ((uint8_t) 0x06)
    #define ADC2CFG_GAIN2_128                   ((uint8_t) 0x07)



/* Register 0x16 (ADC2MUX) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |               MUXP2[3:0]               |              MUXN2[3:0]               |
 * ---------------------------------------------------------------------------------
 */

    /* ADC2MUX register address */
    #define REG_ADDR_ADC2MUX                    ((uint8_t) 0x16)

    /* ADC2MUX default (reset) value */
    #define ADC2MUX_DEFAULT                     ((uint8_t) 0x01)

    /* ADC2MUX register field masks */
    #define ADC2MUX_MUXP_MASK                   ((uint8_t) 0xF0)
    #define ADC2MUX_MUXN_MASK                   ((uint8_t) 0x0F)

    /* MUXP2 field values */
    #define ADC2MUX_MUXP2_AIN0                  ((uint8_t) 0x00)
    #define ADC2MUX_MUXP2_AIN1                  ((uint8_t) 0x10)
    #define ADC2MUX_MUXP2_AIN2                  ((uint8_t) 0x20)
    #define ADC2MUX_MUXP2_AIN3                  ((uint8_t) 0x30)
    #define ADC2MUX_MUXP2_AIN4                  ((uint8_t) 0x40)
    #define ADC2MUX_MUXP2_AIN5                  ((uint8_t) 0x50)
    #define ADC2MUX_MUXP2_AIN6                  ((uint8_t) 0x60)
    #define ADC2MUX_MUXP2_AIN7                  ((uint8_t) 0x70)
    #define ADC2MUX_MUXP2_AIN8                  ((uint8_t) 0x80)
    #define ADC2MUX_MUXP2_AIN9                  ((uint8_t) 0x90)
    #define ADC2MUX_MUXP2_AINCOM                ((uint8_t) 0xA0)
    #define ADC2MUX_MUXP2_TEMP_P                ((uint8_t) 0xB0)
    #define ADC2MUX_MUXP2_AVDD_P                ((uint8_t) 0xC0)
    #define ADC2MUX_MUXP2_DVDD_P                ((uint8_t) 0xD0)
    #define ADC2MUX_MUXP2_TDAC_P                ((uint8_t) 0xE0)
    #define ADC2MUX_MUXP2_OPEN                  ((uint8_t) 0xF0)

    /* MUXN2 field values */
    #define ADC2MUX_MUXN2_AIN0                  ((uint8_t) 0x00)
    #define ADC2MUX_MUXN2_AIN1                  ((uint8_t) 0x01)
    #define ADC2MUX_MUXN2_AIN2                  ((uint8_t) 0x02)
    #define ADC2MUX_MUXN2_AIN3                  ((uint8_t) 0x03)
    #define ADC2MUX_MUXN2_AIN4                  ((uint8_t) 0x04)
    #define ADC2MUX_MUXN2_AIN5                  ((uint8_t) 0x05)
    #define ADC2MUX_MUXN2_AIN6                  ((uint8_t) 0x06)
    #define ADC2MUX_MUXN2_AIN7                  ((uint8_t) 0x07)
    #define ADC2MUX_MUXN2_AIN8                  ((uint8_t) 0x08)
    #define ADC2MUX_MUXN2_AIN9                  ((uint8_t) 0x09)
    #define ADC2MUX_MUXN2_AINCOM                ((uint8_t) 0x0A)
    #define ADC2MUX_MUXN2_TEMP_P                ((uint8_t) 0x0B)
    #define ADC2MUX_MUXN2_AVDD_P                ((uint8_t) 0x0C)
    #define ADC2MUX_MUXN2_DVDD_P                ((uint8_t) 0x0D)
    #define ADC2MUX_MUXN2_TDAC_P                ((uint8_t) 0x0E)
    #define ADC2MUX_MUXN2_OPEN                  ((uint8_t) 0x0F)



/* Register 0x17 (ADC2OFC0) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                    OFC2[7:0]                                  |
 * ---------------------------------------------------------------------------------
 */

    /* ADC2OFC0 register address */
    #define REG_ADDR_ADC2OFC0                   ((uint8_t) 0x17)

    /* ADC2OFC0 default (reset) value */
    #define ADC2OFC0_DEFAULT                    ((uint8_t) 0x00)



/* Register 0x18 (ADC2OFC1) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                   OFC2[15:8]                                  |
 * ---------------------------------------------------------------------------------
 */

    /* ADC2OFC1 register address */
    #define REG_ADDR_ADC2OFC1                   ((uint8_t) 0x18)

    /* ADC2OFC1 default (reset) value */
    #define ADC2OFC1_DEFAULT                    ((uint8_t) 0x00)



/* Register 0x19 (ADC2FSC0) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                   FSC2[7:0]                                   |
 * ---------------------------------------------------------------------------------
 */

    /* ADC2FSC0 register address */
    #define REG_ADDR_ADC2FSC0                   ((uint8_t) 0x19)

    /* ADC2FSC0 default (reset) value */
    #define ADC2FSC0_DEFAULT                    ((uint8_t) 0x00)



/* Register 0x1A (ADC2FSC1) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                   FSC2[15:9]                                  |
 * ---------------------------------------------------------------------------------
 */

    /* ADC2FSC1 register address */
    #define REG_ADDR_ADC2FSC1                   ((uint8_t) 0x1A)

    /* ADC2FSC1 default (reset) value */
    #define ADC2FSC1_DEFAULT                    ((uint8_t) 0x40)


#endif /* ADS1263_ONLY_FEATURES */



//****************************************************************************
//
// Function Prototypes
//
//****************************************************************************

void    adcStartupRoutine(void);
int32_t readData(uint8_t status[], uint8_t data[], uint8_t crc[]);
uint8_t readSingleRegister(uint8_t address);
void    readMultipleRegisters(uint8_t startAddress, uint8_t count);
void    sendCommand(uint8_t op_code);
void    startConversions(void);
void    writeSingleRegister(uint8_t address, uint8_t data);
void    writeMultipleRegisters(uint8_t startAddress, uint8_t count, const uint8_t regData[]);
void    restoreRegisterDefaults(void);

// Internal variable "getters"
uint8_t getRegisterValue(uint8_t address);



//*****************************************************************************
//
// Macros
//
//*****************************************************************************

/** Register bit checking macros...
 *  Return true if register bit is set (since last read or write).
 */
#define STATUS_BYTE_ENABLED     ((bool) (getRegisterValue(REG_ADDR_INTERFACE) & INTERFACE_STATUS_MASK))
#define CRC_BYTE_ENABLED        ((bool) (getRegisterValue(REG_ADDR_INTERFACE) & INTERFACE_CRC_MASK))



#endif /* ADS1263_H_ */
