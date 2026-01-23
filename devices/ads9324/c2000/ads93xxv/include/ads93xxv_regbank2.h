/*
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

#ifndef ADS93XXV_PAGE2_H_
#define ADS93XXV_PAGE2_H_

#include <stdint.h>


//**********************************************************************************
//
// Function prototypes
//
//**********************************************************************************



//**********************************************************************************
//
// Device commands
//
//**********************************************************************************



//**********************************************************************************
//
// Register definitions
//
//**********************************************************************************


/* Register 0x08 (PGA_CONFIG_AIN15_16) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * | CME_CORR_EN_AIN15 |                    CM_RANGE_AIN15[2:0]                    |      RESERVED     |                   INPUT_RANGE_AIN15[2:0]                  |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * | CME_CORR_EN_AIN16 |                    CM_RANGE_AIN16[2:0]                    |      RESERVED     |                   INPUT_RANGE_AIN16[2:0]                  |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PGA_CONFIG_AIN15_16 register */
    #define BANK2_PGA_CONFIG_AIN15_16_ADDRESS					((uint8_t) 0x08)
    #define BANK2_PGA_CONFIG_AIN15_16_DEFAULT					((uint16_t) 0x0000)

    /* CME_CORR_EN_AIN15 field */
    #define BANK2_PGA_CONFIG_AIN15_16_CME_CORR_EN_AIN15_MASK	((uint16_t) 0x8000)
    #define BANK2_PGA_CONFIG_AIN15_16_CME_CORR_EN_AIN15_BITOFFSET	(15)
    #define BANK2_PGA_CONFIG_AIN15_16_CME_CORR_EN_AIN15_DISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN15_16_CME_CORR_EN_AIN15_ENABLED	((uint16_t) 0x8000)

    /* CM_RANGE_AIN15 field */
    #define BANK2_PGA_CONFIG_AIN15_16_CM_RANGE_AIN15_MASK		((uint16_t) 0x7000)
    #define BANK2_PGA_CONFIG_AIN15_16_CM_RANGE_AIN15_BITOFFSET	(12)
    #define BANK2_PGA_CONFIG_AIN15_16_CM_RANGE_AIN15_DIFFERENTIAL	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN15_16_CM_RANGE_AIN15_SINGLEENDED	((uint16_t) 0x5000)
    #define BANK2_PGA_CONFIG_AIN15_16_CM_RANGE_AIN15_SINGLEENDEDOPENWIRESAFE	((uint16_t) 0x6000)

    /* INPUT_RANGE_AIN15 field */
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN15_MASK	((uint16_t) 0x0700)
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN15_BITOFFSET	(8)
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN15_5V		((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN15_RESERVED0	((uint16_t) 0x0100)
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN15_2		((uint16_t) 0x0200)
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN15_6		((uint16_t) 0x0300)
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN15_10V		((uint16_t) 0x0400)
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN15_12		((uint16_t) 0x0500)
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN15_RESERVED1	((uint16_t) 0x0600)
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN15_RESERVED2	((uint16_t) 0x0700)

    /* CME_CORR_EN_AIN16 field */
    #define BANK2_PGA_CONFIG_AIN15_16_CME_CORR_EN_AIN16_MASK	((uint16_t) 0x0080)
    #define BANK2_PGA_CONFIG_AIN15_16_CME_CORR_EN_AIN16_BITOFFSET	(7)
    #define BANK2_PGA_CONFIG_AIN15_16_CME_CORR_EN_AIN16_DISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN15_16_CME_CORR_EN_AIN16_ENABLED	((uint16_t) 0x0080)

    /* CM_RANGE_AIN16 field */
    #define BANK2_PGA_CONFIG_AIN15_16_CM_RANGE_AIN16_MASK		((uint16_t) 0x0070)
    #define BANK2_PGA_CONFIG_AIN15_16_CM_RANGE_AIN16_BITOFFSET	(4)
    #define BANK2_PGA_CONFIG_AIN15_16_CM_RANGE_AIN16_DIFFERENTIAL	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN15_16_CM_RANGE_AIN16_SINGLEENDED	((uint16_t) 0x0050)
    #define BANK2_PGA_CONFIG_AIN15_16_CM_RANGE_AIN16_SINGLEENDEDOPENWIRESAFE	((uint16_t) 0x0060)

    /* INPUT_RANGE_AIN16 field */
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN16_MASK	((uint16_t) 0x0007)
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN16_BITOFFSET	(0)
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN16_5V		((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN16_RESERVED0	((uint16_t) 0x0001)
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN16_2		((uint16_t) 0x0002)
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN16_6		((uint16_t) 0x0003)
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN16_10V		((uint16_t) 0x0004)
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN16_12		((uint16_t) 0x0005)
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN16_RESERVED1	((uint16_t) 0x0006)
    #define BANK2_PGA_CONFIG_AIN15_16_INPUT_RANGE_AIN16_RESERVED2	((uint16_t) 0x0007)


/* Register 0x09 (PGA_CONFIG_AIN13_14) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * | CME_CORR_EN_AIN13 |                    CM_RANGE_AIN13[2:0]                    |      RESERVED     |                   INPUT_RANGE_AIN13[2:0]                  |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * | CME_CORR_EN_AIN14 |                    CM_RANGE_AIN14[2:0]                    |      RESERVED     |                   INPUT_RANGE_AIN14[2:0]                  |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PGA_CONFIG_AIN13_14 register */
    #define BANK2_PGA_CONFIG_AIN13_14_ADDRESS					((uint8_t) 0x09)
    #define BANK2_PGA_CONFIG_AIN13_14_DEFAULT					((uint16_t) 0x0000)

    /* CME_CORR_EN_AIN13 field */
    #define BANK2_PGA_CONFIG_AIN13_14_CME_CORR_EN_AIN13_MASK	((uint16_t) 0x8000)
    #define BANK2_PGA_CONFIG_AIN13_14_CME_CORR_EN_AIN13_BITOFFSET	(15)
    #define BANK2_PGA_CONFIG_AIN13_14_CME_CORR_EN_AIN13_DISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN13_14_CME_CORR_EN_AIN13_ENABLED	((uint16_t) 0x8000)

    /* CM_RANGE_AIN13 field */
    #define BANK2_PGA_CONFIG_AIN13_14_CM_RANGE_AIN13_MASK		((uint16_t) 0x7000)
    #define BANK2_PGA_CONFIG_AIN13_14_CM_RANGE_AIN13_BITOFFSET	(12)
    #define BANK2_PGA_CONFIG_AIN13_14_CM_RANGE_AIN13_DIFFERENTIAL	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN13_14_CM_RANGE_AIN13_SINGLEENDED	((uint16_t) 0x5000)
    #define BANK2_PGA_CONFIG_AIN13_14_CM_RANGE_AIN13_SINGLEENDEDOPENWIRESAFE	((uint16_t) 0x6000)

    /* INPUT_RANGE_AIN13 field */
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN13_MASK	((uint16_t) 0x0700)
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN13_BITOFFSET	(8)
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN13_5V		((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN13_RESERVED0	((uint16_t) 0x0100)
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN13_2		((uint16_t) 0x0200)
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN13_6		((uint16_t) 0x0300)
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN13_10V		((uint16_t) 0x0400)
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN13_12		((uint16_t) 0x0500)
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN13_RESERVED1	((uint16_t) 0x0600)
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN13_RESERVED2	((uint16_t) 0x0700)

    /* CME_CORR_EN_AIN14 field */
    #define BANK2_PGA_CONFIG_AIN13_14_CME_CORR_EN_AIN14_MASK	((uint16_t) 0x0080)
    #define BANK2_PGA_CONFIG_AIN13_14_CME_CORR_EN_AIN14_BITOFFSET	(7)
    #define BANK2_PGA_CONFIG_AIN13_14_CME_CORR_EN_AIN14_DISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN13_14_CME_CORR_EN_AIN14_ENABLED	((uint16_t) 0x0080)

    /* CM_RANGE_AIN14 field */
    #define BANK2_PGA_CONFIG_AIN13_14_CM_RANGE_AIN14_MASK		((uint16_t) 0x0070)
    #define BANK2_PGA_CONFIG_AIN13_14_CM_RANGE_AIN14_BITOFFSET	(4)
    #define BANK2_PGA_CONFIG_AIN13_14_CM_RANGE_AIN14_DIFFERENTIAL	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN13_14_CM_RANGE_AIN14_SINGLEENDED	((uint16_t) 0x0050)
    #define BANK2_PGA_CONFIG_AIN13_14_CM_RANGE_AIN14_SINGLEENDEDOPENWIRESAFE	((uint16_t) 0x0060)

    /* INPUT_RANGE_AIN14 field */
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN14_MASK	((uint16_t) 0x0007)
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN14_BITOFFSET	(0)
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN14_5V		((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN14_RESERVED0	((uint16_t) 0x0001)
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN14_2		((uint16_t) 0x0002)
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN14_6		((uint16_t) 0x0003)
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN14_10V		((uint16_t) 0x0004)
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN14_12		((uint16_t) 0x0005)
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN14_RESERVED1	((uint16_t) 0x0006)
    #define BANK2_PGA_CONFIG_AIN13_14_INPUT_RANGE_AIN14_RESERVED2	((uint16_t) 0x0007)


/* Register 0x0A (PGA_CONFIG_AIN11_12) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * | CME_CORR_EN_AIN11 |                    CM_RANGE_AIN11[2:0]                    |      RESERVED     |                   INPUT_RANGE_AIN11[2:0]                  |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * | CME_CORR_EN_AIN12 |                    CM_RANGE_AIN12[2:0]                    |      RESERVED     |                   INPUT_RANGE_AIN12[2:0]                  |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PGA_CONFIG_AIN11_12 register */
    #define BANK2_PGA_CONFIG_AIN11_12_ADDRESS					((uint8_t) 0x0A)
    #define BANK2_PGA_CONFIG_AIN11_12_DEFAULT					((uint16_t) 0x0000)

    /* CME_CORR_EN_AIN11 field */
    #define BANK2_PGA_CONFIG_AIN11_12_CME_CORR_EN_AIN11_MASK	((uint16_t) 0x8000)
    #define BANK2_PGA_CONFIG_AIN11_12_CME_CORR_EN_AIN11_BITOFFSET	(15)
    #define BANK2_PGA_CONFIG_AIN11_12_CME_CORR_EN_AIN11_DISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN11_12_CME_CORR_EN_AIN11_ENABLED	((uint16_t) 0x8000)

    /* CM_RANGE_AIN11 field */
    #define BANK2_PGA_CONFIG_AIN11_12_CM_RANGE_AIN11_MASK		((uint16_t) 0x7000)
    #define BANK2_PGA_CONFIG_AIN11_12_CM_RANGE_AIN11_BITOFFSET	(12)
    #define BANK2_PGA_CONFIG_AIN11_12_CM_RANGE_AIN11_DIFFERENTIAL	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN11_12_CM_RANGE_AIN11_SINGLEENDED	((uint16_t) 0x5000)
    #define BANK2_PGA_CONFIG_AIN11_12_CM_RANGE_AIN11_SINGLEENDEDOPENWIRESAFE	((uint16_t) 0x6000)

    /* INPUT_RANGE_AIN11 field */
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN11_MASK	((uint16_t) 0x0700)
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN11_BITOFFSET	(8)
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN11_5V		((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN11_RESERVED0	((uint16_t) 0x0100)
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN11_2		((uint16_t) 0x0200)
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN11_6		((uint16_t) 0x0300)
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN11_10V		((uint16_t) 0x0400)
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN11_12		((uint16_t) 0x0500)
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN11_RESERVED1	((uint16_t) 0x0600)
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN11_RESERVED2	((uint16_t) 0x0700)

    /* CME_CORR_EN_AIN12 field */
    #define BANK2_PGA_CONFIG_AIN11_12_CME_CORR_EN_AIN12_MASK	((uint16_t) 0x0080)
    #define BANK2_PGA_CONFIG_AIN11_12_CME_CORR_EN_AIN12_BITOFFSET	(7)
    #define BANK2_PGA_CONFIG_AIN11_12_CME_CORR_EN_AIN12_DISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN11_12_CME_CORR_EN_AIN12_ENABLED	((uint16_t) 0x0080)

    /* CM_RANGE_AIN12 field */
    #define BANK2_PGA_CONFIG_AIN11_12_CM_RANGE_AIN12_MASK		((uint16_t) 0x0070)
    #define BANK2_PGA_CONFIG_AIN11_12_CM_RANGE_AIN12_BITOFFSET	(4)
    #define BANK2_PGA_CONFIG_AIN11_12_CM_RANGE_AIN12_FULLYDIFFERENTIAL	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN11_12_CM_RANGE_AIN12_SINGLEENDED	((uint16_t) 0x0050)
    #define BANK2_PGA_CONFIG_AIN11_12_CM_RANGE_AIN12_SINGLEENDEDOPENWIRESAFE	((uint16_t) 0x0060)

    /* INPUT_RANGE_AIN12 field */
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN12_MASK	((uint16_t) 0x0007)
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN12_BITOFFSET	(0)
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN12_5V		((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN12_RESERVED0	((uint16_t) 0x0001)
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN12_2		((uint16_t) 0x0002)
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN12_6		((uint16_t) 0x0003)
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN12_10V		((uint16_t) 0x0004)
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN12_12		((uint16_t) 0x0005)
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN12_RESERVED1	((uint16_t) 0x0006)
    #define BANK2_PGA_CONFIG_AIN11_12_INPUT_RANGE_AIN12_RESERVED2	((uint16_t) 0x0007)


/* Register 0x0B (PGA_CONFIG_AIN9_10) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |  CME_CORR_EN_AIN9 |                     CM_RANGE_AIN9[2:0]                    |      RESERVED     |                   INPUT_RANGE_AIN9[2:0]                   |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * | CME_CORR_EN_AIN10 |                    CM_RANGE_AIN10[2:0]                    |      RESERVED     |                   INPUT_RANGE_AIN10[2:0]                  |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PGA_CONFIG_AIN9_10 register */
    #define BANK2_PGA_CONFIG_AIN9_10_ADDRESS					((uint8_t) 0x0B)
    #define BANK2_PGA_CONFIG_AIN9_10_DEFAULT					((uint16_t) 0x0000)

    /* CME_CORR_EN_AIN9 field */
    #define BANK2_PGA_CONFIG_AIN9_10_CME_CORR_EN_AIN9_MASK		((uint16_t) 0x8000)
    #define BANK2_PGA_CONFIG_AIN9_10_CME_CORR_EN_AIN9_BITOFFSET	(15)
    #define BANK2_PGA_CONFIG_AIN9_10_CME_CORR_EN_AIN9_DISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN9_10_CME_CORR_EN_AIN9_ENABLED	((uint16_t) 0x8000)

    /* CM_RANGE_AIN9 field */
    #define BANK2_PGA_CONFIG_AIN9_10_CM_RANGE_AIN9_MASK			((uint16_t) 0x7000)
    #define BANK2_PGA_CONFIG_AIN9_10_CM_RANGE_AIN9_BITOFFSET	(12)
    #define BANK2_PGA_CONFIG_AIN9_10_CM_RANGE_AIN9_DIFFERENTIAL	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN9_10_CM_RANGE_AIN9_SINGLEENDED	((uint16_t) 0x5000)
    #define BANK2_PGA_CONFIG_AIN9_10_CM_RANGE_AIN9_SINGLEENDEDOPENWIRESAFE	((uint16_t) 0x6000)

    /* INPUT_RANGE_AIN9 field */
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN9_MASK		((uint16_t) 0x0700)
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN9_BITOFFSET	(8)
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN9_5V		((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN9_RESERVED0	((uint16_t) 0x0100)
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN9_2			((uint16_t) 0x0200)
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN9_6			((uint16_t) 0x0300)
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN9_10V		((uint16_t) 0x0400)
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN9_12		((uint16_t) 0x0500)
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN9_RESERVED1	((uint16_t) 0x0600)
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN9_RESERVED2	((uint16_t) 0x0700)

    /* CME_CORR_EN_AIN10 field */
    #define BANK2_PGA_CONFIG_AIN9_10_CME_CORR_EN_AIN10_MASK		((uint16_t) 0x0080)
    #define BANK2_PGA_CONFIG_AIN9_10_CME_CORR_EN_AIN10_BITOFFSET	(7)
    #define BANK2_PGA_CONFIG_AIN9_10_CME_CORR_EN_AIN10_DISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN9_10_CME_CORR_EN_AIN10_ENABLED	((uint16_t) 0x0080)

    /* CM_RANGE_AIN10 field */
    #define BANK2_PGA_CONFIG_AIN9_10_CM_RANGE_AIN10_MASK		((uint16_t) 0x0070)
    #define BANK2_PGA_CONFIG_AIN9_10_CM_RANGE_AIN10_BITOFFSET	(4)
    #define BANK2_PGA_CONFIG_AIN9_10_CM_RANGE_AIN10_DIFFERENTIAL	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN9_10_CM_RANGE_AIN10_SINGLEENDED	((uint16_t) 0x0050)
    #define BANK2_PGA_CONFIG_AIN9_10_CM_RANGE_AIN10_SINGLEENDEDOPENWIRESAFE	((uint16_t) 0x0060)

    /* INPUT_RANGE_AIN10 field */
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN10_MASK		((uint16_t) 0x0007)
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN10_BITOFFSET	(0)
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN10_5V		((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN10_RESERVED0	((uint16_t) 0x0001)
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN10_2		((uint16_t) 0x0002)
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN10_6		((uint16_t) 0x0003)
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN10_10V		((uint16_t) 0x0004)
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN10_12		((uint16_t) 0x0005)
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN10_RESERVED1	((uint16_t) 0x0006)
    #define BANK2_PGA_CONFIG_AIN9_10_INPUT_RANGE_AIN10_RESERVED2	((uint16_t) 0x0007)


/* Register 0x0C (PGA_BW_SEL_AIN9_16) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |          PGA_BW_SEL_AIN9[1:0]         |         PGA_BW_SEL_AIN10[1:0]         |         PGA_BW_SEL_AIN11[1:0]         |         PGA_BW_SEL_AIN12[1:0]         |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         PGA_BW_SEL_AIN13[1:0]         |         PGA_BW_SEL_AIN14[1:0]         |         PGA_BW_SEL_AIN15[1:0]         |         PGA_BW_SEL_AIN16[1:0]         |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PGA_BW_SEL_AIN9_16 register */
    #define BANK2_PGA_BW_SEL_AIN9_16_ADDRESS					((uint8_t) 0x0C)
    #define BANK2_PGA_BW_SEL_AIN9_16_DEFAULT					((uint16_t) 0x0000)

    /* PGA_BW_SEL_AIN9 field */
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN9_MASK		((uint16_t) 0xC000)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN9_BITOFFSET	(14)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN9_LOWBANDWIDTH	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN9_WIDEBANDWIDTH	((uint16_t) 0x4000)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN9_RESERVED0	((uint16_t) 0x8000)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN9_RESERVED1	((uint16_t) 0xC000)

    /* PGA_BW_SEL_AIN10 field */
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN10_MASK		((uint16_t) 0x3000)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN10_BITOFFSET	(12)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN10_LOWBANDWIDTH	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN10_WIDEBANDWIDTH	((uint16_t) 0x1000)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN10_RESERVED0	((uint16_t) 0x2000)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN10_RESERVED1	((uint16_t) 0x3000)

    /* PGA_BW_SEL_AIN11 field */
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN11_MASK		((uint16_t) 0x0C00)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN11_BITOFFSET	(10)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN11_LOWBANDWIDTH	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN11_WIDEBANDWIDTH	((uint16_t) 0x0400)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN11_RESERVED0	((uint16_t) 0x0800)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN11_RESERVED1	((uint16_t) 0x0C00)

    /* PGA_BW_SEL_AIN12 field */
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN12_MASK		((uint16_t) 0x0300)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN12_BITOFFSET	(8)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN12_LOWBANDWIDTH	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN12_WIDEBANDWIDTH	((uint16_t) 0x0100)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN12_RESERVED0	((uint16_t) 0x0200)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN12_RESERVED1	((uint16_t) 0x0300)

    /* PGA_BW_SEL_AIN13 field */
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN13_MASK		((uint16_t) 0x00C0)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN13_BITOFFSET	(6)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN13_LOWBANDWIDTH	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN13_WIDEBANDWIDTH	((uint16_t) 0x0040)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN13_RESERVED0	((uint16_t) 0x0080)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN13_RESERVED1	((uint16_t) 0x00C0)

    /* PGA_BW_SEL_AIN14 field */
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN14_MASK		((uint16_t) 0x0030)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN14_BITOFFSET	(4)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN14_LOWBANDWIDTH	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN14_WIDEBANDWIDTH	((uint16_t) 0x0010)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN14_RESERVED0	((uint16_t) 0x0020)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN14_RESERVED1	((uint16_t) 0x0030)

    /* PGA_BW_SEL_AIN15 field */
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN15_MASK		((uint16_t) 0x000C)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN15_BITOFFSET	(2)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN15_LOWBANDWIDTH	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN15_WIDEBANDWIDTH	((uint16_t) 0x0004)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN15_RESERVED0	((uint16_t) 0x0008)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN15_RESERVED1	((uint16_t) 0x000C)

    /* PGA_BW_SEL_AIN16 field */
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN16_MASK		((uint16_t) 0x0003)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN16_BITOFFSET	(0)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN16_LOWBANDWIDTH	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN16_WIDEBANDWIDTH	((uint16_t) 0x0001)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN16_RESERVED0	((uint16_t) 0x0002)
    #define BANK2_PGA_BW_SEL_AIN9_16_PGA_BW_SEL_AIN16_RESERVED1	((uint16_t) 0x0003)


/* Register 0x0D (PHASE_DELAY_AIN15_16) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     PHASE_DELAY_AIN15[7:0]                                                                    |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                     PHASE_DELAY_AIN16[7:0]                                                                    |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PHASE_DELAY_AIN15_16 register */
    #define BANK2_PHASE_DELAY_AIN15_16_ADDRESS					((uint8_t) 0x0D)
    #define BANK2_PHASE_DELAY_AIN15_16_DEFAULT					((uint16_t) 0x0000)

    /* PHASE_DELAY_AIN15 field */
    #define BANK2_PHASE_DELAY_AIN15_16_PHASE_DELAY_AIN15_MASK	((uint16_t) 0xFF00)
    #define BANK2_PHASE_DELAY_AIN15_16_PHASE_DELAY_AIN15_BITOFFSET	(8)

    /* PHASE_DELAY_AIN16 field */
    #define BANK2_PHASE_DELAY_AIN15_16_PHASE_DELAY_AIN16_MASK	((uint16_t) 0x00FF)
    #define BANK2_PHASE_DELAY_AIN15_16_PHASE_DELAY_AIN16_BITOFFSET	(0)


/* Register 0x0E (PHASE_DELAY_AIN13_14) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     PHASE_DELAY_AIN13[7:0]                                                                    |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                     PHASE_DELAY_AIN14[7:0]                                                                    |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PHASE_DELAY_AIN13_14 register */
    #define BANK2_PHASE_DELAY_AIN13_14_ADDRESS					((uint8_t) 0x0E)
    #define BANK2_PHASE_DELAY_AIN13_14_DEFAULT					((uint16_t) 0x0000)

    /* PHASE_DELAY_AIN13 field */
    #define BANK2_PHASE_DELAY_AIN13_14_PHASE_DELAY_AIN13_MASK	((uint16_t) 0xFF00)
    #define BANK2_PHASE_DELAY_AIN13_14_PHASE_DELAY_AIN13_BITOFFSET	(8)

    /* PHASE_DELAY_AIN14 field */
    #define BANK2_PHASE_DELAY_AIN13_14_PHASE_DELAY_AIN14_MASK	((uint16_t) 0x00FF)
    #define BANK2_PHASE_DELAY_AIN13_14_PHASE_DELAY_AIN14_BITOFFSET	(0)


/* Register 0x0F (PHASE_DELAY_AIN11_12) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     PHASE_DELAY_AIN11[7:0]                                                                    |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                     PHASE_DELAY_AIN12[7:0]                                                                    |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PHASE_DELAY_AIN11_12 register */
    #define BANK2_PHASE_DELAY_AIN11_12_ADDRESS					((uint8_t) 0x0F)
    #define BANK2_PHASE_DELAY_AIN11_12_DEFAULT					((uint16_t) 0x0000)

    /* PHASE_DELAY_AIN11 field */
    #define BANK2_PHASE_DELAY_AIN11_12_PHASE_DELAY_AIN11_MASK	((uint16_t) 0xFF00)
    #define BANK2_PHASE_DELAY_AIN11_12_PHASE_DELAY_AIN11_BITOFFSET	(8)

    /* PHASE_DELAY_AIN12 field */
    #define BANK2_PHASE_DELAY_AIN11_12_PHASE_DELAY_AIN12_MASK	((uint16_t) 0x00FF)
    #define BANK2_PHASE_DELAY_AIN11_12_PHASE_DELAY_AIN12_BITOFFSET	(0)


/* Register 0x10 (PHASE_DELAY_AIN9_10) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     PHASE_DELAY_AIN9[7:0]                                                                     |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                     PHASE_DELAY_AIN10[7:0]                                                                    |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PHASE_DELAY_AIN9_10 register */
    #define BANK2_PHASE_DELAY_AIN9_10_ADDRESS					((uint8_t) 0x10)
    #define BANK2_PHASE_DELAY_AIN9_10_DEFAULT					((uint16_t) 0x0000)

    /* PHASE_DELAY_AIN9 field */
    #define BANK2_PHASE_DELAY_AIN9_10_PHASE_DELAY_AIN9_MASK		((uint16_t) 0xFF00)
    #define BANK2_PHASE_DELAY_AIN9_10_PHASE_DELAY_AIN9_BITOFFSET	(8)

    /* PHASE_DELAY_AIN10 field */
    #define BANK2_PHASE_DELAY_AIN9_10_PHASE_DELAY_AIN10_MASK	((uint16_t) 0x00FF)
    #define BANK2_PHASE_DELAY_AIN9_10_PHASE_DELAY_AIN10_BITOFFSET	(0)


/* Register 0x11 (OFS_AIN16) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                     RESERVED[5:0]                                                     |             OFS_AIN16[9:8]             
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           OFS_AIN16[7:0]                                                                         
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFS_AIN16 register */
    #define BANK2_OFS_AIN16_ADDRESS								((uint8_t) 0x11)
    #define BANK2_OFS_AIN16_DEFAULT								((uint16_t) 0x0000)

    /* OFS_AIN16 field */
    #define BANK2_OFS_AIN16_MASK								((uint16_t) 0x03FF)
    #define BANK2_OFS_AIN16_BITOFFSET							(0)


/* Register 0x12 (OFS_AIN15) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                     RESERVED[5:0]                                                     |             OFS_AIN15[9:8]             
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           OFS_AIN15[7:0]                                                                         
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFS_AIN15 register */
    #define BANK2_OFS_AIN15_ADDRESS								((uint8_t) 0x12)
    #define BANK2_OFS_AIN15_DEFAULT								((uint16_t) 0x0000)

    /* OFS_AIN15 field */
    #define BANK2_OFS_AIN15_MASK								((uint16_t) 0x03FF)
    #define BANK2_OFS_AIN15_BITOFFSET							(0)


/* Register 0x13 (OFS_AIN14) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                     RESERVED[5:0]                                                     |             OFS_AIN14[9:8]             
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           OFS_AIN14[7:0]                                                                         
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFS_AIN14 register */
    #define BANK2_OFS_AIN14_ADDRESS								((uint8_t) 0x13)
    #define BANK2_OFS_AIN14_DEFAULT								((uint16_t) 0x0000)

    /* OFS_AIN14 field */
    #define BANK2_OFS_AIN14_MASK								((uint16_t) 0x03FF)
    #define BANK2_OFS_AIN14_BITOFFSET							(0)


/* Register 0x14 (OFS_AIN13) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                     RESERVED[5:0]                                                     |             OFS_AIN13[9:8]             
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           OFS_AIN13[7:0]                                                                         
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFS_AIN13 register */
    #define BANK2_OFS_AIN13_ADDRESS								((uint8_t) 0x14)
    #define BANK2_OFS_AIN13_DEFAULT								((uint16_t) 0x0000)

    /* OFS_AIN13 field */
    #define BANK2_OFS_AIN13_MASK								((uint16_t) 0x03FF)
    #define BANK2_OFS_AIN13_BITOFFSET							(0)


/* Register 0x15 (OFS_AIN12) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                     RESERVED[5:0]                                                     |             OFS_AIN12[9:8]             
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           OFS_AIN12[7:0]                                                                         
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFS_AIN12 register */
    #define BANK2_OFS_AIN12_ADDRESS								((uint8_t) 0x15)
    #define BANK2_OFS_AIN12_DEFAULT								((uint16_t) 0x0000)

    /* OFS_AIN12 field */
    #define BANK2_OFS_AIN12_MASK								((uint16_t) 0x03FF)
    #define BANK2_OFS_AIN12_BITOFFSET							(0)


/* Register 0x16 (OFS_AIN11) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                     RESERVED[5:0]                                                     |             OFS_AIN11[9:8]             
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           OFS_AIN11[7:0]                                                                         
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFS_AIN11 register */
    #define BANK2_OFS_AIN11_ADDRESS								((uint8_t) 0x16)
    #define BANK2_OFS_AIN11_DEFAULT								((uint16_t) 0x0000)

    /* OFS_AIN11 field */
    #define BANK2_OFS_AIN11_MASK								((uint16_t) 0x03FF)
    #define BANK2_OFS_AIN11_BITOFFSET							(0)


/* Register 0x17 (OFS_AIN10) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                     RESERVED[5:0]                                                     |             OFS_AIN10[9:8]             
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           OFS_AIN10[7:0]                                                                         
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFS_AIN10 register */
    #define BANK2_OFS_AIN10_ADDRESS								((uint8_t) 0x17)
    #define BANK2_OFS_AIN10_DEFAULT								((uint16_t) 0x0000)

    /* OFS_AIN10 field */
    #define BANK2_OFS_AIN10_MASK								((uint16_t) 0x03FF)
    #define BANK2_OFS_AIN10_BITOFFSET							(0)


/* Register 0x18 (OFS_AIN9) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                     RESERVED[5:0]                                                     |             OFS_AIN9[9:8]              
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           OFS_AIN9[7:0]                                                                          
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFS_AIN9 register */
    #define BANK2_OFS_AIN9_ADDRESS								((uint8_t) 0x18)
    #define BANK2_OFS_AIN9_DEFAULT								((uint16_t) 0x0000)

    /* OFS_AIN9 field */
    #define BANK2_OFS_AIN9_MASK									((uint16_t) 0x03FF)
    #define BANK2_OFS_AIN9_BITOFFSET							(0)


/* Register 0x19 (GAN_AIN16) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |             RESERVED[1:0]             |                                                    GAN_AIN16[13:8]                                                     
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           GAN_AIN16[7:0]                                                                         
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAN_AIN16 register */
    #define BANK2_GAN_AIN16_ADDRESS								((uint8_t) 0x19)
    #define BANK2_GAN_AIN16_DEFAULT								((uint16_t) 0x0000)

    /* GAN_AIN16 field */
    #define BANK2_GAN_AIN16_MASK								((uint16_t) 0x3FFF)
    #define BANK2_GAN_AIN16_BITOFFSET							(0)


/* Register 0x1A (GAN_AIN15) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |             RESERVED[1:0]             |                                                    GAN_AIN15[13:8]                                                     
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           GAN_AIN15[7:0]                                                                         
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAN_AIN15 register */
    #define BANK2_GAN_AIN15_ADDRESS								((uint8_t) 0x1A)
    #define BANK2_GAN_AIN15_DEFAULT								((uint16_t) 0x0000)

    /* GAN_AIN15 field */
    #define BANK2_GAN_AIN15_MASK								((uint16_t) 0x3FFF)
    #define BANK2_GAN_AIN15_BITOFFSET							(0)


/* Register 0x1B (GAN_AIN14) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |             RESERVED[1:0]             |                                                    GAN_AIN14[13:8]                                                     
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           GAN_AIN14[7:0]                                                                         
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAN_AIN14 register */
    #define BANK2_GAN_AIN14_ADDRESS								((uint8_t) 0x1B)
    #define BANK2_GAN_AIN14_DEFAULT								((uint16_t) 0x0000)

    /* GAN_AIN14 field */
    #define BANK2_GAN_AIN14_MASK								((uint16_t) 0x3FFF)
    #define BANK2_GAN_AIN14_BITOFFSET							(0)


/* Register 0x1C (GAN_AIN13) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |             RESERVED[1:0]             |                                                    GAN_AIN13[13:8]                                                     
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           GAN_AIN13[7:0]                                                                         
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAN_AIN13 register */
    #define BANK2_GAN_AIN13_ADDRESS								((uint8_t) 0x1C)
    #define BANK2_GAN_AIN13_DEFAULT								((uint16_t) 0x0000)

    /* GAN_AIN13 field */
    #define BANK2_GAN_AIN13_MASK								((uint16_t) 0x3FFF)
    #define BANK2_GAN_AIN13_BITOFFSET							(0)


/* Register 0x1D (GAN_AIN12) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |             RESERVED[1:0]             |                                                    GAN_AIN12[13:8]                                                     
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           GAN_AIN12[7:0]                                                                         
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAN_AIN12 register */
    #define BANK2_GAN_AIN12_ADDRESS								((uint8_t) 0x1D)
    #define BANK2_GAN_AIN12_DEFAULT								((uint16_t) 0x0000)

    /* GAN_AIN12 field */
    #define BANK2_GAN_AIN12_MASK								((uint16_t) 0x3FFF)
    #define BANK2_GAN_AIN12_BITOFFSET							(0)


/* Register 0x1E (GAN_AIN11) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |             RESERVED[1:0]             |                                                    GAN_AIN11[13:8]                                                     
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           GAN_AIN11[7:0]                                                                         
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAN_AIN11 register */
    #define BANK2_GAN_AIN11_ADDRESS								((uint8_t) 0x1E)
    #define BANK2_GAN_AIN11_DEFAULT								((uint16_t) 0x0000)

    /* GAN_AIN11 field */
    #define BANK2_GAN_AIN11_MASK								((uint16_t) 0x3FFF)
    #define BANK2_GAN_AIN11_BITOFFSET							(0)


/* Register 0x1F (GAN_AIN10) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |             RESERVED[1:0]             |                                                    GAN_AIN10[13:8]                                                     
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           GAN_AIN10[7:0]                                                                         
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAN_AIN10 register */
    #define BANK2_GAN_AIN10_ADDRESS								((uint8_t) 0x1F)
    #define BANK2_GAN_AIN10_DEFAULT								((uint16_t) 0x0000)

    /* GAN_AIN10 field */
    #define BANK2_GAN_AIN10_MASK								((uint16_t) 0x3FFF)
    #define BANK2_GAN_AIN10_BITOFFSET							(0)


/* Register 0x20 (GAN_AIN9) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |             RESERVED[1:0]             |                                                     GAN_AIN9[13:8]                                                     
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           GAN_AIN9[7:0]                                                                          
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAN_AIN9 register */
    #define BANK2_GAN_AIN9_ADDRESS								((uint8_t) 0x20)
    #define BANK2_GAN_AIN9_DEFAULT								((uint16_t) 0x0000)

    /* GAN_AIN9 field */
    #define BANK2_GAN_AIN9_MASK									((uint16_t) 0x3FFF)
    #define BANK2_GAN_AIN9_BITOFFSET							(0)


/* Register 0x21 (DWC_CFG) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |    DWC_STAT_RST   |                       RESERVED[2:0]                       |                              DWC_GLITCH_FILT[3:0]                             |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |    DWC_EN_AIN9    |    DWC_EN_AIN10   |    DWC_EN_AIN11   |    DWC_EN_AIN12   |    DWC_EN_AIN13   |    DWC_EN_AIN14   |    DWC_EN_AIN15   |    DWC_EN_AIN16   |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_CFG register */
    #define BANK2_DWC_CFG_ADDRESS								((uint8_t) 0x21)
    #define BANK2_DWC_CFG_DEFAULT								((uint16_t) 0x0000)

    /* DWC_STAT_RST field */
    #define BANK2_DWC_CFG_DWC_STAT_RST_MASK						((uint16_t) 0x8000)
    #define BANK2_DWC_CFG_DWC_STAT_RST_BITOFFSET				(15)

    /* DWC_GLITCH_FILT field */
    #define BANK2_DWC_CFG_DWC_GLITCH_FILT_MASK					((uint16_t) 0x0F00)
    #define BANK2_DWC_CFG_DWC_GLITCH_FILT_BITOFFSET				(8)

    /* DWC_EN_AIN9 field */
    #define BANK2_DWC_CFG_DWC_EN_AIN9_MASK						((uint16_t) 0x0080)
    #define BANK2_DWC_CFG_DWC_EN_AIN9_BITOFFSET					(7)
    #define BANK2_DWC_CFG_DWC_EN_AIN9_DISABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_DWC_CFG_DWC_EN_AIN9_ENABLED					((uint16_t) 0x0080)

    /* DWC_EN_AIN10 field */
    #define BANK2_DWC_CFG_DWC_EN_AIN10_MASK						((uint16_t) 0x0040)
    #define BANK2_DWC_CFG_DWC_EN_AIN10_BITOFFSET				(6)
    #define BANK2_DWC_CFG_DWC_EN_AIN10_DISABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_DWC_CFG_DWC_EN_AIN10_ENABLED					((uint16_t) 0x0040)

    /* DWC_EN_AIN11 field */
    #define BANK2_DWC_CFG_DWC_EN_AIN11_MASK						((uint16_t) 0x0020)
    #define BANK2_DWC_CFG_DWC_EN_AIN11_BITOFFSET				(5)
    #define BANK2_DWC_CFG_DWC_EN_AIN11_DISABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_DWC_CFG_DWC_EN_AIN11_ENABLED					((uint16_t) 0x0020)

    /* DWC_EN_AIN12 field */
    #define BANK2_DWC_CFG_DWC_EN_AIN12_MASK						((uint16_t) 0x0010)
    #define BANK2_DWC_CFG_DWC_EN_AIN12_BITOFFSET				(4)
    #define BANK2_DWC_CFG_DWC_EN_AIN12_DISABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_DWC_CFG_DWC_EN_AIN12_ENABLED					((uint16_t) 0x0010)

    /* DWC_EN_AIN13 field */
    #define BANK2_DWC_CFG_DWC_EN_AIN13_MASK						((uint16_t) 0x0008)
    #define BANK2_DWC_CFG_DWC_EN_AIN13_BITOFFSET				(3)
    #define BANK2_DWC_CFG_DWC_EN_AIN13_DISABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_DWC_CFG_DWC_EN_AIN13_ENABLED					((uint16_t) 0x0008)

    /* DWC_EN_AIN14 field */
    #define BANK2_DWC_CFG_DWC_EN_AIN14_MASK						((uint16_t) 0x0004)
    #define BANK2_DWC_CFG_DWC_EN_AIN14_BITOFFSET				(2)
    #define BANK2_DWC_CFG_DWC_EN_AIN14_DISABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_DWC_CFG_DWC_EN_AIN14_ENABLED					((uint16_t) 0x0004)

    /* DWC_EN_AIN15 field */
    #define BANK2_DWC_CFG_DWC_EN_AIN15_MASK						((uint16_t) 0x0002)
    #define BANK2_DWC_CFG_DWC_EN_AIN15_BITOFFSET				(1)
    #define BANK2_DWC_CFG_DWC_EN_AIN15_DISABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_DWC_CFG_DWC_EN_AIN15_ENABLED					((uint16_t) 0x0002)

    /* DWC_EN_AIN16 field */
    #define BANK2_DWC_CFG_DWC_EN_AIN16_MASK						((uint16_t) 0x0001)
    #define BANK2_DWC_CFG_DWC_EN_AIN16_BITOFFSET				(0)
    #define BANK2_DWC_CFG_DWC_EN_AIN16_DISABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_DWC_CFG_DWC_EN_AIN16_ENABLED					((uint16_t) 0x0001)


/* Register 0x22 (DWC_TH_AIN16) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                       HIGH_TH_AIN16[7:0]                                                                      |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                       LOW_TH_AIN16[7:0]                                                                       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_TH_AIN16 register */
    #define BANK2_DWC_TH_AIN16_ADDRESS							((uint8_t) 0x22)
    #define BANK2_DWC_TH_AIN16_DEFAULT							((uint16_t) 0xFF00)

    /* HIGH_TH_AIN16 field */
    #define BANK2_DWC_TH_AIN16_HIGH_TH_AIN16_MASK				((uint16_t) 0xFF00)
    #define BANK2_DWC_TH_AIN16_HIGH_TH_AIN16_BITOFFSET			(8)

    /* LOW_TH_AIN16 field */
    #define BANK2_DWC_TH_AIN16_LOW_TH_AIN16_MASK				((uint16_t) 0x00FF)
    #define BANK2_DWC_TH_AIN16_LOW_TH_AIN16_BITOFFSET			(0)


/* Register 0x23 (DWC_TH_AIN15) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                       HIGH_TH_AIN15[7:0]                                                                      |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                       LOW_TH_AIN15[7:0]                                                                       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_TH_AIN15 register */
    #define BANK2_DWC_TH_AIN15_ADDRESS							((uint8_t) 0x23)
    #define BANK2_DWC_TH_AIN15_DEFAULT							((uint16_t) 0xFF00)

    /* HIGH_TH_AIN15 field */
    #define BANK2_DWC_TH_AIN15_HIGH_TH_AIN15_MASK				((uint16_t) 0xFF00)
    #define BANK2_DWC_TH_AIN15_HIGH_TH_AIN15_BITOFFSET			(8)

    /* LOW_TH_AIN15 field */
    #define BANK2_DWC_TH_AIN15_LOW_TH_AIN15_MASK				((uint16_t) 0x00FF)
    #define BANK2_DWC_TH_AIN15_LOW_TH_AIN15_BITOFFSET			(0)


/* Register 0x24 (DWC_TH_AIN14) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                       HIGH_TH_AIN14[7:0]                                                                      |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                       LOW_TH_AIN14[7:0]                                                                       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_TH_AIN14 register */
    #define BANK2_DWC_TH_AIN14_ADDRESS							((uint8_t) 0x24)
    #define BANK2_DWC_TH_AIN14_DEFAULT							((uint16_t) 0xFF00)

    /* HIGH_TH_AIN14 field */
    #define BANK2_DWC_TH_AIN14_HIGH_TH_AIN14_MASK				((uint16_t) 0xFF00)
    #define BANK2_DWC_TH_AIN14_HIGH_TH_AIN14_BITOFFSET			(8)

    /* LOW_TH_AIN14 field */
    #define BANK2_DWC_TH_AIN14_LOW_TH_AIN14_MASK				((uint16_t) 0x00FF)
    #define BANK2_DWC_TH_AIN14_LOW_TH_AIN14_BITOFFSET			(0)


/* Register 0x25 (DWC_TH_AIN13) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                       HIGH_TH_AIN13[7:0]                                                                      |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                       LOW_TH_AIN13[7:0]                                                                       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_TH_AIN13 register */
    #define BANK2_DWC_TH_AIN13_ADDRESS							((uint8_t) 0x25)
    #define BANK2_DWC_TH_AIN13_DEFAULT							((uint16_t) 0xFF00)

    /* HIGH_TH_AIN13 field */
    #define BANK2_DWC_TH_AIN13_HIGH_TH_AIN13_MASK				((uint16_t) 0xFF00)
    #define BANK2_DWC_TH_AIN13_HIGH_TH_AIN13_BITOFFSET			(8)

    /* LOW_TH_AIN13 field */
    #define BANK2_DWC_TH_AIN13_LOW_TH_AIN13_MASK				((uint16_t) 0x00FF)
    #define BANK2_DWC_TH_AIN13_LOW_TH_AIN13_BITOFFSET			(0)


/* Register 0x26 (DWC_TH_AIN12) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                       HIGH_TH_AIN12[7:0]                                                                      |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                       LOW_TH_AIN12[7:0]                                                                       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_TH_AIN12 register */
    #define BANK2_DWC_TH_AIN12_ADDRESS							((uint8_t) 0x26)
    #define BANK2_DWC_TH_AIN12_DEFAULT							((uint16_t) 0xFF00)

    /* HIGH_TH_AIN12 field */
    #define BANK2_DWC_TH_AIN12_HIGH_TH_AIN12_MASK				((uint16_t) 0xFF00)
    #define BANK2_DWC_TH_AIN12_HIGH_TH_AIN12_BITOFFSET			(8)

    /* LOW_TH_AIN12 field */
    #define BANK2_DWC_TH_AIN12_LOW_TH_AIN12_MASK				((uint16_t) 0x00FF)
    #define BANK2_DWC_TH_AIN12_LOW_TH_AIN12_BITOFFSET			(0)


/* Register 0x27 (DWC_TH_AIN11) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                       HIGH_TH_AIN11[7:0]                                                                      |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                       LOW_TH_AIN11[7:0]                                                                       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_TH_AIN11 register */
    #define BANK2_DWC_TH_AIN11_ADDRESS							((uint8_t) 0x27)
    #define BANK2_DWC_TH_AIN11_DEFAULT							((uint16_t) 0xFF00)

    /* HIGH_TH_AIN11 field */
    #define BANK2_DWC_TH_AIN11_HIGH_TH_AIN11_MASK				((uint16_t) 0xFF00)
    #define BANK2_DWC_TH_AIN11_HIGH_TH_AIN11_BITOFFSET			(8)

    /* LOW_TH_AIN11 field */
    #define BANK2_DWC_TH_AIN11_LOW_TH_AIN11_MASK				((uint16_t) 0x00FF)
    #define BANK2_DWC_TH_AIN11_LOW_TH_AIN11_BITOFFSET			(0)


/* Register 0x28 (DWC_TH_AIN10) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                       HIGH_TH_AIN10[7:0]                                                                      |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                       LOW_TH_AIN10[7:0]                                                                       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_TH_AIN10 register */
    #define BANK2_DWC_TH_AIN10_ADDRESS							((uint8_t) 0x28)
    #define BANK2_DWC_TH_AIN10_DEFAULT							((uint16_t) 0xFF00)

    /* HIGH_TH_AIN10 field */
    #define BANK2_DWC_TH_AIN10_HIGH_TH_AIN10_MASK				((uint16_t) 0xFF00)
    #define BANK2_DWC_TH_AIN10_HIGH_TH_AIN10_BITOFFSET			(8)

    /* LOW_TH_AIN10 field */
    #define BANK2_DWC_TH_AIN10_LOW_TH_AIN10_MASK				((uint16_t) 0x00FF)
    #define BANK2_DWC_TH_AIN10_LOW_TH_AIN10_BITOFFSET			(0)


/* Register 0x29 (DWC_TH_AIN9) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                       HIGH_TH_AIN9[7:0]                                                                       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                        LOW_TH_AIN9[7:0]                                                                       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_TH_AIN9 register */
    #define BANK2_DWC_TH_AIN9_ADDRESS							((uint8_t) 0x29)
    #define BANK2_DWC_TH_AIN9_DEFAULT							((uint16_t) 0xFF00)

    /* HIGH_TH_AIN9 field */
    #define BANK2_DWC_TH_AIN9_HIGH_TH_AIN9_MASK					((uint16_t) 0xFF00)
    #define BANK2_DWC_TH_AIN9_HIGH_TH_AIN9_BITOFFSET			(8)

    /* LOW_TH_AIN9 field */
    #define BANK2_DWC_TH_AIN9_LOW_TH_AIN9_MASK					((uint16_t) 0x00FF)
    #define BANK2_DWC_TH_AIN9_LOW_TH_AIN9_BITOFFSET				(0)


/* Register 0x2A (DWC_HYS_AIN15_16) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                         HYS_AIN15[7:0]                                                                        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                         HYS_AIN16[7:0]                                                                        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_HYS_AIN15_16 register */
    #define BANK2_DWC_HYS_AIN15_16_ADDRESS						((uint8_t) 0x2A)
    #define BANK2_DWC_HYS_AIN15_16_DEFAULT						((uint16_t) 0x0000)

    /* HYS_AIN15 field */
    #define BANK2_DWC_HYS_AIN15_16_HYS_AIN15_MASK				((uint16_t) 0xFF00)
    #define BANK2_DWC_HYS_AIN15_16_HYS_AIN15_BITOFFSET			(8)

    /* HYS_AIN16 field */
    #define BANK2_DWC_HYS_AIN15_16_HYS_AIN16_MASK				((uint16_t) 0x00FF)
    #define BANK2_DWC_HYS_AIN15_16_HYS_AIN16_BITOFFSET			(0)


/* Register 0x2B (DWC_HYS_AIN13_14) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                         HYS_AIN13[7:0]                                                                        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                         HYS_AIN14[7:0]                                                                        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_HYS_AIN13_14 register */
    #define BANK2_DWC_HYS_AIN13_14_ADDRESS						((uint8_t) 0x2B)
    #define BANK2_DWC_HYS_AIN13_14_DEFAULT						((uint16_t) 0xFF00)

    /* HYS_AIN13 field */
    #define BANK2_DWC_HYS_AIN13_14_HYS_AIN13_MASK				((uint16_t) 0xFF00)
    #define BANK2_DWC_HYS_AIN13_14_HYS_AIN13_BITOFFSET			(8)

    /* HYS_AIN14 field */
    #define BANK2_DWC_HYS_AIN13_14_HYS_AIN14_MASK				((uint16_t) 0x00FF)
    #define BANK2_DWC_HYS_AIN13_14_HYS_AIN14_BITOFFSET			(0)


/* Register 0x2C (DWC_HYS_AIN11_12) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                         HYS_AIN11[7:0]                                                                        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                         HYS_AIN12[7:0]                                                                        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_HYS_AIN11_12 register */
    #define BANK2_DWC_HYS_AIN11_12_ADDRESS						((uint8_t) 0x2C)
    #define BANK2_DWC_HYS_AIN11_12_DEFAULT						((uint16_t) 0xFF00)

    /* HYS_AIN11 field */
    #define BANK2_DWC_HYS_AIN11_12_HYS_AIN11_MASK				((uint16_t) 0xFF00)
    #define BANK2_DWC_HYS_AIN11_12_HYS_AIN11_BITOFFSET			(8)

    /* HYS_AIN12 field */
    #define BANK2_DWC_HYS_AIN11_12_HYS_AIN12_MASK				((uint16_t) 0x00FF)
    #define BANK2_DWC_HYS_AIN11_12_HYS_AIN12_BITOFFSET			(0)


/* Register 0x2D (DWC_HYS_AIN9_10) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                         HYS_AIN9[7:0]                                                                         |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                         HYS_AIN10[7:0]                                                                        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_HYS_AIN9_10 register */
    #define BANK2_DWC_HYS_AIN9_10_ADDRESS						((uint8_t) 0x2D)
    #define BANK2_DWC_HYS_AIN9_10_DEFAULT						((uint16_t) 0xFF00)

    /* HYS_AIN9 field */
    #define BANK2_DWC_HYS_AIN9_10_HYS_AIN9_MASK					((uint16_t) 0xFF00)
    #define BANK2_DWC_HYS_AIN9_10_HYS_AIN9_BITOFFSET			(8)

    /* HYS_AIN10 field */
    #define BANK2_DWC_HYS_AIN9_10_HYS_AIN10_MASK				((uint16_t) 0x00FF)
    #define BANK2_DWC_HYS_AIN9_10_HYS_AIN10_BITOFFSET			(0)


/* Register 0x2E (TP_CFG) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                         RESERVED[8:1]                                                                          
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *     RESERVED[0:0]   |                        TP_MODE[2:0]                       |      RESERVED     |     TP_DIS_IDX    |    TP_UPD_MODE    |       TP_EN       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_CFG register */
    #define BANK2_TP_CFG_ADDRESS								((uint8_t) 0x2E)
    #define BANK2_TP_CFG_DEFAULT								((uint16_t) 0x0000)

    /* TP_MODE field */
    #define BANK2_TP_CFG_TP_MODE_MASK							((uint16_t) 0x0070)
    #define BANK2_TP_CFG_TP_MODE_BITOFFSET						(4)
    #define BANK2_TP_CFG_TP_MODE_CONSTANTPATTERN				((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_TP_CFG_TP_MODE_RESERVED0						((uint16_t) 0x0010)
    #define BANK2_TP_CFG_TP_MODE_RAMPPATTERN					((uint16_t) 0x0020)
    #define BANK2_TP_CFG_TP_MODE_RESERVED1						((uint16_t) 0x0030)
    #define BANK2_TP_CFG_TP_MODE_RESERVED2						((uint16_t) 0x0040)
    #define BANK2_TP_CFG_TP_MODE_RESERVED3						((uint16_t) 0x0050)

    /* TP_DIS_IDX field */
    #define BANK2_TP_CFG_TP_DIS_IDX_MASK						((uint16_t) 0x0004)
    #define BANK2_TP_CFG_TP_DIS_IDX_BITOFFSET					(2)

    /* TP_UPD_MODE field */
    #define BANK2_TP_CFG_TP_UPD_MODE_MASK						((uint16_t) 0x0002)
    #define BANK2_TP_CFG_TP_UPD_MODE_BITOFFSET					(1)
    #define BANK2_TP_CFG_TP_UPD_MODE_INCREMENTHAPPENSATCHANNELFRAMEBOUNDARY	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_TP_CFG_TP_UPD_MODE_INCREMENTHAPPENSATEVERYCONVST	((uint16_t) 0x0002)

    /* TP_EN field */
    #define BANK2_TP_CFG_TP_EN_MASK								((uint16_t) 0x0001)
    #define BANK2_TP_CFG_TP_EN_BITOFFSET						(0)
    #define BANK2_TP_CFG_TP_EN_ADCCONVERSIONRESULTISLAUNCHEDONTHEDATAINTERFACE	((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_TP_CFG_TP_EN_DIGITALTESTPATTERNISLAUNCHEDONTHEDATAINTERFACE	((uint16_t) 0x0001)


/* Register 0x2F (TP_AIN16) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                         TP_AIN16[15:8]                                                                         
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           TP_AIN16[7:0]                                                                          
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_AIN16 register */
    #define BANK2_TP_AIN16_ADDRESS								((uint8_t) 0x2F)
    #define BANK2_TP_AIN16_DEFAULT								((uint16_t) 0x0000)

    /* TP_AIN16 field */
    #define BANK2_TP_AIN16_MASK									((uint16_t) 0xFFFF)
    #define BANK2_TP_AIN16_BITOFFSET							(0)


/* Register 0x30 (TP_AIN15) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                         TP_AIN15[15:8]                                                                         
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           TP_AIN15[7:0]                                                                          
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_AIN15 register */
    #define BANK2_TP_AIN15_ADDRESS								((uint8_t) 0x30)
    #define BANK2_TP_AIN15_DEFAULT								((uint16_t) 0x0000)

    /* TP_AIN15 field */
    #define BANK2_TP_AIN15_MASK									((uint16_t) 0xFFFF)
    #define BANK2_TP_AIN15_BITOFFSET							(0)


/* Register 0x31 (TP_AIN14) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                         TP_AIN14[15:8]                                                                         
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           TP_AIN14[7:0]                                                                          
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_AIN14 register */
    #define BANK2_TP_AIN14_ADDRESS								((uint8_t) 0x31)
    #define BANK2_TP_AIN14_DEFAULT								((uint16_t) 0x0000)

    /* TP_AIN14 field */
    #define BANK2_TP_AIN14_MASK									((uint16_t) 0xFFFF)
    #define BANK2_TP_AIN14_BITOFFSET							(0)


/* Register 0x32 (TP_AIN13) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                         TP_AIN13[15:8]                                                                         
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           TP_AIN13[7:0]                                                                          
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_AIN13 register */
    #define BANK2_TP_AIN13_ADDRESS								((uint8_t) 0x32)
    #define BANK2_TP_AIN13_DEFAULT								((uint16_t) 0x0000)

    /* TP_AIN13 field */
    #define BANK2_TP_AIN13_MASK									((uint16_t) 0xFFFF)
    #define BANK2_TP_AIN13_BITOFFSET							(0)


/* Register 0x33 (TP_AIN12) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                         TP_AIN12[15:8]                                                                         
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           TP_AIN12[7:0]                                                                          
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_AIN12 register */
    #define BANK2_TP_AIN12_ADDRESS								((uint8_t) 0x33)
    #define BANK2_TP_AIN12_DEFAULT								((uint16_t) 0x0000)

    /* TP_AIN12 field */
    #define BANK2_TP_AIN12_MASK									((uint16_t) 0xFFFF)
    #define BANK2_TP_AIN12_BITOFFSET							(0)


/* Register 0x34 (TP_AIN11) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                         TP_AIN11[15:8]                                                                         
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           TP_AIN11[7:0]                                                                          
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_AIN11 register */
    #define BANK2_TP_AIN11_ADDRESS								((uint8_t) 0x34)
    #define BANK2_TP_AIN11_DEFAULT								((uint16_t) 0x0000)

    /* TP_AIN11 field */
    #define BANK2_TP_AIN11_MASK									((uint16_t) 0xFFFF)
    #define BANK2_TP_AIN11_BITOFFSET							(0)


/* Register 0x35 (TP_AIN10) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                         TP_AIN10[15:8]                                                                         
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                           TP_AIN10[7:0]                                                                          
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_AIN10 register */
    #define BANK2_TP_AIN10_ADDRESS								((uint8_t) 0x35)
    #define BANK2_TP_AIN10_DEFAULT								((uint16_t) 0x0000)

    /* TP_AIN10 field */
    #define BANK2_TP_AIN10_MASK									((uint16_t) 0xFFFF)
    #define BANK2_TP_AIN10_BITOFFSET							(0)


/* Register 0x36 (TP_AIN9) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                         TP_AIN9[15:8]                                                                          
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                            TP_AIN9[7:0]                                                                          
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_AIN9 register */
    #define BANK2_TP_AIN9_ADDRESS								((uint8_t) 0x36)
    #define BANK2_TP_AIN9_DEFAULT								((uint16_t) 0x0000)

    /* TP_AIN9 field */
    #define BANK2_TP_AIN9_MASK									((uint16_t) 0xFFFF)
    #define BANK2_TP_AIN9_BITOFFSET								(0)


/* Register 0x37 (GEN_CFG5) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                         RESERVED[10:3]                                                                         
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                         RESERVED[2:0]                       |      RESERVED     |             RESERVED[1:0]             |    OFS_CORR_DIS   |    GAN_CORR_DIS   |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GEN_CFG5 register */
    #define BANK2_GEN_CFG5_ADDRESS								((uint8_t) 0x37)
    #define BANK2_GEN_CFG5_DEFAULT								((uint16_t) 0x0000)

    /* OFS_CORR_DIS field */
    #define BANK2_GEN_CFG5_OFS_CORR_DIS_MASK					((uint16_t) 0x0002)
    #define BANK2_GEN_CFG5_OFS_CORR_DIS_BITOFFSET				(1)
    #define BANK2_GEN_CFG5_OFS_CORR_DIS_ENABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_GEN_CFG5_OFS_CORR_DIS_DISABLED				((uint16_t) 0x0002)

    /* GAN_CORR_DIS field */
    #define BANK2_GEN_CFG5_GAN_CORR_DIS_MASK					((uint16_t) 0x0001)
    #define BANK2_GEN_CFG5_GAN_CORR_DIS_BITOFFSET				(0)
    #define BANK2_GEN_CFG5_GAN_CORR_DIS_ENABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK2_GEN_CFG5_GAN_CORR_DIS_DISABLED				((uint16_t) 0x0001)


/* Register 0x3E (DWC_FLAG_AIN9_16) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       Bit 15      |       Bit 14      |       Bit 13      |       Bit 12      |       Bit 11      |       Bit 10      |       Bit 9       |       Bit 8       |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   HIGH_FLAG_AIN9  |  HIGH_FLAG_AIN10  |  HIGH_FLAG_AIN11  |  HIGH_FLAG_AIN12  |  HIGH_FLAG_AIN13  |  HIGH_FLAG_AIN14  |  HIGH_FLAG_AIN15  |  HIGH_FLAG_AIN16  |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7       |       Bit 6       |       Bit 5       |       Bit 4       |       Bit 3       |       Bit 2       |       Bit 1       |       Bit 0       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |   LOW_FLAG_AIN9   |   LOW_FLAG_AIN10  |   LOW_FLAG_AIN11  |   LOW_FLAG_AIN12  |   LOW_FLAG_AIN13  |   LOW_FLAG_AIN14  |   LOW_FLAG_AIN15  |   LOW_FLAG_AIN16  |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_FLAG_AIN9_16 register */
    #define BANK2_DWC_FLAG_AIN9_16_ADDRESS						((uint8_t) 0x3E)
    #define BANK2_DWC_FLAG_AIN9_16_DEFAULT						((uint16_t) 0x0000)

    /* HIGH_FLAG_AIN9 field */
    #define BANK2_DWC_FLAG_AIN9_16_HIGH_FLAG_AIN9_MASK			((uint16_t) 0x8000)
    #define BANK2_DWC_FLAG_AIN9_16_HIGH_FLAG_AIN9_BITOFFSET		(15)

    /* HIGH_FLAG_AIN10 field */
    #define BANK2_DWC_FLAG_AIN9_16_HIGH_FLAG_AIN10_MASK			((uint16_t) 0x4000)
    #define BANK2_DWC_FLAG_AIN9_16_HIGH_FLAG_AIN10_BITOFFSET	(14)

    /* HIGH_FLAG_AIN11 field */
    #define BANK2_DWC_FLAG_AIN9_16_HIGH_FLAG_AIN11_MASK			((uint16_t) 0x2000)
    #define BANK2_DWC_FLAG_AIN9_16_HIGH_FLAG_AIN11_BITOFFSET	(13)

    /* HIGH_FLAG_AIN12 field */
    #define BANK2_DWC_FLAG_AIN9_16_HIGH_FLAG_AIN12_MASK			((uint16_t) 0x1000)
    #define BANK2_DWC_FLAG_AIN9_16_HIGH_FLAG_AIN12_BITOFFSET	(12)

    /* HIGH_FLAG_AIN13 field */
    #define BANK2_DWC_FLAG_AIN9_16_HIGH_FLAG_AIN13_MASK			((uint16_t) 0x0800)
    #define BANK2_DWC_FLAG_AIN9_16_HIGH_FLAG_AIN13_BITOFFSET	(11)

    /* HIGH_FLAG_AIN14 field */
    #define BANK2_DWC_FLAG_AIN9_16_HIGH_FLAG_AIN14_MASK			((uint16_t) 0x0400)
    #define BANK2_DWC_FLAG_AIN9_16_HIGH_FLAG_AIN14_BITOFFSET	(10)

    /* HIGH_FLAG_AIN15 field */
    #define BANK2_DWC_FLAG_AIN9_16_HIGH_FLAG_AIN15_MASK			((uint16_t) 0x0200)
    #define BANK2_DWC_FLAG_AIN9_16_HIGH_FLAG_AIN15_BITOFFSET	(9)

    /* HIGH_FLAG_AIN16 field */
    #define BANK2_DWC_FLAG_AIN9_16_HIGH_FLAG_AIN16_MASK			((uint16_t) 0x0100)
    #define BANK2_DWC_FLAG_AIN9_16_HIGH_FLAG_AIN16_BITOFFSET	(8)

    /* LOW_FLAG_AIN9 field */
    #define BANK2_DWC_FLAG_AIN9_16_LOW_FLAG_AIN9_MASK			((uint16_t) 0x0080)
    #define BANK2_DWC_FLAG_AIN9_16_LOW_FLAG_AIN9_BITOFFSET		(7)

    /* LOW_FLAG_AIN10 field */
    #define BANK2_DWC_FLAG_AIN9_16_LOW_FLAG_AIN10_MASK			((uint16_t) 0x0040)
    #define BANK2_DWC_FLAG_AIN9_16_LOW_FLAG_AIN10_BITOFFSET		(6)

    /* LOW_FLAG_AIN11 field */
    #define BANK2_DWC_FLAG_AIN9_16_LOW_FLAG_AIN11_MASK			((uint16_t) 0x0020)
    #define BANK2_DWC_FLAG_AIN9_16_LOW_FLAG_AIN11_BITOFFSET		(5)

    /* LOW_FLAG_AIN12 field */
    #define BANK2_DWC_FLAG_AIN9_16_LOW_FLAG_AIN12_MASK			((uint16_t) 0x0010)
    #define BANK2_DWC_FLAG_AIN9_16_LOW_FLAG_AIN12_BITOFFSET		(4)

    /* LOW_FLAG_AIN13 field */
    #define BANK2_DWC_FLAG_AIN9_16_LOW_FLAG_AIN13_MASK			((uint16_t) 0x0008)
    #define BANK2_DWC_FLAG_AIN9_16_LOW_FLAG_AIN13_BITOFFSET		(3)

    /* LOW_FLAG_AIN14 field */
    #define BANK2_DWC_FLAG_AIN9_16_LOW_FLAG_AIN14_MASK			((uint16_t) 0x0004)
    #define BANK2_DWC_FLAG_AIN9_16_LOW_FLAG_AIN14_BITOFFSET		(2)

    /* LOW_FLAG_AIN15 field */
    #define BANK2_DWC_FLAG_AIN9_16_LOW_FLAG_AIN15_MASK			((uint16_t) 0x0002)
    #define BANK2_DWC_FLAG_AIN9_16_LOW_FLAG_AIN15_BITOFFSET		(1)

    /* LOW_FLAG_AIN16 field */
    #define BANK2_DWC_FLAG_AIN9_16_LOW_FLAG_AIN16_MASK			((uint16_t) 0x0001)
    #define BANK2_DWC_FLAG_AIN9_16_LOW_FLAG_AIN16_BITOFFSET		(0)


#endif /* ADS93XXV_PAGE2_H_ */
