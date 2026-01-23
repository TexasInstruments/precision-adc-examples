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

#ifndef ADS93XXV_PAGE1_H_
#define ADS93XXV_PAGE1_H_

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


/* Register 0x08 (PGA_CONFIG_AIN1_2) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * | CME_CORR_EN_AIN2 |                   CM_RANGE_AIN2[2:0]                   |     RESERVED     |                  INPUT_RANGE_AIN2[2:0]                 |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * | CME_CORR_EN_AIN1 |                   CM_RANGE_AIN1[2:0]                   |     RESERVED     |                  INPUT_RANGE_AIN1[2:0]                 |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PGA_CONFIG_AIN1_2 register */
    #define BANK1_PGA_CONFIG_AIN1_2_ADDRESS						((uint8_t) 0x08)
    #define BANK1_PGA_CONFIG_AIN1_2_DEFAULT						((uint16_t) 0x0000)

    /* CME_CORR_EN_AIN2 field */
    #define BANK1_PGA_CONFIG_AIN1_2_CME_CORR_EN_AIN2_MASK		((uint16_t) 0x8000)
    #define BANK1_PGA_CONFIG_AIN1_2_CME_CORR_EN_AIN2_BITOFFSET	(15)
    #define BANK1_PGA_CONFIG_AIN1_2_CME_CORR_EN_AIN2_DISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN1_2_CME_CORR_EN_AIN2_ENABLED	((uint16_t) 0x8000)

    /* CM_RANGE_AIN2 field */
    #define BANK1_PGA_CONFIG_AIN1_2_CM_RANGE_AIN2_MASK			((uint16_t) 0x7000)
    #define BANK1_PGA_CONFIG_AIN1_2_CM_RANGE_AIN2_BITOFFSET		(12)
    #define BANK1_PGA_CONFIG_AIN1_2_CM_RANGE_AIN2_DIFFERENTIAL	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN1_2_CM_RANGE_AIN2_SINGLEENDED	((uint16_t) 0x5000)
    #define BANK1_PGA_CONFIG_AIN1_2_CM_RANGE_AIN2_SINGLEENDEDOPENWIRESAFE	((uint16_t) 0x6000)

    /* INPUT_RANGE_AIN2 field */
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN2_MASK		((uint16_t) 0x0700)
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN2_BITOFFSET	(8)
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN2_5V			((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN2_RESERVED0	((uint16_t) 0x0100)
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN2_2			((uint16_t) 0x0200)
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN2_6			((uint16_t) 0x0300)
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN2_10V		((uint16_t) 0x0400)
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN2_12			((uint16_t) 0x0500)
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN2_RESERVED1	((uint16_t) 0x0600)
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN2_RESERVED2	((uint16_t) 0x0700)

    /* CME_CORR_EN_AIN1 field */
    #define BANK1_PGA_CONFIG_AIN1_2_CME_CORR_EN_AIN1_MASK		((uint16_t) 0x0080)
    #define BANK1_PGA_CONFIG_AIN1_2_CME_CORR_EN_AIN1_BITOFFSET	(7)
    #define BANK1_PGA_CONFIG_AIN1_2_CME_CORR_EN_AIN1_DISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN1_2_CME_CORR_EN_AIN1_ENABLED	((uint16_t) 0x0080)

    /* CM_RANGE_AIN1 field */
    #define BANK1_PGA_CONFIG_AIN1_2_CM_RANGE_AIN1_MASK			((uint16_t) 0x0070)
    #define BANK1_PGA_CONFIG_AIN1_2_CM_RANGE_AIN1_BITOFFSET		(4)
    #define BANK1_PGA_CONFIG_AIN1_2_CM_RANGE_AIN1_DIFFERENTIAL	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN1_2_CM_RANGE_AIN1_SINGLEENDED	((uint16_t) 0x0050)
    #define BANK1_PGA_CONFIG_AIN1_2_CM_RANGE_AIN1_SINGLEENDEDOPENWIRESAFE	((uint16_t) 0x0060)

    /* INPUT_RANGE_AIN1 field */
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN1_MASK		((uint16_t) 0x0007)
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN1_BITOFFSET	(0)
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN1_5V			((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN1_RESERVED0	((uint16_t) 0x0001)
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN1_2			((uint16_t) 0x0002)
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN1_6			((uint16_t) 0x0003)
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN1_10V		((uint16_t) 0x0004)
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN1_12			((uint16_t) 0x0005)
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN1_RESERVED1	((uint16_t) 0x0006)
    #define BANK1_PGA_CONFIG_AIN1_2_INPUT_RANGE_AIN1_RESERVED2	((uint16_t) 0x0007)


/* Register 0x09 (PGA_CONFIG_AIN3_4) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * | CME_CORR_EN_AIN4 |                   CM_RANGE_AIN4[2:0]                   |     RESERVED     |                  INPUT_RANGE_AIN4[2:0]                 |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * | CME_CORR_EN_AIN3 |                   CM_RANGE_AIN3[2:0]                   |     RESERVED     |                  INPUT_RANGE_AIN3[2:0]                 |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PGA_CONFIG_AIN3_4 register */
    #define BANK1_PGA_CONFIG_AIN3_4_ADDRESS						((uint8_t) 0x09)
    #define BANK1_PGA_CONFIG_AIN3_4_DEFAULT						((uint16_t) 0x0000)

    /* CME_CORR_EN_AIN4 field */
    #define BANK1_PGA_CONFIG_AIN3_4_CME_CORR_EN_AIN4_MASK		((uint16_t) 0x8000)
    #define BANK1_PGA_CONFIG_AIN3_4_CME_CORR_EN_AIN4_BITOFFSET	(15)
    #define BANK1_PGA_CONFIG_AIN3_4_CME_CORR_EN_AIN4_DISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN3_4_CME_CORR_EN_AIN4_ENABLED	((uint16_t) 0x8000)

    /* CM_RANGE_AIN4 field */
    #define BANK1_PGA_CONFIG_AIN3_4_CM_RANGE_AIN4_MASK			((uint16_t) 0x7000)
    #define BANK1_PGA_CONFIG_AIN3_4_CM_RANGE_AIN4_BITOFFSET		(12)
    #define BANK1_PGA_CONFIG_AIN3_4_CM_RANGE_AIN4_DIFFERENTIAL	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN3_4_CM_RANGE_AIN4_SINGLEENDED	((uint16_t) 0x5000)
    #define BANK1_PGA_CONFIG_AIN3_4_CM_RANGE_AIN4_SINGLEENDEDOPENWIRESAFE	((uint16_t) 0x6000)

    /* INPUT_RANGE_AIN4 field */
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN4_MASK		((uint16_t) 0x0700)
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN4_BITOFFSET	(8)
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN4_5V			((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN4_RESERVED0	((uint16_t) 0x0100)
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN4_2			((uint16_t) 0x0200)
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN4_6			((uint16_t) 0x0300)
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN4_10V		((uint16_t) 0x0400)
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN4_12			((uint16_t) 0x0500)
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN4_RESERVED1	((uint16_t) 0x0600)
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN4_RESERVED2	((uint16_t) 0x0700)

    /* CME_CORR_EN_AIN3 field */
    #define BANK1_PGA_CONFIG_AIN3_4_CME_CORR_EN_AIN3_MASK		((uint16_t) 0x0080)
    #define BANK1_PGA_CONFIG_AIN3_4_CME_CORR_EN_AIN3_BITOFFSET	(7)
    #define BANK1_PGA_CONFIG_AIN3_4_CME_CORR_EN_AIN3_DISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN3_4_CME_CORR_EN_AIN3_ENABLED	((uint16_t) 0x0080)

    /* CM_RANGE_AIN3 field */
    #define BANK1_PGA_CONFIG_AIN3_4_CM_RANGE_AIN3_MASK			((uint16_t) 0x0070)
    #define BANK1_PGA_CONFIG_AIN3_4_CM_RANGE_AIN3_BITOFFSET		(4)
    #define BANK1_PGA_CONFIG_AIN3_4_CM_RANGE_AIN3_DIFFERENTIAL	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN3_4_CM_RANGE_AIN3_SINGLEENDED	((uint16_t) 0x0050)
    #define BANK1_PGA_CONFIG_AIN3_4_CM_RANGE_AIN3_SINGLEENDEDOPENWIRESAFE	((uint16_t) 0x0060)

    /* INPUT_RANGE_AIN3 field */
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN3_MASK		((uint16_t) 0x0007)
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN3_BITOFFSET	(0)
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN3_5V			((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN3_RESERVED0	((uint16_t) 0x0001)
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN3_2			((uint16_t) 0x0002)
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN3_6			((uint16_t) 0x0003)
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN3_10V		((uint16_t) 0x0004)
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN3_12			((uint16_t) 0x0005)
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN3_RESERVED1	((uint16_t) 0x0006)
    #define BANK1_PGA_CONFIG_AIN3_4_INPUT_RANGE_AIN3_RESERVED2	((uint16_t) 0x0007)


/* Register 0x0A (PGA_CONFIG_AIN5_6) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * | CME_CORR_EN_AIN6 |                   CM_RANGE_AIN6[2:0]                   |     RESERVED     |                  INPUT_RANGE_AIN6[2:0]                 |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * | CME_CORR_EN_AIN5 |                   CM_RANGE_AIN5[2:0]                   |     RESERVED     |                  INPUT_RANGE_AIN5[2:0]                 |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PGA_CONFIG_AIN5_6 register */
    #define BANK1_PGA_CONFIG_AIN5_6_ADDRESS						((uint8_t) 0x0A)
    #define BANK1_PGA_CONFIG_AIN5_6_DEFAULT						((uint16_t) 0x0000)

    /* CME_CORR_EN_AIN6 field */
    #define BANK1_PGA_CONFIG_AIN5_6_CME_CORR_EN_AIN6_MASK		((uint16_t) 0x8000)
    #define BANK1_PGA_CONFIG_AIN5_6_CME_CORR_EN_AIN6_BITOFFSET	(15)
    #define BANK1_PGA_CONFIG_AIN5_6_CME_CORR_EN_AIN6_DISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN5_6_CME_CORR_EN_AIN6_ENABLED	((uint16_t) 0x8000)

    /* CM_RANGE_AIN6 field */
    #define BANK1_PGA_CONFIG_AIN5_6_CM_RANGE_AIN6_MASK			((uint16_t) 0x7000)
    #define BANK1_PGA_CONFIG_AIN5_6_CM_RANGE_AIN6_BITOFFSET		(12)
    #define BANK1_PGA_CONFIG_AIN5_6_CM_RANGE_AIN6_DIFFERENTIAL	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN5_6_CM_RANGE_AIN6_SINGLEENDED	((uint16_t) 0x5000)
    #define BANK1_PGA_CONFIG_AIN5_6_CM_RANGE_AIN6_SINGLEENDEDOPENWIRESAFE	((uint16_t) 0x6000)

    /* INPUT_RANGE_AIN6 field */
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN6_MASK		((uint16_t) 0x0700)
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN6_BITOFFSET	(8)
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN6_5V			((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN6_RESERVED0	((uint16_t) 0x0100)
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN6_2			((uint16_t) 0x0200)
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN6_6			((uint16_t) 0x0300)
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN6_10V		((uint16_t) 0x0400)
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN6_12			((uint16_t) 0x0500)
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN6_RESERVED1	((uint16_t) 0x0600)
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN6_RESERVED2	((uint16_t) 0x0700)

    /* CME_CORR_EN_AIN5 field */
    #define BANK1_PGA_CONFIG_AIN5_6_CME_CORR_EN_AIN5_MASK		((uint16_t) 0x0080)
    #define BANK1_PGA_CONFIG_AIN5_6_CME_CORR_EN_AIN5_BITOFFSET	(7)
    #define BANK1_PGA_CONFIG_AIN5_6_CME_CORR_EN_AIN5_DISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN5_6_CME_CORR_EN_AIN5_ENABLED	((uint16_t) 0x0080)

    /* CM_RANGE_AIN5 field */
    #define BANK1_PGA_CONFIG_AIN5_6_CM_RANGE_AIN5_MASK			((uint16_t) 0x0070)
    #define BANK1_PGA_CONFIG_AIN5_6_CM_RANGE_AIN5_BITOFFSET		(4)
    #define BANK1_PGA_CONFIG_AIN5_6_CM_RANGE_AIN5_DIFFERENTIAL	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN5_6_CM_RANGE_AIN5_SINGLEENDED	((uint16_t) 0x0050)
    #define BANK1_PGA_CONFIG_AIN5_6_CM_RANGE_AIN5_SINGLEENDEDOPENWIRESAFE	((uint16_t) 0x0060)

    /* INPUT_RANGE_AIN5 field */
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN5_MASK		((uint16_t) 0x0007)
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN5_BITOFFSET	(0)
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN5_5V			((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN5_RESERVED0	((uint16_t) 0x0001)
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN5_2			((uint16_t) 0x0002)
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN5_6			((uint16_t) 0x0003)
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN5_10V		((uint16_t) 0x0004)
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN5_12			((uint16_t) 0x0005)
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN5_RESERVED1	((uint16_t) 0x0006)
    #define BANK1_PGA_CONFIG_AIN5_6_INPUT_RANGE_AIN5_RESERVED2	((uint16_t) 0x0007)


/* Register 0x0B (PGA_CONFIG_AIN7_8) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * | CME_CORR_EN_AIN8 |                   CM_RANGE_AIN8[2:0]                   |     RESERVED     |                  INPUT_RANGE_AIN8[2:0]                 |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * | CME_CORR_EN_AIN7 |                   CM_RANGE_AIN7[2:0]                   |     RESERVED     |                  INPUT_RANGE_AIN7[2:0]                 |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PGA_CONFIG_AIN7_8 register */
    #define BANK1_PGA_CONFIG_AIN7_8_ADDRESS						((uint8_t) 0x0B)
    #define BANK1_PGA_CONFIG_AIN7_8_DEFAULT						((uint16_t) 0x0000)

    /* CME_CORR_EN_AIN8 field */
    #define BANK1_PGA_CONFIG_AIN7_8_CME_CORR_EN_AIN8_MASK		((uint16_t) 0x8000)
    #define BANK1_PGA_CONFIG_AIN7_8_CME_CORR_EN_AIN8_BITOFFSET	(15)
    #define BANK1_PGA_CONFIG_AIN7_8_CME_CORR_EN_AIN8_DISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN7_8_CME_CORR_EN_AIN8_ENABLED	((uint16_t) 0x8000)

    /* CM_RANGE_AIN8 field */
    #define BANK1_PGA_CONFIG_AIN7_8_CM_RANGE_AIN8_MASK			((uint16_t) 0x7000)
    #define BANK1_PGA_CONFIG_AIN7_8_CM_RANGE_AIN8_BITOFFSET		(12)
    #define BANK1_PGA_CONFIG_AIN7_8_CM_RANGE_AIN8_DIFFERENTIAL	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN7_8_CM_RANGE_AIN8_SINGLEENDED	((uint16_t) 0x5000)
    #define BANK1_PGA_CONFIG_AIN7_8_CM_RANGE_AIN8_SINGLEENDEDOPENWIRESAFE	((uint16_t) 0x6000)

    /* INPUT_RANGE_AIN8 field */
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN8_MASK		((uint16_t) 0x0700)
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN8_BITOFFSET	(8)
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN8_5V			((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN8_RESERVED0	((uint16_t) 0x0100)
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN8_2			((uint16_t) 0x0200)
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN8_6			((uint16_t) 0x0300)
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN8_10V		((uint16_t) 0x0400)
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN8_12			((uint16_t) 0x0500)
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN8_RESERVED1	((uint16_t) 0x0600)
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN8_RESERVED2	((uint16_t) 0x0700)

    /* CME_CORR_EN_AIN7 field */
    #define BANK1_PGA_CONFIG_AIN7_8_CME_CORR_EN_AIN7_MASK		((uint16_t) 0x0080)
    #define BANK1_PGA_CONFIG_AIN7_8_CME_CORR_EN_AIN7_BITOFFSET	(7)
    #define BANK1_PGA_CONFIG_AIN7_8_CME_CORR_EN_AIN7_DISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN7_8_CME_CORR_EN_AIN7_ENABLED	((uint16_t) 0x0080)

    /* CM_RANGE_AIN7 field */
    #define BANK1_PGA_CONFIG_AIN7_8_CM_RANGE_AIN7_MASK			((uint16_t) 0x0070)
    #define BANK1_PGA_CONFIG_AIN7_8_CM_RANGE_AIN7_BITOFFSET		(4)
    #define BANK1_PGA_CONFIG_AIN7_8_CM_RANGE_AIN7_DIFFERENTIAL	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN7_8_CM_RANGE_AIN7_SINGLEENDED	((uint16_t) 0x0050)
    #define BANK1_PGA_CONFIG_AIN7_8_CM_RANGE_AIN7_SINGLEENDEDOPENWIRESAFE	((uint16_t) 0x0060)

    /* INPUT_RANGE_AIN7 field */
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN7_MASK		((uint16_t) 0x0007)
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN7_BITOFFSET	(0)
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN7_5V			((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN7_RESERVED0	((uint16_t) 0x0001)
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN7_2			((uint16_t) 0x0002)
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN7_6			((uint16_t) 0x0003)
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN7_10V		((uint16_t) 0x0004)
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN7_12			((uint16_t) 0x0005)
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN7_RESERVED1	((uint16_t) 0x0006)
    #define BANK1_PGA_CONFIG_AIN7_8_INPUT_RANGE_AIN7_RESERVED2	((uint16_t) 0x0007)


/* Register 0x0C (PGA_BW_SEL_AIN1_8) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |         PGA_BW_SEL_AIN8[1:0]        |         PGA_BW_SEL_AIN7[1:0]        |         PGA_BW_SEL_AIN6[1:0]        |         PGA_BW_SEL_AIN5[1:0]        |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         PGA_BW_SEL_AIN4[1:0]        |         PGA_BW_SEL_AIN3[1:0]        |         PGA_BW_SEL_AIN2[1:0]        |         PGA_BW_SEL_AIN1[1:0]        |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PGA_BW_SEL_AIN1_8 register */
    #define BANK1_PGA_BW_SEL_AIN1_8_ADDRESS						((uint8_t) 0x0C)
    #define BANK1_PGA_BW_SEL_AIN1_8_DEFAULT						((uint16_t) 0x0000)

    /* PGA_BW_SEL_AIN8 field */
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN8_MASK		((uint16_t) 0xC000)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN8_BITOFFSET	(14)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN8_LOWBANDWIDTH	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN8_WIDEBANDWIDTH	((uint16_t) 0x4000)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN8_RESERVED0	((uint16_t) 0x8000)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN8_RESERVED1	((uint16_t) 0xC000)

    /* PGA_BW_SEL_AIN7 field */
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN7_MASK		((uint16_t) 0x3000)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN7_BITOFFSET	(12)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN7_LOWBANDWIDTH	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN7_WIDEBANDWIDTH	((uint16_t) 0x1000)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN7_RESERVED0	((uint16_t) 0x2000)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN7_RESERVED1	((uint16_t) 0x3000)

    /* PGA_BW_SEL_AIN6 field */
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN6_MASK		((uint16_t) 0x0C00)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN6_BITOFFSET	(10)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN6_LOWBANDWIDTH	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN6_WIDEBANDWIDTH	((uint16_t) 0x0400)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN6_RESERVED0	((uint16_t) 0x0800)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN6_RESERVED1	((uint16_t) 0x0C00)

    /* PGA_BW_SEL_AIN5 field */
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN5_MASK		((uint16_t) 0x0300)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN5_BITOFFSET	(8)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN5_LOWBANDWIDTH	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN5_WIDEBANDWIDTH	((uint16_t) 0x0100)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN5_RESERVED0	((uint16_t) 0x0200)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN5_RESERVED1	((uint16_t) 0x0300)

    /* PGA_BW_SEL_AIN4 field */
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN4_MASK		((uint16_t) 0x00C0)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN4_BITOFFSET	(6)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN4_LOWBANDWIDTH	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN4_WIDEBANDWIDTH	((uint16_t) 0x0040)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN4_RESERVED0	((uint16_t) 0x0080)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN4_RESERVED1	((uint16_t) 0x00C0)

    /* PGA_BW_SEL_AIN3 field */
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN3_MASK		((uint16_t) 0x0030)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN3_BITOFFSET	(4)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN3_LOWBANDWIDTH	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN3_WIDEBANDWIDTH	((uint16_t) 0x0010)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN3_RESERVED0	((uint16_t) 0x0020)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN3_RESERVED1	((uint16_t) 0x0030)

    /* PGA_BW_SEL_AIN2 field */
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN2_MASK		((uint16_t) 0x000C)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN2_BITOFFSET	(2)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN2_LOWBANDWIDTH	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN2_WIDEBANDWIDTH	((uint16_t) 0x0004)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN2_RESERVED0	((uint16_t) 0x0008)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN2_RESERVED1	((uint16_t) 0x000C)

    /* PGA_BW_SEL_AIN1 field */
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN1_MASK		((uint16_t) 0x0003)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN1_BITOFFSET	(0)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN1_LOWBANDWIDTH	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN1_WIDEBANDWIDTH	((uint16_t) 0x0001)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN1_RESERVED0	((uint16_t) 0x0002)
    #define BANK1_PGA_BW_SEL_AIN1_8_PGA_BW_SEL_AIN1_RESERVED1	((uint16_t) 0x0003)


/* Register 0x0D (PHASE_DELAY_AIN1_2) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                 PHASE_DELAY_AIN2[7:0]                                                                 |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                 PHASE_DELAY_AIN1[7:0]                                                                 |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PHASE_DELAY_AIN1_2 register */
    #define BANK1_PHASE_DELAY_AIN1_2_ADDRESS					((uint8_t) 0x0D)
    #define BANK1_PHASE_DELAY_AIN1_2_DEFAULT					((uint16_t) 0x0000)

    /* PHASE_DELAY_AIN2 field */
    #define BANK1_PHASE_DELAY_AIN1_2_PHASE_DELAY_AIN2_MASK		((uint16_t) 0xFF00)
    #define BANK1_PHASE_DELAY_AIN1_2_PHASE_DELAY_AIN2_BITOFFSET	(8)

    /* PHASE_DELAY_AIN1 field */
    #define BANK1_PHASE_DELAY_AIN1_2_PHASE_DELAY_AIN1_MASK		((uint16_t) 0x00FF)
    #define BANK1_PHASE_DELAY_AIN1_2_PHASE_DELAY_AIN1_BITOFFSET	(0)


/* Register 0x0E (PHASE_DELAY_AIN3_4) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                 PHASE_DELAY_AIN4[7:0]                                                                 |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                 PHASE_DELAY_AIN3[7:0]                                                                 |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PHASE_DELAY_AIN3_4 register */
    #define BANK1_PHASE_DELAY_AIN3_4_ADDRESS					((uint8_t) 0x0E)
    #define BANK1_PHASE_DELAY_AIN3_4_DEFAULT					((uint16_t) 0x0000)

    /* PHASE_DELAY_AIN4 field */
    #define BANK1_PHASE_DELAY_AIN3_4_PHASE_DELAY_AIN4_MASK		((uint16_t) 0xFF00)
    #define BANK1_PHASE_DELAY_AIN3_4_PHASE_DELAY_AIN4_BITOFFSET	(8)

    /* PHASE_DELAY_AIN3 field */
    #define BANK1_PHASE_DELAY_AIN3_4_PHASE_DELAY_AIN3_MASK		((uint16_t) 0x00FF)
    #define BANK1_PHASE_DELAY_AIN3_4_PHASE_DELAY_AIN3_BITOFFSET	(0)


/* Register 0x0F (PHASE_DELAY_AIN5_6) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                 PHASE_DELAY_AIN6[7:0]                                                                 |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                 PHASE_DELAY_AIN5[7:0]                                                                 |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PHASE_DELAY_AIN5_6 register */
    #define BANK1_PHASE_DELAY_AIN5_6_ADDRESS					((uint8_t) 0x0F)
    #define BANK1_PHASE_DELAY_AIN5_6_DEFAULT					((uint16_t) 0x0000)

    /* PHASE_DELAY_AIN6 field */
    #define BANK1_PHASE_DELAY_AIN5_6_PHASE_DELAY_AIN6_MASK		((uint16_t) 0xFF00)
    #define BANK1_PHASE_DELAY_AIN5_6_PHASE_DELAY_AIN6_BITOFFSET	(8)

    /* PHASE_DELAY_AIN5 field */
    #define BANK1_PHASE_DELAY_AIN5_6_PHASE_DELAY_AIN5_MASK		((uint16_t) 0x00FF)
    #define BANK1_PHASE_DELAY_AIN5_6_PHASE_DELAY_AIN5_BITOFFSET	(0)


/* Register 0x10 (PHASE_DELAY_AIN7_8) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                 PHASE_DELAY_AIN8[7:0]                                                                 |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                 PHASE_DELAY_AIN7[7:0]                                                                 |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PHASE_DELAY_AIN7_8 register */
    #define BANK1_PHASE_DELAY_AIN7_8_ADDRESS					((uint8_t) 0x10)
    #define BANK1_PHASE_DELAY_AIN7_8_DEFAULT					((uint16_t) 0x0000)

    /* PHASE_DELAY_AIN8 field */
    #define BANK1_PHASE_DELAY_AIN7_8_PHASE_DELAY_AIN8_MASK		((uint16_t) 0xFF00)
    #define BANK1_PHASE_DELAY_AIN7_8_PHASE_DELAY_AIN8_BITOFFSET	(8)

    /* PHASE_DELAY_AIN7 field */
    #define BANK1_PHASE_DELAY_AIN7_8_PHASE_DELAY_AIN7_MASK		((uint16_t) 0x00FF)
    #define BANK1_PHASE_DELAY_AIN7_8_PHASE_DELAY_AIN7_BITOFFSET	(0)


/* Register 0x11 (OFS_AIN1) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                  RESERVED[5:0]                                                  |            OFS_AIN1[9:8]             
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                       OFS_AIN1[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFS_AIN1 register */
    #define BANK1_OFS_AIN1_ADDRESS								((uint8_t) 0x11)
    #define BANK1_OFS_AIN1_DEFAULT								((uint16_t) 0x0000)

    /* OFS_AIN1 field */
    #define BANK1_OFS_AIN1_MASK									((uint16_t) 0x03FF)
    #define BANK1_OFS_AIN1_BITOFFSET							(0)


/* Register 0x12 (OFS_AIN2) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                  RESERVED[5:0]                                                  |            OFS_AIN2[9:8]             
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                       OFS_AIN2[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFS_AIN2 register */
    #define BANK1_OFS_AIN2_ADDRESS								((uint8_t) 0x12)
    #define BANK1_OFS_AIN2_DEFAULT								((uint16_t) 0x0000)

    /* OFS_AIN2 field */
    #define BANK1_OFS_AIN2_MASK									((uint16_t) 0x03FF)
    #define BANK1_OFS_AIN2_BITOFFSET							(0)


/* Register 0x13 (OFS_AIN3) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                  RESERVED[5:0]                                                  |            OFS_AIN3[9:8]             
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                       OFS_AIN3[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFS_AIN3 register */
    #define BANK1_OFS_AIN3_ADDRESS								((uint8_t) 0x13)
    #define BANK1_OFS_AIN3_DEFAULT								((uint16_t) 0x0000)

    /* OFS_AIN3 field */
    #define BANK1_OFS_AIN3_MASK									((uint16_t) 0x03FF)
    #define BANK1_OFS_AIN3_BITOFFSET							(0)


/* Register 0x14 (OFS_AIN4) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                  RESERVED[5:0]                                                  |            OFS_AIN4[9:8]             
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                       OFS_AIN4[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFS_AIN4 register */
    #define BANK1_OFS_AIN4_ADDRESS								((uint8_t) 0x14)
    #define BANK1_OFS_AIN4_DEFAULT								((uint16_t) 0x0000)

    /* OFS_AIN4 field */
    #define BANK1_OFS_AIN4_MASK									((uint16_t) 0x03FF)
    #define BANK1_OFS_AIN4_BITOFFSET							(0)


/* Register 0x15 (OFS_AIN5) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                  RESERVED[5:0]                                                  |            OFS_AIN5[9:8]             
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                       OFS_AIN5[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFS_AIN5 register */
    #define BANK1_OFS_AIN5_ADDRESS								((uint8_t) 0x15)
    #define BANK1_OFS_AIN5_DEFAULT								((uint16_t) 0x0000)

    /* OFS_AIN5 field */
    #define BANK1_OFS_AIN5_MASK									((uint16_t) 0x03FF)
    #define BANK1_OFS_AIN5_BITOFFSET							(0)


/* Register 0x16 (OFS_AIN6) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                  RESERVED[5:0]                                                  |            OFS_AIN6[9:8]             
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                       OFS_AIN6[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFS_AIN6 register */
    #define BANK1_OFS_AIN6_ADDRESS								((uint8_t) 0x16)
    #define BANK1_OFS_AIN6_DEFAULT								((uint16_t) 0x0000)

    /* OFS_AIN6 field */
    #define BANK1_OFS_AIN6_MASK									((uint16_t) 0x03FF)
    #define BANK1_OFS_AIN6_BITOFFSET							(0)


/* Register 0x17 (OFS_AIN7) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                  RESERVED[5:0]                                                  |            OFS_AIN7[9:8]             
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                       OFS_AIN7[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFS_AIN7 register */
    #define BANK1_OFS_AIN7_ADDRESS								((uint8_t) 0x17)
    #define BANK1_OFS_AIN7_DEFAULT								((uint16_t) 0x0000)

    /* OFS_AIN7 field */
    #define BANK1_OFS_AIN7_MASK									((uint16_t) 0x03FF)
    #define BANK1_OFS_AIN7_BITOFFSET							(0)


/* Register 0x18 (OFS_AIN8) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                  RESERVED[5:0]                                                  |            OFS_AIN8[9:8]             
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                       OFS_AIN8[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFS_AIN8 register */
    #define BANK1_OFS_AIN8_ADDRESS								((uint8_t) 0x18)
    #define BANK1_OFS_AIN8_DEFAULT								((uint16_t) 0x0000)

    /* OFS_AIN8 field */
    #define BANK1_OFS_AIN8_MASK									((uint16_t) 0x03FF)
    #define BANK1_OFS_AIN8_BITOFFSET							(0)


/* Register 0x19 (GAN_AIN1) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |            RESERVED[1:0]            |                                                  GAN_AIN1[13:8]                                                  
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                       GAN_AIN1[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAN_AIN1 register */
    #define BANK1_GAN_AIN1_ADDRESS								((uint8_t) 0x19)
    #define BANK1_GAN_AIN1_DEFAULT								((uint16_t) 0x0000)

    /* GAN_AIN1 field */
    #define BANK1_GAN_AIN1_MASK									((uint16_t) 0x3FFF)
    #define BANK1_GAN_AIN1_BITOFFSET							(0)


/* Register 0x1A (GAN_AIN2) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |            RESERVED[1:0]            |                                                  GAN_AIN2[13:8]                                                  
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                       GAN_AIN2[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAN_AIN2 register */
    #define BANK1_GAN_AIN2_ADDRESS								((uint8_t) 0x1A)
    #define BANK1_GAN_AIN2_DEFAULT								((uint16_t) 0x0000)

    /* GAN_AIN2 field */
    #define BANK1_GAN_AIN2_MASK									((uint16_t) 0x3FFF)
    #define BANK1_GAN_AIN2_BITOFFSET							(0)


/* Register 0x1B (GAN_AIN3) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |            RESERVED[1:0]            |                                                  GAN_AIN3[13:8]                                                  
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                       GAN_AIN3[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAN_AIN3 register */
    #define BANK1_GAN_AIN3_ADDRESS								((uint8_t) 0x1B)
    #define BANK1_GAN_AIN3_DEFAULT								((uint16_t) 0x0000)

    /* GAN_AIN3 field */
    #define BANK1_GAN_AIN3_MASK									((uint16_t) 0x3FFF)
    #define BANK1_GAN_AIN3_BITOFFSET							(0)


/* Register 0x1C (GAN_AIN4) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |            RESERVED[1:0]            |                                                  GAN_AIN4[13:8]                                                  
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                       GAN_AIN4[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAN_AIN4 register */
    #define BANK1_GAN_AIN4_ADDRESS								((uint8_t) 0x1C)
    #define BANK1_GAN_AIN4_DEFAULT								((uint16_t) 0x0000)

    /* GAN_AIN4 field */
    #define BANK1_GAN_AIN4_MASK									((uint16_t) 0x3FFF)
    #define BANK1_GAN_AIN4_BITOFFSET							(0)


/* Register 0x1D (GAN_AIN5) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |            RESERVED[1:0]            |                                                  GAN_AIN5[13:8]                                                  
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                       GAN_AIN5[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAN_AIN5 register */
    #define BANK1_GAN_AIN5_ADDRESS								((uint8_t) 0x1D)
    #define BANK1_GAN_AIN5_DEFAULT								((uint16_t) 0x0000)

    /* GAN_AIN5 field */
    #define BANK1_GAN_AIN5_MASK									((uint16_t) 0x3FFF)
    #define BANK1_GAN_AIN5_BITOFFSET							(0)


/* Register 0x1E (GAN_AIN6) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |            RESERVED[1:0]            |                                                  GAN_AIN6[13:8]                                                  
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                       GAN_AIN6[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAN_AIN6 register */
    #define BANK1_GAN_AIN6_ADDRESS								((uint8_t) 0x1E)
    #define BANK1_GAN_AIN6_DEFAULT								((uint16_t) 0x0000)

    /* GAN_AIN6 field */
    #define BANK1_GAN_AIN6_MASK									((uint16_t) 0x3FFF)
    #define BANK1_GAN_AIN6_BITOFFSET							(0)


/* Register 0x1F (GAN_AIN7) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |            RESERVED[1:0]            |                                                  GAN_AIN7[13:8]                                                  
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                       GAN_AIN7[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAN_AIN7 register */
    #define BANK1_GAN_AIN7_ADDRESS								((uint8_t) 0x1F)
    #define BANK1_GAN_AIN7_DEFAULT								((uint16_t) 0x0000)

    /* GAN_AIN7 field */
    #define BANK1_GAN_AIN7_MASK									((uint16_t) 0x3FFF)
    #define BANK1_GAN_AIN7_BITOFFSET							(0)


/* Register 0x20 (GAN_AIN8) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |            RESERVED[1:0]            |                                                  GAN_AIN8[13:8]                                                  
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                       GAN_AIN8[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAN_AIN8 register */
    #define BANK1_GAN_AIN8_ADDRESS								((uint8_t) 0x20)
    #define BANK1_GAN_AIN8_DEFAULT								((uint16_t) 0x0000)

    /* GAN_AIN8 field */
    #define BANK1_GAN_AIN8_MASK									((uint16_t) 0x3FFF)
    #define BANK1_GAN_AIN8_BITOFFSET							(0)


/* Register 0x21 (DWC_CFG) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |   DWC_STAT_RST   |                      RESERVED[2:0]                     |                            DWC_GLITCH_FILT[3:0]                           |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |    DWC_EN_AIN8   |    DWC_EN_AIN7   |    DWC_EN_AIN6   |    DWC_EN_AIN5   |    DWC_EN_AIN4   |    DWC_EN_AIN3   |    DWC_EN_AIN2   |    DWC_EN_AIN1   |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_CFG register */
    #define BANK1_DWC_CFG_ADDRESS								((uint8_t) 0x21)
    #define BANK1_DWC_CFG_DEFAULT								((uint16_t) 0x0000)

    /* DWC_STAT_RST field */
    #define BANK1_DWC_CFG_DWC_STAT_RST_MASK						((uint16_t) 0x8000)
    #define BANK1_DWC_CFG_DWC_STAT_RST_BITOFFSET				(15)

    /* DWC_GLITCH_FILT field */
    #define BANK1_DWC_CFG_DWC_GLITCH_FILT_MASK					((uint16_t) 0x0F00)
    #define BANK1_DWC_CFG_DWC_GLITCH_FILT_BITOFFSET				(8)

    /* DWC_EN_AIN8 field */
    #define BANK1_DWC_CFG_DWC_EN_AIN8_MASK						((uint16_t) 0x0080)
    #define BANK1_DWC_CFG_DWC_EN_AIN8_BITOFFSET					(7)
    #define BANK1_DWC_CFG_DWC_EN_AIN8_DISABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_DWC_CFG_DWC_EN_AIN8_ENABLED					((uint16_t) 0x0080)

    /* DWC_EN_AIN7 field */
    #define BANK1_DWC_CFG_DWC_EN_AIN7_MASK						((uint16_t) 0x0040)
    #define BANK1_DWC_CFG_DWC_EN_AIN7_BITOFFSET					(6)
    #define BANK1_DWC_CFG_DWC_EN_AIN7_DISABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_DWC_CFG_DWC_EN_AIN7_ENABLED					((uint16_t) 0x0040)

    /* DWC_EN_AIN6 field */
    #define BANK1_DWC_CFG_DWC_EN_AIN6_MASK						((uint16_t) 0x0020)
    #define BANK1_DWC_CFG_DWC_EN_AIN6_BITOFFSET					(5)
    #define BANK1_DWC_CFG_DWC_EN_AIN6_DISABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_DWC_CFG_DWC_EN_AIN6_ENABLED					((uint16_t) 0x0020)

    /* DWC_EN_AIN5 field */
    #define BANK1_DWC_CFG_DWC_EN_AIN5_MASK						((uint16_t) 0x0010)
    #define BANK1_DWC_CFG_DWC_EN_AIN5_BITOFFSET					(4)
    #define BANK1_DWC_CFG_DWC_EN_AIN5_DISABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_DWC_CFG_DWC_EN_AIN5_ENABLED					((uint16_t) 0x0010)

    /* DWC_EN_AIN4 field */
    #define BANK1_DWC_CFG_DWC_EN_AIN4_MASK						((uint16_t) 0x0008)
    #define BANK1_DWC_CFG_DWC_EN_AIN4_BITOFFSET					(3)
    #define BANK1_DWC_CFG_DWC_EN_AIN4_DISABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_DWC_CFG_DWC_EN_AIN4_ENABLED					((uint16_t) 0x0008)

    /* DWC_EN_AIN3 field */
    #define BANK1_DWC_CFG_DWC_EN_AIN3_MASK						((uint16_t) 0x0004)
    #define BANK1_DWC_CFG_DWC_EN_AIN3_BITOFFSET					(2)
    #define BANK1_DWC_CFG_DWC_EN_AIN3_DISABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_DWC_CFG_DWC_EN_AIN3_ENABLED					((uint16_t) 0x0004)

    /* DWC_EN_AIN2 field */
    #define BANK1_DWC_CFG_DWC_EN_AIN2_MASK						((uint16_t) 0x0002)
    #define BANK1_DWC_CFG_DWC_EN_AIN2_BITOFFSET					(1)
    #define BANK1_DWC_CFG_DWC_EN_AIN2_DISABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_DWC_CFG_DWC_EN_AIN2_ENABLED					((uint16_t) 0x0002)

    /* DWC_EN_AIN1 field */
    #define BANK1_DWC_CFG_DWC_EN_AIN1_MASK						((uint16_t) 0x0001)
    #define BANK1_DWC_CFG_DWC_EN_AIN1_BITOFFSET					(0)
    #define BANK1_DWC_CFG_DWC_EN_AIN1_DISABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_DWC_CFG_DWC_EN_AIN1_ENABLED					((uint16_t) 0x0001)


/* Register 0x22 (DWC_TH_AIN1) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                   HIGH_TH_AIN1[7:0]                                                                   |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                    LOW_TH_AIN1[7:0]                                                                   |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_TH_AIN1 register */
    #define BANK1_DWC_TH_AIN1_ADDRESS							((uint8_t) 0x22)
    #define BANK1_DWC_TH_AIN1_DEFAULT							((uint16_t) 0xFF00)

    /* HIGH_TH_AIN1 field */
    #define BANK1_DWC_TH_AIN1_HIGH_TH_AIN1_MASK					((uint16_t) 0xFF00)
    #define BANK1_DWC_TH_AIN1_HIGH_TH_AIN1_BITOFFSET			(8)

    /* LOW_TH_AIN1 field */
    #define BANK1_DWC_TH_AIN1_LOW_TH_AIN1_MASK					((uint16_t) 0x00FF)
    #define BANK1_DWC_TH_AIN1_LOW_TH_AIN1_BITOFFSET				(0)


/* Register 0x23 (DWC_TH_AIN2) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                   HIGH_TH_AIN2[7:0]                                                                   |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                    LOW_TH_AIN2[7:0]                                                                   |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_TH_AIN2 register */
    #define BANK1_DWC_TH_AIN2_ADDRESS							((uint8_t) 0x23)
    #define BANK1_DWC_TH_AIN2_DEFAULT							((uint16_t) 0xFF00)

    /* HIGH_TH_AIN2 field */
    #define BANK1_DWC_TH_AIN2_HIGH_TH_AIN2_MASK					((uint16_t) 0xFF00)
    #define BANK1_DWC_TH_AIN2_HIGH_TH_AIN2_BITOFFSET			(8)

    /* LOW_TH_AIN2 field */
    #define BANK1_DWC_TH_AIN2_LOW_TH_AIN2_MASK					((uint16_t) 0x00FF)
    #define BANK1_DWC_TH_AIN2_LOW_TH_AIN2_BITOFFSET				(0)


/* Register 0x24 (DWC_TH_AIN3) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                   HIGH_TH_AIN3[7:0]                                                                   |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                    LOW_TH_AIN3[7:0]                                                                   |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_TH_AIN3 register */
    #define BANK1_DWC_TH_AIN3_ADDRESS							((uint8_t) 0x24)
    #define BANK1_DWC_TH_AIN3_DEFAULT							((uint16_t) 0xFF00)

    /* HIGH_TH_AIN3 field */
    #define BANK1_DWC_TH_AIN3_HIGH_TH_AIN3_MASK					((uint16_t) 0xFF00)
    #define BANK1_DWC_TH_AIN3_HIGH_TH_AIN3_BITOFFSET			(8)

    /* LOW_TH_AIN3 field */
    #define BANK1_DWC_TH_AIN3_LOW_TH_AIN3_MASK					((uint16_t) 0x00FF)
    #define BANK1_DWC_TH_AIN3_LOW_TH_AIN3_BITOFFSET				(0)


/* Register 0x25 (DWC_TH_AIN4) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                   HIGH_TH_AIN4[7:0]                                                                   |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                    LOW_TH_AIN4[7:0]                                                                   |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_TH_AIN4 register */
    #define BANK1_DWC_TH_AIN4_ADDRESS							((uint8_t) 0x25)
    #define BANK1_DWC_TH_AIN4_DEFAULT							((uint16_t) 0xFF00)

    /* HIGH_TH_AIN4 field */
    #define BANK1_DWC_TH_AIN4_HIGH_TH_AIN4_MASK					((uint16_t) 0xFF00)
    #define BANK1_DWC_TH_AIN4_HIGH_TH_AIN4_BITOFFSET			(8)

    /* LOW_TH_AIN4 field */
    #define BANK1_DWC_TH_AIN4_LOW_TH_AIN4_MASK					((uint16_t) 0x00FF)
    #define BANK1_DWC_TH_AIN4_LOW_TH_AIN4_BITOFFSET				(0)


/* Register 0x26 (DWC_TH_AIN5) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                   HIGH_TH_AIN5[7:0]                                                                   |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                    LOW_TH_AIN5[7:0]                                                                   |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_TH_AIN5 register */
    #define BANK1_DWC_TH_AIN5_ADDRESS							((uint8_t) 0x26)
    #define BANK1_DWC_TH_AIN5_DEFAULT							((uint16_t) 0xFF00)

    /* HIGH_TH_AIN5 field */
    #define BANK1_DWC_TH_AIN5_HIGH_TH_AIN5_MASK					((uint16_t) 0xFF00)
    #define BANK1_DWC_TH_AIN5_HIGH_TH_AIN5_BITOFFSET			(8)

    /* LOW_TH_AIN5 field */
    #define BANK1_DWC_TH_AIN5_LOW_TH_AIN5_MASK					((uint16_t) 0x00FF)
    #define BANK1_DWC_TH_AIN5_LOW_TH_AIN5_BITOFFSET				(0)


/* Register 0x27 (DWC_TH_AIN6) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                   HIGH_TH_AIN6[7:0]                                                                   |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                    LOW_TH_AIN6[7:0]                                                                   |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_TH_AIN6 register */
    #define BANK1_DWC_TH_AIN6_ADDRESS							((uint8_t) 0x27)
    #define BANK1_DWC_TH_AIN6_DEFAULT							((uint16_t) 0xFF00)

    /* HIGH_TH_AIN6 field */
    #define BANK1_DWC_TH_AIN6_HIGH_TH_AIN6_MASK					((uint16_t) 0xFF00)
    #define BANK1_DWC_TH_AIN6_HIGH_TH_AIN6_BITOFFSET			(8)

    /* LOW_TH_AIN6 field */
    #define BANK1_DWC_TH_AIN6_LOW_TH_AIN6_MASK					((uint16_t) 0x00FF)
    #define BANK1_DWC_TH_AIN6_LOW_TH_AIN6_BITOFFSET				(0)


/* Register 0x28 (DWC_TH_AIN7) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                   HIGH_TH_AIN7[7:0]                                                                   |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                    LOW_TH_AIN7[7:0]                                                                   |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_TH_AIN7 register */
    #define BANK1_DWC_TH_AIN7_ADDRESS							((uint8_t) 0x28)
    #define BANK1_DWC_TH_AIN7_DEFAULT							((uint16_t) 0xFF00)

    /* HIGH_TH_AIN7 field */
    #define BANK1_DWC_TH_AIN7_HIGH_TH_AIN7_MASK					((uint16_t) 0xFF00)
    #define BANK1_DWC_TH_AIN7_HIGH_TH_AIN7_BITOFFSET			(8)

    /* LOW_TH_AIN7 field */
    #define BANK1_DWC_TH_AIN7_LOW_TH_AIN7_MASK					((uint16_t) 0x00FF)
    #define BANK1_DWC_TH_AIN7_LOW_TH_AIN7_BITOFFSET				(0)


/* Register 0x29 (DWC_TH_AIN8) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                   HIGH_TH_AIN8[7:0]                                                                   |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                    LOW_TH_AIN8[7:0]                                                                   |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_TH_AIN8 register */
    #define BANK1_DWC_TH_AIN8_ADDRESS							((uint8_t) 0x29)
    #define BANK1_DWC_TH_AIN8_DEFAULT							((uint16_t) 0xFF00)

    /* HIGH_TH_AIN8 field */
    #define BANK1_DWC_TH_AIN8_HIGH_TH_AIN8_MASK					((uint16_t) 0xFF00)
    #define BANK1_DWC_TH_AIN8_HIGH_TH_AIN8_BITOFFSET			(8)

    /* LOW_TH_AIN8 field */
    #define BANK1_DWC_TH_AIN8_LOW_TH_AIN8_MASK					((uint16_t) 0x00FF)
    #define BANK1_DWC_TH_AIN8_LOW_TH_AIN8_BITOFFSET				(0)


/* Register 0x2A (DWC_HYS_AIN1_2) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     HYS_AIN2[7:0]                                                                     |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                     HYS_AIN1[7:0]                                                                     |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_HYS_AIN1_2 register */
    #define BANK1_DWC_HYS_AIN1_2_ADDRESS						((uint8_t) 0x2A)
    #define BANK1_DWC_HYS_AIN1_2_DEFAULT						((uint16_t) 0x0000)

    /* HYS_AIN2 field */
    #define BANK1_DWC_HYS_AIN1_2_HYS_AIN2_MASK					((uint16_t) 0xFF00)
    #define BANK1_DWC_HYS_AIN1_2_HYS_AIN2_BITOFFSET				(8)

    /* HYS_AIN1 field */
    #define BANK1_DWC_HYS_AIN1_2_HYS_AIN1_MASK					((uint16_t) 0x00FF)
    #define BANK1_DWC_HYS_AIN1_2_HYS_AIN1_BITOFFSET				(0)


/* Register 0x2B (DWC_HYS_AIN3_4) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     HYS_AIN4[7:0]                                                                     |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                     HYS_AIN3[7:0]                                                                     |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_HYS_AIN3_4 register */
    #define BANK1_DWC_HYS_AIN3_4_ADDRESS						((uint8_t) 0x2B)
    #define BANK1_DWC_HYS_AIN3_4_DEFAULT						((uint16_t) 0xFF00)

    /* HYS_AIN4 field */
    #define BANK1_DWC_HYS_AIN3_4_HYS_AIN4_MASK					((uint16_t) 0xFF00)
    #define BANK1_DWC_HYS_AIN3_4_HYS_AIN4_BITOFFSET				(8)

    /* HYS_AIN3 field */
    #define BANK1_DWC_HYS_AIN3_4_HYS_AIN3_MASK					((uint16_t) 0x00FF)
    #define BANK1_DWC_HYS_AIN3_4_HYS_AIN3_BITOFFSET				(0)


/* Register 0x2C (DWC_HYS_AIN5_6) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     HYS_AIN6[7:0]                                                                     |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                     HYS_AIN5[7:0]                                                                     |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_HYS_AIN5_6 register */
    #define BANK1_DWC_HYS_AIN5_6_ADDRESS						((uint8_t) 0x2C)
    #define BANK1_DWC_HYS_AIN5_6_DEFAULT						((uint16_t) 0xFF00)

    /* HYS_AIN6 field */
    #define BANK1_DWC_HYS_AIN5_6_HYS_AIN6_MASK					((uint16_t) 0xFF00)
    #define BANK1_DWC_HYS_AIN5_6_HYS_AIN6_BITOFFSET				(8)

    /* HYS_AIN5 field */
    #define BANK1_DWC_HYS_AIN5_6_HYS_AIN5_MASK					((uint16_t) 0x00FF)
    #define BANK1_DWC_HYS_AIN5_6_HYS_AIN5_BITOFFSET				(0)


/* Register 0x2D (DWC_HYS_AIN7_8) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     HYS_AIN8[7:0]                                                                     |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                     HYS_AIN7[7:0]                                                                     |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_HYS_AIN7_8 register */
    #define BANK1_DWC_HYS_AIN7_8_ADDRESS						((uint8_t) 0x2D)
    #define BANK1_DWC_HYS_AIN7_8_DEFAULT						((uint16_t) 0xFF00)

    /* HYS_AIN8 field */
    #define BANK1_DWC_HYS_AIN7_8_HYS_AIN8_MASK					((uint16_t) 0xFF00)
    #define BANK1_DWC_HYS_AIN7_8_HYS_AIN8_BITOFFSET				(8)

    /* HYS_AIN7 field */
    #define BANK1_DWC_HYS_AIN7_8_HYS_AIN7_MASK					((uint16_t) 0x00FF)
    #define BANK1_DWC_HYS_AIN7_8_HYS_AIN7_BITOFFSET				(0)


/* Register 0x2E (TP_CFG) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     RESERVED[8:1]                                                                      
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *     RESERVED[0:0]  |                      TP_MODE[2:0]                      |     RESERVED     |    TP_DIS_IDX    |    TP_UPD_MODE   |       TP_EN      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_CFG register */
    #define BANK1_TP_CFG_ADDRESS								((uint8_t) 0x2E)
    #define BANK1_TP_CFG_DEFAULT								((uint16_t) 0x0000)

    /* TP_MODE field */
    #define BANK1_TP_CFG_TP_MODE_MASK							((uint16_t) 0x0070)
    #define BANK1_TP_CFG_TP_MODE_BITOFFSET						(4)
    #define BANK1_TP_CFG_TP_MODE_CONSTANTPATTERN				((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_TP_CFG_TP_MODE_RESERVED0						((uint16_t) 0x0010)
    #define BANK1_TP_CFG_TP_MODE_RAMPPATTERN					((uint16_t) 0x0020)
    #define BANK1_TP_CFG_TP_MODE_RESERVED1						((uint16_t) 0x0030)
    #define BANK1_TP_CFG_TP_MODE_RESERVED2						((uint16_t) 0x0040)
    #define BANK1_TP_CFG_TP_MODE_RESERVED3						((uint16_t) 0x0050)

    /* TP_DIS_IDX field */
    #define BANK1_TP_CFG_TP_DIS_IDX_MASK						((uint16_t) 0x0004)
    #define BANK1_TP_CFG_TP_DIS_IDX_BITOFFSET					(2)

    /* TP_UPD_MODE field */
    #define BANK1_TP_CFG_TP_UPD_MODE_MASK						((uint16_t) 0x0002)
    #define BANK1_TP_CFG_TP_UPD_MODE_BITOFFSET					(1)
    #define BANK1_TP_CFG_TP_UPD_MODE_INCREMENTHAPPENSATCHANNELFRAMEBOUNDARY	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_TP_CFG_TP_UPD_MODE_INCREMENTHAPPENSATEVERYCONVST	((uint16_t) 0x0002)

    /* TP_EN field */
    #define BANK1_TP_CFG_TP_EN_MASK								((uint16_t) 0x0001)
    #define BANK1_TP_CFG_TP_EN_BITOFFSET						(0)
    #define BANK1_TP_CFG_TP_EN_ADCCONVERSIONRESULTISLAUNCHEDONTHEDATAINTERFACE	((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_TP_CFG_TP_EN_DIGITALTESTPATTERNISLAUNCHEDONTHEDATAINTERFACE	((uint16_t) 0x0001)


/* Register 0x2F (TP_AIN1) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     TP_AIN1[15:8]                                                                      
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                        TP_AIN1[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_AIN1 register */
    #define BANK1_TP_AIN1_ADDRESS								((uint8_t) 0x2F)
    #define BANK1_TP_AIN1_DEFAULT								((uint16_t) 0x0000)

    /* TP_AIN1 field */
    #define BANK1_TP_AIN1_MASK									((uint16_t) 0xFFFF)
    #define BANK1_TP_AIN1_BITOFFSET								(0)


/* Register 0x30 (TP_AIN2) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     TP_AIN2[15:8]                                                                      
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                        TP_AIN2[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_AIN2 register */
    #define BANK1_TP_AIN2_ADDRESS								((uint8_t) 0x30)
    #define BANK1_TP_AIN2_DEFAULT								((uint16_t) 0x0000)

    /* TP_AIN2 field */
    #define BANK1_TP_AIN2_MASK									((uint16_t) 0xFFFF)
    #define BANK1_TP_AIN2_BITOFFSET								(0)


/* Register 0x31 (TP_AIN3) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     TP_AIN3[15:8]                                                                      
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                        TP_AIN3[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_AIN3 register */
    #define BANK1_TP_AIN3_ADDRESS								((uint8_t) 0x31)
    #define BANK1_TP_AIN3_DEFAULT								((uint16_t) 0x0000)

    /* TP_AIN3 field */
    #define BANK1_TP_AIN3_MASK									((uint16_t) 0xFFFF)
    #define BANK1_TP_AIN3_BITOFFSET								(0)


/* Register 0x32 (TP_AIN4) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     TP_AIN4[15:8]                                                                      
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                        TP_AIN4[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_AIN4 register */
    #define BANK1_TP_AIN4_ADDRESS								((uint8_t) 0x32)
    #define BANK1_TP_AIN4_DEFAULT								((uint16_t) 0x0000)

    /* TP_AIN4 field */
    #define BANK1_TP_AIN4_MASK									((uint16_t) 0xFFFF)
    #define BANK1_TP_AIN4_BITOFFSET								(0)


/* Register 0x33 (TP_AIN5) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     TP_AIN5[15:8]                                                                      
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                        TP_AIN5[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_AIN5 register */
    #define BANK1_TP_AIN5_ADDRESS								((uint8_t) 0x33)
    #define BANK1_TP_AIN5_DEFAULT								((uint16_t) 0x0000)

    /* TP_AIN5 field */
    #define BANK1_TP_AIN5_MASK									((uint16_t) 0xFFFF)
    #define BANK1_TP_AIN5_BITOFFSET								(0)


/* Register 0x34 (TP_AIN6) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     TP_AIN6[15:8]                                                                      
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                        TP_AIN6[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_AIN6 register */
    #define BANK1_TP_AIN6_ADDRESS								((uint8_t) 0x34)
    #define BANK1_TP_AIN6_DEFAULT								((uint16_t) 0x0000)

    /* TP_AIN6 field */
    #define BANK1_TP_AIN6_MASK									((uint16_t) 0xFFFF)
    #define BANK1_TP_AIN6_BITOFFSET								(0)


/* Register 0x35 (TP_AIN7) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     TP_AIN7[15:8]                                                                      
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                        TP_AIN7[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_AIN7 register */
    #define BANK1_TP_AIN7_ADDRESS								((uint8_t) 0x35)
    #define BANK1_TP_AIN7_DEFAULT								((uint16_t) 0x0000)

    /* TP_AIN7 field */
    #define BANK1_TP_AIN7_MASK									((uint16_t) 0xFFFF)
    #define BANK1_TP_AIN7_BITOFFSET								(0)


/* Register 0x36 (TP_AIN8) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     TP_AIN8[15:8]                                                                      
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                        TP_AIN8[7:0]                                                                      
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TP_AIN8 register */
    #define BANK1_TP_AIN8_ADDRESS								((uint8_t) 0x36)
    #define BANK1_TP_AIN8_DEFAULT								((uint16_t) 0x0000)

    /* TP_AIN8 field */
    #define BANK1_TP_AIN8_MASK									((uint16_t) 0xFFFF)
    #define BANK1_TP_AIN8_BITOFFSET								(0)


/* Register 0x37 (GEN_CFG5) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                     RESERVED[10:3]                                                                     
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                        RESERVED[2:0]                     |     RESERVED     |            RESERVED[1:0]            |   OFS_CORR_DIS   |   GAN_CORR_DIS   |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GEN_CFG5 register */
    #define BANK1_GEN_CFG5_ADDRESS								((uint8_t) 0x37)
    #define BANK1_GEN_CFG5_DEFAULT								((uint16_t) 0x0000)

    /* OFS_CORR_DIS field */
    #define BANK1_GEN_CFG5_OFS_CORR_DIS_MASK					((uint16_t) 0x0002)
    #define BANK1_GEN_CFG5_OFS_CORR_DIS_BITOFFSET				(1)
    #define BANK1_GEN_CFG5_OFS_CORR_DIS_ENABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_GEN_CFG5_OFS_CORR_DIS_DISABLED				((uint16_t) 0x0002)

    /* GAN_CORR_DIS field */
    #define BANK1_GEN_CFG5_GAN_CORR_DIS_MASK					((uint16_t) 0x0001)
    #define BANK1_GEN_CFG5_GAN_CORR_DIS_BITOFFSET				(0)
    #define BANK1_GEN_CFG5_GAN_CORR_DIS_ENABLED					((uint16_t) 0x0000)    // DEFAULT
    #define BANK1_GEN_CFG5_GAN_CORR_DIS_DISABLED				((uint16_t) 0x0001)


/* Register 0x3E (DWC_FLAG_AIN1_8) definition
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |      Bit 15      |      Bit 14      |      Bit 13      |      Bit 12      |      Bit 11      |      Bit 10      |       Bit 9      |       Bit 8      |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * |  HIGH_FLAG_AIN8  |  HIGH_FLAG_AIN7  |  HIGH_FLAG_AIN6  |  HIGH_FLAG_AIN5  |  HIGH_FLAG_AIN4  |  HIGH_FLAG_AIN3  |  HIGH_FLAG_AIN2  |  HIGH_FLAG_AIN1  |
 * |--------------------------------------------------------------------------------------------------------------------------------------------------------
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       Bit 7      |       Bit 6      |       Bit 5      |       Bit 4      |       Bit 3      |       Bit 2      |       Bit 1      |       Bit 0      |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |   LOW_FLAG_AIN8  |   LOW_FLAG_AIN7  |   LOW_FLAG_AIN6  |   LOW_FLAG_AIN5  |   LOW_FLAG_AIN4  |   LOW_FLAG_AIN3  |   LOW_FLAG_AIN2  |   LOW_FLAG_AIN1  |
 * --------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DWC_FLAG_AIN1_8 register */
    #define BANK1_DWC_FLAG_AIN1_8_ADDRESS						((uint8_t) 0x3E)
    #define BANK1_DWC_FLAG_AIN1_8_DEFAULT						((uint16_t) 0x0000)

    /* HIGH_FLAG_AIN8 field */
    #define BANK1_DWC_FLAG_AIN1_8_HIGH_FLAG_AIN8_MASK			((uint16_t) 0x8000)
    #define BANK1_DWC_FLAG_AIN1_8_HIGH_FLAG_AIN8_BITOFFSET		(15)

    /* HIGH_FLAG_AIN7 field */
    #define BANK1_DWC_FLAG_AIN1_8_HIGH_FLAG_AIN7_MASK			((uint16_t) 0x4000)
    #define BANK1_DWC_FLAG_AIN1_8_HIGH_FLAG_AIN7_BITOFFSET		(14)

    /* HIGH_FLAG_AIN6 field */
    #define BANK1_DWC_FLAG_AIN1_8_HIGH_FLAG_AIN6_MASK			((uint16_t) 0x2000)
    #define BANK1_DWC_FLAG_AIN1_8_HIGH_FLAG_AIN6_BITOFFSET		(13)

    /* HIGH_FLAG_AIN5 field */
    #define BANK1_DWC_FLAG_AIN1_8_HIGH_FLAG_AIN5_MASK			((uint16_t) 0x1000)
    #define BANK1_DWC_FLAG_AIN1_8_HIGH_FLAG_AIN5_BITOFFSET		(12)

    /* HIGH_FLAG_AIN4 field */
    #define BANK1_DWC_FLAG_AIN1_8_HIGH_FLAG_AIN4_MASK			((uint16_t) 0x0800)
    #define BANK1_DWC_FLAG_AIN1_8_HIGH_FLAG_AIN4_BITOFFSET		(11)

    /* HIGH_FLAG_AIN3 field */
    #define BANK1_DWC_FLAG_AIN1_8_HIGH_FLAG_AIN3_MASK			((uint16_t) 0x0400)
    #define BANK1_DWC_FLAG_AIN1_8_HIGH_FLAG_AIN3_BITOFFSET		(10)

    /* HIGH_FLAG_AIN2 field */
    #define BANK1_DWC_FLAG_AIN1_8_HIGH_FLAG_AIN2_MASK			((uint16_t) 0x0200)
    #define BANK1_DWC_FLAG_AIN1_8_HIGH_FLAG_AIN2_BITOFFSET		(9)

    /* HIGH_FLAG_AIN1 field */
    #define BANK1_DWC_FLAG_AIN1_8_HIGH_FLAG_AIN1_MASK			((uint16_t) 0x0100)
    #define BANK1_DWC_FLAG_AIN1_8_HIGH_FLAG_AIN1_BITOFFSET		(8)

    /* LOW_FLAG_AIN8 field */
    #define BANK1_DWC_FLAG_AIN1_8_LOW_FLAG_AIN8_MASK			((uint16_t) 0x0080)
    #define BANK1_DWC_FLAG_AIN1_8_LOW_FLAG_AIN8_BITOFFSET		(7)

    /* LOW_FLAG_AIN7 field */
    #define BANK1_DWC_FLAG_AIN1_8_LOW_FLAG_AIN7_MASK			((uint16_t) 0x0040)
    #define BANK1_DWC_FLAG_AIN1_8_LOW_FLAG_AIN7_BITOFFSET		(6)

    /* LOW_FLAG_AIN6 field */
    #define BANK1_DWC_FLAG_AIN1_8_LOW_FLAG_AIN6_MASK			((uint16_t) 0x0020)
    #define BANK1_DWC_FLAG_AIN1_8_LOW_FLAG_AIN6_BITOFFSET		(5)

    /* LOW_FLAG_AIN5 field */
    #define BANK1_DWC_FLAG_AIN1_8_LOW_FLAG_AIN5_MASK			((uint16_t) 0x0010)
    #define BANK1_DWC_FLAG_AIN1_8_LOW_FLAG_AIN5_BITOFFSET		(4)

    /* LOW_FLAG_AIN4 field */
    #define BANK1_DWC_FLAG_AIN1_8_LOW_FLAG_AIN4_MASK			((uint16_t) 0x0008)
    #define BANK1_DWC_FLAG_AIN1_8_LOW_FLAG_AIN4_BITOFFSET		(3)

    /* LOW_FLAG_AIN3 field */
    #define BANK1_DWC_FLAG_AIN1_8_LOW_FLAG_AIN3_MASK			((uint16_t) 0x0004)
    #define BANK1_DWC_FLAG_AIN1_8_LOW_FLAG_AIN3_BITOFFSET		(2)

    /* LOW_FLAG_AIN2 field */
    #define BANK1_DWC_FLAG_AIN1_8_LOW_FLAG_AIN2_MASK			((uint16_t) 0x0002)
    #define BANK1_DWC_FLAG_AIN1_8_LOW_FLAG_AIN2_BITOFFSET		(1)

    /* LOW_FLAG_AIN1 field */
    #define BANK1_DWC_FLAG_AIN1_8_LOW_FLAG_AIN1_MASK			((uint16_t) 0x0001)
    #define BANK1_DWC_FLAG_AIN1_8_LOW_FLAG_AIN1_BITOFFSET		(0)


#endif /* ADS93XXV_PAGE1_H_ */
