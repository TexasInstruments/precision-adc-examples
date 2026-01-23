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

#ifndef ADS93XXV_PAGE0_H_
#define ADS93XXV_PAGE0_H_

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


/* Register 0x01 (GEN_CFG1) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                    REG_RD_ADD[7:0]                                                                                    |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                              RESERVED[5:0]                                                              |        SW_RST        |       REG_RD_EN      |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GEN_CFG1 register */
    #define BANK0_GEN_CFG1_ADDRESS								((uint8_t) 0x01)
    #define BANK0_GEN_CFG1_DEFAULT								((uint16_t) 0x0000)

    /* REG_RD_ADD field */
    #define BANK0_GEN_CFG1_REG_RD_ADD_MASK						((uint16_t) 0xFF00)
    #define BANK0_GEN_CFG1_REG_RD_ADD_BITOFFSET					(8)

    /* SW_RST field */
    #define BANK0_GEN_CFG1_SW_RST_MASK							((uint16_t) 0x0002)
    #define BANK0_GEN_CFG1_SW_RST_BITOFFSET						(1)

    /* REG_RD_EN field */
    #define BANK0_GEN_CFG1_REG_RD_EN_MASK						((uint16_t) 0x0001)
    #define BANK0_GEN_CFG1_REG_RD_EN_BITOFFSET					(0)


/* Register 0x02 (BANK_SEL) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                     RESERVED[12:5]                                                                                     
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                     RESERVED[4:0]                                                  |                            BANK_SEL[2:0]                           |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* BANK_SEL register */
    #define BANK0_BANK_SEL_ADDRESS								((uint8_t) 0x02)
    #define BANK0_BANK_SEL_DEFAULT								((uint16_t) 0x0001)

    /* BANK_SEL field */
    #define BANK0_BANK_SEL_MASK									((uint16_t) 0x0007)
    #define BANK0_BANK_SEL_BITOFFSET							(0)
    #define BANK0_BANK_SEL_COMMONREGISTERS						((uint16_t) 0x0001)    // DEFAULT
    #define BANK0_BANK_SEL_CHANNELREGISTERSAIN1AIN8				((uint16_t) 0x0002)
    #define BANK0_BANK_SEL_CHANNELREGISTERSAIN9AIN16			((uint16_t) 0x0004)


/* Register 0x07 (DIAG_CTRL) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                      |                      |                      |                      |                      |                      |                      |   RESET_DETECT_FLAG  |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                      |                      |                      |                      |                      |                      |                      |                      |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DIAG_CTRL register */
    #define BANK0_DIAG_CTRL_ADDRESS								((uint8_t) 0x07)
    #define BANK0_DIAG_CTRL_DEFAULT								((uint16_t) 0x0000)

    /* RESET_DETECT_FLAG field */
    #define BANK0_DIAG_CTRL_RESET_DETECT_FLAG_MASK				((uint16_t) 0x0100)
    #define BANK0_DIAG_CTRL_RESET_DETECT_FLAG_BITOFFSET			(8)


/* Register 0x08 (PDN_CTL) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                     RESERVED[14:7]                                                                                     
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                            RESERVED[6:0]                                                                         |      DEVICE_PDN      |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PDN_CTL register */
    #define BANK0_PDN_CTL_ADDRESS								((uint8_t) 0x08)
    #define BANK0_PDN_CTL_DEFAULT								((uint16_t) 0x0000)

    /* DEVICE_PDN field */
    #define BANK0_PDN_CTL_DEVICE_PDN_MASK						((uint16_t) 0x0001)
    #define BANK0_PDN_CTL_DEVICE_PDN_BITOFFSET					(0)
    #define BANK0_PDN_CTL_DEVICE_PDN_NORMALOPERATION			((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_PDN_CTL_DEVICE_PDN_DEVICEISPOWEREDDOWN		((uint16_t) 0x0001)


/* Register 0x09 (GEN_CFG2) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                       RESERVED[3:0]                                       |                                     DAISY_CHN_LOC[3:0]                                    |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                   DAISY_CHN_NUM_DEV[3:0]                                  |                            RESERVED[2:0]                           |     DAISY_CHN_EN     |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GEN_CFG2 register */
    #define BANK0_GEN_CFG2_ADDRESS								((uint8_t) 0x09)
    #define BANK0_GEN_CFG2_DEFAULT								((uint16_t) 0x0000)

    /* DAISY_CHN_LOC field */
    #define BANK0_GEN_CFG2_DAISY_CHN_LOC_MASK					((uint16_t) 0x0F00)
    #define BANK0_GEN_CFG2_DAISY_CHN_LOC_BITOFFSET				(8)

    /* DAISY_CHN_NUM_DEV field */
    #define BANK0_GEN_CFG2_DAISY_CHN_NUM_DEV_MASK				((uint16_t) 0x00F0)
    #define BANK0_GEN_CFG2_DAISY_CHN_NUM_DEV_BITOFFSET			(4)

    /* DAISY_CHN_EN field */
    #define BANK0_GEN_CFG2_DAISY_CHN_EN_MASK					((uint16_t) 0x0001)
    #define BANK0_GEN_CFG2_DAISY_CHN_EN_BITOFFSET				(0)
    #define BANK0_GEN_CFG2_DAISY_CHN_EN_DAISYCHAINCONFIGURATIONDISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_GEN_CFG2_DAISY_CHN_EN_DAISYCHAINCONFIGURATIONENABLED	((uint16_t) 0x0001)


/* Register 0x0A (GEN_CFG3) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                            RESERVED[2:0]                           |     EN_OFS_BINARY    |                RESERVED[1:0]                |      EN_XOR_PATT     |     EN_DIAG_FLAG     |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                RESERVED[1:0]                |              DOUT_LANE_SEL[1:0]             |               DOUT_LENGTH[1:0]              |   ADC_DATA_SDOUT_EN  |       RESERVED       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GEN_CFG3 register */
    #define BANK0_GEN_CFG3_ADDRESS								((uint8_t) 0x0A)
    #define BANK0_GEN_CFG3_DEFAULT								((uint16_t) 0x0000)

    /* EN_OFS_BINARY field */
    #define BANK0_GEN_CFG3_EN_OFS_BINARY_MASK					((uint16_t) 0x1000)
    #define BANK0_GEN_CFG3_EN_OFS_BINARY_BITOFFSET				(12)
    #define BANK0_GEN_CFG3_EN_OFS_BINARY_TWOSCOMPLEMENT			((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_GEN_CFG3_EN_OFS_BINARY_OFFSETBINARY			((uint16_t) 0x1000)

    /* EN_XOR_PATT field */
    #define BANK0_GEN_CFG3_EN_XOR_PATT_MASK						((uint16_t) 0x0200)
    #define BANK0_GEN_CFG3_EN_XOR_PATT_BITOFFSET				(9)

    /* EN_DIAG_FLAG field */
    #define BANK0_GEN_CFG3_EN_DIAG_FLAG_MASK					((uint16_t) 0x0100)
    #define BANK0_GEN_CFG3_EN_DIAG_FLAG_BITOFFSET				(8)

    /* DOUT_LANE_SEL field */
    #define BANK0_GEN_CFG3_DOUT_LANE_SEL_MASK					((uint16_t) 0x0030)
    #define BANK0_GEN_CFG3_DOUT_LANE_SEL_BITOFFSET				(4)
    #define BANK0_GEN_CFG3_DOUT_LANE_SEL_8LANES					((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_GEN_CFG3_DOUT_LANE_SEL_4LANES					((uint16_t) 0x0010)
    #define BANK0_GEN_CFG3_DOUT_LANE_SEL_2LANES					((uint16_t) 0x0020)
    #define BANK0_GEN_CFG3_DOUT_LANE_SEL_1LANE					((uint16_t) 0x0030)

    /* DOUT_LENGTH field */
    #define BANK0_GEN_CFG3_DOUT_LENGTH_MASK						((uint16_t) 0x000C)
    #define BANK0_GEN_CFG3_DOUT_LENGTH_BITOFFSET				(2)
    #define BANK0_GEN_CFG3_DOUT_LENGTH_16BIT					((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_GEN_CFG3_DOUT_LENGTH_RESERVED					((uint16_t) 0x0004)
    #define BANK0_GEN_CFG3_DOUT_LENGTH_24BIT					((uint16_t) 0x0008)

    /* ADC_DATA_SDOUT_EN field */
    #define BANK0_GEN_CFG3_ADC_DATA_SDOUT_EN_MASK				((uint16_t) 0x0002)
    #define BANK0_GEN_CFG3_ADC_DATA_SDOUT_EN_BITOFFSET			(1)
    #define BANK0_GEN_CFG3_ADC_DATA_SDOUT_EN_ADCDATAOUTPUTOND7	((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_GEN_CFG3_ADC_DATA_SDOUT_EN_ADCDATAOUTPUTONSDOUTPIN	((uint16_t) 0x0002)


/* Register 0x0B (XOR_BITS_CTL) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                     RESERVED[7:0]                                                                                     |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                XOR_MODE[1:0]                |              NUM_XOR_BITS[1:0]              |                            RESERVED[2:0]                           |      XOR_BIT_SEL     |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* XOR_BITS_CTL register */
    #define BANK0_XOR_BITS_CTL_ADDRESS							((uint8_t) 0x0B)
    #define BANK0_XOR_BITS_CTL_DEFAULT							((uint16_t) 0x0000)

    /* XOR_MODE field */
    #define BANK0_XOR_BITS_CTL_XOR_MODE_MASK					((uint16_t) 0x00C0)
    #define BANK0_XOR_BITS_CTL_XOR_MODE_BITOFFSET				(6)

    /* NUM_XOR_BITS field */
    #define BANK0_XOR_BITS_CTL_NUM_XOR_BITS_MASK				((uint16_t) 0x0030)
    #define BANK0_XOR_BITS_CTL_NUM_XOR_BITS_BITOFFSET			(4)

    /* XOR_BIT_SEL field */
    #define BANK0_XOR_BITS_CTL_XOR_BIT_SEL_MASK					((uint16_t) 0x0001)
    #define BANK0_XOR_BITS_CTL_XOR_BIT_SEL_BITOFFSET			(0)


/* Register 0x0C (DRDY_ALARM_SEL) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                     ALRM_MASK[7:0]                                                                                    |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                RESERVED[1:0]                |       ALRM_TYPE      |       ALRM_POL       |                                     DRDY_ALRM_SEL[3:0]                                    |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DRDY_ALARM_SEL register */
    #define BANK0_DRDY_ALARM_SEL_ADDRESS						((uint8_t) 0x0C)
    #define BANK0_DRDY_ALARM_SEL_DEFAULT						((uint16_t) 0x0000)

    /* ALRM_MASK field */
    #define BANK0_DRDY_ALARM_SEL_ALRM_MASK_MASK					((uint16_t) 0xFF00)
    #define BANK0_DRDY_ALARM_SEL_ALRM_MASK_BITOFFSET			(8)

    /* ALRM_TYPE field */
    #define BANK0_DRDY_ALARM_SEL_ALRM_TYPE_MASK					((uint16_t) 0x0020)
    #define BANK0_DRDY_ALARM_SEL_ALRM_TYPE_BITOFFSET			(5)
    #define BANK0_DRDY_ALARM_SEL_ALRM_TYPE_LEVELBASED			((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DRDY_ALARM_SEL_ALRM_TYPE_PULSEBASED			((uint16_t) 0x0020)

    /* ALRM_POL field */
    #define BANK0_DRDY_ALARM_SEL_ALRM_POL_MASK					((uint16_t) 0x0010)
    #define BANK0_DRDY_ALARM_SEL_ALRM_POL_BITOFFSET				(4)
    #define BANK0_DRDY_ALARM_SEL_ALRM_POL_ACTIVEHIGH			((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DRDY_ALARM_SEL_ALRM_POL_ACTIVELOW				((uint16_t) 0x0010)

    /* DRDY_ALRM_SEL field */
    #define BANK0_DRDY_ALARM_SEL_DRDY_ALRM_SEL_MASK				((uint16_t) 0x000F)
    #define BANK0_DRDY_ALARM_SEL_DRDY_ALRM_SEL_BITOFFSET		(0)
    #define BANK0_DRDY_ALARM_SEL_DRDY_ALRM_SEL_ADCDATAREADYFLAG	((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DRDY_ALARM_SEL_DRDY_ALRM_SEL_DWCOUTPUT		((uint16_t) 0x0001)
    #define BANK0_DRDY_ALARM_SEL_DRDY_ALRM_SEL_RESERVED0			((uint16_t) 0x0002)
    #define BANK0_DRDY_ALARM_SEL_DRDY_ALRM_SEL_RESERVED1			((uint16_t) 0x0003)
    #define BANK0_DRDY_ALARM_SEL_DRDY_ALRM_SEL_RESERVED2			((uint16_t) 0x0004)
    #define BANK0_DRDY_ALARM_SEL_DRDY_ALRM_SEL_RESERVED3			((uint16_t) 0x0005)
    #define BANK0_DRDY_ALARM_SEL_DRDY_ALRM_SEL_ADC_CALDONEFLAG	((uint16_t) 0x0006)
    #define BANK0_DRDY_ALARM_SEL_DRDY_ALRM_SEL_RESERVED4			((uint16_t) 0x0007)
    #define BANK0_DRDY_ALARM_SEL_DRDY_ALRM_SEL_OREDOFALLABOVEFLAGSWITHINVIDUALMASKBEFOREORING	((uint16_t) 0x0008)


/* Register 0x0D (GEN_CFG4) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                     RESERVED[9:2]                                                                                      
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                  RESERVED[1:0]                |       ALRM_DIS       |       RESERVED       |     DIG_DELAY_EN     |       RESERVED       |             DRIVE_STRENGTH[1:0]             |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GEN_CFG4 register */
    #define BANK0_GEN_CFG4_ADDRESS								((uint8_t) 0x0D)
    #define BANK0_GEN_CFG4_DEFAULT								((uint16_t) 0x0000)

    /* ALRM_DIS field */
    #define BANK0_GEN_CFG4_ALRM_DIS_MASK						((uint16_t) 0x0020)
    #define BANK0_GEN_CFG4_ALRM_DIS_BITOFFSET					(5)
    #define BANK0_GEN_CFG4_ALRM_DIS_ENABLED						((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_GEN_CFG4_ALRM_DIS_DISABLED					((uint16_t) 0x0020)

    /* DIG_DELAY_EN field */
    #define BANK0_GEN_CFG4_DIG_DELAY_EN_MASK					((uint16_t) 0x0008)
    #define BANK0_GEN_CFG4_DIG_DELAY_EN_BITOFFSET				(3)

    /* DRIVE_STRENGTH field */
    #define BANK0_GEN_CFG4_DRIVE_STRENGTH_MASK					((uint16_t) 0x0003)
    #define BANK0_GEN_CFG4_DRIVE_STRENGTH_BITOFFSET				(0)
    #define BANK0_GEN_CFG4_DRIVE_STRENGTH_NORMALDEVICEOPERATION	((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_GEN_CFG4_DRIVE_STRENGTH_0						((uint16_t) 0x0001)
    #define BANK0_GEN_CFG4_DRIVE_STRENGTH_2XDRIVESTRENGTH		((uint16_t) 0x0002)
    #define BANK0_GEN_CFG4_DRIVE_STRENGTH_1						((uint16_t) 0x0003)


/* Register 0x0E (DIG_DELAY_CFG1) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |       RESERVED       |                        DIG_DELAY_SDOUT[2:0]                        |                          DIG_DELAY_D3[2:0]                         |   DIG_DELAY_D2[2:2]   
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                DIG_DELAY_D2[1:0]              |                          DIG_DELAY_D1[2:0]                         |                          DIG_DELAY_D0[2:0]                         |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DIG_DELAY_CFG1 register */
    #define BANK0_DIG_DELAY_CFG1_ADDRESS						((uint8_t) 0x0E)
    #define BANK0_DIG_DELAY_CFG1_DEFAULT						((uint16_t) 0x0000)

    /* DIG_DELAY_SDOUT field */
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_SDOUT_MASK			((uint16_t) 0x7000)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_SDOUT_BITOFFSET		(12)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_SDOUT_0NSDELAY		((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_SDOUT_1NSDELAY		((uint16_t) 0x1000)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_SDOUT_2NSDELAY		((uint16_t) 0x2000)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_SDOUT_3NSDELAY		((uint16_t) 0x3000)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_SDOUT_4NSDELAY		((uint16_t) 0x4000)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_SDOUT_5NSDELAY		((uint16_t) 0x5000)

    /* DIG_DELAY_D3 field */
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D3_MASK				((uint16_t) 0x0E00)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D3_BITOFFSET			(9)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D3_0NSDELAY			((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D3_1NSDELAY			((uint16_t) 0x0200)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D3_2NSDELAY			((uint16_t) 0x0400)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D3_3NSDELAY			((uint16_t) 0x0600)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D3_4NSDELAY			((uint16_t) 0x0800)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D3_5NSDELAY			((uint16_t) 0x0A00)

    /* DIG_DELAY_D2 field */
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D2_MASK				((uint16_t) 0x01C0)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D2_BITOFFSET			(6)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D2_0NSDELAY			((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D2_1NSDELAY			((uint16_t) 0x0040)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D2_2NSDELAY			((uint16_t) 0x0080)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D2_3NSDELAY			((uint16_t) 0x00C0)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D2_4NSDELAY			((uint16_t) 0x0100)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D2_5NSDELAY			((uint16_t) 0x0140)

    /* DIG_DELAY_D1 field */
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D1_MASK				((uint16_t) 0x0038)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D1_BITOFFSET			(3)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D1_0NSDELAY			((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D1_1NSDELAY			((uint16_t) 0x0008)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D1_2NSDELAY			((uint16_t) 0x0010)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D1_3NSDELAY			((uint16_t) 0x0018)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D1_4NSDELAY			((uint16_t) 0x0020)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D1_5NSDELAY			((uint16_t) 0x0028)

    /* DIG_DELAY_D0 field */
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D0_MASK				((uint16_t) 0x0007)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D0_BITOFFSET			(0)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D0_0NSDELAY			((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D0_1NSDELAY			((uint16_t) 0x0001)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D0_2NSDELAY			((uint16_t) 0x0002)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D0_3NSDELAY			((uint16_t) 0x0003)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D0_4NSDELAY			((uint16_t) 0x0004)
    #define BANK0_DIG_DELAY_CFG1_DIG_DELAY_D0_5NSDELAY			((uint16_t) 0x0005)


/* Register 0x0F (DIG_DELAY_CFG2) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                       RESERVED[3:0]                                       |                          DIG_DELAY_D7[2:0]                         |   DIG_DELAY_D6[2:2]   
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                DIG_DELAY_D6[1:0]              |                          DIG_DELAY_D5[2:0]                         |                          DIG_DELAY_D4[2:0]                         |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DIG_DELAY_CFG2 register */
    #define BANK0_DIG_DELAY_CFG2_ADDRESS						((uint8_t) 0x0F)
    #define BANK0_DIG_DELAY_CFG2_DEFAULT						((uint16_t) 0x0000)

    /* DIG_DELAY_D7 field */
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D7_MASK				((uint16_t) 0x0E00)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D7_BITOFFSET			(9)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D7_0NSDELAY			((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D7_1NSDELAY			((uint16_t) 0x0200)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D7_2NSDELAY			((uint16_t) 0x0400)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D7_3NSDELAY			((uint16_t) 0x0600)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D7_4NSDELAY			((uint16_t) 0x0800)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D7_5NSDELAY			((uint16_t) 0x0A00)

    /* DIG_DELAY_D6 field */
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D6_MASK				((uint16_t) 0x01C0)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D6_BITOFFSET			(6)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D6_0NSDELAY			((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D6_1NSDELAY			((uint16_t) 0x0040)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D6_2NSDELAY			((uint16_t) 0x0080)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D6_3NSDELAY			((uint16_t) 0x00C0)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D6_4NSDELAY			((uint16_t) 0x0100)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D6_5NSDELAY			((uint16_t) 0x0140)

    /* DIG_DELAY_D5 field */
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D5_MASK				((uint16_t) 0x0038)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D5_BITOFFSET			(3)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D5_0NSDELAY			((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D5_1NSDELAY			((uint16_t) 0x0008)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D5_2NSDELAY			((uint16_t) 0x0010)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D5_3NSDELAY			((uint16_t) 0x0018)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D5_4NSDELAY			((uint16_t) 0x0020)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D5_5NSDELAY			((uint16_t) 0x0028)

    /* DIG_DELAY_D4 field */
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D4_MASK				((uint16_t) 0x0007)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D4_BITOFFSET			(0)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D4_0NSDELAY			((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D4_1NSDELAY			((uint16_t) 0x0001)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D4_2NSDELAY			((uint16_t) 0x0002)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D4_3NSDELAY			((uint16_t) 0x0003)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D4_4NSDELAY			((uint16_t) 0x0004)
    #define BANK0_DIG_DELAY_CFG2_DIG_DELAY_D4_5NSDELAY			((uint16_t) 0x0005)


/* Register 0x10 (ANA_CFG1) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                     RESERVED[13:6]                                                                                     
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                                                RESERVED[5:0]                                                              |    REFSEL_CTRL_DIS   |      EXT_REF_EN      |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* ANA_CFG1 register */
    #define BANK0_ANA_CFG1_ADDRESS								((uint8_t) 0x10)
    #define BANK0_ANA_CFG1_DEFAULT								((uint16_t) 0x0000)

    /* REFSEL_CTRL_DIS field */
    #define BANK0_ANA_CFG1_REFSEL_CTRL_DIS_MASK					((uint16_t) 0x0002)
    #define BANK0_ANA_CFG1_REFSEL_CTRL_DIS_BITOFFSET			(1)
    #define BANK0_ANA_CFG1_REFSEL_CTRL_DIS_ENABLED				((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_ANA_CFG1_REFSEL_CTRL_DIS_DISABLED				((uint16_t) 0x0002)

    /* EXT_REF_EN field */
    #define BANK0_ANA_CFG1_EXT_REF_EN_MASK						((uint16_t) 0x0001)
    #define BANK0_ANA_CFG1_EXT_REF_EN_BITOFFSET					(0)
    #define BANK0_ANA_CFG1_EXT_REF_EN_INTERNALREFERENCE			((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_ANA_CFG1_EXT_REF_EN_EXTERNALREFERENCE			((uint16_t) 0x0001)


/* Register 0x11 (ANA_CFG2) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |  CH_XTALK_LOW_SPEED  |                                                                          RESERVED[7:1]                                                                          
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *       RESERVED[0:0]    |                           ADC_CH_SEL[2:0]                          |               ADC_NUM_SEL[1:0]              |                RESERVED[1:0]                |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* ANA_CFG2 register */
    #define BANK0_ANA_CFG2_ADDRESS								((uint8_t) 0x11)
    #define BANK0_ANA_CFG2_DEFAULT								((uint16_t) 0x0000)

    /* CH_XTALK_LOW_SPEED field */
    #define BANK0_ANA_CFG2_CH_XTALK_LOW_SPEED_MASK				((uint16_t) 0x8000)
    #define BANK0_ANA_CFG2_CH_XTALK_LOW_SPEED_BITOFFSET			(15)

    /* ADC_CH_SEL field */
    #define BANK0_ANA_CFG2_ADC_CH_SEL_MASK						((uint16_t) 0x0070)
    #define BANK0_ANA_CFG2_ADC_CH_SEL_BITOFFSET					(4)

    /* ADC_NUM_SEL field */
    #define BANK0_ANA_CFG2_ADC_NUM_SEL_MASK						((uint16_t) 0x000C)
    #define BANK0_ANA_CFG2_ADC_NUM_SEL_BITOFFSET				(2)
    #define BANK0_ANA_CFG2_ADC_NUM_SEL_16CHANNELS				((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_ANA_CFG2_ADC_NUM_SEL_8CHANNELS				((uint16_t) 0x0004)
    #define BANK0_ANA_CFG2_ADC_NUM_SEL_4CHANNELS				((uint16_t) 0x0008)
    #define BANK0_ANA_CFG2_ADC_NUM_SEL_2CHANNELS				((uint16_t) 0x000C)


/* Register 0x12 (ADC_CAL) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                RESERVED[1:0]                |  SE_DIFF_MODE_AIN1_8 | SE_DIFF_MODE_AIN9_16 |                            RESERVED[2:0]                           |     ADC_CAL_TRIG     |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                              RESERVED[5:0]                                                              |              ADC_CAL_MODE[1:0]              |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* ADC_CAL register */
    #define BANK0_ADC_CAL_ADDRESS								((uint8_t) 0x12)
    #define BANK0_ADC_CAL_DEFAULT								((uint16_t) 0x0000)

    /* SE_DIFF_MODE_AIN1_8 field */
    #define BANK0_ADC_CAL_SE_DIFF_MODE_AIN1_8_MASK				((uint16_t) 0x2000)
    #define BANK0_ADC_CAL_SE_DIFF_MODE_AIN1_8_BITOFFSET			(13)
    #define BANK0_ADC_CAL_SE_DIFF_MODE_AIN1_8_SINGLEENDEDINPUTSIGNALSONAIN1TOAIN8	((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_ADC_CAL_SE_DIFF_MODE_AIN1_8_DIFFERENTIALINPUTSIGNALSONAIN1TOAIN8	((uint16_t) 0x2000)

    /* SE_DIFF_MODE_AIN9_16 field */
    #define BANK0_ADC_CAL_SE_DIFF_MODE_AIN9_16_MASK				((uint16_t) 0x1000)
    #define BANK0_ADC_CAL_SE_DIFF_MODE_AIN9_16_BITOFFSET		(12)
    #define BANK0_ADC_CAL_SE_DIFF_MODE_AIN9_16_SINGLEENDEDINPUTSIGNALSONAIN9TOAIN16	((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_ADC_CAL_SE_DIFF_MODE_AIN9_16_DIFFERENTIALINPUTSIGNALSONAIN9TOAIN16	((uint16_t) 0x1000)

    /* ADC_CAL_TRIG field */
    #define BANK0_ADC_CAL_ADC_CAL_TRIG_MASK						((uint16_t) 0x0100)
    #define BANK0_ADC_CAL_ADC_CAL_TRIG_BITOFFSET				(8)

    /* ADC_CAL_MODE field */
    #define BANK0_ADC_CAL_ADC_CAL_MODE_MASK						((uint16_t) 0x0003)
    #define BANK0_ADC_CAL_ADC_CAL_MODE_BITOFFSET				(0)
    #define BANK0_ADC_CAL_ADC_CAL_MODE_CALIBRATIONISDISABLED	((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_ADC_CAL_ADC_CAL_MODE_OFFSETERRORCALIBRATION	((uint16_t) 0x0001)
    #define BANK0_ADC_CAL_ADC_CAL_MODE_GAINERRORCALIBRATION		((uint16_t) 0x0002)
    #define BANK0_ADC_CAL_ADC_CAL_MODE_OFFSETANDGAINERRORCALIBRATION	((uint16_t) 0x0003)


/* Register 0x14 (DIG_FILTER) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |    PHASE_DELAY_EN    |                          FIR_FILT_SEL[2:0]                         |                                      MVG_AVG_LEN[3:0]                                     |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                      BLK_AVG_OSR[3:0]                                     |                RESERVED[1:0]                |     INT_TRIG_MODE    |    DIG_FILT_SYSREF   |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DIG_FILTER register */
    #define BANK0_DIG_FILTER_ADDRESS							((uint8_t) 0x14)
    #define BANK0_DIG_FILTER_DEFAULT							((uint16_t) 0x0000)

    /* PHASE_DELAY_EN field */
    #define BANK0_DIG_FILTER_PHASE_DELAY_EN_MASK				((uint16_t) 0x8000)
    #define BANK0_DIG_FILTER_PHASE_DELAY_EN_BITOFFSET			(15)
    #define BANK0_DIG_FILTER_PHASE_DELAY_EN_DISABLED			((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DIG_FILTER_PHASE_DELAY_EN_ENABLED				((uint16_t) 0x8000)

    /* FIR_FILT_SEL field */
    #define BANK0_DIG_FILTER_FIR_FILT_SEL_MASK					((uint16_t) 0x7000)
    #define BANK0_DIG_FILTER_FIR_FILT_SEL_BITOFFSET				(12)
    #define BANK0_DIG_FILTER_FIR_FILT_SEL_DISABLED				((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DIG_FILTER_FIR_FILT_SEL_FIR1					((uint16_t) 0x1000)
    #define BANK0_DIG_FILTER_FIR_FILT_SEL_FIR2					((uint16_t) 0x2000)
    #define BANK0_DIG_FILTER_FIR_FILT_SEL_FIR3					((uint16_t) 0x3000)
    #define BANK0_DIG_FILTER_FIR_FILT_SEL_FIR4					((uint16_t) 0x4000)
    #define BANK0_DIG_FILTER_FIR_FILT_SEL_RESERVED				((uint16_t) 0x5000)
    #define BANK0_DIG_FILTER_FIR_FILT_SEL_FIR6					((uint16_t) 0x6000)
    #define BANK0_DIG_FILTER_FIR_FILT_SEL_FIR7					((uint16_t) 0x7000)

    /* MVG_AVG_LEN field */
    #define BANK0_DIG_FILTER_MVG_AVG_LEN_MASK					((uint16_t) 0x0F00)
    #define BANK0_DIG_FILTER_MVG_AVG_LEN_BITOFFSET				(8)
    #define BANK0_DIG_FILTER_MVG_AVG_LEN_NOAVERAGING			((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DIG_FILTER_MVG_AVG_LEN_2SAMPLESAVERAGED		((uint16_t) 0x0100)
    #define BANK0_DIG_FILTER_MVG_AVG_LEN_4SAMPLESAVERAGED		((uint16_t) 0x0200)
    #define BANK0_DIG_FILTER_MVG_AVG_LEN_6SAMPLESAVERAGED		((uint16_t) 0x0300)
    #define BANK0_DIG_FILTER_MVG_AVG_LEN_8SAMPLESAVERAGED		((uint16_t) 0x0400)
    #define BANK0_DIG_FILTER_MVG_AVG_LEN_10SAMPLESAVERAGED		((uint16_t) 0x0500)
    #define BANK0_DIG_FILTER_MVG_AVG_LEN_12SAMPLESAVERAGED		((uint16_t) 0x0600)
    #define BANK0_DIG_FILTER_MVG_AVG_LEN_16SAMPLESAVERAGED		((uint16_t) 0x0700)
    #define BANK0_DIG_FILTER_MVG_AVG_LEN_20SAMPLESAVERAGED		((uint16_t) 0x0800)
    #define BANK0_DIG_FILTER_MVG_AVG_LEN_32SAMPLESAVERAGED		((uint16_t) 0x0900)
    #define BANK0_DIG_FILTER_MVG_AVG_LEN_64SAMPLESAVERAGED		((uint16_t) 0x0A00)
    #define BANK0_DIG_FILTER_MVG_AVG_LEN_128SAMPLESAVERAGED		((uint16_t) 0x0B00)

    /* BLK_AVG_OSR field */
    #define BANK0_DIG_FILTER_BLK_AVG_OSR_MASK					((uint16_t) 0x00F0)
    #define BANK0_DIG_FILTER_BLK_AVG_OSR_BITOFFSET				(4)
    #define BANK0_DIG_FILTER_BLK_AVG_OSR_NOAVERAGING			((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DIG_FILTER_BLK_AVG_OSR_2SAMPLESAVERAGED		((uint16_t) 0x0010)
    #define BANK0_DIG_FILTER_BLK_AVG_OSR_4SAMPLESAVERAGED		((uint16_t) 0x0020)
    #define BANK0_DIG_FILTER_BLK_AVG_OSR_6SAMPLESAVERAGED		((uint16_t) 0x0030)
    #define BANK0_DIG_FILTER_BLK_AVG_OSR_8SAMPLESAVERAGED		((uint16_t) 0x0040)
    #define BANK0_DIG_FILTER_BLK_AVG_OSR_10SAMPLESAVERAGED		((uint16_t) 0x0050)
    #define BANK0_DIG_FILTER_BLK_AVG_OSR_12SAMPLESAVERAGED		((uint16_t) 0x0060)
    #define BANK0_DIG_FILTER_BLK_AVG_OSR_16SAMPLESAVERAGED		((uint16_t) 0x0070)
    #define BANK0_DIG_FILTER_BLK_AVG_OSR_20SAMPLESAVERAGED		((uint16_t) 0x0080)
    #define BANK0_DIG_FILTER_BLK_AVG_OSR_32SAMPLESAVERAGED		((uint16_t) 0x0090)
    #define BANK0_DIG_FILTER_BLK_AVG_OSR_64SAMPLESAVERAGED		((uint16_t) 0x00A0)
    #define BANK0_DIG_FILTER_BLK_AVG_OSR_128SAMPLESAVERAGED		((uint16_t) 0x00B0)

    /* INT_TRIG_MODE field */
    #define BANK0_DIG_FILTER_INT_TRIG_MODE_MASK					((uint16_t) 0x0002)
    #define BANK0_DIG_FILTER_INT_TRIG_MODE_BITOFFSET			(1)
    #define BANK0_DIG_FILTER_INT_TRIG_MODE_DISABLED				((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DIG_FILTER_INT_TRIG_MODE_ENABLED				((uint16_t) 0x0002)

    /* DIG_FILT_SYSREF field */
    #define BANK0_DIG_FILTER_DIG_FILT_SYSREF_MASK				((uint16_t) 0x0001)
    #define BANK0_DIG_FILTER_DIG_FILT_SYSREF_BITOFFSET			(0)


/* Register 0x15 (GEN_CFG5) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                     RESERVED[11:4]                                                                                     
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                         RESERVED[3:0]                                       |     T_MODE_OVR_EN    |        T_MODE        |    AVG_MODE_OVR_EN   |       AVG_MODE       |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GEN_CFG5 register */
    #define BANK0_GEN_CFG5_ADDRESS								((uint8_t) 0x15)
    #define BANK0_GEN_CFG5_DEFAULT								((uint16_t) 0x0000)

    /* T_MODE_OVR_EN field */
    #define BANK0_GEN_CFG5_T_MODE_OVR_EN_MASK					((uint16_t) 0x0008)
    #define BANK0_GEN_CFG5_T_MODE_OVR_EN_BITOFFSET				(3)

    /* T_MODE field */
    #define BANK0_GEN_CFG5_T_MODE_MASK							((uint16_t) 0x0004)
    #define BANK0_GEN_CFG5_T_MODE_BITOFFSET						(2)

    /* AVG_MODE_OVR_EN field */
    #define BANK0_GEN_CFG5_AVG_MODE_OVR_EN_MASK					((uint16_t) 0x0002)
    #define BANK0_GEN_CFG5_AVG_MODE_OVR_EN_BITOFFSET			(1)

    /* AVG_MODE field */
    #define BANK0_GEN_CFG5_AVG_MODE_MASK						((uint16_t) 0x0001)
    #define BANK0_GEN_CFG5_AVG_MODE_BITOFFSET					(0)


/* Register 0x1D (DEVICE_STATUS) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                     RESERVED[11:4]                                                                                     
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 *                                         RESERVED[3:0]                                       |    CALIB_BUSY_FLAG   |                            RESERVED[2:0]                           |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DEVICE_STATUS register */
    #define BANK0_DEVICE_STATUS_ADDRESS							((uint8_t) 0x1D)
    #define BANK0_DEVICE_STATUS_DEFAULT							((uint16_t) 0x0000)

    /* CALIB_BUSY_FLAG field */
    #define BANK0_DEVICE_STATUS_CALIB_BUSY_FLAG_MASK			((uint16_t) 0x0008)
    #define BANK0_DEVICE_STATUS_CALIB_BUSY_FLAG_BITOFFSET		(3)
    #define BANK0_DEVICE_STATUS_CALIB_BUSY_FLAG_ADC_CALMODULEISIDLE	((uint16_t) 0x0000)    // DEFAULT
    #define BANK0_DEVICE_STATUS_CALIB_BUSY_FLAG_ADC_CALMODULEISRUNNING	((uint16_t) 0x0008)


/* Register 0x21 (DEVICE_ID) definition
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |        Bit 15        |        Bit 14        |        Bit 13        |        Bit 12        |        Bit 11        |        Bit 10        |         Bit 9        |         Bit 8        |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * |                                                                                     RESERVED[7:0]                                                                                     |
 * |----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                     DEVICE_ID[7:0]                                                                                    |
 * ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DEVICE_ID register */
    #define BANK0_DEVICE_ID_ADDRESS								((uint8_t) 0x21)
    #define BANK0_DEVICE_ID_DEFAULT								((uint16_t) 0x0004)

    /* DEVICE_ID field */
    #define BANK0_DEVICE_ID_MASK								((uint16_t) 0x00FF)
    #define BANK0_DEVICE_ID_BITOFFSET							(0)


#endif /* ADS93XXV_PAGE0_H_ */
