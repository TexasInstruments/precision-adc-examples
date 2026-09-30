/*
 * @file ads124s18_page1.h
 *
 * @brief ADS124S18 Descriptor
 *
 * @copyright Copyright (C) 2026 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef ADS124S18_PAGE1_H_
#define ADS124S18_PAGE1_H_

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


/* Register 0x00 (STEPX_AINP_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                             RESERVED[2:0]                             |                                                    STEPx_AINP[4:0]                                                    |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_AINP_CFG register */
    #define STEPX_AINP_CFG_ADDRESS								((uint8_t) 0x00)
    #define STEPX_AINP_CFG_DEFAULT								((uint8_t) 0x12)

    /* STEPx_AINP field */
    #define STEPX_AINP_MASK										((uint8_t) 0x1F)
    #define STEPX_AINP_CFG_STEPX_AINP_BITOFFSET					(0)
    #define STEPX_AINP_AIN0										((uint8_t) 0x00)
    #define STEPX_AINP_AIN1										((uint8_t) 0x01)
    #define STEPX_AINP_AIN2										((uint8_t) 0x02)
    #define STEPX_AINP_AIN3										((uint8_t) 0x03)
    #define STEPX_AINP_AIN4										((uint8_t) 0x04)
    #define STEPX_AINP_AIN5										((uint8_t) 0x05)
    #define STEPX_AINP_AIN6										((uint8_t) 0x06)
    #define STEPX_AINP_AIN7										((uint8_t) 0x07)
    #define STEPX_AINP_AIN8AVSSADS1X4S14						((uint8_t) 0x08)
    #define STEPX_AINP_AIN9AVSSADS1X4S14						((uint8_t) 0x09)
    #define STEPX_AINP_AIN10AVSSADS1X4S14						((uint8_t) 0x0A)
    #define STEPX_AINP_AIN11AVSSADS1X4S14						((uint8_t) 0x0B)
    #define STEPX_AINP_AIN12AVSSADS1X4S14						((uint8_t) 0x0C)
    #define STEPX_AINP_AIN13AVSSADS1X4S14						((uint8_t) 0x0D)
    #define STEPX_AINP_AIN14AVSSADS1X4S14						((uint8_t) 0x0E)
    #define STEPX_AINP_AIN15AVSSADS1X4S14						((uint8_t) 0x0F)
    #define STEPX_AINP_AIN16AVSSADS1X4S14						((uint8_t) 0x10)
    #define STEPX_AINP_AIN17AVSSADS1X4S14						((uint8_t) 0x11)
    #define STEPX_AINP_AVSS										((uint8_t) 0x12)    // DEFAULT
    #define STEPX_AINP_OPEN										((uint8_t) 0x1F)


/* Register 0x01 (STEPX_AINN_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                             RESERVED[2:0]                             |                                                    STEPx_AINN[4:0]                                                    |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_AINN_CFG register */
    #define STEPX_AINN_CFG_ADDRESS								((uint8_t) 0x01)
    #define STEPX_AINN_CFG_DEFAULT								((uint8_t) 0x12)

    /* STEPx_AINN field */
    #define STEPX_AINN_MASK										((uint8_t) 0x1F)
    #define STEPX_AINN_CFG_STEPX_AINN_BITOFFSET					(0)
    #define STEPX_AINN_AIN0										((uint8_t) 0x00)
    #define STEPX_AINN_AIN1										((uint8_t) 0x01)
    #define STEPX_AINN_AIN2										((uint8_t) 0x02)
    #define STEPX_AINN_AIN3										((uint8_t) 0x03)
    #define STEPX_AINN_AIN4										((uint8_t) 0x04)
    #define STEPX_AINN_AIN5										((uint8_t) 0x05)
    #define STEPX_AINN_AIN6										((uint8_t) 0x06)
    #define STEPX_AINN_AIN7										((uint8_t) 0x07)
    #define STEPX_AINN_AIN8AVSSADS1X4S14						((uint8_t) 0x08)
    #define STEPX_AINN_AIN9AVSSADS1X4S14						((uint8_t) 0x09)
    #define STEPX_AINN_AIN10AVSSADS1X4S14						((uint8_t) 0x0A)
    #define STEPX_AINN_AIN11AVSSADS1X4S14						((uint8_t) 0x0B)
    #define STEPX_AINN_AIN12AVSSADS1X4S14						((uint8_t) 0x0C)
    #define STEPX_AINN_AIN13AVSSADS1X4S14						((uint8_t) 0x0D)
    #define STEPX_AINN_AIN14AVSSADS1X4S14						((uint8_t) 0x0E)
    #define STEPX_AINN_AIN15AVSSADS1X4S14						((uint8_t) 0x0F)
    #define STEPX_AINN_AIN16AVSSADS1X4S14						((uint8_t) 0x10)
    #define STEPX_AINN_AIN17AVSSADS1X4S14						((uint8_t) 0x11)
    #define STEPX_AINN_AVSS										((uint8_t) 0x12)    // DEFAULT
    #define STEPX_AINN_OPEN										((uint8_t) 0x1F)


/* Register 0x02 (STEPX_GAIN_REF_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |   STEPx_REFP_BUF_EN   |   STEPx_REFN_BUF_EN   |               STEPx_REF_SEL[1:0]              |                                        STEPx_GAIN[3:0]                                        |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_GAIN_REF_CFG register */
    #define STEPX_GAIN_REF_CFG_ADDRESS							((uint8_t) 0x02)
    #define STEPX_GAIN_REF_CFG_DEFAULT							((uint8_t) 0x01)

    /* STEPx_REFP_BUF_EN field */
    #define STEPX_REFP_BUF_EN_MASK								((uint8_t) 0x80)
    #define STEPX_GAIN_REF_CFG_STEPX_REFP_BUF_EN_BITOFFSET		(7)
    #define STEPX_REFP_BUF_EN_DISABLED							((uint8_t) 0x00)    // DEFAULT
    #define STEPX_REFP_BUF_EN_ENABLED							((uint8_t) 0x80)

    /* STEPx_REFN_BUF_EN field */
    #define STEPX_REFN_BUF_EN_MASK								((uint8_t) 0x40)
    #define STEPX_GAIN_REF_CFG_STEPX_REFN_BUF_EN_BITOFFSET		(6)
    #define STEPX_REFN_BUF_EN_DISABLED							((uint8_t) 0x00)    // DEFAULT
    #define STEPX_REFN_BUF_EN_ENABLED							((uint8_t) 0x40)

    /* STEPx_REF_SEL field */
    #define STEPX_REF_SEL_MASK									((uint8_t) 0x30)
    #define STEPX_GAIN_REF_CFG_STEPX_REF_SEL_BITOFFSET			(4)
    #define STEPX_REF_SEL_INTERNALVOLTAGEREFERENCE				((uint8_t) 0x00)    // DEFAULT
    #define STEPX_REF_SEL_EXTERNALVOLTAGEREFERENCE				((uint8_t) 0x10)
    #define STEPX_REF_SEL_AVDDDASHAVSS							((uint8_t) 0x30)

    /* STEPx_GAIN field */
    #define STEPX_GAIN_MASK										((uint8_t) 0x0F)
    #define STEPX_GAIN_REF_CFG_STEPX_GAIN_BITOFFSET				(0)
    #define STEPX_GAIN_0										((uint8_t) 0x00)
    #define STEPX_GAIN_1										((uint8_t) 0x01)    // DEFAULT
    #define STEPX_GAIN_2										((uint8_t) 0x02)
    #define STEPX_GAIN_4										((uint8_t) 0x03)
    #define STEPX_GAIN_5										((uint8_t) 0x04)
    #define STEPX_GAIN_8										((uint8_t) 0x05)
    #define STEPX_GAIN_10										((uint8_t) 0x06)
    #define STEPX_GAIN_16										((uint8_t) 0x07)
    #define STEPX_GAIN_20										((uint8_t) 0x08)
    #define STEPX_GAIN_32										((uint8_t) 0x09)
    #define STEPX_GAIN_50										((uint8_t) 0x0A)
    #define STEPX_GAIN_64										((uint8_t) 0x0B)
    #define STEPX_GAIN_100										((uint8_t) 0x0C)
    #define STEPX_GAIN_128										((uint8_t) 0x0D)
    #define STEPX_GAIN_200										((uint8_t) 0x0E)
    #define STEPX_GAIN_256										((uint8_t) 0x0F)


/* Register 0x03 (STEPX_DIGITAL_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                   SPARE[1:0]                  |      STEPx_CODING     |                                                  STEPx_NUM_CONV[4:0]                                                  |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_DIGITAL_CFG register */
    #define STEPX_DIGITAL_CFG_ADDRESS							((uint8_t) 0x03)
    #define STEPX_DIGITAL_CFG_DEFAULT							((uint8_t) 0x00)

    /* SPARE field */
    #define STEPX_SPARE_MASK									((uint8_t) 0xC0)
    #define STEPX_DIGITAL_CFG_SPARE_BITOFFSET					(6)

    /* STEPx_CODING field */
    #define STEPX_CODING_MASK									((uint8_t) 0x20)
    #define STEPX_DIGITAL_CFG_STEPX_CODING_BITOFFSET			(5)
    #define STEPX_CODING_BINARYTWOSCOMPLEMENT					((uint8_t) 0x00)    // DEFAULT
    #define STEPX_CODING_UNIPOLARSTRAIGHTBINARY					((uint8_t) 0x20)

    /* STEPx_NUM_CONV field */
    #define STEPX_NUM_CONV_MASK									((uint8_t) 0x1F)
    #define STEPX_DIGITAL_CFG_STEPX_NUM_CONV_BITOFFSET			(0)


/* Register 0x04 (STEPX_FLTR_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                   SPARE[1:0]                  |    STEPx_FLTR_MODE    |                                                  STEPx_FLTR_OSR[4:0]                                                  |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_FLTR_CFG register */
    #define STEPX_FLTR_CFG_ADDRESS								((uint8_t) 0x04)
    #define STEPX_FLTR_CFG_DEFAULT								((uint8_t) 0x01)

    /* SPARE field */
    #define SPARE_MASK											((uint8_t) 0xC0)
    #define STEPX_FLTR_CFG_SPARE_BITOFFSET						(6)

    /* STEPx_FLTR_MODE field */
    #define STEPX_FLTR_MODE_MASK								((uint8_t) 0x20)
    #define STEPX_FLTR_CFG_STEPX_FLTR_MODE_BITOFFSET			(5)
    #define STEPX_FLTR_MODE_SINC4								((uint8_t) 0x00)    // DEFAULT
    #define STEPX_FLTR_MODE_SINC3								((uint8_t) 0x20)

    /* STEPx_FLTR_OSR field */
    #define STEPX_FLTR_OSR_MASK									((uint8_t) 0x1F)
    #define STEPX_FLTR_CFG_STEPX_FLTR_OSR_BITOFFSET				(0)
    #define STEPX_FLTR_OSR_OSR8_SINCX									((uint8_t) 0x00)
    #define STEPX_FLTR_OSR_OSR16_SINCX								((uint8_t) 0x01)    // DEFAULT
    #define STEPX_FLTR_OSR_OSR24_SINCX								((uint8_t) 0x02)
    #define STEPX_FLTR_OSR_OSR32_SINCX								((uint8_t) 0x03)
    #define STEPX_FLTR_OSR_OSR64_SINCX								((uint8_t) 0x04)
    #define STEPX_FLTR_OSR_OSR128_SINCX								((uint8_t) 0x05)
    #define STEPX_FLTR_OSR_OSR256_SINCX								((uint8_t) 0x06)
    #define STEPX_FLTR_OSR_OSR512_SINCX								((uint8_t) 0x07)
    #define STEPX_FLTR_OSR_OSR640_SINCX								((uint8_t) 0x08)
    #define STEPX_FLTR_OSR_OSR1024_SINCX								((uint8_t) 0x09)
    #define STEPX_FLTR_OSR_OSR2048_SINCX								((uint8_t) 0x0A)
    #define STEPX_FLTR_OSR_OSR4096_SINCX								((uint8_t) 0x0B)
    #define STEPX_FLTR_OSR_OSR8192_SINCX								((uint8_t) 0x0C)
    #define STEPX_FLTR_OSR_FDATA60SPS_SINCX   						((uint8_t) 0x0D)
    #define STEPX_FLTR_OSR_FDATA50SPS_SINCX	    					((uint8_t) 0x0E)
    #define STEPX_FLTR_OSR_FDATA16_SINCX			        			((uint8_t) 0x0F)
    #define STEPX_FLTR_OSR_FDATA10SPS_SINCX				    		((uint8_t) 0x10)
    #define STEPX_FLTR_OSR_OSR64_SINC4								((uint8_t) 0x11)
    #define STEPX_FLTR_OSR_OSR128_SINC4								((uint8_t) 0x12)
    #define STEPX_FLTR_OSR_OSR256_SINC4								((uint8_t) 0x13)
    #define STEPX_FLTR_OSR_OSR512_SINC4								((uint8_t) 0x14)
    #define STEPX_FLTR_OSR_OSR640_SINC4								((uint8_t) 0x15)
    #define STEPX_FLTR_OSR_OSR1024_SINC4								((uint8_t) 0x16)
    #define STEPX_FLTR_OSR_OSR2048_SINC4								((uint8_t) 0x17)
    #define STEPX_FLTR_OSR_OSR4096_SINC4								((uint8_t) 0x18)
    #define STEPX_FLTR_OSR_OSR8192_SINC4								((uint8_t) 0x19)
    #define STEPX_FLTR_OSR_FDATA60SPS_SINC4   						((uint8_t) 0x1A)
    #define STEPX_FLTR_OSR_FDATA50SPS_SINC4	    					((uint8_t) 0x1B)
    #define STEPX_FLTR_OSR_FDATA16_SINC4			        			((uint8_t) 0x1C)
    #define STEPX_FLTR_OSR_FDATA10SPS_SINC4				    		((uint8_t) 0x1D)
    #define STEPX_FLTR_OSR_FDATA25SPS_FIR				    	((uint8_t) 0x1E)
    #define STEPX_FLTR_OSR_FDATA20SPS_FIR					    ((uint8_t) 0x1F)


/* Register 0x05 (STEPX_DELAY_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        RESERVED       |                                                                            STEPx_DELAY[6:0]                                                                           |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_DELAY_CFG register */
    #define STEPX_DELAY_CFG_ADDRESS								((uint8_t) 0x05)
    #define STEPX_DELAY_CFG_DEFAULT								((uint8_t) 0x00)

    /* STEPx_DELAY field */
    #define STEPX_DELAY_MASK									((uint8_t) 0x7F)
    #define STEPX_DELAY_CFG_STEPX_DELAY_BITOFFSET				(0)


/* Register 0x06 (STEPX_OFFSET_CAL_MSB) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                    STEPx_OFFSET_CAL[23:16]                                                                                    |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_OFFSET_CAL_MSB register */
    #define STEPX_OFFSET_CAL_MSB_ADDRESS						((uint8_t) 0x06)
    #define STEPX_OFFSET_CAL_MSB_DEFAULT						((uint8_t) 0x00)

    /* STEPx_OFFSET_CAL field */
    #define STEPX_OFFSET_CAL_MASK								((uint8_t) 0xFF)
    #define STEPX_OFFSET_CAL_MSB_STEPX_OFFSET_CAL_BITOFFSET		(0)


/* Register 0x07 (STEPX_OFFSET_CAL_ISB) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                     STEPx_OFFSET_CAL[15:8]                                                                                    |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_OFFSET_CAL_ISB register */
    #define STEPX_OFFSET_CAL_ISB_ADDRESS						((uint8_t) 0x07)
    #define STEPX_OFFSET_CAL_ISB_DEFAULT						((uint8_t) 0x00)

    /* STEPx_OFFSET_CAL field */
    #define STEPX_OFFSET_CAL_MASK								((uint8_t) 0xFF)
    #define STEPX_OFFSET_CAL_ISB_STEPX_OFFSET_CAL_BITOFFSET		(0)


/* Register 0x08 (STEPX_OFFSET_CAL_LSB) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                     STEPx_OFFSET_CAL[7:0]                                                                                     |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_OFFSET_CAL_LSB register */
    #define STEPX_OFFSET_CAL_LSB_ADDRESS						((uint8_t) 0x08)
    #define STEPX_OFFSET_CAL_LSB_DEFAULT						((uint8_t) 0x00)

    /* STEPx_OFFSET_CAL field */
    #define STEPX_OFFSET_CAL_MASK								((uint8_t) 0xFF)
    #define STEPX_OFFSET_CAL_LSB_STEPX_OFFSET_CAL_BITOFFSET		(0)


/* Register 0x09 (STEPX_GAIN_CAL_MSB) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                      STEPx_GAIN_CAL[15:8]                                                                                     |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_GAIN_CAL_MSB register */
    #define STEPX_GAIN_CAL_MSB_ADDRESS							((uint8_t) 0x09)
    #define STEPX_GAIN_CAL_MSB_DEFAULT							((uint8_t) 0x40)

    /* STEPx_GAIN_CAL field */
    #define STEPX_GAIN_CAL_MASK									((uint8_t) 0xFF)
    #define STEPX_GAIN_CAL_MSB_STEPX_GAIN_CAL_BITOFFSET			(0)


/* Register 0x0A (STEPX_GAIN_CAL_LSB) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                      STEPx_GAIN_CAL[7:0]                                                                                      |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_GAIN_CAL_LSB register */
    #define STEPX_GAIN_CAL_LSB_ADDRESS							((uint8_t) 0x0A)
    #define STEPX_GAIN_CAL_LSB_DEFAULT							((uint8_t) 0x00)

    /* STEPx_GAIN_CAL field */
    #define STEPX_GAIN_CAL_MASK									((uint8_t) 0xFF)
    #define STEPX_GAIN_CAL_LSB_STEPX_GAIN_CAL_BITOFFSET			(0)


/* Register 0x0B (STEPX_OW_SYSMON_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                 RESERVED[1:0]                 |                STEPx_BOCS[1:0]                |        RESERVED       |                           STEPx_SYS_MON[2:0]                          |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_OW_SYSMON_CFG register */
    #define STEPX_OW_SYSMON_CFG_ADDRESS							((uint8_t) 0x0B)
    #define STEPX_OW_SYSMON_CFG_DEFAULT							((uint8_t) 0x00)

    /* STEPx_BOCS field */
    #define STEPX_BOCS_MASK										((uint8_t) 0x30)
    #define STEPX_OW_SYSMON_CFG_STEPX_BOCS_BITOFFSET			(4)
    #define STEPX_BOCS_DISABLED									((uint8_t) 0x00)    // DEFAULT
    #define STEPX_BOCS_0										((uint8_t) 0x10)
    #define STEPX_BOCS_1MUA										((uint8_t) 0x20)
    #define STEPX_BOCS_10MUA									((uint8_t) 0x30)

    /* STEPx_SYS_MON field */
    #define STEPX_SYS_MON_MASK									((uint8_t) 0x07)
    #define STEPX_OW_SYSMON_CFG_STEPX_SYS_MON_BITOFFSET			(0)
    #define STEPX_SYS_MON_STEPX_SYS_MON_DISABLED				((uint8_t) 0x00)    // DEFAULT
    #define STEPX_SYS_MON_STEPX_SYS_MON_INTERNALSHORT			((uint8_t) 0x01)
    #define STEPX_SYS_MON_STEPX_SYS_MON_INTERNALTEMPERATURESENSOR	((uint8_t) 0x02)
    #define STEPX_SYS_MON_STEPX_SYS_MON_REFP0_REFN0_DIV_8		((uint8_t) 0x03)
    #define STEPX_SYS_MON_STEPX_SYS_MON_REFP1_REFN1_DIV_8		((uint8_t) 0x04)
    #define STEPX_SYS_MON_STEPX_SYS_MON_AVDD_AVSS_DIV_8			((uint8_t) 0x05)
    #define STEPX_SYS_MON_STEPX_SYS_MON_IOVDD_DGND_DIV_8		((uint8_t) 0x06)
    #define STEPX_SYS_MON_STEPX_SYS_MON_VLDO_DIV_4				((uint8_t) 0x07)


/* Register 0x0C (STEPX_IDAC_MAG_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                        STEPx_I2MAG[3:0]                                       |                                        STEPx_I1MAG[3:0]                                       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_IDAC_MAG_CFG register */
    #define STEPX_IDAC_MAG_CFG_ADDRESS							((uint8_t) 0x0C)
    #define STEPX_IDAC_MAG_CFG_DEFAULT							((uint8_t) 0x00)

    /* STEPx_I2MAG field */
    #define STEPX_I2MAG_MASK									((uint8_t) 0xF0)
    #define STEPX_IDAC_MAG_CFG_STEPX_I2MAG_BITOFFSET			(4)
    #define STEPX_I2MAG_DISABLED								((uint8_t) 0x00)    // DEFAULT
    #define STEPX_I2MAG_1XIUNIT									((uint8_t) 0x10)
    #define STEPX_I2MAG_10XIUNIT								((uint8_t) 0x20)
    #define STEPX_I2MAG_20XIUNIT								((uint8_t) 0x30)
    #define STEPX_I2MAG_30XIUNIT								((uint8_t) 0x40)
    #define STEPX_I2MAG_40XIUNIT								((uint8_t) 0x50)
    #define STEPX_I2MAG_50XIUNIT								((uint8_t) 0x60)
    #define STEPX_I2MAG_60XIUNIT								((uint8_t) 0x70)
    #define STEPX_I2MAG_70XIUNIT								((uint8_t) 0x80)
    #define STEPX_I2MAG_80XIUNIT								((uint8_t) 0x90)
    #define STEPX_I2MAG_90XIUNIT								((uint8_t) 0xA0)
    #define STEPX_I2MAG_100XIUNIT								((uint8_t) 0xB0)
  

    /* STEPx_I1MAG field */
    #define STEPX_I1MAG_MASK									((uint8_t) 0x0F)
    #define STEPX_IDAC_MAG_CFG_STEPX_I1MAG_BITOFFSET			(0)
    #define STEPX_I1MAG_DISABLED								((uint8_t) 0x00)    // DEFAULT
    #define STEPX_I1MAG_1XIUNIT									((uint8_t) 0x01)
    #define STEPX_I1MAG_10XIUNIT								((uint8_t) 0x02)
    #define STEPX_I1MAG_20XIUNIT								((uint8_t) 0x03)
    #define STEPX_I1MAG_30XIUNIT								((uint8_t) 0x04)
    #define STEPX_I1MAG_40XIUNIT								((uint8_t) 0x05)
    #define STEPX_I1MAG_50XIUNIT								((uint8_t) 0x06)
    #define STEPX_I1MAG_60XIUNIT								((uint8_t) 0x07)
    #define STEPX_I1MAG_70XIUNIT								((uint8_t) 0x08)
    #define STEPX_I1MAG_80XIUNIT								((uint8_t) 0x09)
    #define STEPX_I1MAG_90XIUNIT								((uint8_t) 0x0A)
    #define STEPX_I1MAG_100XIUNIT								((uint8_t) 0x0B)
   


/* Register 0x0D (STEPX_IDAC1_MUX_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |      STEPx_IUNIT      |                 RESERVED[1:0]                 |                                                    STEPx_I1MUX[4:0]                                                   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_IDAC1_MUX_CFG register */
    #define STEPX_IDAC1_MUX_CFG_ADDRESS							((uint8_t) 0x0D)
    #define STEPX_IDAC1_MUX_CFG_DEFAULT							((uint8_t) 0x1F)

    /* STEPx_IUNIT field */
    #define STEPX_IUNIT_MASK									((uint8_t) 0x80)
    #define STEPX_IDAC1_MUX_CFG_STEPX_IUNIT_BITOFFSET			(7)
    #define STEPX_IUNIT_1MUA									((uint8_t) 0x00)    // DEFAULT
    #define STEPX_IUNIT_10MUA									((uint8_t) 0x80)

    /* STEPx_I1MUX field */
    #define STEPX_I1MUX_MASK									((uint8_t) 0x1F)
    #define STEPX_IDAC1_MUX_CFG_STEPX_I1MUX_BITOFFSET			(0)
    #define STEPX_I1MUX_AIN0									((uint8_t) 0x00)
    #define STEPX_I1MUX_AIN1									((uint8_t) 0x01)
    #define STEPX_I1MUX_AIN2									((uint8_t) 0x02)
    #define STEPX_I1MUX_AIN3									((uint8_t) 0x03)
    #define STEPX_I1MUX_AIN4									((uint8_t) 0x04)
    #define STEPX_I1MUX_AIN5									((uint8_t) 0x05)
    #define STEPX_I1MUX_AIN6									((uint8_t) 0x06)
    #define STEPX_I1MUX_AIN7									((uint8_t) 0x07)
    #define STEPX_I1MUX_AIN8DISCONNECTEDADS1X4S14				((uint8_t) 0x08)
    #define STEPX_I1MUX_AIN9DISCONNECTEDADS1X4S14				((uint8_t) 0x09)
    #define STEPX_I1MUX_AIN10DISCONNECTEDADS1X4S14				((uint8_t) 0x0A)
    #define STEPX_I1MUX_AIN11DISCONNECTEDADS1X4S14				((uint8_t) 0x0B)
    #define STEPX_I1MUX_AIN12DISCONNECTEDADS1X4S14				((uint8_t) 0x0C)
    #define STEPX_I1MUX_AIN13DISCONNECTEDADS1X4S14				((uint8_t) 0x0D)
    #define STEPX_I1MUX_AIN14DISCONNECTEDADS1X4S14				((uint8_t) 0x0E)
    #define STEPX_I1MUX_AIN15DISCONNECTEDADS1X4S14				((uint8_t) 0x0F)
    #define STEPX_I1MUX_AIN16DISCONNECTEDADS1X4S14				((uint8_t) 0x10)
    #define STEPX_I1MUX_AIN17DISCONNECTEDADS1X4S14				((uint8_t) 0x11)
    #define STEPX_I1MUX_DISCONNECTED							((uint8_t) 0x1F)    // DEFAULT


/* Register 0x0E (STEPX_IDAC2_MUX_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                             RESERVED[2:0]                             |                                                    STEPx_I2MUX[4:0]                                                   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_IDAC2_MUX_CFG register */
    #define STEPX_IDAC2_MUX_CFG_ADDRESS							((uint8_t) 0x0E)
    #define STEPX_IDAC2_MUX_CFG_DEFAULT							((uint8_t) 0x1F)

    /* STEPx_I2MUX field */
    #define STEPX_I2MUX_MASK									((uint8_t) 0x1F)
    #define STEPX_IDAC2_MUX_CFG_STEPX_I2MUX_BITOFFSET			(0)
    #define STEPX_I2MUX_AIN0									((uint8_t) 0x00)
    #define STEPX_I2MUX_AIN1									((uint8_t) 0x01)
    #define STEPX_I2MUX_AIN2									((uint8_t) 0x02)
    #define STEPX_I2MUX_AIN3									((uint8_t) 0x03)
    #define STEPX_I2MUX_AIN4									((uint8_t) 0x04)
    #define STEPX_I2MUX_AIN5									((uint8_t) 0x05)
    #define STEPX_I2MUX_AIN6									((uint8_t) 0x06)
    #define STEPX_I2MUX_AIN7									((uint8_t) 0x07)
    #define STEPX_I2MUX_AIN8DISCONNECTEDADS1X4S14				((uint8_t) 0x08)
    #define STEPX_I2MUX_AIN9DISCONNECTEDADS1X4S14				((uint8_t) 0x09)
    #define STEPX_I2MUX_AIN10DISCONNECTEDADS1X4S14				((uint8_t) 0x0A)
    #define STEPX_I2MUX_AIN11DISCONNECTEDADS1X4S14				((uint8_t) 0x0B)
    #define STEPX_I2MUX_AIN12DISCONNECTEDADS1X4S14				((uint8_t) 0x0C)
    #define STEPX_I2MUX_AIN13DISCONNECTEDADS1X4S14				((uint8_t) 0x0D)
    #define STEPX_I2MUX_AIN14DISCONNECTEDADS1X4S14				((uint8_t) 0x0E)
    #define STEPX_I2MUX_AIN15DISCONNECTEDADS1X4S14				((uint8_t) 0x0F)
    #define STEPX_I2MUX_AIN16DISCONNECTEDADS1X4S14				((uint8_t) 0x10)
    #define STEPX_I2MUX_AIN17DISCONNECTEDADS1X4S14				((uint8_t) 0x11)
    #define STEPX_I2MUX_DISCONNECTED							((uint8_t) 0x1F)    // DEFAULT


/* Register 0x0F (STEPX_GPIO_DATA_OUT) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |  STEPx_GPIO7_DAT_OUT  |  STEPx_GPIO6_DAT_OUT  |  STEPx_GPIO5_DAT_OUT  |  STEPx_GPIO4_DAT_OUT  |  STEPx_GPIO3_DAT_OUT  |  STEPx_GPIO2_DAT_OUT  |  STEPx_GPIO1_DAT_OUT  |  STEPx_GPIO0_DAT_OUT  |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_GPIO_DATA_OUT register */
    #define STEPX_GPIO_DATA_OUT_ADDRESS							((uint8_t) 0x0F)
    #define STEPX_GPIO_DATA_OUT_DEFAULT							((uint8_t) 0x00)

    /* STEPx_GPIO7_DAT_OUT field */
    #define STEPX_GPIO7_DAT_OUT_MASK							((uint8_t) 0x80)
    #define STEPX_GPIO_DATA_OUT_STEPX_GPIO7_DAT_OUT_BITOFFSET	(7)
    #define STEPX_GPIO7_DAT_OUT_LOW								((uint8_t) 0x00)    // DEFAULT
    #define STEPX_GPIO7_DAT_OUT_HIGH							((uint8_t) 0x80)

    /* STEPx_GPIO6_DAT_OUT field */
    #define STEPX_GPIO6_DAT_OUT_MASK							((uint8_t) 0x40)
    #define STEPX_GPIO_DATA_OUT_STEPX_GPIO6_DAT_OUT_BITOFFSET	(6)
    #define STEPX_GPIO6_DAT_OUT_LOW								((uint8_t) 0x00)    // DEFAULT
    #define STEPX_GPIO6_DAT_OUT_HIGH							((uint8_t) 0x40)

    /* STEPx_GPIO5_DAT_OUT field */
    #define STEPX_GPIO5_DAT_OUT_MASK							((uint8_t) 0x20)
    #define STEPX_GPIO_DATA_OUT_STEPX_GPIO5_DAT_OUT_BITOFFSET	(5)
    #define STEPX_GPIO5_DAT_OUT_LOW								((uint8_t) 0x00)    // DEFAULT
    #define STEPX_GPIO5_DAT_OUT_HIGH							((uint8_t) 0x20)

    /* STEPx_GPIO4_DAT_OUT field */
    #define STEPX_GPIO4_DAT_OUT_MASK							((uint8_t) 0x10)
    #define STEPX_GPIO_DATA_OUT_STEPX_GPIO4_DAT_OUT_BITOFFSET	(4)
    #define STEPX_GPIO4_DAT_OUT_LOW								((uint8_t) 0x00)    // DEFAULT
    #define STEPX_GPIO4_DAT_OUT_HIGH							((uint8_t) 0x10)

    /* STEPx_GPIO3_DAT_OUT field */
    #define STEPX_GPIO3_DAT_OUT_MASK							((uint8_t) 0x08)
    #define STEPX_GPIO_DATA_OUT_STEPX_GPIO3_DAT_OUT_BITOFFSET	(3)
    #define STEPX_GPIO3_DAT_OUT_LOW								((uint8_t) 0x00)    // DEFAULT
    #define STEPX_GPIO3_DAT_OUT_HIGH							((uint8_t) 0x08)

    /* STEPx_GPIO2_DAT_OUT field */
    #define STEPX_GPIO2_DAT_OUT_MASK							((uint8_t) 0x04)
    #define STEPX_GPIO_DATA_OUT_STEPX_GPIO2_DAT_OUT_BITOFFSET	(2)
    #define STEPX_GPIO2_DAT_OUT_LOW								((uint8_t) 0x00)    // DEFAULT
    #define STEPX_GPIO2_DAT_OUT_HIGH							((uint8_t) 0x04)

    /* STEPx_GPIO1_DAT_OUT field */
    #define STEPX_GPIO1_DAT_OUT_MASK							((uint8_t) 0x02)
    #define STEPX_GPIO_DATA_OUT_STEPX_GPIO1_DAT_OUT_BITOFFSET	(1)
    #define STEPX_GPIO1_DAT_OUT_LOW								((uint8_t) 0x00)    // DEFAULT
    #define STEPX_GPIO1_DAT_OUT_HIGH							((uint8_t) 0x02)

    /* STEPx_GPIO0_DAT_OUT field */
    #define STEPX_GPIO0_DAT_OUT_MASK							((uint8_t) 0x01)
    #define STEPX_GPIO_DATA_OUT_STEPX_GPIO0_DAT_OUT_BITOFFSET	(0)
    #define STEPX_GPIO0_DAT_OUT_LOW								((uint8_t) 0x00)    // DEFAULT
    #define STEPX_GPIO0_DAT_OUT_HIGH							((uint8_t) 0x01)


/* Register 0x10 (STEPX_COMPARATOR_HIGH_THRESHOLD_MSB) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                    STEPx_COMP_HIGH_TH[15:8]                                                                                   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_COMPARATOR_HIGH_THRESHOLD_MSB register */
    #define STEPX_COMPARATOR_HIGH_THRESHOLD_MSB_ADDRESS			((uint8_t) 0x10)
    #define STEPX_COMPARATOR_HIGH_THRESHOLD_MSB_DEFAULT			((uint8_t) 0x00)

    /* STEPx_COMP_HIGH_TH field */
    #define STEPX_COMP_HIGH_TH_MASK								((uint8_t) 0xFF)
    #define STEPX_COMPARATOR_HIGH_THRESHOLD_MSB_STEPX_COMP_HIGH_TH_BITOFFSET	(0)


/* Register 0x11 (STEPX_COMPARATOR_HIGH_THRESHOLD_LSB) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                    STEPx_COMP_HIGH_TH[7:0]                                                                                    |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_COMPARATOR_HIGH_THRESHOLD_LSB register */
    #define STEPX_COMPARATOR_HIGH_THRESHOLD_LSB_ADDRESS			((uint8_t) 0x11)
    #define STEPX_COMPARATOR_HIGH_THRESHOLD_LSB_DEFAULT			((uint8_t) 0x00)

    /* STEPx_COMP_HIGH_TH field */
    #define STEPX_COMP_HIGH_TH_MASK								((uint8_t) 0xFF)
    #define STEPX_COMPARATOR_HIGH_THRESHOLD_LSB_STEPX_COMP_HIGH_TH_BITOFFSET	(0)


/* Register 0x12 (STEPX_COMPARATOR_LOW_THRESHOLD_MSB) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                    STEPx_COMP_LOW_TH[15:8]                                                                                    |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_COMPARATOR_LOW_THRESHOLD_MSB register */
    #define STEPX_COMPARATOR_LOW_THRESHOLD_MSB_ADDRESS			((uint8_t) 0x12)
    #define STEPX_COMPARATOR_LOW_THRESHOLD_MSB_DEFAULT			((uint8_t) 0x00)

    /* STEPx_COMP_LOW_TH field */
    #define STEPX_COMP_LOW_TH_MASK								((uint8_t) 0xFF)
    #define STEPX_COMPARATOR_LOW_THRESHOLD_MSB_STEPX_COMP_LOW_TH_BITOFFSET	(0)


/* Register 0x13 (STEPX_COMPARATOR_LOW_THRESHOLD_LSB) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                     STEPx_COMP_LOW_TH[7:0]                                                                                    |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_COMPARATOR_LOW_THRESHOLD_LSB register */
    #define STEPX_COMPARATOR_LOW_THRESHOLD_LSB_ADDRESS			((uint8_t) 0x13)
    #define STEPX_COMPARATOR_LOW_THRESHOLD_LSB_DEFAULT			((uint8_t) 0x00)

    /* STEPx_COMP_LOW_TH field */
    #define STEPX_COMP_LOW_TH_MASK								((uint8_t) 0xFF)
    #define STEPX_COMPARATOR_LOW_THRESHOLD_LSB_STEPX_COMP_LOW_TH_BITOFFSET	(0)


/* Register 0x14 (STEPX_COMPARATOR_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                         RESERVED[3:0]                                         |              STEPx_COMP_NUM[1:0]              | STEPx_COMP_HIGH_TH_EN |  STEPx_COMP_LOW_TH_EN |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_COMPARATOR_CFG register */
    #define STEPX_COMPARATOR_CFG_ADDRESS						((uint8_t) 0x14)
    #define STEPX_COMPARATOR_CFG_DEFAULT						((uint8_t) 0x00)

    /* STEPx_COMP_NUM field */
    #define STEPX_COMP_NUM_MASK									((uint8_t) 0x0C)
    #define STEPX_COMPARATOR_CFG_STEPX_COMP_NUM_BITOFFSET		(2)
    #define STEPX_COMP_NUM_1									((uint8_t) 0x00)    // DEFAULT
    #define STEPX_COMP_NUM_2									((uint8_t) 0x04)
    #define STEPX_COMP_NUM_4									((uint8_t) 0x08)
    #define STEPX_COMP_NUM_8									((uint8_t) 0x0C)

    /* STEPx_COMP_HIGH_TH_EN field */
    #define STEPX_COMP_HIGH_TH_EN_MASK							((uint8_t) 0x02)
    #define STEPX_COMPARATOR_CFG_STEPX_COMP_HIGH_TH_EN_BITOFFSET	(1)
    #define STEPX_COMP_HIGH_TH_EN_DISABLED						((uint8_t) 0x00)    // DEFAULT
    #define STEPX_COMP_HIGH_TH_EN_ENABLED						((uint8_t) 0x02)

    /* STEPx_COMP_LOW_TH_EN field */
    #define STEPX_COMP_LOW_TH_EN_MASK							((uint8_t) 0x01)
    #define STEPX_COMPARATOR_CFG_STEPX_COMP_LOW_TH_EN_BITOFFSET	(0)
    #define STEPX_COMP_LOW_TH_EN_DISABLED						((uint8_t) 0x00)    // DEFAULT
    #define STEPX_COMP_LOW_TH_EN_ENABLED						((uint8_t) 0x01)


/* Register 0x3D (STEPX_REG_MAP_CRC) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                  STEPx_REG_MAP_CRC_VALUE[7:0]                                                                                 |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_REG_MAP_CRC register */
    #define STEPX_REG_MAP_CRC_ADDRESS							((uint8_t) 0x3D)
    #define STEPX_REG_MAP_CRC_DEFAULT							((uint8_t) 0x0E)

    /* STEPx_REG_MAP_CRC_VALUE field */
    #define STEPX_REG_MAP_CRC_VALUE_MASK						((uint8_t) 0xFF)
    #define STEPX_REG_MAP_CRC_STEPX_REG_MAP_CRC_VALUE_BITOFFSET	(0)


/* Register 0x3E (STEPX_PAGE_INDICATOR) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                   STEPx_PAGE_INDICATOR[7:0]                                                                                   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_PAGE_INDICATOR register */
    #define STEPX_PAGE_INDICATOR_ADDRESS						((uint8_t) 0x3E)
    #define STEPX_PAGE_INDICATOR_DEFAULT						((uint8_t) 0x00)

    /* STEPx_PAGE_INDICATOR field */
    #define STEPX_PAGE_INDICATOR_MASK							((uint8_t) 0xFF)
    #define STEPX_PAGE_INDICATOR_BITOFFSET						(0)


/* Register 0x3F (STEPX_PAGE_POINTER) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7         |         Bit 6         |         Bit 5         |         Bit 4         |         Bit 3         |         Bit 2         |         Bit 1         |         Bit 0         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                    STEPx_PAGE_POINTER[7:0]                                                                                    |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_PAGE_POINTER register */
    #define STEPX_PAGE_POINTER_ADDRESS							((uint8_t) 0x3F)
    #define STEPX_PAGE_POINTER_DEFAULT							((uint8_t) 0x00)

    /* STEPx_PAGE_POINTER field */
    #define STEPX_PAGE_POINTER_MASK								((uint8_t) 0xFF)
    #define STEPX_PAGE_POINTER_BITOFFSET						(0)


#endif /* ADS124S18_PAGE1_H_ */
