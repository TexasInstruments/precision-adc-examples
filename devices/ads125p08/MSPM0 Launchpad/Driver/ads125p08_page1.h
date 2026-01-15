/*
 *
 * @copyright Copyright (C) 2024-2026 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef ADS125P08_PAGE1_H_
#define ADS125P08_PAGE1_H_

#include <stdint.h>
#include <stdbool.h>



/* Register 0x00 (STEPX_AINP_CFG) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                            RESERVED[2:0]                           |                                                  STEPx_AINP[4:0]                                                 |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_AINP_CFG register */
    #define STEPX_AINP_CFG_ADDRESS								((uint8_t) 0x00)
    #define STEPX_AINP_CFG_DEFAULT								((uint8_t) 0x00)

    /* STEPx_AINP field */
    #define STEPX_AINP_MASK										((uint8_t) 0x1F)
    #define STEPX_AINP_CFG_STEPX_AINP_BITOFFSET					(0)
    #define STEPX_AINP_AIN0										((uint8_t) 0x00)    // DEFAULT
    #define STEPX_AINP_AIN1										((uint8_t) 0x01)
    #define STEPX_AINP_AIN2										((uint8_t) 0x02)
    #define STEPX_AINP_AIN3										((uint8_t) 0x03)
    #define STEPX_AINP_AIN4										((uint8_t) 0x04)
    #define STEPX_AINP_AIN5										((uint8_t) 0x05)
    #define STEPX_AINP_AIN6										((uint8_t) 0x06)
    #define STEPX_AINP_AIN7										((uint8_t) 0x07)
    #define STEPX_AINP_AIN8										((uint8_t) 0x08)
    #define STEPX_AINP_AIN9										((uint8_t) 0x09)
    #define STEPX_AINP_AIN10									((uint8_t) 0x0A)
    #define STEPX_AINP_AIN11									((uint8_t) 0x0B)
    #define STEPX_AINP_AIN12									((uint8_t) 0x0C)
    #define STEPX_AINP_AIN13									((uint8_t) 0x0D)
    #define STEPX_AINP_AIN14									((uint8_t) 0x0E)
    #define STEPX_AINP_AIN15									((uint8_t) 0x0F)
    #define STEPX_AINP_AINCOM									((uint8_t) 0x10)
    #define STEPX_AINP_OPEN										((uint8_t) 0x11)



/* Register 0x01 (STEPX_AINN_CFG) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                            RESERVED[2:0]                           |                                                  STEPx_AINN[4:0]                                                 |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_AINN_CFG register */
    #define STEPX_AINN_CFG_ADDRESS								((uint8_t) 0x01)
    #define STEPX_AINN_CFG_DEFAULT								((uint8_t) 0x00)

    /* STEPx_AINN field */
    #define STEPX_AINN_MASK										((uint8_t) 0x1F)
    #define STEPX_AINN_CFG_STEPX_AINN_BITOFFSET					(0)
    #define STEPX_AINN_AIN0										((uint8_t) 0x00)    // DEFAULT
    #define STEPX_AINN_AIN1										((uint8_t) 0x01)
    #define STEPX_AINN_AIN2										((uint8_t) 0x02)
    #define STEPX_AINN_AIN3										((uint8_t) 0x03)
    #define STEPX_AINN_AIN4										((uint8_t) 0x04)
    #define STEPX_AINN_AIN5										((uint8_t) 0x05)
    #define STEPX_AINN_AIN6										((uint8_t) 0x06)
    #define STEPX_AINN_AIN7										((uint8_t) 0x07)
    #define STEPX_AINN_AIN8										((uint8_t) 0x08)
    #define STEPX_AINN_AIN9										((uint8_t) 0x09)
    #define STEPX_AINN_AIN10									((uint8_t) 0x0A)
    #define STEPX_AINN_AIN11									((uint8_t) 0x0B)
    #define STEPX_AINN_AIN12									((uint8_t) 0x0C)
    #define STEPX_AINN_AIN13									((uint8_t) 0x0D)
    #define STEPX_AINN_AIN14									((uint8_t) 0x0E)
    #define STEPX_AINN_AIN15									((uint8_t) 0x0F)
    #define STEPX_AINN_AINCOM									((uint8_t) 0x10)
    #define STEPX_AINN_OPEN										((uint8_t) 0x11)



/* Register 0x02 (STEPX_ADC_REF_CFG) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       RESERVED       |     STEPx_EXT_RNG    |     STEPx_CODING     |     STEPx_REF_SEL    |                                    STEPx_NUM_CONV[3:0]                                    |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_ADC_REF_CFG register */
    #define STEPX_ADC_REF_CFG_ADDRESS							((uint8_t) 0x02)
    #define STEPX_ADC_REF_CFG_DEFAULT							((uint8_t) 0x00)

    /* STEPx_EXT_RNG field */
    #define STEPX_EXT_RNG_MASK									((uint8_t) 0x40)
    #define STEPX_ADC_REF_CFG_STEPX_EXT_RNG_BITOFFSET			(6)
    #define STEPX_EXT_RNG_STANDARDINPUTRANGE					((uint8_t) 0x00)    // DEFAULT
    #define STEPX_EXT_RNG_25EXTENDEDINPUTRANGE					((uint8_t) 0x40)

    /* STEPx_CODING field */
    #define STEPX_CODING_MASK									((uint8_t) 0x20)
    #define STEPX_ADC_REF_CFG_STEPX_CODING_BITOFFSET			(5)
    #define STEPX_CODING_BIPOLARTWOSCOMPLEMENTFORMAT			((uint8_t) 0x00)    // DEFAULT
    #define STEPX_CODING_UNIPOLARSTRAIGHTBINARYFORMAT			((uint8_t) 0x20)

    /* STEPx_REF_SEL field */
    #define STEPX_REF_SEL_MASK									((uint8_t) 0x10)
    #define STEPX_ADC_REF_CFG_STEPX_REF_SEL_BITOFFSET			(4)
    #define STEPX_REF_SEL_EXTERNALVOLTAGEREFERENCE				((uint8_t) 0x00)    // DEFAULT
    #define STEPX_REF_SEL_INTERNALVOLTAGEREFERENCE				((uint8_t) 0x10)

    /* STEPx_NUM_CONV field */
    #define STEPX_NUM_CONV_MASK									((uint8_t) 0x0F)
    #define STEPX_ADC_REF_CFG_STEPX_NUM_CONV_BITOFFSET			(0)
    #define STEPX_NUM_CONV_1CONVERSION							((uint8_t) 0x00)    // DEFAULT
    #define STEPX_NUM_CONV_2CONVERSIONS							((uint8_t) 0x01)
    #define STEPX_NUM_CONV_3CONVERSIONS							((uint8_t) 0x02)
    #define STEPX_NUM_CONV_4CONVERSIONS							((uint8_t) 0x03)
    #define STEPX_NUM_CONV_6CONVERSIONS							((uint8_t) 0x04)
    #define STEPX_NUM_CONV_8CONVERSIONS							((uint8_t) 0x05)
    #define STEPX_NUM_CONV_10CONVERSIONS						((uint8_t) 0x06)
    #define STEPX_NUM_CONV_12CONVERSIONS						((uint8_t) 0x07)
    #define STEPX_NUM_CONV_14CONVERSIONS						((uint8_t) 0x08)
    #define STEPX_NUM_CONV_16CONVERSIONS						((uint8_t) 0x09)
    #define STEPX_NUM_CONV_24CONVERSIONS						((uint8_t) 0x0A)
    #define STEPX_NUM_CONV_32CONVERSIONS						((uint8_t) 0x0B)
    #define STEPX_NUM_CONV_64CONVERSIONS						((uint8_t) 0x0C)
    #define STEPX_NUM_CONV_128CONVERSIONS						((uint8_t) 0x0D)
    #define STEPX_NUM_CONV_256CONVERSIONS						((uint8_t) 0x0E)
    #define STEPX_NUM_CONV_512CONVERSIONS						((uint8_t) 0x0F)


/* Register 0x03 (STEPX_FLTR1_CFG) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                RESERVED[1:0]                |    STEPx_FLTR_MODE   |                                                STEPx_FLTR_OSR[4:0]                                               |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_FLTR1_CFG register */
    #define STEPX_FLTR1_CFG_ADDRESS								((uint8_t) 0x03)
    #define STEPX_FLTR1_CFG_DEFAULT								((uint8_t) 0x01)

    /* STEPx_FLTR_MODE field */
    #define STEPX_FLTR_MODE_MASK								((uint8_t) 0x20)
    #define STEPX_FLTR1_CFG_STEPX_FLTR_MODE_BITOFFSET			(5)
    #define STEPX_FLTR_MODE_SINC4FIRSTSTAGEFILTER				((uint8_t) 0x00)    // DEFAULT
    #define STEPX_FLTR_MODE_SINC3FIRSTSTAGEFILTER				((uint8_t) 0x20)

    /* STEPx_FLTR_OSR field */
    #define STEPX_FLTR_OSR_MASK									((uint8_t) 0x1F)
    #define STEPX_FLTR1_CFG_STEPX_FLTR_OSR_BITOFFSET			(0)
    #define STEPX_FLTR_OSR_SINCXOSR12							((uint8_t) 0x00)
    #define STEPX_FLTR_OSR_SINCXOSR16							((uint8_t) 0x01)    // DEFAULT
    #define STEPX_FLTR_OSR_SINCXOSR24							((uint8_t) 0x02)
    #define STEPX_FLTR_OSR_SINCXOSR32							((uint8_t) 0x03)
    #define STEPX_FLTR_OSR_SINCXOSR64							((uint8_t) 0x04)
    #define STEPX_FLTR_OSR_SINCXOSR128							((uint8_t) 0x05)
    #define STEPX_FLTR_OSR_SINCXOSR256							((uint8_t) 0x06)
    #define STEPX_FLTR_OSR_SINCXOSR512							((uint8_t) 0x07)
    #define STEPX_FLTR_OSR_SINCXOSR1024							((uint8_t) 0x08)
    #define STEPX_FLTR_OSR_SINCXOSR2048							((uint8_t) 0x09)
    #define STEPX_FLTR_OSR_SINCXOSR4000							((uint8_t) 0x0A)
    #define STEPX_FLTR_OSR_SINCXOSR8000							((uint8_t) 0x0B)
    #define STEPX_FLTR_OSR_SINCXOSR16000						((uint8_t) 0x0C)
    #define STEPX_FLTR_OSR_SINCXOSR26667						((uint8_t) 0x0D)
    #define STEPX_FLTR_OSR_SINCXOSR32000						((uint8_t) 0x0E)
    #define STEPX_FLTR_OSR_SINCXOSR96000						((uint8_t) 0x0F)
    #define STEPX_FLTR_OSR_SINCXOSR160000						((uint8_t) 0x10)
    #define STEPX_FLTR_OSR_SINC4OSR32SINC1OSR2					((uint8_t) 0x11)
    #define STEPX_FLTR_OSR_SINC4OSR32SINC1OSR4					((uint8_t) 0x12)
    #define STEPX_FLTR_OSR_SINC4OSR32SINC1OSR8					((uint8_t) 0x13)
    #define STEPX_FLTR_OSR_SINC4OSR32SINC1OSR16					((uint8_t) 0x14)
    #define STEPX_FLTR_OSR_SINC4OSR32SINC1OSR32					((uint8_t) 0x15)
    #define STEPX_FLTR_OSR_SINC4OSR32SINC1OSR64					((uint8_t) 0x16)
    #define STEPX_FLTR_OSR_SINC4OSR32SINC1OSR125				((uint8_t) 0x17)
    #define STEPX_FLTR_OSR_SINC4OSR32SINC1OSR250				((uint8_t) 0x18)
    #define STEPX_FLTR_OSR_SINC4OSR32SINC1OSR500				((uint8_t) 0x19)
    #define STEPX_FLTR_OSR_SINC4OSR32SINC1OSR833				((uint8_t) 0x1A)
    #define STEPX_FLTR_OSR_SINC4OSR32SINC1OSR1000				((uint8_t) 0x1B)
    #define STEPX_FLTR_OSR_SINC4OSR32SINC1OSR3000				((uint8_t) 0x1C)
    #define STEPX_FLTR_OSR_SINC4OSR32SINC1OSR5000				((uint8_t) 0x1D)
    #define STEPX_FLTR_OSR_SINC4OSR32SINC1OSR2099TAPFIR25SPS	((uint8_t) 0x1E)
    #define STEPX_FLTR_OSR_SINC4OSR32SINC1OSR20124TAPFIR20SPS	((uint8_t) 0x1F)


/* Register 0x04 (STEPX_DELAY_MSB_CFG) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                  STEPx_DELAY_MSB[7:0]                                                                                 |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_DELAY_MSB_CFG register */
    #define STEPX_DELAY_MSB_CFG_ADDRESS							((uint8_t) 0x04)
    #define STEPX_DELAY_MSB_CFG_DEFAULT							((uint8_t) 0x00)

    /* STEPx_DELAY_MSB field */
    #define STEPX_DELAY_MSB_MASK								((uint8_t) 0xFF)
    #define STEPX_DELAY_MSB_CFG_STEPX_DELAY_MSB_BITOFFSET		(0)


/* Register 0x05 (STEPX_DELAY_LSB_CFG) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                  STEPx_DELAY_LSB[7:0]                                                                                 |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_DELAY_LSB_CFG register */
    #define STEPX_DELAY_LSB_CFG_ADDRESS							((uint8_t) 0x05)
    #define STEPX_DELAY_LSB_CFG_DEFAULT							((uint8_t) 0x00)

    /* STEPx_DELAY_LSB field */
    #define STEPX_DELAY_LSB_MASK								((uint8_t) 0xFF)
    #define STEPX_DELAY_LSB_CFG_STEPX_DELAY_LSB_BITOFFSET		(0)


/* Register 0x06 (STEPX_OFFSET_CAL_MSB) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                STEPx_OFFSET_CAL[23:16]                                                                                |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_OFFSET_CAL_MSB register */
    #define STEPX_OFFSET_CAL_MSB_ADDRESS						((uint8_t) 0x06)
    #define STEPX_OFFSET_CAL_MSB_DEFAULT						((uint8_t) 0x00)

    /* STEPx_OFFSET_CAL field */
    #define STEPX_OFFSET_CAL_MASK								((uint8_t) 0xFF)
    #define STEPX_OFFSET_CAL_MSB_STEPX_OFFSET_CAL_BITOFFSET		(0)


/* Register 0x07 (STEPX_OFFSET_CAL_ISB) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                 STEPx_OFFSET_CAL[15:8]                                                                                |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_OFFSET_CAL_ISB register */
    #define STEPX_OFFSET_CAL_ISB_ADDRESS						((uint8_t) 0x07)
    #define STEPX_OFFSET_CAL_ISB_DEFAULT						((uint8_t) 0x00)

    /* STEPx_OFFSET_CAL field */
    #define STEPX_OFFSET_CAL_MASK								((uint8_t) 0xFF)
    #define STEPX_OFFSET_CAL_ISB_STEPX_OFFSET_CAL_BITOFFSET		(0)


/* Register 0x08 (STEPX_OFFSET_CAL_LSB) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                 STEPx_OFFSET_CAL[7:0]                                                                                 |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_OFFSET_CAL_LSB register */
    #define STEPX_OFFSET_CAL_LSB_ADDRESS						((uint8_t) 0x08)
    #define STEPX_OFFSET_CAL_LSB_DEFAULT						((uint8_t) 0x00)

    /* STEPx_OFFSET_CAL field */
    #define STEPX_OFFSET_CAL_MASK								((uint8_t) 0xFF)
    #define STEPX_OFFSET_CAL_LSB_STEPX_OFFSET_CAL_BITOFFSET		(0)


/* Register 0x09 (STEPX_GAIN_CAL_MSB) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                  STEPx_GAIN_CAL[15:8]                                                                                 |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_GAIN_CAL_MSB register */
    #define STEPX_GAIN_CAL_MSB_ADDRESS							((uint8_t) 0x09)
    #define STEPX_GAIN_CAL_MSB_DEFAULT							((uint8_t) 0x40)

    /* STEPx_GAIN_CAL field */
    #define STEPX_GAIN_CAL_MASK									((uint8_t) 0xFF)
    #define STEPX_GAIN_CAL_MSB_STEPX_GAIN_CAL_BITOFFSET			(0)


/* Register 0x0A (STEPX_GAIN_CAL_LSB) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                  STEPx_GAIN_CAL[7:0]                                                                                  |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_GAIN_CAL_LSB register */
    #define STEPX_GAIN_CAL_LSB_ADDRESS							((uint8_t) 0x0A)
    #define STEPX_GAIN_CAL_LSB_DEFAULT							((uint8_t) 0x00)

    /* STEPx_GAIN_CAL field */
    #define STEPX_GAIN_CAL_MASK									((uint8_t) 0xFF)
    #define STEPX_GAIN_CAL_LSB_STEPX_GAIN_CAL_BITOFFSET			(0)


/* Register 0x0B (STEPX_OW_SYSMON_CFG) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       RESERVED       |       RESERVED       |               STEPx_BOCS[1:0]               |                                     STEPx_SYS_MON[3:0]                                    |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_OW_SYSMON_CFG register */
    #define STEPX_OW_SYSMON_CFG_ADDRESS							((uint8_t) 0x0B)
    #define STEPX_OW_SYSMON_CFG_DEFAULT							((uint8_t) 0x00)

    /* STEPx_BOCS field */
    #define STEPX_BOCS_MASK										((uint8_t) 0x30)
    #define STEPX_OW_SYSMON_CFG_STEPX_BOCS_BITOFFSET			(4)
    #define STEPX_BOCS_OFF										((uint8_t) 0x00)    // DEFAULT
    #define STEPX_BOCS_2A										((uint8_t) 0x10)
    #define STEPX_BOCS_8A										((uint8_t) 0x20)
    #define STEPX_BOCS_85A										((uint8_t) 0x30)

    /* STEPx_SYS_MON field */
    #define STEPX_SYS_MON_MASK									((uint8_t) 0x0F)
    #define STEPX_OW_SYSMON_CFG_STEPX_SYS_MON_BITOFFSET			(0)
    #define STEPX_SYS_MON_OFF									((uint8_t) 0x00)    // DEFAULT
    #define STEPX_SYS_MON_INTERNALSHORT                     	((uint8_t) 0x01)
    #define STEPX_SYS_MON_TEMPERATURESENSOR						((uint8_t) 0x02)
    #define STEPX_SYS_MON_AVDD_AVSS_DIV_3						((uint8_t) 0x03)
    #define STEPX_SYS_MON_CAPA									((uint8_t) 0x04)
    #define STEPX_SYS_MON_IOVDD_DIV_3							((uint8_t) 0x05)
    #define STEPX_SYS_MON_CAPD									((uint8_t) 0x06)
    #define STEPX_SYS_MON_REFP_REFN_DIV_3						((uint8_t) 0x07)
    #define STEPX_SYS_MON_RESP_RESN_DIV_3						((uint8_t) 0x08)


/* Register 0x0C (STEPX_TDAC_CFG0) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                            RESERVED[2:0]                           |                                                STEPx_TDAC_VAL[4:0]                                               |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_TDAC_CFG0 register */
    #define STEPX_TDAC_CFG0_ADDRESS								((uint8_t) 0x0C)
    #define STEPX_TDAC_CFG0_DEFAULT								((uint8_t) 0x00)

    /* STEPx_TDAC_VAL field */
    #define STEPX_TDAC_VAL_MASK									((uint8_t) 0x1F)
    #define STEPX_TDAC_CFG0_STEPX_TDAC_VAL_BITOFFSET			(0)


/* Register 0x0D (STEPX_TDAC_CFG1) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                            RESERVED[2:0]                           |                                                STEPx_TDAC_SEL[4:0]                                               |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_TDAC_CFG1 register */
    #define STEPX_TDAC_CFG1_ADDRESS								((uint8_t) 0x0D)
    #define STEPX_TDAC_CFG1_DEFAULT								((uint8_t) 0x00)

    /* STEPx_TDAC_SEL field */
    #define STEPX_TDAC_SEL_MASK									((uint8_t) 0x1F)
    #define STEPX_TDAC_CFG1_STEPX_TDAC_SEL_BITOFFSET			(0)
    #define STEPX_TDAC_SEL_OPEN									((uint8_t) 0x00)    // DEFAULT
    #define STEPX_TDAC_SEL_TDACUNBUFFEREDTOPOSITIVEINPUTNEGATIVEINPUTISCONNECTEDTOAVSS	((uint8_t) 0x01)
    #define STEPX_TDAC_SEL_TDACUNBUFFEREDTONEGATIVEINPUTPOSITIVEINPUTISCONNECTEDTOAVSS	((uint8_t) 0x02)
    #define STEPX_TDAC_SEL_AIN0									((uint8_t) 0x03)
    #define STEPX_TDAC_SEL_AIN1									((uint8_t) 0x04)
    #define STEPX_TDAC_SEL_AIN2									((uint8_t) 0x05)
    #define STEPX_TDAC_SEL_AIN3									((uint8_t) 0x06)
    #define STEPX_TDAC_SEL_AIN4									((uint8_t) 0x07)
    #define STEPX_TDAC_SEL_AIN5									((uint8_t) 0x08)
    #define STEPX_TDAC_SEL_AIN6									((uint8_t) 0x09)
    #define STEPX_TDAC_SEL_AIN7									((uint8_t) 0x0A)
    #define STEPX_TDAC_SEL_AIN8									((uint8_t) 0x0B)
    #define STEPX_TDAC_SEL_AIN9									((uint8_t) 0x0C)
    #define STEPX_TDAC_SEL_AIN10								((uint8_t) 0x0D)
    #define STEPX_TDAC_SEL_AIN11								((uint8_t) 0x0E)
    #define STEPX_TDAC_SEL_AIN12								((uint8_t) 0x0F)
    #define STEPX_TDAC_SEL_AIN13								((uint8_t) 0x10)
    #define STEPX_TDAC_SEL_AIN14								((uint8_t) 0x11)
    #define STEPX_TDAC_SEL_AIN15								((uint8_t) 0x12)
    #define STEPX_TDAC_SEL_REFPTDACPIN							((uint8_t) 0x13)




/* Register 0x0E (STEPX_SPARE_CFG) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     STEPx_SPARE7     |     STEPx_SPARE6     |     STEPx_SPARE5     |     STEPx_SPARE4     |     STEPx_SPARE3     |     STEPx_SPARE2     |     STEPx_SPARE1     |     STEPx_SPARE0     |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_SPARE_CFG register */
    #define STEPX_SPARE_CFG_ADDRESS								((uint8_t) 0x0E)
    #define STEPX_SPARE_CFG_DEFAULT								((uint8_t) 0x00)

    /* STEPx_SPARE7 field */
    #define STEPX_SPARE7_MASK									((uint8_t) 0x80)
    #define STEPX_SPARE_CFG_STEPX_SPARE7_BITOFFSET				(7)
    #define STEPX_SPARE7_SPAREISPROGRAMMEDTO0B					((uint8_t) 0x00)    // DEFAULT
    #define STEPX_SPARE7_SPAREISPROGRAMMEDTO1B					((uint8_t) 0x80)

    /* STEPx_SPARE6 field */
    #define STEPX_SPARE6_MASK									((uint8_t) 0x40)
    #define STEPX_SPARE_CFG_STEPX_SPARE6_BITOFFSET				(6)
    #define STEPX_SPARE6_SPAREISPROGRAMMEDTO0B					((uint8_t) 0x00)    // DEFAULT
    #define STEPX_SPARE6_SPAREISPROGRAMMEDTO1B					((uint8_t) 0x40)

    /* STEPx_SPARE5 field */
    #define STEPX_SPARE5_MASK									((uint8_t) 0x20)
    #define STEPX_SPARE_CFG_STEPX_SPARE5_BITOFFSET				(5)
    #define STEPX_SPARE5_SPAREISPROGRAMMEDTO0B					((uint8_t) 0x00)    // DEFAULT
    #define STEPX_SPARE5_SPAREISPROGRAMMEDTO1B					((uint8_t) 0x20)

    /* STEPx_SPARE4 field */
    #define STEPX_SPARE4_MASK									((uint8_t) 0x10)
    #define STEPX_SPARE_CFG_STEPX_SPARE4_BITOFFSET				(4)
    #define STEPX_SPARE4_SPAREISPROGRAMMEDTO0B					((uint8_t) 0x00)    // DEFAULT
    #define STEPX_SPARE4_SPAREISPROGRAMMEDTO1B					((uint8_t) 0x10)

    /* STEPx_SPARE3 field */
    #define STEPX_SPARE3_MASK									((uint8_t) 0x08)
    #define STEPX_SPARE_CFG_STEPX_SPARE3_BITOFFSET				(3)
    #define STEPX_SPARE3_SPAREISPROGRAMMEDTO0B					((uint8_t) 0x00)    // DEFAULT
    #define STEPX_SPARE3_SPAREISPROGRAMMEDTO1B					((uint8_t) 0x08)

    /* STEPx_SPARE2 field */
    #define STEPX_SPARE2_MASK									((uint8_t) 0x04)
    #define STEPX_SPARE_CFG_STEPX_SPARE2_BITOFFSET				(2)
    #define STEPX_SPARE2_SPAREISPROGRAMMEDTO0B					((uint8_t) 0x00)    // DEFAULT
    #define STEPX_SPARE2_SPAREISPROGRAMMEDTO1B					((uint8_t) 0x04)

    /* STEPx_SPARE1 field */
    #define STEPX_SPARE1_MASK									((uint8_t) 0x02)
    #define STEPX_SPARE_CFG_STEPX_SPARE1_BITOFFSET				(1)
    #define STEPX_SPARE1_SPAREISPROGRAMMEDTO0B					((uint8_t) 0x00)    // DEFAULT
    #define STEPX_SPARE1_SPAREISPROGRAMMEDTO1B					((uint8_t) 0x02)

    /* STEPx_SPARE0 field */
    #define STEPX_SPARE0_MASK									((uint8_t) 0x01)
    #define STEPX_SPARE_CFG_STEPX_SPARE0_BITOFFSET				(0)
    #define STEPX_SPARE0_SPAREISPROGRAMMEDTO0B					((uint8_t) 0x00)    // DEFAULT
    #define STEPX_SPARE0_SPAREISPROGRAMMEDTO1B					((uint8_t) 0x01)


/* Register 0x0F (STEPX_AGPIO_DATA_OUT) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * | STEPx_AGPIO7_DAT_OUT | STEPx_AGPIO6_DAT_OUT | STEPx_AGPIO5_DAT_OUT | STEPx_AGPIO4_DAT_OUT | STEPx_AGPIO3_DAT_OUT | STEPx_AGPIO2_DAT_OUT | STEPx_AGPIO1_DAT_OUT | STEPx_AGPIO0_DAT_OUT |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_AGPIO_DATA_OUT register */
    #define STEPX_AGPIO_DATA_OUT_ADDRESS						((uint8_t) 0x0F)
    #define STEPX_AGPIO_DATA_OUT_DEFAULT						((uint8_t) 0x00)

    /* STEPx_AGPIO7_DAT_OUT field */
    #define STEPX_AGPIO7_DAT_OUT_MASK							((uint8_t) 0x80)
    #define STEPX_AGPIO_DATA_OUT_STEPX_AGPIO7_DAT_OUT_BITOFFSET	(7)
    #define STEPX_AGPIO7_DAT_OUT_LOW							((uint8_t) 0x00)    // DEFAULT
    #define STEPX_AGPIO7_DAT_OUT_HIGH							((uint8_t) 0x80)

    /* STEPx_AGPIO6_DAT_OUT field */
    #define STEPX_AGPIO6_DAT_OUT_MASK							((uint8_t) 0x40)
    #define STEPX_AGPIO_DATA_OUT_STEPX_AGPIO6_DAT_OUT_BITOFFSET	(6)
    #define STEPX_AGPIO6_DAT_OUT_LOW							((uint8_t) 0x00)    // DEFAULT
    #define STEPX_AGPIO6_DAT_OUT_HIGH							((uint8_t) 0x40)

    /* STEPx_AGPIO5_DAT_OUT field */
    #define STEPX_AGPIO5_DAT_OUT_MASK							((uint8_t) 0x20)
    #define STEPX_AGPIO_DATA_OUT_STEPX_AGPIO5_DAT_OUT_BITOFFSET	(5)
    #define STEPX_AGPIO5_DAT_OUT_LOW							((uint8_t) 0x00)    // DEFAULT
    #define STEPX_AGPIO5_DAT_OUT_HIGH							((uint8_t) 0x20)

    /* STEPx_AGPIO4_DAT_OUT field */
    #define STEPX_AGPIO4_DAT_OUT_MASK							((uint8_t) 0x10)
    #define STEPX_AGPIO_DATA_OUT_STEPX_AGPIO4_DAT_OUT_BITOFFSET	(4)
    #define STEPX_AGPIO4_DAT_OUT_LOW							((uint8_t) 0x00)    // DEFAULT
    #define STEPX_AGPIO4_DAT_OUT_HIGH							((uint8_t) 0x10)

    /* STEPx_AGPIO3_DAT_OUT field */
    #define STEPX_AGPIO3_DAT_OUT_MASK							((uint8_t) 0x08)
    #define STEPX_AGPIO_DATA_OUT_STEPX_AGPIO3_DAT_OUT_BITOFFSET	(3)
    #define STEPX_AGPIO3_DAT_OUT_LOW							((uint8_t) 0x00)    // DEFAULT
    #define STEPX_AGPIO3_DAT_OUT_HIGH							((uint8_t) 0x08)

    /* STEPx_AGPIO2_DAT_OUT field */
    #define STEPX_AGPIO2_DAT_OUT_MASK							((uint8_t) 0x04)
    #define STEPX_AGPIO_DATA_OUT_STEPX_AGPIO2_DAT_OUT_BITOFFSET	(2)
    #define STEPX_AGPIO2_DAT_OUT_LOW							((uint8_t) 0x00)    // DEFAULT
    #define STEPX_AGPIO2_DAT_OUT_HIGH							((uint8_t) 0x04)

    /* STEPx_AGPIO1_DAT_OUT field */
    #define STEPX_AGPIO1_DAT_OUT_MASK							((uint8_t) 0x02)
    #define STEPX_AGPIO_DATA_OUT_STEPX_AGPIO1_DAT_OUT_BITOFFSET	(1)
    #define STEPX_AGPIO1_DAT_OUT_LOW							((uint8_t) 0x00)    // DEFAULT
    #define STEPX_AGPIO1_DAT_OUT_HIGH							((uint8_t) 0x02)

    /* STEPx_AGPIO0_DAT_OUT field */
    #define STEPX_AGPIO0_DAT_OUT_MASK							((uint8_t) 0x01)
    #define STEPX_AGPIO_DATA_OUT_STEPX_AGPIO0_DAT_OUT_BITOFFSET	(0)
    #define STEPX_AGPIO0_DAT_OUT_LOW							((uint8_t) 0x00)    // DEFAULT
    #define STEPX_AGPIO0_DAT_OUT_HIGH							((uint8_t) 0x01)


/* Register 0x10 (STEPX_GPIO_DATA_OUT) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                       RESERVED[3:0]                                       |  STEPx_GPIO3_DAT_OUT |  STEPx_GPIO2_DAT_OUT |  STEPx_GPIO1_DAT_OUT |  STEPx_GPIO0_DAT_OUT |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_GPIO_DATA_OUT register */
    #define STEPX_GPIO_DATA_OUT_ADDRESS							((uint8_t) 0x10)
    #define STEPX_GPIO_DATA_OUT_DEFAULT							((uint8_t) 0x00)

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


/* Register 0x3D (STEPX_REG_MAP_CRC) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                              STEPx_REG_MAP_CRC_VALUE[7:0]                                                                             |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_REG_MAP_CRC register */
    #define STEPX_REG_MAP_CRC_ADDRESS							((uint8_t) 0x3D)
    #define STEPX_REG_MAP_CRC_DEFAULT							((uint8_t) 0x00)

    /* STEPx_REG_MAP_CRC_VALUE field */
    #define STEPX_REG_MAP_CRC_VALUE_MASK						((uint8_t) 0xFF)
    #define STEPX_REG_MAP_CRC_STEPX_REG_MAP_CRC_VALUE_BITOFFSET	(0)


/* Register 0x3E (STEPX_PAGE_INDICATOR) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                               STEPx_PAGE_INDICATOR[7:0]                                                                               |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_PAGE_INDICATOR register */
    #define STEPX_PAGE_INDICATOR_ADDRESS						((uint8_t) 0x3E)
    #define STEPX_PAGE_INDICATOR_DEFAULT						((uint8_t) 0x00)

    /* STEPx_PAGE_INDICATOR field */
    #define STEPX_PAGE_INDICATOR_MASK							((uint8_t) 0xFF)
    #define STEPX_PAGE_INDICATOR_BITOFFSET						(0)


/* Register 0x3F (STEPX_PAGE_POINTER) definition
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |         Bit 7        |         Bit 6        |         Bit 5        |         Bit 4        |         Bit 3        |         Bit 2        |         Bit 1        |         Bit 0        |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                STEPx_PAGE_POINTER[7:0]                                                                                |
 * |---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STEPX_PAGE_POINTER register */
    #define STEPX_PAGE_POINTER_ADDRESS							((uint8_t) 0x3F)
    #define STEPX_PAGE_POINTER_DEFAULT							((uint8_t) 0x00)

    /* STEPx_PAGE_POINTER field */
    #define STEPX_PAGE_POINTER_MASK								((uint8_t) 0xFF)
    #define STEPX_PAGE_POINTER_BITOFFSET						(0)

#endif /* ADS125P08_PAGE1_H_ */