/*
 * @file ads122s1x_page3.h
 *
 * @brief ADS122S1x Descriptor
 *
 * @copyright Copyright (C) 2025 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef ADS122S1X_PAGE3_H_
#define ADS122S1X_PAGE3_H_

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


/* Register 0x00 (DIEID0_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                  DIE_ID0[7:0]                                                                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DIEID0_VREG register */
    #define DIEID0_VREG_ADDRESS                                 ((uint8_t) 0x00)
    #define DIEID0_VREG_DEFAULT                                 ((uint8_t) 0x00)

    /* DIE_ID0 field */
    #define DIEID0_VREG_DIE_ID0_MASK                            ((uint8_t) 0xFF)
    #define DIEID0_VREG_DIE_ID0_BITOFFSET                       (0)


/* Register 0x01 (DIEID1_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                  DIE_ID1[7:0]                                                                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DIEID1_VREG register */
    #define DIEID1_VREG_ADDRESS                                 ((uint8_t) 0x01)
    #define DIEID1_VREG_DEFAULT                                 ((uint8_t) 0x00)

    /* DIE_ID1 field */
    #define DIEID1_VREG_DIE_ID1_MASK                            ((uint8_t) 0xFF)
    #define DIEID1_VREG_DIE_ID1_BITOFFSET                       (0)


/* Register 0x02 (DIEID2_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                  DIE_ID2[7:0]                                                                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DIEID2_VREG register */
    #define DIEID2_VREG_ADDRESS                                 ((uint8_t) 0x02)
    #define DIEID2_VREG_DEFAULT                                 ((uint8_t) 0x00)

    /* DIE_ID2 field */
    #define DIEID2_VREG_DIE_ID2_MASK                            ((uint8_t) 0xFF)
    #define DIEID2_VREG_DIE_ID2_BITOFFSET                       (0)


/* Register 0x03 (DIEID3_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                  DIE_ID3[7:0]                                                                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DIEID3_VREG register */
    #define DIEID3_VREG_ADDRESS                                 ((uint8_t) 0x03)
    #define DIEID3_VREG_DEFAULT                                 ((uint8_t) 0x00)

    /* DIE_ID3 field */
    #define DIEID3_VREG_DIE_ID3_MASK                            ((uint8_t) 0xFF)
    #define DIEID3_VREG_DIE_ID3_BITOFFSET                       (0)


/* Register 0x04 (DIEID4_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                  DIE_ID4[7:0]                                                                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DIEID4_VREG register */
    #define DIEID4_VREG_ADDRESS                                 ((uint8_t) 0x04)
    #define DIEID4_VREG_DEFAULT                                 ((uint8_t) 0x00)

    /* DIE_ID4 field */
    #define DIEID4_VREG_DIE_ID4_MASK                            ((uint8_t) 0xFF)
    #define DIEID4_VREG_DIE_ID4_BITOFFSET                       (0)


/* Register 0x05 (DIEID5_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                  DIE_ID5[7:0]                                                                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DIEID5_VREG register */
    #define DIEID5_VREG_ADDRESS                                 ((uint8_t) 0x05)
    #define DIEID5_VREG_DEFAULT                                 ((uint8_t) 0x00)

    /* DIE_ID5 field */
    #define DIEID5_VREG_DIE_ID5_MASK                            ((uint8_t) 0xFF)
    #define DIEID5_VREG_DIE_ID5_BITOFFSET                       (0)


/* Register 0x06 (DIEID6_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                  DIE_ID6[7:0]                                                                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DIEID6_VREG register */
    #define DIEID6_VREG_ADDRESS                                 ((uint8_t) 0x06)
    #define DIEID6_VREG_DEFAULT                                 ((uint8_t) 0x00)

    /* DIE_ID6 field */
    #define DIEID6_VREG_DIE_ID6_MASK                            ((uint8_t) 0xFF)
    #define DIEID6_VREG_DIE_ID6_BITOFFSET                       (0)


/* Register 0x07 (DEVID_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                               DEV_ID_ACTUAL[7:0]                                                                              |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DEVID_VREG register */
    #define DEVID_VREG_ADDRESS                                  ((uint8_t) 0x07)
    #define DEVID_VREG_DEFAULT                                  ((uint8_t) 0x00)

    /* DEV_ID_ACTUAL field */
    #define DEVID_VREG_DEV_ID_ACTUAL_MASK                       ((uint8_t) 0xFF)
    #define DEVID_VREG_DEV_ID_ACTUAL_BITOFFSET                  (0)


/* Register 0x08 (FSEL_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     FILT_SETTLE     |                OSR_SEL[1:0]               |        GC_DIS       |     CLK_SEL_DIS     |       MUX_SEL       |                CRC_SEL[1:0]               |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* FSEL_VREG register */
    #define FSEL_VREG_ADDRESS                                   ((uint8_t) 0x08)
    #define FSEL_VREG_DEFAULT                                   ((uint8_t) 0x00)

    /* FILT_SETTLE field */
    #define FSEL_VREG_FILT_SETTLE_MASK                          ((uint8_t) 0x80)
    #define FSEL_VREG_FILT_SETTLE_BITOFFSET                     (7)
    #define FSEL_VREG_FILT_SETTLE_ONLYFIRSTCONVERSIONFULLYSETTLESBEFOREDATAREADYSUBSEQUENTCONVERSIONSCOMEFASTER ((uint8_t) 0x00)    // DEFAULT
    #define FSEL_VREG_FILT_SETTLE_EVERYCONVERSIONALLOWEDTOFULLYSETTLEBEFOREDATAREADY ((uint8_t) 0x80)

    /* OSR_SEL field */
    #define FSEL_VREG_OSR_SEL_MASK                              ((uint8_t) 0x60)
    #define FSEL_VREG_OSR_SEL_BITOFFSET                         (5)
    #define FSEL_VREG_OSR_SEL_OSRSELECTEDBYFLTR_OSRBITS         ((uint8_t) 0x00)    // DEFAULT
    #define FSEL_VREG_OSR_SEL_OSRFORCEDTO8                      ((uint8_t) 0x20)
    #define FSEL_VREG_OSR_SEL_OSRFORCEDTO16                     ((uint8_t) 0x40)
    #define FSEL_VREG_OSR_SEL_OSRFORCEDTO32                     ((uint8_t) 0x60)

    /* GC_DIS field */
    #define FSEL_VREG_GC_DIS_MASK                               ((uint8_t) 0x10)
    #define FSEL_VREG_GC_DIS_BITOFFSET                          (4)
    #define FSEL_VREG_GC_DIS_GC_ENANDDELAYBITSAREACTIVE         ((uint8_t) 0x00)    // DEFAULT
    #define FSEL_VREG_GC_DIS_GC_ENANDDELAYBITSAREALLJAMMEDTO0   ((uint8_t) 0x10)

    /* CLK_SEL_DIS field */
    #define FSEL_VREG_CLK_SEL_DIS_MASK                          ((uint8_t) 0x08)
    #define FSEL_VREG_CLK_SEL_DIS_BITOFFSET                     (3)
    #define FSEL_VREG_CLK_SEL_DIS_CLK_SELBITPROGRAMMABLEBYUSER  ((uint8_t) 0x00)    // DEFAULT
    #define FSEL_VREG_CLK_SEL_DIS_CLK_SELBITJAMMEDTO0           ((uint8_t) 0x08)

    /* MUX_SEL field */
    #define FSEL_VREG_MUX_SEL_MASK                              ((uint8_t) 0x04)
    #define FSEL_VREG_MUX_SEL_BITOFFSET                         (2)
    #define FSEL_VREG_MUX_SEL_AVAILBLEANALOGINPUTSAIN0TOAIN7    ((uint8_t) 0x00)    // DEFAULT
    #define FSEL_VREG_MUX_SEL_AVAILBLEANALOGINPUTSAIN0TOAIN3    ((uint8_t) 0x04)

    /* CRC_SEL field */
    #define FSEL_VREG_CRC_SEL_MASK                              ((uint8_t) 0x03)
    #define FSEL_VREG_CRC_SEL_BITOFFSET                         (0)
    #define FSEL_VREG_CRC_SEL_00SPIREG_MAP_CRC_FAULTZSETBYSPIREG_MAP_CRC_ENREGBITS ((uint8_t) 0x00)    // DEFAULT
    #define FSEL_VREG_CRC_SEL_01SPIREG_MAP_CRC_FAULTZISJAMMEDTO1SPIREG_MAP_CRC_ENBITSJAMMEDTO0 ((uint8_t) 0x01)
    #define FSEL_VREG_CRC_SEL_10SPIREG_MAP_CRC_FAULTZACTIVESPIREG_MAP_CRC_ENBITSJAMMEDTO1 ((uint8_t) 0x02)
    #define FSEL_VREG_CRC_SEL_11SPI_CRC_FAULTZSETBYSPI_CRC_ENREGBITREG_MAP_CRC_ENREGBITJAMMEDTO0REG_MAP_CRC_FAULTZJAMMEDTO1 ((uint8_t) 0x03)


/* Register 0x09 (PARTCFG1_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     ONLY_SDODRDY    |      ONLY_4WIRE     |             DATASIZE_SEL[1:0]             |      ONLY_STBY      |             CONV_MODE_SEL[1:0]            |       PWDN_DIS      |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PARTCFG1_VREG register */
    #define PARTCFG1_VREG_ADDRESS                               ((uint8_t) 0x09)
    #define PARTCFG1_VREG_DEFAULT                               ((uint8_t) 0x00)

    /* ONLY_SDODRDY field */
    #define PARTCFG1_VREG_ONLY_SDODRDY_MASK                     ((uint8_t) 0x80)
    #define PARTCFG1_VREG_ONLY_SDODRDY_BITOFFSET                (7)
    #define PARTCFG1_VREG_ONLY_SDODRDY_SDOOVERDRDYOVERPINFUNCTIONSETBYSDO_MODEBIT ((uint8_t) 0x00)    // DEFAULT
    #define PARTCFG1_VREG_ONLY_SDODRDY_ALWAYSCOMBINEDFUNCTIONSDO_MODEBITJAMMEDTO1 ((uint8_t) 0x80)

    /* ONLY_4WIRE field */
    #define PARTCFG1_VREG_ONLY_4WIRE_MASK                       ((uint8_t) 0x40)
    #define PARTCFG1_VREG_ONLY_4WIRE_BITOFFSET                  (6)
    #define PARTCFG1_VREG_ONLY_4WIRE_3OR4WIRESPIDEPENDINGONCSZPINSTATEATPOWERUP ((uint8_t) 0x00)    // DEFAULT
    #define PARTCFG1_VREG_ONLY_4WIRE_ALWAYS4WIREMODEWIRE_MODEBITJAMMEDTO0 ((uint8_t) 0x40)

    /* DATASIZE_SEL field */
    #define PARTCFG1_VREG_DATASIZE_SEL_MASK                     ((uint8_t) 0x30)
    #define PARTCFG1_VREG_DATASIZE_SEL_BITOFFSET                (4)
    #define PARTCFG1_VREG_DATASIZE_SEL_24BITRESOLUTION          ((uint8_t) 0x00)    // DEFAULT
    #define PARTCFG1_VREG_DATASIZE_SEL_16BITRESOLUTION          ((uint8_t) 0x10)
    #define PARTCFG1_VREG_DATASIZE_SEL_12BITRESOLUTION          ((uint8_t) 0x20)
    #define PARTCFG1_VREG_DATASIZE_SEL_12BITRESOLUTION          ((uint8_t) 0x30)

    /* ONLY_STBY field */
    #define PARTCFG1_VREG_ONLY_STBY_MASK                        ((uint8_t) 0x08)
    #define PARTCFG1_VREG_ONLY_STBY_BITOFFSET                   (3)
    #define PARTCFG1_VREG_ONLY_STBY_STANDBYSTATUSSETBYSTDBY_MODEBIT ((uint8_t) 0x00)    // DEFAULT
    #define PARTCFG1_VREG_ONLY_STBY_ALWAYSINSTANDBYWHENIDLESTBY_MODEBITJAMMEDTO1 ((uint8_t) 0x08)

    /* CONV_MODE_SEL field */
    #define PARTCFG1_VREG_CONV_MODE_SEL_MASK                    ((uint8_t) 0x06)
    #define PARTCFG1_VREG_CONV_MODE_SEL_BITOFFSET               (1)
    #define PARTCFG1_VREG_CONV_MODE_SEL_CONTONESHOT             ((uint8_t) 0x00)    // DEFAULT
    #define PARTCFG1_VREG_CONV_MODE_SEL_ONLYCONT                ((uint8_t) 0x02)
    #define PARTCFG1_VREG_CONV_MODE_SEL_ONLYONESHOT             ((uint8_t) 0x04)
    #define PARTCFG1_VREG_CONV_MODE_SEL_ONLYONESHOT             ((uint8_t) 0x06)

    /* PWDN_DIS field */
    #define PARTCFG1_VREG_PWDN_DIS_MASK                         ((uint8_t) 0x01)
    #define PARTCFG1_VREG_PWDN_DIS_BITOFFSET                    (0)
    #define PARTCFG1_VREG_PWDN_DIS_PWDNBITISACTIVE              ((uint8_t) 0x00)    // DEFAULT
    #define PARTCFG1_VREG_PWDN_DIS_PWDNISINACTIVEFORCEDTO1B0    ((uint8_t) 0x01)


/* Register 0x0A (PARTCFG2_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |    GPIO3_SRC_DIS    |    GPIO2_SRC_DIS    |    CONT_READ_DIS    |    MEM_FAULTZ_DIS   |     SYS_MON_SEL     |      REF_UV_DIS     |     DBL_SAMP_DIS    |      MAXGAIN10      |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PARTCFG2_VREG register */
    #define PARTCFG2_VREG_ADDRESS                               ((uint8_t) 0x0A)
    #define PARTCFG2_VREG_DEFAULT                               ((uint8_t) 0x00)

    /* GPIO3_SRC_DIS field */
    #define PARTCFG2_VREG_GPIO3_SRC_DIS_MASK                    ((uint8_t) 0x80)
    #define PARTCFG2_VREG_GPIO3_SRC_DIS_BITOFFSET               (7)
    #define PARTCFG2_VREG_GPIO3_SRC_DIS_GPIO3_SRCBITISACTIVE    ((uint8_t) 0x00)    // DEFAULT
    #define PARTCFG2_VREG_GPIO3_SRC_DIS_GPIO3_SRCISINACTIVEFORCEDTO1B0 ((uint8_t) 0x80)

    /* GPIO2_SRC_DIS field */
    #define PARTCFG2_VREG_GPIO2_SRC_DIS_MASK                    ((uint8_t) 0x40)
    #define PARTCFG2_VREG_GPIO2_SRC_DIS_BITOFFSET               (6)
    #define PARTCFG2_VREG_GPIO2_SRC_DIS_GPIO2_SRCBITISACTIVE    ((uint8_t) 0x00)    // DEFAULT
    #define PARTCFG2_VREG_GPIO2_SRC_DIS_GPIO2_SRCISINACTIVEFORCEDTO1B0 ((uint8_t) 0x40)

    /* CONT_READ_DIS field */
    #define PARTCFG2_VREG_CONT_READ_DIS_MASK                    ((uint8_t) 0x20)
    #define PARTCFG2_VREG_CONT_READ_DIS_BITOFFSET               (5)
    #define PARTCFG2_VREG_CONT_READ_DIS_CONT_READ_ENISACTIVE    ((uint8_t) 0x00)    // DEFAULT
    #define PARTCFG2_VREG_CONT_READ_DIS_CONT_READ_ENISFORCEDTO0 ((uint8_t) 0x20)

    /* MEM_FAULTZ_DIS field */
    #define PARTCFG2_VREG_MEM_FAULTZ_DIS_MASK                   ((uint8_t) 0x10)
    #define PARTCFG2_VREG_MEM_FAULTZ_DIS_BITOFFSET              (4)
    #define PARTCFG2_VREG_MEM_FAULTZ_DIS_MEM_FAULTZBITISACTIVE  ((uint8_t) 0x00)    // DEFAULT
    #define PARTCFG2_VREG_MEM_FAULTZ_DIS_MEM_FAULTZBITISFORCEDTO1 ((uint8_t) 0x10)

    /* SYS_MON_SEL field */
    #define PARTCFG2_VREG_SYS_MON_SEL_MASK                      ((uint8_t) 0x08)
    #define PARTCFG2_VREG_SYS_MON_SEL_BITOFFSET                 (3)
    #define PARTCFG2_VREG_SYS_MON_SEL_SYS_MONALLBITSACTIVE      ((uint8_t) 0x00)    // DEFAULT
    #define PARTCFG2_VREG_SYS_MON_SEL_SYS_MONMSBBITFORCEDTO0    ((uint8_t) 0x08)

    /* REF_UV_DIS field */
    #define PARTCFG2_VREG_REF_UV_DIS_MASK                       ((uint8_t) 0x04)
    #define PARTCFG2_VREG_REF_UV_DIS_BITOFFSET                  (2)
    #define PARTCFG2_VREG_REF_UV_DIS_REF_UVZANDREF_UV_ENBITAREACTIVE ((uint8_t) 0x00)    // DEFAULT
    #define PARTCFG2_VREG_REF_UV_DIS_REF_UVZFORCEDTO1REF_UV_ENISFORCEDTO0ROBITS ((uint8_t) 0x04)

    /* DBL_SAMP_DIS field */
    #define PARTCFG2_VREG_DBL_SAMP_DIS_MASK                     ((uint8_t) 0x02)
    #define PARTCFG2_VREG_DBL_SAMP_DIS_BITOFFSET                (1)
    #define PARTCFG2_VREG_DBL_SAMP_DIS_DOUBLESAMPLINGISACTIVE   ((uint8_t) 0x00)    // DEFAULT
    #define PARTCFG2_VREG_DBL_SAMP_DIS_DOUBLESAMPLINGISINACTIVE ((uint8_t) 0x02)

    /* MAXGAIN10 field */
    #define PARTCFG2_VREG_MAXGAIN10_MASK                        ((uint8_t) 0x01)
    #define PARTCFG2_VREG_MAXGAIN10_BITOFFSET                   (0)
    #define PARTCFG2_VREG_MAXGAIN10_FULLGAINTABLEACTIVE         ((uint8_t) 0x00)    // DEFAULT
    #define PARTCFG2_VREG_MAXGAIN10_GAINLIMITEDTO10             ((uint8_t) 0x01)


/* Register 0x0B (SPEED1_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |              REF_BUF_SEL[1:0]             |                         SPEED_REG01[2:0]                        |                         SPEED_REG00[2:0]                        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SPEED1_VREG register */
    #define SPEED1_VREG_ADDRESS                                 ((uint8_t) 0x0B)
    #define SPEED1_VREG_DEFAULT                                 ((uint8_t) 0x00)

    /* REF_BUF_SEL field */
    #define SPEED1_VREG_REF_BUF_SEL_MASK                        ((uint8_t) 0xC0)
    #define SPEED1_VREG_REF_BUF_SEL_BITOFFSET                   (6)
    #define SPEED1_VREG_REF_BUF_SEL_BOTHREFP_BUF_ENANDREFN_BUF_ENACTIVE ((uint8_t) 0x00)    // DEFAULT
    #define SPEED1_VREG_REF_BUF_SEL_REFP_BUF_ENFORCEDTO0ANDREFN_BUF_ENFORCEDTO0 ((uint8_t) 0x40)
    #define SPEED1_VREG_REF_BUF_SEL_REFP_BUF_ENFORCEDTO1REFN_BUF_ENFORCEDTO1 ((uint8_t) 0x80)
    #define SPEED1_VREG_REF_BUF_SEL_REFP_BUF_ENFORCEDTO1REFN_BUFENFORCEDTO0 ((uint8_t) 0xC0)

    /* SPEED_REG01 field */
    #define SPEED1_VREG_SPEED_REG01_MASK                        ((uint8_t) 0x38)
    #define SPEED1_VREG_SPEED_REG01_BITOFFSET                   (3)
    #define SPEED1_VREG_SPEED_REG01_FMOD32KHZ                   ((uint8_t) 0x00)    // DEFAULT
    #define SPEED1_VREG_SPEED_REG01_FMOD64KHZ                   ((uint8_t) 0x08)
    #define SPEED1_VREG_SPEED_REG01_FMOD128KHZ                  ((uint8_t) 0x10)
    #define SPEED1_VREG_SPEED_REG01_FMOD256KHZ                  ((uint8_t) 0x18)
    #define SPEED1_VREG_SPEED_REG01_FMOD512KHZ                  ((uint8_t) 0x20)
    #define SPEED1_VREG_SPEED_REG01_FMOD1024KHZ                 ((uint8_t) 0x28)
    #define SPEED1_VREG_SPEED_REG01_FMOD1024KHZ                 ((uint8_t) 0x30)
    #define SPEED1_VREG_SPEED_REG01_FMOD1024KHZ                 ((uint8_t) 0x38)

    /* SPEED_REG00 field */
    #define SPEED1_VREG_SPEED_REG00_MASK                        ((uint8_t) 0x07)
    #define SPEED1_VREG_SPEED_REG00_BITOFFSET                   (0)
    #define SPEED1_VREG_SPEED_REG00_FMOD32KHZ                   ((uint8_t) 0x00)    // DEFAULT
    #define SPEED1_VREG_SPEED_REG00_FMOD64KHZ                   ((uint8_t) 0x01)
    #define SPEED1_VREG_SPEED_REG00_FMOD128KHZ                  ((uint8_t) 0x02)
    #define SPEED1_VREG_SPEED_REG00_FMOD256KHZ                  ((uint8_t) 0x03)
    #define SPEED1_VREG_SPEED_REG00_FMOD512KHZ                  ((uint8_t) 0x04)
    #define SPEED1_VREG_SPEED_REG00_FMOD1024KHZ                 ((uint8_t) 0x05)
    #define SPEED1_VREG_SPEED_REG00_FMOD1024KHZ                 ((uint8_t) 0x06)
    #define SPEED1_VREG_SPEED_REG00_FMOD1024KHZ                 ((uint8_t) 0x07)


/* Register 0x0C (SPEED2_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       BOCS_DIS      |       IDAC_DIS      |                         SPEED_REG11[2:0]                        |                         SPEED_REG10[2:0]                        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SPEED2_VREG register */
    #define SPEED2_VREG_ADDRESS                                 ((uint8_t) 0x0C)
    #define SPEED2_VREG_DEFAULT                                 ((uint8_t) 0x00)

    /* BOCS_DIS field */
    #define SPEED2_VREG_BOCS_DIS_MASK                           ((uint8_t) 0x80)
    #define SPEED2_VREG_BOCS_DIS_BITOFFSET                      (7)
    #define SPEED2_VREG_BOCS_DIS_BOCSBITSAREACTIVE              ((uint8_t) 0x00)    // DEFAULT
    #define SPEED2_VREG_BOCS_DIS_BOCSBITSAREFORCEDTO2B00        ((uint8_t) 0x80)

    /* IDAC_DIS field */
    #define SPEED2_VREG_IDAC_DIS_MASK                           ((uint8_t) 0x40)
    #define SPEED2_VREG_IDAC_DIS_BITOFFSET                      (6)
    #define SPEED2_VREG_IDAC_DIS_IB12BMAGIB12BMUXIUNITBITSAREACTIVE ((uint8_t) 0x00)    // DEFAULT
    #define SPEED2_VREG_IDAC_DIS_IB12BMAGIB12BMUXIUNITBITSAREFORCEDTO0 ((uint8_t) 0x40)

    /* SPEED_REG11 field */
    #define SPEED2_VREG_SPEED_REG11_MASK                        ((uint8_t) 0x38)
    #define SPEED2_VREG_SPEED_REG11_BITOFFSET                   (3)
    #define SPEED2_VREG_SPEED_REG11_FMOD32KHZ                   ((uint8_t) 0x00)    // DEFAULT
    #define SPEED2_VREG_SPEED_REG11_FMOD64KHZ                   ((uint8_t) 0x08)
    #define SPEED2_VREG_SPEED_REG11_FMOD128KHZ                  ((uint8_t) 0x10)
    #define SPEED2_VREG_SPEED_REG11_FMOD256KHZ                  ((uint8_t) 0x18)
    #define SPEED2_VREG_SPEED_REG11_FMOD512KHZ                  ((uint8_t) 0x20)
    #define SPEED2_VREG_SPEED_REG11_FMOD1024KHZ                 ((uint8_t) 0x28)
    #define SPEED2_VREG_SPEED_REG11_FMOD1024KHZ                 ((uint8_t) 0x30)
    #define SPEED2_VREG_SPEED_REG11_FMOD1024KHZ                 ((uint8_t) 0x38)

    /* SPEED_REG10 field */
    #define SPEED2_VREG_SPEED_REG10_MASK                        ((uint8_t) 0x07)
    #define SPEED2_VREG_SPEED_REG10_BITOFFSET                   (0)
    #define SPEED2_VREG_SPEED_REG10_FMOD32KHZ                   ((uint8_t) 0x00)    // DEFAULT
    #define SPEED2_VREG_SPEED_REG10_FMOD64KHZ                   ((uint8_t) 0x01)
    #define SPEED2_VREG_SPEED_REG10_FMOD128KHZ                  ((uint8_t) 0x02)
    #define SPEED2_VREG_SPEED_REG10_FMOD256KHZ                  ((uint8_t) 0x03)
    #define SPEED2_VREG_SPEED_REG10_FMOD512KHZ                  ((uint8_t) 0x04)
    #define SPEED2_VREG_SPEED_REG10_FMOD1024KHZ                 ((uint8_t) 0x05)
    #define SPEED2_VREG_SPEED_REG10_FMOD1024KHZ                 ((uint8_t) 0x06)
    #define SPEED2_VREG_SPEED_REG10_FMOD1024KHZ                 ((uint8_t) 0x07)


/* Register 0x0D (OSCLSB_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       QUAL_SEL      |     ONLY_INT_REF    |                                                         TRIM_OSC_LSB[5:0]                                                         |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OSCLSB_VREG register */
    #define OSCLSB_VREG_ADDRESS                                 ((uint8_t) 0x0D)
    #define OSCLSB_VREG_DEFAULT                                 ((uint8_t) 0x00)

    /* QUAL_SEL field */
    #define OSCLSB_VREG_QUAL_SEL_MASK                           ((uint8_t) 0x80)
    #define OSCLSB_VREG_QUAL_SEL_BITOFFSET                      (7)

    /* ONLY_INT_REF field */
    #define OSCLSB_VREG_ONLY_INT_REF_MASK                       ((uint8_t) 0x40)
    #define OSCLSB_VREG_ONLY_INT_REF_BITOFFSET                  (6)
    #define OSCLSB_VREG_ONLY_INT_REF_NORMALOPERATION            ((uint8_t) 0x00)    // DEFAULT
    #define OSCLSB_VREG_ONLY_INT_REF_REF_SEL10JAMMEDTO00        ((uint8_t) 0x40)

    /* TRIM_OSC_LSB field */
    #define OSCLSB_VREG_TRIM_OSC_LSB_MASK                       ((uint8_t) 0x3F)
    #define OSCLSB_VREG_TRIM_OSC_LSB_BITOFFSET                  (0)


/* Register 0x0E (OSCMSB_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     OVERLOAD_DIS    |    PGA_FSTART_DIS   |                                                         TRIM_OSC_MSB[5:0]                                                         |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OSCMSB_VREG register */
    #define OSCMSB_VREG_ADDRESS                                 ((uint8_t) 0x0E)
    #define OSCMSB_VREG_DEFAULT                                 ((uint8_t) 0x00)

    /* OVERLOAD_DIS field */
    #define OSCMSB_VREG_OVERLOAD_DIS_MASK                       ((uint8_t) 0x80)
    #define OSCMSB_VREG_OVERLOAD_DIS_BITOFFSET                  (7)

    /* PGA_FSTART_DIS field */
    #define OSCMSB_VREG_PGA_FSTART_DIS_MASK                     ((uint8_t) 0x40)
    #define OSCMSB_VREG_PGA_FSTART_DIS_BITOFFSET                (6)

    /* TRIM_OSC_MSB field */
    #define OSCMSB_VREG_TRIM_OSC_MSB_MASK                       ((uint8_t) 0x3F)
    #define OSCMSB_VREG_TRIM_OSC_MSB_BITOFFSET                  (0)


/* Register 0x0F (OSCLSB_LP_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     CLKOUTPAD_EN    |      OSC_LP_DIS     |                                                        TRIM_OSC_LSB_LP[5:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OSCLSB_LP_VREG register */
    #define OSCLSB_LP_VREG_ADDRESS                              ((uint8_t) 0x0F)
    #define OSCLSB_LP_VREG_DEFAULT                              ((uint8_t) 0x00)

    /* CLKOUTPAD_EN field */
    #define OSCLSB_LP_VREG_CLKOUTPAD_EN_MASK                    ((uint8_t) 0x80)
    #define OSCLSB_LP_VREG_CLKOUTPAD_EN_BITOFFSET               (7)
    #define OSCLSB_LP_VREG_CLKOUTPAD_EN_NOOUTPUTCLOCKONCLKOUTPIN ((uint8_t) 0x00)    // DEFAULT
    #define OSCLSB_LP_VREG_CLKOUTPAD_EN_OUTPUT512KHZCLOCKONCLKOUTPAD ((uint8_t) 0x80)

    /* OSC_LP_DIS field */
    #define OSCLSB_LP_VREG_OSC_LP_DIS_MASK                      ((uint8_t) 0x40)
    #define OSCLSB_LP_VREG_OSC_LP_DIS_BITOFFSET                 (6)

    /* TRIM_OSC_LSB_LP field */
    #define OSCLSB_LP_VREG_TRIM_OSC_LSB_LP_MASK                 ((uint8_t) 0x3F)
    #define OSCLSB_LP_VREG_TRIM_OSC_LSB_LP_BITOFFSET            (0)


/* Register 0x10 (OSCMSB_LP_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |           TRIM_OSC_LP_RNGXT[1:0]          |                                                        TRIM_OSC_MSB_LP[5:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OSCMSB_LP_VREG register */
    #define OSCMSB_LP_VREG_ADDRESS                              ((uint8_t) 0x10)
    #define OSCMSB_LP_VREG_DEFAULT                              ((uint8_t) 0x00)

    /* TRIM_OSC_LP_RNGXT field */
    #define OSCMSB_LP_VREG_TRIM_OSC_LP_RNGXT_MASK               ((uint8_t) 0xC0)
    #define OSCMSB_LP_VREG_TRIM_OSC_LP_RNGXT_BITOFFSET          (6)

    /* TRIM_OSC_MSB_LP field */
    #define OSCMSB_LP_VREG_TRIM_OSC_MSB_LP_MASK                 ((uint8_t) 0x3F)
    #define OSCMSB_LP_VREG_TRIM_OSC_MSB_LP_BITOFFSET            (0)


/* Register 0x11 (REFTRM1_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |    SPEED_MODE_DIS   |              STATUS_SEL[1:0]              |                                            REF_CC_KNEE_1V25[4:0]                                            |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REFTRM1_VREG register */
    #define REFTRM1_VREG_ADDRESS                                ((uint8_t) 0x11)
    #define REFTRM1_VREG_DEFAULT                                ((uint8_t) 0x00)

    /* SPEED_MODE_DIS field */
    #define REFTRM1_VREG_SPEED_MODE_DIS_MASK                    ((uint8_t) 0x80)
    #define REFTRM1_VREG_SPEED_MODE_DIS_BITOFFSET               (7)
    #define REFTRM1_VREG_SPEED_MODE_DIS_NORMALOPERATION         ((uint8_t) 0x00)    // DEFAULT
    #define REFTRM1_VREG_SPEED_MODE_DIS_SPEED_MODE10ISJAMMEDTO00 ((uint8_t) 0x80)

    /* STATUS_SEL field */
    #define REFTRM1_VREG_STATUS_SEL_MASK                        ((uint8_t) 0x60)
    #define REFTRM1_VREG_STATUS_SEL_BITOFFSET                   (5)
    #define REFTRM1_VREG_STATUS_SEL_STATUSWORDUSAGESETBYSTATUS_ENUSERREGBIT ((uint8_t) 0x00)    // DEFAULT
    #define REFTRM1_VREG_STATUS_SEL_STATUSWORDALWAYSOFFSTATUS_ENJAMMEDTO0 ((uint8_t) 0x20)
    #define REFTRM1_VREG_STATUS_SEL_STATUSWORDALWAYSONSTATUS_ENJAMMEDTO1 ((uint8_t) 0x40)
    #define REFTRM1_VREG_STATUS_SEL_STATUSWORDALWAYSONSTATUS_ENJAMMEDTO1 ((uint8_t) 0x60)

    /* REF_CC_KNEE_1V25 field */
    #define REFTRM1_VREG_REF_CC_KNEE_1V25_MASK                  ((uint8_t) 0x1F)
    #define REFTRM1_VREG_REF_CC_KNEE_1V25_BITOFFSET             (0)


/* Register 0x12 (REFTRM2_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |   CHOP_PGA_PREQ_PH  |               CLKOUT_PH[1:0]              |                                             REF_CC_KNEE_2V5[4:0]                                            |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REFTRM2_VREG register */
    #define REFTRM2_VREG_ADDRESS                                ((uint8_t) 0x12)
    #define REFTRM2_VREG_DEFAULT                                ((uint8_t) 0x00)

    /* CHOP_PGA_PREQ_PH field */
    #define REFTRM2_VREG_CHOP_PGA_PREQ_PH_MASK                  ((uint8_t) 0x80)
    #define REFTRM2_VREG_CHOP_PGA_PREQ_PH_BITOFFSET             (7)
    #define REFTRM2_VREG_CHOP_PGA_PREQ_PH_BETWEENPH2FALLANDPH1RISE ((uint8_t) 0x00)    // DEFAULT
    #define REFTRM2_VREG_CHOP_PGA_PREQ_PH_BETWEENPH1FALLANDPH2RISE ((uint8_t) 0x80)

    /* CLKOUT_PH field */
    #define REFTRM2_VREG_CLKOUT_PH_MASK                         ((uint8_t) 0x60)
    #define REFTRM2_VREG_CLKOUT_PH_BITOFFSET                    (5)
    #define REFTRM2_VREG_CLKOUT_PH_0DEGREESHIFT                 ((uint8_t) 0x00)    // DEFAULT
    #define REFTRM2_VREG_CLKOUT_PH_90DEGREESHIFT                ((uint8_t) 0x20)
    #define REFTRM2_VREG_CLKOUT_PH_180DEGREESHIFT               ((uint8_t) 0x40)
    #define REFTRM2_VREG_CLKOUT_PH_270DEGREESHIFT               ((uint8_t) 0x60)

    /* REF_CC_KNEE_2V5 field */
    #define REFTRM2_VREG_REF_CC_KNEE_2V5_MASK                   ((uint8_t) 0x1F)
    #define REFTRM2_VREG_REF_CC_KNEE_2V5_BITOFFSET              (0)


/* Register 0x13 (REFTRM3_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                         CLKOUT_DLY[2:0]                         |                                               REF_CC_1V25[4:0]                                              |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REFTRM3_VREG register */
    #define REFTRM3_VREG_ADDRESS                                ((uint8_t) 0x13)
    #define REFTRM3_VREG_DEFAULT                                ((uint8_t) 0x00)

    /* CLKOUT_DLY field */
    #define REFTRM3_VREG_CLKOUT_DLY_MASK                        ((uint8_t) 0xE0)
    #define REFTRM3_VREG_CLKOUT_DLY_BITOFFSET                   (5)
    #define REFTRM3_VREG_CLKOUT_DLY_0NSTYPDELAY                 ((uint8_t) 0x00)    // DEFAULT
    #define REFTRM3_VREG_CLKOUT_DLY_5NSTYPDELAY                 ((uint8_t) 0x20)
    #define REFTRM3_VREG_CLKOUT_DLY_10NSTYPDELAY                ((uint8_t) 0x40)
    #define REFTRM3_VREG_CLKOUT_DLY_15NSTYPDELAY                ((uint8_t) 0x60)
    #define REFTRM3_VREG_CLKOUT_DLY_20NSTYPDELAY                ((uint8_t) 0x80)
    #define REFTRM3_VREG_CLKOUT_DLY_25NSTYPDELAY                ((uint8_t) 0xA0)
    #define REFTRM3_VREG_CLKOUT_DLY_30NSTYPDELAY                ((uint8_t) 0xC0)
    #define REFTRM3_VREG_CLKOUT_DLY_40NSTYPDELAY                ((uint8_t) 0xE0)

    /* REF_CC_1V25 field */
    #define REFTRM3_VREG_REF_CC_1V25_MASK                       ((uint8_t) 0x1F)
    #define REFTRM3_VREG_REF_CC_1V25_BITOFFSET                  (0)


/* Register 0x14 (REFTRM4_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     PGAPREQ_PHCC    |   REF_FLIPCC_1V25   |    REF_FLIPCC_2V5   |                                               REF_CC_2V5[4:0]                                               |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REFTRM4_VREG register */
    #define REFTRM4_VREG_ADDRESS                                ((uint8_t) 0x14)
    #define REFTRM4_VREG_DEFAULT                                ((uint8_t) 0x00)

    /* PGAPREQ_PHCC field */
    #define REFTRM4_VREG_PGAPREQ_PHCC_MASK                      ((uint8_t) 0x80)
    #define REFTRM4_VREG_PGAPREQ_PHCC_BITOFFSET                 (7)
    #define REFTRM4_VREG_PGAPREQ_PHCC_18PHASE                   ((uint8_t) 0x00)    // DEFAULT
    #define REFTRM4_VREG_PGAPREQ_PHCC_14PHASE                   ((uint8_t) 0x80)

    /* REF_FLIPCC_1V25 field */
    #define REFTRM4_VREG_REF_FLIPCC_1V25_MASK                   ((uint8_t) 0x40)
    #define REFTRM4_VREG_REF_FLIPCC_1V25_BITOFFSET              (6)

    /* REF_FLIPCC_2V5 field */
    #define REFTRM4_VREG_REF_FLIPCC_2V5_MASK                    ((uint8_t) 0x20)
    #define REFTRM4_VREG_REF_FLIPCC_2V5_BITOFFSET               (5)

    /* REF_CC_2V5 field */
    #define REFTRM4_VREG_REF_CC_2V5_MASK                        ((uint8_t) 0x1F)
    #define REFTRM4_VREG_REF_CC_2V5_BITOFFSET                   (0)


/* Register 0x15 (REFTRM5_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                   REF_HTCC_1V25[3:0]                                  |                                   REF_HTCC_2V5[3:0]                                   |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REFTRM5_VREG register */
    #define REFTRM5_VREG_ADDRESS                                ((uint8_t) 0x15)
    #define REFTRM5_VREG_DEFAULT                                ((uint8_t) 0x00)

    /* REF_HTCC_1V25 field */
    #define REFTRM5_VREG_REF_HTCC_1V25_MASK                     ((uint8_t) 0xF0)
    #define REFTRM5_VREG_REF_HTCC_1V25_BITOFFSET                (4)

    /* REF_HTCC_2V5 field */
    #define REFTRM5_VREG_REF_HTCC_2V5_MASK                      ((uint8_t) 0x0F)
    #define REFTRM5_VREG_REF_HTCC_2V5_BITOFFSET                 (0)


/* Register 0x16 (REFTRM6_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                              REF_INULL_1V25[7:0]                                                                              |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REFTRM6_VREG register */
    #define REFTRM6_VREG_ADDRESS                                ((uint8_t) 0x16)
    #define REFTRM6_VREG_DEFAULT                                ((uint8_t) 0x00)

    /* REF_INULL_1V25 field */
    #define REFTRM6_VREG_REF_INULL_1V25_MASK                    ((uint8_t) 0xFF)
    #define REFTRM6_VREG_REF_INULL_1V25_BITOFFSET               (0)


/* Register 0x17 (REFTRM7_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                               REF_INULL_2V5[7:0]                                                                              |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REFTRM7_VREG register */
    #define REFTRM7_VREG_ADDRESS                                ((uint8_t) 0x17)
    #define REFTRM7_VREG_DEFAULT                                ((uint8_t) 0x00)

    /* REF_INULL_2V5 field */
    #define REFTRM7_VREG_REF_INULL_2V5_MASK                     ((uint8_t) 0xFF)
    #define REFTRM7_VREG_REF_INULL_2V5_BITOFFSET                (0)


/* Register 0x18 (REFTRM8_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     IPGAIRSHIFT     |                                   REF_LVL_1V25[3:0]                                   |                      REF_ACC_1V25_MSB[2:0]                      |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REFTRM8_VREG register */
    #define REFTRM8_VREG_ADDRESS                                ((uint8_t) 0x18)
    #define REFTRM8_VREG_DEFAULT                                ((uint8_t) 0x00)

    /* IPGAIRSHIFT field */
    #define REFTRM8_VREG_IPGAIRSHIFT_MASK                       ((uint8_t) 0x80)
    #define REFTRM8_VREG_IPGAIRSHIFT_BITOFFSET                  (7)
    #define REFTRM8_VREG_IPGAIRSHIFT_NOCHANGE                   ((uint8_t) 0x00)    // DEFAULT
    #define REFTRM8_VREG_IPGAIRSHIFT_INCREASEPGAIRSHIFTCURRENTBY16201 ((uint8_t) 0x80)

    /* REF_LVL_1V25 field */
    #define REFTRM8_VREG_REF_LVL_1V25_MASK                      ((uint8_t) 0x78)
    #define REFTRM8_VREG_REF_LVL_1V25_BITOFFSET                 (3)

    /* REF_ACC_1V25_MSB field */
    #define REFTRM8_VREG_REF_ACC_1V25_MSB_MASK                  ((uint8_t) 0x07)
    #define REFTRM8_VREG_REF_ACC_1V25_MSB_BITOFFSET             (0)


/* Register 0x19 (REFTRM9_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                             REF_ACC_1V25_LSB[7:0]                                                                             |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REFTRM9_VREG register */
    #define REFTRM9_VREG_ADDRESS                                ((uint8_t) 0x19)
    #define REFTRM9_VREG_DEFAULT                                ((uint8_t) 0x00)

    /* REF_ACC_1V25_LSB field */
    #define REFTRM9_VREG_REF_ACC_1V25_LSB_MASK                  ((uint8_t) 0xFF)
    #define REFTRM9_VREG_REF_ACC_1V25_LSB_BITOFFSET             (0)


/* Register 0x1A (REFTRM10_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |   IPGA_1750NA_MSB   |                                    REF_LVL_2V5[3:0]                                   |                       REF_ACC_2V5_MSB[2:0]                      |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REFTRM10_VREG register */
    #define REFTRM10_VREG_ADDRESS                               ((uint8_t) 0x1A)
    #define REFTRM10_VREG_DEFAULT                               ((uint8_t) 0x00)

    /* IPGA_1750NA_MSB field */
    #define REFTRM10_VREG_IPGA_1750NA_MSB_MASK                  ((uint8_t) 0x80)
    #define REFTRM10_VREG_IPGA_1750NA_MSB_BITOFFSET             (7)

    /* REF_LVL_2V5 field */
    #define REFTRM10_VREG_REF_LVL_2V5_MASK                      ((uint8_t) 0x78)
    #define REFTRM10_VREG_REF_LVL_2V5_BITOFFSET                 (3)

    /* REF_ACC_2V5_MSB field */
    #define REFTRM10_VREG_REF_ACC_2V5_MSB_MASK                  ((uint8_t) 0x07)
    #define REFTRM10_VREG_REF_ACC_2V5_MSB_BITOFFSET             (0)


/* Register 0x1B (REFTRM11_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                              REF_ACC_2V5_LSB[7:0]                                                                             |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REFTRM11_VREG register */
    #define REFTRM11_VREG_ADDRESS                               ((uint8_t) 0x1B)
    #define REFTRM11_VREG_DEFAULT                               ((uint8_t) 0x00)

    /* REF_ACC_2V5_LSB field */
    #define REFTRM11_VREG_REF_ACC_2V5_LSB_MASK                  ((uint8_t) 0xFF)
    #define REFTRM11_VREG_REF_ACC_2V5_LSB_BITOFFSET             (0)


/* Register 0x1C (REFTRM12_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                            REF_TC_1STORD_1V25[7:0]                                                                            |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REFTRM12_VREG register */
    #define REFTRM12_VREG_ADDRESS                               ((uint8_t) 0x1C)
    #define REFTRM12_VREG_DEFAULT                               ((uint8_t) 0x00)

    /* REF_TC_1STORD_1V25 field */
    #define REFTRM12_VREG_REF_TC_1STORD_1V25_MASK               ((uint8_t) 0xFF)
    #define REFTRM12_VREG_REF_TC_1STORD_1V25_BITOFFSET          (0)


/* Register 0x1D (REFTRM13_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                             REF_TC_1STORD_2V5[7:0]                                                                            |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REFTRM13_VREG register */
    #define REFTRM13_VREG_ADDRESS                               ((uint8_t) 0x1D)
    #define REFTRM13_VREG_DEFAULT                               ((uint8_t) 0x00)

    /* REF_TC_1STORD_2V5 field */
    #define REFTRM13_VREG_REF_TC_1STORD_2V5_MASK                ((uint8_t) 0xFF)
    #define REFTRM13_VREG_REF_TC_1STORD_2V5_BITOFFSET           (0)


/* Register 0x1E (REFINITACC_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                 REF_INITACC_1V25[3:0]                                 |                                  REF_INITACC_2V5[3:0]                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REFINITACC_VREG register */
    #define REFINITACC_VREG_ADDRESS                             ((uint8_t) 0x1E)
    #define REFINITACC_VREG_DEFAULT                             ((uint8_t) 0x00)

    /* REF_INITACC_1V25 field */
    #define REFINITACC_VREG_REF_INITACC_1V25_MASK               ((uint8_t) 0xF0)
    #define REFINITACC_VREG_REF_INITACC_1V25_BITOFFSET          (4)

    /* REF_INITACC_2V5 field */
    #define REFINITACC_VREG_REF_INITACC_2V5_MASK                ((uint8_t) 0x0F)
    #define REFINITACC_VREG_REF_INITACC_2V5_BITOFFSET           (0)


/* Register 0x1F (BUFOPTS_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                 SPARE[1:0]                |                                                         REF_BUF_OPTS[5:0]                                                         |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* BUFOPTS_VREG register */
    #define BUFOPTS_VREG_ADDRESS                                ((uint8_t) 0x1F)
    #define BUFOPTS_VREG_DEFAULT                                ((uint8_t) 0x00)

    /* SPARE field */
    #define BUFOPTS_VREG_SPARE_MASK                             ((uint8_t) 0xC0)
    #define BUFOPTS_VREG_SPARE_BITOFFSET                        (6)

    /* REF_BUF_OPTS field */
    #define BUFOPTS_VREG_REF_BUF_OPTS_MASK                      ((uint8_t) 0x3F)
    #define BUFOPTS_VREG_REF_BUF_OPTS_BITOFFSET                 (0)


/* Register 0x20 (BIASTRM_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * | IOTA1_512_1024K_MSB |                                                                      TRIM_BIAS[6:0]                                                                     |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* BIASTRM_VREG register */
    #define BIASTRM_VREG_ADDRESS                                ((uint8_t) 0x20)
    #define BIASTRM_VREG_DEFAULT                                ((uint8_t) 0x00)

    /* IOTA1_512_1024K_MSB field */
    #define BIASTRM_VREG_IOTA1_512_1024K_MSB_MASK               ((uint8_t) 0x80)
    #define BIASTRM_VREG_IOTA1_512_1024K_MSB_BITOFFSET          (7)
    #define BIASTRM_VREG_IOTA1_512_1024K_MSB_SAMETABLEASIOTA1_128_256K ((uint8_t) 0x00)    // DEFAULT

    /* TRIM_BIAS field */
    #define BIASTRM_VREG_TRIM_BIAS_MASK                         ((uint8_t) 0x7F)
    #define BIASTRM_VREG_TRIM_BIAS_BITOFFSET                    (0)
    #define BIASTRM_VREG_TRIM_BIAS_ACTIVETRIM                   ((uint8_t) 0x00)    // DEFAULT


/* Register 0x21 (IOTA1_1XS_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |          IOTA1_512_1024K_LSB[1:0]         |                       IOTA1_128_256K[2:0]                       |                        IOTA1_32_64K[2:0]                        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* IOTA1_1XS_VREG register */
    #define IOTA1_1XS_VREG_ADDRESS                              ((uint8_t) 0x21)
    #define IOTA1_1XS_VREG_DEFAULT                              ((uint8_t) 0x00)

    /* IOTA1_512_1024K_LSB field */
    #define IOTA1_1XS_VREG_IOTA1_512_1024K_LSB_MASK             ((uint8_t) 0xC0)
    #define IOTA1_1XS_VREG_IOTA1_512_1024K_LSB_BITOFFSET        (6)
    #define IOTA1_1XS_VREG_IOTA1_512_1024K_LSB_SAMETABLEASIOTA1_128_256K ((uint8_t) 0x00)    // DEFAULT

    /* IOTA1_128_256K field */
    #define IOTA1_1XS_VREG_IOTA1_128_256K_MASK                  ((uint8_t) 0x38)
    #define IOTA1_1XS_VREG_IOTA1_128_256K_BITOFFSET             (3)
    #define IOTA1_1XS_VREG_IOTA1_128_256K_2737                  ((uint8_t) 0x00)    // DEFAULT
    #define IOTA1_1XS_VREG_IOTA1_128_256K_1825OTHERS            ((uint8_t) 0x08)
    #define IOTA1_1XS_VREG_IOTA1_128_256K_912                   ((uint8_t) 0x10)
    #define IOTA1_1XS_VREG_IOTA1_128_256K_0                     ((uint8_t) 0x18)
    #define IOTA1_1XS_VREG_IOTA1_128_256K_912                   ((uint8_t) 0x20)
    #define IOTA1_1XS_VREG_IOTA1_128_256K_1825OTHERS            ((uint8_t) 0x28)
    #define IOTA1_1XS_VREG_IOTA1_128_256K_2737                  ((uint8_t) 0x30)
    #define IOTA1_1XS_VREG_IOTA1_128_256K_3650OTHERS            ((uint8_t) 0x38)

    /* IOTA1_32_64K field */
    #define IOTA1_1XS_VREG_IOTA1_32_64K_MASK                    ((uint8_t) 0x07)
    #define IOTA1_1XS_VREG_IOTA1_32_64K_BITOFFSET               (0)
    #define IOTA1_1XS_VREG_IOTA1_32_64K_2737                    ((uint8_t) 0x00)    // DEFAULT
    #define IOTA1_1XS_VREG_IOTA1_32_64K_1825OTHERS              ((uint8_t) 0x01)
    #define IOTA1_1XS_VREG_IOTA1_32_64K_912                     ((uint8_t) 0x02)
    #define IOTA1_1XS_VREG_IOTA1_32_64K_0                       ((uint8_t) 0x03)
    #define IOTA1_1XS_VREG_IOTA1_32_64K_912                     ((uint8_t) 0x04)
    #define IOTA1_1XS_VREG_IOTA1_32_64K_1825OTHERS              ((uint8_t) 0x05)
    #define IOTA1_1XS_VREG_IOTA1_32_64K_2737                    ((uint8_t) 0x06)
    #define IOTA1_1XS_VREG_IOTA1_32_64K_3650OTHERS              ((uint8_t) 0x07)


/* Register 0x22 (IDAC_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |           IOTA23_512_1024K[1:0]           |                                                           TRIM_IDAC[5:0]                                                          |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* IDAC_VREG register */
    #define IDAC_VREG_ADDRESS                                   ((uint8_t) 0x22)
    #define IDAC_VREG_DEFAULT                                   ((uint8_t) 0x00)

    /* IOTA23_512_1024K field */
    #define IDAC_VREG_IOTA23_512_1024K_MASK                     ((uint8_t) 0xC0)
    #define IDAC_VREG_IOTA23_512_1024K_BITOFFSET                (6)
    #define IDAC_VREG_IOTA23_512_1024K_25                       ((uint8_t) 0x00)    // DEFAULT
    #define IDAC_VREG_IOTA23_512_1024K_0                        ((uint8_t) 0x40)
    #define IDAC_VREG_IOTA23_512_1024K_25                       ((uint8_t) 0x80)
    #define IDAC_VREG_IOTA23_512_1024K_50                       ((uint8_t) 0xC0)

    /* TRIM_IDAC field */
    #define IDAC_VREG_TRIM_IDAC_MASK                            ((uint8_t) 0x3F)
    #define IDAC_VREG_TRIM_IDAC_BITOFFSET                       (0)


/* Register 0x23 (IOTA23_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |             ISUM_128_256K[1:0]            |              ISUM_32_64K[1:0]             |            IOTA23_128_256K[1:0]           |             IOTA23_32_64K[1:0]            |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* IOTA23_VREG register */
    #define IOTA23_VREG_ADDRESS                                 ((uint8_t) 0x23)
    #define IOTA23_VREG_DEFAULT                                 ((uint8_t) 0x00)

    /* ISUM_128_256K field */
    #define IOTA23_VREG_ISUM_128_256K_MASK                      ((uint8_t) 0xC0)
    #define IOTA23_VREG_ISUM_128_256K_BITOFFSET                 (6)
    #define IOTA23_VREG_ISUM_128_256K_25                        ((uint8_t) 0x00)    // DEFAULT
    #define IOTA23_VREG_ISUM_128_256K_0                         ((uint8_t) 0x40)
    #define IOTA23_VREG_ISUM_128_256K_25                        ((uint8_t) 0x80)
    #define IOTA23_VREG_ISUM_128_256K_50                        ((uint8_t) 0xC0)

    /* ISUM_32_64K field */
    #define IOTA23_VREG_ISUM_32_64K_MASK                        ((uint8_t) 0x30)
    #define IOTA23_VREG_ISUM_32_64K_BITOFFSET                   (4)
    #define IOTA23_VREG_ISUM_32_64K_25                          ((uint8_t) 0x00)    // DEFAULT
    #define IOTA23_VREG_ISUM_32_64K_0                           ((uint8_t) 0x10)
    #define IOTA23_VREG_ISUM_32_64K_25                          ((uint8_t) 0x20)
    #define IOTA23_VREG_ISUM_32_64K_50                          ((uint8_t) 0x30)

    /* IOTA23_128_256K field */
    #define IOTA23_VREG_IOTA23_128_256K_MASK                    ((uint8_t) 0x0C)
    #define IOTA23_VREG_IOTA23_128_256K_BITOFFSET               (2)
    #define IOTA23_VREG_IOTA23_128_256K_25                      ((uint8_t) 0x00)    // DEFAULT
    #define IOTA23_VREG_IOTA23_128_256K_0                       ((uint8_t) 0x04)
    #define IOTA23_VREG_IOTA23_128_256K_25                      ((uint8_t) 0x08)
    #define IOTA23_VREG_IOTA23_128_256K_50                      ((uint8_t) 0x0C)

    /* IOTA23_32_64K field */
    #define IOTA23_VREG_IOTA23_32_64K_MASK                      ((uint8_t) 0x03)
    #define IOTA23_VREG_IOTA23_32_64K_BITOFFSET                 (0)
    #define IOTA23_VREG_IOTA23_32_64K_25                        ((uint8_t) 0x00)    // DEFAULT
    #define IOTA23_VREG_IOTA23_32_64K_0                         ((uint8_t) 0x01)
    #define IOTA23_VREG_IOTA23_32_64K_25                        ((uint8_t) 0x02)
    #define IOTA23_VREG_IOTA23_32_64K_50                        ((uint8_t) 0x03)


/* Register 0x24 (ICMP_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |            ICMP_512_1024K[1:0]            |             ICMP_128_256K[1:0]            |              ICMP_32_64K[1:0]             |            ISUM_512_1024K[1:0]            |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* ICMP_VREG register */
    #define ICMP_VREG_ADDRESS                                   ((uint8_t) 0x24)
    #define ICMP_VREG_DEFAULT                                   ((uint8_t) 0x00)

    /* ICMP_512_1024K field */
    #define ICMP_VREG_ICMP_512_1024K_MASK                       ((uint8_t) 0xC0)
    #define ICMP_VREG_ICMP_512_1024K_BITOFFSET                  (6)
    #define ICMP_VREG_ICMP_512_1024K_25                         ((uint8_t) 0x00)    // DEFAULT
    #define ICMP_VREG_ICMP_512_1024K_0                          ((uint8_t) 0x40)
    #define ICMP_VREG_ICMP_512_1024K_25                         ((uint8_t) 0x80)
    #define ICMP_VREG_ICMP_512_1024K_50                         ((uint8_t) 0xC0)

    /* ICMP_128_256K field */
    #define ICMP_VREG_ICMP_128_256K_MASK                        ((uint8_t) 0x30)
    #define ICMP_VREG_ICMP_128_256K_BITOFFSET                   (4)
    #define ICMP_VREG_ICMP_128_256K_25                          ((uint8_t) 0x00)    // DEFAULT
    #define ICMP_VREG_ICMP_128_256K_0                           ((uint8_t) 0x10)
    #define ICMP_VREG_ICMP_128_256K_25                          ((uint8_t) 0x20)
    #define ICMP_VREG_ICMP_128_256K_50                          ((uint8_t) 0x30)

    /* ICMP_32_64K field */
    #define ICMP_VREG_ICMP_32_64K_MASK                          ((uint8_t) 0x0C)
    #define ICMP_VREG_ICMP_32_64K_BITOFFSET                     (2)
    #define ICMP_VREG_ICMP_32_64K_25                            ((uint8_t) 0x00)    // DEFAULT
    #define ICMP_VREG_ICMP_32_64K_0                             ((uint8_t) 0x04)
    #define ICMP_VREG_ICMP_32_64K_25                            ((uint8_t) 0x08)
    #define ICMP_VREG_ICMP_32_64K_50                            ((uint8_t) 0x0C)

    /* ISUM_512_1024K field */
    #define ICMP_VREG_ISUM_512_1024K_MASK                       ((uint8_t) 0x03)
    #define ICMP_VREG_ISUM_512_1024K_BITOFFSET                  (0)
    #define ICMP_VREG_ISUM_512_1024K_25                         ((uint8_t) 0x00)    // DEFAULT
    #define ICMP_VREG_ISUM_512_1024K_0                          ((uint8_t) 0x01)
    #define ICMP_VREG_ISUM_512_1024K_25                         ((uint8_t) 0x02)
    #define ICMP_VREG_ISUM_512_1024K_50                         ((uint8_t) 0x03)


/* Register 0x25 (INCM_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                       INCM2[3:0]                                      |                                       INCM1[3:0]                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* INCM_VREG register */
    #define INCM_VREG_ADDRESS                                   ((uint8_t) 0x25)
    #define INCM_VREG_DEFAULT                                   ((uint8_t) 0x00)

    /* INCM2 field */
    #define INCM_VREG_INCM2_MASK                                ((uint8_t) 0xF0)
    #define INCM_VREG_INCM2_BITOFFSET                           (4)

    /* INCM1 field */
    #define INCM_VREG_INCM1_MASK                                ((uint8_t) 0x0F)
    #define INCM_VREG_INCM1_BITOFFSET                           (0)
    #define INCM_VREG_INCM1_64MV                                ((uint8_t) 0x00)    // DEFAULT
    #define INCM_VREG_INCM1_48                                  ((uint8_t) 0x01)
    #define INCM_VREG_INCM1_32                                  ((uint8_t) 0x02)
    #define INCM_VREG_INCM1_16                                  ((uint8_t) 0x03)
    #define INCM_VREG_INCM1_0MV                                 ((uint8_t) 0x04)
    #define INCM_VREG_INCM1_16                                  ((uint8_t) 0x05)
    #define INCM_VREG_INCM1_32                                  ((uint8_t) 0x06)
    #define INCM_VREG_INCM1_48                                  ((uint8_t) 0x07)
    #define INCM_VREG_INCM1_64                                  ((uint8_t) 0x08)
    #define INCM_VREG_INCM1_80                                  ((uint8_t) 0x09)
    #define INCM_VREG_INCM1_96                                  ((uint8_t) 0x0A)
    #define INCM_VREG_INCM1_112                                 ((uint8_t) 0x0B)
    #define INCM_VREG_INCM1_128                                 ((uint8_t) 0x0C)
    #define INCM_VREG_INCM1_144                                 ((uint8_t) 0x0D)
    #define INCM_VREG_INCM1_161                                 ((uint8_t) 0x0E)
    #define INCM_VREG_INCM1_177                                 ((uint8_t) 0x0F)


/* Register 0x26 (OCM_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                       OCM2[3:0]                                       |                                       OCM1[3:0]                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OCM_VREG register */
    #define OCM_VREG_ADDRESS                                    ((uint8_t) 0x26)
    #define OCM_VREG_DEFAULT                                    ((uint8_t) 0x00)

    /* OCM2 field */
    #define OCM_VREG_OCM2_MASK                                  ((uint8_t) 0xF0)
    #define OCM_VREG_OCM2_BITOFFSET                             (4)
    #define OCM_VREG_OCM2_112MV                                 ((uint8_t) 0x00)    // DEFAULT
    #define OCM_VREG_OCM2_96MV                                  ((uint8_t) 0x10)
    #define OCM_VREG_OCM2_80MV                                  ((uint8_t) 0x20)
    #define OCM_VREG_OCM2_63MV                                  ((uint8_t) 0x30)
    #define OCM_VREG_OCM2_48MV                                  ((uint8_t) 0x40)
    #define OCM_VREG_OCM2_32MV                                  ((uint8_t) 0x50)
    #define OCM_VREG_OCM2_16MV                                  ((uint8_t) 0x60)
    #define OCM_VREG_OCM2_0MV                                   ((uint8_t) 0x70)
    #define OCM_VREG_OCM2_16MV                                  ((uint8_t) 0x80)
    #define OCM_VREG_OCM2_32MV                                  ((uint8_t) 0x90)
    #define OCM_VREG_OCM2_48MV                                  ((uint8_t) 0xA0)
    #define OCM_VREG_OCM2_64MV                                  ((uint8_t) 0xB0)
    #define OCM_VREG_OCM2_80MV                                  ((uint8_t) 0xC0)
    #define OCM_VREG_OCM2_96MV                                  ((uint8_t) 0xD0)
    #define OCM_VREG_OCM2_112MV                                 ((uint8_t) 0xE0)
    #define OCM_VREG_OCM2_128MV                                 ((uint8_t) 0xF0)

    /* OCM1 field */
    #define OCM_VREG_OCM1_MASK                                  ((uint8_t) 0x0F)
    #define OCM_VREG_OCM1_BITOFFSET                             (0)
    #define OCM_VREG_OCM1_128MV                                 ((uint8_t) 0x00)    // DEFAULT
    #define OCM_VREG_OCM1_112MV                                 ((uint8_t) 0x01)
    #define OCM_VREG_OCM1_96MV                                  ((uint8_t) 0x02)
    #define OCM_VREG_OCM1_80MV                                  ((uint8_t) 0x03)
    #define OCM_VREG_OCM1_75MV                                  ((uint8_t) 0x04)
    #define OCM_VREG_OCM1_48MV                                  ((uint8_t) 0x05)
    #define OCM_VREG_OCM1_32MV                                  ((uint8_t) 0x06)
    #define OCM_VREG_OCM1_16MV                                  ((uint8_t) 0x07)
    #define OCM_VREG_OCM1_0MV                                   ((uint8_t) 0x08)
    #define OCM_VREG_OCM1_16MV                                  ((uint8_t) 0x09)
    #define OCM_VREG_OCM1_32MV                                  ((uint8_t) 0x0A)
    #define OCM_VREG_OCM1_48MV                                  ((uint8_t) 0x0B)
    #define OCM_VREG_OCM1_64MV                                  ((uint8_t) 0x0C)
    #define OCM_VREG_OCM1_80MV                                  ((uint8_t) 0x0D)
    #define OCM_VREG_OCM1_96MV                                  ((uint8_t) 0x0E)
    #define OCM_VREG_OCM1_112MV                                 ((uint8_t) 0x0F)


/* Register 0x27 (IPGA_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |            IPGA_1750NA_LSB[1:0]           |                         IPGA_500NA[2:0]                         |                         IPGA_100NA[2:0]                         |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* IPGA_VREG register */
    #define IPGA_VREG_ADDRESS                                   ((uint8_t) 0x27)
    #define IPGA_VREG_DEFAULT                                   ((uint8_t) 0x00)

    /* IPGA_1750NA_LSB field */
    #define IPGA_VREG_IPGA_1750NA_LSB_MASK                      ((uint8_t) 0xC0)
    #define IPGA_VREG_IPGA_1750NA_LSB_BITOFFSET                 (6)

    /* IPGA_500NA field */
    #define IPGA_VREG_IPGA_500NA_MASK                           ((uint8_t) 0x38)
    #define IPGA_VREG_IPGA_500NA_BITOFFSET                      (3)
    #define IPGA_VREG_IPGA_500NA_37                             ((uint8_t) 0x00)    // DEFAULT
    #define IPGA_VREG_IPGA_500NA_25                             ((uint8_t) 0x08)
    #define IPGA_VREG_IPGA_500NA_12                             ((uint8_t) 0x10)
    #define IPGA_VREG_IPGA_500NA_0                              ((uint8_t) 0x18)
    #define IPGA_VREG_IPGA_500NA_12                             ((uint8_t) 0x20)
    #define IPGA_VREG_IPGA_500NA_25                             ((uint8_t) 0x28)
    #define IPGA_VREG_IPGA_500NA_37                             ((uint8_t) 0x30)
    #define IPGA_VREG_IPGA_500NA_50                             ((uint8_t) 0x38)

    /* IPGA_100NA field */
    #define IPGA_VREG_IPGA_100NA_MASK                           ((uint8_t) 0x07)
    #define IPGA_VREG_IPGA_100NA_BITOFFSET                      (0)
    #define IPGA_VREG_IPGA_100NA_37                             ((uint8_t) 0x00)    // DEFAULT
    #define IPGA_VREG_IPGA_100NA_25                             ((uint8_t) 0x01)
    #define IPGA_VREG_IPGA_100NA_12                             ((uint8_t) 0x02)
    #define IPGA_VREG_IPGA_100NA_0                              ((uint8_t) 0x03)
    #define IPGA_VREG_IPGA_100NA_12                             ((uint8_t) 0x04)
    #define IPGA_VREG_IPGA_100NA_25                             ((uint8_t) 0x05)
    #define IPGA_VREG_IPGA_100NA_37                             ((uint8_t) 0x06)
    #define IPGA_VREG_IPGA_100NA_50                             ((uint8_t) 0x07)


/* Register 0x28 (PGACHOP_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |           PGAPREQ_CHOP_LSB[1:0]           |                     PGA_CHOP_256_1024K[2:0]                     |                      PGA_CHOP_32_128K[2:0]                      |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PGACHOP_VREG register */
    #define PGACHOP_VREG_ADDRESS                                ((uint8_t) 0x28)
    #define PGACHOP_VREG_DEFAULT                                ((uint8_t) 0x00)

    /* PGAPREQ_CHOP_LSB field */
    #define PGACHOP_VREG_PGAPREQ_CHOP_LSB_MASK                  ((uint8_t) 0xC0)
    #define PGACHOP_VREG_PGAPREQ_CHOP_LSB_BITOFFSET             (6)

    /* PGA_CHOP_256_1024K field */
    #define PGACHOP_VREG_PGA_CHOP_256_1024K_MASK                ((uint8_t) 0x38)
    #define PGACHOP_VREG_PGA_CHOP_256_1024K_BITOFFSET           (3)

    /* PGA_CHOP_32_128K field */
    #define PGACHOP_VREG_PGA_CHOP_32_128K_MASK                  ((uint8_t) 0x07)
    #define PGACHOP_VREG_PGA_CHOP_32_128K_BITOFFSET             (0)


/* Register 0x29 (PREQCHOP_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        IPGAPREQ_1XS_512_1024K[1:0]        |              LCAP_PH_SEL[1:0]             |                         REFBUF_CHOP[2:0]                        |   PGAPREQ_CHOP_MSB  |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PREQCHOP_VREG register */
    #define PREQCHOP_VREG_ADDRESS                               ((uint8_t) 0x29)
    #define PREQCHOP_VREG_DEFAULT                               ((uint8_t) 0x00)

    /* IPGAPREQ_1XS_512_1024K field */
    #define PREQCHOP_VREG_IPGAPREQ_1XS_512_1024K_MASK           ((uint8_t) 0xC0)
    #define PREQCHOP_VREG_IPGAPREQ_1XS_512_1024K_BITOFFSET      (6)
    #define PREQCHOP_VREG_IPGAPREQ_1XS_512_1024K_25             ((uint8_t) 0x00)    // DEFAULT
    #define PREQCHOP_VREG_IPGAPREQ_1XS_512_1024K_0              ((uint8_t) 0x40)
    #define PREQCHOP_VREG_IPGAPREQ_1XS_512_1024K_25             ((uint8_t) 0x80)
    #define PREQCHOP_VREG_IPGAPREQ_1XS_512_1024K_50             ((uint8_t) 0xC0)

    /* LCAP_PH_SEL field */
    #define PREQCHOP_VREG_LCAP_PH_SEL_MASK                      ((uint8_t) 0x30)
    #define PREQCHOP_VREG_LCAP_PH_SEL_BITOFFSET                 (4)
    #define PREQCHOP_VREG_LCAP_PH_SEL_NEVERACTIVE               ((uint8_t) 0x00)    // DEFAULT
    #define PREQCHOP_VREG_LCAP_PH_SEL_ALWAYSACTIVE              ((uint8_t) 0x10)
    #define PREQCHOP_VREG_LCAP_PH_SEL_ACTIVEALLPH1              ((uint8_t) 0x20)
    #define PREQCHOP_VREG_LCAP_PH_SEL_ACTIVEPH1CC               ((uint8_t) 0x30)

    /* REFBUF_CHOP field */
    #define PREQCHOP_VREG_REFBUF_CHOP_MASK                      ((uint8_t) 0x0E)
    #define PREQCHOP_VREG_REFBUF_CHOP_BITOFFSET                 (1)

    /* PGAPREQ_CHOP_MSB field */
    #define PREQCHOP_VREG_PGAPREQ_CHOP_MSB_MASK                 ((uint8_t) 0x01)
    #define PREQCHOP_VREG_PGAPREQ_CHOP_MSB_BITOFFSET            (0)


/* Register 0x2A (IPGAPREQ_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        IPGAPREQ_2XS_512_1024K[1:0]        |         IPGAPREQ_2XS_128_256K[1:0]        |         IPGAPREQ_1XS_128_256K[1:0]        |          IPGAPREQ_1XS_32_64K[1:0]         |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* IPGAPREQ_VREG register */
    #define IPGAPREQ_VREG_ADDRESS                               ((uint8_t) 0x2A)
    #define IPGAPREQ_VREG_DEFAULT                               ((uint8_t) 0x00)

    /* IPGAPREQ_2XS_512_1024K field */
    #define IPGAPREQ_VREG_IPGAPREQ_2XS_512_1024K_MASK           ((uint8_t) 0xC0)
    #define IPGAPREQ_VREG_IPGAPREQ_2XS_512_1024K_BITOFFSET      (6)
    #define IPGAPREQ_VREG_IPGAPREQ_2XS_512_1024K_25             ((uint8_t) 0x00)    // DEFAULT
    #define IPGAPREQ_VREG_IPGAPREQ_2XS_512_1024K_0              ((uint8_t) 0x40)
    #define IPGAPREQ_VREG_IPGAPREQ_2XS_512_1024K_25             ((uint8_t) 0x80)
    #define IPGAPREQ_VREG_IPGAPREQ_2XS_512_1024K_50             ((uint8_t) 0xC0)

    /* IPGAPREQ_2XS_128_256K field */
    #define IPGAPREQ_VREG_IPGAPREQ_2XS_128_256K_MASK            ((uint8_t) 0x30)
    #define IPGAPREQ_VREG_IPGAPREQ_2XS_128_256K_BITOFFSET       (4)
    #define IPGAPREQ_VREG_IPGAPREQ_2XS_128_256K_25              ((uint8_t) 0x00)    // DEFAULT
    #define IPGAPREQ_VREG_IPGAPREQ_2XS_128_256K_0               ((uint8_t) 0x10)
    #define IPGAPREQ_VREG_IPGAPREQ_2XS_128_256K_25              ((uint8_t) 0x20)
    #define IPGAPREQ_VREG_IPGAPREQ_2XS_128_256K_50              ((uint8_t) 0x30)

    /* IPGAPREQ_1XS_128_256K field */
    #define IPGAPREQ_VREG_IPGAPREQ_1XS_128_256K_MASK            ((uint8_t) 0x0C)
    #define IPGAPREQ_VREG_IPGAPREQ_1XS_128_256K_BITOFFSET       (2)
    #define IPGAPREQ_VREG_IPGAPREQ_1XS_128_256K_25              ((uint8_t) 0x00)    // DEFAULT
    #define IPGAPREQ_VREG_IPGAPREQ_1XS_128_256K_0               ((uint8_t) 0x04)
    #define IPGAPREQ_VREG_IPGAPREQ_1XS_128_256K_25              ((uint8_t) 0x08)
    #define IPGAPREQ_VREG_IPGAPREQ_1XS_128_256K_50              ((uint8_t) 0x0C)

    /* IPGAPREQ_1XS_32_64K field */
    #define IPGAPREQ_VREG_IPGAPREQ_1XS_32_64K_MASK              ((uint8_t) 0x03)
    #define IPGAPREQ_VREG_IPGAPREQ_1XS_32_64K_BITOFFSET         (0)
    #define IPGAPREQ_VREG_IPGAPREQ_1XS_32_64K_25                ((uint8_t) 0x00)    // DEFAULT
    #define IPGAPREQ_VREG_IPGAPREQ_1XS_32_64K_0                 ((uint8_t) 0x01)
    #define IPGAPREQ_VREG_IPGAPREQ_1XS_32_64K_25                ((uint8_t) 0x02)
    #define IPGAPREQ_VREG_IPGAPREQ_1XS_32_64K_50                ((uint8_t) 0x03)


/* Register 0x2B (IREFBUF_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |           IREFBUF_512_1024K[1:0]          |           IREFBUF_128_256K[1:0]           |            IREFBUF_32_64K[1:0]            |          IPGAPREQ_2XS_32_64K[1:0]         |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* IREFBUF_VREG register */
    #define IREFBUF_VREG_ADDRESS                                ((uint8_t) 0x2B)
    #define IREFBUF_VREG_DEFAULT                                ((uint8_t) 0x00)

    /* IREFBUF_512_1024K field */
    #define IREFBUF_VREG_IREFBUF_512_1024K_MASK                 ((uint8_t) 0xC0)
    #define IREFBUF_VREG_IREFBUF_512_1024K_BITOFFSET            (6)
    #define IREFBUF_VREG_IREFBUF_512_1024K_25                   ((uint8_t) 0x00)    // DEFAULT
    #define IREFBUF_VREG_IREFBUF_512_1024K_0                    ((uint8_t) 0x40)
    #define IREFBUF_VREG_IREFBUF_512_1024K_25                   ((uint8_t) 0x80)
    #define IREFBUF_VREG_IREFBUF_512_1024K_50                   ((uint8_t) 0xC0)

    /* IREFBUF_128_256K field */
    #define IREFBUF_VREG_IREFBUF_128_256K_MASK                  ((uint8_t) 0x30)
    #define IREFBUF_VREG_IREFBUF_128_256K_BITOFFSET             (4)
    #define IREFBUF_VREG_IREFBUF_128_256K_25                    ((uint8_t) 0x00)    // DEFAULT
    #define IREFBUF_VREG_IREFBUF_128_256K_0                     ((uint8_t) 0x10)
    #define IREFBUF_VREG_IREFBUF_128_256K_25                    ((uint8_t) 0x20)
    #define IREFBUF_VREG_IREFBUF_128_256K_50                    ((uint8_t) 0x30)

    /* IREFBUF_32_64K field */
    #define IREFBUF_VREG_IREFBUF_32_64K_MASK                    ((uint8_t) 0x0C)
    #define IREFBUF_VREG_IREFBUF_32_64K_BITOFFSET               (2)
    #define IREFBUF_VREG_IREFBUF_32_64K_25                      ((uint8_t) 0x00)    // DEFAULT
    #define IREFBUF_VREG_IREFBUF_32_64K_0                       ((uint8_t) 0x04)
    #define IREFBUF_VREG_IREFBUF_32_64K_25                      ((uint8_t) 0x08)
    #define IREFBUF_VREG_IREFBUF_32_64K_50                      ((uint8_t) 0x0C)

    /* IPGAPREQ_2XS_32_64K field */
    #define IREFBUF_VREG_IPGAPREQ_2XS_32_64K_MASK               ((uint8_t) 0x03)
    #define IREFBUF_VREG_IPGAPREQ_2XS_32_64K_BITOFFSET          (0)
    #define IREFBUF_VREG_IPGAPREQ_2XS_32_64K_25                 ((uint8_t) 0x00)    // DEFAULT
    #define IREFBUF_VREG_IPGAPREQ_2XS_32_64K_0                  ((uint8_t) 0x01)
    #define IREFBUF_VREG_IPGAPREQ_2XS_32_64K_25                 ((uint8_t) 0x02)
    #define IREFBUF_VREG_IPGAPREQ_2XS_32_64K_50                 ((uint8_t) 0x03)


/* Register 0x2C (CHOP1_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                     INT1_CHOP[3:0]                                    |                                      IN_CHOP[3:0]                                     |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CHOP1_VREG register */
    #define CHOP1_VREG_ADDRESS                                  ((uint8_t) 0x2C)
    #define CHOP1_VREG_DEFAULT                                  ((uint8_t) 0x00)

    /* INT1_CHOP field */
    #define CHOP1_VREG_INT1_CHOP_MASK                           ((uint8_t) 0xF0)
    #define CHOP1_VREG_INT1_CHOP_BITOFFSET                      (4)

    /* IN_CHOP field */
    #define CHOP1_VREG_IN_CHOP_MASK                             ((uint8_t) 0x0F)
    #define CHOP1_VREG_IN_CHOP_BITOFFSET                        (0)


/* Register 0x2D (QPUMP_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                TRIM_LDOA[4:0]                                               |     QCLK2_DELAY     |      QCLK2_STBY     |      QCLK2_DIV2     |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* QPUMP_VREG register */
    #define QPUMP_VREG_ADDRESS                                  ((uint8_t) 0x2D)
    #define QPUMP_VREG_DEFAULT                                  ((uint8_t) 0x00)

    /* TRIM_LDOA field */
    #define QPUMP_VREG_TRIM_LDOA_MASK                           ((uint8_t) 0xF8)
    #define QPUMP_VREG_TRIM_LDOA_BITOFFSET                      (3)
    #define QPUMP_VREG_TRIM_LDOA_ACTIVETRIM                     ((uint8_t) 0x00)    // DEFAULT

    /* QCLK2_DELAY field */
    #define QPUMP_VREG_QCLK2_DELAY_MASK                         ((uint8_t) 0x04)
    #define QPUMP_VREG_QCLK2_DELAY_BITOFFSET                    (2)

    /* QCLK2_STBY field */
    #define QPUMP_VREG_QCLK2_STBY_MASK                          ((uint8_t) 0x02)
    #define QPUMP_VREG_QCLK2_STBY_BITOFFSET                     (1)

    /* QCLK2_DIV2 field */
    #define QPUMP_VREG_QCLK2_DIV2_MASK                          ((uint8_t) 0x01)
    #define QPUMP_VREG_QCLK2_DIV2_BITOFFSET                     (0)


/* Register 0x2E (TIME_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     CHOP_OTA1_PH    |                CLK_DLY[1:0]               |      MODCLK_TN3     |      MODCLK_TN2     |      MODCLK_TN1     |    REFCMSLEW_DIS    |       DACDEL1       |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TIME_VREG register */
    #define TIME_VREG_ADDRESS                                   ((uint8_t) 0x2E)
    #define TIME_VREG_DEFAULT                                   ((uint8_t) 0x00)

    /* CHOP_OTA1_PH field */
    #define TIME_VREG_CHOP_OTA1_PH_MASK                         ((uint8_t) 0x80)
    #define TIME_VREG_CHOP_OTA1_PH_BITOFFSET                    (7)

    /* CLK_DLY field */
    #define TIME_VREG_CLK_DLY_MASK                              ((uint8_t) 0x60)
    #define TIME_VREG_CLK_DLY_BITOFFSET                         (5)
    #define TIME_VREG_CLK_DLY_0NSNORMALLY0NSIFSPEED_DIG101      ((uint8_t) 0x00)    // DEFAULT
    #define TIME_VREG_CLK_DLY_8NSNORMALLY4NSIFSPEED_DIG101      ((uint8_t) 0x20)
    #define TIME_VREG_CLK_DLY_16NSNORMALLY8NSIFSPEED_DIG101     ((uint8_t) 0x40)
    #define TIME_VREG_CLK_DLY_24NSNORMALLY12NSIFSPEED_DIG101    ((uint8_t) 0x60)

    /* MODCLK_TN3 field */
    #define TIME_VREG_MODCLK_TN3_MASK                           ((uint8_t) 0x10)
    #define TIME_VREG_MODCLK_TN3_BITOFFSET                      (4)
    #define TIME_VREG_MODCLK_TN3_NOEXTRADELAY                   ((uint8_t) 0x00)    // DEFAULT
    #define TIME_VREG_MODCLK_TN3_ADD0                           ((uint8_t) 0x10)

    /* MODCLK_TN2 field */
    #define TIME_VREG_MODCLK_TN2_MASK                           ((uint8_t) 0x08)
    #define TIME_VREG_MODCLK_TN2_BITOFFSET                      (3)
    #define TIME_VREG_MODCLK_TN2_1NSFORPH1PH22                  ((uint8_t) 0x00)    // DEFAULT
    #define TIME_VREG_MODCLK_TN2_ADD0                           ((uint8_t) 0x08)

    /* MODCLK_TN1 field */
    #define TIME_VREG_MODCLK_TN1_MASK                           ((uint8_t) 0x04)
    #define TIME_VREG_MODCLK_TN1_BITOFFSET                      (2)
    #define TIME_VREG_MODCLK_TN1_0                              ((uint8_t) 0x00)    // DEFAULT
    #define TIME_VREG_MODCLK_TN1_0                              ((uint8_t) 0x04)

    /* REFCMSLEW_DIS field */
    #define TIME_VREG_REFCMSLEW_DIS_MASK                        ((uint8_t) 0x02)
    #define TIME_VREG_REFCMSLEW_DIS_BITOFFSET                   (1)

    /* DACDEL1 field */
    #define TIME_VREG_DACDEL1_MASK                              ((uint8_t) 0x01)
    #define TIME_VREG_DACDEL1_BITOFFSET                         (0)
    #define TIME_VREG_DACDEL1_PH1DD                             ((uint8_t) 0x00)    // DEFAULT
    #define TIME_VREG_DACDEL1_PH1D                              ((uint8_t) 0x01)


/* Register 0x2F (DELAY_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |            SETTLE_32_128K[1:0]            |          STBY_DLY_512_1024K[1:0]          |             STBY_DLY_256K[1:0]            |           STBY_DLY_32_128K[1:0]           |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DELAY_VREG register */
    #define DELAY_VREG_ADDRESS                                  ((uint8_t) 0x2F)
    #define DELAY_VREG_DEFAULT                                  ((uint8_t) 0x00)

    /* SETTLE_32_128K field */
    #define DELAY_VREG_SETTLE_32_128K_MASK                      ((uint8_t) 0xC0)
    #define DELAY_VREG_SETTLE_32_128K_BITOFFSET                 (6)

    /* STBY_DLY_512_1024K field */
    #define DELAY_VREG_STBY_DLY_512_1024K_MASK                  ((uint8_t) 0x30)
    #define DELAY_VREG_STBY_DLY_512_1024K_BITOFFSET             (4)

    /* STBY_DLY_256K field */
    #define DELAY_VREG_STBY_DLY_256K_MASK                       ((uint8_t) 0x0C)
    #define DELAY_VREG_STBY_DLY_256K_BITOFFSET                  (2)

    /* STBY_DLY_32_128K field */
    #define DELAY_VREG_STBY_DLY_32_128K_MASK                    ((uint8_t) 0x03)
    #define DELAY_VREG_STBY_DLY_32_128K_BITOFFSET               (0)


/* Register 0x30 (PH2DEL_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |           SETTLE_512_1024K[1:0]           |              SETTLE_256K[1:0]             |     LIN_BIAS_DIS    |                         PHASE2_DEL[2:0]                         |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PH2DEL_VREG register */
    #define PH2DEL_VREG_ADDRESS                                 ((uint8_t) 0x30)
    #define PH2DEL_VREG_DEFAULT                                 ((uint8_t) 0x00)

    /* SETTLE_512_1024K field */
    #define PH2DEL_VREG_SETTLE_512_1024K_MASK                   ((uint8_t) 0xC0)
    #define PH2DEL_VREG_SETTLE_512_1024K_BITOFFSET              (6)

    /* SETTLE_256K field */
    #define PH2DEL_VREG_SETTLE_256K_MASK                        ((uint8_t) 0x30)
    #define PH2DEL_VREG_SETTLE_256K_BITOFFSET                   (4)

    /* LIN_BIAS_DIS field */
    #define PH2DEL_VREG_LIN_BIAS_DIS_MASK                       ((uint8_t) 0x08)
    #define PH2DEL_VREG_LIN_BIAS_DIS_BITOFFSET                  (3)

    /* PHASE2_DEL field */
    #define PH2DEL_VREG_PHASE2_DEL_MASK                         ((uint8_t) 0x07)
    #define PH2DEL_VREG_PHASE2_DEL_BITOFFSET                    (0)


/* Register 0x31 (TSGAIN_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                  TSGAIN[7:0]                                                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TSGAIN_VREG register */
    #define TSGAIN_VREG_ADDRESS                                 ((uint8_t) 0x31)
    #define TSGAIN_VREG_DEFAULT                                 ((uint8_t) 0x00)

    /* TSGAIN field */
    #define TSGAIN_VREG_TSGAIN_MASK                             ((uint8_t) 0xFF)
    #define TSGAIN_VREG_TSGAIN_BITOFFSET                        (0)


/* Register 0x32 (ISOSENSE1_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                               ISOSENSE_CFG1[7:0]                                                                              |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* ISOSENSE1_VREG register */
    #define ISOSENSE1_VREG_ADDRESS                              ((uint8_t) 0x32)
    #define ISOSENSE1_VREG_DEFAULT                              ((uint8_t) 0x00)

    /* ISOSENSE_CFG1 field */
    #define ISOSENSE1_VREG_ISOSENSE_CFG1_MASK                   ((uint8_t) 0xFF)
    #define ISOSENSE1_VREG_ISOSENSE_CFG1_BITOFFSET              (0)


/* Register 0x33 (ISOSENSE2_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                               ISOSENSE_CFG2[7:0]                                                                              |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* ISOSENSE2_VREG register */
    #define ISOSENSE2_VREG_ADDRESS                              ((uint8_t) 0x33)
    #define ISOSENSE2_VREG_DEFAULT                              ((uint8_t) 0x00)

    /* ISOSENSE_CFG2 field */
    #define ISOSENSE2_VREG_ISOSENSE_CFG2_MASK                   ((uint8_t) 0xFF)
    #define ISOSENSE2_VREG_ISOSENSE_CFG2_BITOFFSET              (0)


/* Register 0x34 (I2CCFG_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |      PGA_OCP_EN     |     I2C_ADDR_MSB    |                         IO_MODE_1V2[2:0]                        |                          I2C_MODE[2:0]                          |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* I2CCFG_VREG register */
    #define I2CCFG_VREG_ADDRESS                                 ((uint8_t) 0x34)
    #define I2CCFG_VREG_DEFAULT                                 ((uint8_t) 0x00)

    /* PGA_OCP_EN field */
    #define I2CCFG_VREG_PGA_OCP_EN_MASK                         ((uint8_t) 0x80)
    #define I2CCFG_VREG_PGA_OCP_EN_BITOFFSET                    (7)

    /* I2C_ADDR_MSB field */
    #define I2CCFG_VREG_I2C_ADDR_MSB_MASK                       ((uint8_t) 0x40)
    #define I2CCFG_VREG_I2C_ADDR_MSB_BITOFFSET                  (6)

    /* IO_MODE_1V2 field */
    #define I2CCFG_VREG_IO_MODE_1V2_MASK                        ((uint8_t) 0x38)
    #define I2CCFG_VREG_IO_MODE_1V2_BITOFFSET                   (3)

    /* I2C_MODE field */
    #define I2CCFG_VREG_I2C_MODE_MASK                           ((uint8_t) 0x07)
    #define I2CCFG_VREG_I2C_MODE_BITOFFSET                      (0)


/* Register 0x35 (OTPCRC_VREG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                  OTP_CRC[7:0]                                                                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OTPCRC_VREG register */
    #define OTPCRC_VREG_ADDRESS                                 ((uint8_t) 0x35)
    #define OTPCRC_VREG_DEFAULT                                 ((uint8_t) 0x00)

    /* OTP_CRC field */
    #define OTPCRC_VREG_OTP_CRC_MASK                            ((uint8_t) 0xFF)
    #define OTPCRC_VREG_OTP_CRC_BITOFFSET                       (0)


/* Register 0x3F (PAGE_POINTER) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                               PAGE_POINTER[7:0]                                                                               |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PAGE_POINTER register */
    #define PAGE_POINTER_ADDRESS                                ((uint8_t) 0x3F)
    #define PAGE_POINTER_DEFAULT                                ((uint8_t) 0x00)

    /* PAGE_POINTER field */
    #define PAGE_POINTER_MASK                                   ((uint8_t) 0xFF)
    #define PAGE_POINTER_BITOFFSET                              (0)


#endif /* ADS122S1X_PAGE3_H_ */
