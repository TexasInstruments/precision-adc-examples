/*
 * @file ads122s1x_page1.h
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

#ifndef ADS122S1X_PAGE1_H_
#define ADS122S1X_PAGE1_H_

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


/* Register 0x00 (ATB_TREG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |      RESERVED      |                           ATB1[2:0]                          |                                     ATB0[3:0]                                     |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* ATB_TREG register */
    #define ATB_TREG_ADDRESS                                    ((uint8_t) 0x00)
    #define ATB_TREG_DEFAULT                                    ((uint8_t) 0x00)

    /* ATB1 field */
    #define ATB_TREG_ATB1_MASK                                  ((uint8_t) 0x70)
    #define ATB_TREG_ATB1_BITOFFSET                             (4)

    /* ATB0 field */
    #define ATB_TREG_ATB0_MASK                                  ((uint8_t) 0x0F)
    #define ATB_TREG_ATB0_BITOFFSET                             (0)


/* Register 0x01 (DTB_TREG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                   RESERVED[3:0]                                   |                                      DTB[3:0]                                     |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DTB_TREG register */
    #define DTB_TREG_ADDRESS                                    ((uint8_t) 0x01)
    #define DTB_TREG_DEFAULT                                    ((uint8_t) 0x00)

    /* DTB field */
    #define DTB_TREG_DTB_MASK                                   ((uint8_t) 0x0F)
    #define DTB_TREG_DTB_BITOFFSET                              (0)


/* Register 0x02 (DFT1_TREG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |      TEMPSENSE     |    MOD_IN_SHORT    |       PGA_DIS      |     TEST_REFVSS    |        HTEN        |      RESERVED      |      NCP_OVST      |       VBOXLO       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DFT1_TREG register */
    #define DFT1_TREG_ADDRESS                                   ((uint8_t) 0x02)
    #define DFT1_TREG_DEFAULT                                   ((uint8_t) 0x00)

    /* TEMPSENSE field */
    #define DFT1_TREG_TEMPSENSE_MASK                            ((uint8_t) 0x80)
    #define DFT1_TREG_TEMPSENSE_BITOFFSET                       (7)
    #define DFT1_TREG_TEMPSENSE_TMP461DISCONNECTED              ((uint8_t) 0x00)    // DEFAULT
    #define DFT1_TREG_TEMPSENSE_ENABLESTHETMP461                ((uint8_t) 0x80)

    /* MOD_IN_SHORT field */
    #define DFT1_TREG_MOD_IN_SHORT_MASK                         ((uint8_t) 0x40)
    #define DFT1_TREG_MOD_IN_SHORT_BITOFFSET                    (6)
    #define DFT1_TREG_MOD_IN_SHORT_NOCHANGE                     ((uint8_t) 0x00)    // DEFAULT
    #define DFT1_TREG_MOD_IN_SHORT_MOD_IN_SHORTENABLED          ((uint8_t) 0x40)

    /* PGA_DIS field */
    #define DFT1_TREG_PGA_DIS_MASK                              ((uint8_t) 0x20)
    #define DFT1_TREG_PGA_DIS_BITOFFSET                         (5)
    #define DFT1_TREG_PGA_DIS_NORMALOPERATION                   ((uint8_t) 0x00)    // DEFAULT
    #define DFT1_TREG_PGA_DIS_PGAOFF                            ((uint8_t) 0x20)

    /* TEST_REFVSS field */
    #define DFT1_TREG_TEST_REFVSS_MASK                          ((uint8_t) 0x10)
    #define DFT1_TREG_TEST_REFVSS_BITOFFSET                     (4)
    #define DFT1_TREG_TEST_REFVSS_NOCHANGE                      ((uint8_t) 0x00)    // DEFAULT
    #define DFT1_TREG_TEST_REFVSS_TEST_REFVSSENABLED            ((uint8_t) 0x10)

    /* HTEN field */
    #define DFT1_TREG_HTEN_MASK                                 ((uint8_t) 0x08)
    #define DFT1_TREG_HTEN_BITOFFSET                            (3)
    #define DFT1_TREG_HTEN_NOCHANGE                             ((uint8_t) 0x00)    // DEFAULT
    #define DFT1_TREG_HTEN_HTEN1B1                              ((uint8_t) 0x08)

    /* NCP_OVST field */
    #define DFT1_TREG_NCP_OVST_MASK                             ((uint8_t) 0x02)
    #define DFT1_TREG_NCP_OVST_BITOFFSET                        (1)
    #define DFT1_TREG_NCP_OVST_NOCHANGE                         ((uint8_t) 0x00)    // DEFAULT
    #define DFT1_TREG_NCP_OVST_NCP_OVST_EN1B1                   ((uint8_t) 0x02)

    /* VBOXLO field */
    #define DFT1_TREG_VBOXLO_MASK                               ((uint8_t) 0x01)
    #define DFT1_TREG_VBOXLO_BITOFFSET                          (0)
    #define DFT1_TREG_VBOXLO_NOCHANGE                           ((uint8_t) 0x00)    // DEFAULT
    #define DFT1_TREG_VBOXLO_PORZ_VBOXL1B1                      ((uint8_t) 0x01)


/* Register 0x03 (DFT2_TREG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |              RESERVED[1:0]              |                 DEM[1:0]                |    NCP1_OVERCLK    |      QPUMP_DIS     |      LDOA_DIS      |      RESERVED      |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DFT2_TREG register */
    #define DFT2_TREG_ADDRESS                                   ((uint8_t) 0x03)
    #define DFT2_TREG_DEFAULT                                   ((uint8_t) 0x00)

    /* DEM field */
    #define DFT2_TREG_DEM_MASK                                  ((uint8_t) 0x30)
    #define DFT2_TREG_DEM_BITOFFSET                             (4)
    #define DFT2_TREG_DEM_212DWA                                ((uint8_t) 0x00)    // DEFAULT
    #define DFT2_TREG_DEM_210DWA                                ((uint8_t) 0x10)
    #define DFT2_TREG_DEM_OFF                                   ((uint8_t) 0x20)
    #define DFT2_TREG_DEM_OFF                                   ((uint8_t) 0x30)

    /* NCP1_OVERCLK field */
    #define DFT2_TREG_NCP1_OVERCLK_MASK                         ((uint8_t) 0x08)
    #define DFT2_TREG_NCP1_OVERCLK_BITOFFSET                    (3)

    /* QPUMP_DIS field */
    #define DFT2_TREG_QPUMP_DIS_MASK                            ((uint8_t) 0x04)
    #define DFT2_TREG_QPUMP_DIS_BITOFFSET                       (2)
    #define DFT2_TREG_QPUMP_DIS_NOCHANGE                        ((uint8_t) 0x00)    // DEFAULT
    #define DFT2_TREG_QPUMP_DIS_STOPCLOCKTOQPUMP                ((uint8_t) 0x04)

    /* LDOA_DIS field */
    #define DFT2_TREG_LDOA_DIS_MASK                             ((uint8_t) 0x02)
    #define DFT2_TREG_LDOA_DIS_BITOFFSET                        (1)
    #define DFT2_TREG_LDOA_DIS_LDOAENABLED                      ((uint8_t) 0x00)    // DEFAULT
    #define DFT2_TREG_LDOA_DIS_LDOADISABLED                     ((uint8_t) 0x02)


/* Register 0x04 (OVERLOAD_TREG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                   RESERVED[6:0]                                                                  |    OVERLOAD_FLAG   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OVERLOAD_TREG register */
    #define OVERLOAD_TREG_ADDRESS                               ((uint8_t) 0x04)
    #define OVERLOAD_TREG_DEFAULT                               ((uint8_t) 0x00)

    /* OVERLOAD_FLAG field */
    #define OVERLOAD_TREG_OVERLOAD_FLAG_MASK                    ((uint8_t) 0x01)
    #define OVERLOAD_TREG_OVERLOAD_FLAG_BITOFFSET               (0)


/* Register 0x05 (REF_TREG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |              RESERVED[1:0]              |  REF_TMD_SENSEVOUT |   REF_TMD_V_TKNEE  | REF_TMD_TEST_INULL |    REF_TMD_INULL   |             REF_TMD_BUF[1:0]            |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REF_TREG register */
    #define REF_TREG_ADDRESS                                    ((uint8_t) 0x05)
    #define REF_TREG_DEFAULT                                    ((uint8_t) 0x00)

    /* REF_TMD_SENSEVOUT field */
    #define REF_TREG_REF_TMD_SENSEVOUT_MASK                     ((uint8_t) 0x20)
    #define REF_TREG_REF_TMD_SENSEVOUT_BITOFFSET                (5)

    /* REF_TMD_V_TKNEE field */
    #define REF_TREG_REF_TMD_V_TKNEE_MASK                       ((uint8_t) 0x10)
    #define REF_TREG_REF_TMD_V_TKNEE_BITOFFSET                  (4)

    /* REF_TMD_TEST_INULL field */
    #define REF_TREG_REF_TMD_TEST_INULL_MASK                    ((uint8_t) 0x08)
    #define REF_TREG_REF_TMD_TEST_INULL_BITOFFSET               (3)

    /* REF_TMD_INULL field */
    #define REF_TREG_REF_TMD_INULL_MASK                         ((uint8_t) 0x04)
    #define REF_TREG_REF_TMD_INULL_BITOFFSET                    (2)

    /* REF_TMD_BUF field */
    #define REF_TREG_REF_TMD_BUF_MASK                           ((uint8_t) 0x03)
    #define REF_TREG_REF_TMD_BUF_BITOFFSET                      (0)
    #define REF_TREG_REF_TMD_BUF_NOCHANGE                       ((uint8_t) 0x00)    // DEFAULT
    #define REF_TREG_REF_TMD_BUF_DISABLEREFERENCEOUTPUTBUFFER   ((uint8_t) 0x01)
    #define REF_TREG_REF_TMD_BUF_OVSTONCAPSWITHINTHEREFERENCEOUTPUTBUFFER ((uint8_t) 0x02)
    #define REF_TREG_REF_TMD_BUF_OVSTONCAPSWITHINTHEREFERENCEOUTPUTBUFFER ((uint8_t) 0x03)


/* Register 0x06 (USERCRC_TREG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                           CALC_USERCRC[7:0]                                                                           |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* USERCRC_TREG register */
    #define USERCRC_TREG_ADDRESS                                ((uint8_t) 0x06)
    #define USERCRC_TREG_DEFAULT                                ((uint8_t) 0x00)

    /* CALC_USERCRC field */
    #define USERCRC_TREG_CALC_USERCRC_MASK                      ((uint8_t) 0xFF)
    #define USERCRC_TREG_CALC_USERCRC_BITOFFSET                 (0)


/* Register 0x07 (OTPCRC_TREG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                            CALC_OTPCRC[7:0]                                                                           |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OTPCRC_TREG register */
    #define OTPCRC_TREG_ADDRESS                                 ((uint8_t) 0x07)
    #define OTPCRC_TREG_DEFAULT                                 ((uint8_t) 0x00)

    /* CALC_OTPCRC field */
    #define OTPCRC_TREG_CALC_OTPCRC_MASK                        ((uint8_t) 0xFF)
    #define OTPCRC_TREG_CALC_OTPCRC_BITOFFSET                   (0)


/* Register 0x3D (TMEN1_TREG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                             TMODE_EN1[7:0]                                                                            |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TMEN1_TREG register */
    #define TMEN1_TREG_ADDRESS                                  ((uint8_t) 0x3D)
    #define TMEN1_TREG_DEFAULT                                  ((uint8_t) 0x00)

    /* TMODE_EN1 field */
    #define TMEN1_TREG_TMODE_EN1_MASK                           ((uint8_t) 0xFF)
    #define TMEN1_TREG_TMODE_EN1_BITOFFSET                      (0)


/* Register 0x3E (TMEN2_TREG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                             TMODE_EN2[7:0]                                                                            |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* TMEN2_TREG register */
    #define TMEN2_TREG_ADDRESS                                  ((uint8_t) 0x3E)
    #define TMEN2_TREG_DEFAULT                                  ((uint8_t) 0x00)

    /* TMODE_EN2 field */
    #define TMEN2_TREG_TMODE_EN2_MASK                           ((uint8_t) 0xFF)
    #define TMEN2_TREG_TMODE_EN2_BITOFFSET                      (0)


/* Register 0x3F (PAGE_POINTER) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                           PAGE_POINTER[7:0]                                                                           |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PAGE_POINTER register */
    #define PAGE_POINTER_ADDRESS                                ((uint8_t) 0x3F)
    #define PAGE_POINTER_DEFAULT                                ((uint8_t) 0x00)

    /* PAGE_POINTER field */
    #define PAGE_POINTER_MASK                                   ((uint8_t) 0xFF)
    #define PAGE_POINTER_BITOFFSET                              (0)


#endif /* ADS122S1X_PAGE1_H_ */
