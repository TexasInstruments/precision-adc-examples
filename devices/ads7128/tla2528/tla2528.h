/*
 * @file tla2528.h
 *
 * @brief TLA2528 Descriptor
 *
 * @copyright Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef TLA2528_H_
#define TLA2528_H_

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


/* Register 0x00 (SYSTEM_STATUS) definition
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7    |     Bit 6    |     Bit 5    |     Bit 4    |     Bit 3    |     Bit 2    |     Bit 1    |     Bit 0    |
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |     RSVD     |  SEQ_STATUS  |   I2C_SPEED  |   RESERVED   |   OSR_DONE   | CRC_ERR_FUSE |  CRC_ERR_IN  |      BOR     |
 * |-----------------------------------------------------------------------------------------------------------------------|
 */

    /* SYSTEM_STATUS register */
    #define SYSTEM_STATUS_ADDRESS											((uint8_t) 0x00)
    #define SYSTEM_STATUS_DEFAULT											((uint8_t) 0x81)

    /* SEQ_STATUS field */
    #define SEQ_STATUS_MASK													((uint8_t) 0x40)
    #define SEQ_STATUS_STOPPED												((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STATUS_RUNNING												((uint8_t) 0x40)

    /* I2C_SPEED field */
    #define I2C_SPEED_MASK													((uint8_t) 0x20)
    #define I2C_SPEED_NORMAL												((uint8_t) 0x00)    // DEFAULT
    #define I2C_SPEED_HIGH													((uint8_t) 0x20)

    /* OSR_DONE field */
    #define OSR_DONE_MASK													((uint8_t) 0x08)
    #define OSR_DONE_WAITING												((uint8_t) 0x00)    // DEFAULT
    #define OSR_DONE_COMPLETE												((uint8_t) 0x08)

    /* CRC_ERR_FUSE field */
    #define CRC_ERR_FUSE_MASK												((uint8_t) 0x04)
    #define CRC_ERR_FUSE_OKAY												((uint8_t) 0x00)    // DEFAULT
    #define CRC_ERR_FUSE_ERROR												((uint8_t) 0x04)

    /* CRC_ERR_IN field */
    #define CRC_ERR_IN_MASK													((uint8_t) 0x02)
    #define CRC_ERR_IN_OKAY													((uint8_t) 0x00)    // DEFAULT
    #define CRC_ERR_IN_ERROR												((uint8_t) 0x02)

    /* BOR field */
    #define BOR_MASK														((uint8_t) 0x01)
    #define BOR_OKAY														((uint8_t) 0x00)
    #define BOR_ERROR														((uint8_t) 0x01)    // DEFAULT



/* Register 0x01 (GENERAL_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7    |     Bit 6    |     Bit 5    |     Bit 4    |     Bit 3    |     Bit 2    |     Bit 1    |     Bit 0    |
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |                       RESERVED[3:0]                       |     CNVST    |    CH_RST    |      CAL     |      RST     |
 * |-----------------------------------------------------------------------------------------------------------------------|
 */

    /* GENERAL_CFG register */
    #define GENERAL_CFG_ADDRESS												((uint8_t) 0x01)
    #define GENERAL_CFG_DEFAULT												((uint8_t) 0x00)

    /* CNVST field */
    #define CNVST_MASK														((uint8_t) 0x08)
    #define CNVST_NORMAL_SCL_STRETCHED										((uint8_t) 0x00)    // DEFAULT
    #define CNVST_START_NO_STRETCH											((uint8_t) 0x08)

    /* CH_RST field */
    #define CH_RST_MASK														((uint8_t) 0x04)
    #define CH_RST_NORMAL													((uint8_t) 0x00)    // DEFAULT
    #define CH_RST_FORCE_AIN												((uint8_t) 0x04)

    /* CAL field */
    #define CAL_MASK														((uint8_t) 0x02)
    #define CAL_COMPLETE													((uint8_t) 0x00)    // DEFAULT
    #define CAL_START														((uint8_t) 0x02)

    /* RST field */
    #define RST_MASK														((uint8_t) 0x01)
    #define RST_COMPLETE													((uint8_t) 0x00)    // DEFAULT
    #define RST_START														((uint8_t) 0x01)



/* Register 0x02 (DATA_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7    |     Bit 6    |     Bit 5    |     Bit 4    |     Bit 3    |     Bit 2    |     Bit 1    |     Bit 0    |
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |    FIX_PAT   |   RESERVED   |      APPEND_STATUS[1:0]     |                       RESERVED[3:0]                       |
 * |-----------------------------------------------------------------------------------------------------------------------|
 */

    /* DATA_CFG register */
    #define DATA_CFG_ADDRESS												((uint8_t) 0x02)
    #define DATA_CFG_DEFAULT												((uint8_t) 0x00)

    /* FIX_PAT field */
    #define FIX_PAT_MASK													((uint8_t) 0x80)
    #define FIX_PAT_NORMAL													((uint8_t) 0x00)    // DEFAULT
    #define FIX_PAT_ENABLE													((uint8_t) 0x80)

    /* APPEND_STATUS field */
    #define APPEND_STATUS_MASK												((uint8_t) 0x30)
    #define APPEND_STATUS_DISABLE											((uint8_t) 0x00)    // DEFAULT
    #define APPEND_STATUS_ID												((uint8_t) 0x10)
    #define APPEND_STATUS_ONLY												((uint8_t) 0x20)



/* Register 0x03 (OSR_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7    |     Bit 6    |     Bit 5    |     Bit 4    |     Bit 3    |     Bit 2    |     Bit 1    |     Bit 0    |
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |                               RESERVED[4:0]                              |                  OSR[2:0]                  |
 * |-----------------------------------------------------------------------------------------------------------------------|
 */

    /* OSR_CFG register */
    #define OSR_CFG_ADDRESS													((uint8_t) 0x03)
    #define OSR_CFG_DEFAULT													((uint8_t) 0x00)

    /* OSR field */
    #define OSR_MASK														((uint8_t) 0x07)
    #define OSR_1															((uint8_t) 0x00)    // DEFAULT
    #define OSR_2															((uint8_t) 0x01)
    #define OSR_4															((uint8_t) 0x02)
    #define OSR_8															((uint8_t) 0x03)
    #define OSR_16															((uint8_t) 0x04)
    #define OSR_32															((uint8_t) 0x05)
    #define OSR_64															((uint8_t) 0x06)
    #define OSR_128															((uint8_t) 0x07)



/* Register 0x04 (OPMODE_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7    |     Bit 6    |     Bit 5    |     Bit 4    |     Bit 3    |     Bit 2    |     Bit 1    |     Bit 0    |
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |                RESERVED[2:0]               |    OSC_SEL   |                        CLK_DIV[3:0]                       |
 * |-----------------------------------------------------------------------------------------------------------------------|
 */

    /* OPMODE_CFG register */
    #define OPMODE_CFG_ADDRESS												((uint8_t) 0x04)
    #define OPMODE_CFG_DEFAULT												((uint8_t) 0x00)

    /* OSC_SEL field */
    #define OSC_SEL_MASK													((uint8_t) 0x10)
    #define OSC_SEL_HIGH_SPEED												((uint8_t) 0x00)    // DEFAULT
    #define OSC_SEL_LOW_POWER												((uint8_t) 0x10)



/* Register 0x05 (PIN_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7    |     Bit 6    |     Bit 5    |     Bit 4    |     Bit 3    |     Bit 2    |     Bit 1    |     Bit 0    |
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |                                                      PIN_CFG[7:0]                                                     |
 * |-----------------------------------------------------------------------------------------------------------------------|
 */

    /* PIN_CFG register */
    #define PIN_CFG_ADDRESS													((uint8_t) 0x05)
    #define PIN_CFG_DEFAULT													((uint8_t) 0x00)

    /* PIN_CFG field */
    #define PIN_CFG_MASK													((uint8_t) 0xFF)
    #define PIN_CFG_AIN0													((uint8_t) 0x01)
    #define PIN_CFG_AIN1													((uint8_t) 0x02)
    #define PIN_CFG_AIN2													((uint8_t) 0x04)
    #define PIN_CFG_AIN3													((uint8_t) 0x08)
    #define PIN_CFG_AIN4													((uint8_t) 0x10)
    #define PIN_CFG_AIN5													((uint8_t) 0x20)
    #define PIN_CFG_AIN6													((uint8_t) 0x40)
    #define PIN_CFG_AIN7													((uint8_t) 0x80)



/* Register 0x07 (GPIO_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7    |     Bit 6    |     Bit 5    |     Bit 4    |     Bit 3    |     Bit 2    |     Bit 1    |     Bit 0    |
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |                                                     GPIO_CFG[7:0]                                                     |
 * |-----------------------------------------------------------------------------------------------------------------------|
 */

    /* GPIO_CFG register */
    #define GPIO_CFG_ADDRESS												((uint8_t) 0x07)
    #define GPIO_CFG_DEFAULT												((uint8_t) 0x00)

    /* GPIO_CFG field */
    #define GPIO_CFG_MASK													((uint8_t) 0xFF)
    #define GPIO_CFG_GPO0													((uint8_t) 0x01)
    #define GPIO_CFG_GPO1													((uint8_t) 0x02)
    #define GPIO_CFG_GPO2													((uint8_t) 0x04)
    #define GPIO_CFG_GPO3													((uint8_t) 0x08)
    #define GPIO_CFG_GPO4													((uint8_t) 0x10)
    #define GPIO_CFG_GPO5													((uint8_t) 0x20)
    #define GPIO_CFG_GPO6													((uint8_t) 0x40)
    #define GPIO_CFG_GPO7													((uint8_t) 0x80)



/* Register 0x09 (GPO_DRIVE_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7    |     Bit 6    |     Bit 5    |     Bit 4    |     Bit 3    |     Bit 2    |     Bit 1    |     Bit 0    |
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |                                                   GPO_DRIVE_CFG[7:0]                                                  |
 * |-----------------------------------------------------------------------------------------------------------------------|
 */

    /* GPO_DRIVE_CFG register */
    #define GPO_DRIVE_CFG_ADDRESS											((uint8_t) 0x09)
    #define GPO_DRIVE_CFG_DEFAULT											((uint8_t) 0x00)

    /* GPO_DRIVE_CFG field */
    #define GPO_DRIVE_CFG_MASK												((uint8_t) 0xFF)
    #define GPO_DRIVE_CFG_GPO0												((uint8_t) 0x01)
    #define GPO_DRIVE_CFG_GPO1												((uint8_t) 0x02)
    #define GPO_DRIVE_CFG_GPO2												((uint8_t) 0x04)
    #define GPO_DRIVE_CFG_GPO3												((uint8_t) 0x08)
    #define GPO_DRIVE_CFG_GPO4												((uint8_t) 0x10)
    #define GPO_DRIVE_CFG_GPO5												((uint8_t) 0x20)
    #define GPO_DRIVE_CFG_GPO6												((uint8_t) 0x40)
    #define GPO_DRIVE_CFG_GPO7												((uint8_t) 0x80)



/* Register 0x0B (GPO_VALUE) definition
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7    |     Bit 6    |     Bit 5    |     Bit 4    |     Bit 3    |     Bit 2    |     Bit 1    |     Bit 0    |
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |                                                     GPO_VALUE[7:0]                                                    |
 * |-----------------------------------------------------------------------------------------------------------------------|
 */

    /* GPO_VALUE register */
    #define GPO_VALUE_ADDRESS												((uint8_t) 0x0B)
    #define GPO_VALUE_DEFAULT												((uint8_t) 0x00)

    /* GPO_VALUE field */
    #define GPO_VALUE_MASK													((uint8_t) 0xFF)
    #define GPO_VALUE_GPO0_HIGH												((uint8_t) 0x01)
    #define GPO_VALUE_GPO1_HIGH												((uint8_t) 0x02)
    #define GPO_VALUE_GPO2_HIGH												((uint8_t) 0x04)
    #define GPO_VALUE_GPO3_HIGH												((uint8_t) 0x08)
    #define GPO_VALUE_GPO4_HIGH												((uint8_t) 0x10)
    #define GPO_VALUE_GPO5_HIGH												((uint8_t) 0x20)
    #define GPO_VALUE_GPO6_HIGH												((uint8_t) 0x40)
    #define GPO_VALUE_GPO7_HIGH												((uint8_t) 0x80)



/* Register 0x0D (GPI_VALUE) definition
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7    |     Bit 6    |     Bit 5    |     Bit 4    |     Bit 3    |     Bit 2    |     Bit 1    |     Bit 0    |
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |                                                     GPI_VALUE[7:0]                                                    |
 * |-----------------------------------------------------------------------------------------------------------------------|
 */

    /* GPI_VALUE register */
    #define GPI_VALUE_ADDRESS												((uint8_t) 0x0D)
    #define GPI_VALUE_DEFAULT												((uint8_t) 0x00)

    /* GPI_VALUE field */
    #define GPI_VALUE_MASK													((uint8_t) 0xFF)
    #define GPI_VALUE_GPI0_HIGH												((uint8_t) 0x01)
    #define GPI_VALUE_GPI1_HIGH												((uint8_t) 0x02)
    #define GPI_VALUE_GPI2_HIGH												((uint8_t) 0x04)
    #define GPI_VALUE_GPI3_HIGH												((uint8_t) 0x08)
    #define GPI_VALUE_GPI4_HIGH												((uint8_t) 0x10)
    #define GPI_VALUE_GPI5_HIGH												((uint8_t) 0x20)
    #define GPI_VALUE_GPI6_HIGH												((uint8_t) 0x40)
    #define GPI_VALUE_GPI7_HIGH												((uint8_t) 0x80)



/* Register 0x10 (SEQUENCE_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7    |     Bit 6    |     Bit 5    |     Bit 4    |     Bit 3    |     Bit 2    |     Bit 1    |     Bit 0    |
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |                RESERVED[2:0]               |   SEQ_START  |        RESERVED[1:0]        |        SEQ_MODE[1:0]        |
 * |-----------------------------------------------------------------------------------------------------------------------|
 */

    /* SEQUENCE_CFG register */
    #define SEQUENCE_CFG_ADDRESS											((uint8_t) 0x10)
    #define SEQUENCE_CFG_DEFAULT											((uint8_t) 0x00)

    /* SEQ_START field */
    #define SEQ_START_MASK													((uint8_t) 0x10)
    #define SEQ_START_END													((uint8_t) 0x00)    // DEFAULT
    #define SEQ_START_ASSEND												((uint8_t) 0x10)

    /* SEQ_MODE field */
    #define SEQ_MODE_MASK													((uint8_t) 0x03)
    #define SEQ_MODE_MANUAL													((uint8_t) 0x00)    // DEFAULT
    #define SEQ_MODE_AUTO													((uint8_t) 0x01)



/* Register 0x11 (MANUAL_CH_SEL) definition
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7    |     Bit 6    |     Bit 5    |     Bit 4    |     Bit 3    |     Bit 2    |     Bit 1    |     Bit 0    |
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |                       RESERVED[3:0]                       |                      MANUAL_CHID[3:0]                     |
 * |-----------------------------------------------------------------------------------------------------------------------|
 */

    /* MANUAL_CH_SEL register */
    #define MANUAL_CH_SEL_ADDRESS											((uint8_t) 0x11)
    #define MANUAL_CH_SEL_DEFAULT											((uint8_t) 0x00)

    /* MANUAL_CHID field */
    #define MANUAL_CHID_MASK												((uint8_t) 0x0F)
    #define MANUAL_CHID_AIN0												((uint8_t) 0x00)    // DEFAULT
    #define MANUAL_CHID_AIN1												((uint8_t) 0x01)
    #define MANUAL_CHID_AIN2												((uint8_t) 0x02)
    #define MANUAL_CHID_AIN3												((uint8_t) 0x03)
    #define MANUAL_CHID_AIN4												((uint8_t) 0x04)
    #define MANUAL_CHID_AIN5												((uint8_t) 0x05)
    #define MANUAL_CHID_AIN6												((uint8_t) 0x06)
    #define MANUAL_CHID_AIN7												((uint8_t) 0x07)



/* Register 0x12 (AUTO_SEQ_CH_SEL) definition
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7    |     Bit 6    |     Bit 5    |     Bit 4    |     Bit 3    |     Bit 2    |     Bit 1    |     Bit 0    |
 * |-----------------------------------------------------------------------------------------------------------------------|
 * |                                                  AUTO_SEQ_CH_SEL[7:0]                                                 |
 * |-----------------------------------------------------------------------------------------------------------------------|
 */

    /* AUTO_SEQ_CH_SEL register */
    #define AUTO_SEQ_CH_SEL_ADDRESS											((uint8_t) 0x12)
    #define AUTO_SEQ_CH_SEL_DEFAULT											((uint8_t) 0x00)

    /* AUTO_SEQ_CH_SEL field */
    #define AUTO_SEQ_CH_SEL_MASK											((uint8_t) 0xFF)
    #define AUTO_SEQ_CH_SEL_AIN0											((uint8_t) 0x01)
    #define AUTO_SEQ_CH_SEL_AIN1											((uint8_t) 0x02)
    #define AUTO_SEQ_CH_SEL_AIN2											((uint8_t) 0x04)
    #define AUTO_SEQ_CH_SEL_AIN3											((uint8_t) 0x08)
    #define AUTO_SEQ_CH_SEL_AIN4											((uint8_t) 0x10)
    #define AUTO_SEQ_CH_SEL_AIN5											((uint8_t) 0x20)
    #define AUTO_SEQ_CH_SEL_AIN6											((uint8_t) 0x40)
    #define AUTO_SEQ_CH_SEL_AIN7											((uint8_t) 0x80)



#endif /* TLA2528_H_ */
