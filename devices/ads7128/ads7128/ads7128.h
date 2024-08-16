/*
 * Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef ADS7128_H_
#define ADS7128_H_

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
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |      RSVD     |   SEQ_STATUS  |   I2C_SPEED   |    RESERVED   |    OSR_DONE   |  CRC_ERR_FUSE |   CRC_ERR_IN  |      BOR      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
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
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    RESERVED   |     CRC_EN    |    STATS_EN   |     DWC_EN    |     CNVST     |     CH_RST    |      CAL      |      RST      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GENERAL_CFG register */
    #define GENERAL_CFG_ADDRESS												((uint8_t) 0x01)
    #define GENERAL_CFG_DEFAULT												((uint8_t) 0x00)

    /* CRC_EN field */
    #define CRC_EN_MASK														((uint8_t) 0x40)
    #define CRC_EN_DISABLE													((uint8_t) 0x00)    // DEFAULT
    #define CRC_EN_ENABLE													((uint8_t) 0x40)

    /* STATS_EN field */
    #define STATS_EN_MASK													((uint8_t) 0x20)
    #define STATS_EN_DISABLE												((uint8_t) 0x00)    // DEFAULT
    #define STATS_EN_ENABLE_CLEAR											((uint8_t) 0x20)

    /* DWC_EN field */
    #define DWC_EN_MASK														((uint8_t) 0x10)
    #define DWC_EN_DISABLE_RESET											((uint8_t) 0x00)    // DEFAULT
    #define DWC_EN_ENABLE													((uint8_t) 0x10)

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
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    FIX_PAT    |    RESERVED   |       APPEND_STATUS[1:0]      |                         RESERVED[3:0]                         |
 * |-------------------------------------------------------------------------------------------------------------------------------|
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
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                 RESERVED[4:0]                                 |                    OSR[2:0]                   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
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
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |  CONV_ON_ERR  |         CONV_MODE[1:0]        |    OSC_SEL    |                          CLK_DIV[3:0]                         |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OPMODE_CFG register */
    #define OPMODE_CFG_ADDRESS												((uint8_t) 0x04)
    #define OPMODE_CFG_DEFAULT												((uint8_t) 0x00)

    /* CONV_ON_ERR field */
    #define CONV_ON_ERR_MASK												((uint8_t) 0x80)
    #define CONV_ON_ERR_CONTINUE											((uint8_t) 0x00)    // DEFAULT
    #define CONV_ON_ERR_PAUSE												((uint8_t) 0x80)

    /* CONV_MODE field */
    #define CONV_MODE_MASK													((uint8_t) 0x60)
    #define CONV_MODE_MANUAL												((uint8_t) 0x00)    // DEFAULT
    #define CONV_MODE_AUTO													((uint8_t) 0x20)

    /* OSC_SEL field */
    #define OSC_SEL_MASK													((uint8_t) 0x10)
    #define OSC_SEL_HIGH_SPEED												((uint8_t) 0x00)    // DEFAULT
    #define OSC_SEL_LOW_POWER												((uint8_t) 0x10)



/* Register 0x05 (PIN_CFG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                          PIN_CFG[7:0]                                                         |
 * |-------------------------------------------------------------------------------------------------------------------------------|
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
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                         GPIO_CFG[7:0]                                                         |
 * |-------------------------------------------------------------------------------------------------------------------------------|
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
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       GPO_DRIVE_CFG[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
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
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                         GPO_VALUE[7:0]                                                        |
 * |-------------------------------------------------------------------------------------------------------------------------------|
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
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                         GPI_VALUE[7:0]                                                        |
 * |-------------------------------------------------------------------------------------------------------------------------------|
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



/* Register 0x0F (ZCD_BLANKING_CFG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    MULT_EN    |                                               ZCD_BLANKING[6:0]                                               |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* ZCD_BLANKING_CFG register */
    #define ZCD_BLANKING_CFG_ADDRESS										((uint8_t) 0x0F)
    #define ZCD_BLANKING_CFG_DEFAULT										((uint8_t) 0x00)

    /* MULT_EN field */
    #define MULT_EN_MASK													((uint8_t) 0x80)
    #define MULT_EN_1X														((uint8_t) 0x00)    // DEFAULT
    #define MULT_EN_8X														((uint8_t) 0x80)



/* Register 0x10 (SEQUENCE_CFG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                 RESERVED[2:0]                 |   SEQ_START   |         RESERVED[1:0]         |         SEQ_MODE[1:0]         |
 * |-------------------------------------------------------------------------------------------------------------------------------|
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
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                         RESERVED[3:0]                         |                        MANUAL_CHID[3:0]                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
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
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      AUTO_SEQ_CH_SEL[7:0]                                                     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
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



/* Register 0x14 (ALERT_CH_SEL) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       ALERT_CH_SEL[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* ALERT_CH_SEL register */
    #define ALERT_CH_SEL_ADDRESS											((uint8_t) 0x14)
    #define ALERT_CH_SEL_DEFAULT											((uint8_t) 0x00)

    /* ALERT_CH_SEL field */
    #define ALERT_CH_SEL_MASK												((uint8_t) 0xFF)
    #define ALERT_CH_SEL_AIN0												((uint8_t) 0x01)
    #define ALERT_CH_SEL_AIN1												((uint8_t) 0x02)
    #define ALERT_CH_SEL_AIN2												((uint8_t) 0x04)
    #define ALERT_CH_SEL_AIN3												((uint8_t) 0x08)
    #define ALERT_CH_SEL_AIN4												((uint8_t) 0x10)
    #define ALERT_CH_SEL_AIN5												((uint8_t) 0x20)
    #define ALERT_CH_SEL_AIN6												((uint8_t) 0x40)
    #define ALERT_CH_SEL_AIN7												((uint8_t) 0x80)



/* Register 0x16 (ALERT_FUNC_SEL) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                 RESERVED[6:0]                                                 |  ALERT_CRCIN  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* ALERT_FUNC_SEL register */
    #define ALERT_FUNC_SEL_ADDRESS											((uint8_t) 0x16)
    #define ALERT_FUNC_SEL_DEFAULT											((uint8_t) 0x00)

    /* ALERT_CRCIN field */
    #define ALERT_CRCIN_MASK												((uint8_t) 0x01)
    #define ALERT_CRCIN_NO_ASSERT											((uint8_t) 0x00)    // DEFAULT
    #define ALERT_CRCIN_ASSERT												((uint8_t) 0x01)



/* Register 0x17 (ALERT_PIN_CFG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                 RESERVED[4:0]                                 |     DRIVE     |           LOGIC[1:0]          |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* ALERT_PIN_CFG register */
    #define ALERT_PIN_CFG_ADDRESS											((uint8_t) 0x17)
    #define ALERT_PIN_CFG_DEFAULT											((uint8_t) 0x00)

    /* ALERT_DRIVE field */
    #define ALERT_DRIVE_MASK												((uint8_t) 0x04)
    #define ALERT_DRIVE_OPEN_DRAIN											((uint8_t) 0x00)    // DEFAULT
    #define ALERT_DRIVE_PUSH_PULL											((uint8_t) 0x04)

    /* ALERT_LOGIC field */
    #define ALERT_LOGIC_MASK												((uint8_t) 0x03)
    #define ALERT_LOGIC_ACTIVE_LOW											((uint8_t) 0x00)    // DEFAULT
    #define ALERT_LOGIC_ACTIVE_HIGH											((uint8_t) 0x01)
    #define ALERT_LOGIC_PULSE_LOW											((uint8_t) 0x02)
    #define ALERT_LOGIC_PULSE_HIGH											((uint8_t) 0x03)



/* Register 0x18 (EVENT_FLAG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                        EVENT_FLAG[7:0]                                                        |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* EVENT_FLAG register */
    #define EVENT_FLAG_ADDRESS												((uint8_t) 0x18)
    #define EVENT_FLAG_DEFAULT												((uint8_t) 0x00)

    /* EVENT_FLAG field */
    #define EVENT_FLAG_MASK													((uint8_t) 0xFF)
    #define EVENT_FLAG_CH0													((uint8_t) 0x01)
    #define EVENT_FLAG_CH1													((uint8_t) 0x02)
    #define EVENT_FLAG_CH2													((uint8_t) 0x04)
    #define EVENT_FLAG_CH3													((uint8_t) 0x08)
    #define EVENT_FLAG_CH4													((uint8_t) 0x10)
    #define EVENT_FLAG_CH5													((uint8_t) 0x20)
    #define EVENT_FLAG_CH6													((uint8_t) 0x40)
    #define EVENT_FLAG_CH7													((uint8_t) 0x80)



/* Register 0x1A (EVENT_HIGH_FLAG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      EVENT_HIGH_FLAG[7:0]                                                     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* EVENT_HIGH_FLAG register */
    #define EVENT_HIGH_FLAG_ADDRESS											((uint8_t) 0x1A)
    #define EVENT_HIGH_FLAG_DEFAULT											((uint8_t) 0x00)

    /* EVENT_HIGH_FLAG field */
    #define EVENT_HIGH_FLAG_MASK											((uint8_t) 0xFF)
    #define EVENT_HIGH_FLAG_CH0												((uint8_t) 0x01)
    #define EVENT_HIGH_FLAG_CH1												((uint8_t) 0x02)
    #define EVENT_HIGH_FLAG_CH2												((uint8_t) 0x04)
    #define EVENT_HIGH_FLAG_CH3												((uint8_t) 0x08)
    #define EVENT_HIGH_FLAG_CH4												((uint8_t) 0x10)
    #define EVENT_HIGH_FLAG_CH5												((uint8_t) 0x20)
    #define EVENT_HIGH_FLAG_CH6												((uint8_t) 0x40)
    #define EVENT_HIGH_FLAG_CH7												((uint8_t) 0x80)



/* Register 0x1C (EVENT_LOW_FLAG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      EVENT_LOW_FLAG[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* EVENT_LOW_FLAG register */
    #define EVENT_LOW_FLAG_ADDRESS											((uint8_t) 0x1C)
    #define EVENT_LOW_FLAG_DEFAULT											((uint8_t) 0x00)

    /* EVENT_LOW_FLAG field */
    #define EVENT_LOW_FLAG_MASK												((uint8_t) 0xFF)
    #define EVENT_LOW_FLAG_CH0												((uint8_t) 0x01)
    #define EVENT_LOW_FLAG_CH1												((uint8_t) 0x02)
    #define EVENT_LOW_FLAG_CH2												((uint8_t) 0x04)
    #define EVENT_LOW_FLAG_CH3												((uint8_t) 0x08)
    #define EVENT_LOW_FLAG_CH4												((uint8_t) 0x10)
    #define EVENT_LOW_FLAG_CH5												((uint8_t) 0x20)
    #define EVENT_LOW_FLAG_CH6												((uint8_t) 0x40)
    #define EVENT_LOW_FLAG_CH7												((uint8_t) 0x80)



/* Register 0x1E (EVENT_RGN) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                         EVENT_RGN[7:0]                                                        |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* EVENT_RGN register */
    #define EVENT_RGN_ADDRESS												((uint8_t) 0x1E)
    #define EVENT_RGN_DEFAULT												((uint8_t) 0x00)

    /* EVENT_RGN field */
    #define EVENT_RGN_MASK													((uint8_t) 0xFF)
    #define EVENT_RGN_CH0_LOGIC0_OR_INBAND									((uint8_t) 0x01)
    #define EVENT_RGN_CH1_LOGIC0_OR_INBAND									((uint8_t) 0x02)
    #define EVENT_RGN_CH2_LOGIC0_OR_INBAND									((uint8_t) 0x04)
    #define EVENT_RGN_CH3_LOGIC0_OR_INBAND									((uint8_t) 0x08)
    #define EVENT_RGN_CH4_LOGIC0_OR_INBAND									((uint8_t) 0x10)
    #define EVENT_RGN_CH5_LOGIC0_OR_INBAND									((uint8_t) 0x20)
    #define EVENT_RGN_CH6_LOGIC0_OR_INBAND									((uint8_t) 0x40)
    #define EVENT_RGN_CH7_LOGIC0_OR_INBAND									((uint8_t) 0x80)



/* Register 0x20 (HYSTERESIS_CH0) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                  HIGH_THRESHOLD_CH0_LSB[3:0]                  |                      HYSTERESIS_CH0[3:0]                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HYSTERESIS_CH0 register */
    #define HYSTERESIS_CH0_ADDRESS											((uint8_t) 0x20)
    #define HYSTERESIS_CH0_DEFAULT											((uint8_t) 0xF0)



/* Register 0x21 (HIGH_TH_CH0) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                  HIGH_THRESHOLD_CH0_MSB[7:0]                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HIGH_TH_CH0 register */
    #define HIGH_TH_CH0_ADDRESS												((uint8_t) 0x21)
    #define HIGH_TH_CH0_DEFAULT												((uint8_t) 0xFF)



/* Register 0x22 (EVENT_COUNT_CH0) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                   LOW_THRESHOLD_CH0_LSB[3:0]                  |                      EVENT_COUNT_CH0[3:0]                     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* EVENT_COUNT_CH0 register */
    #define EVENT_COUNT_CH0_ADDRESS											((uint8_t) 0x22)
    #define EVENT_COUNT_CH0_DEFAULT											((uint8_t) 0x00)



/* Register 0x23 (LOW_TH_CH0) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                   LOW_THRESHOLD_CH0_MSB[7:0]                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* LOW_TH_CH0 register */
    #define LOW_TH_CH0_ADDRESS												((uint8_t) 0x23)
    #define LOW_TH_CH0_DEFAULT												((uint8_t) 0x00)



/* Register 0x24 (HYSTERESIS_CH1) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                  HIGH_THRESHOLD_CH1_LSB[3:0]                  |                      HYSTERESIS_CH1[3:0]                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HYSTERESIS_CH1 register */
    #define HYSTERESIS_CH1_ADDRESS											((uint8_t) 0x24)
    #define HYSTERESIS_CH1_DEFAULT											((uint8_t) 0xF0)



/* Register 0x25 (HIGH_TH_CH1) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                  HIGH_THRESHOLD_CH1_MSB[7:0]                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HIGH_TH_CH1 register */
    #define HIGH_TH_CH1_ADDRESS												((uint8_t) 0x25)
    #define HIGH_TH_CH1_DEFAULT												((uint8_t) 0xFF)



/* Register 0x26 (EVENT_COUNT_CH1) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                   LOW_THRESHOLD_CH1_LSB[3:0]                  |                      EVENT_COUNT_CH1[3:0]                     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* EVENT_COUNT_CH1 register */
    #define EVENT_COUNT_CH1_ADDRESS											((uint8_t) 0x26)
    #define EVENT_COUNT_CH1_DEFAULT											((uint8_t) 0x00)



/* Register 0x27 (LOW_TH_CH1) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                   LOW_THRESHOLD_CH1_MSB[7:0]                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* LOW_TH_CH1 register */
    #define LOW_TH_CH1_ADDRESS												((uint8_t) 0x27)
    #define LOW_TH_CH1_DEFAULT												((uint8_t) 0x00)



/* Register 0x28 (HYSTERESIS_CH2) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                  HIGH_THRESHOLD_CH2_LSB[3:0]                  |                      HYSTERESIS_CH2[3:0]                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HYSTERESIS_CH2 register */
    #define HYSTERESIS_CH2_ADDRESS											((uint8_t) 0x28)
    #define HYSTERESIS_CH2_DEFAULT											((uint8_t) 0xF0)



/* Register 0x29 (HIGH_TH_CH2) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                  HIGH_THRESHOLD_CH2_MSB[7:0]                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HIGH_TH_CH2 register */
    #define HIGH_TH_CH2_ADDRESS												((uint8_t) 0x29)
    #define HIGH_TH_CH2_DEFAULT												((uint8_t) 0xFF)



/* Register 0x2A (EVENT_COUNT_CH2) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                   LOW_THRESHOLD_CH2_LSB[3:0]                  |                      EVENT_COUNT_CH2[3:0]                     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* EVENT_COUNT_CH2 register */
    #define EVENT_COUNT_CH2_ADDRESS											((uint8_t) 0x2A)
    #define EVENT_COUNT_CH2_DEFAULT											((uint8_t) 0x00)



/* Register 0x2B (LOW_TH_CH2) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                   LOW_THRESHOLD_CH2_MSB[7:0]                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* LOW_TH_CH2 register */
    #define LOW_TH_CH2_ADDRESS												((uint8_t) 0x2B)
    #define LOW_TH_CH2_DEFAULT												((uint8_t) 0x00)



/* Register 0x2C (HYSTERESIS_CH3) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                  HIGH_THRESHOLD_CH3_LSB[3:0]                  |                      HYSTERESIS_CH3[3:0]                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HYSTERESIS_CH3 register */
    #define HYSTERESIS_CH3_ADDRESS											((uint8_t) 0x2C)
    #define HYSTERESIS_CH3_DEFAULT											((uint8_t) 0xF0)



/* Register 0x2D (HIGH_TH_CH3) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                  HIGH_THRESHOLD_CH3_MSB[7:0]                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HIGH_TH_CH3 register */
    #define HIGH_TH_CH3_ADDRESS												((uint8_t) 0x2D)
    #define HIGH_TH_CH3_DEFAULT												((uint8_t) 0xFF)



/* Register 0x2E (EVENT_COUNT_CH3) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                   LOW_THRESHOLD_CH3_LSB[3:0]                  |                      EVENT_COUNT_CH3[3:0]                     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* EVENT_COUNT_CH3 register */
    #define EVENT_COUNT_CH3_ADDRESS											((uint8_t) 0x2E)
    #define EVENT_COUNT_CH3_DEFAULT											((uint8_t) 0x00)



/* Register 0x2F (LOW_TH_CH3) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                   LOW_THRESHOLD_CH3_MSB[7:0]                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* LOW_TH_CH3 register */
    #define LOW_TH_CH3_ADDRESS												((uint8_t) 0x2F)
    #define LOW_TH_CH3_DEFAULT												((uint8_t) 0x00)



/* Register 0x30 (HYSTERESIS_CH4) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                  HIGH_THRESHOLD_CH4_LSB[3:0]                  |                      HYSTERESIS_CH4[3:0]                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HYSTERESIS_CH4 register */
    #define HYSTERESIS_CH4_ADDRESS											((uint8_t) 0x30)
    #define HYSTERESIS_CH4_DEFAULT											((uint8_t) 0xF0)



/* Register 0x31 (HIGH_TH_CH4) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                  HIGH_THRESHOLD_CH4_MSB[7:0]                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HIGH_TH_CH4 register */
    #define HIGH_TH_CH4_ADDRESS												((uint8_t) 0x31)
    #define HIGH_TH_CH4_DEFAULT												((uint8_t) 0xFF)



/* Register 0x32 (EVENT_COUNT_CH4) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                   LOW_THRESHOLD_CH4_LSB[3:0]                  |                      EVENT_COUNT_CH4[3:0]                     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* EVENT_COUNT_CH4 register */
    #define EVENT_COUNT_CH4_ADDRESS											((uint8_t) 0x32)
    #define EVENT_COUNT_CH4_DEFAULT											((uint8_t) 0x00)



/* Register 0x33 (LOW_TH_CH4) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                   LOW_THRESHOLD_CH4_MSB[7:0]                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* LOW_TH_CH4 register */
    #define LOW_TH_CH4_ADDRESS												((uint8_t) 0x33)
    #define LOW_TH_CH4_DEFAULT												((uint8_t) 0x00)



/* Register 0x34 (HYSTERESIS_CH5) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                  HIGH_THRESHOLD_CH5_LSB[3:0]                  |                      HYSTERESIS_CH5[3:0]                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HYSTERESIS_CH5 register */
    #define HYSTERESIS_CH5_ADDRESS											((uint8_t) 0x34)
    #define HYSTERESIS_CH5_DEFAULT											((uint8_t) 0xF0)



/* Register 0x35 (HIGH_TH_CH5) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                  HIGH_THRESHOLD_CH5_MSB[7:0]                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HIGH_TH_CH5 register */
    #define HIGH_TH_CH5_ADDRESS												((uint8_t) 0x35)
    #define HIGH_TH_CH5_DEFAULT												((uint8_t) 0xFF)



/* Register 0x36 (EVENT_COUNT_CH5) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                   LOW_THRESHOLD_CH5_LSB[3:0]                  |                      EVENT_COUNT_CH5[3:0]                     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* EVENT_COUNT_CH5 register */
    #define EVENT_COUNT_CH5_ADDRESS											((uint8_t) 0x36)
    #define EVENT_COUNT_CH5_DEFAULT											((uint8_t) 0x00)



/* Register 0x37 (LOW_TH_CH5) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                   LOW_THRESHOLD_CH5_MSB[7:0]                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* LOW_TH_CH5 register */
    #define LOW_TH_CH5_ADDRESS												((uint8_t) 0x37)
    #define LOW_TH_CH5_DEFAULT												((uint8_t) 0x00)



/* Register 0x38 (HYSTERESIS_CH6) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                  HIGH_THRESHOLD_CH6_LSB[3:0]                  |                      HYSTERESIS_CH6[3:0]                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HYSTERESIS_CH6 register */
    #define HYSTERESIS_CH6_ADDRESS											((uint8_t) 0x38)
    #define HYSTERESIS_CH6_DEFAULT											((uint8_t) 0xF0)



/* Register 0x39 (HIGH_TH_CH6) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                  HIGH_THRESHOLD_CH6_MSB[7:0]                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HIGH_TH_CH6 register */
    #define HIGH_TH_CH6_ADDRESS												((uint8_t) 0x39)
    #define HIGH_TH_CH6_DEFAULT												((uint8_t) 0xFF)



/* Register 0x3A (EVENT_COUNT_CH6) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                   LOW_THRESHOLD_CH6_LSB[3:0]                  |                      EVENT_COUNT_CH6[3:0]                     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* EVENT_COUNT_CH6 register */
    #define EVENT_COUNT_CH6_ADDRESS											((uint8_t) 0x3A)
    #define EVENT_COUNT_CH6_DEFAULT											((uint8_t) 0x00)



/* Register 0x3B (LOW_TH_CH6) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                   LOW_THRESHOLD_CH6_MSB[7:0]                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* LOW_TH_CH6 register */
    #define LOW_TH_CH6_ADDRESS												((uint8_t) 0x3B)
    #define LOW_TH_CH6_DEFAULT												((uint8_t) 0x00)



/* Register 0x3C (HYSTERESIS_CH7) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                  HIGH_THRESHOLD_CH7_LSB[3:0]                  |                      HYSTERESIS_CH7[3:0]                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HYSTERESIS_CH7 register */
    #define HYSTERESIS_CH7_ADDRESS											((uint8_t) 0x3C)
    #define HYSTERESIS_CH7_DEFAULT											((uint8_t) 0xF0)



/* Register 0x3D (HIGH_TH_CH7) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                  HIGH_THRESHOLD_CH7_MSB[7:0]                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* HIGH_TH_CH7 register */
    #define HIGH_TH_CH7_ADDRESS												((uint8_t) 0x3D)
    #define HIGH_TH_CH7_DEFAULT												((uint8_t) 0xFF)



/* Register 0x3E (EVENT_COUNT_CH7) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                   LOW_THRESHOLD_CH7_LSB[3:0]                  |                      EVENT_COUNT_CH7[3:0]                     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* EVENT_COUNT_CH7 register */
    #define EVENT_COUNT_CH7_ADDRESS											((uint8_t) 0x3E)
    #define EVENT_COUNT_CH7_DEFAULT											((uint8_t) 0x00)



/* Register 0x3F (LOW_TH_CH7) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                   LOW_THRESHOLD_CH7_MSB[7:0]                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* LOW_TH_CH7 register */
    #define LOW_TH_CH7_ADDRESS												((uint8_t) 0x3F)
    #define LOW_TH_CH7_DEFAULT												((uint8_t) 0x00)



/* Register 0x60 (MAX_CH0_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MAX_VALUE_CH0_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MAX_CH0_LSB register */
    #define MAX_CH0_LSB_ADDRESS												((uint8_t) 0x60)
    #define MAX_CH0_LSB_DEFAULT												((uint8_t) 0x00)



/* Register 0x61 (MAX_CH0_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MAX_VALUE_CH0_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MAX_CH0_MSB register */
    #define MAX_CH0_MSB_ADDRESS												((uint8_t) 0x61)
    #define MAX_CH0_MSB_DEFAULT												((uint8_t) 0x00)



/* Register 0x62 (MAX_CH1_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MAX_VALUE_CH1_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MAX_CH1_LSB register */
    #define MAX_CH1_LSB_ADDRESS												((uint8_t) 0x62)
    #define MAX_CH1_LSB_DEFAULT												((uint8_t) 0x00)



/* Register 0x63 (MAX_CH1_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MAX_VALUE_CH1_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MAX_CH1_MSB register */
    #define MAX_CH1_MSB_ADDRESS												((uint8_t) 0x63)
    #define MAX_CH1_MSB_DEFAULT												((uint8_t) 0x00)



/* Register 0x64 (MAX_CH2_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MAX_VALUE_CH2_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MAX_CH2_LSB register */
    #define MAX_CH2_LSB_ADDRESS												((uint8_t) 0x64)
    #define MAX_CH2_LSB_DEFAULT												((uint8_t) 0x00)



/* Register 0x65 (MAX_CH2_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MAX_VALUE_CH2_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MAX_CH2_MSB register */
    #define MAX_CH2_MSB_ADDRESS												((uint8_t) 0x65)
    #define MAX_CH2_MSB_DEFAULT												((uint8_t) 0x00)



/* Register 0x66 (MAX_CH3_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MAX_VALUE_CH3_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MAX_CH3_LSB register */
    #define MAX_CH3_LSB_ADDRESS												((uint8_t) 0x66)
    #define MAX_CH3_LSB_DEFAULT												((uint8_t) 0x00)



/* Register 0x67 (MAX_CH3_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MAX_VALUE_CH3_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MAX_CH3_MSB register */
    #define MAX_CH3_MSB_ADDRESS												((uint8_t) 0x67)
    #define MAX_CH3_MSB_DEFAULT												((uint8_t) 0x00)



/* Register 0x68 (MAX_CH4_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MAX_VALUE_CH4_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MAX_CH4_LSB register */
    #define MAX_CH4_LSB_ADDRESS												((uint8_t) 0x68)
    #define MAX_CH4_LSB_DEFAULT												((uint8_t) 0x00)



/* Register 0x69 (MAX_CH4_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MAX_VALUE_CH4_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MAX_CH4_MSB register */
    #define MAX_CH4_MSB_ADDRESS												((uint8_t) 0x69)
    #define MAX_CH4_MSB_DEFAULT												((uint8_t) 0x00)



/* Register 0x6A (MAX_CH5_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MAX_VALUE_CH5_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MAX_CH5_LSB register */
    #define MAX_CH5_LSB_ADDRESS												((uint8_t) 0x6A)
    #define MAX_CH5_LSB_DEFAULT												((uint8_t) 0x00)



/* Register 0x6B (MAX_CH5_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MAX_VALUE_CH5_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MAX_CH5_MSB register */
    #define MAX_CH5_MSB_ADDRESS												((uint8_t) 0x6B)
    #define MAX_CH5_MSB_DEFAULT												((uint8_t) 0x00)



/* Register 0x6C (MAX_CH6_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MAX_VALUE_CH6_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MAX_CH6_LSB register */
    #define MAX_CH6_LSB_ADDRESS												((uint8_t) 0x6C)
    #define MAX_CH6_LSB_DEFAULT												((uint8_t) 0x00)



/* Register 0x6D (MAX_CH6_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MAX_VALUE_CH6_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MAX_CH6_MSB register */
    #define MAX_CH6_MSB_ADDRESS												((uint8_t) 0x6D)
    #define MAX_CH6_MSB_DEFAULT												((uint8_t) 0x00)



/* Register 0x6E (MAX_CH7_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MAX_VALUE_CH7_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MAX_CH7_LSB register */
    #define MAX_CH7_LSB_ADDRESS												((uint8_t) 0x6E)
    #define MAX_CH7_LSB_DEFAULT												((uint8_t) 0x00)



/* Register 0x6F (MAX_CH7_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MAX_VALUE_CH7_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MAX_CH7_MSB register */
    #define MAX_CH7_MSB_ADDRESS												((uint8_t) 0x6F)
    #define MAX_CH7_MSB_DEFAULT												((uint8_t) 0x00)



/* Register 0x80 (MIN_CH0_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MIN_VALUE_CH0_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MIN_CH0_LSB register */
    #define MIN_CH0_LSB_ADDRESS												((uint8_t) 0x80)
    #define MIN_CH0_LSB_DEFAULT												((uint8_t) 0xFF)



/* Register 0x81 (MIN_CH0_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MIN_VALUE_CH0_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MIN_CH0_MSB register */
    #define MIN_CH0_MSB_ADDRESS												((uint8_t) 0x81)
    #define MIN_CH0_MSB_DEFAULT												((uint8_t) 0xFF)



/* Register 0x82 (MIN_CH1_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MIN_VALUE_CH1_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MIN_CH1_LSB register */
    #define MIN_CH1_LSB_ADDRESS												((uint8_t) 0x82)
    #define MIN_CH1_LSB_DEFAULT												((uint8_t) 0xFF)



/* Register 0x83 (MIN_CH1_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MIN_VALUE_CH1_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MIN_CH1_MSB register */
    #define MIN_CH1_MSB_ADDRESS												((uint8_t) 0x83)
    #define MIN_CH1_MSB_DEFAULT												((uint8_t) 0xFF)



/* Register 0x84 (MIN_CH2_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MIN_VALUE_CH2_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MIN_CH2_LSB register */
    #define MIN_CH2_LSB_ADDRESS												((uint8_t) 0x84)
    #define MIN_CH2_LSB_DEFAULT												((uint8_t) 0xFF)



/* Register 0x85 (MIN_CH2_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MIN_VALUE_CH2_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MIN_CH2_MSB register */
    #define MIN_CH2_MSB_ADDRESS												((uint8_t) 0x85)
    #define MIN_CH2_MSB_DEFAULT												((uint8_t) 0xFF)



/* Register 0x86 (MIN_CH3_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MIN_VALUE_CH3_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MIN_CH3_LSB register */
    #define MIN_CH3_LSB_ADDRESS												((uint8_t) 0x86)
    #define MIN_CH3_LSB_DEFAULT												((uint8_t) 0xFF)



/* Register 0x87 (MIN_CH3_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MIN_VALUE_CH3_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MIN_CH3_MSB register */
    #define MIN_CH3_MSB_ADDRESS												((uint8_t) 0x87)
    #define MIN_CH3_MSB_DEFAULT												((uint8_t) 0xFF)



/* Register 0x88 (MIN_CH4_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MIN_VALUE_CH4_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MIN_CH4_LSB register */
    #define MIN_CH4_LSB_ADDRESS												((uint8_t) 0x88)
    #define MIN_CH4_LSB_DEFAULT												((uint8_t) 0xFF)



/* Register 0x89 (MIN_CH4_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MIN_VALUE_CH4_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MIN_CH4_MSB register */
    #define MIN_CH4_MSB_ADDRESS												((uint8_t) 0x89)
    #define MIN_CH4_MSB_DEFAULT												((uint8_t) 0xFF)



/* Register 0x8A (MIN_CH5_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MIN_VALUE_CH5_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MIN_CH5_LSB register */
    #define MIN_CH5_LSB_ADDRESS												((uint8_t) 0x8A)
    #define MIN_CH5_LSB_DEFAULT												((uint8_t) 0xFF)



/* Register 0x8B (MIN_CH5_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MIN_VALUE_CH5_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MIN_CH5_MSB register */
    #define MIN_CH5_MSB_ADDRESS												((uint8_t) 0x8B)
    #define MIN_CH5_MSB_DEFAULT												((uint8_t) 0xFF)



/* Register 0x8C (MIN_CH6_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MIN_VALUE_CH6_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MIN_CH6_LSB register */
    #define MIN_CH6_LSB_ADDRESS												((uint8_t) 0x8C)
    #define MIN_CH6_LSB_DEFAULT												((uint8_t) 0xFF)



/* Register 0x8D (MIN_CH6_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MIN_VALUE_CH6_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MIN_CH6_MSB register */
    #define MIN_CH6_MSB_ADDRESS												((uint8_t) 0x8D)
    #define MIN_CH6_MSB_DEFAULT												((uint8_t) 0xFF)



/* Register 0x8E (MIN_CH7_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MIN_VALUE_CH7_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MIN_CH7_LSB register */
    #define MIN_CH7_LSB_ADDRESS												((uint8_t) 0x8E)
    #define MIN_CH7_LSB_DEFAULT												((uint8_t) 0xFF)



/* Register 0x8F (MIN_CH7_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     MIN_VALUE_CH7_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MIN_CH7_MSB register */
    #define MIN_CH7_MSB_ADDRESS												((uint8_t) 0x8F)
    #define MIN_CH7_MSB_DEFAULT												((uint8_t) 0xFF)



/* Register 0xA0 (RECENT_CH0_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    LAST_VALUE_CH0_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RECENT_CH0_LSB register */
    #define RECENT_CH0_LSB_ADDRESS											((uint8_t) 0xA0)
    #define RECENT_CH0_LSB_DEFAULT											((uint8_t) 0x00)



/* Register 0xA1 (RECENT_CH0_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    LAST_VALUE_CH0_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RECENT_CH0_MSB register */
    #define RECENT_CH0_MSB_ADDRESS											((uint8_t) 0xA1)
    #define RECENT_CH0_MSB_DEFAULT											((uint8_t) 0x00)



/* Register 0xA2 (RECENT_CH1_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    LAST_VALUE_CH1_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RECENT_CH1_LSB register */
    #define RECENT_CH1_LSB_ADDRESS											((uint8_t) 0xA2)
    #define RECENT_CH1_LSB_DEFAULT											((uint8_t) 0x00)



/* Register 0xA3 (RECENT_CH1_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    LAST_VALUE_CH1_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RECENT_CH1_MSB register */
    #define RECENT_CH1_MSB_ADDRESS											((uint8_t) 0xA3)
    #define RECENT_CH1_MSB_DEFAULT											((uint8_t) 0x00)



/* Register 0xA4 (RECENT_CH2_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    LAST_VALUE_CH2_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RECENT_CH2_LSB register */
    #define RECENT_CH2_LSB_ADDRESS											((uint8_t) 0xA4)
    #define RECENT_CH2_LSB_DEFAULT											((uint8_t) 0x00)



/* Register 0xA5 (RECENT_CH2_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    LAST_VALUE_CH2_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RECENT_CH2_MSB register */
    #define RECENT_CH2_MSB_ADDRESS											((uint8_t) 0xA5)
    #define RECENT_CH2_MSB_DEFAULT											((uint8_t) 0x00)



/* Register 0xA6 (RECENT_CH3_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    LAST_VALUE_CH3_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RECENT_CH3_LSB register */
    #define RECENT_CH3_LSB_ADDRESS											((uint8_t) 0xA6)
    #define RECENT_CH3_LSB_DEFAULT											((uint8_t) 0x00)



/* Register 0xA7 (RECENT_CH3_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    LAST_VALUE_CH3_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RECENT_CH3_MSB register */
    #define RECENT_CH3_MSB_ADDRESS											((uint8_t) 0xA7)
    #define RECENT_CH3_MSB_DEFAULT											((uint8_t) 0x00)



/* Register 0xA8 (RECENT_CH4_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    LAST_VALUE_CH4_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RECENT_CH4_LSB register */
    #define RECENT_CH4_LSB_ADDRESS											((uint8_t) 0xA8)
    #define RECENT_CH4_LSB_DEFAULT											((uint8_t) 0x00)



/* Register 0xA9 (RECENT_CH4_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    LAST_VALUE_CH4_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RECENT_CH4_MSB register */
    #define RECENT_CH4_MSB_ADDRESS											((uint8_t) 0xA9)
    #define RECENT_CH4_MSB_DEFAULT											((uint8_t) 0x00)



/* Register 0xAA (RECENT_CH5_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    LAST_VALUE_CH5_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RECENT_CH5_LSB register */
    #define RECENT_CH5_LSB_ADDRESS											((uint8_t) 0xAA)
    #define RECENT_CH5_LSB_DEFAULT											((uint8_t) 0x00)



/* Register 0xAB (RECENT_CH5_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    LAST_VALUE_CH5_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RECENT_CH5_MSB register */
    #define RECENT_CH5_MSB_ADDRESS											((uint8_t) 0xAB)
    #define RECENT_CH5_MSB_DEFAULT											((uint8_t) 0x00)



/* Register 0xAC (RECENT_CH6_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    LAST_VALUE_CH6_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RECENT_CH6_LSB register */
    #define RECENT_CH6_LSB_ADDRESS											((uint8_t) 0xAC)
    #define RECENT_CH6_LSB_DEFAULT											((uint8_t) 0x00)



/* Register 0xAD (RECENT_CH6_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    LAST_VALUE_CH6_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RECENT_CH6_MSB register */
    #define RECENT_CH6_MSB_ADDRESS											((uint8_t) 0xAD)
    #define RECENT_CH6_MSB_DEFAULT											((uint8_t) 0x00)



/* Register 0xAE (RECENT_CH7_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    LAST_VALUE_CH7_LSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RECENT_CH7_LSB register */
    #define RECENT_CH7_LSB_ADDRESS											((uint8_t) 0xAE)
    #define RECENT_CH7_LSB_DEFAULT											((uint8_t) 0x00)



/* Register 0xAF (RECENT_CH7_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    LAST_VALUE_CH7_MSB[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RECENT_CH7_MSB register */
    #define RECENT_CH7_MSB_ADDRESS											((uint8_t) 0xAF)
    #define RECENT_CH7_MSB_DEFAULT											((uint8_t) 0x00)



/* Register 0xC0 (RMS_CFG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                         RMS_CHID[3:0]                         |    RESERVED   |   RMS_DC_SUB  |        RMS_SAMPLES[1:0]       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RMS_CFG register */
    #define RMS_CFG_ADDRESS													((uint8_t) 0xC0)
    #define RMS_CFG_DEFAULT													((uint8_t) 0x00)

    /* RMS_CHID field */
    #define RMS_CHID_MASK													((uint8_t) 0xF0)
    #define RMS_CHID_AIN0													((uint8_t) 0x00)    // DEFAULT
    #define RMS_CHID_AIN1													((uint8_t) 0x10)
    #define RMS_CHID_AIN2													((uint8_t) 0x20)
    #define RMS_CHID_AIN3													((uint8_t) 0x30)
    #define RMS_CHID_AIN4													((uint8_t) 0x40)
    #define RMS_CHID_AIN5													((uint8_t) 0x50)
    #define RMS_CHID_AIN6													((uint8_t) 0x60)
    #define RMS_CHID_AIN7													((uint8_t) 0x70)

    /* RMS_DC_SUB field */
    #define RMS_DC_SUB_MASK													((uint8_t) 0x04)
    #define RMS_DC_SUB_DISABLE												((uint8_t) 0x00)    // DEFAULT
    #define RMS_DC_SUB_ENABLE												((uint8_t) 0x04)

    /* RMS_SAMPLES field */
    #define RMS_SAMPLES_MASK												((uint8_t) 0x03)
    #define RMS_SAMPLES_1024												((uint8_t) 0x00)    // DEFAULT
    #define RMS_SAMPLES_4096												((uint8_t) 0x01)
    #define RMS_SAMPLES_16384												((uint8_t) 0x02)
    #define RMS_SAMPLES_65536												((uint8_t) 0x03)



/* Register 0xC1 (RMS_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      RMS_RESULT_LSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RMS_LSB register */
    #define RMS_LSB_ADDRESS													((uint8_t) 0xC1)
    #define RMS_LSB_DEFAULT													((uint8_t) 0x00)



/* Register 0xC2 (RMS_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      RMS_RESULT_MSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RMS_MSB register */
    #define RMS_MSB_ADDRESS													((uint8_t) 0xC2)
    #define RMS_MSB_DEFAULT													((uint8_t) 0x00)



/* Register 0xC3 (GPO0_TRIG_EVENT_SEL) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    GPO0_TRIG_EVENT_SEL[7:0]                                                   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPO0_TRIG_EVENT_SEL register */
    #define GPO0_TRIG_EVENT_SEL_ADDRESS										((uint8_t) 0xC3)
    #define GPO0_TRIG_EVENT_SEL_DEFAULT										((uint8_t) 0x02)



/* Register 0xC5 (GPO1_TRIG_EVENT_SEL) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    GPO1_TRIG_EVENT_SEL[7:0]                                                   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPO1_TRIG_EVENT_SEL register */
    #define GPO1_TRIG_EVENT_SEL_ADDRESS										((uint8_t) 0xC5)
    #define GPO1_TRIG_EVENT_SEL_DEFAULT										((uint8_t) 0x02)



/* Register 0xC7 (GPO2_TRIG_EVENT_SEL) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    GPO2_TRIG_EVENT_SEL[7:0]                                                   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPO2_TRIG_EVENT_SEL register */
    #define GPO2_TRIG_EVENT_SEL_ADDRESS										((uint8_t) 0xC7)
    #define GPO2_TRIG_EVENT_SEL_DEFAULT										((uint8_t) 0x02)



/* Register 0xC9 (GPO3_TRIG_EVENT_SEL) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    GPO3_TRIG_EVENT_SEL[7:0]                                                   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPO3_TRIG_EVENT_SEL register */
    #define GPO3_TRIG_EVENT_SEL_ADDRESS										((uint8_t) 0xC9)
    #define GPO3_TRIG_EVENT_SEL_DEFAULT										((uint8_t) 0x02)



/* Register 0xCB (GPO4_TRIG_EVENT_SEL) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    GPO4_TRIG_EVENT_SEL[7:0]                                                   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPO4_TRIG_EVENT_SEL register */
    #define GPO4_TRIG_EVENT_SEL_ADDRESS										((uint8_t) 0xCB)
    #define GPO4_TRIG_EVENT_SEL_DEFAULT										((uint8_t) 0x02)



/* Register 0xCD (GPO5_TRIG_EVENT_SEL) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    GPO5_TRIG_EVENT_SEL[7:0]                                                   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPO5_TRIG_EVENT_SEL register */
    #define GPO5_TRIG_EVENT_SEL_ADDRESS										((uint8_t) 0xCD)
    #define GPO5_TRIG_EVENT_SEL_DEFAULT										((uint8_t) 0x02)



/* Register 0xCF (GPO6_TRIG_EVENT_SEL) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    GPO6_TRIG_EVENT_SEL[7:0]                                                   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPO6_TRIG_EVENT_SEL register */
    #define GPO6_TRIG_EVENT_SEL_ADDRESS										((uint8_t) 0xCF)
    #define GPO6_TRIG_EVENT_SEL_DEFAULT										((uint8_t) 0x02)



/* Register 0xD1 (GPO7_TRIG_EVENT_SEL) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                    GPO7_TRIG_EVENT_SEL[7:0]                                                   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPO7_TRIG_EVENT_SEL register */
    #define GPO7_TRIG_EVENT_SEL_ADDRESS										((uint8_t) 0xD1)
    #define GPO7_TRIG_EVENT_SEL_DEFAULT										((uint8_t) 0x02)



/* Register 0xE3 (GPO_VALUE_ZCD_CFG_CH0_CH3) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |   GPO_VALUE_ZCD_CFG_CH3[1:0]  |   GPO_VALUE_ZCD_CFG_CH2[1:0]  |   GPO_VALUE_ZCD_CFG_CH1[1:0]  |   GPO_VALUE_ZCD_CFG_CH0[1:0]  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPO_VALUE_ZCD_CFG_CH0_CH3 register */
    #define GPO_VALUE_ZCD_CFG_CH0_CH3_ADDRESS								((uint8_t) 0xE3)
    #define GPO_VALUE_ZCD_CFG_CH0_CH3_DEFAULT								((uint8_t) 0x00)

    /* GPO_VALUE_ZCD_CFG_CH3 field */
    #define GPO_VALUE_ZCD_CFG_CH3_MASK										((uint8_t) 0xC0)
    #define GPO_VALUE_ZCD_CFG_CH3_RISE0_FALL0								((uint8_t) 0x00)    // DEFAULT
    #define GPO_VALUE_ZCD_CFG_CH3_RISE0_FALL1								((uint8_t) 0x40)
    #define GPO_VALUE_ZCD_CFG_CH3_RISE1_FALL0								((uint8_t) 0x80)
    #define GPO_VALUE_ZCD_CFG_CH3_RISE1_FALL1								((uint8_t) 0xC0)

    /* GPO_VALUE_ZCD_CFG_CH2 field */
    #define GPO_VALUE_ZCD_CFG_CH2_MASK										((uint8_t) 0x30)
    #define GPO_VALUE_ZCD_CFG_CH2_RISE0_FALL0								((uint8_t) 0x00)    // DEFAULT
    #define GPO_VALUE_ZCD_CFG_CH2_RISE0_FALL1								((uint8_t) 0x10)
    #define GPO_VALUE_ZCD_CFG_CH2_RISE1_FALL0								((uint8_t) 0x20)
    #define GPO_VALUE_ZCD_CFG_CH2_RISE1_FALL1								((uint8_t) 0x30)

    /* GPO_VALUE_ZCD_CFG_CH1 field */
    #define GPO_VALUE_ZCD_CFG_CH1_MASK										((uint8_t) 0x0C)
    #define GPO_VALUE_ZCD_CFG_CH1_RISE0_FALL0								((uint8_t) 0x00)    // DEFAULT
    #define GPO_VALUE_ZCD_CFG_CH1_RISE0_FALL1								((uint8_t) 0x04)
    #define GPO_VALUE_ZCD_CFG_CH1_RISE1_FALL0								((uint8_t) 0x08)
    #define GPO_VALUE_ZCD_CFG_CH1_RISE1_FALL1								((uint8_t) 0x0C)

    /* GPO_VALUE_ZCD_CFG_CH0 field */
    #define GPO_VALUE_ZCD_CFG_CH0_MASK										((uint8_t) 0x03)
    #define GPO_VALUE_ZCD_CFG_CH0_RISE0_FALL0								((uint8_t) 0x00)    // DEFAULT
    #define GPO_VALUE_ZCD_CFG_CH0_RISE0_FALL1								((uint8_t) 0x01)
    #define GPO_VALUE_ZCD_CFG_CH0_RISE1_FALL0								((uint8_t) 0x02)
    #define GPO_VALUE_ZCD_CFG_CH0_RISE1_FALL1								((uint8_t) 0x03)



/* Register 0xE4 (GPO_VALUE_ZCD_CFG_CH4_CH7) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |   GPO_VALUE_ZCD_CFG_CH7[1:0]  |   GPO_VALUE_ZCD_CFG_CH6[1:0]  |   GPO_VALUE_ZCD_CFG_CH5[1:0]  |   GPO_VALUE_ZCD_CFG_CH4[1:0]  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPO_VALUE_ZCD_CFG_CH4_CH7 register */
    #define GPO_VALUE_ZCD_CFG_CH4_CH7_ADDRESS								((uint8_t) 0xE4)
    #define GPO_VALUE_ZCD_CFG_CH4_CH7_DEFAULT								((uint8_t) 0x00)

    /* GPO_VALUE_ZCD_CFG_CH7 field */
    #define GPO_VALUE_ZCD_CFG_CH7_MASK										((uint8_t) 0xC0)
    #define GPO_VALUE_ZCD_CFG_CH7_RISE0_FALL0								((uint8_t) 0x00)    // DEFAULT
    #define GPO_VALUE_ZCD_CFG_CH7_RISE0_FALL1								((uint8_t) 0x40)
    #define GPO_VALUE_ZCD_CFG_CH7_RISE1_FALL0								((uint8_t) 0x80)
    #define GPO_VALUE_ZCD_CFG_CH7_RISE1_FALL1								((uint8_t) 0xC0)

    /* GPO_VALUE_ZCD_CFG_CH6 field */
    #define GPO_VALUE_ZCD_CFG_CH6_MASK										((uint8_t) 0x30)
    #define GPO_VALUE_ZCD_CFG_CH6_RISE0_FALL0								((uint8_t) 0x00)    // DEFAULT
    #define GPO_VALUE_ZCD_CFG_CH6_RISE0_FALL1								((uint8_t) 0x10)
    #define GPO_VALUE_ZCD_CFG_CH6_RISE1_FALL0								((uint8_t) 0x20)
    #define GPO_VALUE_ZCD_CFG_CH6_RISE1_FALL1								((uint8_t) 0x30)

    /* GPO_VALUE_ZCD_CFG_CH5 field */
    #define GPO_VALUE_ZCD_CFG_CH5_MASK										((uint8_t) 0x0C)
    #define GPO_VALUE_ZCD_CFG_CH5_RISE0_FALL0								((uint8_t) 0x00)    // DEFAULT
    #define GPO_VALUE_ZCD_CFG_CH5_RISE0_FALL1								((uint8_t) 0x04)
    #define GPO_VALUE_ZCD_CFG_CH5_RISE1_FALL0								((uint8_t) 0x08)
    #define GPO_VALUE_ZCD_CFG_CH5_RISE1_FALL1								((uint8_t) 0x0C)

    /* GPO_VALUE_ZCD_CFG_CH4 field */
    #define GPO_VALUE_ZCD_CFG_CH4_MASK										((uint8_t) 0x03)
    #define GPO_VALUE_ZCD_CFG_CH4_RISE0_FALL0								((uint8_t) 0x00)    // DEFAULT
    #define GPO_VALUE_ZCD_CFG_CH4_RISE0_FALL1								((uint8_t) 0x01)
    #define GPO_VALUE_ZCD_CFG_CH4_RISE1_FALL0								((uint8_t) 0x02)
    #define GPO_VALUE_ZCD_CFG_CH4_RISE1_FALL1								((uint8_t) 0x03)



/* Register 0xE7 (GPO_ZCD_UPDATE_EN) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                     GPO_ZCD_UPDATE_EN[7:0]                                                    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPO_ZCD_UPDATE_EN register */
    #define GPO_ZCD_UPDATE_EN_ADDRESS										((uint8_t) 0xE7)
    #define GPO_ZCD_UPDATE_EN_DEFAULT										((uint8_t) 0x00)



/* Register 0xE9 (GPO_TRIGGER_CFG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                   GPO_TRIGGER_UPDATE_EN[7:0]                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPO_TRIGGER_CFG register */
    #define GPO_TRIGGER_CFG_ADDRESS											((uint8_t) 0xE9)
    #define GPO_TRIGGER_CFG_DEFAULT											((uint8_t) 0x00)

    /* GPO_TRIGGER_UPDATE_EN field */
    #define GPO_TRIGGER_UPDATE_EN_MASK										((uint8_t) 0xFF)
    #define GPO_TRIGGER_UPDATE_EN_GP0										((uint8_t) 0x01)
    #define GPO_TRIGGER_UPDATE_EN_GP1										((uint8_t) 0x02)
    #define GPO_TRIGGER_UPDATE_EN_GP2										((uint8_t) 0x04)
    #define GPO_TRIGGER_UPDATE_EN_GP3										((uint8_t) 0x08)
    #define GPO_TRIGGER_UPDATE_EN_GP4										((uint8_t) 0x10)
    #define GPO_TRIGGER_UPDATE_EN_GP5										((uint8_t) 0x20)
    #define GPO_TRIGGER_UPDATE_EN_GP6										((uint8_t) 0x40)
    #define GPO_TRIGGER_UPDATE_EN_GP7										((uint8_t) 0x80)



/* Register 0xEB (GPO_VALUE_TRIG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                   GPO_VALUE_ON_TRIGGER[7:0]                                                   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPO_VALUE_TRIG register */
    #define GPO_VALUE_TRIG_ADDRESS											((uint8_t) 0xEB)
    #define GPO_VALUE_TRIG_DEFAULT											((uint8_t) 0x00)

    /* GPO_VALUE_ON_TRIGGER field */
    #define GPO_VALUE_ON_TRIGGER_MASK										((uint8_t) 0xFF)
    #define GPO_VALUE_ON_TRIGGER_GP0_HIGH									((uint8_t) 0x01)
    #define GPO_VALUE_ON_TRIGGER_GP1_HIGH									((uint8_t) 0x02)
    #define GPO_VALUE_ON_TRIGGER_GP2_HIGH									((uint8_t) 0x04)
    #define GPO_VALUE_ON_TRIGGER_GP3_HIGH									((uint8_t) 0x08)
    #define GPO_VALUE_ON_TRIGGER_GP4_HIGH									((uint8_t) 0x10)
    #define GPO_VALUE_ON_TRIGGER_GP5_HIGH									((uint8_t) 0x20)
    #define GPO_VALUE_ON_TRIGGER_GP6_HIGH									((uint8_t) 0x40)
    #define GPO_VALUE_ON_TRIGGER_GP7_HIGH									((uint8_t) 0x80)



#endif /* ADS7128_H_ */
