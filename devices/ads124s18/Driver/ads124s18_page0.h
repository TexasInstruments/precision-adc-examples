/**
 * @file ads124s18_page0.h
 * @brief ADS124S18 Page 0 (GENERAL_SETTINGS) register definitions.
 *
 * Defines all register addresses, bit masks, and default values for the
 * GENERAL_SETTINGS page (Page 0) of the ADS124S18 register map.
 *
 * Page 0 Contents (64 bytes, addresses 0x00-0x3F):
 * - Device ID and status registers (read-only)
 * - Global configuration: power, reference, clock, SPI format
 * - Data format control: resolution, coding, status output
 * - Sequencer and conversion control
 * - GPIO and monitoring configuration
 * - Register map CRC protection
 *
 * Typical initialization sequence:
 * 1. Read DEVICE_ID to verify connected device
 * 2. Configure ADC_CFG (reference, clock speed)
 * 3. Configure DIGITAL_CFG (status, resolution, encoding)
 * 4. Configure CONVERSION_CTRL (conversion mode)
 * 5. Configure SEQUENCER_CFG (if using sequencer mode)
 * 6. Write SEQUENCE_STEP_EN_X registers to enable active steps
 *
 * Shadow registers in driver:
 * registerMap[GENERAL_SETTINGS][address] maintains software copy
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

#ifndef ADS124S18_PAGE0_H_
#define ADS124S18_PAGE0_H_

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


/* Register 0x00 (DEVICE_ID) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |              RESERVED[1:0]              |               CH_CNT[1:0]               |                                    DEV_ID[3:0]                                    |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DEVICE_ID register */
    #define DEVICE_ID_ADDRESS									((uint8_t) 0x00)
    #define DEVICE_ID_DEFAULT									((uint8_t) 0x33)
    #define DEVICE_ID_DEFAULT_MASK								((uint8_t) 0xC0)

    /* CH_CNT field */
    #define CH_CNT_MASK											((uint8_t) 0x30)
    #define DEVICE_ID_CH_CNT_BITOFFSET							(4)
    #define DEVICE_ID_CH_CNT_18_MASK                            ((uint8_t 0x30))
    #define DEVICE_ID_CH_CNT_08_MASK                            ((uint8_t 0x20))

    /* DEV_ID field */
    #define DEV_ID_MASK											((uint8_t) 0x0F)
    #define DEVICE_ID_DEV_ID_BITOFFSET							(0)
    #define DEVICE_IS_16_BIT_MASK                               ((uint8_t) 0x02)
    #define DEVICE_IS_24_BIT_MASK                               ((uint8_t) 0x03)


/* Register 0x01 (REVISION_ID) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                              REV_ID[7:0]                                                                              |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REVISION_ID register */
    #define REVISION_ID_ADDRESS									((uint8_t) 0x01)
    #define REVISION_ID_DEFAULT                                 ((uint8_t) 0x00)

    /* REV_ID field */
    #define REV_ID_MASK											((uint8_t) 0xFF)
    #define REVISION_ID_REV_ID_BITOFFSET						(0)


/* Register 0x02 (STATUS_MSB) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                           STEP_INDICATOR[4:0]                                          |     SAD_FAULTn     |       RESETn       |        DRDY        |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STATUS_MSB register */
    #define STATUS_MSB_ADDRESS									((uint8_t) 0x02)
    #define STATUS_MSB_DEFAULT									((uint8_t) 0x00)

    /* STEP_INDICATOR field */
    #define STEP_INDICATOR_MASK									((uint8_t) 0xF8)
    #define STATUS_MSB_STEP_INDICATOR_BITOFFSET					(3)

    /* SAD_FAULTn field */
    #define SAD_FAULTN_MASK										((uint8_t) 0x04)
    #define STATUS_MSB_SAD_FAULTN_BITOFFSET						(2)
    #define SAD_FAULTN_FAULTOCCURRED							((uint8_t) 0x00)    // DEFAULT
    #define SAD_FAULTN_NOFAULTOCCURRED							((uint8_t) 0x04)

    /* RESETn field */
    #define RESETN_MASK											((uint8_t) 0x02)
    #define STATUS_MSB_RESETN_BITOFFSET							(1)
    #define RESETN_RESETOCCURRED								((uint8_t) 0x00)    // DEFAULT
    #define RESETN_NORESETOCCURRED								((uint8_t) 0x02)

    /* DRDY field */
    #define DRDY_MASK											((uint8_t) 0x01)
    #define STATUS_MSB_DRDY_BITOFFSET							(0)
    #define DRDY_DATAARENOTNEW									((uint8_t) 0x00)    // DEFAULT
    #define DRDY_DATAARENEW										((uint8_t) 0x01)


/* Register 0x03 (STATUS_LSB) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                  CONV_COUNT[3:0]                                  |              RESERVED[1:0]              |  REG_WRITE_FAULTn  |   SPI_CRC_FAULTn   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STATUS_LSB register */
    #define STATUS_LSB_ADDRESS									((uint8_t) 0x03)
    #define STATUS_LSB_DEFAULT									((uint8_t) 0xFF)

    /* CONV_COUNT field */
    #define CONV_COUNT_MASK										((uint8_t) 0xF0)
    #define STATUS_LSB_CONV_COUNT_BITOFFSET						(4)

    /* REG_WRITE_FAULTn field */
    #define REG_WRITE_FAULTN_MASK								((uint8_t) 0x02)
    #define STATUS_LSB_REG_WRITE_FAULTN_BITOFFSET				(1)
    #define REG_WRITE_FAULTN_FAULTOCCURRED						((uint8_t) 0x00)
    #define REG_WRITE_FAULTN_NOFAULTOCCURRED					((uint8_t) 0x02)    // DEFAULT

    /* SPI_CRC_FAULTn field */
    #define SPI_CRC_FAULTN_MASK									((uint8_t) 0x01)
    #define STATUS_LSB_SPI_CRC_FAULTN_BITOFFSET					(0)
    #define SPI_CRC_FAULTN_FAULTOCCURRED						((uint8_t) 0x00)
    #define SPI_CRC_FAULTN_NOFAULTOCCURRED						((uint8_t) 0x01)    // DEFAULT


/* Register 0x04 (SUPPLY_STATUS) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |      RESERVED      |      AVDD_UVn      |                                                        RESERVED[5:0]                                                        |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SUPPLY_STATUS register */
    #define SUPPLY_STATUS_ADDRESS								((uint8_t) 0x04)
    #define SUPPLY_STATUS_DEFAULT								((uint8_t) 0xBF)

    /* AVDD_UVn field */
    #define AVDD_UVN_MASK										((uint8_t) 0x40)
    #define SUPPLY_STATUS_AVDD_UVN_BITOFFSET					(6)
    #define AVDD_UVN_FAULTOCCURRED								((uint8_t) 0x00)    // DEFAULT
    #define AVDD_UVN_NOFAULTOCCURRED							((uint8_t) 0x40)


/* Register 0x05 (ADC_STATUS) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     COMP_ALERTn    |              RESERVED[1:0]              |       REF_UVn      |                                   RESERVED[3:0]                                   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* ADC_STATUS register */
    #define ADC_STATUS_ADDRESS									((uint8_t) 0x05)
    #define ADC_STATUS_DEFAULT									((uint8_t) 0xFF)

    /* COMP_ALERTn field */
    #define COMP_ALERTN_MASK									((uint8_t) 0x80)
    #define ADC_STATUS_COMP_ALERTN_BITOFFSET					(7)
    #define COMP_ALERTN_ALERTOCCURRED							((uint8_t) 0x00)
    #define COMP_ALERTN_NOALERTOCCURRED							((uint8_t) 0x80)    // DEFAULT

    /* REF_UVn field */
    #define REF_UVN_MASK										((uint8_t) 0x10)
    #define ADC_STATUS_REF_UVN_BITOFFSET						(4)
    #define REF_UVN_FAULTOCCURRED								((uint8_t) 0x00)
    #define REF_UVN_NOFAULTOCCURRED								((uint8_t) 0x10)    // DEFAULT


/* Register 0x06 (DIGITAL_STATUS) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     MEM_FAULTn     |      RESERVED      |                                           CRC_FAULT_PAGE[4:0]                                          | REG_MAP_CRC_FAULTn |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DIGITAL_STATUS register */
    #define DIGITAL_STATUS_ADDRESS								((uint8_t) 0x06)
    #define DIGITAL_STATUS_DEFAULT								((uint8_t) 0xFF)

    /* MEM_FAULTn field */
    #define MEM_FAULTN_MASK										((uint8_t) 0x80)
    #define DIGITAL_STATUS_MEM_FAULTN_BITOFFSET					(7)
    #define MEM_FAULTN_FAULTOCCURRED							((uint8_t) 0x00)
    #define MEM_FAULTN_NOFAULTOCCURRED							((uint8_t) 0x80)    // DEFAULT

    /* CRC_FAULT_PAGE field */
    #define CRC_FAULT_PAGE_MASK									((uint8_t) 0x3E)
    #define DIGITAL_STATUS_CRC_FAULT_PAGE_BITOFFSET				(1)

    /* REG_MAP_CRC_FAULTn field */
    #define REG_MAP_CRC_FAULTN_MASK								((uint8_t) 0x01)
    #define DIGITAL_STATUS_REG_MAP_CRC_FAULTN_BITOFFSET			(0)
    #define REG_MAP_CRC_FAULTN_FAULTOCCURRED					((uint8_t) 0x00)
    #define REG_MAP_CRC_FAULTN_NOFAULTOCCURRED					((uint8_t) 0x01)    // DEFAULT


/* Register 0x07 (COMP_ALERT_STATUS) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                         RESERVED[2:0]                        |                                     COMP_ALERT_STEP_INDICATOR[4:0]                                     |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* COMP_ALERT_STATUS register */
    #define COMP_ALERT_STATUS_ADDRESS							((uint8_t) 0x07)
    #define COMP_ALERT_STATUS_DEFAULT							((uint8_t) 0xFF)

    /* COMP_ALERT_STEP_INDICATOR field */
    #define COMP_ALERT_STEP_INDICATOR_MASK						((uint8_t) 0x1F)
    #define COMP_ALERT_STATUS_COMP_ALERT_STEP_INDICATOR_BITOFFSET	(0)


/* Register 0x08 (GPIO_DATA_INPUT) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |    GPIO7_DAT_IN    |    GPIO6_DAT_IN    |    GPIO5_DAT_IN    |    GPIO4_DAT_IN    |    GPIO3_DAT_IN    |    GPIO2_DAT_IN    |    GPIO1_DAT_IN    |    GPIO0_DAT_IN    |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPIO_DATA_INPUT register */
    #define GPIO_DATA_INPUT_ADDRESS								((uint8_t) 0x08)
    #define GPIO_DATA_INPUT_DEFAULT								((uint8_t) 0x00)

    /* GPIO7_DAT_IN field */
    #define GPIO7_DAT_IN_MASK									((uint8_t) 0x80)
    #define GPIO_DATA_INPUT_GPIO7_DAT_IN_BITOFFSET				(7)
    #define GPIO7_DAT_IN_LOW									((uint8_t) 0x00)    // DEFAULT
    #define GPIO7_DAT_IN_HIGH									((uint8_t) 0x80)

    /* GPIO6_DAT_IN field */
    #define GPIO6_DAT_IN_MASK									((uint8_t) 0x40)
    #define GPIO_DATA_INPUT_GPIO6_DAT_IN_BITOFFSET				(6)
    #define GPIO6_DAT_IN_LOW									((uint8_t) 0x00)    // DEFAULT
    #define GPIO6_DAT_IN_HIGH									((uint8_t) 0x40)

    /* GPIO5_DAT_IN field */
    #define GPIO5_DAT_IN_MASK									((uint8_t) 0x20)
    #define GPIO_DATA_INPUT_GPIO5_DAT_IN_BITOFFSET				(5)
    #define GPIO5_DAT_IN_LOW									((uint8_t) 0x00)    // DEFAULT
    #define GPIO5_DAT_IN_HIGH									((uint8_t) 0x20)

    /* GPIO4_DAT_IN field */
    #define GPIO4_DAT_IN_MASK									((uint8_t) 0x10)
    #define GPIO_DATA_INPUT_GPIO4_DAT_IN_BITOFFSET				(4)
    #define GPIO4_DAT_IN_LOW									((uint8_t) 0x00)    // DEFAULT
    #define GPIO4_DAT_IN_HIGH									((uint8_t) 0x10)

    /* GPIO3_DAT_IN field */
    #define GPIO3_DAT_IN_MASK									((uint8_t) 0x08)
    #define GPIO_DATA_INPUT_GPIO3_DAT_IN_BITOFFSET				(3)
    #define GPIO3_DAT_IN_LOW									((uint8_t) 0x00)    // DEFAULT
    #define GPIO3_DAT_IN_HIGH									((uint8_t) 0x08)

    /* GPIO2_DAT_IN field */
    #define GPIO2_DAT_IN_MASK									((uint8_t) 0x04)
    #define GPIO_DATA_INPUT_GPIO2_DAT_IN_BITOFFSET				(2)
    #define GPIO2_DAT_IN_LOW									((uint8_t) 0x00)    // DEFAULT
    #define GPIO2_DAT_IN_HIGH									((uint8_t) 0x04)

    /* GPIO1_DAT_IN field */
    #define GPIO1_DAT_IN_MASK									((uint8_t) 0x02)
    #define GPIO_DATA_INPUT_GPIO1_DAT_IN_BITOFFSET				(1)
    #define GPIO1_DAT_IN_LOW									((uint8_t) 0x00)    // DEFAULT
    #define GPIO1_DAT_IN_HIGH									((uint8_t) 0x02)

    /* GPIO0_DAT_IN field */
    #define GPIO0_DAT_IN_MASK									((uint8_t) 0x01)
    #define GPIO_DATA_INPUT_GPIO0_DAT_IN_BITOFFSET				(0)
    #define GPIO0_DAT_IN_LOW									((uint8_t) 0x00)    // DEFAULT
    #define GPIO0_DAT_IN_HIGH									((uint8_t) 0x01)


/* Register 0x09 (SEQ_STATUS) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     SEQ_ACTIVE     |                                   SEQ_COUNT[3:0]                                  |                         RESERVED[2:0]                        |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SEQ_STATUS register */
    #define SEQ_STATUS_ADDRESS									((uint8_t) 0x09)
    #define SEQ_STATUS_DEFAULT									((uint8_t) 0x07)

    /* SEQ_ACTIVE field */
    #define SEQ_ACTIVE_MASK										((uint8_t) 0x80)
    #define SEQ_STATUS_SEQ_ACTIVE_BITOFFSET						(7)
    #define SEQ_ACTIVE_SEQUENCERNOTACTIVE						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_ACTIVE_SEQUENCERACTIVE							((uint8_t) 0x80)

    /* SEQ_COUNT field */
    #define SEQ_COUNT_MASK										((uint8_t) 0x78)
    #define SEQ_STATUS_SEQ_COUNT_BITOFFSET						(3)


/* Register 0x0A (RESERVED) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                   RESERVED[3:0]                                   |                                   RESERVED[3:0]                                   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */



/* Register 0x0B (CONVERSION_CTRL) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        START       |                                             STEP_INIT[4:0]                                             |      RESERVED      |        STOP        |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CONVERSION_CTRL register */
    #define CONVERSION_CTRL_ADDRESS								((uint8_t) 0x0B)
    #define CONVERSION_CTRL_DEFAULT								((uint8_t) 0x00)

    /* START field */
    #define START_MASK											((uint8_t) 0x80)
    #define CONVERSION_CTRL_START_BITOFFSET						(7)
    #define START_NOOPERATION									((uint8_t) 0x00)    // DEFAULT
    #define START_STARTORRESTARTCONVERSIONS						((uint8_t) 0x80)

    /* STEP_INIT field */
    #define STEP_INIT_MASK										((uint8_t) 0x7C)
    #define CONVERSION_CTRL_STEP_INIT_BITOFFSET					(2)

    /* STOP field */
    #define STOP_MASK											((uint8_t) 0x01)
    #define CONVERSION_CTRL_STOP_BITOFFSET						(0)
    #define STOP_NOOPERATION									((uint8_t) 0x00)    // DEFAULT
    #define STOP_STOPCONVERSIONS								((uint8_t) 0x01)


/* Register 0x0C (RESET) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                            RESET_CODE[7:0]                                                                            |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RESET register */
    #define RESET_ADDRESS										((uint8_t) 0x0C)
    #define RESET_DEFAULT										((uint8_t) 0x00)
   

    /* RESET_CODE field */
    #define RESET_CODE_MASK										((uint8_t) 0xFF)
    #define RESET_RESET_CODE_BITOFFSET							(0)
    #define RESET_CODE                                          ((uint8_t) 0x5A)

/* Register 0x10 (ADC_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |      RESERVED      |       REF_VAL      |       CLK_SEL      |        GC_EN       |             SPEED_MODE[1:0]             |      STBY_MODE     |        PWDN        |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* ADC_CFG register */
    #define ADC_CFG_ADDRESS										((uint8_t) 0x10)
    #define ADC_CFG_DEFAULT										((uint8_t) 0x00)

    /* REF_VAL field */
    #define REF_VAL_MASK										((uint8_t) 0x40)
    #define ADC_CFG_REF_VAL_BITOFFSET							(6)
    #define REF_VAL_1											((uint8_t) 0x00)    // DEFAULT
    #define REF_VAL_2											((uint8_t) 0x40)

    /* CLK_SEL field */
    #define CLK_SEL_MASK										((uint8_t) 0x20)
    #define ADC_CFG_CLK_SEL_BITOFFSET							(5)
    #define CLK_SEL_INTERNALOSCILLATOR							((uint8_t) 0x00)    // DEFAULT
    #define CLK_SEL_EXTERNALCLOCK								((uint8_t) 0x20)

    /* GC_EN field */
    #define GC_EN_MASK											((uint8_t) 0x10)
    #define ADC_CFG_GC_EN_BITOFFSET								(4)
    #define GC_EN_DISABLED										((uint8_t) 0x00)    // DEFAULT
    #define GC_EN_ENABLED										((uint8_t) 0x10)

    /* SPEED_MODE field */
    #define SPEED_MODE_MASK										((uint8_t) 0x0C)
    #define ADC_CFG_SPEED_MODE_BITOFFSET						(2)
    #define SPEED_MODE_SPEEDMODE0								((uint8_t) 0x00)    // DEFAULT
    #define SPEED_MODE_SPEEDMODE1								((uint8_t) 0x04)
    #define SPEED_MODE_SPEEDMODE2								((uint8_t) 0x08)
    #define SPEED_MODE_SPEEDMODE3								((uint8_t) 0x0C)

    /* STBY_MODE field */
    #define STBY_MODE_MASK										((uint8_t) 0x02)
    #define ADC_CFG_STBY_MODE_BITOFFSET							(1)
    #define STBY_MODE_STBY_MODE_IDLE							((uint8_t) 0x00)    // DEFAULT
    #define STBY_MODE_STBY_MODE_STANDBY							((uint8_t) 0x02)

    /* PWDN field */
    #define PWDN_MASK											((uint8_t) 0x01)
    #define ADC_CFG_PWDN_BITOFFSET								(0)
    #define PWDN_ACTIVEMODE										((uint8_t) 0x00)    // DEFAULT
    #define PWDN_POWERDOWNMODE									((uint8_t) 0x01)


/* Register 0x11 (DIGITAL_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                               SPARE[4:0]                                               |      STATUS_EN     |      SDO_MODE      |    CONT_READ_EN    |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DIGITAL_CFG register */
    #define DIGITAL_CFG_ADDRESS									((uint8_t) 0x11)
    #define DIGITAL_CFG_DEFAULT									((uint8_t) 0x00)

    /* SPARE field */
    #define DIGITAL_CFG_SPARE_MASK											((uint8_t) 0xF8)
    #define DIGITAL_CFG_SPARE_BITOFFSET							(3)

    /* STATUS_EN field */
    #define STATUS_EN_MASK										((uint8_t) 0x04)
    #define DIGITAL_CFG_STATUS_EN_BITOFFSET						(2)
    #define STATUS_EN_DISABLED									((uint8_t) 0x00)    // DEFAULT
    #define STATUS_EN_ENABLED									((uint8_t) 0x04)

    /* SDO_MODE field */
    #define SDO_MODE_MASK										((uint8_t) 0x02)
    #define DIGITAL_CFG_SDO_MODE_BITOFFSET						(1)
    #define SDO_MODE_DATAOUTPUTONLYMODE							((uint8_t) 0x00)    // DEFAULT
    #define SDO_MODE_DUALMODEDATAOUTPUTANDDATAREADY				((uint8_t) 0x02)

    /* CONT_READ_EN field */
    #define CONT_READ_EN_MASK									((uint8_t) 0x01)
    #define DIGITAL_CFG_CONT_READ_EN_BITOFFSET					(0)
    #define CONT_READ_EN_DISABLED								((uint8_t) 0x00)    // DEFAULT
    #define CONT_READ_EN_ENABLED								((uint8_t) 0x01)


/* Register 0x12 (VBIAS) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |     VBIAS_AIN7     |     VBIAS_AIN6     |     VBIAS_AIN5     |     VBIAS_AIN4     |     VBIAS_AIN3     |     VBIAS_AIN2     |     VBIAS_AIN1     |     VBIAS_AIN0     |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* VBIAS register */
    #define VBIAS_ADDRESS										((uint8_t) 0x12)
    #define VBIAS_DEFAULT										((uint8_t) 0x00)

    /* VBIAS_AIN7 field */
    #define VBIAS_AIN7_MASK										((uint8_t) 0x80)
    #define VBIAS_VBIAS_AIN7_BITOFFSET							(7)
    #define VBIAS_AIN7_DISABLED									((uint8_t) 0x00)    // DEFAULT
    #define VBIAS_AIN7_ENABLED									((uint8_t) 0x80)

    /* VBIAS_AIN6 field */
    #define VBIAS_AIN6_MASK										((uint8_t) 0x40)
    #define VBIAS_VBIAS_AIN6_BITOFFSET							(6)
    #define VBIAS_AIN6_DISABLED									((uint8_t) 0x00)    // DEFAULT
    #define VBIAS_AIN6_ENABLED									((uint8_t) 0x40)

    /* VBIAS_AIN5 field */
    #define VBIAS_AIN5_MASK										((uint8_t) 0x20)
    #define VBIAS_VBIAS_AIN5_BITOFFSET							(5)
    #define VBIAS_AIN5_DISABLED									((uint8_t) 0x00)    // DEFAULT
    #define VBIAS_AIN5_ENABLED									((uint8_t) 0x20)

    /* VBIAS_AIN4 field */
    #define VBIAS_AIN4_MASK										((uint8_t) 0x10)
    #define VBIAS_VBIAS_AIN4_BITOFFSET							(4)
    #define VBIAS_AIN4_DISABLED									((uint8_t) 0x00)    // DEFAULT
    #define VBIAS_AIN4_ENABLED									((uint8_t) 0x10)

    /* VBIAS_AIN3 field */
    #define VBIAS_AIN3_MASK										((uint8_t) 0x08)
    #define VBIAS_VBIAS_AIN3_BITOFFSET							(3)
    #define VBIAS_AIN3_DISABLED									((uint8_t) 0x00)    // DEFAULT
    #define VBIAS_AIN3_ENABLED									((uint8_t) 0x08)

    /* VBIAS_AIN2 field */
    #define VBIAS_AIN2_MASK										((uint8_t) 0x04)
    #define VBIAS_VBIAS_AIN2_BITOFFSET							(2)
    #define VBIAS_AIN2_DISABLED									((uint8_t) 0x00)    // DEFAULT
    #define VBIAS_AIN2_ENABLED									((uint8_t) 0x04)

    /* VBIAS_AIN1 field */
    #define VBIAS_AIN1_MASK										((uint8_t) 0x02)
    #define VBIAS_VBIAS_AIN1_BITOFFSET							(1)
    #define VBIAS_AIN1_DISABLED									((uint8_t) 0x00)    // DEFAULT
    #define VBIAS_AIN1_ENABLED									((uint8_t) 0x02)

    /* VBIAS_AIN0 field */
    #define VBIAS_AIN0_MASK										((uint8_t) 0x01)
    #define VBIAS_VBIAS_AIN0_BITOFFSET							(0)
    #define VBIAS_AIN0_DISABLED									((uint8_t) 0x00)    // DEFAULT
    #define VBIAS_AIN0_ENABLED									((uint8_t) 0x01)


/* Register 0x13 (GPIO_CFG0) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |              GPIO3_CFG[1:0]             |              GPIO2_CFG[1:0]             |              GPIO1_CFG[1:0]             |              GPIO0_CFG[1:0]             |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPIO_CFG0 register */
    #define GPIO_CFG0_ADDRESS									((uint8_t) 0x13)
    #define GPIO_CFG0_DEFAULT									((uint8_t) 0x00)

    /* GPIO3_CFG field */
    #define GPIO3_CFG_MASK										((uint8_t) 0xC0)
    #define GPIO_CFG0_GPIO3_CFG_BITOFFSET						(6)
    #define GPIO3_CFG_ADS1X4S18PINOPERATESASANALOGINPUT			((uint8_t) 0x00)    // DEFAULT
    #define GPIO3_CFG_DIGITALINPUT								((uint8_t) 0x40)
    #define GPIO3_CFG_PUSHPULLDIGITALOUTPUT						((uint8_t) 0x80)
    #define GPIO3_CFG_OPENDRAINDIGITALOUTPUT					((uint8_t) 0xC0)

    /* GPIO2_CFG field */
    #define GPIO2_CFG_MASK										((uint8_t) 0x30)
    #define GPIO_CFG0_GPIO2_CFG_BITOFFSET						(4)
    #define GPIO2_CFG_ADS1X4S18PINOPERATESASANALOGINPUT			((uint8_t) 0x00)    // DEFAULT
    #define GPIO2_CFG_DIGITALINPUT								((uint8_t) 0x10)
    #define GPIO2_CFG_PUSHPULLDIGITALOUTPUT						((uint8_t) 0x20)
    #define GPIO2_CFG_OPENDRAINDIGITALOUTPUT					((uint8_t) 0x30)

    /* GPIO1_CFG field */
    #define GPIO1_CFG_MASK										((uint8_t) 0x0C)
    #define GPIO_CFG0_GPIO1_CFG_BITOFFSET						(2)
    #define GPIO1_CFG_ADS1X4S18PINOPERATESASANALOGINPUT			((uint8_t) 0x00)    // DEFAULT
    #define GPIO1_CFG_DIGITALINPUT								((uint8_t) 0x04)
    #define GPIO1_CFG_PUSHPULLDIGITALOUTPUT						((uint8_t) 0x08)
    #define GPIO1_CFG_OPENDRAINDIGITALOUTPUT					((uint8_t) 0x0C)

    /* GPIO0_CFG field */
    #define GPIO0_CFG_MASK										((uint8_t) 0x03)
    #define GPIO_CFG0_GPIO0_CFG_BITOFFSET						(0)
    #define GPIO0_CFG_ADS1X4S18PINOPERATESASANALOGINPUT			((uint8_t) 0x00)    // DEFAULT
    #define GPIO0_CFG_DIGITALINPUT								((uint8_t) 0x01)
    #define GPIO0_CFG_PUSHPULLDIGITALOUTPUT						((uint8_t) 0x02)
    #define GPIO0_CFG_OPENDRAINDIGITALOUTPUT					((uint8_t) 0x03)


/* Register 0x14 (GPIO_CFG1) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |              GPIO7_CFG[1:0]             |              GPIO6_CFG[1:0]             |              GPIO5_CFG[1:0]             |              GPIO4_CFG[1:0]             |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPIO_CFG1 register */
    #define GPIO_CFG1_ADDRESS									((uint8_t) 0x14)
    #define GPIO_CFG1_DEFAULT									((uint8_t) 0x00)

    /* GPIO7_CFG field */
    #define GPIO7_CFG_MASK										((uint8_t) 0xC0)
    #define GPIO_CFG1_GPIO7_CFG_BITOFFSET						(6)
    #define GPIO7_CFG_ADS1X4S18PINOPERATESASANALOGORREFN1INPUT	((uint8_t) 0x00)    // DEFAULT
    #define GPIO7_CFG_DIGITALINPUT								((uint8_t) 0x40)
    #define GPIO7_CFG_PUSHPULLDIGITALOUTPUT						((uint8_t) 0x80)
    #define GPIO7_CFG_OPENDRAINDIGITALOUTPUT					((uint8_t) 0xC0)

    /* GPIO6_CFG field */
    #define GPIO6_CFG_MASK										((uint8_t) 0x30)
    #define GPIO_CFG1_GPIO6_CFG_BITOFFSET						(4)
    #define GPIO6_CFG_ADS1X4S18PINOPERATESASANALOGORREFP1INPUT	((uint8_t) 0x00)    // DEFAULT
    #define GPIO6_CFG_DIGITALINPUT								((uint8_t) 0x10)
    #define GPIO6_CFG_PUSHPULLDIGITALOUTPUT						((uint8_t) 0x20)
    #define GPIO6_CFG_OPENDRAINDIGITALOUTPUT					((uint8_t) 0x30)

    /* GPIO5_CFG field */
    #define GPIO5_CFG_MASK										((uint8_t) 0x0C)
    #define GPIO_CFG1_GPIO5_CFG_BITOFFSET						(2)
    #define GPIO5_CFG_ADS1X4S18PINOPERATESASANALOGINPUT			((uint8_t) 0x00)    // DEFAULT
    #define GPIO5_CFG_DIGITALINPUT								((uint8_t) 0x04)
    #define GPIO5_CFG_PUSHPULLDIGITALOUTPUT						((uint8_t) 0x08)
    #define GPIO5_CFG_OPENDRAINDIGITALOUTPUT					((uint8_t) 0x0C)

    /* GPIO4_CFG field */
    #define GPIO4_CFG_MASK										((uint8_t) 0x03)
    #define GPIO_CFG1_GPIO4_CFG_BITOFFSET						(0)
    #define GPIO4_CFG_ADS1X4S18PINOPERATESASANALOGINPUT			((uint8_t) 0x00)    // DEFAULT
    #define GPIO4_CFG_DIGITALINPUT								((uint8_t) 0x01)
    #define GPIO4_CFG_PUSHPULLDIGITALOUTPUT						((uint8_t) 0x02)
    #define GPIO4_CFG_OPENDRAINDIGITALOUTPUT					((uint8_t) 0x03)


/* Register 0x15 (GPIO_CFG2) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                         RESERVED[2:0]                        |  COMP_ALERTn_MASK  | FAULT_PIN_BEHAVIOR |    ALERT_PIN_POL   |      GPIO4_SRC     |      GPIO3_SRC     |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPIO_CFG2 register */
    #define GPIO_CFG2_ADDRESS									((uint8_t) 0x15)
    #define GPIO_CFG2_DEFAULT									((uint8_t) 0x08)

    /* COMP_ALERTn_MASK field */
    #define COMP_ALERTN_MASK_MASK								((uint8_t) 0x10)
    #define GPIO_CFG2_COMP_ALERTN_MASK_BITOFFSET				(4)
    #define COMP_ALERTN_MASK_MASKDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define COMP_ALERTN_MASK_MASKENABLED						((uint8_t) 0x10)

    /* FAULT_PIN_BEHAVIOR field */
    #define FAULT_PIN_BEHAVIOR_MASK								((uint8_t) 0x08)
    #define GPIO_CFG2_FAULT_PIN_BEHAVIOR_BITOFFSET				(3)
    #define FAULT_PIN_BEHAVIOR_STATIC							((uint8_t) 0x00)
    #define FAULT_PIN_BEHAVIOR_HEARTBEAT						((uint8_t) 0x08)    // DEFAULT

    /* ALERT_PIN_POL field */
    #define ALERT_PIN_POL_MASK									((uint8_t) 0x04)
    #define GPIO_CFG2_ALERT_PIN_POL_BITOFFSET					(2)
    #define ALERT_PIN_POL_ACTIVELOW								((uint8_t) 0x00)    // DEFAULT
    #define ALERT_PIN_POL_ACTIVEHIGH							((uint8_t) 0x04)

    /* GPIO4_SRC field */
    #define GPIO4_SRC_MASK										((uint8_t) 0x02)
    #define GPIO_CFG2_GPIO4_SRC_BITOFFSET						(1)
    #define GPIO4_SRC_GPIO4_DAT_OUTBIT							((uint8_t) 0x00)    // DEFAULT
    #define GPIO4_SRC_SAD_FAULTNFLAG							((uint8_t) 0x02)

    /* GPIO3_SRC field */
    #define GPIO3_SRC_MASK										((uint8_t) 0x01)
    #define GPIO_CFG2_GPIO3_SRC_BITOFFSET						(0)
    #define GPIO3_SRC_GPIO3_DAT_OUTBIT							((uint8_t) 0x00)    // DEFAULT
    #define GPIO3_SRC_COMP_ALERTNFLAG							((uint8_t) 0x01)


/* Register 0x16 (MONITOR_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                         RESERVED[2:0]                        |      RESERVED      |      REF_UV_EN     |     REF_UV_SEL     |   REG_MAP_CRC_EN   |     SPI_CRC_EN     |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MONITOR_CFG register */
    #define MONITOR_CFG_ADDRESS									((uint8_t) 0x16)
    #define MONITOR_CFG_DEFAULT									((uint8_t) 0x04)

    /* REF_UV_EN field */
    #define REF_UV_EN_MASK										((uint8_t) 0x08)
    #define MONITOR_CFG_REF_UV_EN_BITOFFSET						(3)
    #define REF_UV_EN_DISABLED									((uint8_t) 0x00)    // DEFAULT
    #define REF_UV_EN_ENABLED									((uint8_t) 0x08)

    /* REF_UV_SEL field */
    #define REF_UV_SEL_MASK										((uint8_t) 0x04)
    #define MONITOR_CFG_REF_UV_SEL_BITOFFSET					(2)
    #define REF_UV_SEL_550MV									((uint8_t) 0x00)
    #define REF_UV_SEL_1V										((uint8_t) 0x04)    // DEFAULT

    /* REG_MAP_CRC_EN field */
    #define REG_MAP_CRC_EN_MASK									((uint8_t) 0x02)
    #define MONITOR_CFG_REG_MAP_CRC_EN_BITOFFSET				(1)
    #define REG_MAP_CRC_EN_DISABLED								((uint8_t) 0x00)    // DEFAULT
    #define REG_MAP_CRC_EN_ENABLED								((uint8_t) 0x02)

    /* SPI_CRC_EN field */
    #define SPI_CRC_EN_MASK										((uint8_t) 0x01)
    #define MONITOR_CFG_SPI_CRC_EN_BITOFFSET					(0)
    #define SPI_CRC_EN_DISABLED									((uint8_t) 0x00)    // DEFAULT
    #define SPI_CRC_EN_ENABLED									((uint8_t) 0x01)


/* Register 0x17 (RESERVED) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                         RESERVED[2:0]                        |              RESERVED[1:0]              |                         RESERVED[2:0]                        |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */


/* Register 0x18 (SPARE) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                               SPARE[7:0]                                                                              |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SPARE register */
    #define SPARE_ADDRESS										((uint8_t) 0x18)
    #define SPARE_DEFAULT										((uint8_t) 0x00)



/* Register 0x19 (SEQUENCER_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |              SEQ_MODE[1:0]              |            STOP_BEHAVIOR[1:0]           |              RESERVED[1:0]              |              RESERVED[1:0]              |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SEQUENCER_CFG register */
    #define SEQUENCER_CFG_ADDRESS								((uint8_t) 0x19)
    #define SEQUENCER_CFG_DEFAULT								((uint8_t) 0x50)

    /* SEQ_MODE field */
    #define SEQ_MODE_MASK										((uint8_t) 0xC0)
    #define SEQUENCER_CFG_SEQ_MODE_BITOFFSET					(6)
    #define SEQ_MODE_SEQ_MODE_SEQUENCERDISABLEDSTEPEXECUTEDONLYONCE	((uint8_t) 0x00)
    #define SEQ_MODE_SEQ_MODE_SEQUENCERDISABLEDSTEPEXECUTEDINDEFINITELY	((uint8_t) 0x40)    // DEFAULT
    #define SEQ_MODE_SEQ_MODE_SEQUENCERENABLEDONETIME			((uint8_t) 0x80)
    #define SEQ_MODE_SEQ_MODE_SEQUENCERENABLEDCONTINUOUS		((uint8_t) 0xC0)

    /* STOP_BEHAVIOR field */
    #define STOP_BEHAVIOR_MASK									((uint8_t) 0x30)
    #define SEQUENCER_CFG_STOP_BEHAVIOR_BITOFFSET				(4)
    #define STOP_BEHAVIOR_STOPIMMEDIATELY						((uint8_t) 0x00)
    #define STOP_BEHAVIOR_STOPAFTERCURRENTCONVERSIONCOMPLETES	((uint8_t) 0x10)    // DEFAULT
    #define STOP_BEHAVIOR_STOPAFTERCURRENTSEQUENCESTEPCOMPLETES	((uint8_t) 0x20)
    #define STOP_BEHAVIOR_STOPAFTERFULLSEQUENCECOMPLETES		((uint8_t) 0x30)


/* Register 0x1A (SEQUENCE_STEP_EN_0) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |    SEQ_STEP_7_EN   |    SEQ_STEP_6_EN   |    SEQ_STEP_5_EN   |    SEQ_STEP_4_EN   |    SEQ_STEP_3_EN   |    SEQ_STEP_2_EN   |    SEQ_STEP_1_EN   |    SEQ_STEP_0_EN   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SEQUENCE_STEP_EN_0 register */
    #define SEQUENCE_STEP_EN_0_ADDRESS							((uint8_t) 0x1A)
    #define SEQUENCE_STEP_EN_0_DEFAULT							((uint8_t) 0x01)

    /* SEQ_STEP_7_EN field */
    #define SEQ_STEP_7_EN_MASK									((uint8_t) 0x80)
    #define SEQUENCE_STEP_EN_0_SEQ_STEP_7_EN_BITOFFSET			(7)
    #define SEQ_STEP_7_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_7_EN_STEPENABLED							((uint8_t) 0x80)

    /* SEQ_STEP_6_EN field */
    #define SEQ_STEP_6_EN_MASK									((uint8_t) 0x40)
    #define SEQUENCE_STEP_EN_0_SEQ_STEP_6_EN_BITOFFSET			(6)
    #define SEQ_STEP_6_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_6_EN_STEPENABLED							((uint8_t) 0x40)

    /* SEQ_STEP_5_EN field */
    #define SEQ_STEP_5_EN_MASK									((uint8_t) 0x20)
    #define SEQUENCE_STEP_EN_0_SEQ_STEP_5_EN_BITOFFSET			(5)
    #define SEQ_STEP_5_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_5_EN_STEPENABLED							((uint8_t) 0x20)

    /* SEQ_STEP_4_EN field */
    #define SEQ_STEP_4_EN_MASK									((uint8_t) 0x10)
    #define SEQUENCE_STEP_EN_0_SEQ_STEP_4_EN_BITOFFSET			(4)
    #define SEQ_STEP_4_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_4_EN_STEPENABLED							((uint8_t) 0x10)

    /* SEQ_STEP_3_EN field */
    #define SEQ_STEP_3_EN_MASK									((uint8_t) 0x08)
    #define SEQUENCE_STEP_EN_0_SEQ_STEP_3_EN_BITOFFSET			(3)
    #define SEQ_STEP_3_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_3_EN_STEPENABLED							((uint8_t) 0x08)

    /* SEQ_STEP_2_EN field */
    #define SEQ_STEP_2_EN_MASK									((uint8_t) 0x04)
    #define SEQUENCE_STEP_EN_0_SEQ_STEP_2_EN_BITOFFSET			(2)
    #define SEQ_STEP_2_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_2_EN_STEPENABLED							((uint8_t) 0x04)

    /* SEQ_STEP_1_EN field */
    #define SEQ_STEP_1_EN_MASK									((uint8_t) 0x02)
    #define SEQUENCE_STEP_EN_0_SEQ_STEP_1_EN_BITOFFSET			(1)
    #define SEQ_STEP_1_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_1_EN_STEPENABLED							((uint8_t) 0x02)

    /* SEQ_STEP_0_EN field */
    #define SEQ_STEP_0_EN_MASK									((uint8_t) 0x01)
    #define SEQUENCE_STEP_EN_0_SEQ_STEP_0_EN_BITOFFSET			(0)
    #define SEQ_STEP_0_EN_STEPDISABLED							((uint8_t) 0x00)
    #define SEQ_STEP_0_EN_STEPENABLED							((uint8_t) 0x01)    // DEFAULT


/* Register 0x1B (SEQUENCE_STEP_EN_1) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |   SEQ_STEP_15_EN   |   SEQ_STEP_14_EN   |   SEQ_STEP_13_EN   |   SEQ_STEP_12_EN   |   SEQ_STEP_11_EN   |   SEQ_STEP_10_EN   |    SEQ_STEP_9_EN   |    SEQ_STEP_8_EN   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SEQUENCE_STEP_EN_1 register */
    #define SEQUENCE_STEP_EN_1_ADDRESS							((uint8_t) 0x1B)
    #define SEQUENCE_STEP_EN_1_DEFAULT							((uint8_t) 0x00)

    /* SEQ_STEP_15_EN field */
    #define SEQ_STEP_15_EN_MASK									((uint8_t) 0x80)
    #define SEQUENCE_STEP_EN_1_SEQ_STEP_15_EN_BITOFFSET			(7)
    #define SEQ_STEP_15_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_15_EN_STEPENABLED							((uint8_t) 0x80)

    /* SEQ_STEP_14_EN field */
    #define SEQ_STEP_14_EN_MASK									((uint8_t) 0x40)
    #define SEQUENCE_STEP_EN_1_SEQ_STEP_14_EN_BITOFFSET			(6)
    #define SEQ_STEP_14_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_14_EN_STEPENABLED							((uint8_t) 0x40)

    /* SEQ_STEP_13_EN field */
    #define SEQ_STEP_13_EN_MASK									((uint8_t) 0x20)
    #define SEQUENCE_STEP_EN_1_SEQ_STEP_13_EN_BITOFFSET			(5)
    #define SEQ_STEP_13_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_13_EN_STEPENABLED							((uint8_t) 0x20)

    /* SEQ_STEP_12_EN field */
    #define SEQ_STEP_12_EN_MASK									((uint8_t) 0x10)
    #define SEQUENCE_STEP_EN_1_SEQ_STEP_12_EN_BITOFFSET			(4)
    #define SEQ_STEP_12_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_12_EN_STEPENABLED							((uint8_t) 0x10)

    /* SEQ_STEP_11_EN field */
    #define SEQ_STEP_11_EN_MASK									((uint8_t) 0x08)
    #define SEQUENCE_STEP_EN_1_SEQ_STEP_11_EN_BITOFFSET			(3)
    #define SEQ_STEP_11_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_11_EN_STEPENABLED							((uint8_t) 0x08)

    /* SEQ_STEP_10_EN field */
    #define SEQ_STEP_10_EN_MASK									((uint8_t) 0x04)
    #define SEQUENCE_STEP_EN_1_SEQ_STEP_10_EN_BITOFFSET			(2)
    #define SEQ_STEP_10_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_10_EN_STEPENABLED							((uint8_t) 0x04)

    /* SEQ_STEP_9_EN field */
    #define SEQ_STEP_9_EN_MASK									((uint8_t) 0x02)
    #define SEQUENCE_STEP_EN_1_SEQ_STEP_9_EN_BITOFFSET			(1)
    #define SEQ_STEP_9_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_9_EN_STEPENABLED							((uint8_t) 0x02)

    /* SEQ_STEP_8_EN field */
    #define SEQ_STEP_8_EN_MASK									((uint8_t) 0x01)
    #define SEQUENCE_STEP_EN_1_SEQ_STEP_8_EN_BITOFFSET			(0)
    #define SEQ_STEP_8_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_8_EN_STEPENABLED							((uint8_t) 0x01)


/* Register 0x1C (SEQUENCE_STEP_EN_2) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |   SEQ_STEP_23_EN   |   SEQ_STEP_22_EN   |   SEQ_STEP_21_EN   |   SEQ_STEP_20_EN   |   SEQ_STEP_19_EN   |   SEQ_STEP_18_EN   |   SEQ_STEP_17_EN   |   SEQ_STEP_16_EN   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SEQUENCE_STEP_EN_2 register */
    #define SEQUENCE_STEP_EN_2_ADDRESS							((uint8_t) 0x1C)
    #define SEQUENCE_STEP_EN_2_DEFAULT							((uint8_t) 0x00)

    /* SEQ_STEP_23_EN field */
    #define SEQ_STEP_23_EN_MASK									((uint8_t) 0x80)
    #define SEQUENCE_STEP_EN_2_SEQ_STEP_23_EN_BITOFFSET			(7)
    #define SEQ_STEP_23_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_23_EN_STEPENABLED							((uint8_t) 0x80)

    /* SEQ_STEP_22_EN field */
    #define SEQ_STEP_22_EN_MASK									((uint8_t) 0x40)
    #define SEQUENCE_STEP_EN_2_SEQ_STEP_22_EN_BITOFFSET			(6)
    #define SEQ_STEP_22_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_22_EN_STEPENABLED							((uint8_t) 0x40)

    /* SEQ_STEP_21_EN field */
    #define SEQ_STEP_21_EN_MASK									((uint8_t) 0x20)
    #define SEQUENCE_STEP_EN_2_SEQ_STEP_21_EN_BITOFFSET			(5)
    #define SEQ_STEP_21_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_21_EN_STEPENABLED							((uint8_t) 0x20)

    /* SEQ_STEP_20_EN field */
    #define SEQ_STEP_20_EN_MASK									((uint8_t) 0x10)
    #define SEQUENCE_STEP_EN_2_SEQ_STEP_20_EN_BITOFFSET			(4)
    #define SEQ_STEP_20_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_20_EN_STEPENABLED							((uint8_t) 0x10)

    /* SEQ_STEP_19_EN field */
    #define SEQ_STEP_19_EN_MASK									((uint8_t) 0x08)
    #define SEQUENCE_STEP_EN_2_SEQ_STEP_19_EN_BITOFFSET			(3)
    #define SEQ_STEP_19_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_19_EN_STEPENABLED							((uint8_t) 0x08)

    /* SEQ_STEP_18_EN field */
    #define SEQ_STEP_18_EN_MASK									((uint8_t) 0x04)
    #define SEQUENCE_STEP_EN_2_SEQ_STEP_18_EN_BITOFFSET			(2)
    #define SEQ_STEP_18_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_18_EN_STEPENABLED							((uint8_t) 0x04)

    /* SEQ_STEP_17_EN field */
    #define SEQ_STEP_17_EN_MASK									((uint8_t) 0x02)
    #define SEQUENCE_STEP_EN_2_SEQ_STEP_17_EN_BITOFFSET			(1)
    #define SEQ_STEP_17_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_17_EN_STEPENABLED							((uint8_t) 0x02)

    /* SEQ_STEP_16_EN field */
    #define SEQ_STEP_16_EN_MASK									((uint8_t) 0x01)
    #define SEQUENCE_STEP_EN_2_SEQ_STEP_16_EN_BITOFFSET			(0)
    #define SEQ_STEP_16_EN_STEPDISABLED							((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_16_EN_STEPENABLED							((uint8_t) 0x01)


/* Register 0x1D (RESERVED) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                   RESERVED[3:0]                                   |                                   RESERVED[3:0]                                   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */



/* Register 0x1E (RESERVED) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                   RESERVED[3:0]                                   |                                   RESERVED[3:0]                                   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */



/* Register 0x1F (RESERVED) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                   RESERVED[3:0]                                   |                                   RESERVED[3:0]                                   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */



/* Register 0x3D (REG_MAP_CRC) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                   GENERAL_CFG_REG_MAP_CRC_VALUE[7:0]                                                                  |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REG_MAP_CRC register */
    #define REG_MAP_CRC_ADDRESS									((uint8_t) 0x3D)
    #define REG_MAP_CRC_DEFAULT									((uint8_t) 0x00)

    /* GENERAL_CFG_REG_MAP_CRC_VALUE field */
    #define GENERAL_CFG_REG_MAP_CRC_VALUE_MASK					((uint8_t) 0xFF)
    #define REG_MAP_CRC_GENERAL_CFG_REG_MAP_CRC_VALUE_BITOFFSET	(0)


/* Register 0x3E (PAGE_INDICATOR) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                          PAGE_INDICATOR[7:0]                                                                          |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PAGE_INDICATOR register */
    #define PAGE_INDICATOR_ADDRESS								((uint8_t) 0x3E)
    #define PAGE_INDICATOR_DEFAULT								((uint8_t) 0x00)

    /* PAGE_INDICATOR field */
    #define PAGE_INDICATOR_MASK									((uint8_t) 0xFF)
    #define PAGE_INDICATOR_BITOFFSET							(0)


/* Register 0x3F (PAGE_POINTER) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                           PAGE_POINTER[7:0]                                                                           |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PAGE_POINTER register */
    #define PAGE_POINTER_ADDRESS								((uint8_t) 0x3F)
    #define PAGE_POINTER_DEFAULT								((uint8_t) 0x00)

    /* PAGE_POINTER field */
    #define PAGE_POINTER_MASK									((uint8_t) 0xFF)
    #define PAGE_POINTER_BITOFFSET								(0)


#endif /* ADS124S18_PAGE0_H_ */

