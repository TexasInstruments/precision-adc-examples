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

#ifndef ADS125P08_PAGE0_H_
#define ADS125P08_PAGE0_H_

#include <stdint.h>
#include <stdbool.h>


#define NUM_REGISTERS                           ((uint8_t) 64)
/** Maximum register address or address of the last register in the regmap */
#define MAX_REGISTER_ADDRESS                    ((uint8_t) 0x3F)


//**********************************************************************************
//
// Register definitions - General settings page
//
//**********************************************************************************


/* Register 0x00 (DEVICE_ID) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |               RESERVED[1:0]               |                CH_CNT[1:0]                |                                      DEV_ID[3:0]                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DEVICE_ID register */
    #define DEVICE_ID_ADDRESS									((uint8_t) 0x00)
    #define DEVICE_ID_DEFAULT									((uint8_t) 0x30)

    /* CH_CNT field */
    #define CH_CNT_MASK											((uint8_t) 0x30)
    #define DEVICE_ID_CH_CNT_BITOFFSET							(4)

    /* DEV_ID field */
    #define DEV_ID_MASK											((uint8_t) 0x0F)
    #define DEVICE_ID_DEV_ID_BITOFFSET							(0)
    #define DEVICE_IS_16_BIT_MASK                     ((uint8_t) 0x04)


/* Register 0x01 (REVISION_ID) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                  REV_ID[7:0]                                                                                  |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REVISION_ID register */
    #define REVISION_ID_ADDRESS									((uint8_t) 0x01)
    #define REVISION_ID_DEFAULT									((uint8_t) 0x01)

    /* REV_ID field */
    #define REV_ID_MASK											((uint8_t) 0xFF)
    #define REVISION_ID_REV_ID_BITOFFSET						(0)


/* Register 0x02 (STATUS_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                             STEP_INDICATOR[4:0]                                             |    ADC_REF_FAULTn   |        RESETn       |         DRDY        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STATUS_MSB register */
    #define STATUS_MSB_ADDRESS									((uint8_t) 0x02)
    #define STATUS_MSB_DEFAULT									((uint8_t) 0x00)

    /* STEP_INDICATOR field */
    #define STEP_INDICATOR_MASK									((uint8_t) 0xF8)
    #define STATUS_MSB_STEP_INDICATOR_BITOFFSET					(3)

    /* ADC_REF_FAULTn field */
    #define ADC_REF_FAULTN_MASK									((uint8_t) 0x04)
    #define STATUS_MSB_ADC_REF_FAULTN_BITOFFSET					(2)
    #define ADC_REF_FAULTN_OUTOFRANGEFAULTOCCURRED				((uint8_t) 0x00)    // DEFAULT
    #define ADC_REF_FAULTN_NOOUTOFRANGEFAULTOCCURRED			((uint8_t) 0x04)

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
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                    CONV_COUNT[3:0]                                    |     FIFO_FAULTn     |   INTERNAL_FAULTn   |   REG_WRITE_FAULTn  |    SPI_CRC_FAULTn   |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STATUS_LSB register */
    #define STATUS_LSB_ADDRESS									((uint8_t) 0x03)
    #define STATUS_LSB_DEFAULT									((uint8_t) 0xFF)

    /* CONV_COUNT field */
    #define CONV_COUNT_MASK										((uint8_t) 0xF0)
    #define STATUS_LSB_CONV_COUNT_BITOFFSET						(4)

    /* FIFO_FAULTn field */
    #define FIFO_FAULTN_MASK									((uint8_t) 0x08)
    #define STATUS_LSB_FIFO_FAULTN_BITOFFSET					(3)
    #define FIFO_FAULTN_FIFOFAULTOCCURRED						((uint8_t) 0x00)
    #define FIFO_FAULTN_NOFIFOFAULTOCCURRED						((uint8_t) 0x08)    // DEFAULT

    /* INTERNAL_FAULTn field */
    #define INTERNAL_FAULTN_MASK								((uint8_t) 0x04)
    #define STATUS_LSB_INTERNAL_FAULTN_BITOFFSET				(2)
    #define INTERNAL_FAULTN_INTERNALFAULTOCCURRED				((uint8_t) 0x00)
    #define INTERNAL_FAULTN_NOINTERNALFAULTOCCURRED				((uint8_t) 0x04)    // DEFAULT

    /* REG_WRITE_FAULTn field */
    #define REG_WRITE_FAULTN_MASK								((uint8_t) 0x02)
    #define STATUS_LSB_REG_WRITE_FAULTN_BITOFFSET				(1)
    #define REG_WRITE_FAULTN_PAGEORREGISTERACCESSFAULTOCCURRED	((uint8_t) 0x00)
    #define REG_WRITE_FAULTN_NOPAGEORREGISTERACCESSFAULTOCCURRED	((uint8_t) 0x02)    // DEFAULT

    /* SPI_CRC_FAULTn field */
    #define SPI_CRC_FAULTN_MASK									((uint8_t) 0x01)
    #define STATUS_LSB_SPI_CRC_FAULTN_BITOFFSET					(0)
    #define SPI_CRC_FAULTN_SPICRCFAULTOCCURRED					((uint8_t) 0x00)
    #define SPI_CRC_FAULTN_NOSPICRCFAULTOCCURRED				((uint8_t) 0x01)    // DEFAULT


/* Register 0x04 (ADC_REF_STATUS) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       RESERVED      |       AVDD_UVn      |       REF_UVn       |    MOD_OVR_FAULTn   |                                     RESERVED[3:0]                                     |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* ADC_REF_STATUS register */
    #define ADC_REF_STATUS_ADDRESS								((uint8_t) 0x04)
    #define ADC_REF_STATUS_DEFAULT								((uint8_t) 0xB0)

    /* AVDD_UVn field */
    #define AVDD_UVN_MASK										((uint8_t) 0x40)
    #define ADC_REF_STATUS_AVDD_UVN_BITOFFSET					(6)
    #define AVDD_UVN_SUPPLYUNDERVOLTAGEFAULTOCCURRED			((uint8_t) 0x00)    // DEFAULT
    #define AVDD_UVN_NOSUPPLYUNDERVOLTAGEFAULTOCCURRED			((uint8_t) 0x40)

    /* REF_UVn field */
    #define REF_UVN_MASK										((uint8_t) 0x20)
    #define ADC_REF_STATUS_REF_UVN_BITOFFSET					(5)
    #define REF_UVN_REFERENCEUNDERVOLTAGEFAULTOCCURRED			((uint8_t) 0x00)
    #define REF_UVN_NOREFERENCEUNDERVOLTAGEFAULTOCCURRED		((uint8_t) 0x20)    // DEFAULT

    /* MOD_OVR_FAULTn field */
    #define MOD_OVR_FAULTN_MASK									((uint8_t) 0x10)
    #define ADC_REF_STATUS_MOD_OVR_FAULTN_BITOFFSET				(4)
    #define MOD_OVR_FAULTN_MODULATOROVERRANGEFAULTOCCURRED		((uint8_t) 0x00)
    #define MOD_OVR_FAULTN_NOMODULATOROVERRANGEFAULTOCCURRED	((uint8_t) 0x10)    // DEFAULT


/* Register 0x05 (DIGITAL_STATUS) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                        CRC_FAULT_PAGE[5:0]                                                        | MEM_INTERNAL_FAULTn |  REG_MAP_CRC_FAULTn |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DIGITAL_STATUS register */
    #define DIGITAL_STATUS_ADDRESS								((uint8_t) 0x05)
    #define DIGITAL_STATUS_DEFAULT								((uint8_t) 0xFF)

    /* CRC_FAULT_PAGE field */
    #define CRC_FAULT_PAGE_MASK									((uint8_t) 0xFC)
    #define DIGITAL_STATUS_CRC_FAULT_PAGE_BITOFFSET				(2)

    /* MEM_INTERNAL_FAULTn field */
    #define MEM_INTERNAL_FAULTN_MASK							((uint8_t) 0x02)
    #define DIGITAL_STATUS_MEM_INTERNAL_FAULTN_BITOFFSET		(1)
    #define MEM_INTERNAL_FAULTN_MEMORYMAPCRCFAULTOCCURRED		((uint8_t) 0x00)
    #define MEM_INTERNAL_FAULTN_NOMEMORYMAPCRCFAULTOCCURRED		((uint8_t) 0x02)    // DEFAULT

    /* REG_MAP_CRC_FAULTn field */
    #define REG_MAP_CRC_FAULTN_MASK								((uint8_t) 0x01)
    #define DIGITAL_STATUS_REG_MAP_CRC_FAULTN_BITOFFSET			(0)
    #define REG_MAP_CRC_FAULTN_REGISTERMAPCRCFAULTOCCURRED		((uint8_t) 0x00)
    #define REG_MAP_CRC_FAULTN_NOREGISTERMAPCRCFAULTOCCURRED	((uint8_t) 0x01)    // DEFAULT


/* Register 0x06 (AGPIO_DATA_INPUT) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |    AGPIO7_DAT_IN    |    AGPIO6_DAT_IN    |    AGPIO5_DAT_IN    |    AGPIO4_DAT_IN    |    AGPIO3_DAT_IN    |    AGPIO2_DAT_IN    |    AGPIO1_DAT_IN    |    AGPIO0_DAT_IN    |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* AGPIO_DATA_INPUT register */
    #define AGPIO_DATA_INPUT_ADDRESS							((uint8_t) 0x06)
    #define AGPIO_DATA_INPUT_DEFAULT							((uint8_t) 0x00)

    /* AGPIO7_DAT_IN field */
    #define AGPIO7_DAT_IN_MASK									((uint8_t) 0x80)
    #define AGPIO_DATA_INPUT_AGPIO7_DAT_IN_BITOFFSET			(7)
    #define AGPIO7_DAT_IN_LOW									((uint8_t) 0x00)    // DEFAULT
    #define AGPIO7_DAT_IN_HIGH									((uint8_t) 0x80)

    /* AGPIO6_DAT_IN field */
    #define AGPIO6_DAT_IN_MASK									((uint8_t) 0x40)
    #define AGPIO_DATA_INPUT_AGPIO6_DAT_IN_BITOFFSET			(6)
    #define AGPIO6_DAT_IN_LOW									((uint8_t) 0x00)    // DEFAULT
    #define AGPIO6_DAT_IN_HIGH									((uint8_t) 0x40)

    /* AGPIO5_DAT_IN field */
    #define AGPIO5_DAT_IN_MASK									((uint8_t) 0x20)
    #define AGPIO_DATA_INPUT_AGPIO5_DAT_IN_BITOFFSET			(5)
    #define AGPIO5_DAT_IN_LOW									((uint8_t) 0x00)    // DEFAULT
    #define AGPIO5_DAT_IN_HIGH									((uint8_t) 0x20)

    /* AGPIO4_DAT_IN field */
    #define AGPIO4_DAT_IN_MASK									((uint8_t) 0x10)
    #define AGPIO_DATA_INPUT_AGPIO4_DAT_IN_BITOFFSET			(4)
    #define AGPIO4_DAT_IN_LOW									((uint8_t) 0x00)    // DEFAULT
    #define AGPIO4_DAT_IN_HIGH									((uint8_t) 0x10)

    /* AGPIO3_DAT_IN field */
    #define AGPIO3_DAT_IN_MASK									((uint8_t) 0x08)
    #define AGPIO_DATA_INPUT_AGPIO3_DAT_IN_BITOFFSET			(3)
    #define AGPIO3_DAT_IN_LOW									((uint8_t) 0x00)    // DEFAULT
    #define AGPIO3_DAT_IN_HIGH									((uint8_t) 0x08)

    /* AGPIO2_DAT_IN field */
    #define AGPIO2_DAT_IN_MASK									((uint8_t) 0x04)
    #define AGPIO_DATA_INPUT_AGPIO2_DAT_IN_BITOFFSET			(2)
    #define AGPIO2_DAT_IN_LOW									((uint8_t) 0x00)    // DEFAULT
    #define AGPIO2_DAT_IN_HIGH									((uint8_t) 0x04)

    /* AGPIO1_DAT_IN field */
    #define AGPIO1_DAT_IN_MASK									((uint8_t) 0x02)
    #define AGPIO_DATA_INPUT_AGPIO1_DAT_IN_BITOFFSET			(1)
    #define AGPIO1_DAT_IN_LOW									((uint8_t) 0x00)    // DEFAULT
    #define AGPIO1_DAT_IN_HIGH									((uint8_t) 0x02)

    /* AGPIO0_DAT_IN field */
    #define AGPIO0_DAT_IN_MASK									((uint8_t) 0x01)
    #define AGPIO_DATA_INPUT_AGPIO0_DAT_IN_BITOFFSET			(0)
    #define AGPIO0_DAT_IN_LOW									((uint8_t) 0x00)    // DEFAULT
    #define AGPIO0_DAT_IN_HIGH									((uint8_t) 0x01)


/* Register 0x07 (GPIO_DATA_INPUT) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                     RESERVED[3:0]                                     |     GPIO3_DAT_IN    |     GPIO2_DAT_IN    |     GPIO1_DAT_IN    |     GPIO0_DAT_IN    |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPIO_DATA_INPUT register */
    #define GPIO_DATA_INPUT_ADDRESS								((uint8_t) 0x07)
    #define GPIO_DATA_INPUT_DEFAULT								((uint8_t) 0x02)

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
    #define GPIO1_DAT_IN_LOW									((uint8_t) 0x00)
    #define GPIO1_DAT_IN_HIGH									((uint8_t) 0x02)    // DEFAULT

    /* GPIO0_DAT_IN field */
    #define GPIO0_DAT_IN_MASK									((uint8_t) 0x01)
    #define GPIO_DATA_INPUT_GPIO0_DAT_IN_BITOFFSET				(0)
    #define GPIO0_DAT_IN_LOW									((uint8_t) 0x00)    // DEFAULT
    #define GPIO0_DAT_IN_HIGH									((uint8_t) 0x01)


/* Register 0x08 (FIFO_SEQ_STATUS) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |      SEQ_ACTIVE     |                                     SEQ_COUNT[3:0]                                    |       FIFO_OFn      |       FIFO_UFn      |   FIFO_CRC_FAULTn   |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* FIFO_SEQ_STATUS register */
    #define FIFO_SEQ_STATUS_ADDRESS								((uint8_t) 0x08)
    #define FIFO_SEQ_STATUS_DEFAULT								((uint8_t) 0x07)

    /* SEQ_ACTIVE field */
    #define SEQ_ACTIVE_MASK										((uint8_t) 0x80)
    #define FIFO_SEQ_STATUS_SEQ_ACTIVE_BITOFFSET				(7)
    #define SEQ_ACTIVE_SEQUENCERNOTACTIVE						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_ACTIVE_SEQUENCERACTIVE							((uint8_t) 0x80)

    /* SEQ_COUNT field */
    #define SEQ_COUNT_MASK										((uint8_t) 0x78)
    #define FIFO_SEQ_STATUS_SEQ_COUNT_BITOFFSET					(3)

    /* FIFO_OFn field */
    #define FIFO_OFN_MASK										((uint8_t) 0x04)
    #define FIFO_SEQ_STATUS_FIFO_OFN_BITOFFSET					(2)
    #define FIFO_OFN_FIFOOVERFLOWOCCURRED						((uint8_t) 0x00)
    #define FIFO_OFN_NOFIFOOVERFLOWOCCURRED						((uint8_t) 0x04)    // DEFAULT

    /* FIFO_UFn field */
    #define FIFO_UFN_MASK										((uint8_t) 0x02)
    #define FIFO_SEQ_STATUS_FIFO_UFN_BITOFFSET					(1)
    #define FIFO_UFN_FIFOUNDERFLOWOCCURRED						((uint8_t) 0x00)
    #define FIFO_UFN_NOFIFOUNDERFLOWOCCURRED					((uint8_t) 0x02)    // DEFAULT

    /* FIFO_CRC_FAULTn field */
    #define FIFO_CRC_FAULTN_MASK								((uint8_t) 0x01)
    #define FIFO_SEQ_STATUS_FIFO_CRC_FAULTN_BITOFFSET			(0)
    #define FIFO_CRC_FAULTN_FIFOCRCFAULTOCCURRED				((uint8_t) 0x00)
    #define FIFO_CRC_FAULTN_NOFIFOCRCFAULTOCCURRED				((uint8_t) 0x01)    // DEFAULT


/* Register 0x09 (FIFO_DEPTH_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                      RESERVED[6:0]                                                                      |    FIFO_DEPTH[8]    |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* FIFO_DEPTH_MSB register */
    #define FIFO_DEPTH_MSB_ADDRESS								((uint8_t) 0x09)
    #define FIFO_DEPTH_MSB_DEFAULT								((uint8_t) 0x00)

    /* FIFO_DEPTH[8] field */
    #define FIFO_DEPTH_MSB_MASK									((uint8_t) 0x01)
    #define FIFO_DEPTH_MSB_FIFO_DEPTH_MSB_BITOFFSET				(0)


/* Register 0x0A (FIFO_DEPTH_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                FIFO_DEPTH[7:0]                                                                                |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* FIFO_DEPTH_LSB register */
    #define FIFO_DEPTH_LSB_ADDRESS								((uint8_t) 0x0A)
    #define FIFO_DEPTH_LSB_DEFAULT								((uint8_t) 0x00)

    /* FIFO_DEPTH field */
    #define FIFO_DEPTH_MASK										((uint8_t) 0xFF)
    #define FIFO_DEPTH_LSB_FIFO_DEPTH_BITOFFSET					(0)


/* Register 0x10 (CONVERSION_CTRL) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        START        |                                                STEP_INIT[4:0]                                               |       RESERVED      |         STOP        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CONVERSION_CTRL register */
    #define CONVERSION_CTRL_ADDRESS								((uint8_t) 0x10)
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


/* Register 0x11 (RESET) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                                RESET_CODE[7:0]                                                                                |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* RESET register */
    #define RESET_ADDRESS										((uint8_t) 0x11)
    #define RESET_DEFAULT										((uint8_t) 0x00)

    /* RESET_CODE field */
    #define RESET_CODE_MASK										((uint8_t) 0xFF)
    #define RESET_RESET_CODE_BITOFFSET							(0)


/* Register 0x12 (ADC_CFG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       RESERVED      |     FIFO_TEST_EN    |               BUF_MODE[1:0]               |              SPEED_MODE[1:0]              |      STBY_MODE      |         PWDN        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* ADC_CFG register */
    #define ADC_CFG_ADDRESS										((uint8_t) 0x12)
    #define ADC_CFG_DEFAULT										((uint8_t) 0x0C)

    /* FIFO_TEST_EN field */
    #define FIFO_TEST_EN_MASK				   					((uint8_t) 0x40)
    #define ADC_CFG_FIFO_TEST_EN_BITOFFSET					(6)
    #define FIFO_TEST_EN_DISABLED								((uint8_t) 0x00)    // DEFAULT
    #define FIFO_TEST_EN_ENABLED		   						((uint8_t) 0x40)

    /* BUF_MODE field */
    #define BUF_MODE_MASK				   						((uint8_t) 0x30)
    #define ADC_CFG_BUF_MODE_BITOFFSET							(4)
    #define BUF_MODE_BUFFERSOFF			         			((uint8_t) 0x00)    // DEFAULT
    #define BUF_MODE_ONLYPBUFFER                      	((uint8_t) 0x10)
    #define BUF_MODE_BOTHPANDNBUFFERS                 	((uint8_t) 0x20)
    #define BUF_MODE_BOTHPANDNBUFFERSWITHOUTLEVELSHIFT	((uint8_t) 0x30)

    /* SPEED_MODE field */
    #define SPEED_MODE_MASK										((uint8_t) 0x0C)
    #define ADC_CFG_SPEED_MODE_BITOFFSET						(2)
    #define SPEED_MODE_VERYLOWSPEEDMODE0						((uint8_t) 0x00)
    #define SPEED_MODE_LOWSPEEDMODE1							((uint8_t) 0x04)
    #define SPEED_MODE_MIDSPEEDMODE6							((uint8_t) 0x08)
    #define SPEED_MODE_HIGHSPEEDMODE12							((uint8_t) 0x0C)    // DEFAULT

    /* STBY_MODE field */
    #define STBY_MODE_MASK								      	((uint8_t) 0x02)
    #define ADC_CFG_STBY_MODE_BITOFFSET						(1)
    #define STBY_MODE_IDLE	                              ((uint8_t) 0x00)    // DEFAULT
    #define STBY_MODE_STANDBY                         	((uint8_t) 0x02)

    /* PWDN field */
    #define PWDN_MASK						   					((uint8_t) 0x01)
    #define ADC_CFG_PWDN_BITOFFSET								(0)
    #define PWDN_ACTIVE						   					((uint8_t) 0x00)    // DEFAULT
    #define PWDN_POWERDOWNMODE									((uint8_t) 0x01)


/* Register 0x13 (REFERENCE_CFG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                           RESERVED[5:0]                                                           |       REF_VAL       |     REFP_BUF_EN     |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REFERENCE_CFG register */
    #define REFERENCE_CFG_ADDRESS						   		((uint8_t) 0x13)
    #define REFERENCE_CFG_DEFAULT						   		((uint8_t) 0x01)

    /* REF_VAL field */
    #define REF_VAL_MASK					   	   				((uint8_t) 0x02)
    #define REFERENCE_CFG_REF_VAL_BITOFFSET						(1)
    #define REF_VAL_INTERNALADCREFERENCEVALUEIS2				((uint8_t) 0x00)    // DEFAULT
    #define REF_VAL_INTERNALADCREFERENCEVALUEIS4				((uint8_t) 0x02)

    /* REFP_BUF_EN field */
    #define REFP_BUF_EN_MASK								      	((uint8_t) 0x01)
    #define REFERENCE_CFG_REFP_BUF_EN_BITOFFSET					(0)
    #define REFP_BUF_EN_DISABLED						      		((uint8_t) 0x00)
    #define REFP_BUF_EN_ENABLED						   			((uint8_t) 0x01)    // DEFAULT


/* Register 0x14 (CLK_DIGITAL_CFG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |               RESERVED[1:0]               |                CLK_DIV[1:0]               |       CLK_SEL       |       OUT_DRV       |       SDO_MODE      |     CONT_READ_EN    |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CLK_DIGITAL_CFG register */
    #define CLK_DIGITAL_CFG_ADDRESS					   			((uint8_t) 0x14)
    #define CLK_DIGITAL_CFG_DEFAULT					   			((uint8_t) 0x04)

    /* CLK_DIV field */
    #define CLK_DIV_MASK								      		((uint8_t) 0x30)
    #define CLK_DIGITAL_CFG_CLK_DIV_BITOFFSET					(4)
    #define CLK_DIV_DIVIDEBY1							      		((uint8_t) 0x00)    // DEFAULT
    #define CLK_DIV_DIVIDEBY2									      ((uint8_t) 0x10)
    #define CLK_DIV_DIVIDEBY8						      			((uint8_t) 0x20)
    #define CLK_DIV_DIVIDEBY16							   		((uint8_t) 0x30)

    /* CLK_SEL field */
    #define CLK_SEL_MASK									      	((uint8_t) 0x08)
    #define CLK_DIGITAL_CFG_CLK_SEL_BITOFFSET					(3)
    #define CLK_SEL_INTERNALOSCILLATOR						   	((uint8_t) 0x00)    // DEFAULT
    #define CLK_SEL_EXTERNALCLOCK							   	((uint8_t) 0x08)

    /* OUT_DRV field */
    #define OUT_DRV_MASK									      	((uint8_t) 0x04)
    #define CLK_DIGITAL_CFG_OUT_DRV_BITOFFSET					(2)
    #define OUT_DRV_FULLDRIVESTRENGTH						   	((uint8_t) 0x00)
    #define OUT_DRV_HALFDRIVESTRENGTH						   	((uint8_t) 0x04)    // DEFAULT

    /* SDO_MODE field */
    #define SDO_MODE_MASK     										((uint8_t) 0x02)
    #define CLK_DIGITAL_CFG_SDO_MODE_BITOFFSET				  	(1)
    #define SDO_MODE_DATAOUTPUTONLYMODE							((uint8_t) 0x00)    // DEFAULT
    #define SDO_MODE_DUALMODEDATAOUTPUTANDDATAREADY			((uint8_t) 0x02)

    /* CONT_READ_EN field */
    #define CONT_READ_EN_MASK								      	((uint8_t) 0x01)
    #define CLK_DIGITAL_CFG_CONT_READ_EN_BITOFFSET				(0)
    #define CONT_READ_EN_CONTINUOUSREADMODEDISABLED			((uint8_t) 0x00)    // DEFAULT
    #define CONT_READ_EN_CONTINUOUSREADMODEENABLED				((uint8_t) 0x01)


/* Register 0x15 (AGPIO_CFG0) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |              AGPIO3_CFG[1:0]              |              AGPIO2_CFG[1:0]              |              AGPIO1_CFG[1:0]              |              AGPIO0_CFG[1:0]              |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* AGPIO_CFG0 register */
    #define AGPIO_CFG0_ADDRESS									((uint8_t) 0x15)
    #define AGPIO_CFG0_DEFAULT									((uint8_t) 0x00)

    /* AGPIO3_CFG field */
    #define AGPIO3_CFG_MASK										((uint8_t) 0xC0)
    #define AGPIO_CFG0_AGPIO3_CFG_BITOFFSET					(6)
    #define AGPIO3_CFG_PINOPERATESASANALOGINPUT				((uint8_t) 0x00)    // DEFAULT
    #define AGPIO3_CFG_DIGITALINPUT								((uint8_t) 0x40)
    #define AGPIO3_CFG_PUSHPULLDIGITALOUTPUT					((uint8_t) 0x80)
    #define AGPIO3_CFG_OPENDRAINDIGITALOUTPUT				((uint8_t) 0xC0)

    /* AGPIO2_CFG field */
    #define AGPIO2_CFG_MASK										((uint8_t) 0x30)
    #define AGPIO_CFG0_AGPIO2_CFG_BITOFFSET					(4)
    #define AGPIO2_CFG_PINOPERATESASANALOGINPUT				((uint8_t) 0x00)    // DEFAULT
    #define AGPIO2_CFG_DIGITALINPUT								((uint8_t) 0x10)
    #define AGPIO2_CFG_PUSHPULLDIGITALOUTPUT					((uint8_t) 0x20)
    #define AGPIO2_CFG_OPENDRAINDIGITALOUTPUT				((uint8_t) 0x30)

    /* AGPIO1_CFG field */
    #define AGPIO1_CFG_MASK										((uint8_t) 0x0C)
    #define AGPIO_CFG0_AGPIO1_CFG_BITOFFSET					(2)
    #define AGPIO1_CFG_PINOPERATESASANALOGINPUT				((uint8_t) 0x00)    // DEFAULT
    #define AGPIO1_CFG_DIGITALINPUT								((uint8_t) 0x04)
    #define AGPIO1_CFG_PUSHPULLDIGITALOUTPUT					((uint8_t) 0x08)
    #define AGPIO1_CFG_OPENDRAINDIGITALOUTPUT				((uint8_t) 0x0C)

    /* AGPIO0_CFG field */
    #define AGPIO0_CFG_MASK										((uint8_t) 0x03)
    #define AGPIO_CFG0_AGPIO0_CFG_BITOFFSET					(0)
    #define AGPIO0_CFG_PINOPERATESASANALOGINPUT				((uint8_t) 0x00)    // DEFAULT
    #define AGPIO0_CFG_DIGITALINPUT								((uint8_t) 0x01)
    #define AGPIO0_CFG_PUSHPULLDIGITALOUTPUT					((uint8_t) 0x02)
    #define AGPIO0_CFG_OPENDRAINDIGITALOUTPUT				((uint8_t) 0x03)


/* Register 0x16 (AGPIO_CFG1) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |              AGPIO7_CFG[1:0]              |              AGPIO6_CFG[1:0]              |              AGPIO5_CFG[1:0]              |              AGPIO4_CFG[1:0]              |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* AGPIO_CFG1 register */
    #define AGPIO_CFG1_ADDRESS									((uint8_t) 0x16)
    #define AGPIO_CFG1_DEFAULT									((uint8_t) 0x00)

    /* AGPIO7_CFG field */
    #define AGPIO7_CFG_MASK										((uint8_t) 0xC0)
    #define AGPIO_CFG1_AGPIO7_CFG_BITOFFSET					(6)
    #define AGPIO7_CFG_PINOPERATESASANALOGINPUT				((uint8_t) 0x00)    // DEFAULT
    #define AGPIO7_CFG_DIGITALINPUT								((uint8_t) 0x40)
    #define AGPIO7_CFG_PUSHPULLDIGITALOUTPUT					((uint8_t) 0x80)
    #define AGPIO7_CFG_OPENDRAINDIGITALOUTPUT				((uint8_t) 0xC0)

    /* AGPIO6_CFG field */
    #define AGPIO6_CFG_MASK										((uint8_t) 0x30)
    #define AGPIO_CFG1_AGPIO6_CFG_BITOFFSET					(4)
    #define AGPIO6_CFG_PINOPERATESASANALOGINPUT				((uint8_t) 0x00)    // DEFAULT
    #define AGPIO6_CFG_DIGITALINPUT								((uint8_t) 0x10)
    #define AGPIO6_CFG_PUSHPULLDIGITALOUTPUT					((uint8_t) 0x20)
    #define AGPIO6_CFG_OPENDRAINDIGITALOUTPUT				((uint8_t) 0x30)

    /* AGPIO5_CFG field */
    #define AGPIO5_CFG_MASK										((uint8_t) 0x0C)
    #define AGPIO_CFG1_AGPIO5_CFG_BITOFFSET					(2)
    #define AGPIO5_CFG_PINOPERATESASANALOGINPUT				((uint8_t) 0x00)    // DEFAULT
    #define AGPIO5_CFG_DIGITALINPUT								((uint8_t) 0x04)
    #define AGPIO5_CFG_PUSHPULLDIGITALOUTPUT					((uint8_t) 0x08)
    #define AGPIO5_CFG_OPENDRAINDIGITALOUTPUT				((uint8_t) 0x0C)

    /* AGPIO4_CFG field */
    #define AGPIO4_CFG_MASK										((uint8_t) 0x03)
    #define AGPIO_CFG1_AGPIO4_CFG_BITOFFSET					(0)
    #define AGPIO4_CFG_PINOPERATESASANALOGINPUT				((uint8_t) 0x00)    // DEFAULT
    #define AGPIO4_CFG_DIGITALINPUT								((uint8_t) 0x01)
    #define AGPIO4_CFG_PUSHPULLDIGITALOUTPUT					((uint8_t) 0x02)
    #define AGPIO4_CFG_OPENDRAINDIGITALOUTPUT				((uint8_t) 0x03)


/* Register 0x17 (GPIO_CFG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |               GPIO3_CFG[1:0]              |               GPIO2_CFG[1:0]              |               GPIO1_CFG[1:0]              |               GPIO0_CFG[1:0]              |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPIO_CFG register */
    #define GPIO_CFG_ADDRESS									((uint8_t) 0x17)
    #define GPIO_CFG_DEFAULT									((uint8_t) 0x0C)

    /* GPIO3_CFG field */
    #define GPIO3_CFG_MASK										((uint8_t) 0xC0)
    #define GPIO_CFG_GPIO3_CFG_BITOFFSET					(6)
    #define GPIO3_CFG_DISABLED								((uint8_t) 0x00)    // DEFAULT
    #define GPIO3_CFG_DIGITALINPUT							((uint8_t) 0x40)
    #define GPIO3_CFG_PUSHPULLDIGITALOUTPUT				((uint8_t) 0x80)
    #define GPIO3_CFG_PINOPERATESASFAULTNOUTPUT			((uint8_t) 0xC0)

    /* GPIO2_CFG field */
    #define GPIO2_CFG_MASK										((uint8_t) 0x30)
    #define GPIO_CFG_GPIO2_CFG_BITOFFSET					(4)
    #define GPIO2_CFG_DISABLED						   	((uint8_t) 0x00)    // DEFAULT
    #define GPIO2_CFG_DIGITALINPUT							((uint8_t) 0x10)
    #define GPIO2_CFG_PUSHPULLDIGITALOUTPUT				((uint8_t) 0x20)
    #define GPIO2_CFG_PINOPERATESASEXTERNALCLOCKINPUT	((uint8_t) 0x30)

    /* GPIO1_CFG field */
    #define GPIO1_CFG_MASK										((uint8_t) 0x0C)
    #define GPIO_CFG_GPIO1_CFG_BITOFFSET					(2)
    #define GPIO1_CFG_DISABLED								((uint8_t) 0x00)
    #define GPIO1_CFG_DIGITALINPUT							((uint8_t) 0x04)
    #define GPIO1_CFG_PUSHPULLDIGITALOUTPUT				((uint8_t) 0x08)
    #define GPIO1_CFG_PINOPERATESASDRDYNOUTPUT			((uint8_t) 0x0C)    // DEFAULT

    /* GPIO0_CFG field */
    #define GPIO0_CFG_MASK										((uint8_t) 0x03)
    #define GPIO_CFG_GPIO0_CFG_BITOFFSET					(0)
    #define GPIO0_CFG_DISABLED							   ((uint8_t) 0x00)    // DEFAULT
    #define GPIO0_CFG_DIGITALINPUT							((uint8_t) 0x01)
    #define GPIO0_CFG_PUSHPULLDIGITALOUTPUT				((uint8_t) 0x02)
    #define GPIO0_CFG_PINOPERATESASSTARTSYNCINPUT		((uint8_t) 0x03)


/* Register 0x18 (SPARE_CFG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        SPARE7       |        SPARE6       |        SPARE5       |        SPARE4       |        SPARE3       |        SPARE2       |        SPARE1       |        SPARE0       |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SPARE_CFG register */
    #define SPARE_CFG_ADDRESS									((uint8_t) 0x18)
    #define SPARE_CFG_DEFAULT									((uint8_t) 0x00)

    /* SPARE7 field */
    #define SPARE7_MASK											((uint8_t) 0x80)
    #define SPARE_CFG_SPARE7_BITOFFSET							(7)
    #define SPARE7_SPAREISPROGRAMMEDTO0B						((uint8_t) 0x00)    // DEFAULT
    #define SPARE7_SPAREISPROGRAMMEDTO1B						((uint8_t) 0x80)

    /* SPARE6 field */
    #define SPARE6_MASK											((uint8_t) 0x40)
    #define SPARE_CFG_SPARE6_BITOFFSET							(6)
    #define SPARE6_SPAREISPROGRAMMEDTO0B						((uint8_t) 0x00)    // DEFAULT
    #define SPARE6_SPAREISPROGRAMMEDTO1B						((uint8_t) 0x40)

    /* SPARE5 field */
    #define SPARE5_MASK											((uint8_t) 0x20)
    #define SPARE_CFG_SPARE5_BITOFFSET							(5)
    #define SPARE5_SPAREISPROGRAMMEDTO0B						((uint8_t) 0x00)    // DEFAULT
    #define SPARE5_SPAREISPROGRAMMEDTO1B						((uint8_t) 0x20)

    /* SPARE4 field */
    #define SPARE4_MASK											((uint8_t) 0x10)
    #define SPARE_CFG_SPARE4_BITOFFSET							(4)
    #define SPARE4_SPAREISPROGRAMMEDTO0B						((uint8_t) 0x00)    // DEFAULT
    #define SPARE4_SPAREISPROGRAMMEDTO1B						((uint8_t) 0x10)

    /* SPARE3 field */
    #define SPARE3_MASK											((uint8_t) 0x08)
    #define SPARE_CFG_SPARE3_BITOFFSET							(3)
    #define SPARE3_SPAREISPROGRAMMEDTO0B						((uint8_t) 0x00)    // DEFAULT
    #define SPARE3_SPAREISPROGRAMMEDTO1B						((uint8_t) 0x08)

    /* SPARE2 field */
    #define SPARE2_MASK											((uint8_t) 0x04)
    #define SPARE_CFG_SPARE2_BITOFFSET							(2)
    #define SPARE2_SPAREISPROGRAMMEDTO0B						((uint8_t) 0x00)    // DEFAULT
    #define SPARE2_SPAREISPROGRAMMEDTO1B						((uint8_t) 0x04)

    /* SPARE1 field */
    #define SPARE1_MASK											((uint8_t) 0x02)
    #define SPARE_CFG_SPARE1_BITOFFSET							(1)
    #define SPARE1_SPAREISPROGRAMMEDTO0B						((uint8_t) 0x00)    // DEFAULT
    #define SPARE1_SPAREISPROGRAMMEDTO1B						((uint8_t) 0x02)

    /* SPARE0 field */
    #define SPARE0_MASK											((uint8_t) 0x01)
    #define SPARE_CFG_SPARE0_BITOFFSET							(0)
    #define SPARE0_SPAREISPROGRAMMEDTO0B						((uint8_t) 0x00)    // DEFAULT
    #define SPARE0_SPAREISPROGRAMMEDTO1B						((uint8_t) 0x01)


/* Register 0x20 (SEQUENCER_CFG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |               SEQ_MODE[1:0]               |             STOP_BEHAVIOR[1:0]            |               RESERVED[1:0]               |               DRDY_CFG[1:0]               |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SEQUENCER_CFG register */
    #define SEQUENCER_CFG_ADDRESS								((uint8_t) 0x20)
    #define SEQUENCER_CFG_DEFAULT								((uint8_t) 0x40)

    /* SEQ_MODE field */
    #define SEQ_MODE_MASK									   	         ((uint8_t) 0xC0)
    #define SEQUENCER_CFG_SEQ_MODE_BITOFFSET				      	   (6)
    #define SEQ_MODE_SEQUENCERDISABLEDSTEPEXECUTEDONLYONCE	      ((uint8_t) 0x00)
    #define SEQ_MODE_SEQUENCERDISABLEDSTEPREPEATEDINDEFINITELY	   ((uint8_t) 0x40)    // DEFAULT
    #define SEQ_MODE_SEQUENCERENABLEDEXECUTESEQUENCEONCE    	   ((uint8_t) 0x80)
    #define SEQ_MODE_SEQUENCERENABLEDEXECUTESEQUENCECONTINUOUSLY  ((uint8_t) 0xC0)

    /* STOP_BEHAVIOR field */
    #define STOP_BEHAVIOR_MASK									      ((uint8_t) 0x30)
    #define SEQUENCER_CFG_STOP_BEHAVIOR_BITOFFSET				   (4)
    #define STOP_BEHAVIOR_IMMEDIATELY						         ((uint8_t) 0x00)    // DEFAULT
    #define STOP_BEHAVIOR_AFTERCURRENTCONVERSIONISCOMPLETED	   ((uint8_t) 0x10)
    #define STOP_BEHAVIOR_AFTERCURRENTSEQUENCESTEPISCOMPLETED	((uint8_t) 0x20)
    #define STOP_BEHAVIOR_AFTERFULLSEQUENCEISCOMPLETED		   ((uint8_t) 0x30)

    /* DRDY_CFG field */
    #define DRDY_CFG_MASK										         ((uint8_t) 0x03)
    #define SEQUENCER_CFG_DRDY_CFG_BITOFFSET					      (0)
    #define DRDY_CFG_COMPLETEDCONVERSION	                     ((uint8_t) 0x00)    // DEFAULT
    #define DRDY_CFG_COMPLETEDSEQUENCESTEP	                  ((uint8_t) 0x01)
    #define DRDY_CFG_COMPLETEDSEQUENCE	                        ((uint8_t) 0x02)
    #define DRDY_CFG_FIFOTHRESHOLD                          	((uint8_t) 0x03)


/* Register 0x21 (SEQUENCE_STEP_EN_0) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |    SEQ_STEP_7_EN    |    SEQ_STEP_6_EN    |    SEQ_STEP_5_EN    |    SEQ_STEP_4_EN    |    SEQ_STEP_3_EN    |    SEQ_STEP_2_EN    |    SEQ_STEP_1_EN    |    SEQ_STEP_0_EN    |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SEQUENCE_STEP_EN_0 register */
    #define SEQUENCE_STEP_EN_0_ADDRESS							((uint8_t) 0x21)
    #define SEQUENCE_STEP_EN_0_DEFAULT							((uint8_t) 0x01)

    /* SEQ_STEP_7_EN field */
    #define SEQ_STEP_7_EN_MASK									((uint8_t) 0x80)
    #define SEQUENCE_STEP_EN_0_SEQ_STEP_7_EN_BITOFFSET			(7)
    #define SEQ_STEP_7_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_7_EN_STEPISENABLED							((uint8_t) 0x80)

    /* SEQ_STEP_6_EN field */
    #define SEQ_STEP_6_EN_MASK									((uint8_t) 0x40)
    #define SEQUENCE_STEP_EN_0_SEQ_STEP_6_EN_BITOFFSET			(6)
    #define SEQ_STEP_6_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_6_EN_STEPISENABLED							((uint8_t) 0x40)

    /* SEQ_STEP_5_EN field */
    #define SEQ_STEP_5_EN_MASK									((uint8_t) 0x20)
    #define SEQUENCE_STEP_EN_0_SEQ_STEP_5_EN_BITOFFSET			(5)
    #define SEQ_STEP_5_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_5_EN_STEPISENABLED							((uint8_t) 0x20)

    /* SEQ_STEP_4_EN field */
    #define SEQ_STEP_4_EN_MASK									((uint8_t) 0x10)
    #define SEQUENCE_STEP_EN_0_SEQ_STEP_4_EN_BITOFFSET			(4)
    #define SEQ_STEP_4_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_4_EN_STEPISENABLED							((uint8_t) 0x10)

    /* SEQ_STEP_3_EN field */
    #define SEQ_STEP_3_EN_MASK									((uint8_t) 0x08)
    #define SEQUENCE_STEP_EN_0_SEQ_STEP_3_EN_BITOFFSET			(3)
    #define SEQ_STEP_3_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_3_EN_STEPISENABLED							((uint8_t) 0x08)

    /* SEQ_STEP_2_EN field */
    #define SEQ_STEP_2_EN_MASK									((uint8_t) 0x04)
    #define SEQUENCE_STEP_EN_0_SEQ_STEP_2_EN_BITOFFSET			(2)
    #define SEQ_STEP_2_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_2_EN_STEPISENABLED							((uint8_t) 0x04)

    /* SEQ_STEP_1_EN field */
    #define SEQ_STEP_1_EN_MASK									((uint8_t) 0x02)
    #define SEQUENCE_STEP_EN_0_SEQ_STEP_1_EN_BITOFFSET			(1)
    #define SEQ_STEP_1_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_1_EN_STEPISENABLED							((uint8_t) 0x02)

    /* SEQ_STEP_0_EN field */
    #define SEQ_STEP_0_EN_MASK									((uint8_t) 0x01)
    #define SEQUENCE_STEP_EN_0_SEQ_STEP_0_EN_BITOFFSET			(0)
    #define SEQ_STEP_0_EN_STEPISDISABLED						((uint8_t) 0x00)
    #define SEQ_STEP_0_EN_STEPISENABLED							((uint8_t) 0x01)    // DEFAULT


/* Register 0x22 (SEQUENCE_STEP_EN_1) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |    SEQ_STEP_15_EN   |    SEQ_STEP_14_EN   |    SEQ_STEP_13_EN   |    SEQ_STEP_12_EN   |    SEQ_STEP_11_EN   |    SEQ_STEP_10_EN   |    SEQ_STEP_9_EN    |    SEQ_STEP_8_EN    |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SEQUENCE_STEP_EN_1 register */
    #define SEQUENCE_STEP_EN_1_ADDRESS							((uint8_t) 0x22)
    #define SEQUENCE_STEP_EN_1_DEFAULT							((uint8_t) 0x00)

    /* SEQ_STEP_15_EN field */
    #define SEQ_STEP_15_EN_MASK									((uint8_t) 0x80)
    #define SEQUENCE_STEP_EN_1_SEQ_STEP_15_EN_BITOFFSET			(7)
    #define SEQ_STEP_15_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_15_EN_STEPISENABLED						((uint8_t) 0x80)

    /* SEQ_STEP_14_EN field */
    #define SEQ_STEP_14_EN_MASK									((uint8_t) 0x40)
    #define SEQUENCE_STEP_EN_1_SEQ_STEP_14_EN_BITOFFSET			(6)
    #define SEQ_STEP_14_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_14_EN_STEPISENABLED						((uint8_t) 0x40)

    /* SEQ_STEP_13_EN field */
    #define SEQ_STEP_13_EN_MASK									((uint8_t) 0x20)
    #define SEQUENCE_STEP_EN_1_SEQ_STEP_13_EN_BITOFFSET			(5)
    #define SEQ_STEP_13_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_13_EN_STEPISENABLED						((uint8_t) 0x20)

    /* SEQ_STEP_12_EN field */
    #define SEQ_STEP_12_EN_MASK									((uint8_t) 0x10)
    #define SEQUENCE_STEP_EN_1_SEQ_STEP_12_EN_BITOFFSET			(4)
    #define SEQ_STEP_12_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_12_EN_STEPISENABLED						((uint8_t) 0x10)

    /* SEQ_STEP_11_EN field */
    #define SEQ_STEP_11_EN_MASK									((uint8_t) 0x08)
    #define SEQUENCE_STEP_EN_1_SEQ_STEP_11_EN_BITOFFSET			(3)
    #define SEQ_STEP_11_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_11_EN_STEPISENABLED						((uint8_t) 0x08)

    /* SEQ_STEP_10_EN field */
    #define SEQ_STEP_10_EN_MASK									((uint8_t) 0x04)
    #define SEQUENCE_STEP_EN_1_SEQ_STEP_10_EN_BITOFFSET			(2)
    #define SEQ_STEP_10_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_10_EN_STEPISENABLED						((uint8_t) 0x04)

    /* SEQ_STEP_9_EN field */
    #define SEQ_STEP_9_EN_MASK									((uint8_t) 0x02)
    #define SEQUENCE_STEP_EN_1_SEQ_STEP_9_EN_BITOFFSET			(1)
    #define SEQ_STEP_9_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_9_EN_STEPISENABLED							((uint8_t) 0x02)

    /* SEQ_STEP_8_EN field */
    #define SEQ_STEP_8_EN_MASK									((uint8_t) 0x01)
    #define SEQUENCE_STEP_EN_1_SEQ_STEP_8_EN_BITOFFSET			(0)
    #define SEQ_STEP_8_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_8_EN_STEPISENABLED							((uint8_t) 0x01)


/* Register 0x23 (SEQUENCE_STEP_EN_2) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |    SEQ_STEP_23_EN   |    SEQ_STEP_22_EN   |    SEQ_STEP_21_EN   |    SEQ_STEP_20_EN   |    SEQ_STEP_19_EN   |    SEQ_STEP_18_EN   |    SEQ_STEP_17_EN   |    SEQ_STEP_16_EN   |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SEQUENCE_STEP_EN_2 register */
    #define SEQUENCE_STEP_EN_2_ADDRESS							((uint8_t) 0x23)
    #define SEQUENCE_STEP_EN_2_DEFAULT							((uint8_t) 0x00)

    /* SEQ_STEP_23_EN field */
    #define SEQ_STEP_23_EN_MASK									((uint8_t) 0x80)
    #define SEQUENCE_STEP_EN_2_SEQ_STEP_23_EN_BITOFFSET			(7)
    #define SEQ_STEP_23_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_23_EN_STEPISENABLED						((uint8_t) 0x80)

    /* SEQ_STEP_22_EN field */
    #define SEQ_STEP_22_EN_MASK									((uint8_t) 0x40)
    #define SEQUENCE_STEP_EN_2_SEQ_STEP_22_EN_BITOFFSET			(6)
    #define SEQ_STEP_22_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_22_EN_STEPISENABLED						((uint8_t) 0x40)

    /* SEQ_STEP_21_EN field */
    #define SEQ_STEP_21_EN_MASK									((uint8_t) 0x20)
    #define SEQUENCE_STEP_EN_2_SEQ_STEP_21_EN_BITOFFSET			(5)
    #define SEQ_STEP_21_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_21_EN_STEPISENABLED						((uint8_t) 0x20)

    /* SEQ_STEP_20_EN field */
    #define SEQ_STEP_20_EN_MASK									((uint8_t) 0x10)
    #define SEQUENCE_STEP_EN_2_SEQ_STEP_20_EN_BITOFFSET			(4)
    #define SEQ_STEP_20_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_20_EN_STEPISENABLED						((uint8_t) 0x10)

    /* SEQ_STEP_19_EN field */
    #define SEQ_STEP_19_EN_MASK									((uint8_t) 0x08)
    #define SEQUENCE_STEP_EN_2_SEQ_STEP_19_EN_BITOFFSET			(3)
    #define SEQ_STEP_19_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_19_EN_STEPISENABLED						((uint8_t) 0x08)

    /* SEQ_STEP_18_EN field */
    #define SEQ_STEP_18_EN_MASK									((uint8_t) 0x04)
    #define SEQUENCE_STEP_EN_2_SEQ_STEP_18_EN_BITOFFSET			(2)
    #define SEQ_STEP_18_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_18_EN_STEPISENABLED						((uint8_t) 0x04)

    /* SEQ_STEP_17_EN field */
    #define SEQ_STEP_17_EN_MASK									((uint8_t) 0x02)
    #define SEQUENCE_STEP_EN_2_SEQ_STEP_17_EN_BITOFFSET			(1)
    #define SEQ_STEP_17_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_17_EN_STEPISENABLED						((uint8_t) 0x02)

    /* SEQ_STEP_16_EN field */
    #define SEQ_STEP_16_EN_MASK									((uint8_t) 0x01)
    #define SEQUENCE_STEP_EN_2_SEQ_STEP_16_EN_BITOFFSET			(0)
    #define SEQ_STEP_16_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_16_EN_STEPISENABLED						((uint8_t) 0x01)


/* Register 0x24 (SEQUENCE_STEP_EN_3) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |    SEQ_STEP_31_EN   |    SEQ_STEP_30_EN   |    SEQ_STEP_29_EN   |    SEQ_STEP_28_EN   |    SEQ_STEP_27_EN   |    SEQ_STEP_26_EN   |    SEQ_STEP_25_EN   |    SEQ_STEP_24_EN   |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* SEQUENCE_STEP_EN_3 register */
    #define SEQUENCE_STEP_EN_3_ADDRESS							((uint8_t) 0x24)
    #define SEQUENCE_STEP_EN_3_DEFAULT							((uint8_t) 0x00)

    /* SEQ_STEP_31_EN field */
    #define SEQ_STEP_31_EN_MASK									((uint8_t) 0x80)
    #define SEQUENCE_STEP_EN_3_SEQ_STEP_31_EN_BITOFFSET			(7)
    #define SEQ_STEP_31_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_31_EN_STEPISENABLED						((uint8_t) 0x80)

    /* SEQ_STEP_30_EN field */
    #define SEQ_STEP_30_EN_MASK									((uint8_t) 0x40)
    #define SEQUENCE_STEP_EN_3_SEQ_STEP_30_EN_BITOFFSET			(6)
    #define SEQ_STEP_30_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_30_EN_STEPISENABLED						((uint8_t) 0x40)

    /* SEQ_STEP_29_EN field */
    #define SEQ_STEP_29_EN_MASK									((uint8_t) 0x20)
    #define SEQUENCE_STEP_EN_3_SEQ_STEP_29_EN_BITOFFSET			(5)
    #define SEQ_STEP_29_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_29_EN_STEPISENABLED						((uint8_t) 0x20)

    /* SEQ_STEP_28_EN field */
    #define SEQ_STEP_28_EN_MASK									((uint8_t) 0x10)
    #define SEQUENCE_STEP_EN_3_SEQ_STEP_28_EN_BITOFFSET			(4)
    #define SEQ_STEP_28_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_28_EN_STEPISENABLED						((uint8_t) 0x10)

    /* SEQ_STEP_27_EN field */
    #define SEQ_STEP_27_EN_MASK									((uint8_t) 0x08)
    #define SEQUENCE_STEP_EN_3_SEQ_STEP_27_EN_BITOFFSET			(3)
    #define SEQ_STEP_27_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_27_EN_STEPISENABLED						((uint8_t) 0x08)

    /* SEQ_STEP_26_EN field */
    #define SEQ_STEP_26_EN_MASK									((uint8_t) 0x04)
    #define SEQUENCE_STEP_EN_3_SEQ_STEP_26_EN_BITOFFSET			(2)
    #define SEQ_STEP_26_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_26_EN_STEPISENABLED						((uint8_t) 0x04)

    /* SEQ_STEP_25_EN field */
    #define SEQ_STEP_25_EN_MASK									((uint8_t) 0x02)
    #define SEQUENCE_STEP_EN_3_SEQ_STEP_25_EN_BITOFFSET			(1)
    #define SEQ_STEP_25_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_25_EN_STEPISENABLED						((uint8_t) 0x02)

    /* SEQ_STEP_24_EN field */
    #define SEQ_STEP_24_EN_MASK									((uint8_t) 0x01)
    #define SEQUENCE_STEP_EN_3_SEQ_STEP_24_EN_BITOFFSET			(0)
    #define SEQ_STEP_24_EN_STEPISDISABLED						((uint8_t) 0x00)    // DEFAULT
    #define SEQ_STEP_24_EN_STEPISENABLED						((uint8_t) 0x01)


/* Register 0x25 (FIFO_CFG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                      RESERVED[6:0]                                                                      |       FIFO_EN       |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* FIFO_CFG register */
    #define FIFO_CFG_ADDRESS									((uint8_t) 0x25)
    #define FIFO_CFG_DEFAULT									((uint8_t) 0x00)

    /* FIFO_EN field */
    #define FIFO_EN_MASK										((uint8_t) 0x01)
    #define FIFO_CFG_FIFO_EN_BITOFFSET							(0)
    #define FIFO_EN_FIFOISDISABLED								((uint8_t) 0x00)    // DEFAULT
    #define FIFO_EN_FIFOISENABLED								((uint8_t) 0x01)


/* Register 0x26 (FIFO_THRES_A_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                      RESERVED[6:0]                                                                      |   FIFO_THRES_A[8]   |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* FIFO_THRES_A_MSB register */
    #define FIFO_THRES_A_MSB_ADDRESS							((uint8_t) 0x26)
    #define FIFO_THRES_A_MSB_DEFAULT							((uint8_t) 0x00)

    /* FIFO_THRES_A[8] field */
    #define FIFO_THRES_A_MSB_MASK								((uint8_t) 0x01)
    #define FIFO_THRES_A_MSB_FIFO_THRES_A_MSB_BITOFFSET			(0)


/* Register 0x27 (FIFO_THRES_A_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                               FIFO_THRES_A[7:0]                                                                               |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* FIFO_THRES_A_LSB register */
    #define FIFO_THRES_A_LSB_ADDRESS							((uint8_t) 0x27)
    #define FIFO_THRES_A_LSB_DEFAULT							((uint8_t) 0x00)

    /* FIFO_THRES_A field */
    #define FIFO_THRES_A_MASK									((uint8_t) 0xFF)
    #define FIFO_THRES_A_LSB_FIFO_THRES_A_BITOFFSET				(0)


/* Register 0x28 (FIFO_THRES_B_MSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                      RESERVED[6:0]                                                                      |   FIFO_THRES_B[8]   |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* FIFO_THRES_B_MSB register */
    #define FIFO_THRES_B_MSB_ADDRESS							((uint8_t) 0x28)
    #define FIFO_THRES_B_MSB_DEFAULT							((uint8_t) 0x00)

    /* FIFO_THRES_B[8] field */
    #define FIFO_THRES_B_MSB_MASK								((uint8_t) 0x01)
    #define FIFO_THRES_B_MSB_FIFO_THRES_B_MSB_BITOFFSET			(0)


/* Register 0x29 (FIFO_THRES_B_LSB) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                               FIFO_THRES_B[7:0]                                                                               |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* FIFO_THRES_B_LSB register */
    #define FIFO_THRES_B_LSB_ADDRESS							((uint8_t) 0x29)
    #define FIFO_THRES_B_LSB_DEFAULT							((uint8_t) 0x00)

    /* FIFO_THRES_B field */
    #define FIFO_THRES_B_MASK									((uint8_t) 0xFF)
    #define FIFO_THRES_B_LSB_FIFO_THRES_B_BITOFFSET				(0)


/* Register 0x2A (DIAG_MONITOR_CFG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       RESERVED      |      TDAC_RANGE     |  FAULT_PIN_BEHAVIOR |    REG_MAP_CRC_EN   |       RESERVED      |      REF_UV_EN      |      STATUS_EN      |      SPI_CRC_EN     |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

   /* DIAG_MONITOR_CFG register */
    #define DIAG_MONITOR_CFG_ADDRESS							((uint8_t) 0x2A)
    #define DIAG_MONITOR_CFG_DEFAULT							((uint8_t) 0x20)

    /* TDAC_RANGE field */
    #define TDAC_RANGE_MASK										((uint8_t) 0x40)
    #define DIAG_MONITOR_CFG_TDAC_RANGE_BITOFFSET				(6)
    #define TDAC_RANGE_TDACRANGEIS2								((uint8_t) 0x00)    // DEFAULT
    #define TDAC_RANGE_TDACRANGEIS4								((uint8_t) 0x40)

    /* FAULT_PIN_BEHAVIOR field */
    #define FAULT_PIN_BEHAVIOR_MASK								((uint8_t) 0x20)
    #define DIAG_MONITOR_CFG_FAULT_PIN_BEHAVIOR_BITOFFSET		(5)
    #define FAULT_PIN_BEHAVIOR_STATIC	                        ((uint8_t) 0x00)
    #define FAULT_PIN_BEHAVIOR_DYNAMIC                      	((uint8_t) 0x20)    // DEFAULT

    /* REG_MAP_CRC_EN field */
    #define REG_MAP_CRC_EN_MASK									((uint8_t) 0x10)
    #define DIAG_MONITOR_CFG_REG_MAP_CRC_EN_BITOFFSET			(4)
    #define REG_MAP_CRC_EN_DISABLED								((uint8_t) 0x00)    // DEFAULT
    #define REG_MAP_CRC_EN_ENABLED								((uint8_t) 0x10)

    /* REF_UV_EN field */
    #define REF_UV_EN_MASK										((uint8_t) 0x04)
    #define DIAG_MONITOR_CFG_REF_UV_EN_BITOFFSET				(2)
    #define REF_UV_EN_REFERENCEMONITORDISABLED					((uint8_t) 0x00)    // DEFAULT
    #define REF_UV_EN_REFERENCEMONITORENABLED					((uint8_t) 0x04)

    /* STATUS_EN field */
    #define STATUS_EN_MASK										((uint8_t) 0x02)
    #define DIAG_MONITOR_CFG_STATUS_EN_BITOFFSET				(1)
    #define STATUS_EN_DISABLED									((uint8_t) 0x00)    // DEFAULT
    #define STATUS_EN_ENABLED									((uint8_t) 0x02)

    /* SPI_CRC_EN field */
    #define SPI_CRC_EN_MASK										((uint8_t) 0x01)
    #define DIAG_MONITOR_CFG_SPI_CRC_EN_BITOFFSET				(0)
    #define SPI_CRC_EN_DISABLED									((uint8_t) 0x00)    // DEFAULT
    #define SPI_CRC_EN_ENABLED									((uint8_t) 0x01)


/* Register 0x2B (POSTFILTER_CFG0) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                RESERVED[4:0]                                                |                PF_AVG[1:0]                |        PF_CFG       |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* POSTFILTER_CFG0 register */
    #define POSTFILTER_CFG0_ADDRESS								((uint8_t) 0x2B)
    #define POSTFILTER_CFG0_DEFAULT								((uint8_t) 0x00)

    /* PF_AVG field */
    #define PF_AVG_MASK											((uint8_t) 0x06)
    #define POSTFILTER_CFG0_PF_AVG_BITOFFSET					(1)
    #define PF_AVG_AVERAGE4										((uint8_t) 0x00)    // DEFAULT
    #define PF_AVG_AVERAGE8										((uint8_t) 0x02)
    #define PF_AVG_AVERAGE16									((uint8_t) 0x04)

    /* PF_CFG field */
    #define PF_CFG_MASK											((uint8_t) 0x01)
    #define POSTFILTER_CFG0_PF_CFG_BITOFFSET					(0)
    #define PF_CFG_FILTERNOTCASCADED							((uint8_t) 0x00)    // DEFAULT
    #define PF_CFG_FILTERCASCADED3X								((uint8_t) 0x01)


/* Register 0x2C (POSTFILTER_CFG1) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        PF7_EN       |        PF6_EN       |        PF5_EN       |        PF4_EN       |        PF3_EN       |        PF2_EN       |        PF1_EN       |        PF0_EN       |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* POSTFILTER_CFG1 register */
    #define POSTFILTER_CFG1_ADDRESS								((uint8_t) 0x2C)
    #define POSTFILTER_CFG1_DEFAULT								((uint8_t) 0x00)

    /* PF7_EN field */
    #define PF7_EN_MASK											((uint8_t) 0x80)
    #define POSTFILTER_CFG1_PF7_EN_BITOFFSET					(7)
    #define PF7_EN_DISABLED										((uint8_t) 0x00)    // DEFAULT
    #define PF7_EN_ENABLED										((uint8_t) 0x80)

    /* PF6_EN field */
    #define PF6_EN_MASK											((uint8_t) 0x40)
    #define POSTFILTER_CFG1_PF6_EN_BITOFFSET					(6)
    #define PF6_EN_DISABLED										((uint8_t) 0x00)    // DEFAULT
    #define PF6_EN_ENABLED										((uint8_t) 0x40)

    /* PF5_EN field */
    #define PF5_EN_MASK											((uint8_t) 0x20)
    #define POSTFILTER_CFG1_PF5_EN_BITOFFSET					(5)
    #define PF5_EN_DISABLED										((uint8_t) 0x00)    // DEFAULT
    #define PF5_EN_ENABLED										((uint8_t) 0x20)

    /* PF4_EN field */
    #define PF4_EN_MASK											((uint8_t) 0x10)
    #define POSTFILTER_CFG1_PF4_EN_BITOFFSET					(4)
    #define PF4_EN_DISABLED										((uint8_t) 0x00)    // DEFAULT
    #define PF4_EN_ENABLED										((uint8_t) 0x10)

    /* PF3_EN field */
    #define PF3_EN_MASK											((uint8_t) 0x08)
    #define POSTFILTER_CFG1_PF3_EN_BITOFFSET					(3)
    #define PF3_EN_DISABLED										((uint8_t) 0x00)    // DEFAULT
    #define PF3_EN_ENABLED										((uint8_t) 0x08)

    /* PF2_EN field */
    #define PF2_EN_MASK											((uint8_t) 0x04)
    #define POSTFILTER_CFG1_PF2_EN_BITOFFSET					(2)
    #define PF2_EN_DISABLED										((uint8_t) 0x00)    // DEFAULT
    #define PF2_EN_ENABLED										((uint8_t) 0x04)

    /* PF1_EN field */
    #define PF1_EN_MASK											((uint8_t) 0x02)
    #define POSTFILTER_CFG1_PF1_EN_BITOFFSET					(1)
    #define PF1_EN_DISABLED										((uint8_t) 0x00)    // DEFAULT
    #define PF1_EN_ENABLED										((uint8_t) 0x02)

    /* PF0_EN field */
    #define PF0_EN_MASK											((uint8_t) 0x01)
    #define POSTFILTER_CFG1_PF0_EN_BITOFFSET					(0)
    #define PF0_EN_DISABLED										((uint8_t) 0x00)    // DEFAULT
    #define PF0_EN_ENABLED										((uint8_t) 0x01)


/* Register 0x2D (POSTFILTER_CFG2) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |      PF7_BYPASS     |      PF6_BYPASS     |      PF5_BYPASS     |      PF4_BYPASS     |      PF3_BYPASS     |      PF2_BYPASS     |      PF1_BYPASS     |      PF0_BYPASS     |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* POSTFILTER_CFG2 register */
    #define POSTFILTER_CFG2_ADDRESS								((uint8_t) 0x2D)
    #define POSTFILTER_CFG2_DEFAULT								((uint8_t) 0xFF)

    /* PF7_BYPASS field */
    #define PF7_BYPASS_MASK										((uint8_t) 0x80)
    #define POSTFILTER_CFG2_PF7_BYPASS_BITOFFSET				(7)
    #define PF7_BYPASS_POSTFILTEREDDATAISPROVIDEDTOTHEOUTPUT	((uint8_t) 0x00)
    #define PF7_BYPASS_POSTFILTERISBYPASSEDDATAISPROVIDEDDIRECTLYFROMSYNC4FILTER	((uint8_t) 0x80)    // DEFAULT

    /* PF6_BYPASS field */
    #define PF6_BYPASS_MASK										((uint8_t) 0x40)
    #define POSTFILTER_CFG2_PF6_BYPASS_BITOFFSET				(6)
    #define PF6_BYPASS_POSTFILTEREDDATAISPROVIDEDTOTHEOUTPUT	((uint8_t) 0x00)
    #define PF6_BYPASS_POSTFILTERISBYPASSEDDATAISPROVIDEDDIRECTLYFROMSYNC4FILTER	((uint8_t) 0x40)    // DEFAULT

    /* PF5_BYPASS field */
    #define PF5_BYPASS_MASK										((uint8_t) 0x20)
    #define POSTFILTER_CFG2_PF5_BYPASS_BITOFFSET				(5)
    #define PF5_BYPASS_POSTFILTEREDDATAISPROVIDEDTOTHEOUTPUT	((uint8_t) 0x00)
    #define PF5_BYPASS_POSTFILTERISBYPASSEDDATAISPROVIDEDDIRECTLYFROMSYNC4FILTER	((uint8_t) 0x20)    // DEFAULT

    /* PF4_BYPASS field */
    #define PF4_BYPASS_MASK										((uint8_t) 0x10)
    #define POSTFILTER_CFG2_PF4_BYPASS_BITOFFSET				(4)
    #define PF4_BYPASS_POSTFILTEREDDATAISPROVIDEDTOTHEOUTPUT	((uint8_t) 0x00)
    #define PF4_BYPASS_POSTFILTERISBYPASSEDDATAISPROVIDEDDIRECTLYFROMSYNC4FILTER	((uint8_t) 0x10)    // DEFAULT

    /* PF3_BYPASS field */
    #define PF3_BYPASS_MASK										((uint8_t) 0x08)
    #define POSTFILTER_CFG2_PF3_BYPASS_BITOFFSET				(3)
    #define PF3_BYPASS_POSTFILTEREDDATAISPROVIDEDTOTHEOUTPUT	((uint8_t) 0x00)
    #define PF3_BYPASS_POSTFILTERISBYPASSEDDATAISPROVIDEDDIRECTLYFROMSYNC4FILTER	((uint8_t) 0x08)    // DEFAULT

    /* PF2_BYPASS field */
    #define PF2_BYPASS_MASK										((uint8_t) 0x04)
    #define POSTFILTER_CFG2_PF2_BYPASS_BITOFFSET				(2)
    #define PF2_BYPASS_POSTFILTEREDDATAISPROVIDEDTOTHEOUTPUT	((uint8_t) 0x00)
    #define PF2_BYPASS_POSTFILTERISBYPASSEDDATAISPROVIDEDDIRECTLYFROMSYNC4FILTER	((uint8_t) 0x04)    // DEFAULT

    /* PF1_BYPASS field */
    #define PF1_BYPASS_MASK										((uint8_t) 0x02)
    #define POSTFILTER_CFG2_PF1_BYPASS_BITOFFSET				(1)
    #define PF1_BYPASS_POSTFILTEREDDATAISPROVIDEDTOTHEOUTPUT	((uint8_t) 0x00)
    #define PF1_BYPASS_POSTFILTERISBYPASSEDDATAISPROVIDEDDIRECTLYFROMSYNC4FILTER	((uint8_t) 0x02)    // DEFAULT

    /* PF0_BYPASS field */
    #define PF0_BYPASS_MASK										((uint8_t) 0x01)
    #define POSTFILTER_CFG2_PF0_BYPASS_BITOFFSET				(0)
    #define PF0_BYPASS_POSTFILTEREDDATAISPROVIDEDTOTHEOUTPUT	((uint8_t) 0x00)
    #define PF0_BYPASS_POSTFILTERISBYPASSEDDATAISPROVIDEDDIRECTLYFROMSYNC4FILTER	((uint8_t) 0x01)    // DEFAULT


/* Register 0x30 (CS_FWD_CFG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                        CS_FWD_EN_CODE[5:0]                                                        |              TIMEOUT_SEL[1:0]             |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CS_FWD_CFG register */
    #define CS_FWD_CFG_ADDRESS									((uint8_t) 0x30)
    #define CS_FWD_CFG_DEFAULT									((uint8_t) 0x00)

    /* CS_FWD_EN_CODE field */
    #define CS_FWD_EN_CODE_MASK									((uint8_t) 0xFC)
    #define CS_FWD_CFG_CS_FWD_EN_CODE_BITOFFSET					(2)

    /* TIMEOUT_SEL field */
    #define TIMEOUT_SEL_MASK									((uint8_t) 0x03)
    #define CS_FWD_CFG_TIMEOUT_SEL_BITOFFSET					(0)
    #define TIMEOUT_SEL_TIMEOUTDISABLE							((uint8_t) 0x00)    // DEFAULT
    #define TIMEOUT_SEL_TIMEOUTENABLEWITHTHESHORTTIMEOUT256MCLKCYCLES	((uint8_t) 0x01)
    #define TIMEOUT_SEL_TIMEOUTENABLEWITHTHEMEDIUMLENGTHTIMEOUT2048MCLKCYCLES	((uint8_t) 0x02)
    #define TIMEOUT_SEL_TIMEOUTENABLEWITHTHELONGTIMEOUT16384MCLKCYCLES	((uint8_t) 0x03)


/* Register 0x31 (AGPIO_FWD_CFG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |    AGPIO7_FWD_EN    |    AGPIO6_FWD_EN    |    AGPIO5__FWD_EN   |    AGPIO4_FWD_EN    |    AGPIO3_FWD_EN    |    AGPIO2_FWD_EN    |    AGPIO1_FWD_EN    |    AGPIO0_FWD_EN    |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* AGPIO_FWD_CFG register */
    #define AGPIO_FWD_CFG_ADDRESS								((uint8_t) 0x31)
    #define AGPIO_FWD_CFG_DEFAULT								((uint8_t) 0x00)

    /* AGPIO7_FWD_EN field */
    #define AGPIO7_FWD_EN_MASK									((uint8_t) 0x80)
    #define AGPIO_FWD_CFG_AGPIO7_FWD_EN_BITOFFSET				(7)
    #define AGPIO7_FWD_EN_AGPIO7ISNOTCONFIGUREDASCSFORWARD		((uint8_t) 0x00)    // DEFAULT
    #define AGPIO7_FWD_EN_AGPIO7ISCONFIGUREDASCSFORWARD			((uint8_t) 0x80)

    /* AGPIO6_FWD_EN field */
    #define AGPIO6_FWD_EN_MASK									((uint8_t) 0x40)
    #define AGPIO_FWD_CFG_AGPIO6_FWD_EN_BITOFFSET				(6)
    #define AGPIO6_FWD_EN_AGPIO6ISNOTCONFIGUREDASCSFORWARD		((uint8_t) 0x00)    // DEFAULT
    #define AGPIO6_FWD_EN_AGPIO6ISCONFIGUREDASCSFORWARD			((uint8_t) 0x40)

    /* AGPIO5__FWD_EN field */
    #define AGPIO5_FWD_EN_MASK									((uint8_t) 0x20)
    #define AGPIO_FWD_CFG_AGPIO5_FWD_EN_BITOFFSET				(5)
    #define AGPIO5_FWD_EN_AGPIO5ISNOTCONFIGUREDASCSFORWARD		((uint8_t) 0x00)    // DEFAULT
    #define AGPIO5_FWD_EN_AGPIO5ISCONFIGUREDASCSFORWARD			((uint8_t) 0x20)

    /* AGPIO4_FWD_EN field */
    #define AGPIO4_FWD_EN_MASK									((uint8_t) 0x10)
    #define AGPIO_FWD_CFG_AGPIO4_FWD_EN_BITOFFSET				(4)
    #define AGPIO4_FWD_EN_AGPIO4ISNOTCONFIGUREDASCSFORWARD		((uint8_t) 0x00)    // DEFAULT
    #define AGPIO4_FWD_EN_AGPIO4ISCONFIGUREDASCSFORWARD			((uint8_t) 0x10)

    /* AGPIO3_FWD_EN field */
    #define AGPIO3_FWD_EN_MASK									((uint8_t) 0x08)
    #define AGPIO_FWD_CFG_AGPIO3_FWD_EN_BITOFFSET				(3)
    #define AGPIO3_FWD_EN_AGPIO3ISNOTCONFIGUREDASCSFORWARD		((uint8_t) 0x00)    // DEFAULT
    #define AGPIO3_FWD_EN_AGPIO3ISCONFIGUREDASCSFORWARD			((uint8_t) 0x08)

    /* AGPIO2_FWD_EN field */
    #define AGPIO2_FWD_EN_MASK									((uint8_t) 0x04)
    #define AGPIO_FWD_CFG_AGPIO2_FWD_EN_BITOFFSET				(2)
    #define AGPIO2_FWD_EN_AGPIO2ISNOTCONFIGUREDASCSFORWARD		((uint8_t) 0x00)    // DEFAULT
    #define AGPIO2_FWD_EN_AGPIO2ISCONFIGUREDASCSFORWARD			((uint8_t) 0x04)

    /* AGPIO1_FWD_EN field */
    #define AGPIO1_FWD_EN_MASK									((uint8_t) 0x02)
    #define AGPIO_FWD_CFG_AGPIO1_FWD_EN_BITOFFSET				(1)
    #define AGPIO1_FWD_EN_AGPIO1ISNOTCONFIGUREDASCSFORWARD		((uint8_t) 0x00)    // DEFAULT
    #define AGPIO1_FWD_EN_AGPIO1ISCONFIGUREDASCSFORWARD			((uint8_t) 0x02)

    /* AGPIO0_FWD_EN field */
    #define AGPIO0_FWD_EN_MASK									((uint8_t) 0x01)
    #define AGPIO_FWD_CFG_AGPIO0_FWD_EN_BITOFFSET				(0)
    #define AGPIO0_FWD_EN_AGPIO0ISNOTCONFIGUREDASCSFORWARD		((uint8_t) 0x00)    // DEFAULT
    #define AGPIO0_FWD_EN_AGPIO0ISCONFIGUREDASCSFORWARD			((uint8_t) 0x01)


/* Register 0x32 (GPIO_FWD_CFG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                     RESERVED[3:0]                                     |     GPIO3_FWD_EN    |     GPIO2_FWD_EN    |    GPIO1__FWD_EN    |     GPIO0_FWD_EN    |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPIO_FWD_CFG register */
    #define GPIO_FWD_CFG_ADDRESS								((uint8_t) 0x32)
    #define GPIO_FWD_CFG_DEFAULT								((uint8_t) 0x00)

    /* GPIO3_FWD_EN field */
    #define GPIO3_FWD_EN_MASK									((uint8_t) 0x08)
    #define GPIO_FWD_CFG_GPIO3_FWD_EN_BITOFFSET					(3)
    #define GPIO3_FWD_EN_GPIO3ISNOTCONFIGUREDASCSFORWARD		((uint8_t) 0x00)    // DEFAULT
    #define GPIO3_FWD_EN_GPIO3ISCONFIGUREDASCSFORWARD			((uint8_t) 0x08)

    /* GPIO2_FWD_EN field */
    #define GPIO2_FWD_EN_MASK									((uint8_t) 0x04)
    #define GPIO_FWD_CFG_GPIO2_FWD_EN_BITOFFSET					(2)
    #define GPIO2_FWD_EN_GPIO2ISNOTCONFIGUREDASCSFORWARD		((uint8_t) 0x00)    // DEFAULT
    #define GPIO2_FWD_EN_GPIO2ISCONFIGUREDASCSFORWARD			((uint8_t) 0x04)

    /* GPIO1__FWD_EN field */
    #define GPIO1_FWD_EN_MASK									((uint8_t) 0x02)
    #define GPIO_FWD_CFG_GPIO1_FWD_EN_BITOFFSET					(1)
    #define GPIO1_FWD_EN_GPIO1ISNOTCONFIGUREDASCSFORWARD		((uint8_t) 0x00)    // DEFAULT
    #define GPIO1_FWD_EN_GPIO1ISCONFIGUREDASCSFORWARD			((uint8_t) 0x02)

    /* GPIO0_FWD_EN field */
    #define GPIO0_FWD_EN_MASK									((uint8_t) 0x01)
    #define GPIO_FWD_CFG_GPIO0_FWD_EN_BITOFFSET					(0)
    #define GPIO0_FWD_EN_GPIO0ISNOTCONFIGUREDASCSFORWARD		((uint8_t) 0x00)    // DEFAULT
    #define GPIO0_FWD_EN_GPIO0ISCONFIGUREDASCSFORWARD			((uint8_t) 0x01)


/* Register 0x3D (REG_MAP_CRC) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                       GENERAL_CFG_REG_MAP_CRC_VALUE[7:0]                                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REG_MAP_CRC register */
    #define REG_MAP_CRC_ADDRESS									((uint8_t) 0x3D)
    #define REG_MAP_CRC_DEFAULT									((uint8_t) 0x00)

    /* GENERAL_CFG_REG_MAP_CRC_VALUE field */
    #define GENERAL_CFG_REG_MAP_CRC_VALUE_MASK					((uint8_t) 0xFF)
    #define REG_MAP_CRC_GENERAL_CFG_REG_MAP_CRC_VALUE_BITOFFSET	(0)


/* Register 0x3E (PAGE_INDICATOR) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                              PAGE_INDICATOR[7:0]                                                                              |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PAGE_INDICATOR register */
    #define PAGE_INDICATOR_ADDRESS								((uint8_t) 0x3E)
    #define PAGE_INDICATOR_DEFAULT								((uint8_t) 0x00)

    /* PAGE_INDICATOR field */
    #define PAGE_INDICATOR_MASK									((uint8_t) 0xFF)
    #define PAGE_INDICATOR_BITOFFSET							(0)


/* Register 0x3F (PAGE_POINTER) definition
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7        |        Bit 6        |        Bit 5        |        Bit 4        |        Bit 3        |        Bit 2        |        Bit 1        |        Bit 0        |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                               PAGE_POINTER[7:0]                                                                               |
 * |-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* PAGE_POINTER register */
    #define PAGE_POINTER_ADDRESS								((uint8_t) 0x3F)
    #define PAGE_POINTER_DEFAULT								((uint8_t) 0x00)

    /* PAGE_POINTER field */
    #define PAGE_POINTER_MASK									((uint8_t) 0xFF)
    #define PAGE_POINTER_BITOFFSET								(0)


#endif /* ADS125P08_PAGE0_H_ */