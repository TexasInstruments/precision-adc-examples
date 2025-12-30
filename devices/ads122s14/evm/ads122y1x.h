/*
 * @file ads122s1x_page0.h
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

#ifndef ADS122Y1X_H_
#define ADS122Y1X_H_

#include <stdint.h>
#include <stdbool.h>

//**********************************************************************************
//
// Typedefs
//
//**********************************************************************************
typedef struct {
    uint8_t status_msb;
    uint8_t status_lsb;
    uint8_t crc;
    int32_t data;
} adc_channel_t;

//**********************************************************************************
//
// Function prototypes
//
//**********************************************************************************

static void restoreRegisterDefaults(void);
uint8_t     getRegisterValue(uint8_t address);
void        adcStartup(void);
void        clearSTATUSflags(void);
uint8_t     readSingleRegister(uint8_t address);
bool        writeSingleRegister(uint8_t address, uint8_t data);
adc_channel_t     readData(void);
int32_t     signExtend(const uint8_t dataBytes[]);
bool        resetDevice(void);
void        enableRegisterMapCrc(bool enable);
bool        isValidCrcOut(void);


//**********************************************************************************
//
// Constants
//
//**********************************************************************************

#define NUM_REGISTERS                           ((uint8_t) 0x10)
/** Maximum register address or address of the last register in the regmap */
#define MAX_REGISTER_ADDRESS                    ((uint8_t) 0x0F)
#define SPI_DELAY           128 // Sets delay in systicks between SPI frames.  

//**********************************************************************************
//
// Register definitions
//
//**********************************************************************************


/* Register 0x00 (DEVICE_ID) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                              DEV_ID[7:0]                                                                              |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DEVICE_ID register */
    #define DEVICE_ID_ADDRESS                                   ((uint8_t) 0x00)

    /* DEV_ID field */
    #define DEVICE_ID_DEV_ID_MASK                               ((uint8_t) 0xFF)
    #define DEVICE_ID_DEV_ID_BITOFFSET                          (0)
    #define DEVICE_IS_16_BIT_MASK                               ((uint8_t) 0x0A)
    #define DEVICE_IS_24_BIT_MASK                               ((uint8_t) 0x0B)

/* Register 0x01 (REVISION_ID) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                              REV_ID[7:0]                                                                              |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REVISION_ID register */
    #define REVISION_ID_ADDRESS                                 ((uint8_t) 0x01)

    /* REV_ID field */
    #define REVISION_ID_REV_ID_MASK                             ((uint8_t) 0xFF)
    #define REVISION_ID_REV_ID_BITOFFSET                        (0)


/* Register 0x02 (STATUS_MSB) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |       RESETn       |      AVDD_UVn      |       REF_UVn      |   SPI_CRC_FAULTn   | REG_MAP_CRC_FAULTn |     MEM_FAULTn     |  REG_WRITE_FAULTn  |        DRDY        |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STATUS_MSB register */
    #define STATUS_MSB_ADDRESS                                  ((uint8_t) 0x02)
    #define STATUS_MSB_DEFAULT                                  ((uint8_t) 0x3E)

    /* RESETn field */
    #define STATUS_MSB_RESETN_MASK                              ((uint8_t) 0x80)
    #define STATUS_MSB_RESETN_BITOFFSET                         (7)
    #define STATUS_MSB_RESETN_RESETOCCURRED                     ((uint8_t) 0x00)    // DEFAULT
    #define STATUS_MSB_RESETN_NORESETOCCURRED                   ((uint8_t) 0x80)

    /* AVDD_UVn field */
    #define STATUS_MSB_AVDD_UVN_MASK                            ((uint8_t) 0x40)
    #define STATUS_MSB_AVDD_UVN_BITOFFSET                       (6)
    #define STATUS_MSB_AVDD_UVN_UNDERVOLTAGEFAULTOCCURRED       ((uint8_t) 0x00)    // DEFAULT
    #define STATUS_MSB_AVDD_UVN_NOUNDERVOLTAGEFAULTOCCURRED     ((uint8_t) 0x40)

    /* REF_UVn field */
    #define STATUS_MSB_REF_UVN_MASK                             ((uint8_t) 0x20)
    #define STATUS_MSB_REF_UVN_BITOFFSET                        (5)
    #define STATUS_MSB_REF_UVN_UNDERVOLTAGEFAULTOCCURRED        ((uint8_t) 0x00)
    #define STATUS_MSB_REF_UVN_NOUNDERVOLTAGEFAULTOCCURRED      ((uint8_t) 0x20)    // DEFAULT

    /* SPI_CRC_FAULTn field */
    #define STATUS_MSB_SPI_CRC_FAULTN_MASK                      ((uint8_t) 0x10)
    #define STATUS_MSB_SPI_CRC_FAULTN_BITOFFSET                 (4)
    #define STATUS_MSB_SPI_CRC_FAULTN_SPICRCFAULTOCCURRED       ((uint8_t) 0x00)
    #define STATUS_MSB_SPI_CRC_FAULTN_NOSPICRCFAULTOCCURRED     ((uint8_t) 0x10)    // DEFAULT

    /* REG_MAP_CRC_FAULTn field */
    #define STATUS_MSB_REG_MAP_CRC_FAULTN_MASK                  ((uint8_t) 0x08)
    #define STATUS_MSB_REG_MAP_CRC_FAULTN_BITOFFSET             (3)
    #define STATUS_MSB_REG_MAP_CRC_FAULTN_REGISTERMAPCRCFAULTOCCURRED ((uint8_t) 0x00)
    #define STATUS_MSB_REG_MAP_CRC_FAULTN_NOREGISTERMAPCRCFAULTOCCURRED ((uint8_t) 0x08)    // DEFAULT

    /* MEM_FAULTn field */
    #define STATUS_MSB_MEM_FAULTN_MASK                          ((uint8_t) 0x04)
    #define STATUS_MSB_MEM_FAULTN_BITOFFSET                     (2)
    #define STATUS_MSB_MEM_FAULTN_MEMORYMAPCRCFAULTOCCURRED     ((uint8_t) 0x00)
    #define STATUS_MSB_MEM_FAULTN_NOMEMORYMAPCRCFAULTOCCURRED   ((uint8_t) 0x04)    // DEFAULT

    /* REG_WRITE_FAULTn field */
    #define STATUS_MSB_REG_WRITE_FAULTN_MASK                    ((uint8_t) 0x02)
    #define STATUS_MSB_REG_WRITE_FAULTN_BITOFFSET               (1)
    #define STATUS_MSB_REG_WRITE_FAULTN_REGISTERACCESSFAULTOCCURRED ((uint8_t) 0x00)
    #define STATUS_MSB_REG_WRITE_FAULTN_NOREGISTERACCESSFAULTOCCURRED ((uint8_t) 0x02)    // DEFAULT

    /* DRDY field */
    #define STATUS_MSB_DRDY_MASK                                ((uint8_t) 0x01)
    #define STATUS_MSB_DRDY_BITOFFSET                           (0)
    #define STATUS_MSB_DRDY_DATAARENOTNEW                       ((uint8_t) 0x00)    // DEFAULT
    #define STATUS_MSB_DRDY_DATAARENEW                          ((uint8_t) 0x01)


/* Register 0x03 (STATUS_LSB) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                  CONV_COUNT[3:0]                                  |    GPIO3_DAT_IN    |    GPIO2_DAT_IN    |    GPIO1_DAT_IN    |    GPIO0_DAT_IN    |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STATUS_LSB register */
    #define STATUS_LSB_ADDRESS                                  ((uint8_t) 0x03)
    #define STATUS_LSB_DEFAULT                                  ((uint8_t) 0xF0)

    /* CONV_COUNT field */
    #define STATUS_LSB_CONV_COUNT_MASK                          ((uint8_t) 0xF0)
    #define STATUS_LSB_CONV_COUNT_BITOFFSET                     (4)

    /* GPIO3_DAT_IN field */
    #define STATUS_LSB_GPIO3_DAT_IN_MASK                        ((uint8_t) 0x08)
    #define STATUS_LSB_GPIO3_DAT_IN_BITOFFSET                   (3)
    #define STATUS_LSB_GPIO3_DAT_IN_LOW                         ((uint8_t) 0x00)    // DEFAULT
    #define STATUS_LSB_GPIO3_DAT_IN_HIGH                        ((uint8_t) 0x08)

    /* GPIO2_DAT_IN field */
    #define STATUS_LSB_GPIO2_DAT_IN_MASK                        ((uint8_t) 0x04)
    #define STATUS_LSB_GPIO2_DAT_IN_BITOFFSET                   (2)
    #define STATUS_LSB_GPIO2_DAT_IN_LOW                         ((uint8_t) 0x00)    // DEFAULT
    #define STATUS_LSB_GPIO2_DAT_IN_HIGH                        ((uint8_t) 0x04)

    /* GPIO1_DAT_IN field */
    #define STATUS_LSB_GPIO1_DAT_IN_MASK                        ((uint8_t) 0x02)
    #define STATUS_LSB_GPIO1_DAT_IN_BITOFFSET                   (1)
    #define STATUS_LSB_GPIO1_DAT_IN_LOW                         ((uint8_t) 0x00)    // DEFAULT
    #define STATUS_LSB_GPIO1_DAT_IN_HIGH                        ((uint8_t) 0x02)

    /* GPIO0_DAT_IN field */
    #define STATUS_LSB_GPIO0_DAT_IN_MASK                        ((uint8_t) 0x01)
    #define STATUS_LSB_GPIO0_DAT_IN_BITOFFSET                   (0)
    #define STATUS_LSB_GPIO0_DAT_IN_LOW                         ((uint8_t) 0x00)    // DEFAULT
    #define STATUS_LSB_GPIO0_DAT_IN_HIGH                        ((uint8_t) 0x01)


/* Register 0x04 (CONVERSION_CTRL) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                          RESET[5:0]                                                         |        START       |        STOP        |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CONVERSION_CTRL register */
    #define CONVERSION_CTRL_ADDRESS                             ((uint8_t) 0x04)
    #define CONVERSION_CTRL_DEFAULT                             ((uint8_t) 0x00)

    /* RESET field */
    #define CONVERSION_CTRL_RESET_MASK                          ((uint8_t) 0xFC)
    #define CONVERSION_CTRL_RESET_BITOFFSET                     (2)

    /* START field */
    #define CONVERSION_CTRL_START_MASK                          ((uint8_t) 0x02)
    #define CONVERSION_CTRL_START_BITOFFSET                     (1)
    #define CONVERSION_CTRL_START_NOOPERATION                   ((uint8_t) 0x00)    // DEFAULT
    #define CONVERSION_CTRL_START_START                         ((uint8_t) 0x02)

    /* STOP field */
    #define CONVERSION_CTRL_STOP_MASK                           ((uint8_t) 0x01)
    #define CONVERSION_CTRL_STOP_BITOFFSET                      (0)
    #define CONVERSION_CTRL_STOP_NOOPERATION                    ((uint8_t) 0x00)    // DEFAULT
    #define CONVERSION_CTRL_STOP_STOP                           ((uint8_t) 0x01)


/* Register 0x05 (DEVICE_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        PWDN        |      STBY_MODE     |                BOCS[1:0]                |       CLK_SEL      |      CONV_MODE     |             SPEED_MODE[1:0]             |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DEVICE_CFG register */
    #define DEVICE_CFG_ADDRESS                                  ((uint8_t) 0x05)
    #define DEVICE_CFG_DEFAULT                                  ((uint8_t) 0x00)

    /* PWDN field */
    #define DEVICE_CFG_PWDN_MASK                                ((uint8_t) 0x80)
    #define DEVICE_CFG_PWDN_BITOFFSET                           (7)
    #define DEVICE_CFG_PWDN_ACTIVEMODE                          ((uint8_t) 0x00)    // DEFAULT
    #define DEVICE_CFG_PWDN_POWERDOWNMODE                       ((uint8_t) 0x80)

    /* STBY_MODE field */
    #define DEVICE_CFG_STBY_MODE_MASK                           ((uint8_t) 0x40)
    #define DEVICE_CFG_STBY_MODE_BITOFFSET                      (6)
    #define DEVICE_CFG_STBY_MODE_IDLE                           ((uint8_t) 0x00)    // DEFAULT
    #define DEVICE_CFG_STBY_MODE_STANDBY                        ((uint8_t) 0x40)

    /* BOCS field */
    #define DEVICE_CFG_BOCS_MASK                                ((uint8_t) 0x30)
    #define DEVICE_CFG_BOCS_BITOFFSET                           (4)
    #define DEVICE_CFG_BOCS_DISABLED                            ((uint8_t) 0x00)    // DEFAULT
    #define DEVICE_CFG_BOCS_0                                   ((uint8_t) 0x10)
    #define DEVICE_CFG_BOCS_1MUA                                ((uint8_t) 0x20)
    #define DEVICE_CFG_BOCS_10MUA                               ((uint8_t) 0x30)

    /* CLK_SEL field */
    #define DEVICE_CFG_CLK_SEL_MASK                             ((uint8_t) 0x08)
    #define DEVICE_CFG_CLK_SEL_BITOFFSET                        (3)
    #define DEVICE_CFG_CLK_SEL_INTERNALOSCILLATOR               ((uint8_t) 0x00)    // DEFAULT
    #define DEVICE_CFG_CLK_SEL_EXTERNALCLOCK                    ((uint8_t) 0x08)

    /* CONV_MODE field */
    #define DEVICE_CFG_CONV_MODE_MASK                           ((uint8_t) 0x04)
    #define DEVICE_CFG_CONV_MODE_BITOFFSET                      (2)
    #define DEVICE_CFG_CONV_MODE_CONTINUOUSCONVERSIONMODE       ((uint8_t) 0x00)    // DEFAULT
    #define DEVICE_CFG_CONV_MODE_SINGLESHOTCONVERSIONMODE       ((uint8_t) 0x04)

    /* SPEED_MODE field */
    #define DEVICE_CFG_SPEED_MODE_MASK                          ((uint8_t) 0x03)
    #define DEVICE_CFG_SPEED_MODE_BITOFFSET                     (0)
    #define DEVICE_CFG_SPEED_MODE_SPEEDMODE0                    ((uint8_t) 0x00)    // DEFAULT
    #define DEVICE_CFG_SPEED_MODE_SPEEDMODE1                    ((uint8_t) 0x01)
    #define DEVICE_CFG_SPEED_MODE_SPEEDMODE2                    ((uint8_t) 0x02)
    #define DEVICE_CFG_SPEED_MODE_SPEEDMODE3                    ((uint8_t) 0x03)


/* Register 0x06 (DATA_RATE_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                     DELAY[3:0]                                    |        GC_EN       |                         FLTR_OSR[2:0]                        |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DATA_RATE_CFG register */
    #define DATA_RATE_CFG_ADDRESS                               ((uint8_t) 0x06)
    #define DATA_RATE_CFG_DEFAULT                               ((uint8_t) 0x00)

    /* DELAY field */
    #define DATA_RATE_CFG_DELAY_MASK                            ((uint8_t) 0xF0)
    #define DATA_RATE_CFG_DELAY_BITOFFSET                       (4)
    #define DATA_RATE_CFG_DELAY_0XTSUBMODSUB                    ((uint8_t) 0x00)    // DEFAULT
    #define DATA_RATE_CFG_DELAY_1XTSUBMODSUB                    ((uint8_t) 0x10)
    #define DATA_RATE_CFG_DELAY_2XTSUBMODSUB                    ((uint8_t) 0x20)
    #define DATA_RATE_CFG_DELAY_4XTSUBMODSUB                    ((uint8_t) 0x30)
    #define DATA_RATE_CFG_DELAY_8XTSUBMODSUB                    ((uint8_t) 0x40)
    #define DATA_RATE_CFG_DELAY_16XTSUBMODSUB                   ((uint8_t) 0x50)
    #define DATA_RATE_CFG_DELAY_32XTSUBMODSUB                   ((uint8_t) 0x60)
    #define DATA_RATE_CFG_DELAY_64XTSUBMODSUB                   ((uint8_t) 0x70)
    #define DATA_RATE_CFG_DELAY_128XTSUBMODSUB                  ((uint8_t) 0x80)
    #define DATA_RATE_CFG_DELAY_256XTSUBMODSUB                  ((uint8_t) 0x90)
    #define DATA_RATE_CFG_DELAY_512XTSUBMODSUB                  ((uint8_t) 0xA0)
    #define DATA_RATE_CFG_DELAY_1024XTSUBMODSUB                 ((uint8_t) 0xB0)
    #define DATA_RATE_CFG_DELAY_2048XTSUBMODSUB                 ((uint8_t) 0xC0)
    #define DATA_RATE_CFG_DELAY_4096XTSUBMODSUB                 ((uint8_t) 0xD0)
    #define DATA_RATE_CFG_DELAY_8192XTSUBMODSUB                 ((uint8_t) 0xE0)
    #define DATA_RATE_CFG_DELAY_16384XTSUBMODSUB                ((uint8_t) 0xF0)

    /* GC_EN field */
    #define DATA_RATE_CFG_GC_EN_MASK                            ((uint8_t) 0x08)
    #define DATA_RATE_CFG_GC_EN_BITOFFSET                       (3)
    #define DATA_RATE_CFG_GC_EN_DISABLED                        ((uint8_t) 0x00)    // DEFAULT
    #define DATA_RATE_CFG_GC_EN_ENABLED                         ((uint8_t) 0x08)

    /* FLTR_OSR field */
    #define DATA_RATE_CFG_FLTR_OSR_MASK                         ((uint8_t) 0x07)
    #define DATA_RATE_CFG_FLTR_OSR_BITOFFSET                    (0)
    #define DATA_RATE_CFG_FLTR_OSR_OSR16                        ((uint8_t) 0x00)    // DEFAULT
    #define DATA_RATE_CFG_FLTR_OSR_OSR32                        ((uint8_t) 0x01)
    #define DATA_RATE_CFG_FLTR_OSR_OSR128                       ((uint8_t) 0x02)
    #define DATA_RATE_CFG_FLTR_OSR_OSR256                       ((uint8_t) 0x03)
    #define DATA_RATE_CFG_FLTR_OSR_OSR512                       ((uint8_t) 0x04)
    #define DATA_RATE_CFG_FLTR_OSR_OSR1024                      ((uint8_t) 0x05)
    #define DATA_RATE_CFG_FLTR_OSR_FSUBDATASUB25SPS             ((uint8_t) 0x06)
    #define DATA_RATE_CFG_FLTR_OSR_FSUBDATASUB20SPS             ((uint8_t) 0x07)


/* Register 0x07 (MUX_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                     AINP[3:0]                                     |                                     AINN[3:0]                                     |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MUX_CFG register */
    #define MUX_CFG_ADDRESS                                     ((uint8_t) 0x07)
    #define MUX_CFG_DEFAULT                                     ((uint8_t) 0x01)

    /* AINP field */
    #define MUX_CFG_AINP_MASK                                   ((uint8_t) 0xF0)
    #define MUX_CFG_AINP_BITOFFSET                              (4)
    #define MUX_CFG_AINP_AIN0                                   ((uint8_t) 0x00)    // DEFAULT
    #define MUX_CFG_AINP_AIN1                                   ((uint8_t) 0x10)
    #define MUX_CFG_AINP_AIN2                                   ((uint8_t) 0x20)
    #define MUX_CFG_AINP_AIN3                                   ((uint8_t) 0x30)
    #define MUX_CFG_AINP_AIN4                                   ((uint8_t) 0x40)
    #define MUX_CFG_AINP_AIN5                                   ((uint8_t) 0x50)
    #define MUX_CFG_AINP_AIN6                                   ((uint8_t) 0x60)
    #define MUX_CFG_AINP_AIN7                                   ((uint8_t) 0x70)
    #define MUX_CFG_AINP_GND                                    ((uint8_t) 0x80)


    /* AINN field */
    #define MUX_CFG_AINN_MASK                                   ((uint8_t) 0x0F)
    #define MUX_CFG_AINN_BITOFFSET                              (0)
    #define MUX_CFG_AINN_AIN0                                   ((uint8_t) 0x00)
    #define MUX_CFG_AINN_AIN1                                   ((uint8_t) 0x01)    // DEFAULT
    #define MUX_CFG_AINN_AIN2                                   ((uint8_t) 0x02)
    #define MUX_CFG_AINN_AIN3                                   ((uint8_t) 0x03)
    #define MUX_CFG_AINN_AIN4                                   ((uint8_t) 0x04)
    #define MUX_CFG_AINN_AIN5                                   ((uint8_t) 0x05)
    #define MUX_CFG_AINN_AIN6                                   ((uint8_t) 0x06)
    #define MUX_CFG_AINN_AIN7                                   ((uint8_t) 0x07)
    #define MUX_CFG_AINN_GND                                    ((uint8_t) 0x08)



/* Register 0x08 (GAIN_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        SPARE       |                         SYS_MON[2:0]                         |                                     GAIN[3:0]                                     |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAIN_CFG register */
    #define GAIN_CFG_ADDRESS                                    ((uint8_t) 0x08)
    #define GAIN_CFG_DEFAULT                                    ((uint8_t) 0x01)

    /* SPARE field */
    #define GAIN_CFG_SPARE_MASK                                 ((uint8_t) 0x80)
    #define GAIN_CFG_SPARE_BITOFFSET                            (7)

    /* SYS_MON field */
    #define GAIN_CFG_SYS_MON_MASK                               ((uint8_t) 0x70)
    #define GAIN_CFG_SYS_MON_BITOFFSET                          (4)
    #define GAIN_CFG_SYS_MON_DISABLED                           ((uint8_t) 0x00)    // DEFAULT
    #define GAIN_CFG_SYS_MON_INTERNALSHORTOFDIFFERENTIALPGAINPUTSTO ((uint8_t) 0x10)
    #define GAIN_CFG_SYS_MON_INTERNALTEMPERATURESENSOR          ((uint8_t) 0x20)
    #define GAIN_CFG_SYS_MON_EXTERNAL8                          ((uint8_t) 0x30)
    #define GAIN_CFG_SYS_MON_AVDD8                              ((uint8_t) 0x40)
    #define GAIN_CFG_SYS_MON_DVDD8                              ((uint8_t) 0x50)

    /* GAIN field */
    #define GAIN_CFG_GAIN_MASK                                  ((uint8_t) 0x0F)
    #define GAIN_CFG_GAIN_BITOFFSET                             (0)
    #define GAIN_CFG_GAIN_0                                     ((uint8_t) 0x00)
    #define GAIN_CFG_GAIN_1                                     ((uint8_t) 0x01)    // DEFAULT
    #define GAIN_CFG_GAIN_2                                     ((uint8_t) 0x02)
    #define GAIN_CFG_GAIN_4                                     ((uint8_t) 0x03)
    #define GAIN_CFG_GAIN_5                                     ((uint8_t) 0x04)
    #define GAIN_CFG_GAIN_8                                     ((uint8_t) 0x05)
    #define GAIN_CFG_GAIN_10                                    ((uint8_t) 0x06)
    #define GAIN_CFG_GAIN_16                                    ((uint8_t) 0x07)
    #define GAIN_CFG_GAIN_20                                    ((uint8_t) 0x08)
    #define GAIN_CFG_GAIN_32                                    ((uint8_t) 0x09)
    #define GAIN_CFG_GAIN_50                                    ((uint8_t) 0x0A)
    #define GAIN_CFG_GAIN_64                                    ((uint8_t) 0x0B)
    #define GAIN_CFG_GAIN_100                                   ((uint8_t) 0x0C)
    #define GAIN_CFG_GAIN_128                                   ((uint8_t) 0x0D)
    #define GAIN_CFG_GAIN_200                                   ((uint8_t) 0x0E)
    #define GAIN_CFG_GAIN_256                                   ((uint8_t) 0x0F)


/* Register 0x09 (REFERENCE_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |      REF_UV_EN     |      RESERVED      |     REFP_BUF_EN    |     REFN_BUF_EN    |      RESERVED      |       REF_VAL      |               REF_SEL[1:0]              |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REFERENCE_CFG register */
    #define REFERENCE_CFG_ADDRESS                               ((uint8_t) 0x09)
    #define REFERENCE_CFG_DEFAULT                               ((uint8_t) 0x00)

    /* REF_UV_EN field */
    #define REFERENCE_CFG_REF_UV_EN_MASK                        ((uint8_t) 0x80)
    #define REFERENCE_CFG_REF_UV_EN_BITOFFSET                   (7)
    #define REFERENCE_CFG_REF_UV_EN_DISABLED                    ((uint8_t) 0x00)    // DEFAULT
    #define REFERENCE_CFG_REF_UV_EN_ENABLED                     ((uint8_t) 0x80)

    /* REFP_BUF_EN field */
    #define REFERENCE_CFG_REFP_BUF_EN_MASK                      ((uint8_t) 0x20)
    #define REFERENCE_CFG_REFP_BUF_EN_BITOFFSET                 (5)
    #define REFERENCE_CFG_REFP_BUF_EN_DISABLED                  ((uint8_t) 0x00)    // DEFAULT
    #define REFERENCE_CFG_REFP_BUF_EN_ENABLED                   ((uint8_t) 0x20)

    /* REFN_BUF_EN field */
    #define REFERENCE_CFG_REFN_BUF_EN_MASK                      ((uint8_t) 0x10)
    #define REFERENCE_CFG_REFN_BUF_EN_BITOFFSET                 (4)
    #define REFERENCE_CFG_REFN_BUF_EN_DISABLED                  ((uint8_t) 0x00)    // DEFAULT
    #define REFERENCE_CFG_REFN_BUF_EN_ENABLED                   ((uint8_t) 0x10)

    /* REF_VAL field */
    #define REFERENCE_CFG_REF_VAL_MASK                          ((uint8_t) 0x04)
    #define REFERENCE_CFG_REF_VAL_BITOFFSET                     (2)
    #define REFERENCE_CFG_REF_VAL_1                             ((uint8_t) 0x00)    // DEFAULT
    #define REFERENCE_CFG_REF_VAL_2                             ((uint8_t) 0x04)

    /* REF_SEL field */
    #define REFERENCE_CFG_REF_SEL_MASK                          ((uint8_t) 0x03)
    #define REFERENCE_CFG_REF_SEL_BITOFFSET                     (0)
    #define REFERENCE_CFG_REF_SEL_INTERNALVOLTAGEREFERENCE      ((uint8_t) 0x00)    // DEFAULT
    #define REFERENCE_CFG_REF_SEL_EXTERNALVOLTAGEREFERENCE      ((uint8_t) 0x01)
    #define REFERENCE_CFG_REF_SEL_AVDD                          ((uint8_t) 0x02)


/* Register 0x0A (DIGITAL_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        SPARE       |   REG_MAP_CRC_EN   |     SPI_CRC_EN     |      STATUS_EN     | FAULT_PIN_BEHAVIOR |    CONT_READ_EN    |       CODING       |      SDO_MODE      |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DIGITAL_CFG register */
    #define DIGITAL_CFG_ADDRESS                                 ((uint8_t) 0x0A)
    #define DIGITAL_CFG_DEFAULT                                 ((uint8_t) 0x00)

    /* SPARE field */
    #define DIGITAL_CFG_SPARE_MASK                              ((uint8_t) 0x80)
    #define DIGITAL_CFG_SPARE_BITOFFSET                         (7)

    /* REG_MAP_CRC_EN field */
    #define DIGITAL_CFG_REG_MAP_CRC_EN_MASK                     ((uint8_t) 0x40)
    #define DIGITAL_CFG_REG_MAP_CRC_EN_BITOFFSET                (6)
    #define DIGITAL_CFG_REG_MAP_CRC_EN_DISABLED                 ((uint8_t) 0x00)    // DEFAULT
    #define DIGITAL_CFG_REG_MAP_CRC_EN_ENABLED                  ((uint8_t) 0x40)

    /* SPI_CRC_EN field */
    #define DIGITAL_CFG_SPI_CRC_EN_MASK                         ((uint8_t) 0x20)
    #define DIGITAL_CFG_SPI_CRC_EN_BITOFFSET                    (5)
    #define DIGITAL_CFG_SPI_CRC_EN_DISABLED                     ((uint8_t) 0x00)    // DEFAULT
    #define DIGITAL_CFG_SPI_CRC_EN_ENABLED                      ((uint8_t) 0x20)

    /* STATUS_EN field */
    #define DIGITAL_CFG_STATUS_EN_MASK                          ((uint8_t) 0x10)
    #define DIGITAL_CFG_STATUS_EN_BITOFFSET                     (4)
    #define DIGITAL_CFG_STATUS_EN_DISABLED                      ((uint8_t) 0x00)    // DEFAULT
    #define DIGITAL_CFG_STATUS_EN_ENABLED                       ((uint8_t) 0x10)

    /* FAULT_PIN_BEHAVIOR field */
    #define DIGITAL_CFG_FAULT_PIN_BEHAVIOR_MASK                 ((uint8_t) 0x08)
    #define DIGITAL_CFG_FAULT_PIN_BEHAVIOR_BITOFFSET            (3)
    #define DIGITAL_CFG_FAULT_PIN_BEHAVIOR_STATIC               ((uint8_t) 0x00)    // DEFAULT
    #define DIGITAL_CFG_FAULT_PIN_BEHAVIOR_HEARTBEAT            ((uint8_t) 0x08)

    /* CONT_READ_EN field */
    #define DIGITAL_CFG_CONT_READ_EN_MASK                       ((uint8_t) 0x04)
    #define DIGITAL_CFG_CONT_READ_EN_BITOFFSET                  (2)
    #define DIGITAL_CFG_CONT_READ_EN_DISABLED                   ((uint8_t) 0x00)    // DEFAULT
    #define DIGITAL_CFG_CONT_READ_EN_ENABLED                    ((uint8_t) 0x04)

    /* CODING field */
    #define DIGITAL_CFG_CODING_MASK                             ((uint8_t) 0x02)
    #define DIGITAL_CFG_CODING_BITOFFSET                        (1)
    #define DIGITAL_CFG_CODING_BINARYTWOSCOMPLEMENT             ((uint8_t) 0x00)    // DEFAULT
    #define DIGITAL_CFG_CODING_UNIPOLARSTRAIGHTBINARY           ((uint8_t) 0x02)

    /* SDO_MODE field */
    #define DIGITAL_CFG_SDO_MODE_MASK                           ((uint8_t) 0x01)
    #define DIGITAL_CFG_SDO_MODE_BITOFFSET                      (0)
    #define DIGITAL_CFG_SDO_MODE_DATAOUTPUTONLYMODE             ((uint8_t) 0x00)    // DEFAULT
    #define DIGITAL_CFG_SDO_MODE_DUALMODEDATAOUTPUTANDDATAREADY ((uint8_t) 0x01)


/* Register 0x0B (GPIO_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |              GPIO3_CFG[1:0]             |              GPIO2_CFG[1:0]             |              GPIO1_CFG[1:0]             |              GPIO0_CFG[1:0]             |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPIO_CFG register */
    #define GPIO_CFG_ADDRESS                                    ((uint8_t) 0x0B)
    #define GPIO_CFG_DEFAULT                                    ((uint8_t) 0x00)

    /* GPIO3_CFG field */
    #define GPIO_CFG_GPIO3_CFG_MASK                             ((uint8_t) 0xC0)
    #define GPIO_CFG_GPIO3_CFG_BITOFFSET                        (6)
    #define GPIO_CFG_GPIO3_CFG_DISABLED                         ((uint8_t) 0x00)    // DEFAULT
    #define GPIO_CFG_GPIO3_CFG_DIGITALINPUTOREXTERNALCLOCKINPUTCLK_SEL1B ((uint8_t) 0x40)
    #define GPIO_CFG_GPIO3_CFG_PUSHPULLDIGITALOUTPUT            ((uint8_t) 0x80)
    #define GPIO_CFG_GPIO3_CFG_OPENDRAINDIGITALOUTPUT           ((uint8_t) 0xC0)

    /* GPIO2_CFG field */
    #define GPIO_CFG_GPIO2_CFG_MASK                             ((uint8_t) 0x30)
    #define GPIO_CFG_GPIO2_CFG_BITOFFSET                        (4)
    #define GPIO_CFG_GPIO2_CFG_DISABLED                         ((uint8_t) 0x00)    // DEFAULT
    #define GPIO_CFG_GPIO2_CFG_DIGITALINPUT                     ((uint8_t) 0x10)
    #define GPIO_CFG_GPIO2_CFG_PUSHPULLDIGITALOUTPUT            ((uint8_t) 0x20)
    #define GPIO_CFG_GPIO2_CFG_OPENDRAINDIGITALOUTPUT           ((uint8_t) 0x30)

    /* GPIO1_CFG field */
    #define GPIO_CFG_GPIO1_CFG_MASK                             ((uint8_t) 0x0C)
    #define GPIO_CFG_GPIO1_CFG_BITOFFSET                        (2)
    #define GPIO_CFG_GPIO1_CFG_DISABLED                         ((uint8_t) 0x00)    // DEFAULT
    #define GPIO_CFG_GPIO1_CFG_DIGITALINPUT                     ((uint8_t) 0x04)
    #define GPIO_CFG_GPIO1_CFG_PUSHPULLDIGITALOUTPUT            ((uint8_t) 0x08)
    #define GPIO_CFG_GPIO1_CFG_OPENDRAINDIGITALOUTPUT           ((uint8_t) 0x0C)

    /* GPIO0_CFG field */
    #define GPIO_CFG_GPIO0_CFG_MASK                             ((uint8_t) 0x03)
    #define GPIO_CFG_GPIO0_CFG_BITOFFSET                        (0)
    #define GPIO_CFG_GPIO0_CFG_DISABLED                         ((uint8_t) 0x00)    // DEFAULT
    #define GPIO_CFG_GPIO0_CFG_DIGITALINPUT                     ((uint8_t) 0x01)
    #define GPIO_CFG_GPIO0_CFG_PUSHPULLDIGITALOUTPUT            ((uint8_t) 0x02)
    #define GPIO_CFG_GPIO0_CFG_OPENDRAINDIGITALOUTPUT           ((uint8_t) 0x03)


/* Register 0x0C (GPIO_DATA_OUTPUT) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |      GPIO3_SRC     |      GPIO2_SRC     |              RESERVED[1:0]              |    GPIO3_DAT_OUT   |    GPIO2_DAT_OUT   |    GPIO1_DAT_OUT   |    GPIO0_DAT_OUT   |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPIO_DATA_OUTPUT register */
    #define GPIO_DATA_OUTPUT_ADDRESS                            ((uint8_t) 0x0C)
    #define GPIO_DATA_OUTPUT_DEFAULT                            ((uint8_t) 0x00)

    /* GPIO3_SRC field */
    #define GPIO_DATA_OUTPUT_GPIO3_SRC_MASK                     ((uint8_t) 0x80)
    #define GPIO_DATA_OUTPUT_GPIO3_SRC_BITOFFSET                (7)
    #define GPIO_DATA_OUTPUT_GPIO3_SRC_GPIO3_DAT_OUTBIT         ((uint8_t) 0x00)    // DEFAULT
    #define GPIO_DATA_OUTPUT_GPIO3_SRC_OVERDRDYOVER             ((uint8_t) 0x80)

    /* GPIO2_SRC field */
    #define GPIO_DATA_OUTPUT_GPIO2_SRC_MASK                     ((uint8_t) 0x40)
    #define GPIO_DATA_OUTPUT_GPIO2_SRC_BITOFFSET                (6)
    #define GPIO_DATA_OUTPUT_GPIO2_SRC_GPIO2_DAT_OUTBIT         ((uint8_t) 0x00)    // DEFAULT
    #define GPIO_DATA_OUTPUT_GPIO2_SRC_OVERFAULTOVER            ((uint8_t) 0x40)

    /* GPIO3_DAT_OUT field */
    #define GPIO_DATA_OUTPUT_GPIO3_DAT_OUT_MASK                 ((uint8_t) 0x08)
    #define GPIO_DATA_OUTPUT_GPIO3_DAT_OUT_BITOFFSET            (3)
    #define GPIO_DATA_OUTPUT_GPIO3_DAT_OUT_LOW                  ((uint8_t) 0x00)    // DEFAULT
    #define GPIO_DATA_OUTPUT_GPIO3_DAT_OUT_HIGH                 ((uint8_t) 0x08)

    /* GPIO2_DAT_OUT field */
    #define GPIO_DATA_OUTPUT_GPIO2_DAT_OUT_MASK                 ((uint8_t) 0x04)
    #define GPIO_DATA_OUTPUT_GPIO2_DAT_OUT_BITOFFSET            (2)
    #define GPIO_DATA_OUTPUT_GPIO2_DAT_OUT_LOW                  ((uint8_t) 0x00)    // DEFAULT
    #define GPIO_DATA_OUTPUT_GPIO2_DAT_OUT_HIGH                 ((uint8_t) 0x04)

    /* GPIO1_DAT_OUT field */
    #define GPIO_DATA_OUTPUT_GPIO1_DAT_OUT_MASK                 ((uint8_t) 0x02)
    #define GPIO_DATA_OUTPUT_GPIO1_DAT_OUT_BITOFFSET            (1)
    #define GPIO_DATA_OUTPUT_GPIO1_DAT_OUT_LOW                  ((uint8_t) 0x00)    // DEFAULT
    #define GPIO_DATA_OUTPUT_GPIO1_DAT_OUT_HIGH                 ((uint8_t) 0x02)

    /* GPIO0_DAT_OUT field */
    #define GPIO_DATA_OUTPUT_GPIO0_DAT_OUT_MASK                 ((uint8_t) 0x01)
    #define GPIO_DATA_OUTPUT_GPIO0_DAT_OUT_BITOFFSET            (0)
    #define GPIO_DATA_OUTPUT_GPIO0_DAT_OUT_LOW                  ((uint8_t) 0x00)    // DEFAULT
    #define GPIO_DATA_OUTPUT_GPIO0_DAT_OUT_HIGH                 ((uint8_t) 0x01)


/* Register 0x0D (IDAC_MAG_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                     I2MAG[3:0]                                    |                                     I1MAG[3:0]                                    |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* IDAC_MAG_CFG register */
    #define IDAC_MAG_CFG_ADDRESS                                ((uint8_t) 0x0D)
    #define IDAC_MAG_CFG_DEFAULT                                ((uint8_t) 0x00)

    /* I2MAG field */
    #define IDAC_MAG_CFG_I2MAG_MASK                             ((uint8_t) 0xF0)
    #define IDAC_MAG_CFG_I2MAG_BITOFFSET                        (4)
    #define IDAC_MAG_CFG_I2MAG_DISABLED                         ((uint8_t) 0x00)    // DEFAULT
    #define IDAC_MAG_CFG_I2MAG_1XIUNIT                          ((uint8_t) 0x10)
    #define IDAC_MAG_CFG_I2MAG_10XIUNIT                         ((uint8_t) 0x20)
    #define IDAC_MAG_CFG_I2MAG_20XIUNIT                         ((uint8_t) 0x30)
    #define IDAC_MAG_CFG_I2MAG_30XIUNIT                         ((uint8_t) 0x40)
    #define IDAC_MAG_CFG_I2MAG_40XIUNIT                         ((uint8_t) 0x50)
    #define IDAC_MAG_CFG_I2MAG_50XIUNIT                         ((uint8_t) 0x60)
    #define IDAC_MAG_CFG_I2MAG_60XIUNIT                         ((uint8_t) 0x70)
    #define IDAC_MAG_CFG_I2MAG_70XIUNIT                         ((uint8_t) 0x80)
    #define IDAC_MAG_CFG_I2MAG_80XIUNIT                         ((uint8_t) 0x90)
    #define IDAC_MAG_CFG_I2MAG_90XIUNIT                         ((uint8_t) 0xA0)
    #define IDAC_MAG_CFG_I2MAG_100XIUNIT                        ((uint8_t) 0xB0)


    /* I1MAG field */
    #define IDAC_MAG_CFG_I1MAG_MASK                             ((uint8_t) 0x0F)
    #define IDAC_MAG_CFG_I1MAG_BITOFFSET                        (0)
    #define IDAC_MAG_CFG_I1MAG_DISABLED                         ((uint8_t) 0x00)    // DEFAULT
    #define IDAC_MAG_CFG_I1MAG_1XIUNIT                          ((uint8_t) 0x01)
    #define IDAC_MAG_CFG_I1MAG_10XIUNIT                         ((uint8_t) 0x02)
    #define IDAC_MAG_CFG_I1MAG_20XIUNIT                         ((uint8_t) 0x03)
    #define IDAC_MAG_CFG_I1MAG_30XIUNIT                         ((uint8_t) 0x04)
    #define IDAC_MAG_CFG_I1MAG_40XIUNIT                         ((uint8_t) 0x05)
    #define IDAC_MAG_CFG_I1MAG_50XIUNIT                         ((uint8_t) 0x06)
    #define IDAC_MAG_CFG_I1MAG_60XIUNIT                         ((uint8_t) 0x07)
    #define IDAC_MAG_CFG_I1MAG_70XIUNIT                         ((uint8_t) 0x08)
    #define IDAC_MAG_CFG_I1MAG_80XIUNIT                         ((uint8_t) 0x09)
    #define IDAC_MAG_CFG_I1MAG_90XIUNIT                         ((uint8_t) 0x0A)
    #define IDAC_MAG_CFG_I1MAG_100XIUNIT                        ((uint8_t) 0x0B)


/* Register 0x0E (IDAC_MUX_CFG) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        IUNIT       |                          I2MUX[2:0]                          |      RESERVED      |                          I1MUX[2:0]                          |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* IDAC_MUX_CFG register */
    #define IDAC_MUX_CFG_ADDRESS                                ((uint8_t) 0x0E)
    #define IDAC_MUX_CFG_DEFAULT                                ((uint8_t) 0x10)

    /* IUNIT field */
    #define IDAC_MUX_CFG_IUNIT_MASK                             ((uint8_t) 0x80)
    #define IDAC_MUX_CFG_IUNIT_BITOFFSET                        (7)
    #define IDAC_MUX_CFG_IUNIT_1MUA                             ((uint8_t) 0x00)    // DEFAULT
    #define IDAC_MUX_CFG_IUNIT_10MUA                            ((uint8_t) 0x80)

    /* I2MUX field */
    #define IDAC_MUX_CFG_I2MUX_MASK                             ((uint8_t) 0x70)
    #define IDAC_MUX_CFG_I2MUX_BITOFFSET                        (4)
    #define IDAC_MUX_CFG_I2MUX_AIN0                             ((uint8_t) 0x00)
    #define IDAC_MUX_CFG_I2MUX_AIN1                             ((uint8_t) 0x10)    // DEFAULT
    #define IDAC_MUX_CFG_I2MUX_AIN2                             ((uint8_t) 0x20)
    #define IDAC_MUX_CFG_I2MUX_AIN3                             ((uint8_t) 0x30)
    #define IDAC_MUX_CFG_I2MUX_AIN4                             ((uint8_t) 0x40)
    #define IDAC_MUX_CFG_I2MUX_AIN5                             ((uint8_t) 0x50)
    #define IDAC_MUX_CFG_I2MUX_AIN6                             ((uint8_t) 0x60)
    #define IDAC_MUX_CFG_I2MUX_AIN7                             ((uint8_t) 0x70)

    /* I1MUX field */
    #define IDAC_MUX_CFG_I1MUX_MASK                             ((uint8_t) 0x07)
    #define IDAC_MUX_CFG_I1MUX_BITOFFSET                        (0)
    #define IDAC_MUX_CFG_I1MUX_AIN0                             ((uint8_t) 0x00)    // DEFAULT
    #define IDAC_MUX_CFG_I1MUX_AIN1                             ((uint8_t) 0x01)
    #define IDAC_MUX_CFG_I1MUX_AIN2                             ((uint8_t) 0x02)
    #define IDAC_MUX_CFG_I1MUX_AIN3                             ((uint8_t) 0x03)
    #define IDAC_MUX_CFG_I1MUX_AIN4                             ((uint8_t) 0x04)
    #define IDAC_MUX_CFG_I1MUX_AIN5                             ((uint8_t) 0x05)
    #define IDAC_MUX_CFG_I1MUX_AIN6                             ((uint8_t) 0x06)
    #define IDAC_MUX_CFG_I1MUX_AIN7                             ((uint8_t) 0x07)


/* Register 0x0F (REG_MAP_CRC) definition
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |        Bit 7       |        Bit 6       |        Bit 5       |        Bit 4       |        Bit 3       |        Bit 2       |        Bit 1       |        Bit 0       |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 * |                                                                          REG_MAP_CRC_VAL[7:0]                                                                         |
 * |-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REG_MAP_CRC register */
    #define REG_MAP_CRC_ADDRESS                                 ((uint8_t) 0x0F)
    #define REG_MAP_CRC_DEFAULT                                 ((uint8_t) 0x00)

    /* REG_MAP_CRC_VAL field */
    #define REG_MAP_CRC_REG_MAP_CRC_VAL_MASK                    ((uint8_t) 0xFF)
    #define REG_MAP_CRC_REG_MAP_CRC_VAL_BITOFFSET               (0)

//**********************************************************************************
//
// Register macros
//
//**********************************************************************************
#define MASKED_REG_DATA(addr, mask)     (getRegisterValue(addr) & (mask))

/** Returns true if STATUS bit is set in CONFIG3 shadow register */
#define STATUS_ENABLED          ((bool) MASKED_REG_DATA(DIGITAL_CFG_ADDRESS, DIGITAL_CFG_STATUS_EN_MASK))

/** Returns true if SPI_CRC bit is set in CONFIG3 shadow register */
#define SPI_CRC_ENABLED         ((bool) MASKED_REG_DATA(DIGITAL_CFG_ADDRESS, DIGITAL_CFG_SPI_CRC_EN_MASK))

#define RESOLUTION_IS_16_BIT    ((bool) MASKED_REG_DATA(DEVICE_ID_ADDRESS, DEVICE_IS_16_BIT_MASK))

#define RESOLUTION_IS_24_BIT   ((bool) MASKED_REG_DATA(DEVICE_ID_ADDRESS, DEVICE_IS_24_BIT_MASK))

#endif /* ADS122S1X_PAGE0_H_ */
