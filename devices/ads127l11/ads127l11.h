/*
 * \brief ADS127L11 Descriptor
 *
 * \copyright Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef ADS127L11_H_
#define ADS127L11_H_

// Standard libraries
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

// Custom libraries
#include "hal.h"
#include "crc8.h"


//**********************************************************************************
//
// Constants
//
//**********************************************************************************

// Define SPI buffer length...
// Longest frame is 5 bytes: STATUS + DATA + CMD_BYTE1 + CMD_BYTE2 + CRC
// SPI buffer length can be reduced to 3 or 4 if STATUS and/or CRC bytes are not used
#define SPI_BUFFER_SIZE                         (5)

#define NUM_REGISTERS                           (16)

#define OPCODE_NOP                              ((uint8_t) 0x00)
#define OPCODE_RREG                             ((uint8_t) 0x40)
#define OPCODE_WREG                             ((uint8_t) 0x80)

#define CRC8_POLYNOMIAL                         ((uint8_t) 0x07)
#define CRC8_INITIAL_SEED                       ((uint8_t) 0xFF)


//**********************************************************************************
//
// Typedefs
//
//**********************************************************************************
typedef struct {
    uint8_t status;
    uint8_t crc;
    int32_t data;
} adc_channel_t;


//**********************************************************************************
//
// Function prototypes
//
//**********************************************************************************
void        adcStartup(void);
int32_t     readData(adc_channel_t *dataStruct);
uint8_t     readSingleRegister(uint8_t address);
void        readMultipleRegisters(uint8_t startAddress, uint8_t count);
void        writeSingleRegister(uint8_t address, uint8_t data);

void        clearSTATUSflags(void);
uint8_t     getRegisterValue(uint8_t address);
bool        isValidCrcOut(void);
void        resetDeviceByCommand(void);
void        resetDeviceByPattern(void);
void        resetSPIframe(void);
void        restoreRegisterDefaults(void);
int32_t     signExtend(const uint8_t dataBytes[]);


//**********************************************************************************
//
// Register definitions
//
//**********************************************************************************


/* Register 0x00 (DEV_ID) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                           DEV_ID[7:0]                                                         |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DEV_ID register */
    #define DEV_ID_ADDRESS                                              ((uint8_t) 0x00)
    #define DEV_ID_DEFAULT                                              ((uint8_t) 0x00)

    /* DEV_ID field */
    #define DEV_ID_ADS127L11                                            ((uint8_t) 0x00)    // DEFAULT


/* Register 0x01 (REV_ID) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                           REV_ID[7:0]                                                         |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REV_ID register */
    #define REV_ID_ADDRESS                                              ((uint8_t) 0x01)

    /* REV_ID field */
    // The die revision ID can change during device production without notice


/* Register 0x02 (STATUS) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    CS_MODE    |    ALV_FLAG   |    POR_FLAG   |    SPI_ERR    |    REG_ERR    |     ADC_ERR   |   MOD_FLAG    |      DRDY     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STATUS register */
    #define STATUS_ADDRESS                                              ((uint8_t) 0x02)
    #define STATUS_DEFAULT                                              ((uint8_t) 0x00)
    #define STATUS_CLEAR_ERRORS                                         ((uint8_t) 0x70)

    /* CS Mode field */
    #define STATUS_CS_MODE_MASK                                         ((uint8_t) 0x80)
    #define STATUS_CS_MODE_0                                            ((uint8_t) 0x00)    // DEFAULT
    #define STATUS_CS_MODE_1                                            ((uint8_t) 0x80)

    /* Analog Supply Low Voltage field */
    #define STATUS_ALV_FLAG_MASK                                        ((uint8_t) 0x40)
    #define STATUS_ALV_FLAG_0                                           ((uint8_t) 0x00)    // DEFAULT
    #define STATUS_ALV_FLAG_1                                           ((uint8_t) 0x40)

    /* POR Flag field */
    #define STATUS_POR_FLAG_MASK                                        ((uint8_t) 0x20)
    #define STATUS_POR_FLAG_0                                           ((uint8_t) 0x00)    // DEFAULT
    #define STATUS_POR_FLAG_1                                           ((uint8_t) 0x20)

    /* SPI CRC Error field */
    #define STATUS_SPI_ERR_MASK                                         ((uint8_t) 0x10)
    #define STATUS_SPI_ERR_0                                            ((uint8_t) 0x00)    // DEFAULT
    #define STATUS_SPI_ERR_1                                            ((uint8_t) 0x10)

    /* Register Map CRC Error field */
    #define STATUS_REG_ERR_MASK                                         ((uint8_t) 0x08)
    #define STATUS_REG_ERR_0                                            ((uint8_t) 0x00)    // DEFAULT
    #define STATUS_REG_ERR_1                                            ((uint8_t) 0x08)

    /* Internal ADC Error field */
    #define STATUS_ADC_ERR_MASK                                         ((uint8_t) 0x04)
    #define STATUS_ADC_ERR_0                                            ((uint8_t) 0x00)    // DEFAULT
    #define STATUS_ADC_ERR_1                                            ((uint8_t) 0x04)

    /* Modulator Overload Flag */
    #define STATUS_MOD_FLAG_MASK                                        ((uint8_t) 0x02)
    #define STATUS_MOD_FLAG_0                                           ((uint8_t) 0x00)    // DEFAULT
    #define STATUS_MOD_FLAG_1                                           ((uint8_t) 0x02)

    /* Data Ready Bit */
    #define STATUS_DRDY_MASK                                            ((uint8_t) 0x01)
    #define STATUS_DRDY_0                                               ((uint8_t) 0x00)    // DEFAULT
    #define STATUS_DRDY_1                                               ((uint8_t) 0x01)


/* Register 0x03 (CONTROL) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                           RESET[5:0]                                          |     START     |      STOP     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CONTROL register */
    #define CONTROL_ADDRESS                                            ((uint8_t) 0x03)
    #define CONTROL_DEFAULT                                            ((uint8_t) 0x00)

    /* RESET field */
    #define CONTROL_RESET_MASK                                         ((uint8_t) 0xFC)
    #define CONTROL_RESET_COMMAND                                      ((uint8_t) 0x58)

    /* START field */
    #define CONTROL_START_MASK                                         ((uint8_t) 0x02)
    #define CONTROL_START_0                                            ((uint8_t) 0x00)    // DEFAULT
    #define CONTROL_START_1                                            ((uint8_t) 0x02)

    /* STOP field */
    #define CONTROL_STOP_MASK                                          ((uint8_t) 0x01)
    #define CONTROL_STOP_0                                             ((uint8_t) 0x00)    // DEFAULT
    #define CONTROL_STOP_1                                             ((uint8_t) 0x01)


/* Register 0x04 (MUX) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                           RESERVED                                            |             MUX[1:0]          |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* MUX register */
    #define MUX_ADDRESS                                                 ((uint8_t) 0x04)
    #define MUX_DEFAULT                                                 ((uint8_t) 0x00)

    /* MUX field */
    #define MUX_MUX_MASK                                                ((uint8_t) 0x03)
    #define MUX_MUX_NORMAL                                              ((uint8_t) 0x00)    // DEFAULT
    #define MUX_MUX_INVERTED                                            ((uint8_t) 0x01)
    #define MUX_MUX_OFFSET_TEST                                         ((uint8_t) 0x02)
    #define MUX_MUX_CM_TEST                                             ((uint8_t) 0x03)


/* Register 0x05 (CONFIG1) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    RESERVED   |    REF_RNG    |    INP_RNG    |    VCM        |    REFP_BUF   |    RESERVED   |    AINP_BUF   |    AINN_BUFF  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CONFIG1 register */
    #define CONFIG1_ADDRESS                                             ((uint8_t) 0x05)
    #define CONFIG1_DEFAULT                                             ((uint8_t) 0x00)

    /* Voltage Reference Range field */
    #define CONFIG1_REF_RNG_MASK                                        ((uint8_t) 0x40)
    #define CONFIG1_REF_RNG_0                                           ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG1_REF_RNG_1                                           ((uint8_t) 0x40)

    /* Input Range field */
    #define CONFIG1_INP_RNG_MASK                                        ((uint8_t) 0x20)
    #define CONFIG1_INP_RNG_0                                           ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG1_INP_RNG_1                                           ((uint8_t) 0x20)

    /* VCM Output field */
    #define CONFIG1_VCM_MASK                                            ((uint8_t) 0x10)
    #define CONFIG1_VCM_0                                               ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG1_VCM_1                                               ((uint8_t) 0x10)

    /* Reference Positive Buffer field */
    #define CONFIG1_REFP_BUFF_MASK                                      ((uint8_t) 0x08)
    #define CONFIG1_REFP_BUFF_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG1_REFP_BUFF_1                                         ((uint8_t) 0x08)

    /* Analog Input Positive Buffer field */
    #define CONFIG1_AINP_BUF_MASK                                       ((uint8_t) 0x02)
    #define CONFIG1_AINP_BUF_0                                          ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG1_AINP_BUF_1                                          ((uint8_t) 0x02)

    /* VCM Output field */
    #define CONFIG1_AINN_BUF_MASK                                       ((uint8_t) 0x01)
    #define CONFIG1_AINN_BUF_0                                          ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG1_AINN_BUF_1                                          ((uint8_t) 0x01)


/* Register 0x06 (CONFIG2) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    EXT_RNG    |    RESERVED   |    SDO_MODE   |        START_MODE[1:0]        |   SPEED_MODE  |    STBY_MODE  |     PWDN      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CONFIG2 register */
    #define CONFIG2_ADDRESS                                             ((uint8_t) 0x06)
    #define CONFIG2_DEFAULT                                             ((uint8_t) 0x00)

    /* Extended Input Range field */
    #define CONFIG2_EXT_RNG_MASK                                        ((uint8_t) 0x80)
    #define CONFIG2_EXT_RNG_0                                           ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG2_EXT_RNG_1                                           ((uint8_t) 0x80)

    /* SDO/nDRDY Mode field */
    #define CONFIG2_SDO_MODE_MASK                                       ((uint8_t) 0x20)
    #define CONFIG2_SDO_MODE_0                                          ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG2_SDO_MODE_1                                          ((uint8_t) 0x20)

    /* START Mode field */
    #define CONFIG2_START_MODE_MASK                                     ((uint8_t) 0x18)
    #define CONFIG2_START_MODE_START_STOP                               ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG2_START_MODE_ONESHOT                                  ((uint8_t) 0x08)
    #define CONFIG2_START_MODE_SYNC                                     ((uint8_t) 0x10)
    #define CONFIG2_START_MODE_RESERVED                                 ((uint8_t) 0x18)

    /* Speed Mode field */
    #define CONFIG2_SPEED_MODE_MASK                                     ((uint8_t) 0x04)
    #define CONFIG2_SPEED_MODE_0                                        ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG2_SPEED_MODE_1                                        ((uint8_t) 0x04)

    /* Standby Mode field */
    #define CONFIG2_STBY_MODE_MASK                                      ((uint8_t) 0x02)
    #define CONFIG2_STBY_MODE_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG2_STBY_MODE_1                                         ((uint8_t) 0x02)

    /* Software Power Down field */
    #define CONFIG2_PWDN_MASK                                           ((uint8_t) 0x01)
    #define CONFIG2_PWDN_0                                              ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG2_PWDN_1                                              ((uint8_t) 0x01)


/* Register 0x07 (CONFIG3) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                  DELAY[2:0]                   |                            FILTER[4:0]                                        |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CONFIG3 register */
    #define CONFIG3_ADDRESS                                             ((uint8_t) 0x07)
    #define CONFIG3_DEFAULT                                             ((uint8_t) 0x00)

    /* Conversion-Start Delay Time field */
    #define CONFIG3_DELAY_MASK                                          ((uint8_t) 0xE0)
    #define CONFIG3_DELAY_0                                             ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG3_DELAY_4                                             ((uint8_t) 0x20)
    #define CONFIG3_DELAY_8                                             ((uint8_t) 0x40)
    #define CONFIG3_DELAY_16                                            ((uint8_t) 0x60)
    #define CONFIG3_DELAY_32                                            ((uint8_t) 0x80)
    #define CONFIG3_DELAY_128                                           ((uint8_t) 0xA0)
    #define CONFIG3_DELAY_512                                           ((uint8_t) 0xC0)
    #define CONFIG3_DELAY_1024                                          ((uint8_t) 0xE0)


    /* Filter field */
    #define CONFIG3_FILTER_MASK                                         ((uint8_t) 0x1F)


/* Register 0x08 (CONFIG4) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    CLK_SEL    |    CLK_DIV    |    OUT_DRV    |    RESERVED   |     DATA      |    SPI_CRC    |     REG_CRC   |     STATUS    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CONFIG4 register */
    #define CONFIG4_ADDRESS                                             ((uint8_t) 0x08)
    #define CONFIG4_DEFAULT                                             ((uint8_t) 0x00)

    /* Clock Selection field */
    #define CONFIG4_CLK_SEL_MASK                                        ((uint8_t) 0x80)
    #define CONFIG4_CLK_SEL_0                                           ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG4_CLK_SEL_1                                           ((uint8_t) 0x80)

    /* External Clock Division Selection field */
    #define CONFIG4_CLK_DIV_MASK                                        ((uint8_t) 0x40)
    #define CONFIG4_CLK_DIV_0                                           ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG4_CLK_DIV_1                                           ((uint8_t) 0x40)

    /* Digital OUtput Drive Selection field */
    #define CONFIG4_OUT_DRV_MASK                                        ((uint8_t) 0x20)
    #define CONFIG4_OUT_DRV_0                                           ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG4_OUT_DRV_1                                           ((uint8_t) 0x20)

    /* Data Resolution Selection field */
    #define CONFIG4_DATA_MASK                                           ((uint8_t) 0x08)
    #define CONFIG4_DATA_0                                              ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG4_DATA_1                                              ((uint8_t) 0x08)

    /* SPI CRC Enable field */
    #define CONFIG4_SPI_CRC_MASK                                        ((uint8_t) 0x04)
    #define CONFIG4_SPI_CRC_0                                           ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG4_SPI_CRC_1                                           ((uint8_t) 0x04)

    /* Register Map CRC Enable field */
    #define CONFIG4_REG_CRC_MASK                                        ((uint8_t) 0x02)
    #define CONFIG4_REG_CRC_0                                           ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG4_REG_CRC_1                                           ((uint8_t) 0x02)

    /* Status Byte Output Enable field */
    #define CONFIG4_STATUS_MASK                                         ((uint8_t) 0x01)
    #define CONFIG4_STATUS_0                                            ((uint8_t) 0x00)    // DEFAULT
    #define CONFIG4_STATUS_1                                            ((uint8_t) 0x01)


/* Register 0x09 (OFFSET2) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       OFFSET[23:16]                                                           |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFFSET2 register */
    #define OFFSET2_ADDRESS                                             ((uint8_t) 0x09)
    #define OFFSET2_DEFAULT                                             ((uint8_t) 0x00)


/* Register 0x0A (OFFSET1) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       OFFSET[15:8]                                                            |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFFSET1 register */
    #define OFFSET1_ADDRESS                                             ((uint8_t) 0x0A)
    #define OFFSET1_DEFAULT                                             ((uint8_t) 0x00)


/* Register 0x0B (OFFSET0) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       OFFSET[7:0]                                                             |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* OFFSET0 register */
    #define OFFSET0_ADDRESS                                             ((uint8_t) 0x0B)
    #define OFFSET0_DEFAULT                                             ((uint8_t) 0x00)


/* Register 0x0C (GAIN2) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       GAIN[23:16]                                                             |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAIN2 register */
    #define GAIN2_ADDRESS                                               ((uint8_t) 0x0C)
    #define GAIN2_DEFAULT                                               ((uint8_t) 0x40)


/* Register 0x0D (GAIN1) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       GAIN[15:8]                                                              |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAIN1 register */
    #define GAIN1_ADDRESS                                               ((uint8_t) 0x0D)
    #define GAIN1_DEFAULT                                               ((uint8_t) 0x00)


/* Register 0x0E (GAIN0) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       GAIN[7:0]                                                               |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAIN0 register */
    #define GAIN0_ADDRESS                                               ((uint8_t) 0x0E)
    #define GAIN0_DEFAULT                                               ((uint8_t) 0x00)


/* Register 0x0F (CRC) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CRC[7:0]                                                                |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CRC register */
    #define CRC_ADDRESS                                                 ((uint8_t) 0x0F)
    #define CRC_DEFAULT                                                 ((uint8_t) 0x00)


//**********************************************************************************
//
// Register macros
//
//**********************************************************************************
#define MASKED_REG_DATA(addr, mask)     (getRegisterValue(addr) & (mask))

/** Returns true if SPI_ERR bit is set in STATUS1 shadow register */
#define SPI_3_WIRE_MODE         ((bool) MASKED_REG_DATA(STATUS_ADDRESS, STATUS_CS_MODE_MASK))

/** Returns true if SPI_ERR bit is set in STATUS1 shadow register */
#define SPI_ERR_FLAG            ((bool) MASKED_REG_DATA(STATUS_ADDRESS, STATUS_SPI_ERR_MASK))

/** Returns true if DATA bit is set in CONFIG4 shadow register */
#define RESOLUTION_IS_16_BIT    ((bool) MASKED_REG_DATA(CONFIG4_ADDRESS, CONFIG4_DATA_MASK))

/** Returns true if STATUS bit is set in CONFIG4 shadow register */
#define STATUS_ENABLED          ((bool) MASKED_REG_DATA(CONFIG4_ADDRESS, CONFIG4_STATUS_1))

/** Returns true if SPI_CRC bit is set in CONFIG4 shadow register */
#define SPI_CRC_ENABLED         ((bool) MASKED_REG_DATA(CONFIG4_ADDRESS, CONFIG4_SPI_CRC_1))

/** Returns true if REG_CRC bit is set in CONFIG4 shadow register */
#define REG_CRC_ENABLED         ((bool) MASKED_REG_DATA(CONFIG4_ADDRESS, CONFIG4_REG_CRC_MASK))


#endif /* ADS127L11_H_ */
