/*
 * \brief ADS127L21 Descriptor
 *
 * \copyright Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef ADS127L21_H_
#define ADS127L21_H_

// Standard libraries
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

// Custom libraries
#include "hal.h"
#include "crc8.h"
#include "fir_coeff.h"
#include "iir_coeff.h"


//**********************************************************************************
//
// Constants
//
//**********************************************************************************

// Define SPI buffer length...
// Longest frame is 5 bytes: STATUS + DATA + CMD_BYTE1 + CMD_BYTE2 + CRC
// SPI buffer length can be reduced to 3 or 4 if STATUS and/or CRC bytes are not used
#define SPI_BUFFER_SIZE                         (5)

#define NUM_REGISTERS                           (24)

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
void        enableRegisterMapCrc(bool enable);
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
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                          DEV_ID[7:0]                                          |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* DEV_ID register */
    #define DEV_ID_ADDRESS                                      ((uint8_t) 0x00)
    #define DEV_ID_DEFAULT                                      ((uint8_t) 0x02)

    /* DEV_ID field */
    #define DEV_ID_MASK                                         ((uint8_t) 0xFF)
    #define DEV_ID_BITOFFSET                                    (0)
    #define DEV_ID_ADS127L21                                    ((uint8_t) 0x02)    // DEFAULT


/* Register 0x01 (REV_ID) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                          REV_ID[7:0]                                          |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* REV_ID register */
    #define REV_ID_ADDRESS                                      ((uint8_t) 0x01)

    /* REV_ID field */
    #define REV_ID_MASK                                         ((uint8_t) 0xFF)
    #define REV_ID_BITOFFSET                                    (0)


/* Register 0x02 (STATUS1) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |  CS_MODE  |  ALV_FLAG |  POR_FLAG |  SPI_ERR  | G_CRC_ERR |  ADC_ERR  |  MOD_FLAG |    DRDY   |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* STATUS1 register */
    #define STATUS1_ADDRESS                                     ((uint8_t) 0x02)
    #define STATUS1_DEFAULT                                     ((uint8_t) 0x60)
    #define STATUS1_CLEAR_ERRORS                                ((uint8_t) 0x70)

    /* CS_MODE field */
    #define STATUS1_CS_MODE_MASK                                ((uint8_t) 0x80)
    #define STATUS1_CS_MODE_BITOFFSET                           (7)

    /* ALV_FLAG field */
    #define STATUS1_ALV_FLAG_MASK                               ((uint8_t) 0x40)
    #define STATUS1_ALV_FLAG_BITOFFSET                          (6)

    /* POR_FLAG field */
    #define STATUS1_POR_FLAG_MASK                               ((uint8_t) 0x20)
    #define STATUS1_POR_FLAG_BITOFFSET                          (5)

    /* SPI_ERR field */
    #define STATUS1_SPI_ERR_MASK                                ((uint8_t) 0x10)
    #define STATUS1_SPI_ERR_BITOFFSET                           (4)

    /* G_CRC_ERR field */
    #define STATUS1_CRC_ERR_MASK                                ((uint8_t) 0x08)
    #define STATUS1_CRC_ERR_BITOFFSET                           (3)

    /* ADC_ERR field */
    #define STATUS1_ADC_ERR_MASK                                ((uint8_t) 0x04)
    #define STATUS1_ADC_ERR_BITOFFSET                           (2)

    /* MOD_FLAG field */
    #define STATUS1_MOD_FLAG_MASK                               ((uint8_t) 0x02)
    #define STATUS1_MOD_FLAG_BITOFFSET                          (1)

    /* DRDY field */
    #define STATUS1_DRDY_MASK                                   ((uint8_t) 0x01)
    #define STATUS1_DRDY_BITOFFSET                              (0)


/* Register 0x03 (STATUS2) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                       RESERVED[4:0]                       | I_CRC_ERR | F_CRC_ERR | M_CRC_ERR |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* STATUS2 register */
    #define STATUS2_ADDRESS                                     ((uint8_t) 0x03)
    #define STATUS2_DEFAULT                                     ((uint8_t) 0x00)

    /* I_CRC_ERR field */
    #define STATUS2_I_CRC_ERR_MASK                              ((uint8_t) 0x04)
    #define STATUS2_I_CRC_ERR_BITOFFSET                         (2)

    /* F_CRC_ERR field */
    #define STATUS2_F_CRC_ERR_MASK                              ((uint8_t) 0x02)
    #define STATUS2_F_CRC_ERR_BITOFFSET                         (1)

    /* M_CRC_ERR field */
    #define STATUS2_M_CRC_ERR_MASK                              ((uint8_t) 0x01)
    #define STATUS2_M_CRC_ERR_BITOFFSET                         (0)


/* Register 0x04 (CONTROL) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                               RESET[5:0]                              |   START   |    STOP   |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* CONTROL register */
    #define CONTROL_ADDRESS                                     ((uint8_t) 0x04)
    #define CONTROL_DEFAULT                                     ((uint8_t) 0x00)

    /* RESET field */
    #define CONTROL_RESET_MASK                                  ((uint8_t) 0xFC)
    #define CONTROL_RESET_BITOFFSET                             (2)
    #define CONTROL_RESET_COMMAND                               ((uint8_t) 0x58)

    /* START field */
    #define CONTROL_START_MASK                                  ((uint8_t) 0x02)
    #define CONTROL_START_BITOFFSET                             (1)

    /* STOP field */
    #define CONTROL_STOP_MASK                                   ((uint8_t) 0x01)
    #define CONTROL_STOP_BITOFFSET                              (0)


/* Register 0x05 (MUX) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                             RESERVED[5:0]                             |        MUX[1:0]       |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* MUX register */
    #define MUX_ADDRESS                                         ((uint8_t) 0x05)
    #define MUX_DEFAULT                                         ((uint8_t) 0x00)

    /* MUX field */
    #define MUX_MASK                                            ((uint8_t) 0x03)
    #define MUX_BITOFFSET                                       (0)
    #define MUX_NORMAL_POLARITY                                 ((uint8_t) 0x00)    // Default
    #define MUX_INVERTED_POLARITY                               ((uint8_t) 0x01)
    #define MUX_OFFSET_TEST                                     ((uint8_t) 0x02)
    #define MUX_CM_TEST                                         ((uint8_t) 0x03)


/* Register 0x06 (CONFIG1) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |    DATA   |  EXT_RNG  |  REF_RNG  |  INP_RNG  |    VCM    |  REFP_BUF |  AINP_BUF |  AINN_BUF |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* CONFIG1 register */
    #define CONFIG1_ADDRESS                                     ((uint8_t) 0x06)
    #define CONFIG1_DEFAULT                                     ((uint8_t) 0x00)

    /* DATA field */
    #define CONFIG1_DATA_MASK                                   ((uint8_t) 0x80)
    #define CONFIG1_DATA_BITOFFSET                              (7)

    /* EXT_RNG field */
    #define CONFIG1_EXT_RNG_MASK                                ((uint8_t) 0x40)
    #define CONFIG1_EXT_RNG_BITOFFSET                           (6)

    /* REF_RNG field */
    #define CONFIG1_REF_RNG_MASK                                ((uint8_t) 0x20)
    #define CONFIG1_REF_RNG_BITOFFSET                           (5)

    /* INP_RNG field */
    #define CONFIG1_INP_RNG_MASK                                ((uint8_t) 0x10)
    #define CONFIG1_INP_RNG_BITOFFSET                           (4)

    /* VCM field */
    #define CONFIG1_VCM_MASK                                    ((uint8_t) 0x08)
    #define CONFIG1_VCM_BITOFFSET                               (3)

    /* REFP_BUF field */
    #define CONFIG1_REFP_BUF_MASK                               ((uint8_t) 0x04)
    #define CONFIG1_REFP_BUF_BITOFFSET                          (2)

    /* AINP_BUF field */
    #define CONFIG1_AINP_BUF_MASK                               ((uint8_t) 0x02)
    #define CONFIG1_AINP_BUF_BITOFFSET                          (1)

    /* AINN_BUF field */
    #define CONFIG1_AINN_BUF_MASK                               ((uint8_t) 0x01)
    #define CONFIG1_AINN_BUF_BITOFFSET                          (0)


/* Register 0x07 (CONFIG2) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |     RESERVED[1:0]     |    START_MODE[1:0]    |    SPEED_MODE[1:0]    | STBY_MODE |    PWDN   |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* CONFIG2 register */
    #define CONFIG2_ADDRESS                                     ((uint8_t) 0x07)
    #define CONFIG2_DEFAULT                                     ((uint8_t) 0x08)

    /* START_MODE field */
    #define CONFIG2_START_MODE_MASK                             ((uint8_t) 0x30)
    #define CONFIG2_START_MODE_BITOFFSET                        (4)
    #define CONFIG2_START_MODE_STARTSTOP                        ((uint8_t) 0x00)    // Default
    #define CONFIG2_START_MODE_ONESHOT                          ((uint8_t) 0x10)
    #define CONFIG2_START_MODE_SYNCHRONIZED                     ((uint8_t) 0x20)

    /* SPEED_MODE field */
    #define CONFIG2_SPEED_MODE_MASK                             ((uint8_t) 0x0C)
    #define CONFIG2_SPEED_MODE_BITOFFSET                        (2)
    #define CONFIG2_SPEED_MODE_LOW                              ((uint8_t) 0x00)
    #define CONFIG2_SPEED_MODE_MID                              ((uint8_t) 0x04)
    #define CONFIG2_SPEED_MODE_HIGH                             ((uint8_t) 0x08)    // Default
    #define CONFIG2_SPEED_MODE_MAX                              ((uint8_t) 0x0C)

    /* STBY_MODE field */
    #define CONFIG2_STBY_MODE_MASK                              ((uint8_t) 0x02)
    #define CONFIG2_STBY_MODE_BITOFFSET                         (1)

    /* PWDN field */
    #define CONFIG2_PWDN_MASK                                   ((uint8_t) 0x01)
    #define CONFIG2_PWDN_BITOFFSET                              (0)


/* Register 0x08 (CONFIG3) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |  CLK_SEL  |      CLK_DIV[1:0]     |  OUT_DRV  |  RESERVED |  SPI_CRC  |  REG_CRC  |   STATUS  |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* CONFIG3 register */
    #define CONFIG3_ADDRESS                                     ((uint8_t) 0x08)
    #define CONFIG3_DEFAULT                                     ((uint8_t) 0x00)

    /* CLK_SEL field */
    #define CONFIG3_CLK_SEL_MASK                                ((uint8_t) 0x80)
    #define CONFIG3_CLK_SEL_BITOFFSET                           (7)

    /* CLK_DIV field */
    #define CONFIG3_CLK_DIV_MASK                                ((uint8_t) 0x60)
    #define CONFIG3_CLK_DIV_BITOFFSET                           (5)
    #define CONFIG3_CLK_DIV_1                                   ((uint8_t) 0x00)    // Default
    #define CONFIG3_CLK_DIV_2                                   ((uint8_t) 0x20)
    #define CONFIG3_CLK_DIV_8                                   ((uint8_t) 0x40)
    #define CONFIG3_CLK_DIV_16                                  ((uint8_t) 0x40)

    /* OUT_DRV field */
    #define CONFIG3_OUT_DRV_MASK                                ((uint8_t) 0x10)
    #define CONFIG3_OUT_DRV_BITOFFSET                           (4)

    /* SPI_CRC field */
    #define CONFIG3_SPI_CRC_MASK                                ((uint8_t) 0x04)
    #define CONFIG3_SPI_CRC_BITOFFSET                           (2)

    /* REG_CRC field */
    #define CONFIG3_REG_CRC_MASK                                ((uint8_t) 0x02)
    #define CONFIG3_REG_CRC_BITOFFSET                           (1)

    /* STATUS field */
    #define CONFIG3_STATUS_MASK                                 ((uint8_t) 0x01)
    #define CONFIG3_STATUS_BITOFFSET                            (0)


/* Register 0x09 (FILTER1) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |           FLTR_SEL[2:0]           |                       FLTR_OSR[4:0]                       |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* FILTER1 register */
    #define FILTER1_ADDRESS                                     ((uint8_t) 0x09)
    #define FILTER1_DEFAULT                                     ((uint8_t) 0x00)

    /* FLTR_SEL field */
    #define FILTER1_FLTR_SEL_MASK                               ((uint8_t) 0xE0)
    #define FILTER1_FLTR_SEL_BITOFFSET                          (5)
    #define FILTER1_FLTR_SEL_PRESET_OR_SINC4                    ((uint8_t) 0x00)    // Default
    #define FILTER1_FLTR_SEL_SINC3                              ((uint8_t) 0x20)
    #define FILTER1_FLTR_SEL_PROGRAMMABLE                       ((uint8_t) 0xE0)

    /* FLTR_OSR field */
    #define FILTER1_FLTR_OSR_MASK                               ((uint8_t) 0x1F)
    #define FILTER1_FLTR_OSR_BITOFFSET                          (0)
    #define FILTER1_FLTR_OSR_WIDEBAND_32                        ((uint8_t) 0x00)    // Default
    #define FILTER1_FLTR_OSR_WIDEBAND_64                        ((uint8_t) 0x01)
    #define FILTER1_FLTR_OSR_WIDEBAND_128                       ((uint8_t) 0x02)
    #define FILTER1_FLTR_OSR_WIDEBAND_256                       ((uint8_t) 0x03)
    #define FILTER1_FLTR_OSR_WIDEBAND_512                       ((uint8_t) 0x04)
    #define FILTER1_FLTR_OSR_WIDEBAND_1024                      ((uint8_t) 0x05)
    #define FILTER1_FLTR_OSR_WIDEBAND_2048                      ((uint8_t) 0x06)
    #define FILTER1_FLTR_OSR_WIDEBAND_4096                      ((uint8_t) 0x07)
    #define FILTER1_FLTR_OSR_SINCx_12                           ((uint8_t) 0x08)
    #define FILTER1_FLTR_OSR_SINCx_16                           ((uint8_t) 0x09)
    #define FILTER1_FLTR_OSR_SINCx_24                           ((uint8_t) 0x0A)
    #define FILTER1_FLTR_OSR_SINCx_32                           ((uint8_t) 0x0B)
    #define FILTER1_FLTR_OSR_SINCx_64                           ((uint8_t) 0x0C)
    #define FILTER1_FLTR_OSR_SINCx_128                          ((uint8_t) 0x0D)
    #define FILTER1_FLTR_OSR_SINCx_256_167                      ((uint8_t) 0x0E)
    #define FILTER1_FLTR_OSR_SINCx_333_256                      ((uint8_t) 0x0F)
    #define FILTER1_FLTR_OSR_SINCx_512_333                      ((uint8_t) 0x10)
    #define FILTER1_FLTR_OSR_SINCx_667_512                      ((uint8_t) 0x11)
    #define FILTER1_FLTR_OSR_SINCx_1024_667                     ((uint8_t) 0x12)
    #define FILTER1_FLTR_OSR_SINCx_1333_1024                    ((uint8_t) 0x13)
    #define FILTER1_FLTR_OSR_SINCx_2048_1333                    ((uint8_t) 0x14)
    #define FILTER1_FLTR_OSR_SINCx_2667_2048                    ((uint8_t) 0x15)
    #define FILTER1_FLTR_OSR_SINCx_4096_2667                    ((uint8_t) 0x16)
    #define FILTER1_FLTR_OSR_SINCx_5333_4096                    ((uint8_t) 0x17)
    #define FILTER1_FLTR_OSR_SINCx_26667_13333                  ((uint8_t) 0x18)
    #define FILTER1_FLTR_OSR_SINCx_32000_16000                  ((uint8_t) 0x19)
    #define FILTER1_FLTR_OSR_SINCx_96000_48000                  ((uint8_t) 0x1A)
    #define FILTER1_FLTR_OSR_SINCx_160000_80000                 ((uint8_t) 0x1B)
    #define FILTER1_FLTR_OSR_SINCx_SINC1_26656_13334            ((uint8_t) 0x1C)
    #define FILTER1_FLTR_OSR_SINCx_SINC1_32000_16000            ((uint8_t) 0x1D)
    #define FILTER1_FLTR_OSR_SINCx_SINC1_96000_48000            ((uint8_t) 0x1E)
    #define FILTER1_FLTR_OSR_SINCx_SINC1_160000_80000           ((uint8_t) 0x1F)


/* Register 0x0A (FILTER2) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |  RESERVED |             DELAY[2:0]            |  FLTR_SEQ |  FIR2_DIS |  FIR3_DIS |  IIR_DIS  |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* FILTER2 register */
    #define FILTER2_ADDRESS                                     ((uint8_t) 0x0A)
    #define FILTER2_DEFAULT                                     ((uint8_t) 0x01)

    /* DELAY field */
    #define FILTER2_DELAY_MASK                                  ((uint8_t) 0x70)
    #define FILTER2_DELAY_BITOFFSET                             (4)
    #define FILTER2_DELAY_0_fMOD                                ((uint8_t) 0x00)    // Default
    #define FILTER2_DELAY_4_fMOD                                ((uint8_t) 0x10)
    #define FILTER2_DELAY_8_fMOD                                ((uint8_t) 0x20)
    #define FILTER2_DELAY_16_fMOD                               ((uint8_t) 0x30)
    #define FILTER2_DELAY_32_fMOD                               ((uint8_t) 0x40)
    #define FILTER2_DELAY_128_fMOD                              ((uint8_t) 0x50)
    #define FILTER2_DELAY_512_fMOD                              ((uint8_t) 0x60)
    #define FILTER2_DELAY_1024_fMOD                             ((uint8_t) 0x70)

    /* FLTR_SEQ field */
    #define FILTER2_FLTR_SEQ_MASK                               ((uint8_t) 0x08)
    #define FILTER2_FLTR_SEQ_BITOFFSET                          (3)

    /* FIR2_DIS field */
    #define FILTER2_FIR2_DIS_MASK                               ((uint8_t) 0x04)
    #define FILTER2_FIR2_DIS_BITOFFSET                          (2)

    /* FIR3_DIS field */
    #define FILTER2_FIR3_DIS_MASK                               ((uint8_t) 0x02)
    #define FILTER2_FIR3_DIS_BITOFFSET                          (1)

    /* IIR_DIS field */
    #define FILTER2_IIR_DIS_MASK                                ((uint8_t) 0x01)    // Default
    #define FILTER2_IIR_DIS_BITOFFSET                           (0)


/* Register 0x0B (FILTER3) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                       RESERVED[4:0]                       |  OUT2_SEL |     DATA_MODE[1:0]    |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* FILTER3 register */
    #define FILTER3_ADDRESS                                     ((uint8_t) 0x0B)
    #define FILTER3_DEFAULT                                     ((uint8_t) 0x01)

    /* OUT2_SEL field */
    #define FILTER3_OUT2_SEL_MASK                               ((uint8_t) 0x04)
    #define FILTER3_OUT2_SEL_BITOFFSET                          (2)

    /* DATA_MODE field */
    #define FILTER3_DATA_MODE_MASK                              ((uint8_t) 0x03)
    #define FILTER3_DATA_MODE_BITOFFSET                         (0)
    #define FILTER3_DATA_MODE_DATA                              ((uint8_t) 0x00)
    #define FILTER3_DATA_MODE_DUAL                              ((uint8_t) 0x01)    // Default
    #define FILTER3_DATA_MODE_DATA_ACTIVE                       ((uint8_t) 0x02)


/* Register 0x0C (OFFSET2) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                         OFFSET[23:16]                                         |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* OFFSET2 register */
    #define OFFSET2_ADDRESS                                     ((uint8_t) 0x0C)
    #define OFFSET2_DEFAULT                                     ((uint8_t) 0x00)

    /* OFFSET field */
    #define OFFSET2_OFFSET_MASK                                 ((uint8_t) 0xFF)
    #define OFFSET2_OFFSET_BITOFFSET                            (0)


/* Register 0x0D (OFFSET1) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                          OFFSET[15:8]                                         |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* OFFSET1 register */
    #define OFFSET1_ADDRESS                                     ((uint8_t) 0x0D)
    #define OFFSET1_DEFAULT                                     ((uint8_t) 0x00)

    /* OFFSET field */
    #define OFFSET1_OFFSET_MASK                                 ((uint8_t) 0xFF)
    #define OFFSET1_OFFSET_BITOFFSET                            (0)


/* Register 0x0E (OFFSET0) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                          OFFSET[7:0]                                          |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* OFFSET0 register */
    #define OFFSET0_ADDRESS                                     ((uint8_t) 0x0E)
    #define OFFSET0_DEFAULT                                     ((uint8_t) 0x00)

    /* OFFSET field */
    #define OFFSET0_OFFSET_MASK                                 ((uint8_t) 0xFF)
    #define OFFSET0_OFFSET_BITOFFSET                            (0)


/* Register 0x0F (GAIN2) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                          GAIN[23:16]                                          |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* GAIN2 register */
    #define GAIN2_ADDRESS                                       ((uint8_t) 0x0F)
    #define GAIN2_DEFAULT                                       ((uint8_t) 0x40)

    /* GAIN field */
    #define GAIN2_GAIN_MASK                                     ((uint8_t) 0xFF)
    #define GAIN2_GAIN_BITOFFSET                                (0)


/* Register 0x10 (GAIN1) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                           GAIN[15:8]                                          |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* GAIN1 register */
    #define GAIN1_ADDRESS                                       ((uint8_t) 0x10)
    #define GAIN1_DEFAULT                                       ((uint8_t) 0x00)

    /* GAIN field */
    #define GAIN1_GAIN_MASK                                     ((uint8_t) 0xFF)
    #define GAIN1_GAIN_BITOFFSET                                (0)


/* Register 0x11 (GAIN0) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                           GAIN[7:0]                                           |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* GAIN0 register */
    #define GAIN0_ADDRESS                                       ((uint8_t) 0x11)
    #define GAIN0_DEFAULT                                       ((uint8_t) 0x00)

    /* GAIN field */
    #define GAIN0_GAIN_MASK                                     ((uint8_t) 0xFF)
    #define GAIN0_GAIN_BITOFFSET                                (0)


/* Register 0x12 (MAIN_CRC) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                         MAIN_CRC[7:0]                                         |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* MAIN_CRC register */
    #define MAIN_CRC_ADDRESS                                    ((uint8_t) 0x12)
    #define MAIN_CRC_DEFAULT                                    ((uint8_t) 0x00)

    /* MAIN_CRC field */
    #define MAIN_CRC_MASK                                       ((uint8_t) 0xFF)
    #define MAIN_CRC_BITOFFSET                                  (0)


/* Register 0x13 (FIR_BANK) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                         FIR_BANK[7:0]                                         |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* FIR_BANK register */
    #define FIR_BANK_ADDRESS                                    ((uint8_t) 0x13)

    /* FIR_BANK field */
    #define FIR_BANK_MASK                                       ((uint8_t) 0xFF)
    #define FIR_BANK_BITOFFSET                                  (0)


/* Register 0x14 (FIR_CRC1) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                         FIR_CRC1[7:0]                                         |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* FIR_CRC1 register */
    #define FIR_CRC1_ADDRESS                                    ((uint8_t) 0x14)

    /* FIR_CRC_MSB field */
    #define FIR_CRC1_FIR_CRC1_MASK                              ((uint8_t) 0xFF)
    #define FIR_CRC1_FIR_CRC1_BITOFFSET                         (0)


/* Register 0x15 (FIR_CRC0) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                         FIR_CRC0[7:0]                                         |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* FIR_CRC0 register */
    #define FIR_CRC0_ADDRESS                                    ((uint8_t) 0x15)

    /* FIR_CRC_LSB field */
    #define FIR_CRC0_FIR_CRC0_MASK                              ((uint8_t) 0xFF)
    #define FIR_CRC0_FIR_CRC0_BITOFFSET                         (0)


/* Register 0x16 (IIR_BANK) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                         IIR_BANK[7:0]                                         |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* IIR_BANK register */
    #define IIR_BANK_ADDRESS                                    ((uint8_t) 0x16)

    /* IIR_BANK field */
    #define IIR_BANK_MASK                                       ((uint8_t) 0xFF)
    #define IIR_BANK_BITOFFSET                                  (0)


/* Register 0x17 (IIR_CRC) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                          IIR_CRC[7:0]                                         |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* IIR_CRC register */
    #define IIR_CRC_ADDRESS                                     ((uint8_t) 0x17)

    /* IIR_CRC field */
    #define IIR_CRC_MASK                                        ((uint8_t) 0xFF)
    #define IIR_CRC_BITOFFSET                                   (0)


//**********************************************************************************
//
// Register macros
//
//**********************************************************************************
#define MASKED_REG_DATA(addr, mask)     (getRegisterValue(addr) & (mask))

/** Returns true if SPI_ERR bit is set in STATUS1 shadow register */
#define SPI_3_WIRE_MODE         ((bool) MASKED_REG_DATA(STATUS1_ADDRESS, STATUS1_CS_MODE_MASK))

/** Returns true if SPI_ERR bit is set in STATUS1 shadow register */
#define SPI_ERR_FLAG            ((bool) MASKED_REG_DATA(STATUS1_ADDRESS, STATUS1_SPI_ERR_MASK))

/** Returns true if DATA bit is set in CONFIG1 shadow register */
#define RESOLUTION_IS_16_BIT    ((bool) MASKED_REG_DATA(CONFIG1_ADDRESS, CONFIG1_DATA_MASK))

/** Returns true if STATUS bit is set in CONFIG3 shadow register */
#define STATUS_ENABLED          ((bool) MASKED_REG_DATA(CONFIG3_ADDRESS, CONFIG3_STATUS_MASK))

/** Returns true if SPI_CRC bit is set in CONFIG3 shadow register */
#define SPI_CRC_ENABLED         ((bool) MASKED_REG_DATA(CONFIG3_ADDRESS, CONFIG3_SPI_CRC_MASK))


#endif /* ADS127L21_H_ */
