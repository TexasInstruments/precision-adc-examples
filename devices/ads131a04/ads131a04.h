/**
 *
 * \copyright Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef ADS131A04_H_
#define ADS131A04_H_


// Standard libraries
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

// Custom libraries
#include "hal.h"


//
// Configure Register settings - NOTE: These values can be change, but FIXED mode is recommended!
//

    // Fixed SPI Frame Mode - NOTE: Dynamic frame mode requires additional logic
    // (not provided in this example code) to prevent the F_SPI error flag.
    #define SET_FIXED

    // CRC Enable - NOTE: In this example enabling CRC will increase the amount
    // of time it takes the MCU to construct and send an SPI frame!
    //#define SET_CRC_EN

    // CRC Mode - Selects which SPI words get included in the CRC computation
    //#define SET_CRC_MODE


//
// Pin settings - NOTE: These values are NOT configurable in this example!
// This example code is only targeted at asynchronous slave mode with hamming validation turned off.
//

    // M0 -> Tied to IOVDD
    #define ASYNC_SLAVE_MODE

    // M1 -> Tied to GND
    #define WORD_LENGTH_BITS        ((uint8_t) 24)
    #define WORD_LENGTH_24BIT
    //#define WORD_LENGTH_16BIT
    //#define WORD_LENGTH_32BIT

    // M2 -> Tied to GND
    //#define HAMMING_ENABLED


//****************************************************************************
//
// Channel data structure
//
//****************************************************************************

typedef struct
{
    int32_t channel1;
    int32_t channel2;
    int32_t channel3;
    int32_t channel4;
    uint16_t response;
    uint16_t crc;

} adc_data_struct;



//**********************************************************************************
//
// Function prototypes
//
//**********************************************************************************

uint8_t     getRegisterValue(uint8_t address);
void        adcStartup(void);
uint8_t     readSingleRegister(uint8_t address);
void        writeSingleRegister(uint8_t address, uint8_t data);
bool        readData(adc_data_struct *dataStruct);
uint16_t    sendCommand(uint16_t opcode);
bool        lockRegisters(void);
bool        unlockRegisters(void);
uint16_t    calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint16_t initialValue);

// Helper functions
void        restoreRegisterDefaults(void);
int32_t     signExtend(const uint8_t dataBytes[]);



//**********************************************************************************
//
// Device commands
//
//**********************************************************************************

#define OPCODE_NULL                             ((uint16_t) 0x0000)
#define OPCODE_RESET                            ((uint16_t) 0x0011)
#define OPCODE_STANDBY                          ((uint16_t) 0x0022)
#define OPCODE_WAKEUP                           ((uint16_t) 0x0033)
#define OPCODE_LOCK                             ((uint16_t) 0x0555)
#define OPCODE_UNLOCK                           ((uint16_t) 0x0655)
#define OPCODE_RREG                             ((uint16_t) 0x2000)
#define OPCODE_WREG                             ((uint16_t) 0x4000)

// NOTE: The following command(s) are not implemented in this example...
//#define OPCODE_WREGS                            ((uint16_t) 0x6000)



//**********************************************************************************
//
// Register definitions
//
//**********************************************************************************

#define NUM_REGISTERS           ((uint8_t) 21)


/* Register 0x00 (ID_MSB) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |                                           NU_CH[7:0]                                          |
 * -------------------------------------------------------------------------------------------------
 */

    /* ID_MSB register address */
    #define ID_MSB_ADDRESS													((uint8_t) 0x00)

    /* ID_MSB register field masks */
    #define ID_MSB_NU_CH_MASK												((uint8_t) 0xFF)

    /* NU_CH field values */
    #define ID_MSB_NU_CH_2													((uint8_t) 0x02)
    #define ID_MSB_NU_CH_4													((uint8_t) 0x04)



/* Register 0x01 (ID_LSB) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |                                          REV_ID[7:0]                                          |
 * -------------------------------------------------------------------------------------------------
 */

    /* ID_LSB register address */
    #define ID_LSB_ADDRESS													((uint8_t) 0x01)

    /* ID_LSB register field masks */
    #define ID_LSB_REV_ID_MASK												((uint8_t) 0xFF)



/* Register 0x02 (STAT_1) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |   F_OPC   |   F_SPI   |  F_ADCIN  |   F_WDT   |  F_RESYNC |   F_DRDY  |  F_CHECK  |
 * -------------------------------------------------------------------------------------------------
 */

    /* STAT_1 register address */
    #define STAT_1_ADDRESS													((uint8_t) 0x02)

    /* STAT_1 default (reset) value */
    #define STAT_1_DEFAULT													((uint8_t) 0x00)

    /* STAT_1 register field masks */
    #define STAT_1_F_OPC_MASK												((uint8_t) 0x40)
    #define STAT_1_F_SPI_MASK												((uint8_t) 0x20)
    #define STAT_1_F_ADCIN_MASK												((uint8_t) 0x10)
    #define STAT_1_F_WDT_MASK												((uint8_t) 0x08)
    #define STAT_1_F_RESYNC_MASK											((uint8_t) 0x04)
    #define STAT_1_F_DRDY_MASK												((uint8_t) 0x02)
    #define STAT_1_F_CHECK_MASK												((uint8_t) 0x01)



/* Register 0x03 (STAT_P) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |   F_IN4P  |   F_IN3P  |   F_IN2P  |   F_IN1P  |
 * -------------------------------------------------------------------------------------------------
 */

    /* STAT_P register address */
    #define STAT_P_ADDRESS													((uint8_t) 0x03)

    /* STAT_P default (reset) value */
    #define STAT_P_DEFAULT													((uint8_t) 0x00)

    /* STAT_P register field masks */
    #define STAT_P_F_IN4P_MASK												((uint8_t) 0x08)
    #define STAT_P_F_IN3P_MASK												((uint8_t) 0x04)
    #define STAT_P_F_IN2P_MASK												((uint8_t) 0x02)
    #define STAT_P_F_IN1P_MASK												((uint8_t) 0x01)



/* Register 0x04 (STAT_N) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |   F_IN4N  |   F_IN3N  |   F_IN2N  |   F_IN1N  |
 * -------------------------------------------------------------------------------------------------
 */

    /* STAT_N register address */
    #define STAT_N_ADDRESS													((uint8_t) 0x04)

    /* STAT_N default (reset) value */
    #define STAT_N_DEFAULT													((uint8_t) 0x00)

    /* STAT_N register field masks */
    #define STAT_N_F_IN4N_MASK												((uint8_t) 0x08)
    #define STAT_N_F_IN3N_MASK												((uint8_t) 0x04)
    #define STAT_N_F_IN2N_MASK												((uint8_t) 0x02)
    #define STAT_N_F_IN1N_MASK												((uint8_t) 0x01)



/* Register 0x05 (STAT_S) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     | F_STARTUP |    F_CS   |  F_FRAME  |
 * -------------------------------------------------------------------------------------------------
 */

    /* STAT_S register address */
    #define STAT_S_ADDRESS													((uint8_t) 0x05)

    /* STAT_S default (reset) value */
    #define STAT_S_DEFAULT													((uint8_t) 0x00)

    /* STAT_S register field masks */
    #define STAT_S_F_STARTUP_MASK											((uint8_t) 0x04)
    #define STAT_S_F_CS_MASK												((uint8_t) 0x02)
    #define STAT_S_F_FRAME_MASK												((uint8_t) 0x01)



/* Register 0x06 (ERROR_CNT) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |                                            ER[7:0]                                            |
 * -------------------------------------------------------------------------------------------------
 */

    /* ERROR_CNT register address */
    #define ERROR_CNT_ADDRESS												((uint8_t) 0x06)

    /* ERROR_CNT default (reset) value */
    #define ERROR_CNT_DEFAULT												((uint8_t) 0x00)

    /* ERROR_CNT register field masks */
    #define ERROR_CNT_ER_MASK												((uint8_t) 0xFF)



/* Register 0x07 (STAT_M2) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |       M2PIN[1:0]      |       M1PIN[1:0]      |       M0PIN[1:0]      |
 * -------------------------------------------------------------------------------------------------
 */

    /* STAT_M2 register address */
    #define STAT_M2_ADDRESS													((uint8_t) 0x07)

    /* STAT_M2 default (reset) value */
    #define STAT_M2_DEFAULT													((uint8_t) 0x00)
    #define STAT_M2_DEFAULT_MASK											((uint8_t) 0xC0)

    /* STAT_M2 register field masks */
    #define STAT_M2_M2PIN_MASK												((uint8_t) 0x30)
    #define STAT_M2_M1PIN_MASK												((uint8_t) 0x0C)
    #define STAT_M2_M0PIN_MASK												((uint8_t) 0x03)

    /* M2PIN field values */
    #define STAT_M2_M2PIN_M2_HAMMING_OFF									((uint8_t) 0x00)
    #define STAT_M2_M2PIN_M2_HAMMING_ON										((uint8_t) 0x10)
    #define STAT_M2_M2PIN_M2_NC												((uint8_t) 0x20)

    /* M1PIN field values */
    #define STAT_M2_M1PIN_M1_24BIT											((uint8_t) 0x00)
    #define STAT_M2_M1PIN_M1_32BIT											((uint8_t) 0x04)
    #define STAT_M2_M1PIN_M1_16BIT											((uint8_t) 0x08)

    /* M0PIN field values */
    #define STAT_M2_M0PIN_M0_SYNC_MASTER									((uint8_t) 0x00)
    #define STAT_M2_M0PIN_M0_ASYNC_SLAVE									((uint8_t) 0x01)
    #define STAT_M2_M0PIN_M0_SYNC_SLAVE										((uint8_t) 0x02)



/* Register 0x08 (RESERVED0) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |     0     |     0     |     0     |
 * -------------------------------------------------------------------------------------------------
 */



/* Register 0x09 (RESERVED1) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |     0     |     0     |     0     |
 * -------------------------------------------------------------------------------------------------
 */



/* Register 0x0A (RESERVED2) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |     0     |     0     |     0     |
 * -------------------------------------------------------------------------------------------------
 */



/* Register 0x0B (A_SYS_CFG) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |   VNCPEN  |    HRM    |     0     |  VREF_4V  | INT_REFEN |            COMP_TH[2:0]           |
 * -------------------------------------------------------------------------------------------------
 */

    /* A_SYS_CFG register address */
    #define A_SYS_CFG_ADDRESS												((uint8_t) 0x0B)

    /* A_SYS_CFG default (reset) value */
    #define A_SYS_CFG_DEFAULT												((uint8_t) 0x60)

    /* A_SYS_CFG register field masks */
    #define A_SYS_CFG_VNCPEN_MASK											((uint8_t) 0x80)
    #define A_SYS_CFG_HRM_MASK												((uint8_t) 0x40)
    #define A_SYS_CFG_VREF_4V_MASK											((uint8_t) 0x10)
    #define A_SYS_CFG_INT_REFEN_MASK										((uint8_t) 0x08)
    #define A_SYS_CFG_COMP_TH_MASK											((uint8_t) 0x07)

    /* COMP_TH field values */
    #define A_SYS_CFG_COMP_TH_HIGH_95_LOW_5									((uint8_t) 0x00)
    #define A_SYS_CFG_COMP_TH_HIGH_92p5_LOW_7p5								((uint8_t) 0x01)
    #define A_SYS_CFG_COMP_TH_HIGH_90_LOW_10								((uint8_t) 0x02)
    #define A_SYS_CFG_COMP_TH_HIGH_87p5_LOW_12p5							((uint8_t) 0x03)
    #define A_SYS_CFG_COMP_TH_HIGH_85_LOW_15								((uint8_t) 0x04)
    #define A_SYS_CFG_COMP_TH_HIGH_80_LOW_20								((uint8_t) 0x05)
    #define A_SYS_CFG_COMP_TH_HIGH_75_LOW_25								((uint8_t) 0x06)
    #define A_SYS_CFG_COMP_TH_HIGH_70_LOW_30								((uint8_t) 0x07)



/* Register 0x0C (D_SYS_CFG) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |   WDT_EN  |  CRC_MODE |       DNDLY[1:0]      |      HIZDLY[1:0]      |   FIXED   |   CRC_EN  |
 * -------------------------------------------------------------------------------------------------
 */

    /* D_SYS_CFG register address */
    #define D_SYS_CFG_ADDRESS												((uint8_t) 0x0C)

    /* D_SYS_CFG default (reset) value */
    #define D_SYS_CFG_DEFAULT												((uint8_t) 0x3C)

    /* D_SYS_CFG register field masks */
    #define D_SYS_CFG_WDT_EN_MASK											((uint8_t) 0x80)
    #define D_SYS_CFG_CRC_MODE_MASK											((uint8_t) 0x40)
    #define D_SYS_CFG_DNDLY_MASK											((uint8_t) 0x30)
    #define D_SYS_CFG_HIZDLY_MASK											((uint8_t) 0x0C)
    #define D_SYS_CFG_FIXED_MASK											((uint8_t) 0x02)
    #define D_SYS_CFG_CRC_EN_MASK											((uint8_t) 0x01)

    /* DNDLY field values */
    #define D_SYS_CFG_DNDLY_6ns												((uint8_t) 0x00)
    #define D_SYS_CFG_DNDLY_8ns												((uint8_t) 0x10)
    #define D_SYS_CFG_DNDLY_10ns											((uint8_t) 0x20)
    #define D_SYS_CFG_DNDLY_12ns											((uint8_t) 0x30)

    /* HIZDLY field values */
    #define D_SYS_CFG_HIZDLY_6ns											((uint8_t) 0x00)
    #define D_SYS_CFG_HIZDLY_8ns											((uint8_t) 0x04)
    #define D_SYS_CFG_HIZDLY_10ns											((uint8_t) 0x08)
    #define D_SYS_CFG_HIZDLY_12ns											((uint8_t) 0x0C)



/* Register 0x0D (CLK1) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |   CLKSRC  |     0     |     0     |     0     |            CLK_DIV[2:0]           |     0     |
 * -------------------------------------------------------------------------------------------------
 */

    /* CLK1 register address */
    #define CLK1_ADDRESS													((uint8_t) 0x0D)

    /* CLK1 default (reset) value */
    #define CLK1_DEFAULT													((uint8_t) 0x08)

    /* CLK1 register field masks */
    #define CLK1_CLKSRC_MASK												((uint8_t) 0x80)
    #define CLK1_CLK_DIV_MASK												((uint8_t) 0x0E)

    /* CLK_DIV field values */
    #define CLK1_CLK_DIV_2													((uint8_t) 0x02)
    #define CLK1_CLK_DIV_4													((uint8_t) 0x04)
    #define CLK1_CLK_DIV_6													((uint8_t) 0x06)
    #define CLK1_CLK_DIV_8													((uint8_t) 0x08)
    #define CLK1_CLK_DIV_10													((uint8_t) 0x0A)
    #define CLK1_CLK_DIV_12													((uint8_t) 0x0C)
    #define CLK1_CLK_DIV_14													((uint8_t) 0x0E)



/* Register 0x0E (CLK2) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |           ICLK_DIV[2:0]           |     0     |                    OSR[3:0]                   |
 * -------------------------------------------------------------------------------------------------
 */

    /* CLK2 register address */
    #define CLK2_ADDRESS													((uint8_t) 0x0E)

    /* CLK2 default (reset) value */
    #define CLK2_DEFAULT													((uint8_t) 0x86)

    /* CLK2 register field masks */
    #define CLK2_ICLK_DIV_MASK												((uint8_t) 0xE0)
    #define CLK2_OSR_MASK													((uint8_t) 0x0F)

    /* ICLK_DIV field values */
    #define CLK2_ICLK_DIV_2													((uint8_t) 0x20)
    #define CLK2_ICLK_DIV_4													((uint8_t) 0x40)
    #define CLK2_ICLK_DIV_6													((uint8_t) 0x60)
    #define CLK2_ICLK_DIV_8													((uint8_t) 0x80)
    #define CLK2_ICLK_DIV_10												((uint8_t) 0xA0)
    #define CLK2_ICLK_DIV_12												((uint8_t) 0xC0)
    #define CLK2_ICLK_DIV_14												((uint8_t) 0xE0)

    /* OSR field values */
    #define CLK2_OSR_4096													((uint8_t) 0x00)
    #define CLK2_OSR_2048													((uint8_t) 0x01)
    #define CLK2_OSR_1024													((uint8_t) 0x02)
    #define CLK2_OSR_800													((uint8_t) 0x03)
    #define CLK2_OSR_768													((uint8_t) 0x04)
    #define CLK2_OSR_512													((uint8_t) 0x05)
    #define CLK2_OSR_400													((uint8_t) 0x06)
    #define CLK2_OSR_384													((uint8_t) 0x07)
    #define CLK2_OSR_256													((uint8_t) 0x08)
    #define CLK2_OSR_200													((uint8_t) 0x09)
    #define CLK2_OSR_192													((uint8_t) 0x0A)
    #define CLK2_OSR_128													((uint8_t) 0x0B)
    #define CLK2_OSR_96														((uint8_t) 0x0C)
    #define CLK2_OSR_64														((uint8_t) 0x0D)
    #define CLK2_OSR_48														((uint8_t) 0x0E)
    #define CLK2_OSR_32														((uint8_t) 0x0F)



/* Register 0x0F (ADC_ENA) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |                    ENA[3:0]                   |
 * -------------------------------------------------------------------------------------------------
 */

    /* ADC_ENA register address */
    #define ADC_ENA_ADDRESS													((uint8_t) 0x0F)

    /* ADC_ENA default (reset) value */
    #define ADC_ENA_DEFAULT													((uint8_t) 0x00)

    /* ADC_ENA register field masks */
    #define ADC_ENA_ENA_MASK												((uint8_t) 0x0F)

    /* ENA field values */
    #define ADC_ENA_ENA_ALL_CH_PWDN											((uint8_t) 0x00)
    #define ADC_ENA_ENA_ALL_CH_PWUP											((uint8_t) 0x0F)



/* Register 0x10 (RESERVED3) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |     0     |     0     |     0     |
 * -------------------------------------------------------------------------------------------------
 */



/* Register 0x11 (ADC1) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |             GAIN1[2:0]            |
 * -------------------------------------------------------------------------------------------------
 */

    /* ADC1 register address */
    #define ADC1_ADDRESS													((uint8_t) 0x11)

    /* ADC1 default (reset) value */
    #define ADC1_DEFAULT													((uint8_t) 0x00)

    /* ADC1 register field masks */
    #define ADC1_GAIN1_MASK													((uint8_t) 0x07)

    /* GAIN1 field values */
    #define ADC1_GAIN1_1													((uint8_t) 0x00)
    #define ADC1_GAIN1_2													((uint8_t) 0x01)
    #define ADC1_GAIN1_4													((uint8_t) 0x02)
    #define ADC1_GAIN1_8													((uint8_t) 0x03)
    #define ADC1_GAIN1_16													((uint8_t) 0x04)



/* Register 0x12 (ADC2) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |             GAIN2[2:0]            |
 * -------------------------------------------------------------------------------------------------
 */

    /* ADC2 register address */
    #define ADC2_ADDRESS													((uint8_t) 0x12)

    /* ADC2 default (reset) value */
    #define ADC2_DEFAULT													((uint8_t) 0x00)

    /* ADC2 register field masks */
    #define ADC2_GAIN2_MASK													((uint8_t) 0x07)

    /* GAIN2 field values */
    #define ADC2_GAIN2_1													((uint8_t) 0x00)
    #define ADC2_GAIN2_2													((uint8_t) 0x01)
    #define ADC2_GAIN2_4													((uint8_t) 0x02)
    #define ADC2_GAIN2_8													((uint8_t) 0x03)
    #define ADC2_GAIN2_16													((uint8_t) 0x04)



/* Register 0x13 (ADC3) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |             GAIN3[2:0]            |
 * -------------------------------------------------------------------------------------------------
 */

    /* ADC3 register address */
    #define ADC3_ADDRESS													((uint8_t) 0x13)

    /* ADC3 default (reset) value */
    #define ADC3_DEFAULT													((uint8_t) 0x00)

    /* ADC3 register field masks */
    #define ADC3_GAIN3_MASK													((uint8_t) 0x07)

    /* GAIN3 field values */
    #define ADC3_GAIN3_1													((uint8_t) 0x00)
    #define ADC3_GAIN3_2													((uint8_t) 0x01)
    #define ADC3_GAIN3_4													((uint8_t) 0x02)
    #define ADC3_GAIN3_8													((uint8_t) 0x03)
    #define ADC3_GAIN3_16													((uint8_t) 0x04)



/* Register 0x14 (ADC4) definition
 * -------------------------------------------------------------------------------------------------
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * -------------------------------------------------------------------------------------------------
 * |     0     |     0     |     0     |     0     |     0     |             GAIN4[2:0]            |
 * -------------------------------------------------------------------------------------------------
 */

    /* ADC4 register address */
    #define ADC4_ADDRESS													((uint8_t) 0x14)

    /* ADC4 default (reset) value */
    #define ADC4_DEFAULT													((uint8_t) 0x00)

    /* ADC4 register field masks */
    #define ADC4_GAIN4_MASK													((uint8_t) 0x07)

    /* GAIN4 field values */
    #define ADC4_GAIN4_1													((uint8_t) 0x00)
    #define ADC4_GAIN4_2													((uint8_t) 0x01)
    #define ADC4_GAIN4_4													((uint8_t) 0x02)
    #define ADC4_GAIN4_8													((uint8_t) 0x03)
    #define ADC4_GAIN4_16													((uint8_t) 0x04)



//****************************************************************************
//
// Register settings macros
//
//****************************************************************************

// NOTE: These macros can be used in logical expressions for checking known register values.
// These macros will take on the value from the last register read/write operation, or the
// default register value if no register operations have occurred or the device is reset.

#define CRC_EN              ((bool) (getRegisterValue(D_SYS_CFG_ADDRESS) & D_SYS_CFG_CRC_EN_MASK))
#define CRC_MODE            ((bool) (getRegisterValue(D_SYS_CFG_ADDRESS) & D_SYS_CFG_CRC_MODE_MASK))
#define FIXED               ((bool) (getRegisterValue(D_SYS_CFG_ADDRESS) & D_SYS_CFG_FIXED_MASK))



#endif /* ADS131A04_H_ */
