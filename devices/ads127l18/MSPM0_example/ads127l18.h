/*
 * @file ads127l18.h
 *
 * @brief ADS127L18 Descriptor
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

#ifndef ADS127L18_H_
#define ADS127L18_H_

// Standard libraries
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

// Custom libraries
#include "hal.h"

//****************************************************************************
//
// Function prototypes
//
//****************************************************************************
uint8_t     readSingleRegister(uint8_t address);
void        writeSingleRegister(uint8_t address, uint8_t data);
void        sendCommand(uint8_t opcode[]);
void        initalizeRegisters (uint16_t OSR, uint8_t speed_mode, int DP_TDM, int DCLK_DIV, int ADC_CLK_SEL, int ADC_CLK_DIV);

// Getter functions
uint16_t    getRegisterValue(uint8_t address);

//****************************************************************************
//
// Register macros
//
//****************************************************************************

/** Returns the number of bytes in an SPI word */
#define SPI_BUFFER_SIZE         4

//**********************************************************************************
//
// Device commands
//
//**********************************************************************************

#define OPCODE_NULL                             ((uint8_t) 0x00)
#define OPCODE_RREG                             ((uint8_t) 0x40) // FIXME: changing this from 0x00 back to 0x40 makes the code run not sure why
#define OPCODE_WREG                             ((uint8_t) 0x80)

//**********************************************************************************
//
// Constants
//
//**********************************************************************************

#define NUM_REGISTERS                           ((uint8_t) 81) 

//**********************************************************************************
//
// Register definitions
//
//**********************************************************************************


/* Register 0x00 (DEV_ID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                           DEV_ID[7:0]                                                         |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DEVID_REG register */
    #define DEV_ID_REG_ADDRESS												((uint8_t) 0x00)
    #define DEV_ID_REG_DEFAULT												((uint8_t) 0x00)
    #define DEV_ID_REG_0                                                    ((uint8_t) 0x06)



/* Register 0x01 (REV_ID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                           REV_ID[7:0]                                                         |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* REVID_REG register */
    #define REV_ID_REG_ADDRESS												((uint8_t) 0x01)
    #define REV_ID_REG_DEFAULT												((uint8_t) 0x00)



/* Register 0x02 (STATUS_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    RESERVED   |    ALV_FLAG   |    POR_FLAG   |    SPI_ERR    |    REG_ERR    |    ADC_ERR    |    ADDR_ERR   |    SCLK_ERR   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* STATUS_REG register */
    #define STATUS_REG_ADDRESS												((uint8_t) 0x02)
    #define STATUS_REG_DEFAULT												((uint8_t) 0x60)

    /* Analog supply low-voltage flag field */
    #define STATUS_REG_ALV_FLAG_MASK										((uint8_t) 0x40)
    #define STATUS_REG_ALV_FLAG_0											((uint8_t) 0x00)    
    #define STATUS_REG_ALV_FLAG_1											((uint8_t) 0x40)    // DEFAULT

    /* Power-on reset flag field */
    #define STATUS_REG_POR_FLAG_MASK										((uint8_t) 0x20)
    #define STATUS_REG_POR_FLAG_0											((uint8_t) 0x00)    
    #define STATUS_REG_POR_FLAG_1											((uint8_t) 0x20)    // DEFAULT

    /* SPI received data CRC error field */
    #define STATUS_REG_SPI_ERR_MASK								    		((uint8_t) 0x10)
    #define STATUS_REG_SPI_ERR_0									    	((uint8_t) 0x00)    // DEFAULT
    #define STATUS_REG_SPI_ERR_1									    	((uint8_t) 0x10)

    /* Register map error field */
    #define STATUS_REG_REG_ERR_MASK										    ((uint8_t) 0x08)
    #define STATUS_REG_REG_ERR_0											((uint8_t) 0x00)    // DEFAULT
    #define STATUS_REG_REG_ERR_1											((uint8_t) 0x08)

    /* ADC Error field */
    #define STATUS_REG_ADC_ERR_MASK								    		((uint8_t) 0x04)
    #define STATUS_REG_ADC_ERR_0									    	((uint8_t) 0x00)    // DEFAULT
    #define STATUS_REG_ADC_ERR_1									    	((uint8_t) 0x04)

    /* SPI register address error field */
    #define STATUS_REG_ADDR_ERR_MASK								    	((uint8_t) 0x02)
    #define STATUS_REG_ADDR_ERR_0									    	((uint8_t) 0x00)    // DEFAULT
    #define STATUS_REG_ADDR_ERR_1									    	((uint8_t) 0x02)

    /* SPI SCLK count error field */
    #define STATUS_REG_SCLK_ERR_MASK									    ((uint8_t) 0x01)
    #define STATUS_REG_SCLK_ERR_0										    ((uint8_t) 0x00)    // DEFAULT
    #define STATUS_REG_SCLK_ERR_1									    	((uint8_t) 0x01)



/* Register 0x03 (CLK_CNT_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                          CLK_CNT[7:0]                                                         |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CLK_CNT_REG register */
    #define CLK_CNT_REG_ADDRESS                                             ((uint8_t) 0x03)
    #define CLK_CNT_REG_DEFAULT                                             ((uint8_t) 0x00)



/* Register 0x04 (GPIO_RD_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                          GPIO_RD[7:0]                                                         |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPIO_RD_REG register */
    #define GPIO_RD_REG_ADDRESS									            ((uint8_t) 0x04)
    #define GPIO_RD_REG_DEFAULT									    		((uint8_t) 0x00)



/* Register 0x05 (CRC_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                          CRC_MSB[7:0]                                                         |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CRC_MSB register */
    #define CRC_MSB_REG_ADDRESS												((uint8_t) 0x05)
    #define CRC_MSB_REG_DEFAULT												((uint8_t) 0x00)



/* Register 0x06 (CRC_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                          CRC_LSB[7:0]                                                         |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CRC_LSB register */
    #define CRC_LSB_REG_ADDRESS                                             ((uint8_t) 0x06)
    #define CRC_LSB_REG_DEFAULT                                             ((uint8_t) 0x00)



/* Register 0x07 (CONTROL_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                          RESET[5:0]                                           |     START     |     STOP      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CONTROL register */
    #define CONTROL_REG_ADDRESS										    	((uint8_t) 0x07)
    #define CONTROL_REG_DEFAULT										    	((uint8_t) 0x00)

    /* Software reset field */
    #define CONTROL_REG_RESET_MASK                                          ((uint8_t) 0xFC)
    #define CONTROL_REG_RESET_0                                             ((uint8_t) 0x00)    // DEFAULT
    #define CONTROL_REG_RESET_COMMAND                                       ((uint8_t) 0x58)

    /* START channel conversions field */
    #define CONTROL_REG_START_MASK                                          ((uint8_t) 0x02)
    #define CONTROL_REG_START_0                                             ((uint8_t) 0x00)    // DEFAULT
    #define CONTROL_REG_START_1                                             ((uint8_t) 0x02)

    /* Stop channel conversions field */
    #define CONTROL_REG_STOP_MASK                                           ((uint8_t) 0x01)
    #define CONTROL_REG_STOP_0                                              ((uint8_t) 0x00)    // DEFAULT
    #define CONTROL_REG_STOP_1                                              ((uint8_t) 0x01)



/* Register 0x08 (GEN_CFG1_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |           RESERVED            |                   DELAY[2:0]                  |      VCM      |    REFP_BUF   |    REF_RNG    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GEN_CFG1 register */
    #define GEN_CFG1_REG_ADDRESS                                            ((uint8_t) 0x08)
    #define GEN_CFG1_REG_DEFAULT                                            ((uint8_t) 0x00)

    /* Conversion start delay time selection field */
    #define GEN_CFG1_REG_DELAY_MASK                                         ((uint8_t) 0x38)
    #define GEN_CFG1_REG_DELAY_0                                            ((uint8_t) 0x00)    // DEFAULT
    #define GEN_CFG1_REG_DELAY_4                                            ((uint8_t) 0x08)
    #define GEN_CFG1_REG_DELAY_8                                            ((uint8_t) 0x10)
    #define GEN_CFG1_REG_DELAY_16                                           ((uint8_t) 0x18)
    #define GEN_CFG1_REG_DELAY_32                                           ((uint8_t) 0x20)
    #define GEN_CFG1_REG_DELAY_128                                          ((uint8_t) 0x28)
    #define GEN_CFG1_REG_DELAY_512                                          ((uint8_t) 0x30)
    #define GEN_CFG1_REG_DELAY_1024                                         ((uint8_t) 0x38)

    /* Common-mode voltage output enable field */
    #define GEN_CFG1_REG_VCM_MASK                                           ((uint8_t) 0x04)
    #define GEN_CFG1_REG_VCM_0                                              ((uint8_t) 0x00)    // DEFAULT
    #define GEN_CFG1_REG_VCM_1                                              ((uint8_t) 0x04)

    /* Reference positive buffer enable field */
    #define GEN_CFG1_REG_REFP_BUF_MASK                                      ((uint8_t) 0x02)
    #define GEN_CFG1_REG_REFP_BUF_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define GEN_CFG1_REG_REFP_BUF_1                                         ((uint8_t) 0x02)

    /* Voltage reference range selection field */
    #define GEN_CFG1_REG_REF_RNG_MASK                                       ((uint8_t) 0x01)
    #define GEN_CFG1_REG_REF_RNG_0                                          ((uint8_t) 0x00)    // DEFAULT
    #define GEN_CFG1_REG_REF_RNG_1                                          ((uint8_t) 0x01)



/* Register 0x09 (GEN_CFG2_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |         AVG_MODE[1:0]         |   RESERVED    |        START_MODE[1:0]        |        SPEED_MODE[1:0]        |   STBY_MODE   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GEN_CFG2_REG register */
    #define GEN_CFG2_REG_ADDRESS											((uint8_t) 0x09)
    #define GEN_CFG2_REG_DEFAULT											((uint8_t) 0x04)

    /* Channel data averaging mode field */
    #define GEN_CFG2_REG_AVG_MODE_MASK                                      ((uint8_t) 0xC0)
    #define GEN_CFG2_REG_AVG_MODE_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define GEN_CFG2_REG_AVG_MODE_1                                         ((uint8_t) 0x40)
    #define GEN_CFG2_REG_AVG_MODE_2                                         ((uint8_t) 0x80)
    #define GEN_CFG2_REG_AVG_MODE_3                                         ((uint8_t) 0xC0)

    /* START mode selection field */
    #define GEN_CFG2_REG_START_MODE_MASK                                    ((uint8_t) 0x18)
    #define GEN_CFG2_REG_START_MODE_0                                       ((uint8_t) 0x00)    // DEFAULT
    #define GEN_CFG2_REG_START_MODE_1                                       ((uint8_t) 0x08)
    #define GEN_CFG2_REG_START_MODE_2                                       ((uint8_t) 0x10)
    #define GEN_CFG2_REG_START_MODE_3                                       ((uint8_t) 0x18)

    /* Speed mode selection field */
    #define GEN_CFG2_REG_SPEED_MODE_MASK                                    ((uint8_t) 0x06)
    #define GEN_CFG2_REG_SPEED_MODE_0                                       ((uint8_t) 0x00)
    #define GEN_CFG2_REG_SPEED_MODE_1                                       ((uint8_t) 0x02)
    #define GEN_CFG2_REG_SPEED_MODE_2                                       ((uint8_t) 0x04)    // DEFAULT
    #define GEN_CFG2_REG_SPEED_MODE_3                                       ((uint8_t) 0x06)

    /* Standby mode selection field */
    #define GEN_CFG2_REG_STBY_MODE_MASK                                     ((uint8_t) 0x01)
    #define GEN_CFG2_REG_STBY_MODE_0                                        ((uint8_t) 0x00)    // DEFAULT
    #define GEN_CFG2_REG_STBY_MODE_1                                        ((uint8_t) 0x01)



/* Register 0x0A (GEN_CFG3_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    OUT_DRV    |     DATA      |  CLK_CNT_EN   |  SPI_STAT_EN  |  SPI_ADDR_EN  |  SCLK_CNT_EN  |  SPI_CRC_EN   |  REG_CRC_EN   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GEN_CFG3_REG register */
    #define GEN_CFG3_REG_ADDRESS                                            ((uint8_t) 0x0A)
    #define GEN_CFG3_REG_DEFAULT                                            ((uint8_t) 0x80)

    /* Digital output driver power selection field */
    #define GEN_CFG3_REG_OUT_DRV_MASK                                       ((uint8_t) 0x80)
    #define GEN_CFG3_REG_OUT_DRV_0                                          ((uint8_t) 0x00)    
    #define GEN_CFG3_REG_OUT_DRV_1                                          ((uint8_t) 0x80)    // DEFAULT

    /* Data resolution selection field */
    #define GEN_CFG3_REG_DATA_MASK                                          ((uint8_t) 0x40)
    #define GEN_CFG3_REG_DATA_0                                             ((uint8_t) 0x00)    // DEFAULT    
    #define GEN_CFG3_REG_DATA_1                                             ((uint8_t) 0x40)

    /* Clock counter enable field */
    #define GEN_CFG3_REG_CLK_CNT_EN_MASK                                    ((uint8_t) 0x20)
    #define GEN_CFG3_REG_CLK_CNT_EN_0                                       ((uint8_t) 0x00)    // DEFAULT    
    #define GEN_CFG3_REG_CLK_CNT_EN_1                                       ((uint8_t) 0x20)

    /* SPI status byte output enable field */
    #define GEN_CFG3_REG_SPI_STAT_EN_MASK                                   ((uint8_t) 0x10)
    #define GEN_CFG3_REG_SPI_STAT_EN_0                                      ((uint8_t) 0x00)    // DEFAULT    
    #define GEN_CFG3_REG_SPI_STAT_EN_1                                      ((uint8_t) 0x10)

    /* SPI register address enable field */
    #define GEN_CFG3_REG_SPI_ADDR_EN_MASK                                   ((uint8_t) 0x08)
    #define GEN_CFG3_REG_SPI_ADDR_EN_0                                      ((uint8_t) 0x00)    // DEFAULT    
    #define GEN_CFG3_REG_SPI_ADDR_EN_1                                      ((uint8_t) 0x08)

    /* SCLK count enable field */
    #define GEN_CFG3_REG_SCLK_CNT_EN_MASK                                   ((uint8_t) 0x04)
    #define GEN_CFG3_REG_SCLK_CNT_EN_0                                      ((uint8_t) 0x00)    // DEFAULT    
    #define GEN_CFG3_REG_SCLK_CNT_EN_1                                      ((uint8_t) 0x04)

    /* SPI CRC enable field */
    #define GEN_CFG3_REG_SPI_CRC_EN_MASK                                    ((uint8_t) 0x02)
    #define GEN_CFG3_REG_SPI_CRC_EN_0                                       ((uint8_t) 0x00)    // DEFAULT    
    #define GEN_CFG3_REG_SPI_CRC_EN_1                                       ((uint8_t) 0x02)

    /* Register map CRC enable field */
    #define GEN_CFG3_REG_REG_CRC_EN_MASK                                    ((uint8_t) 0x01)
    #define GEN_CFG3_REG_REG_CRC_EN_0                                       ((uint8_t) 0x00)    // DEFAULT    
    #define GEN_CFG3_REG_REG_CRC_EN_1                                       ((uint8_t) 0x01)


/* Register 0x0B (DP_CFG1_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |   DP_CRC_EN   |  DP_STAT_EN   |          DP_TDM[1:0]          |           RESERVED            |   DP_DAISY    |    DP_MODE    |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DP_CFG1_REG register */
    #define DP_CFG1_REG_ADDRESS                                             ((uint8_t) 0x0B)
    #define DP_CFG1_REG_DEFAULT                                             ((uint8_t) 0x20)

    /* Data port CRC byte enable field */
    #define DP_CFG1_REG_DP_CRC_EN_MASK                                      ((uint8_t) 0x80)
    #define DP_CFG1_REG_DP_CRC_EN_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define DP_CFG1_REG_DP_CRC_EN_1                                         ((uint8_t) 0x80)

    /* Data port status byte enable field */
    #define DP_CFG1_REG_DP_STAT_EN_MASK                                     ((uint8_t) 0x40)
    #define DP_CFG1_REG_DP_STAT_EN_0                                        ((uint8_t) 0x00)    // DEFAULT
    #define DP_CFG1_REG_DP_STAT_EN_1                                        ((uint8_t) 0x40)

    /* Data port time division multiplexing (TDM) configuration field */
    #define DP_CFG1_REG_DP_TDM_MASK                                         ((uint8_t) 0x30)
    #define DP_CFG1_REG_DP_TDM_0                                            ((uint8_t) 0x00)
    #define DP_CFG1_REG_DP_TDM_1                                            ((uint8_t) 0x10)    
    #define DP_CFG1_REG_DP_TDM_2                                            ((uint8_t) 0x20)    // DEFAULT
    #define DP_CFG1_REG_DP_TDM_3                                            ((uint8_t) 0x30)

    /* Data port daisy-chain mode field */
    #define DP_CFG1_REG_DP_DAISY_MASK                                       ((uint8_t) 0x02)
    #define DP_CFG1_REG_DP_DAISY_0                                          ((uint8_t) 0x00)    // DEFAULT
    #define DP_CFG1_REG_DP_DAISY_1                                          ((uint8_t) 0x02)

    /* Data port settled-data mode field */
    #define DP_CFG1_REG_DP_MODE_MASK                                        ((uint8_t) 0x01)
    #define DP_CFG1_REG_DP_MODE_0                                           ((uint8_t) 0x00)    // DEFAULT
    #define DP_CFG1_REG_DP_MODE_1                                           ((uint8_t) 0x01)



/* Register 0x0C (DP_CFG2_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    RESERVED   |         DCLK_DIV[1:0]         |                                 DOUT_DLY[4:0]                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* DP_CFG2_REG register */
    #define DP_CFG2_REG_ADDRESS                                             ((uint8_t) 0x0C)
    #define DP_CFG2_REG_DEFAULT                                             ((uint8_t) 0x00)

    /* Data port DCLK frequency divider field */
    #define DP_CFG2_REG_DCLK_DIV_MASK                                       ((uint8_t) 0x60)
    #define DP_CFG2_REG_DCLK_DIV_0                                          ((uint8_t) 0x00)    // DEFAULT
    #define DP_CFG2_REG_DCLK_DIV_1                                          ((uint8_t) 0x20)    
    #define DP_CFG2_REG_DCLK_DIV_2                                          ((uint8_t) 0x40)
    #define DP_CFG2_REG_DCLK_DIV_3                                          ((uint8_t) 0x60)

    /* Data port DOUT delay field */
    #define DP_CFG2_REG_DOUT_DLY_MASK                                       ((uint8_t) 0x1F)



/* Register 0x0D (CLK_CFG_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                           RESERVED                            |    CLK_SEL    |                 CLK_DIV[2:0]                  |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CLK_CFG_REG register */
    #define CLK_CFG_REG_ADDRESS                                             ((uint8_t) 0x0D)
    #define CLK_CFG_REG_DEFAULT                                             ((uint8_t) 0x00)

    /* ADC clock selection field */
    #define CLK_CFG_REG_CLK_SEL_MASK                                        ((uint8_t) 0x08)
    #define CLK_CFG_REG_CLK_SEL_0                                           ((uint8_t) 0x00)    // DEFAULT
    #define CLK_CFG_REG_CLK_SEL_1                                           ((uint8_t) 0x08)

    /* ADC clock divider field */
    #define CLK_CFG_REG_CLK_DIV_MASK                                        ((uint8_t) 0x07)
    #define CLK_CFG_REG_CLK_DIV_0                                           ((uint8_t) 0x00)    // DEFAULT
    #define CLK_CFG_REG_CLK_DIV_1                                           ((uint8_t) 0x01)
    #define CLK_CFG_REG_CLK_DIV_2                                           ((uint8_t) 0x02)
    #define CLK_CFG_REG_CLK_DIV_3                                           ((uint8_t) 0x03)
    #define CLK_CFG_REG_CLK_DIV_4                                           ((uint8_t) 0x04)
    #define CLK_CFG_REG_CLK_DIV_5                                           ((uint8_t) 0x05)
    #define CLK_CFG_REG_CLK_DIV_6                                           ((uint8_t) 0x06)
    #define CLK_CFG_REG_CLK_DIV_7                                           ((uint8_t) 0x07)



/* Register 0x0E (GPIO_WR_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       GPIO_WR_REG[7:0]                                                        |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GAIN_MSB_REG register */
    #define GPIO_WR_REG_ADDRESS                                             ((uint8_t) 0x0E)
    #define GPIO_WR_REG_DEFAULT                                             ((uint8_t) 0x00)



/* Register 0x0F (GPIO_DIR_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       GPIO_DIR_REG[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPIO_DIR_REG register */
    #define GPIO_DIR_REG_ADDRESS                                            ((uint8_t) 0x0F)
    #define GPIO_DIR_REG_DEFAULT                                            ((uint8_t) 0x00)



/* Register 0x10 (GPIO_EN_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       GPIO_EN_REG[7:0]                                                        |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* GPIO_EN_REG register */
    #define GPIO_EN_REG_ADDRESS                                             ((uint8_t) 0x10)
    #define GPIO_EN_REG_DEFAULT                                             ((uint8_t) 0x00)



/* Register 0x11 (CH0_CFG1_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    RESERVED   |                 CH0_MUX[2:0]                  |  CH0_INP_RNG  |  CH0_EX_RNG   |    CH0_BUFN   |    CH0_BUFP   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH0_CFG1_REG register */
    #define CH0_CFG1_REG_ADDRESS                                            ((uint8_t) 0x11)
    #define CH0_CFG1_REG_DEFAULT                                            ((uint8_t) 0x00)

    /* Channel input multiplexer selection */
    #define CH0_CFG1_REG_CH0_MUX_MASK                                       ((uint8_t) 0x70)
    #define CH0_CFG1_REG_CH0_MUX_0                                          ((uint8_t) 0x00)    // DEFAULT
    #define CH0_CFG1_REG_CH0_MUX_1                                          ((uint8_t) 0x10)
    #define CH0_CFG1_REG_CH0_MUX_2                                          ((uint8_t) 0x20)
    #define CH0_CFG1_REG_CH0_MUX_3                                          ((uint8_t) 0x30)
    #define CH0_CFG1_REG_CH0_MUX_4                                          ((uint8_t) 0x40)
    #define CH0_CFG1_REG_CH0_MUX_5                                          ((uint8_t) 0x50)
    #define CH0_CFG1_REG_CH0_MUX_6                                          ((uint8_t) 0x60)
    #define CH0_CFG1_REG_CH0_MUX_7                                          ((uint8_t) 0x70)

    /* Channel input range selection */
    #define CH0_CFG1_REG_CH0_INP_RNG_MASK                                   ((uint8_t) 0x08)
    #define CH0_CFG1_REG_CH0_INP_RNG_0                                      ((uint8_t) 0x00)    // DEFAULT
    #define CH0_CFG1_REG_CH0_INP_RNG_1                                      ((uint8_t) 0x08)

    /* Channel extended input range selection */
    #define CH0_CFG1_REG_CH0_EX_RNG_MASK                                    ((uint8_t) 0x04)
    #define CH0_CFG1_REG_CH0_EX_RNG_0                                       ((uint8_t) 0x00)    // DEFAULT
    #define CH0_CFG1_REG_CH0_EX_RNG_1                                       ((uint8_t) 0x04)

    /* Channel analog input negative buffer enable */
    #define CH0_CFG1_REG_CH0_BUFN_MASK                                      ((uint8_t) 0x02)
    #define CH0_CFG1_REG_CH0_BUFN_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH0_CFG1_REG_CH0_BUFN_1                                         ((uint8_t) 0x02)

    /* Channel analog input positive buffer enable */
    #define CH0_CFG1_REG_CH0_BUFP_MASK                                      ((uint8_t) 0x01)
    #define CH0_CFG1_REG_CH0_BUFP_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH0_CFG1_REG_CH0_BUFP_1                                         ((uint8_t) 0x01)



/* Register 0x12 (CH0_CFG2_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |           RESERVED            |    CH0_PWDN   |                                 CH0_FLTR[4:0]                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH0_CFG1_REG register */
    #define CH0_CFG2_REG_ADDRESS                                            ((uint8_t) 0x12)
    #define CH0_CFG2_REG_DEFAULT                                            ((uint8_t) 0x00)

    /* Channel power-down mode selection */
    #define CH0_CFG2_REG_CH0_PWDN_MASK                                      ((uint8_t) 0x20)
    #define CH0_CFG2_REG_CH0_PWDN_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH0_CFG2_REG_CH0_PWDN_1                                         ((uint8_t) 0x20)

    /* Channel digital filter and oversampling ratio value selection */
    #define CH0_CFG2_REG_CH0_FLTR_MASK                                      ((uint8_t) 0x1F)
    #define CH0_CFG2_REG_CH0_FLTR_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH0_CFG2_REG_CH0_FLTR_1                                         ((uint8_t) 0x01)
    #define CH0_CFG2_REG_CH0_FLTR_2                                         ((uint8_t) 0x02)
    #define CH0_CFG2_REG_CH0_FLTR_3                                         ((uint8_t) 0x03)
    #define CH0_CFG2_REG_CH0_FLTR_4                                         ((uint8_t) 0x04)
    #define CH0_CFG2_REG_CH0_FLTR_5                                         ((uint8_t) 0x05)
    #define CH0_CFG2_REG_CH0_FLTR_6                                         ((uint8_t) 0x06)
    #define CH0_CFG2_REG_CH0_FLTR_7                                         ((uint8_t) 0x07)
    #define CH0_CFG2_REG_CH0_FLTR_8                                         ((uint8_t) 0x08)
    #define CH0_CFG2_REG_CH0_FLTR_9                                         ((uint8_t) 0x09)
    #define CH0_CFG2_REG_CH0_FLTR_10                                        ((uint8_t) 0x0A)
    #define CH0_CFG2_REG_CH0_FLTR_11                                        ((uint8_t) 0x0B)
    #define CH0_CFG2_REG_CH0_FLTR_12                                        ((uint8_t) 0x0C)
    #define CH0_CFG2_REG_CH0_FLTR_13                                        ((uint8_t) 0x0D)
    #define CH0_CFG2_REG_CH0_FLTR_14                                        ((uint8_t) 0x0E)
    #define CH0_CFG2_REG_CH0_FLTR_15                                        ((uint8_t) 0x0F)
    #define CH0_CFG2_REG_CH0_FLTR_16                                        ((uint8_t) 0x10)
    #define CH0_CFG2_REG_CH0_FLTR_17                                        ((uint8_t) 0x11)
    #define CH0_CFG2_REG_CH0_FLTR_18                                        ((uint8_t) 0x12)
    #define CH0_CFG2_REG_CH0_FLTR_19                                        ((uint8_t) 0x13)
    #define CH0_CFG2_REG_CH0_FLTR_20                                        ((uint8_t) 0x14)
    #define CH0_CFG2_REG_CH0_FLTR_21                                        ((uint8_t) 0x15)
    #define CH0_CFG2_REG_CH0_FLTR_22                                        ((uint8_t) 0x16)
    #define CH0_CFG2_REG_CH0_FLTR_23                                        ((uint8_t) 0x17)
    #define CH0_CFG2_REG_CH0_FLTR_24                                        ((uint8_t) 0x18)
    #define CH0_CFG2_REG_CH0_FLTR_25                                        ((uint8_t) 0x19)
    #define CH0_CFG2_REG_CH0_FLTR_26                                        ((uint8_t) 0x1A)
    #define CH0_CFG2_REG_CH0_FLTR_27                                        ((uint8_t) 0x1B)
    #define CH0_CFG2_REG_CH0_FLTR_28                                        ((uint8_t) 0x1C)
    #define CH0_CFG2_REG_CH0_FLTR_29                                        ((uint8_t) 0x1D)
    #define CH0_CFG2_REG_CH0_FLTR_30                                        ((uint8_t) 0x1E)
    #define CH0_CFG2_REG_CH0_FLTR_31                                        ((uint8_t) 0x1F)



/* Register 0x13 (CH0_OFS_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH0_OFFSET_MSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH0_OFS_MSB_REG register */
    #define CH0_OFS_MSB_REG_ADDRESS                                         ((uint8_t) 0x13)
    #define CH0_OFS_MSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x14 (CH0_OFS_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH0_OFFSET_MID[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH0_OFS_MID_REG register */
    #define CH0_OFS_MID_REG_ADDRESS                                         ((uint8_t) 0x14)
    #define CH0_OFS_MID_REG_DEFAULT                                         ((uint8_t) 0x00)

    

/* Register 0x15 (CH0_OFS_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH0_OFFSET_LSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH0_OFS_LSB_REG register */
    #define CH0_OFS_LSB_REG_ADDRESS                                         ((uint8_t) 0x15)
    #define CH0_OFS_LSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x16 (CH0_GAN_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH0_GAIN_MSB[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH0_GAN_MSB_REG register */
    #define CH0_GAN_MSB_REG_ADDRESS                                         ((uint8_t) 0x16)
    #define CH0_GAN_MSB_REG_DEFAULT                                         ((uint8_t) 0x40)



/* Register 0x17 (CH0_GAN_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH0_GAIN_MID[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */
 
    /* CH0_GAN_MID_REG register */
    #define CH0_GAN_MID_REG_ADDRESS                                         ((uint8_t) 0x17)
    #define CH0_GAN_MID_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x18 (CH0_GAN_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH0_GAIN_LSB[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */
 
    /* CH0_GAN_LSB_REG register */
    #define CH0_GAN_LSB_REG_ADDRESS                                         ((uint8_t) 0x18)
    #define CH0_GAN_LSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x19 (CH1_CFG1_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    RESERVED   |                 CH1_MUX[2:0]                  |  CH1_INP_RNG  |  CH1_EX_RNG   |    CH1_BUFN   |    CH1_BUFP   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH1_CFG1_REG register */
    #define CH1_CFG1_REG_ADDRESS                                            ((uint8_t) 0x19)
    #define CH1_CFG1_REG_DEFAULT                                            ((uint8_t) 0x00)

    /* Channel input multiplexer selection */
    #define CH1_CFG1_REG_CH1_MUX_MASK                                       ((uint8_t) 0x70)
    #define CH1_CFG1_REG_CH1_MUX_0                                          ((uint8_t) 0x00)    // DEFAULT
    #define CH1_CFG1_REG_CH1_MUX_1                                          ((uint8_t) 0x10)
    #define CH1_CFG1_REG_CH1_MUX_2                                          ((uint8_t) 0x20)
    #define CH1_CFG1_REG_CH1_MUX_3                                          ((uint8_t) 0x30)
    #define CH1_CFG1_REG_CH1_MUX_4                                          ((uint8_t) 0x40)
    #define CH1_CFG1_REG_CH1_MUX_5                                          ((uint8_t) 0x50)
    #define CH1_CFG1_REG_CH1_MUX_6                                          ((uint8_t) 0x60)
    #define CH1_CFG1_REG_CH1_MUX_7                                          ((uint8_t) 0x70)

    /* Channel input range selection */
    #define CH1_CFG1_REG_CH1_INP_RNG_MASK                                   ((uint8_t) 0x08)
    #define CH1_CFG1_REG_CH1_INP_RNG_0                                      ((uint8_t) 0x00)    // DEFAULT
    #define CH1_CFG1_REG_CH1_INP_RNG_1                                      ((uint8_t) 0x08)

    /* Channel extended input range selection */
    #define CH1_CFG1_REG_CH1_EX_RNG_MASK                                    ((uint8_t) 0x04)
    #define CH1_CFG1_REG_CH1_EX_RNG_0                                       ((uint8_t) 0x00)    // DEFAULT
    #define CH1_CFG1_REG_CH1_EX_RNG_1                                       ((uint8_t) 0x04)

    /* Channel analog input negative buffer enable */
    #define CH1_CFG1_REG_CH1_BUFN_MASK                                      ((uint8_t) 0x02)
    #define CH1_CFG1_REG_CH1_BUFN_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH1_CFG1_REG_CH1_BUFN_1                                         ((uint8_t) 0x02)

    /* Channel analog input positive buffer enable */
    #define CH1_CFG1_REG_CH1_BUFP_MASK                                      ((uint8_t) 0x01)
    #define CH1_CFG1_REG_CH1_BUFP_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH1_CFG1_REG_CH1_BUFP_1                                         ((uint8_t) 0x01)



/* Register 0x1A (CH1_CFG2_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |           RESERVED            |    CH1_PWDN   |                                 CH1_FLTR[4:0]                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH1_CFG1_REG register */
    #define CH1_CFG2_REG_ADDRESS                                            ((uint8_t) 0x1A)
    #define CH1_CFG2_REG_DEFAULT                                            ((uint8_t) 0x00)

    /* Channel power-down mode selection */
    #define CH1_CFG2_REG_CH1_PWDN_MASK                                      ((uint8_t) 0x20)
    #define CH1_CFG2_REG_CH1_PWDN_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH1_CFG2_REG_CH1_PWDN_1                                         ((uint8_t) 0x20)

    /* Channel digital filter and oversampling ratio value selection */
    #define CH1_CFG2_REG_CH1_FLTR_MASK                                      ((uint8_t) 0x1F)
    #define CH1_CFG2_REG_CH1_FLTR_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH1_CFG2_REG_CH1_FLTR_1                                         ((uint8_t) 0x01)
    #define CH1_CFG2_REG_CH1_FLTR_2                                         ((uint8_t) 0x02)
    #define CH1_CFG2_REG_CH1_FLTR_3                                         ((uint8_t) 0x03)
    #define CH1_CFG2_REG_CH1_FLTR_4                                         ((uint8_t) 0x04)
    #define CH1_CFG2_REG_CH1_FLTR_5                                         ((uint8_t) 0x05)
    #define CH1_CFG2_REG_CH1_FLTR_6                                         ((uint8_t) 0x06)
    #define CH1_CFG2_REG_CH1_FLTR_7                                         ((uint8_t) 0x07)
    #define CH1_CFG2_REG_CH1_FLTR_8                                         ((uint8_t) 0x08)
    #define CH1_CFG2_REG_CH1_FLTR_9                                         ((uint8_t) 0x09)
    #define CH1_CFG2_REG_CH1_FLTR_10                                        ((uint8_t) 0x0A)
    #define CH1_CFG2_REG_CH1_FLTR_11                                        ((uint8_t) 0x0B)
    #define CH1_CFG2_REG_CH1_FLTR_12                                        ((uint8_t) 0x0C)
    #define CH1_CFG2_REG_CH1_FLTR_13                                        ((uint8_t) 0x0D)
    #define CH1_CFG2_REG_CH1_FLTR_14                                        ((uint8_t) 0x0E)
    #define CH1_CFG2_REG_CH1_FLTR_15                                        ((uint8_t) 0x0F)
    #define CH1_CFG2_REG_CH1_FLTR_16                                        ((uint8_t) 0x10)
    #define CH1_CFG2_REG_CH1_FLTR_17                                        ((uint8_t) 0x11)
    #define CH1_CFG2_REG_CH1_FLTR_18                                        ((uint8_t) 0x12)
    #define CH1_CFG2_REG_CH1_FLTR_19                                        ((uint8_t) 0x13)
    #define CH1_CFG2_REG_CH1_FLTR_20                                        ((uint8_t) 0x14)
    #define CH1_CFG2_REG_CH1_FLTR_21                                        ((uint8_t) 0x15)
    #define CH1_CFG2_REG_CH1_FLTR_22                                        ((uint8_t) 0x16)
    #define CH1_CFG2_REG_CH1_FLTR_23                                        ((uint8_t) 0x17)
    #define CH1_CFG2_REG_CH1_FLTR_24                                        ((uint8_t) 0x18)
    #define CH1_CFG2_REG_CH1_FLTR_25                                        ((uint8_t) 0x19)
    #define CH1_CFG2_REG_CH1_FLTR_26                                        ((uint8_t) 0x1A)
    #define CH1_CFG2_REG_CH1_FLTR_27                                        ((uint8_t) 0x1B)
    #define CH1_CFG2_REG_CH1_FLTR_28                                        ((uint8_t) 0x1C)
    #define CH1_CFG2_REG_CH1_FLTR_29                                        ((uint8_t) 0x1D)
    #define CH1_CFG2_REG_CH1_FLTR_30                                        ((uint8_t) 0x1E)
    #define CH1_CFG2_REG_CH1_FLTR_31                                        ((uint8_t) 0x1F)



/* Register 0x1B (CH1_OFS_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH1_OFFSET_MSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH1_OFS_MSB_REG register */
    #define CH1_OFS_MSB_REG_ADDRESS                                         ((uint8_t) 0x1B)
    #define CH1_OFS_MSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x1C (CH1_OFS_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH1_OFFSET_MID[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH1_OFS_MID_REG register */
    #define CH1_OFS_MID_REG_ADDRESS                                         ((uint8_t) 0x1C)
    #define CH1_OFS_MID_REG_DEFAULT                                         ((uint8_t) 0x00)

    

/* Register 0x1D (CH1_OFS_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH1_OFFSET_LSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH1_OFS_LSB_REG register */
    #define CH1_OFS_LSB_REG_ADDRESS                                         ((uint8_t) 0x1D)
    #define CH1_OFS_LSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x1E (CH1_GAN_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH1_GAIN_MSB[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH1_GAN_MSB_REG register */
    #define CH1_GAN_MSB_REG_ADDRESS                                         ((uint8_t) 0x1E)
    #define CH1_GAN_MSB_REG_DEFAULT                                         ((uint8_t) 0x40)



/* Register 0x1F (CH1_GAN_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH1_GAIN_MID[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */
 
    /* CH1_GAN_MID_REG register */
    #define CH1_GAN_MID_REG_ADDRESS                                         ((uint8_t) 0x1F)
    #define CH1_GAN_MID_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x20 (CH1_GAN_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH1_GAIN_LSB[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */
 
    /* CH1_GAN_LSB_REG register */
    #define CH1_GAN_LSB_REG_ADDRESS                                         ((uint8_t) 0x20)
    #define CH1_GAN_LSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x21 (CH2_CFG1_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    RESERVED   |                 CH2_MUX[2:0]                  |  CH2_INP_RNG  |  CH2_EX_RNG   |    CH2_BUFN   |    CH2_BUFP   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH2_CFG1_REG register */
    #define CH2_CFG1_REG_ADDRESS                                            ((uint8_t) 0x21)
    #define CH2_CFG1_REG_DEFAULT                                            ((uint8_t) 0x00)

    /* Channel input multiplexer selection */
    #define CH2_CFG1_REG_CH2_MUX_MASK                                       ((uint8_t) 0x70)
    #define CH2_CFG1_REG_CH2_MUX_0                                          ((uint8_t) 0x00)    // DEFAULT
    #define CH2_CFG1_REG_CH2_MUX_1                                          ((uint8_t) 0x10)
    #define CH2_CFG1_REG_CH2_MUX_2                                          ((uint8_t) 0x20)
    #define CH2_CFG1_REG_CH2_MUX_3                                          ((uint8_t) 0x30)
    #define CH2_CFG1_REG_CH2_MUX_4                                          ((uint8_t) 0x40)
    #define CH2_CFG1_REG_CH2_MUX_5                                          ((uint8_t) 0x50)
    #define CH2_CFG1_REG_CH2_MUX_6                                          ((uint8_t) 0x60)
    #define CH2_CFG1_REG_CH2_MUX_7                                          ((uint8_t) 0x70)

    /* Channel input range selection */
    #define CH2_CFG1_REG_CH2_INP_RNG_MASK                                   ((uint8_t) 0x08)
    #define CH2_CFG1_REG_CH2_INP_RNG_0                                      ((uint8_t) 0x00)    // DEFAULT
    #define CH2_CFG1_REG_CH2_INP_RNG_1                                      ((uint8_t) 0x08)

    /* Channel extended input range selection */
    #define CH2_CFG1_REG_CH2_EX_RNG_MASK                                    ((uint8_t) 0x04)
    #define CH2_CFG1_REG_CH2_EX_RNG_0                                       ((uint8_t) 0x00)    // DEFAULT
    #define CH2_CFG1_REG_CH2_EX_RNG_1                                       ((uint8_t) 0x04)

    /* Channel analog input negative buffer enable */
    #define CH2_CFG1_REG_CH2_BUFN_MASK                                      ((uint8_t) 0x02)
    #define CH2_CFG1_REG_CH2_BUFN_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH2_CFG1_REG_CH2_BUFN_1                                         ((uint8_t) 0x02)

    /* Channel analog input positive buffer enable */
    #define CH2_CFG1_REG_CH2_BUFP_MASK                                      ((uint8_t) 0x01)
    #define CH2_CFG1_REG_CH2_BUFP_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH2_CFG1_REG_CH2_BUFP_1                                         ((uint8_t) 0x01)



/* Register 0x22 (CH2_CFG2_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |           RESERVED            |    CH2_PWDN   |                                 CH2_FLTR[4:0]                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH2_CFG1_REG register */
    #define CH2_CFG2_REG_ADDRESS                                            ((uint8_t) 0x22)
    #define CH2_CFG2_REG_DEFAULT                                            ((uint8_t) 0x00)

    /* Channel power-down mode selection */
    #define CH2_CFG2_REG_CH2_PWDN_MASK                                      ((uint8_t) 0x20)
    #define CH2_CFG2_REG_CH2_PWDN_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH2_CFG2_REG_CH2_PWDN_1                                         ((uint8_t) 0x20)

    /* Channel digital filter and oversampling ratio value selection */
    #define CH2_CFG2_REG_CH2_FLTR_MASK                                      ((uint8_t) 0x1F)
    #define CH2_CFG2_REG_CH2_FLTR_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH2_CFG2_REG_CH2_FLTR_1                                         ((uint8_t) 0x01)
    #define CH2_CFG2_REG_CH2_FLTR_2                                         ((uint8_t) 0x02)
    #define CH2_CFG2_REG_CH2_FLTR_3                                         ((uint8_t) 0x03)
    #define CH2_CFG2_REG_CH2_FLTR_4                                         ((uint8_t) 0x04)
    #define CH2_CFG2_REG_CH2_FLTR_5                                         ((uint8_t) 0x05)
    #define CH2_CFG2_REG_CH2_FLTR_6                                         ((uint8_t) 0x06)
    #define CH2_CFG2_REG_CH2_FLTR_7                                         ((uint8_t) 0x07)
    #define CH2_CFG2_REG_CH2_FLTR_8                                         ((uint8_t) 0x08)
    #define CH2_CFG2_REG_CH2_FLTR_9                                         ((uint8_t) 0x09)
    #define CH2_CFG2_REG_CH2_FLTR_10                                        ((uint8_t) 0x0A)
    #define CH2_CFG2_REG_CH2_FLTR_11                                        ((uint8_t) 0x0B)
    #define CH2_CFG2_REG_CH2_FLTR_12                                        ((uint8_t) 0x0C)
    #define CH2_CFG2_REG_CH2_FLTR_13                                        ((uint8_t) 0x0D)
    #define CH2_CFG2_REG_CH2_FLTR_14                                        ((uint8_t) 0x0E)
    #define CH2_CFG2_REG_CH2_FLTR_15                                        ((uint8_t) 0x0F)
    #define CH2_CFG2_REG_CH2_FLTR_16                                        ((uint8_t) 0x10)
    #define CH2_CFG2_REG_CH2_FLTR_17                                        ((uint8_t) 0x11)
    #define CH2_CFG2_REG_CH2_FLTR_18                                        ((uint8_t) 0x12)
    #define CH2_CFG2_REG_CH2_FLTR_19                                        ((uint8_t) 0x13)
    #define CH2_CFG2_REG_CH2_FLTR_20                                        ((uint8_t) 0x14)
    #define CH2_CFG2_REG_CH2_FLTR_21                                        ((uint8_t) 0x15)
    #define CH2_CFG2_REG_CH2_FLTR_22                                        ((uint8_t) 0x16)
    #define CH2_CFG2_REG_CH2_FLTR_23                                        ((uint8_t) 0x17)
    #define CH2_CFG2_REG_CH2_FLTR_24                                        ((uint8_t) 0x18)
    #define CH2_CFG2_REG_CH2_FLTR_25                                        ((uint8_t) 0x19)
    #define CH2_CFG2_REG_CH2_FLTR_26                                        ((uint8_t) 0x1A)
    #define CH2_CFG2_REG_CH2_FLTR_27                                        ((uint8_t) 0x1B)
    #define CH2_CFG2_REG_CH2_FLTR_28                                        ((uint8_t) 0x1C)
    #define CH2_CFG2_REG_CH2_FLTR_29                                        ((uint8_t) 0x1D)
    #define CH2_CFG2_REG_CH2_FLTR_30                                        ((uint8_t) 0x1E)
    #define CH2_CFG2_REG_CH2_FLTR_31                                        ((uint8_t) 0x1F)



/* Register 0x23 (CH2_OFS_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH2_OFFSET_MSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH2_OFS_MSB_REG register */
    #define CH2_OFS_MSB_REG_ADDRESS                                         ((uint8_t) 0x23)
    #define CH2_OFS_MSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x24 (CH2_OFS_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH2_OFFSET_MID[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH2_OFS_MID_REG register */
    #define CH2_OFS_MID_REG_ADDRESS                                         ((uint8_t) 0x24)
    #define CH2_OFS_MID_REG_DEFAULT                                         ((uint8_t) 0x00)

    

/* Register 0x25 (CH2_OFS_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH2_OFFSET_LSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH2_OFS_LSB_REG register */
    #define CH2_OFS_LSB_REG_ADDRESS                                         ((uint8_t) 0x25)
    #define CH2_OFS_LSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x26 (CH2_GAN_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH2_GAIN_MSB[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH2_GAN_MSB_REG register */
    #define CH2_GAN_MSB_REG_ADDRESS                                         ((uint8_t) 0x26)
    #define CH2_GAN_MSB_REG_DEFAULT                                         ((uint8_t) 0x40)



/* Register 0x27 (CH2_GAN_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH2_GAIN_MID[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */
 
    /* CH2_GAN_MID_REG register */
    #define CH2_GAN_MID_REG_ADDRESS                                         ((uint8_t) 0x27)
    #define CH2_GAN_MID_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x28 (CH2_GAN_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH2_GAIN_LSB[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */
 
    /* CH2_GAN_LSB_REG register */
    #define CH2_GAN_LSB_REG_ADDRESS                                         ((uint8_t) 0x28)
    #define CH2_GAN_LSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x29 (CH3_CFG1_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    RESERVED   |                 CH3_MUX[2:0]                  |  CH3_INP_RNG  |  CH3_EX_RNG   |    CH3_BUFN   |    CH3_BUFP   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH3_CFG1_REG register */
    #define CH3_CFG1_REG_ADDRESS                                            ((uint8_t) 0x29)
    #define CH3_CFG1_REG_DEFAULT                                            ((uint8_t) 0x00)

    /* Channel input multiplexer selection */
    #define CH3_CFG1_REG_CH3_MUX_MASK                                       ((uint8_t) 0x70)
    #define CH3_CFG1_REG_CH3_MUX_0                                          ((uint8_t) 0x00)    // DEFAULT
    #define CH3_CFG1_REG_CH3_MUX_1                                          ((uint8_t) 0x10)
    #define CH3_CFG1_REG_CH3_MUX_2                                          ((uint8_t) 0x20)
    #define CH3_CFG1_REG_CH3_MUX_3                                          ((uint8_t) 0x30)
    #define CH3_CFG1_REG_CH3_MUX_4                                          ((uint8_t) 0x40)
    #define CH3_CFG1_REG_CH3_MUX_5                                          ((uint8_t) 0x50)
    #define CH3_CFG1_REG_CH3_MUX_6                                          ((uint8_t) 0x60)
    #define CH3_CFG1_REG_CH3_MUX_7                                          ((uint8_t) 0x70)

    /* Channel input range selection */
    #define CH3_CFG1_REG_CH3_INP_RNG_MASK                                   ((uint8_t) 0x08)
    #define CH3_CFG1_REG_CH3_INP_RNG_0                                      ((uint8_t) 0x00)    // DEFAULT
    #define CH3_CFG1_REG_CH3_INP_RNG_1                                      ((uint8_t) 0x08)

    /* Channel extended input range selection */
    #define CH3_CFG1_REG_CH3_EX_RNG_MASK                                    ((uint8_t) 0x04)
    #define CH3_CFG1_REG_CH3_EX_RNG_0                                       ((uint8_t) 0x00)    // DEFAULT
    #define CH3_CFG1_REG_CH3_EX_RNG_1                                       ((uint8_t) 0x04)

    /* Channel analog input negative buffer enable */
    #define CH3_CFG1_REG_CH3_BUFN_MASK                                      ((uint8_t) 0x02)
    #define CH3_CFG1_REG_CH3_BUFN_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH3_CFG1_REG_CH3_BUFN_1                                         ((uint8_t) 0x02)

    /* Channel analog input positive buffer enable */
    #define CH3_CFG1_REG_CH3_BUFP_MASK                                      ((uint8_t) 0x01)
    #define CH3_CFG1_REG_CH3_BUFP_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH3_CFG1_REG_CH3_BUFP_1                                         ((uint8_t) 0x01)



/* Register 0x2A (CH3_CFG2_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |           RESERVED            |    CH3_PWDN   |                                 CH3_FLTR[4:0]                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH3_CFG1_REG register */
    #define CH3_CFG2_REG_ADDRESS                                            ((uint8_t) 0x2A)
    #define CH3_CFG2_REG_DEFAULT                                            ((uint8_t) 0x00)

    /* Channel power-down mode selection */
    #define CH3_CFG2_REG_CH3_PWDN_MASK                                      ((uint8_t) 0x20)
    #define CH3_CFG2_REG_CH3_PWDN_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH3_CFG2_REG_CH3_PWDN_1                                         ((uint8_t) 0x20)

    /* Channel digital filter and oversampling ratio value selection */
    #define CH3_CFG2_REG_CH3_FLTR_MASK                                      ((uint8_t) 0x1F)
    #define CH3_CFG2_REG_CH3_FLTR_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH3_CFG2_REG_CH3_FLTR_1                                         ((uint8_t) 0x01)
    #define CH3_CFG2_REG_CH3_FLTR_2                                         ((uint8_t) 0x02)
    #define CH3_CFG2_REG_CH3_FLTR_3                                         ((uint8_t) 0x03)
    #define CH3_CFG2_REG_CH3_FLTR_4                                         ((uint8_t) 0x04)
    #define CH3_CFG2_REG_CH3_FLTR_5                                         ((uint8_t) 0x05)
    #define CH3_CFG2_REG_CH3_FLTR_6                                         ((uint8_t) 0x06)
    #define CH3_CFG2_REG_CH3_FLTR_7                                         ((uint8_t) 0x07)
    #define CH3_CFG2_REG_CH3_FLTR_8                                         ((uint8_t) 0x08)
    #define CH3_CFG2_REG_CH3_FLTR_9                                         ((uint8_t) 0x09)
    #define CH3_CFG2_REG_CH3_FLTR_10                                        ((uint8_t) 0x0A)
    #define CH3_CFG2_REG_CH3_FLTR_11                                        ((uint8_t) 0x0B)
    #define CH3_CFG2_REG_CH3_FLTR_12                                        ((uint8_t) 0x0C)
    #define CH3_CFG2_REG_CH3_FLTR_13                                        ((uint8_t) 0x0D)
    #define CH3_CFG2_REG_CH3_FLTR_14                                        ((uint8_t) 0x0E)
    #define CH3_CFG2_REG_CH3_FLTR_15                                        ((uint8_t) 0x0F)
    #define CH3_CFG2_REG_CH3_FLTR_16                                        ((uint8_t) 0x10)
    #define CH3_CFG2_REG_CH3_FLTR_17                                        ((uint8_t) 0x11)
    #define CH3_CFG2_REG_CH3_FLTR_18                                        ((uint8_t) 0x12)
    #define CH3_CFG2_REG_CH3_FLTR_19                                        ((uint8_t) 0x13)
    #define CH3_CFG2_REG_CH3_FLTR_20                                        ((uint8_t) 0x14)
    #define CH3_CFG2_REG_CH3_FLTR_21                                        ((uint8_t) 0x15)
    #define CH3_CFG2_REG_CH3_FLTR_22                                        ((uint8_t) 0x16)
    #define CH3_CFG2_REG_CH3_FLTR_23                                        ((uint8_t) 0x17)
    #define CH3_CFG2_REG_CH3_FLTR_24                                        ((uint8_t) 0x18)
    #define CH3_CFG2_REG_CH3_FLTR_25                                        ((uint8_t) 0x19)
    #define CH3_CFG2_REG_CH3_FLTR_26                                        ((uint8_t) 0x1A)
    #define CH3_CFG2_REG_CH3_FLTR_27                                        ((uint8_t) 0x1B)
    #define CH3_CFG2_REG_CH3_FLTR_28                                        ((uint8_t) 0x1C)
    #define CH3_CFG2_REG_CH3_FLTR_29                                        ((uint8_t) 0x1D)
    #define CH3_CFG2_REG_CH3_FLTR_30                                        ((uint8_t) 0x1E)
    #define CH3_CFG2_REG_CH3_FLTR_31                                        ((uint8_t) 0x1F)



/* Register 0x2B (CH3_OFS_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH3_OFFSET_MSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH3_OFS_MSB_REG register */
    #define CH3_OFS_MSB_REG_ADDRESS                                         ((uint8_t) 0x2B)
    #define CH3_OFS_MSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x2C (CH3_OFS_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH3_OFFSET_MID[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH3_OFS_MID_REG register */
    #define CH3_OFS_MID_REG_ADDRESS                                         ((uint8_t) 0x2C)
    #define CH3_OFS_MID_REG_DEFAULT                                         ((uint8_t) 0x00)

    

/* Register 0x2D (CH3_OFS_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH3_OFFSET_LSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH3_OFS_LSB_REG register */
    #define CH3_OFS_LSB_REG_ADDRESS                                         ((uint8_t) 0x2D)
    #define CH3_OFS_LSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x2E (CH3_GAN_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH3_GAIN_MSB[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH3_GAN_MSB_REG register */
    #define CH3_GAN_MSB_REG_ADDRESS                                         ((uint8_t) 0x2E)
    #define CH3_GAN_MSB_REG_DEFAULT                                         ((uint8_t) 0x40)



/* Register 0x2F (CH3_GAN_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH3_GAIN_MID[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */
 
    /* CH3_GAN_MID_REG register */
    #define CH3_GAN_MID_REG_ADDRESS                                         ((uint8_t) 0x2F)
    #define CH3_GAN_MID_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x30 (CH3_GAN_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH3_GAIN_LSB[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */
 
    /* CH3_GAN_LSB_REG register */
    #define CH3_GAN_LSB_REG_ADDRESS                                         ((uint8_t) 0x30)
    #define CH3_GAN_LSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x31 (CH4_CFG1_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    RESERVED   |                 CH4_MUX[2:0]                  |  CH4_INP_RNG  |  CH4_EX_RNG   |    CH4_BUFN   |    CH4_BUFP   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH4_CFG1_REG register */
    #define CH4_CFG1_REG_ADDRESS                                            ((uint8_t) 0x31)
    #define CH4_CFG1_REG_DEFAULT                                            ((uint8_t) 0x00)

    /* Channel input multiplexer selection */
    #define CH4_CFG1_REG_CH4_MUX_MASK                                       ((uint8_t) 0x70)
    #define CH4_CFG1_REG_CH4_MUX_0                                          ((uint8_t) 0x00)    // DEFAULT
    #define CH4_CFG1_REG_CH4_MUX_1                                          ((uint8_t) 0x10)
    #define CH4_CFG1_REG_CH4_MUX_2                                          ((uint8_t) 0x20)
    #define CH4_CFG1_REG_CH4_MUX_3                                          ((uint8_t) 0x30)
    #define CH4_CFG1_REG_CH4_MUX_4                                          ((uint8_t) 0x40)
    #define CH4_CFG1_REG_CH4_MUX_5                                          ((uint8_t) 0x50)
    #define CH4_CFG1_REG_CH4_MUX_6                                          ((uint8_t) 0x60)
    #define CH4_CFG1_REG_CH4_MUX_7                                          ((uint8_t) 0x70)

    /* Channel input range selection */
    #define CH4_CFG1_REG_CH4_INP_RNG_MASK                                   ((uint8_t) 0x08)
    #define CH4_CFG1_REG_CH4_INP_RNG_0                                      ((uint8_t) 0x00)    // DEFAULT
    #define CH4_CFG1_REG_CH4_INP_RNG_1                                      ((uint8_t) 0x08)

    /* Channel extended input range selection */
    #define CH4_CFG1_REG_CH4_EX_RNG_MASK                                    ((uint8_t) 0x04)
    #define CH4_CFG1_REG_CH4_EX_RNG_0                                       ((uint8_t) 0x00)    // DEFAULT
    #define CH4_CFG1_REG_CH4_EX_RNG_1                                       ((uint8_t) 0x04)

    /* Channel analog input negative buffer enable */
    #define CH4_CFG1_REG_CH4_BUFN_MASK                                      ((uint8_t) 0x02)
    #define CH4_CFG1_REG_CH4_BUFN_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH4_CFG1_REG_CH4_BUFN_1                                         ((uint8_t) 0x02)

    /* Channel analog input positive buffer enable */
    #define CH4_CFG1_REG_CH4_BUFP_MASK                                      ((uint8_t) 0x01)
    #define CH4_CFG1_REG_CH4_BUFP_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH4_CFG1_REG_CH4_BUFP_1                                         ((uint8_t) 0x01)



/* Register 0x32 (CH4_CFG2_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |           RESERVED            |    CH4_PWDN   |                                 CH4_FLTR[4:0]                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH4_CFG1_REG register */
    #define CH4_CFG2_REG_ADDRESS                                            ((uint8_t) 0x32)
    #define CH4_CFG2_REG_DEFAULT                                            ((uint8_t) 0x00)

    /* Channel power-down mode selection */
    #define CH4_CFG2_REG_CH4_PWDN_MASK                                      ((uint8_t) 0x20)
    #define CH4_CFG2_REG_CH4_PWDN_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH4_CFG2_REG_CH4_PWDN_1                                         ((uint8_t) 0x20)

    /* Channel digital filter and oversampling ratio value selection */
    #define CH4_CFG2_REG_CH4_FLTR_MASK                                      ((uint8_t) 0x1F)
    #define CH4_CFG2_REG_CH4_FLTR_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH4_CFG2_REG_CH4_FLTR_1                                         ((uint8_t) 0x01)
    #define CH4_CFG2_REG_CH4_FLTR_2                                         ((uint8_t) 0x02)
    #define CH4_CFG2_REG_CH4_FLTR_3                                         ((uint8_t) 0x03)
    #define CH4_CFG2_REG_CH4_FLTR_4                                         ((uint8_t) 0x04)
    #define CH4_CFG2_REG_CH4_FLTR_5                                         ((uint8_t) 0x05)
    #define CH4_CFG2_REG_CH4_FLTR_6                                         ((uint8_t) 0x06)
    #define CH4_CFG2_REG_CH4_FLTR_7                                         ((uint8_t) 0x07)
    #define CH4_CFG2_REG_CH4_FLTR_8                                         ((uint8_t) 0x08)
    #define CH4_CFG2_REG_CH4_FLTR_9                                         ((uint8_t) 0x09)
    #define CH4_CFG2_REG_CH4_FLTR_10                                        ((uint8_t) 0x0A)
    #define CH4_CFG2_REG_CH4_FLTR_11                                        ((uint8_t) 0x0B)
    #define CH4_CFG2_REG_CH4_FLTR_12                                        ((uint8_t) 0x0C)
    #define CH4_CFG2_REG_CH4_FLTR_13                                        ((uint8_t) 0x0D)
    #define CH4_CFG2_REG_CH4_FLTR_14                                        ((uint8_t) 0x0E)
    #define CH4_CFG2_REG_CH4_FLTR_15                                        ((uint8_t) 0x0F)
    #define CH4_CFG2_REG_CH4_FLTR_16                                        ((uint8_t) 0x10)
    #define CH4_CFG2_REG_CH4_FLTR_17                                        ((uint8_t) 0x11)
    #define CH4_CFG2_REG_CH4_FLTR_18                                        ((uint8_t) 0x12)
    #define CH4_CFG2_REG_CH4_FLTR_19                                        ((uint8_t) 0x13)
    #define CH4_CFG2_REG_CH4_FLTR_20                                        ((uint8_t) 0x14)
    #define CH4_CFG2_REG_CH4_FLTR_21                                        ((uint8_t) 0x15)
    #define CH4_CFG2_REG_CH4_FLTR_22                                        ((uint8_t) 0x16)
    #define CH4_CFG2_REG_CH4_FLTR_23                                        ((uint8_t) 0x17)
    #define CH4_CFG2_REG_CH4_FLTR_24                                        ((uint8_t) 0x18)
    #define CH4_CFG2_REG_CH4_FLTR_25                                        ((uint8_t) 0x19)
    #define CH4_CFG2_REG_CH4_FLTR_26                                        ((uint8_t) 0x1A)
    #define CH4_CFG2_REG_CH4_FLTR_27                                        ((uint8_t) 0x1B)
    #define CH4_CFG2_REG_CH4_FLTR_28                                        ((uint8_t) 0x1C)
    #define CH4_CFG2_REG_CH4_FLTR_29                                        ((uint8_t) 0x1D)
    #define CH4_CFG2_REG_CH4_FLTR_30                                        ((uint8_t) 0x1E)
    #define CH4_CFG2_REG_CH4_FLTR_31                                        ((uint8_t) 0x1F)



/* Register 0x33 (CH4_OFS_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH4_OFFSET_MSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH4_OFS_MSB_REG register */
    #define CH4_OFS_MSB_REG_ADDRESS                                         ((uint8_t) 0x33)
    #define CH4_OFS_MSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x34 (CH4_OFS_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH4_OFFSET_MID[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH4_OFS_MID_REG register */
    #define CH4_OFS_MID_REG_ADDRESS                                         ((uint8_t) 0x34)
    #define CH4_OFS_MID_REG_DEFAULT                                         ((uint8_t) 0x00)

    

/* Register 0x35 (CH4_OFS_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH4_OFFSET_LSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH4_OFS_LSB_REG register */
    #define CH4_OFS_LSB_REG_ADDRESS                                         ((uint8_t) 0x35)
    #define CH4_OFS_LSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x36 (CH4_GAN_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH4_GAIN_MSB[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH4_GAN_MSB_REG register */
    #define CH4_GAN_MSB_REG_ADDRESS                                         ((uint8_t) 0x36)
    #define CH4_GAN_MSB_REG_DEFAULT                                         ((uint8_t) 0x40)



/* Register 0x37 (CH4_GAN_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH4_GAIN_MID[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */
 
    /* CH4_GAN_MID_REG register */
    #define CH4_GAN_MID_REG_ADDRESS                                         ((uint8_t) 0x37)
    #define CH4_GAN_MID_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x38 (CH4_GAN_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH4_GAIN_LSB[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */
 
    /* CH4_GAN_LSB_REG register */
    #define CH4_GAN_LSB_REG_ADDRESS                                         ((uint8_t) 0x38)
    #define CH4_GAN_LSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x39 (CH5_CFG1_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    RESERVED   |                 CH5_MUX[2:0]                  |  CH5_INP_RNG  |  CH5_EX_RNG   |    CH5_BUFN   |    CH5_BUFP   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH5_CFG1_REG register */
    #define CH5_CFG1_REG_ADDRESS                                            ((uint8_t) 0x39)
    #define CH5_CFG1_REG_DEFAULT                                            ((uint8_t) 0x00)

    /* Channel input multiplexer selection */
    #define CH5_CFG1_REG_CH5_MUX_MASK                                       ((uint8_t) 0x70)
    #define CH5_CFG1_REG_CH5_MUX_0                                          ((uint8_t) 0x00)    // DEFAULT
    #define CH5_CFG1_REG_CH5_MUX_1                                          ((uint8_t) 0x10)
    #define CH5_CFG1_REG_CH5_MUX_2                                          ((uint8_t) 0x20)
    #define CH5_CFG1_REG_CH5_MUX_3                                          ((uint8_t) 0x30)
    #define CH5_CFG1_REG_CH5_MUX_4                                          ((uint8_t) 0x40)
    #define CH5_CFG1_REG_CH5_MUX_5                                          ((uint8_t) 0x50)
    #define CH5_CFG1_REG_CH5_MUX_6                                          ((uint8_t) 0x60)
    #define CH5_CFG1_REG_CH5_MUX_7                                          ((uint8_t) 0x70)

    /* Channel input range selection */
    #define CH5_CFG1_REG_CH5_INP_RNG_MASK                                   ((uint8_t) 0x08)
    #define CH5_CFG1_REG_CH5_INP_RNG_0                                      ((uint8_t) 0x00)    // DEFAULT
    #define CH5_CFG1_REG_CH5_INP_RNG_1                                      ((uint8_t) 0x08)

    /* Channel extended input range selection */
    #define CH5_CFG1_REG_CH5_EX_RNG_MASK                                    ((uint8_t) 0x04)
    #define CH5_CFG1_REG_CH5_EX_RNG_0                                       ((uint8_t) 0x00)    // DEFAULT
    #define CH5_CFG1_REG_CH5_EX_RNG_1                                       ((uint8_t) 0x04)

    /* Channel analog input negative buffer enable */
    #define CH5_CFG1_REG_CH5_BUFN_MASK                                      ((uint8_t) 0x02)
    #define CH5_CFG1_REG_CH5_BUFN_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH5_CFG1_REG_CH5_BUFN_1                                         ((uint8_t) 0x02)

    /* Channel analog input positive buffer enable */
    #define CH5_CFG1_REG_CH5_BUFP_MASK                                      ((uint8_t) 0x01)
    #define CH5_CFG1_REG_CH5_BUFP_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH5_CFG1_REG_CH5_BUFP_1                                         ((uint8_t) 0x01)



/* Register 0x3A (CH5_CFG2_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |           RESERVED            |    CH5_PWDN   |                                 CH5_FLTR[4:0]                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH5_CFG1_REG register */
    #define CH5_CFG2_REG_ADDRESS                                            ((uint8_t) 0x3A)
    #define CH5_CFG2_REG_DEFAULT                                            ((uint8_t) 0x00)

    /* Channel power-down mode selection */
    #define CH5_CFG2_REG_CH5_PWDN_MASK                                      ((uint8_t) 0x20)
    #define CH5_CFG2_REG_CH5_PWDN_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH5_CFG2_REG_CH5_PWDN_1                                         ((uint8_t) 0x20)

    /* Channel digital filter and oversampling ratio value selection */
    #define CH5_CFG2_REG_CH5_FLTR_MASK                                      ((uint8_t) 0x1F)
    #define CH5_CFG2_REG_CH5_FLTR_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH5_CFG2_REG_CH5_FLTR_1                                         ((uint8_t) 0x01)
    #define CH5_CFG2_REG_CH5_FLTR_2                                         ((uint8_t) 0x02)
    #define CH5_CFG2_REG_CH5_FLTR_3                                         ((uint8_t) 0x03)
    #define CH5_CFG2_REG_CH5_FLTR_4                                         ((uint8_t) 0x04)
    #define CH5_CFG2_REG_CH5_FLTR_5                                         ((uint8_t) 0x05)
    #define CH5_CFG2_REG_CH5_FLTR_6                                         ((uint8_t) 0x06)
    #define CH5_CFG2_REG_CH5_FLTR_7                                         ((uint8_t) 0x07)
    #define CH5_CFG2_REG_CH5_FLTR_8                                         ((uint8_t) 0x08)
    #define CH5_CFG2_REG_CH5_FLTR_9                                         ((uint8_t) 0x09)
    #define CH5_CFG2_REG_CH5_FLTR_10                                        ((uint8_t) 0x0A)
    #define CH5_CFG2_REG_CH5_FLTR_11                                        ((uint8_t) 0x0B)
    #define CH5_CFG2_REG_CH5_FLTR_12                                        ((uint8_t) 0x0C)
    #define CH5_CFG2_REG_CH5_FLTR_13                                        ((uint8_t) 0x0D)
    #define CH5_CFG2_REG_CH5_FLTR_14                                        ((uint8_t) 0x0E)
    #define CH5_CFG2_REG_CH5_FLTR_15                                        ((uint8_t) 0x0F)
    #define CH5_CFG2_REG_CH5_FLTR_16                                        ((uint8_t) 0x10)
    #define CH5_CFG2_REG_CH5_FLTR_17                                        ((uint8_t) 0x11)
    #define CH5_CFG2_REG_CH5_FLTR_18                                        ((uint8_t) 0x12)
    #define CH5_CFG2_REG_CH5_FLTR_19                                        ((uint8_t) 0x13)
    #define CH5_CFG2_REG_CH5_FLTR_20                                        ((uint8_t) 0x14)
    #define CH5_CFG2_REG_CH5_FLTR_21                                        ((uint8_t) 0x15)
    #define CH5_CFG2_REG_CH5_FLTR_22                                        ((uint8_t) 0x16)
    #define CH5_CFG2_REG_CH5_FLTR_23                                        ((uint8_t) 0x17)
    #define CH5_CFG2_REG_CH5_FLTR_24                                        ((uint8_t) 0x18)
    #define CH5_CFG2_REG_CH5_FLTR_25                                        ((uint8_t) 0x19)
    #define CH5_CFG2_REG_CH5_FLTR_26                                        ((uint8_t) 0x1A)
    #define CH5_CFG2_REG_CH5_FLTR_27                                        ((uint8_t) 0x1B)
    #define CH5_CFG2_REG_CH5_FLTR_28                                        ((uint8_t) 0x1C)
    #define CH5_CFG2_REG_CH5_FLTR_29                                        ((uint8_t) 0x1D)
    #define CH5_CFG2_REG_CH5_FLTR_30                                        ((uint8_t) 0x1E)
    #define CH5_CFG2_REG_CH5_FLTR_31                                        ((uint8_t) 0x1F)



/* Register 0x3B (CH5_OFS_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH5_OFFSET_MSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH5_OFS_MSB_REG register */
    #define CH5_OFS_MSB_REG_ADDRESS                                         ((uint8_t) 0x3B)
    #define CH5_OFS_MSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x3C (CH5_OFS_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH5_OFFSET_MID[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH5_OFS_MID_REG register */
    #define CH5_OFS_MID_REG_ADDRESS                                         ((uint8_t) 0x3C)
    #define CH5_OFS_MID_REG_DEFAULT                                         ((uint8_t) 0x00)

    

/* Register 0x3D (CH5_OFS_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH5_OFFSET_LSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH5_OFS_LSB_REG register */
    #define CH5_OFS_LSB_REG_ADDRESS                                         ((uint8_t) 0x3D)
    #define CH5_OFS_LSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x3E (CH5_GAN_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH5_GAIN_MSB[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH5_GAN_MSB_REG register */
    #define CH5_GAN_MSB_REG_ADDRESS                                         ((uint8_t) 0x3E)
    #define CH5_GAN_MSB_REG_DEFAULT                                         ((uint8_t) 0x40)



/* Register 0x3F (CH5_GAN_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH5_GAIN_MID[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */
 
    /* CH5_GAN_MID_REG register */
    #define CH5_GAN_MID_REG_ADDRESS                                         ((uint8_t) 0x3F)
    #define CH5_GAN_MID_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x40 (CH5_GAN_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH5_GAIN_LSB[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */
 
    /* CH5_GAN_LSB_REG register */
    #define CH5_GAN_LSB_REG_ADDRESS                                         ((uint8_t) 0x40)
    #define CH5_GAN_LSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x41 (CH6_CFG1_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    RESERVED   |                 CH6_MUX[2:0]                  |  CH6_INP_RNG  |  CH6_EX_RNG   |    CH6_BUFN   |    CH6_BUFP   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH6_CFG1_REG register */
    #define CH6_CFG1_REG_ADDRESS                                            ((uint8_t) 0x41)
    #define CH6_CFG1_REG_DEFAULT                                            ((uint8_t) 0x00)

    /* Channel input multiplexer selection */
    #define CH6_CFG1_REG_CH6_MUX_MASK                                       ((uint8_t) 0x70)
    #define CH6_CFG1_REG_CH6_MUX_0                                          ((uint8_t) 0x00)    // DEFAULT
    #define CH6_CFG1_REG_CH6_MUX_1                                          ((uint8_t) 0x10)
    #define CH6_CFG1_REG_CH6_MUX_2                                          ((uint8_t) 0x20)
    #define CH6_CFG1_REG_CH6_MUX_3                                          ((uint8_t) 0x30)
    #define CH6_CFG1_REG_CH6_MUX_4                                          ((uint8_t) 0x40)
    #define CH6_CFG1_REG_CH6_MUX_5                                          ((uint8_t) 0x50)
    #define CH6_CFG1_REG_CH6_MUX_6                                          ((uint8_t) 0x60)
    #define CH6_CFG1_REG_CH6_MUX_7                                          ((uint8_t) 0x70)

    /* Channel input range selection */
    #define CH6_CFG1_REG_CH6_INP_RNG_MASK                                   ((uint8_t) 0x08)
    #define CH6_CFG1_REG_CH6_INP_RNG_0                                      ((uint8_t) 0x00)    // DEFAULT
    #define CH6_CFG1_REG_CH6_INP_RNG_1                                      ((uint8_t) 0x08)

    /* Channel extended input range selection */
    #define CH6_CFG1_REG_CH6_EX_RNG_MASK                                    ((uint8_t) 0x04)
    #define CH6_CFG1_REG_CH6_EX_RNG_0                                       ((uint8_t) 0x00)    // DEFAULT
    #define CH6_CFG1_REG_CH6_EX_RNG_1                                       ((uint8_t) 0x04)

    /* Channel analog input negative buffer enable */
    #define CH6_CFG1_REG_CH6_BUFN_MASK                                      ((uint8_t) 0x02)
    #define CH6_CFG1_REG_CH6_BUFN_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH6_CFG1_REG_CH6_BUFN_1                                         ((uint8_t) 0x02)

    /* Channel analog input positive buffer enable */
    #define CH6_CFG1_REG_CH6_BUFP_MASK                                      ((uint8_t) 0x01)
    #define CH6_CFG1_REG_CH6_BUFP_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH6_CFG1_REG_CH6_BUFP_1                                         ((uint8_t) 0x01)



/* Register 0x42 (CH6_CFG2_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |           RESERVED            |    CH6_PWDN   |                                 CH6_FLTR[4:0]                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH6_CFG1_REG register */
    #define CH6_CFG2_REG_ADDRESS                                            ((uint8_t) 0x42)
    #define CH6_CFG2_REG_DEFAULT                                            ((uint8_t) 0x00)

    /* Channel power-down mode selection */
    #define CH6_CFG2_REG_CH6_PWDN_MASK                                      ((uint8_t) 0x20)
    #define CH6_CFG2_REG_CH6_PWDN_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH6_CFG2_REG_CH6_PWDN_1                                         ((uint8_t) 0x20)

    /* Channel digital filter and oversampling ratio value selection */
    #define CH6_CFG2_REG_CH6_FLTR_MASK                                      ((uint8_t) 0x1F)
    #define CH6_CFG2_REG_CH6_FLTR_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH6_CFG2_REG_CH6_FLTR_1                                         ((uint8_t) 0x01)
    #define CH6_CFG2_REG_CH6_FLTR_2                                         ((uint8_t) 0x02)
    #define CH6_CFG2_REG_CH6_FLTR_3                                         ((uint8_t) 0x03)
    #define CH6_CFG2_REG_CH6_FLTR_4                                         ((uint8_t) 0x04)
    #define CH6_CFG2_REG_CH6_FLTR_5                                         ((uint8_t) 0x05)
    #define CH6_CFG2_REG_CH6_FLTR_6                                         ((uint8_t) 0x06)
    #define CH6_CFG2_REG_CH6_FLTR_7                                         ((uint8_t) 0x07)
    #define CH6_CFG2_REG_CH6_FLTR_8                                         ((uint8_t) 0x08)
    #define CH6_CFG2_REG_CH6_FLTR_9                                         ((uint8_t) 0x09)
    #define CH6_CFG2_REG_CH6_FLTR_10                                        ((uint8_t) 0x0A)
    #define CH6_CFG2_REG_CH6_FLTR_11                                        ((uint8_t) 0x0B)
    #define CH6_CFG2_REG_CH6_FLTR_12                                        ((uint8_t) 0x0C)
    #define CH6_CFG2_REG_CH6_FLTR_13                                        ((uint8_t) 0x0D)
    #define CH6_CFG2_REG_CH6_FLTR_14                                        ((uint8_t) 0x0E)
    #define CH6_CFG2_REG_CH6_FLTR_15                                        ((uint8_t) 0x0F)
    #define CH6_CFG2_REG_CH6_FLTR_16                                        ((uint8_t) 0x10)
    #define CH6_CFG2_REG_CH6_FLTR_17                                        ((uint8_t) 0x11)
    #define CH6_CFG2_REG_CH6_FLTR_18                                        ((uint8_t) 0x12)
    #define CH6_CFG2_REG_CH6_FLTR_19                                        ((uint8_t) 0x13)
    #define CH6_CFG2_REG_CH6_FLTR_20                                        ((uint8_t) 0x14)
    #define CH6_CFG2_REG_CH6_FLTR_21                                        ((uint8_t) 0x15)
    #define CH6_CFG2_REG_CH6_FLTR_22                                        ((uint8_t) 0x16)
    #define CH6_CFG2_REG_CH6_FLTR_23                                        ((uint8_t) 0x17)
    #define CH6_CFG2_REG_CH6_FLTR_24                                        ((uint8_t) 0x18)
    #define CH6_CFG2_REG_CH6_FLTR_25                                        ((uint8_t) 0x19)
    #define CH6_CFG2_REG_CH6_FLTR_26                                        ((uint8_t) 0x1A)
    #define CH6_CFG2_REG_CH6_FLTR_27                                        ((uint8_t) 0x1B)
    #define CH6_CFG2_REG_CH6_FLTR_28                                        ((uint8_t) 0x1C)
    #define CH6_CFG2_REG_CH6_FLTR_29                                        ((uint8_t) 0x1D)
    #define CH6_CFG2_REG_CH6_FLTR_30                                        ((uint8_t) 0x1E)
    #define CH6_CFG2_REG_CH6_FLTR_31                                        ((uint8_t) 0x1F)



/* Register 0x43 (CH6_OFS_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH6_OFFSET_MSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH6_OFS_MSB_REG register */
    #define CH6_OFS_MSB_REG_ADDRESS                                         ((uint8_t) 0x43)
    #define CH6_OFS_MSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x44 (CH6_OFS_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH6_OFFSET_MID[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH6_OFS_MID_REG register */
    #define CH6_OFS_MID_REG_ADDRESS                                         ((uint8_t) 0x44)
    #define CH6_OFS_MID_REG_DEFAULT                                         ((uint8_t) 0x00)

    

/* Register 0x45 (CH6_OFS_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH6_OFFSET_LSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH6_OFS_LSB_REG register */
    #define CH6_OFS_LSB_REG_ADDRESS                                         ((uint8_t) 0x45)
    #define CH6_OFS_LSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x46 (CH6_GAN_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH6_GAIN_MSB[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH6_GAN_MSB_REG register */
    #define CH6_GAN_MSB_REG_ADDRESS                                         ((uint8_t) 0x46)
    #define CH6_GAN_MSB_REG_DEFAULT                                         ((uint8_t) 0x40)



/* Register 0x47 (CH6_GAN_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH6_GAIN_MID[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */
 
    /* CH6_GAN_MID_REG register */
    #define CH6_GAN_MID_REG_ADDRESS                                         ((uint8_t) 0x47)
    #define CH6_GAN_MID_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x48 (CH6_GAN_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH6_GAIN_LSB[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */
 
    /* CH6_GAN_LSB_REG register */
    #define CH6_GAN_LSB_REG_ADDRESS                                         ((uint8_t) 0x48)
    #define CH6_GAN_LSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x49 (CH7_CFG1_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |    RESERVED   |                 CH7_MUX[2:0]                  |  CH7_INP_RNG  |  CH7_EX_RNG   |    CH7_BUFN   |    CH7_BUFP   |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH7_CFG1_REG register */
    #define CH7_CFG1_REG_ADDRESS                                            ((uint8_t) 0x49)
    #define CH7_CFG1_REG_DEFAULT                                            ((uint8_t) 0x00)

    /* Channel input multiplexer selection */
    #define CH7_CFG1_REG_CH7_MUX_MASK                                       ((uint8_t) 0x70)
    #define CH7_CFG1_REG_CH7_MUX_0                                          ((uint8_t) 0x00)    // DEFAULT
    #define CH7_CFG1_REG_CH7_MUX_1                                          ((uint8_t) 0x10)
    #define CH7_CFG1_REG_CH7_MUX_2                                          ((uint8_t) 0x20)
    #define CH7_CFG1_REG_CH7_MUX_3                                          ((uint8_t) 0x30)
    #define CH7_CFG1_REG_CH7_MUX_4                                          ((uint8_t) 0x40)
    #define CH7_CFG1_REG_CH7_MUX_5                                          ((uint8_t) 0x50)
    #define CH7_CFG1_REG_CH7_MUX_6                                          ((uint8_t) 0x60)
    #define CH7_CFG1_REG_CH7_MUX_7                                          ((uint8_t) 0x70)

    /* Channel input range selection */
    #define CH7_CFG1_REG_CH7_INP_RNG_MASK                                   ((uint8_t) 0x08)
    #define CH7_CFG1_REG_CH7_INP_RNG_0                                      ((uint8_t) 0x00)    // DEFAULT
    #define CH7_CFG1_REG_CH7_INP_RNG_1                                      ((uint8_t) 0x08)

    /* Channel extended input range selection */
    #define CH7_CFG1_REG_CH7_EX_RNG_MASK                                    ((uint8_t) 0x04)
    #define CH7_CFG1_REG_CH7_EX_RNG_0                                       ((uint8_t) 0x00)    // DEFAULT
    #define CH7_CFG1_REG_CH7_EX_RNG_1                                       ((uint8_t) 0x04)

    /* Channel analog input negative buffer enable */
    #define CH7_CFG1_REG_CH7_BUFN_MASK                                      ((uint8_t) 0x02)
    #define CH7_CFG1_REG_CH7_BUFN_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH7_CFG1_REG_CH7_BUFN_1                                         ((uint8_t) 0x02)

    /* Channel analog input positive buffer enable */
    #define CH7_CFG1_REG_CH7_BUFP_MASK                                      ((uint8_t) 0x01)
    #define CH7_CFG1_REG_CH7_BUFP_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH7_CFG1_REG_CH7_BUFP_1                                         ((uint8_t) 0x01)



/* Register 0x4A (CH7_CFG2_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |           RESERVED            |    CH7_PWDN   |                                 CH7_FLTR[4:0]                                 |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH7_CFG1_REG register */
    #define CH7_CFG2_REG_ADDRESS                                            ((uint8_t) 0x4A)
    #define CH7_CFG2_REG_DEFAULT                                            ((uint8_t) 0x00)

    /* Channel power-down mode selection */
    #define CH7_CFG2_REG_CH7_PWDN_MASK                                      ((uint8_t) 0x20)
    #define CH7_CFG2_REG_CH7_PWDN_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH7_CFG2_REG_CH7_PWDN_1                                         ((uint8_t) 0x20)

    /* Channel digital filter and oversampling ratio value selection */
    #define CH7_CFG2_REG_CH7_FLTR_MASK                                      ((uint8_t) 0x1F)
    #define CH7_CFG2_REG_CH7_FLTR_0                                         ((uint8_t) 0x00)    // DEFAULT
    #define CH7_CFG2_REG_CH7_FLTR_1                                         ((uint8_t) 0x01)
    #define CH7_CFG2_REG_CH7_FLTR_2                                         ((uint8_t) 0x02)
    #define CH7_CFG2_REG_CH7_FLTR_3                                         ((uint8_t) 0x03)
    #define CH7_CFG2_REG_CH7_FLTR_4                                         ((uint8_t) 0x04)
    #define CH7_CFG2_REG_CH7_FLTR_5                                         ((uint8_t) 0x05)
    #define CH7_CFG2_REG_CH7_FLTR_6                                         ((uint8_t) 0x06)
    #define CH7_CFG2_REG_CH7_FLTR_7                                         ((uint8_t) 0x07)
    #define CH7_CFG2_REG_CH7_FLTR_8                                         ((uint8_t) 0x08)
    #define CH7_CFG2_REG_CH7_FLTR_9                                         ((uint8_t) 0x09)
    #define CH7_CFG2_REG_CH7_FLTR_10                                        ((uint8_t) 0x0A)
    #define CH7_CFG2_REG_CH7_FLTR_11                                        ((uint8_t) 0x0B)
    #define CH7_CFG2_REG_CH7_FLTR_12                                        ((uint8_t) 0x0C)
    #define CH7_CFG2_REG_CH7_FLTR_13                                        ((uint8_t) 0x0D)
    #define CH7_CFG2_REG_CH7_FLTR_14                                        ((uint8_t) 0x0E)
    #define CH7_CFG2_REG_CH7_FLTR_15                                        ((uint8_t) 0x0F)
    #define CH7_CFG2_REG_CH7_FLTR_16                                        ((uint8_t) 0x10)
    #define CH7_CFG2_REG_CH7_FLTR_17                                        ((uint8_t) 0x11)
    #define CH7_CFG2_REG_CH7_FLTR_18                                        ((uint8_t) 0x12)
    #define CH7_CFG2_REG_CH7_FLTR_19                                        ((uint8_t) 0x13)
    #define CH7_CFG2_REG_CH7_FLTR_20                                        ((uint8_t) 0x14)
    #define CH7_CFG2_REG_CH7_FLTR_21                                        ((uint8_t) 0x15)
    #define CH7_CFG2_REG_CH7_FLTR_22                                        ((uint8_t) 0x16)
    #define CH7_CFG2_REG_CH7_FLTR_23                                        ((uint8_t) 0x17)
    #define CH7_CFG2_REG_CH7_FLTR_24                                        ((uint8_t) 0x18)
    #define CH7_CFG2_REG_CH7_FLTR_25                                        ((uint8_t) 0x19)
    #define CH7_CFG2_REG_CH7_FLTR_26                                        ((uint8_t) 0x1A)
    #define CH7_CFG2_REG_CH7_FLTR_27                                        ((uint8_t) 0x1B)
    #define CH7_CFG2_REG_CH7_FLTR_28                                        ((uint8_t) 0x1C)
    #define CH7_CFG2_REG_CH7_FLTR_29                                        ((uint8_t) 0x1D)
    #define CH7_CFG2_REG_CH7_FLTR_30                                        ((uint8_t) 0x1E)
    #define CH7_CFG2_REG_CH7_FLTR_31                                        ((uint8_t) 0x1F)



/* Register 0x4B (CH7_OFS_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH7_OFFSET_MSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH7_OFS_MSB_REG register */
    #define CH7_OFS_MSB_REG_ADDRESS                                         ((uint8_t) 0x4B)
    #define CH7_OFS_MSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x4C (CH7_OFS_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH7_OFFSET_MID[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH7_OFS_MID_REG register */
    #define CH7_OFS_MID_REG_ADDRESS                                         ((uint8_t) 0x4C)
    #define CH7_OFS_MID_REG_DEFAULT                                         ((uint8_t) 0x00)

    

/* Register 0x4D (CH7_OFS_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                      CH7_OFFSET_LSB[7:0]                                                      |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH7_OFS_LSB_REG register */
    #define CH7_OFS_LSB_REG_ADDRESS                                         ((uint8_t) 0x4D)
    #define CH7_OFS_LSB_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x4E (CH7_GAN_MSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH7_GAIN_MSB[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */

    /* CH7_GAN_MSB_REG register */
    #define CH7_GAN_MSB_REG_ADDRESS                                         ((uint8_t) 0x4E)
    #define CH7_GAN_MSB_REG_DEFAULT                                         ((uint8_t) 0x40)



/* Register 0x4F (CH7_GAN_MID_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH7_GAIN_MID[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */
 
    /* CH7_GAN_MID_REG register */
    #define CH7_GAN_MID_REG_ADDRESS                                         ((uint8_t) 0x4F)
    #define CH7_GAN_MID_REG_DEFAULT                                         ((uint8_t) 0x00)



/* Register 0x50 (CH7_GAN_LSB_REG) definition
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |     Bit 7     |     Bit 6     |     Bit 5     |     Bit 4     |     Bit 3     |     Bit 2     |     Bit 1     |     Bit 0     |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 * |                                                       CH7_GAIN_LSB[7:0]                                                       |
 * |-------------------------------------------------------------------------------------------------------------------------------|
 */
 
    /* CH7_GAN_LSB_REG register */
    #define CH7_GAN_LSB_REG_ADDRESS                                         ((uint8_t) 0x50)
    #define CH7_GAN_LSB_REG_DEFAULT                                         ((uint8_t) 0x00)



#endif /* ADS127L18_H_ */