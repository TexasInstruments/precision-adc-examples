/*
 * @file ads1285_page0.h
 *
 * @brief ADS1285 Descriptor
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

#ifndef ADS1285_PAGE0_H_
#define ADS1285_PAGE0_H_

#include "Driver/hal.h"
#include "ti/driverlib/dl_gpio.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#define NUM_REGISTERS                           ((uint8_t) 14)
/** Maximum register address or address of the last register in the regmap */
#define MAX_REGISTER_ADDRESS                    ((uint8_t) 0x0D)

//****Select the appropriate CLK speed for your hardware****//
//#define MCLK_SPEED                          ((double_t) 4096000)
#define MCLK_SPEED                          ((double_t) 8192000)



//**********************************************************************************
//
// Function prototypes
//
//**********************************************************************************
void initADC(void);
void sendCommand(uint8_t CMD);
void resetDevice(void);
void writeSingleRegister(uint8_t address, uint8_t data);
uint8_t readSingleRegister(uint8_t address);
void readMultipleRegisters(uint8_t address, uint8_t numRegs);
uint32_t readDataDirect(void);
uint32_t readDataCMD(void);
void restoreRegisterDefaults(void);
uint8_t getRegisterValue(uint8_t address);
double_t calcFdataPeriod(int CONFIG0_DR);

//**********************************************************************************
//
// Device commands
//
//**********************************************************************************

#define WAKEUP_CMD        ((uint8_t) 0x01)
#define STANDBY_CMD       ((uint8_t) 0x03)
#define SYNC_CMD          ((uint8_t) 0x05)
#define RESET_CMD         ((uint8_t) 0x07)
#define RDATA_CMD         ((uint8_t) 0x12)
#define RREG_CMD          ((uint8_t) 0x20)
#define WREG_CMD          ((uint8_t) 0x40)
#define OFSCAL_CMD        ((uint8_t) 0x60)
#define GANCAL_CMD        ((uint8_t) 0x61)

//**********************************************************************************
//
// Register definitions
//
//**********************************************************************************


/* Register 0x00 (ID_SYNC) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                   REVID[3:0]                  |             DEVID[2:0]            |    SYNC   |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* ID_SYNC register */
    #define ID_SYNC_ADDRESS										((uint8_t) 0x00)
    #define ID_SYNC_DEFAULT										((uint8_t) 0x00)
    #define ID_SYNC_DEFAULT_MASK								((uint8_t) 0x0F)

    /* SYNC field */
    #define ID_SYNC_SYNC_MASK									((uint8_t) 0x01)
    #define ID_SYNC_SYNC_BITOFFSET								(0)
    #define ID_SYNC_SYNC_PULSESYNCMODE							((uint8_t) 0x00)    // DEFAULT
    #define ID_SYNC_SYNC_CONTINUOUSSYNCMODE						((uint8_t) 0x01)

    /* DEVID field */
    #define ID_SYNC_DEVID_MASK									((uint8_t) 0x0E)
    #define ID_SYNC_DEVID_BITOFFSET								(1)
    #define ID_SYNC_DEVID_ADS1285								((uint8_t) 0x00)    // DEFAULT

    /* REVID field */
    #define ID_SYNC_REVID_MASK									((uint8_t) 0xF0)
    #define ID_SYNC_REVID_BITOFFSET								(4)


/* Register 0x01 (CONFIG0) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |       MODE[1:0]       |              DR[2:0]              |   PHASE   |      FILTER[1:0]      |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* CONFIG0 register */
    #define CONFIG0_ADDRESS										((uint8_t) 0x01)
    #define CONFIG0_DEFAULT										((uint8_t) 0x12)

    /* FILTER field */
    #define CONFIG0_FILTER_MASK									((uint8_t) 0x03)
    #define CONFIG0_FILTER_BITOFFSET							(0)
    #define CONFIG0_FILTER_RESERVED								((uint8_t) 0x00)
    #define CONFIG0_FILTER_SINCFILTEROUTPUT						((uint8_t) 0x01)
    #define CONFIG0_FILTER_FIRFILTEROUTPUT						((uint8_t) 0x02)    // DEFAULT
    #define CONFIG0_FILTER_FIRIIRFILTEROUTPUT					((uint8_t) 0x03)

    /* PHASE field */
    #define CONFIG0_PHASE_MASK									((uint8_t) 0x04)
    #define CONFIG0_PHASE_BITOFFSET								(2)
    #define CONFIG0_PHASE_LINEARPHASE							((uint8_t) 0x00)    // DEFAULT
    #define CONFIG0_PHASE_MINIMUMPHASE							((uint8_t) 0x04)

    /* DR field - High_Mid Mode*/  
    #define CONFIG0_DR_MASK										((uint8_t) 0x38)
    #define CONFIG0_DR_BITOFFSET								(3)
    #define CONFIG0_DR_250SPS_HIGH_MID							((uint8_t) 0x00)
    #define CONFIG0_DR_500SPS_HIGH_MID							((uint8_t) 0x08)
    #define CONFIG0_DR_1000SPS_HIGH_MID							((uint8_t) 0x10)    // DEFAULT
    #define CONFIG0_DR_2000SPS_HIGH_MID 						((uint8_t) 0x18)
    #define CONFIG0_DR_4000SPS_HIGH_MID							((uint8_t) 0x20)

    /* DR field - Low Mode*/
    #define CONFIG0_DR_125SPS_LOW								((uint8_t) 0x00)
    #define CONFIG0_DR_250SPS_LOW								((uint8_t) 0x08)
    #define CONFIG0_DR_500SPS_LOW								((uint8_t) 0x10)    // DEFAULT
    #define CONFIG0_DR_1000SPS_LOW								((uint8_t) 0x18)
    #define CONFIG0_DR_2000SPS_LOW								((uint8_t) 0x20)

    /* MODE field */
    #define CONFIG0_MODE_MASK									((uint8_t) 0xC0)
    #define CONFIG0_MODE_BITOFFSET								(6)
    #define CONFIG0_MODE_HIGH									((uint8_t) 0x00)    // DEFAULT
    #define CONFIG0_MODE_MID									((uint8_t) 0x40)
    #define CONFIG0_MODE_LOW									((uint8_t) 0x80)
    #define CONFIG0_MODE_RESERVED								((uint8_t) 0xC0)


/* Register 0x02 (CONFIG1) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |              MUX[2:0]             |        REF[1:0]       |             GAIN[2:0]             |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* CONFIG1 register */
    #define CONFIG1_ADDRESS										((uint8_t) 0x02)
    #define CONFIG1_DEFAULT										((uint8_t) 0x00)

    /* GAIN field */
    #define CONFIG1_GAIN_MASK									((uint8_t) 0x07)
    #define CONFIG1_GAIN_BITOFFSET								(0)
    #define CONFIG1_GAIN_1										((uint8_t) 0x00)    // DEFAULT
    #define CONFIG1_GAIN_2										((uint8_t) 0x01)
    #define CONFIG1_GAIN_4										((uint8_t) 0x02)
    #define CONFIG1_GAIN_8										((uint8_t) 0x03)
    #define CONFIG1_GAIN_16										((uint8_t) 0x04)
    #define CONFIG1_GAIN_32										((uint8_t) 0x05)
    #define CONFIG1_GAIN_64										((uint8_t) 0x06)
    #define CONFIG1_GAIN_BUFFEROPERATION						((uint8_t) 0x07)

    /* REF field */
    #define CONFIG1_REF_MASK									((uint8_t) 0x18)
    #define CONFIG1_REF_BITOFFSET								(3)
    #define CONFIG1_REF_5V										((uint8_t) 0x00)    // DEFAULT
    #define CONFIG1_REF_4										((uint8_t) 0x08)
    #define CONFIG1_REF_2										((uint8_t) 0x10)
    #define CONFIG1_REF_RESERVED								((uint8_t) 0x18)

    /* MUX field */
    #define CONFIG1_MUX_MASK									((uint8_t) 0xE0)
    #define CONFIG1_MUX_BITOFFSET								(5)
    #define CONFIG1_MUX_INPUT1									((uint8_t) 0x00)    // DEFAULT
    #define CONFIG1_MUX_INPUT2									((uint8_t) 0x20)
    #define CONFIG1_MUX_INTERNALSHORTWITHA400ORESISTOR			((uint8_t) 0x40)
    #define CONFIG1_MUX_INPUT1ANDINPUT2							((uint8_t) 0x60)
    #define CONFIG1_MUX_RESERVED								((uint8_t) 0x80)
    #define CONFIG1_MUX_INTERNALSHORTWITHA0ORESISTOR			((uint8_t) 0xA0)

/* Register 0x03 (HPF0) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                            LSB[7:0]                                           |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* HPF0 register */
    #define HPF0_ADDRESS										((uint8_t) 0x03)
    #define HPF0_DEFAULT										((uint8_t) 0x32)

    /* LSB field */
    #define HPF0_LSB_MASK										((uint8_t) 0xFF)
    #define HPF0_LSB_BITOFFSET									(0)


/* Register 0x04 (HPF1) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                            MSB[7:0]                                           |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* HPF1 register */
    #define HPF1_ADDRESS										((uint8_t) 0x04)
    #define HPF1_DEFAULT										((uint8_t) 0x03)

    /* MSB field */
    #define HPF1_MSB_MASK										((uint8_t) 0xFF)
    #define HPF1_MSB_BITOFFSET									(0)


/* Register 0x05 (OFFSET0) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                            LSB[7:0]                                           |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* OFFSET0 register */
    #define OFFSET0_ADDRESS										((uint8_t) 0x05)
    #define OFFSET0_DEFAULT										((uint8_t) 0x00)

    /* LSB field */
    #define OFFSET0_LSB_MASK									((uint8_t) 0xFF)
    #define OFFSET0_LSB_BITOFFSET								(0)


/* Register 0x06 (OFFSET1) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                            MID[7:0]                                           |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* OFFSET1 register */
    #define OFFSET1_ADDRESS										((uint8_t) 0x06)
    #define OFFSET1_DEFAULT										((uint8_t) 0x00)

    /* MID field */
    #define OFFSET1_MID_MASK									((uint8_t) 0xFF)
    #define OFFSET1_MID_BITOFFSET								(0)


/* Register 0x07 (OFFSET2) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                            MSB[7:0]                                           |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* OFFSET2 register */
    #define OFFSET2_ADDRESS										((uint8_t) 0x07)
    #define OFFSET2_DEFAULT										((uint8_t) 0x00)

    /* MSB field */
    #define OFFSET2_MSB_MASK									((uint8_t) 0xFF)
    #define OFFSET2_MSB_BITOFFSET								(0)


/* Register 0x08 (GAIN0) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                            LSB[7:0]                                           |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* GAIN0 register */
    #define GAIN0_ADDRESS										((uint8_t) 0x08)
    #define GAIN0_DEFAULT										((uint8_t) 0x00)

    /* LSB field */
    #define GAIN0_LSB_MASK										((uint8_t) 0xFF)
    #define GAIN0_LSB_BITOFFSET									(0)


/* Register 0x09 (GAIN1) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                            MID[7:0]                                           |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* GAIN1 register */
    #define GAIN1_ADDRESS										((uint8_t) 0x09)
    #define GAIN1_DEFAULT										((uint8_t) 0x00)

    /* MID field */
    #define GAIN1_MID_MASK										((uint8_t) 0xFF)
    #define GAIN1_MID_BITOFFSET									(0)


/* Register 0x0A (GAIN2) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                            MSB[7:0]                                           |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* GAIN2 register */
    #define GAIN2_ADDRESS										((uint8_t) 0x0A)
    #define GAIN2_DEFAULT										((uint8_t) 0x40)

    /* MSB field */
    #define GAIN2_MSB_MASK										((uint8_t) 0xFF)
    #define GAIN2_MSB_BITOFFSET									(0)


/* Register 0x0B (GPIO) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |           RESERVED[2:0]           | GPIO1_DAT | GPIO0_DAT | GPIO1_DIR | GPIO0_DIR |  RESERVED |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* GPIO register */
    #define GPIO_ADDRESS										((uint8_t) 0x0B)
    #define GPIO_DEFAULT										((uint8_t) 0x00)
    #define GPIO_DEFAULT_MASK									((uint8_t) 0xE7)

    /* GPIO0_DIR field */
    #define GPIO_GPIO0_DIR_MASK									((uint8_t) 0x02)
    #define GPIO_GPIO0_DIR_BITOFFSET							(1)
    #define GPIO_GPIO0_DIR_GPIO0ISANINPUT						((uint8_t) 0x00)    // DEFAULT
    #define GPIO_GPIO0_DIR_GPIO0ISANOUTPUT						((uint8_t) 0x02)

    /* GPIO1_DIR field */
    #define GPIO_GPIO1_DIR_MASK									((uint8_t) 0x04)
    #define GPIO_GPIO1_DIR_BITOFFSET							(2)
    #define GPIO_GPIO1_DIR_GPIO1ISANINPUT						((uint8_t) 0x00)    // DEFAULT
    #define GPIO_GPIO1_DIR_GPIO1ISANOUTPUT						((uint8_t) 0x04)

    /* GPIO0_DAT field */
    #define GPIO_GPIO0_DAT_MASK									((uint8_t) 0x08)
    #define GPIO_GPIO0_DAT_BITOFFSET							(3)
    #define GPIO_GPIO0_DAT_GPIO0ISLOW							((uint8_t) 0x00)    // DEFAULT
    #define GPIO_GPIO0_DAT_GPIO0ISHIGH							((uint8_t) 0x08)

    /* GPIO1_DAT field */
    #define GPIO_GPIO1_DAT_MASK									((uint8_t) 0x10)
    #define GPIO_GPIO1_DAT_BITOFFSET							(4)
    #define GPIO_GPIO1_DAT_GPIO1ISLOW							((uint8_t) 0x00)    // DEFAULT
    #define GPIO_GPIO1_DAT_GPIO1ISHIGH							((uint8_t) 0x10)


/* Register 0x0C (SRC0) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                            LSB[7:0]                                           |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* SRC0 register */
    #define SRC0_ADDRESS										((uint8_t) 0x0C)
    #define SRC0_DEFAULT										((uint8_t) 0x00)

    /* LSB field */
    #define SRC0_LSB_MASK										((uint8_t) 0xFF)
    #define SRC0_LSB_BITOFFSET									(0)


/* Register 0x0D (SRC1) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                                            MSB[7:0]                                           |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* SRC1 register */
    #define SRC1_ADDRESS										((uint8_t) 0x0D)
    #define SRC1_DEFAULT										((uint8_t) 0x80)

    /* MSB field */
    #define SRC1_MSB_MASK										((uint8_t) 0xFF)
    #define SRC1_MSB_BITOFFSET									(0)



//**********************************************************************************
//
// Register macros
//
//**********************************************************************************
#define MASKED_REG_DATA(page, addr, mask)     (getRegisterValue(page, addr) & (mask))




#endif /* ADS1285_PAGE0_H_ */
