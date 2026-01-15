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

#ifndef ADS125P08_H_
#define ADS125P08_H_

#include <stdint.h>
#include <stdbool.h>

#include "Driver/ads125p08_pages.h"
#include "Driver/ads125p08_page0.h"
#include "Driver/ads125p08_page1.h"


#define NUM_PAGES                               ((uint8_t) 33)

#define NUM_GENERAL_REGISTERS                           ((uint8_t) 64)
/** Maximum register address or address of the last register in the regmap */
#define MAX_GENERAL_REGISTER_ADDRESS                    ((uint8_t) 0x3F)


#define NUM_STEP_REGISTERS                           ((uint8_t) 20)
/** Maximum register address or address of the last register in the regmap */
#define MAX_STEP_REGISTER_ADDRESS                    ((uint8_t) 0x10)



//**********************************************************************************
//
// Typedefs
//
//**********************************************************************************

 typedef struct {
    uint8_t STATUS_MSB;
    uint8_t STATUS_LSB;
    uint8_t ADC_REF;
    uint8_t DIGITAL_STATUS;
    uint8_t FIFO;
    bool errorPresent;
 } STATUS_IO;

 typedef struct {
    uint32_t    ADC_reading;
    uint32_t    FIFO_reading;
    uint8_t     Register_data;
    uint8_t    STATUS_MSB;
    uint8_t    STATUS_LSB;
    bool        CRC_Valid;
 } ADC_IO;

 typedef struct {
   uint8_t STATUS_MSB;
   uint8_t STATUS_LSB;
   uint8_t Register_data;
   bool    CRC_Valid;
 } REG_IO;

//**********************************************************************************
//
// Function prototypes
//
//**********************************************************************************

uint8_t getRegisterValue(uint8_t page, uint8_t address);
void initADC(void);
void powerDownMode(void);
void activeMode(void);
bool resetDevice(void);
void startAdcConversion(void);
void stopAdcConversion(void);
REG_IO readSingleRegister(uint8_t page, uint8_t address);
REG_IO writeSingleRegister(uint8_t page, uint8_t address, uint8_t data);
STATUS_IO checkStatus(void);
ADC_IO readData(void);
ADC_IO readFIFO(void);
int32_t signExtend(const uint8_t dataBytes[]);
void enableRegisterMapCrc(bool enable);
bool isValidCrcOut(void);
 
//**********************************************************************************
//
// Variables
//
//**********************************************************************************

 STATUS_IO status;
 ADC_IO regValue;
 extern bool g_Status;
 extern bool g_CRC;
 
//**********************************************************************************
//
// Register macros
//
//**********************************************************************************
#define MASKED_REG_DATA(page, addr, mask)     (getRegisterValue(page, addr) & (mask))

/** Returns true if STATUS bit is set in shadow register */
#define STATUS_ENABLED          ((bool) MASKED_REG_DATA(GENERAL_SETTINGS, DIAG_MONITOR_CFG_ADDRESS, STATUS_EN_MASK))

/** Returns true if SPI_CRC bit is set in shadow register */
#define SPI_CRC_ENABLED         ((bool) MASKED_REG_DATA(GENERAL_SETTINGS, DIAG_MONITOR_CFG_ADDRESS, SPI_CRC_EN_MASK))

#define RESOLUTION_IS_16_BIT    ((bool) MASKED_REG_DATA(GENERAL_SETTINGS, DEVICE_ID_ADDRESS, DEVICE_IS_16_BIT_MASK))

#define RESOLUTION_IS_24_BIT   ((bool) !MASKED_REG_DATA(GENERAL_SETTINGS, DEVICE_ID_ADDRESS, DEVICE_IS_16_BIT_MASK))


#endif /* ADS125P08_H_ */
