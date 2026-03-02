/**
 * @file ads122y1x.h
 *
 * @brief This file contains all basic communication and device setup for the ADS122S14 device family.
 * @warning This software utilizes TI Drivers
 *
 * @copyright Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
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

#include "Driver/ads122y1x.h"
#include "Driver/hal.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "crc.h"

const char *adcRegisterNames[NUM_REGISTERS] = {"DEVICE_ID", "REVISION_ID",
     "STATUS_MSB", "STATUS_LSB", "CONVERSION_CTRL", "DEVICE_CFG",
     "DATA_RATE_CFG", "MUX_CFG", "GAIN_CFG", "REFERENCE_CFG","DIGITAL_CFG",
     "GPIO_CFG", "GPIO_DATA_OUTPUT", "IDAC_MAG_CFG", "IDAC_MUX_CFG",
     "REGISTER_MAP_CRC"};


//****************************************************************************
//
// Internal variables and functions
//
//****************************************************************************

// Array used to recall device register map configurations */
static uint8_t registerMap[NUM_REGISTERS];
static uint8_t dataTx[6] = {0};
static uint8_t dataRx[6] = {0};
static uint8_t data00[6] = {0};
static uint8_t numWords = 0;



//****************************************************************************
//
// Function Definitions
//
//****************************************************************************

static void restoreRegisterDefaults(void)
{
// use these registerMap definitions to change your configuration as needed.
   registerMap[DEVICE_ID_ADDRESS]              = 0x00;
   registerMap[REVISION_ID_ADDRESS]            = 0x00;
   registerMap[STATUS_MSB_ADDRESS]             = STATUS_MSB_DEFAULT;
   registerMap[STATUS_LSB_ADDRESS]             = STATUS_LSB_DEFAULT;
   registerMap[CONVERSION_CTRL_ADDRESS]        = CONVERSION_CTRL_DEFAULT;
   registerMap[DEVICE_CFG_ADDRESS]             = DEVICE_CFG_DEFAULT;
   registerMap[DATA_RATE_CFG_ADDRESS]          = DATA_RATE_CFG_DEFAULT;
   registerMap[MUX_CFG_ADDRESS]                = MUX_CFG_DEFAULT;
   registerMap[GAIN_CFG_ADDRESS]               = GAIN_CFG_DEFAULT;
   registerMap[REFERENCE_CFG_ADDRESS]          = REFERENCE_CFG_DEFAULT;
   registerMap[DIGITAL_CFG_ADDRESS]            = DIGITAL_CFG_DEFAULT;
   registerMap[GPIO_CFG_ADDRESS]               = GPIO_CFG_DEFAULT;
   registerMap[GPIO_DATA_OUTPUT_ADDRESS]       = GPIO_DATA_OUTPUT_DEFAULT;
   registerMap[IDAC_MAG_CFG_ADDRESS]           = IDAC_MAG_CFG_DEFAULT;
   registerMap[IDAC_MUX_CFG_ADDRESS]           = IDAC_MUX_CFG_DEFAULT;
   registerMap[REG_MAP_CRC_ADDRESS]            = REG_MAP_CRC_DEFAULT;
}

uint8_t getRegisterValue(uint8_t address)
{
    return registerMap[address];
}

void adcStartup(void)
{
    //Reset ADC. 
    resetDevice();

    //Generate CRC lookup table
    initCRC();
    
    //Read Revision and ID registers. 
    readSingleRegister(REVISION_ID_ADDRESS);
    readSingleRegister(DEVICE_ID_ADDRESS);
    
   
    //Clears any error flags triggered during power-up (e.g. POR_FLAG)
    clearSTATUSflags();
}

void clearSTATUSflags(void)
{
    writeSingleRegister(STATUS_MSB_ADDRESS, 0xFF);
    writeSingleRegister(STATUS_LSB_ADDRESS, 0xFF);
}

uint8_t readSingleRegister(uint8_t address)
{
    uint16_t regValue = 0;
    uint8_t numWords = 1;  // byte index

    if (SPI_CRC_ENABLED)  { numWords++; } // Only CRC-out is supported in C14. there is no input CRC.

//------------------------------------------------------------------------RREG------------------------------------------------------------------------
//  [SDA]  |Start|   Address[6:0]   |Write|ACK|   Read Register CMD[7:0]   |ACK|Repeat|   Address[6:0]   |Read|ACK|   Register Data[7:0]   |NACK|Stop|
//----------------------------------------------------------------------------------------------------------------------------------------------------
//  [SCL]  |-----[     8-clocks     ]---------[          8-clocks          ]----------[      8-clocks    ]--------[         8-clocks       ]---------|
//----------------------------------------------------------------------------------------------------------------------------------------------------

    dataTx[0] = (0x40 + address);
    I2C_Write(dataTx, 1);
    I2C_Read(dataRx, numWords);

    regValue =  dataRx[STATUS_ENABLED?2:0];
    registerMap[address] = regValue;
    return regValue;
}

bool writeSingleRegister(uint8_t address, uint8_t data)
{  
    dataTx[0] = 0x80 + address;
    dataTx[1] = data;

    //------------------------------------------------------WREG-------------------------------------------------------
    //  [SDA]  |Start|   Address[6:0]   |Write|ACK|   Write Register CMD[7:0]   |ACK|   Register Data[7:0]   |ACK|Stop|
    //-----------------------------------------------------------------------------------------------------------------
    //  [SCL]  |-----[     8-clocks     ]----------[          8-clocks          ]----[         8-clocks      ]--------|
    //-----------------------------------------------------------------------------------------------------------------
    
    I2C_Write (dataTx, 2);

        registerMap[address] = data;  
        return 0;
}

adc_channel_t readData(void)
{
    adc_channel_t adcData;
    

     uint8_t numWords = 2;  // byte index

    if (STATUS_ENABLED)         { numWords = numWords + 2; }                 
    if (RESOLUTION_IS_24_BIT)   { numWords++; }  
    if (SPI_CRC_ENABLED)        { numWords++; } // Only CRC-out is supported in C14. there is no input CRC.

//-----------------------------------------------------------Read ADC-------------------------------------------------------------
//  [SDA]  |Start|   Address[6:0]   |Read|ACK|   Conv Data[23:16]   |ACK|   Conv Data[15:8]   |ACK|   Conv Data[7:0]   |NACK|Stop|
//--------------------------------------------------------------------------------------------------------------------------------
//  [SCL]  |-----[     8-clocks     ]---------[       8-clocks      ]----[       8-clocks     ]---[       8-clocks     ]---------|
//--------------------------------------------------------------------------------------------------------------------------------

I2C_Write(data00, 1);
I2C_Read(dataRx, numWords);
    
    adcData.data = (signExtend(dataRx));

    if (STATUS_ENABLED) 
        {
        adcData.status_msb = dataRx[0];
        adcData.status_lsb = dataRx[1];
        }
    if (SPI_CRC_ENABLED)
        {
        adcData.crc = dataRx[numWords];
        }
   
    return (adcData);
}



int32_t signExtend(const uint8_t dataBytes[])
{
    int32_t upperByte, middleByte, lowerByte, shift;
    uint8_t i = 0;

      if (STATUS_ENABLED) { i = i + 2; }

    if (RESOLUTION_IS_16_BIT)
    {
        upperByte   = (int32_t) dataBytes[i] << 24;
        lowerByte   = (int32_t) dataBytes[i + 1] << 16;
        shift       = 16;
    }
    else
    {
        upperByte   = (int32_t) dataBytes[i] << 24;
        middleByte  = (int32_t) dataBytes[i + 1] << 16;
        lowerByte   = (int32_t) dataBytes[i + 2] << 8;
        shift       = 8;
    }

    // Right-shift of signed data maintains sign bit
    return (upperByte | middleByte | lowerByte) >> shift;
}

bool resetDevice(void)
{
    writeSingleRegister(CONVERSION_CTRL_ADDRESS, 0x58); // write value 0x58 to Control register 0x04
    delay_cycles(20000); // wait 500us for reset
    restoreRegisterDefaults();
    return 0;
}

void enableRegisterMapCrc(bool enable)
{
    // Disable the REG_MAP_CRC_EN bit
    readSingleRegister(DIGITAL_CFG_ADDRESS);    // update shadow register
    writeSingleRegister(DIGITAL_CFG_ADDRESS, registerMap[DIGITAL_CFG_ADDRESS] & ~DIGITAL_CFG_REG_MAP_CRC_EN_MASK);

    if (!enable) { return; }
     
    // pre-enable REG_MAP_CRC_MASK, this reg 08h is used in CRC calculation. 
    registerMap[DIGITAL_CFG_ADDRESS] |= DIGITAL_CFG_REG_MAP_CRC_EN_MASK;

    // Compute and update MAIN_CRC
    uint8_t crc = CRC_INITIAL_SEED; 
    crc = getCRC(&registerMap[0x05], 0x09, crc);      // registers 05h to 0Eh (skip 0h to 4h)
    writeSingleRegister(REG_MAP_CRC_ADDRESS, crc);

    // Enable the REG_MAP_CRC_EN bit
    writeSingleRegister(DIGITAL_CFG_ADDRESS, registerMap[DIGITAL_CFG_ADDRESS] | DIGITAL_CFG_REG_MAP_CRC_EN_MASK);
}

bool isValidCrcOut(void)
{
    bool crc_valid = true;
    if (SPI_CRC_ENABLED)
    {
        uint8_t crcIndex = (STATUS_ENABLED ? 2 : 0) + (RESOLUTION_IS_16_BIT ? 2 : 3);

        // (OPTION 1) Comparing bytes
        //crc_valid = (dataTx[crcIndex] == getCRC8(dataTx, crcIndex, CRC8_INITIAL_SEED));

        // (OPTION 2) Including CRC byte
        crc_valid = !(bool) getCRC(dataTx, crcIndex + 1, CRC_INITIAL_SEED);
    }
    return crc_valid;
}
