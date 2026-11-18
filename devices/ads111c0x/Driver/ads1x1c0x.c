/**
 * @file ads1x1c0x.c
 * @brief Communication and device setup for the ADS111C02/C04 device family.
 * @copyright Copyright (C) 2026 Texas Instruments Incorporated - http://www.ti.com/
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
 */

#include"/Driver/Inc/ads_device.h"

#if   defined(ADS_DEVICE_C02)
const char *registerNames[NUM_REGISTERS] = {"DATA", "CONFIG", "LO_THRESH", "HI_THRESH", "ID"};

#elif defined(ADS_DEVICE_C04)
const char *registerNames[NUM_REGISTERS] = {"DATA", "CONFIG", "ID"};

#endif

uint16_t registerMap[NUM_REGISTERS];

static void restoreRegisterDefaults(void)
{
    registerMap[DATA_ADDRESS]       = DATA_DEFAULT;
    registerMap[CONFIG_ADDRESS]     = CONFIG_DEFAULT;
   
    #if   defined(ADS_DEVICE_C02)
        registerMap[LO_THRESH_ADDRESS] = LO_THRESH_DEFAULT;
        registerMap[HI_THRESH_ADDRESS] = HI_THRESH_DEFAULT;
    #endif

}

void adcStartup(void)
{
    delay_ms(50);
    restoreRegisterDefaults();
    readConfigRegister();
}

uint16_t readSingleRegister(uint8_t reg)
{
    uint8_t regTXdata[1];
    regTXdata[0] = ((uint8_t) reg);
    uint8_t regRXdata[2] = {0};

    I2C_Write(regTXdata, 1);    // set the pointer to the value 'reg'
    I2C_Read(regRXdata, 2);     // read the value at 'reg'

    uint16_t regValue = combineBytes(regRXdata[0], regRXdata[1]);
    registerMap[reg] = regValue;
    return regValue;
}

uint16_t readConfigRegister(void)
{
    uint8_t regTXdata[1];
    regTXdata[0] = ((uint8_t) CONFIG_ADDRESS);
    uint8_t regRXdata[2] = {0};

    I2C_Write(regTXdata, 1);    // set the pointer to the value 'reg'
    I2C_Read(regRXdata, 2);     // read the value at 'reg'

    uint16_t regValue = combineBytes(regRXdata[0], regRXdata[1]);
    registerMap[CONFIG_ADDRESS] = regValue;
    return regValue;
}

uint16_t readConversionRegister( void )
{
    uint8_t regTXdata[1];
    regTXdata[0] = ((uint8_t) DATA_ADDRESS);
    uint8_t regRXdata[2] = {0};

    I2C_Write(regTXdata, 1);    // set the pointer to the value 'reg'
    I2C_Read(regRXdata, 2);     // read the value at 'reg'

    uint16_t regValue = combineBytes(regRXdata[0], regRXdata[1]);
    registerMap[DATA_ADDRESS] = regValue;
    return regValue;
}


bool writeSingleRegister(uint8_t reg, uint16_t data)
{
    assert(reg <= MAX_REGISTER_ADDRESS);
    uint8_t regData[3] = {0};
    regData[0] = reg;
    regData[1] = data >> 8;
    regData[2] = data & 0xFF;

    I2C_Write(regData, 3);     // write pointer and data
    
    registerMap[reg] = data;

     return (true); 
        
}

int16_t readData(void)
{
    waitForDRDY(1000);
    return (readConversionRegister());
}

uint16_t startAdcConversion(void)
{
    uint16_t regValue = CONFIG_OS_MASK | registerMap[CONFIG_ADDRESS];
    writeSingleRegister(CONFIG_ADDRESS, regValue);
    return regValue;
}

uint16_t stopAdcConversion(void)
{
    uint16_t regValue = ((registerMap[CONFIG_ADDRESS] & ~CONFIG_OS_MASK) | CONFIG_MODE_SINGLE_SHOT_CONVERSION_MODE);
    writeSingleRegister(CONFIG_ADDRESS, regValue);
    registerMap[CONFIG_ADDRESS] = regValue;
    return regValue;
}

uint16_t getRegisterValue(uint8_t reg)
{
    assert(reg <= MAX_REGISTER_ADDRESS);
    return registerMap[reg];
}

float measureInternalTemperature(void)
{
    float temp_C;
    int16_t cValue;

    registerMap[CONFIG_ADDRESS] = registerMap[CONFIG_ADDRESS] | CONFIG_TS_TEMPERATURE_SENSOR_SELECTED_FOR_MEASUREMENT;
    writeSingleRegister(CONFIG_ADDRESS, registerMap[CONFIG_ADDRESS]);
    startAdcConversion();
   
    cValue = readData();

     if(RESOLUTION_IS_16_BIT)   { 
        cValue  = cValue >> 2;
        temp_C  = cValue * 0.03125f;
    }

    if(!RESOLUTION_IS_16_BIT)    { 
        cValue  = cValue >> 4;
        temp_C  = cValue * 0.125f;
    }   

   

    writeSingleRegister(CONFIG_ADDRESS, registerMap[CONFIG_ADDRESS] & ~CONFIG_TS_TEMPERATURE_SENSOR_SELECTED_FOR_MEASUREMENT);
    return temp_C;
}

uint8_t upperByte(uint16_t uint16_Word)
{
    return (uint8_t)((uint16_Word >> 8) & 0x00FF);
}

uint8_t lowerByte(uint16_t uint16_Word)
{
    return (uint8_t)(uint16_Word & 0x00FF);
}

uint16_t combineBytes(uint8_t upperByte, uint8_t lowerByte)
{
    return ((uint16_t)upperByte << 8) | (uint16_t)lowerByte;
}

int32_t signExtend(const uint8_t dataBytes[])
{
    if (RESOLUTION_IS_16_BIT)   {
        int32_t upper = ((int32_t)dataBytes[0] << 24);
        int32_t lower = ((int32_t)dataBytes[1] << 16);
        return (((int32_t)(upper | lower)) >> 16);     
        }
        else 
        {
        int32_t upper = ((int32_t)dataBytes[0] << 8);
        int32_t lower = ((int32_t)dataBytes[1] << 0);
        return (((int32_t)(upper | lower)));
        }
}

