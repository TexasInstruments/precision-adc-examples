/**
 * @file ads1115.c
 *
 * @brief This file contains all basic communication and device setup for the ADS1115 device family.
 * @warning This software utilizes TI Drivers
 *
 * @copyright Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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

#include <evm/ADS1115.h>
#include <ti/devices/msp432e4/driverlib/sysctl.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CMSP432E4.h>
#include "settings.h"

const char *adcRegisterNames[NUM_REGISTERS] = {"DATA", "CONFIG", "LO_THRESH", "HI_THRESH"};

//****************************************************************************
//
// Internal variables
//
//****************************************************************************

// Array used to recall device register map configurations */
static uint16_t registerMap[NUM_REGISTERS];

//****************************************************************************
//
// Function Definitions
//
//****************************************************************************
/**
 *
 * @brief restoreRegisterDefaults()
 * Updates the registerMap[] array to its default values.
 *
 * NOTES:
 * - If the MCU keeps a copy of the ADS1x15 register settings in memory,
 * then it is important to ensure that these values remain in sync with the
 * actual hardware settings. In order to help facilitate this, this function
 * should be called after powering up or resetting the device.
 *
 * - Reading back all of the registers after resetting the device can
 * accomplish the same result.
 *
 * @return none
 */
static void restoreRegisterDefaults(void)
{
    registerMap[CONFIG_ADDRESS]       = CONFIG_DEFAULT;
	/* The threshold register do not exist on the TLA2024, ADS1013 or ADS1113 */
    registerMap[LO_THRESH_ADDRESS]    = LO_THRESH_DEFAULT;
    registerMap[HI_THRESH_ADDRESS]    = HI_THRESH_DEFAULT;
}

/**
 *
 * @brief adcStartup()
 * Example start up sequence for the ADS1115.
 *
 * Before calling this function, the device must be powered and
 * the I2C/GPIO pins of the MCU must have already been configured.
 *
 * @return none
 */
void adcStartup(void)
{
    //
    // (OPTIONAL) Provide additional delay time for power supply settling
    //
    delay_ms(50);

    //
    // (REQUIRED) Initialize internal 'registerMap' array with device default settings
    //
    restoreRegisterDefaults();

    //
    // (OPTIONAL) Read back all registers and Check STATUS register (if exists) for faults
    //
    registerMap[CONFIG_ADDRESS] = readSingleRegister(CONFIG_ADDRESS);
}

/**
 *
 * @brief readSingleRegister()
 * Reads the contents of a single register at the specified address.
 *
 * @param[in] address Is the 8-bit address of the register to read.
 *
 * @return The 16-bit register read result as an unsigned value, but conversion
 * is binary 2's complement.
 */
uint16_t readSingleRegister(uint8_t address)
{
    //
    // Check that the register address is in range
    //
    assert(address <= MAX_REGISTER_ADDRESS);
    uint16_t regValue = 0;
    uint8_t regRXdata[4] = {0};
    int8_t retStatus;
    retStatus = receiveI2CData(address, regRXdata, 2);
    if(retStatus != false)
    {
        /* I2C communication error took place...Insert a handling routine here */
        return 0;
    }
    regValue =  combineBytes(regRXdata[0], regRXdata[1]);
    registerMap[address] = regValue;
    return regValue;
}
/**
 *
 * @brief readConversionRegister()
 * Reads the contents of the conversion register.
 *
 * @return the 16-bit register conversion result as an signed 32-bit value
 */
int32_t readConversionRegister(void)
{
    int32_t regValue = 0;
    uint8_t regRXdata[4] = {0};
    int8_t retStatus;
    retStatus = receiveI2CDataNoWrite(CONVERSION_ADDRESS, regRXdata, 2, false);
    if(retStatus != false)
    {
        /* I2C communication error took place...Insert a handling routine here */
        return 0;
    }
    regValue = signExtend(regRXdata);
    return regValue;
}

/**
 *
 * @brief writeSingleRegister()
 * Writes data to a single register and also read it back for internal
 * confirmation.
 *
 * @param[in] address Is the address of the register to write to.
 * @param[in] data Is the value to write.
 *
 * @return when true, data transmitted does not match data read
 */
bool writeSingleRegister(uint8_t address, uint16_t data)
{

    //
    // Check that the register address is in range
    //
    assert(address <= MAX_REGISTER_ADDRESS);
    uint8_t regData[2];

    regData[0] = data>>8;
    regData[1] = data & 0xFF;
    int8_t retStatus;
    uint16_t regValue;
    //
    // Cannot write to the conversion register
    //
    if(address == 0)
    {
        /* Insert a handling routine here */
        return 0;
    }
    retStatus = sendI2CData(address, regData, 2);
    if(retStatus != false)
    {
        /* I2C communication error took place...Insert a handling routine here */
        return 0;
    }

    retStatus = receiveI2CData(address, regData, 2);
    if(retStatus != false)
    {
        /* I2C communication error took place...Insert a handling routine here */
        return 0;
    }

    regValue = combineBytes(regData[0], regData[1]);
    if (address == CONFIG_ADDRESS)
    {
        if ((regValue & 0x7FFF) != (data & 0x7FFF)) return 1;
            registerMap[CONFIG_ADDRESS] = regValue & 0x7FFF;
    }
    else
    {
        if (regValue != data) return 1;
        registerMap[address] = regValue;
    }
    return 0;

}
/**
 *
 * @brief writeSingleRegisterNoRead()
 * Writes data to a single register with no readback
 * confirmation.
 *
 * @param[in] address Is the address of the register to write to.
 * @param[in] data Is the value to write.
 *
 * @return true on success, otherwise false
 */
bool writeSingleRegisterNoRead(uint8_t address, uint16_t data)
{

    //
    // Check that the register address is in range
    //
    assert(address <= MAX_REGISTER_ADDRESS);
    uint8_t regData[2];

    regData[0] = data>>8;
    regData[1] = data & 0xFF;
    int8_t retStatus;
    //
    // Cannot write to the conversion register
    //
    if(address == 0)
    {
        /* Insert a handling routine here */
        return 0;
    }
    retStatus = sendI2CRegPointer(address, regData, 2);
    if(retStatus != false)
    {
        /* I2C communication error took place...Insert a handling routine here */
        return 0;
    }

    registerMap[CONFIG_ADDRESS] = data & 0x7FFF;
    return 1;
}
/**
 *
 * @brief readData()
 * Reads 2 bytes of ADC data.
 *
 * @return signed 16-bit integer
 */
int16_t readData(void)
{
    uint16_t regValue;
    // Get the contents of the config register, we want to keep the configuration just start the conversion
    regValue = 1<<15 | registerMap[CONFIG_ADDRESS];
    uint8_t regData[2];
    int8_t retStatus;
    regData[0] = regValue>>8;
    regData[1] = regValue & 0xFF;
    //
    // Write to the config register and start a conversion
    //
    retStatus = sendI2CData(CONFIG_ADDRESS, regData, 2);
    if(retStatus != false)
    {
        /* I2C communication error took place...Insert a handling routine here */
        return 0;
    }
    //
    // To reduce the polling of communication delay a minimum time before checking
    //  the end of conversion based on the selected data rate
    //
    delayBeforeRead();
    //
    // If operating in single-shot mode check for EOC
    //
    if(regValue & CONFIG_MODE_MASK)
    {
        //
        // Poll for the end of conversion flag in the config register
        //
        regData[0] = 0;
        do
        {
            retStatus = receiveI2CData(CONFIG_ADDRESS, regData, 2);
            if(retStatus != false)
            {
                /* I2C communication error took place...Insert a handling routine here */
                return 0;
            }
        } while (!(regData[0] & 0x80) );

    }

    // Read conversion results
    registerMap[CONVERSION_ADDRESS] = readSingleRegister(CONVERSION_ADDRESS);
    return (int16_t) registerMap[CONVERSION_ADDRESS];
}
/**
 *
 * @brief startAdcConversion()
 * Starts the ADC conversion.
 *
 * @return  uint16_t configuration register
 *
 */
uint16_t startAdcConversion(void)
{
    int8_t retStatus;
    // Get the contents of the config register, we want to keep the configuration just start the conversion
    uint16_t regValue = registerMap[CONFIG_ADDRESS];
    regValue = CONFIG_OS_MASK | regValue;
    retStatus = writeSingleRegisterNoRead(CONFIG_ADDRESS, regValue);
    if(retStatus == false)
    {
        // Add error handling routine
        return 0;
    }
    return regValue;
}
/**
 *
 * @brief stopAdcConversion()
 * Stops the ADC conversion for device in continuous conversion mode.
 *
 * @return  uint16_t configuration register
 *
 */
uint16_t stopAdcConversion(void)
{
    int8_t retStatus;
    // Get the contents of the config register, we want to keep the configuration except for MODE
    uint16_t regValue = registerMap[CONFIG_ADDRESS];
    regValue |= CONFIG_MODE_SS;
    retStatus = writeSingleRegisterNoRead(CONFIG_ADDRESS, regValue);
    if(retStatus == false)
    {
        // Add error handling routine
        return 0;
    }
    registerMap[CONFIG_ADDRESS] = regValue;
    return regValue;
}
/**
 *
 * @brief delayBeforeRead()
 * Delays a specific amount of time before checking for EOC based on data rate.
 *
 * @return  none
 *
 */
void delayBeforeRead(void)
{
    uint16_t regValue = 0;
    // Get the contents of the config register, we want to keep the configuration just start the conversion
    regValue = 1<<15 | registerMap[CONFIG_ADDRESS];
#ifdef ADS1115
    if(regValue & CONFIG_MODE_MASK)
    {   //Conversion is single-shot mode
        switch((regValue & CONFIG_DR_MASK))
        {
            case CONFIG_DR_8SPS:
                delay_us(121399);
                break;
            case CONFIG_DR_16SPS:
                delay_us(59760);
                break;
            case CONFIG_DR_32SPS:
                delay_us(29585);
                break;
            case CONFIG_DR_64SPS:
                delay_us(14610);
                break;
            case CONFIG_DR_128SPS:
                delay_us(7173);
                break;
            case CONFIG_DR_250SPS:
                delay_us(3448);
                break;
            case CONFIG_DR_475SPS:
                delay_us(1659);
                break;
            case CONFIG_DR_860SPS:
                delay_us(758);
                break;
        }
    }
    else
    {
        // Conversion is in continuous conversion mode
        switch((regValue & CONFIG_DR_MASK))
        {
            case CONFIG_DR_8SPS:
                delay_us(120043);
                break;
            case CONFIG_DR_16SPS:
                delay_us(59115);
                break;
            case CONFIG_DR_32SPS:
                delay_us(29485);
                break;
            case CONFIG_DR_64SPS:
                delay_us(14655);
                break;
            case CONFIG_DR_128SPS:
                delay_us(7249);
                break;
            case CONFIG_DR_250SPS:
                delay_us(3631);
                break;
            case CONFIG_DR_475SPS:
                delay_us(1838);
                break;
            case CONFIG_DR_860SPS:
                delay_us(943);
                break;
        }
    }
#else
    if(regValue & CONFIG_MODE_MASK)
    {   //Conversion is single-shot mode
    switch((regValue & CONFIG_DR_MASK))
        {
            case CONFIG_DR_128SPS:
                delay_us(7448);
                break;
            case CONFIG_DR_250SPS:
                delay_us(3635);
                break;
            case CONFIG_DR_490SPS:
                delay_us(1676);
                break;
            case CONFIG_DR_920SPS:
                delay_us(722);
                break;
            case CONFIG_DR_1600SPS:
                delay_us(260);
                break;
            case CONFIG_DR_2400SPS:
                delay_us(52);
                break;
            case CONFIG_DR_3300_1SPS:
            case CONFIG_DR_3300_2SPS:
                break;
        }
    }
    else
    {   // Conversion is in continuous conversion mode
        switch((regValue & CONFIG_DR_MASK))
        {
            case CONFIG_DR_128SPS:
                delay_us(7249);
                break;
            case CONFIG_DR_250SPS:
                delay_us(3627);
                break;
            case CONFIG_DR_490SPS:
                delay_us(1778);
                break;
            case CONFIG_DR_920SPS:
                delay_us(882);
                break;
            case CONFIG_DR_1600SPS:
                delay_us(441);
                break;
            case CONFIG_DR_2400SPS:
                delay_us(241);
                break;
            case CONFIG_DR_3300_1SPS:
            case CONFIG_DR_3300_2SPS:
                delay_us(129);
                break;
        }
    }
#endif
}
/**
 *
 * @brief resetDevice()
 * Resets the device and reinitializes the register map array
 * maintained in firmware to default values.
 *
 * @return  Returns boolean; communication sent is true, otherwise false.
 *
 */
bool resetDevice(void)
{
    uint8_t regData[2];
    regData[0] = 0x06;
    int8_t retStatus;
    //
    // Send a general call reset
    //
    retStatus = sendI2CData(0x00,regData,1);
    if(retStatus!=0) return retStatus;

    //
    // Read the registers to verify reset condition and store
    //
    restoreRegisterDefaults();
    return retStatus;
}

/**
 *
 * @brief getRegisterValue()
 * Getter function to access registerMap array from outside of this module.
 *
 * @param[in] address Of desired register contents to return
 *
 * NOTE: The internal registerMap arrays stores the last known register value,
 * since the last read or write operation to that register. This function
 * does not communicate with the device to retrieve the current register value.
 * For the most up-to-date register data or retrieving the value of a hardware
 * controlled register, it is recommend to use readSingleRegister() to read the
 * current register value.
 *
 * @return unsigned 16-bit register value.
 */
uint16_t getRegisterValue(uint8_t address)
{
    assert(address <= MAX_REGISTER_ADDRESS);
    return registerMap[address];
}

//****************************************************************************
//
// Helper functions
//
//****************************************************************************

/**
 *
 * @brief upperByte()
 * Takes a 16-bit word and returns the most-significant byte.
 *
 * @param[in] uint16_Word Is the original 16-bit word.
 *
 * @return 8-bit most-significant byte.
 */
uint8_t upperByte(uint16_t uint16_Word)
{
    uint8_t msByte;
    msByte = (uint8_t) ((uint16_Word >> 8) & 0x00FF);

    return msByte;
}

/**
 *
 * @brief lowerByte()
 * Takes a 16-bit word and returns the least-significant byte.
 *
 * @param[in] uint16_Word Is the original 16-bit word.
 *
 * @return 8-bit least-significant byte.
 */
uint8_t lowerByte(uint16_t uint16_Word)
{
    uint8_t lsByte;
    lsByte = (uint8_t) (uint16_Word & 0x00FF);

    return lsByte;
}

/**
 *
 * @brief combineBytes()
 * Takes two 8-bit words and returns a concatenated 16-bit word.
 *
 * @param[in] upperByte Is the 8-bit value that will become the MSB of the 16-bit word.
 * @param[in] lowerByte Is the 8-bit value that will become the LSB of the 16-bit word.
 *
 * @return concatenated unsigned 16-bit word.
 */
uint16_t combineBytes(uint8_t upperByte, uint8_t lowerByte)
{
    uint16_t combinedValue;
    combinedValue = ((uint16_t) upperByte << 8) | ((uint16_t) lowerByte);

    return combinedValue;
}

/**
 *
 * @brief combineDataBytes()
 * Combines ADC data bytes into a single signed 32-bit word.
 *
 * @param[in] dataBytes Is a pointer to uint8_t[] where the first element is the MSB.
 *
 * @return Returns the signed-extend 32-bit result.
 */
int32_t signExtend(const uint8_t dataBytes[])
{
    int32_t upperByte   = ((int32_t) dataBytes[0] << 24);
    int32_t lowerByte   = ((int32_t) dataBytes[1] << 16);

    return (((int32_t) (upperByte | lowerByte)) >> 16);                 // Right-shift of signed data maintains signed bit

}
