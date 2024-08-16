/**
 * @file ads1118.c
 *
 * @brief This file contains all basic communication and device setup for the ADS1118 device family.
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

#include "ads1118.h"
#include <ti/devices/msp432e4/driverlib/sysctl.h>

const char *adcRegisterNames[NUM_REGISTERS] = {"DATA", "CONFIG"};
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
 * - If the MCU keeps a copy of the ADS1118 register settings in memory,
 * then it is important to ensure that these values remain in sync with the
 * actual hardware settings. In order to help facilitate this, this function
 * should be called after powering up or resetting the device.
 *
 * - Reading back all of the registers after resetting the device can
 * accomplish the same result.
 *
 * @return none
 */
void restoreRegisterDefaults(void)
{
    registerMap[CONVERSION_ADDRESS] = CONVERSION_DEFAULT;
    registerMap[CONFIG_ADDRESS]     = CONFIG_DEFAULT;
}

/**
 *
 * @brief adcStartup()
 * Example start up sequence for the ADS1118.
 *
 * Before calling this function, the device must be powered and
 * the SPI/GPIO pins of the MCU must have already been configured.
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
    uint8_t dLength;
    uint8_t regTXdata[4] = {0};
    uint8_t regRXdata[4] = {0};
    uint16_t regValue = 0;

    regTXdata[0] = 0;
    regTXdata[1] = 0;
    dLength = 2;
    if(address == 1) dLength = 4;

    spiSendReceiveArrays(regTXdata, regRXdata, dLength);

    regValue =  combineBytes(regRXdata[0], regRXdata[1]);
    registerMap[address] = regValue;
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
 * @return uint16_t of config register
 */
uint16_t writeSingleRegister(uint8_t address, uint16_t data)
{
    //
    // Check that the register address is in range
    //
    assert(address <= MAX_REGISTER_ADDRESS);

    uint8_t regData[2], RxData[4];
    regData[0] = data>>8;
    regData[1] = data & 0xFF;

    // Write should take place immediately and then read back for verification
    spiSendReceiveArrays(regData, RxData, 4);
    // Check if what was received was the same as what was sent
    if(data == combineBytes(RxData[2], RxData[3]))
    {
        registerMap[address] = data;
        return data;
    }

    return 0;
}

/**
 * @brief readData()
 * Reads ADC conversion data.
 *
 * @return  int16_t Conversion register contents
 *
 */
int16_t readData(void)
{
    uint16_t regValue = 0;
    // Get the contents of the config register, we want to keep the configuration just start the conversion
    regValue = 1<<15 | registerMap[CONFIG_ADDRESS];

    //
    // Write to the config register and start a conversion
    //
    writeSingleRegister(CONFIG_ADDRESS, regValue);

    // Monitor DRDY
    // and hold CS low
    setCS(LOW);
    waitForDRDYinterrupt(1000);
    setCS(HIGH);
    // Read conversion results
    registerMap[CONVERSION_ADDRESS] = readSingleRegister(CONVERSION_ADDRESS);
    return (int16_t) registerMap[CONVERSION_ADDRESS];
}

/**
 * @brief startAdcConversion()
 * Starts the ADC conversion.
 *
 * @return  uint16_t configuration register
 *
 */
uint16_t startAdcConversion()
{
    // Get the contents of the config register, we want to keep the configuration just start the conversion
    uint16_t regValue = registerMap[CONFIG_ADDRESS];

    regValue = CONFIG_SS_MASK | regValue;
    writeSingleRegister(CONFIG_ADDRESS, regValue);

    return regValue;
}

/**
 * @brief stopAdcConversion()
 *
 * @brief   Stops the ADC conversion if in continuous mode.
 *
 * @return  uint16_t configuration register
 *
 */
uint16_t stopAdcConversion()
{
    // Get the contents of the config register
    uint16_t regValue = registerMap[CONFIG_ADDRESS];

    regValue =  regValue | CONFIG_MODE_SS;
    writeSingleRegister(CONFIG_ADDRESS, regValue);
    registerMap[CONFIG_ADDRESS] = regValue;
    return regValue;
}

/**
 *
 * @brief getRegisterValue()
 * Getter function to access registerMap array from outside of this module.
 *
 * @param[in] address Is the address of the register to read.
 *
 * NOTE: The internal registerMap arrays stores the last known register value,
 * since the last read or write operation to that register. This function
 * does not communicate with the device to retrieve the current register value.
 * For the most up-to-date register data or retrieving the value of a hardware
 * controlled register, it is recommend to use readSingleRegister() to read the
 * current register value.
 *
 * @return uint16_t Register value.
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

/**
 *
 * @brief ads1118_measure_internal_temperature_example()
 * Captures the internal temperature sensor data and returns temperature in deg C
 *
 * @return Returns floating point value of temperature in deg C
 */
float ads1118_measure_internal_temperature_example(void)
{
    float temp_C;
    int16_t cValue;
    /* Store current configuration value to restore later */
    uint16_t regValue = getRegisterValue(CONFIG_ADDRESS);

    /* Change configuration to temperature sensor mode */
    registerMap[CONFIG_ADDRESS] = regValue | TS_MODE_TS;
    /* If operating in continuous conversion mode read the data a second time 
	 *  to make sure that the previous data was not with an incorrect configuration
	 */
	if(regValue && CONFIG_MODE_SS) 
		/* Start conversion and read result */
		cValue = readData();
	else
	{	/* Start conversion and read result */
		cValue = readData();
		/* Start conversion and read result */
		cValue = readData();
	}
#ifdef ADS1018
    /* Temperature is 12-bit left-justified so need to get the correct value */
    cValue = cValue >> 4;
    /* Use proper coefficient to get the temperature */
    temp_C = cValue * 0.125;
#else
    /* Temperature is 14-bit left-justified so need to get the correct value */
    cValue = cValue >> 2;
    /* Use proper coefficient to get the temperature */
    temp_C = cValue * 0.03125;
#endif
    /* Restore the previous configuration */
    writeSingleRegister(CONFIG_ADDRESS, regValue);

    return temp_C;
}
