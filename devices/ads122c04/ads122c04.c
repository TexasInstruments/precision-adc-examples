/**
 * @file ads122c04.c
 *
 * @brief This file contains all basic communication and device setup for the ADS1x2C04x device family.
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

#include "ads122c04.h"
#include "crc.h"
#include "hal.h"

/* Initialize global variables */
/* Array used to recall device register map configurations */
uint8_t ADC_RegisterMap[NUM_REGISTERS];
const char *adcRegisterNames[NUM_REGISTERS] = {"CONFIG0", "CONFIG1", "CONFIG2", "CONFIG3"};

/*****************************************************************************
 *
 * Internal variables
 *
 ****************************************************************************/


/*****************************************************************************
 *
 * Function Definitions
 *
 ****************************************************************************/

/**
 *
 * @brief restoreRegisterDefaults()
 * Updates the ADC_RegisterMap[] array to its default values.
 *
 * NOTES:
 * - If the MCU keeps a copy of the ADS1x2C04 register settings in memory,
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
    // Initialize the register map
    ADC_RegisterMap[CONFIG0_ADDRESS] = CONFIG0_DEFAULT;
    ADC_RegisterMap[CONFIG1_ADDRESS] = CONFIG1_DEFAULT;
    ADC_RegisterMap[CONFIG2_ADDRESS] = CONFIG2_DEFAULT;
    ADC_RegisterMap[CONFIG3_ADDRESS] = CONFIG3_DEFAULT;
    // Verify the register map
    ADC_RegisterMap[CONFIG0_ADDRESS] = registerRead(CONFIG0_ADDRESS);
    ADC_RegisterMap[CONFIG1_ADDRESS] = registerRead(CONFIG1_ADDRESS);
    ADC_RegisterMap[CONFIG2_ADDRESS] = registerRead(CONFIG2_ADDRESS);
    ADC_RegisterMap[CONFIG3_ADDRESS] = registerRead(CONFIG3_ADDRESS);

}

/**
 *
 * @brief adcStartup()
 * Example start up sequence for the ADS122C04.
 *
 * Before calling this function, the device must be powered and
 * the I2C/GPIO pins of the MCU must have already been configured.  Reset must be set high.
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
    // (REQUIRED) Initialize internal 'ADC_RegisterMap' array with device default settings
    //
    restoreRegisterDefaults();

    //
    // (OPTIONAL) Read back all registers and Check STATUS register (if exists) for faults
    //
}

/**
 *
 * @brief registerRead()
 * Reads the contents of a single register at the specified address.
 *
 * @param[in] address Is the 8-bit address of the register to read.
 *
 * @return The 8-bit register read result as an unsigned value.
 */
uint8_t registerRead(uint8_t address)
{
    //
    // Check that the register address is in range
    //
    assert(address <= MAX_REGISTER_ADDRESS);
    uint8_t regValue = 0;
    int8_t retStatus;

    uint8_t dataRX[8], dataTX[8];
    dataTX[0] = CMD_RREG | (address << 2);

    // Data integrity information is transmitted with the register data for verification
    // Check for data integrity modes

    // Inverted data
    if(INVERTED_ENABLED)
    {
        retStatus = I2C_SendReceive(dataTX, COMMAND_BYTE_LENGTH , dataRX, (2 * REGISTER_BYTE_LENGTH ));
        if(retStatus != false)
        {
            /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

            return 0;
        }
        // Validate the data with and XOR
        if((dataRX[0] ^ dataRX[1]) < 0xFF)
        {
            /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

            return 0;
        }
    }
    // CRC data
    else if(CRC_ENABLED)
    {
        retStatus = I2C_SendReceive(dataTX, COMMAND_BYTE_LENGTH , dataRX, ( REGISTER_BYTE_LENGTH + CRC_LENGTH ));
        if(retStatus != false)
        {
            /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

            return 0;
        }
        CRCWORD valueCRC = (dataRX[1] << 8) | dataRX[2];
        // Validate the data by comparing the calculated CRC to the captured CRC
        if(getCRC(dataRX,1, CRC_INITIAL_SEED) != valueCRC)
        {
            /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

            return 0;
        }
    }
    // No data integrity check required
    else
    {
        retStatus = I2C_SendReceive(dataTX, COMMAND_BYTE_LENGTH , dataRX, REGISTER_BYTE_LENGTH );
        if(retStatus != false)
        {
            /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

            return 0;
        }
    }

    regValue =  dataRX[0];
    ADC_RegisterMap[address] = regValue;
    return regValue;
}

/**
 *
 * @brief registerWrite()
 * Writes data to a single register and also read it back for internal
 * confirmation.
 *
 * @param[in] address Is the address of the register to write to.
 * @param[in] data Is the value to write.
 *
 * @return when true, data transmitted does not match data read
 */
uint8_t registerWrite(uint8_t address, uint8_t data)
{
    //
    // Check that the register address is in range
    //
    assert(address <= MAX_REGISTER_ADDRESS);
    int8_t retStatus;
    uint8_t regValue;
    uint8_t dataRX[8], dataTX[8];
    dataTX[0] = CMD_WREG | (address << 2);
    dataTX[1] = data;
    // Write the data to the desired ADC register address
    retStatus = I2C_SendReceive(dataTX, (COMMAND_BYTE_LENGTH + REGISTER_BYTE_LENGTH), dataRX, 0);
    if(retStatus != false)
    {

        /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

        return 1;
    }

    // Store the data to the local memory copy
    ADC_RegisterMap[address] = data;
    // Then read back the register to verify the contents
    regValue = registerRead(address);

    if (regValue != data)
    {
        /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

        return 1;
    }


    // Make sure the local copy agrees with the register read
    ADC_RegisterMap[address] = regValue;
    return 0;
}

/**
 *
 * @brief sendCommand()
 * Sends a single byte command to the ADC.
 *
 * @param[in] command Is the unsigned command byte to be written to the ADC.
 *
 * @return when true, command failed
 */
uint8_t sendCommand(uint8_t command)
{
    /* Assert if this function is used to send any of the following commands */
    assert(CMD_RREG != command);        /* Use "registerRead()"   */
    assert(CMD_WREG != command);        /* Use "registerWrite()"  */
    assert(CMD_RDATA != command);       /* Use "dataRead()"       */

    int8_t retStatus;
    uint8_t dataRX[8], dataTX[8];
    dataTX[0] = command;

    retStatus = I2C_SendReceive(dataTX, COMMAND_BYTE_LENGTH, dataRX, 0);
    if(retStatus != false)
    {
        /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

        return 1;
    }

    return 0;
}

/**
 *
 * @brief resetDevice()
 * Resets the ADC and restores default configuration.
 *
 * @param[in] bAction Is the method used for reset; 0 is by command and 1 by pin.
 *
 * @return true, when reset action completes
 */
bool resetDevice(bool bAction)
{
    uint8_t retStatus;
    if( bAction)
    {
        // Toggle the reset pin
        toggleRESET();
    }
    else
    {
        retStatus = sendCommand(CMD_RESET);
        if(retStatus != false)
        {
            /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

            return false;
        }
    }

    delay_ms(1000);
    restoreRegisterDefaults();
    return true;
}
/**
 *
 * @brief dataRead()
 * Reads and returns ADC data.
 *
 * @param[in,out] *dCount Pointer to conversion count variable.
 * @param[in,out] *dInvCount Pointer to inverted version of conversion count variable.
 * @param[in,out] *dCRC Pointer to either inverted data or CRC depending on configuration.
 *
 * @return signed 32-bit integer
 */
int32_t dataRead(uint8_t *dCount, uint8_t *dInvCount, uint32_t *dCRC)
{
    int32_t dValue = 0;
    uint8_t dTemp[DATA_LENGTH];
    int8_t retStatus;
    uint8_t dataRX[8], dataTX[8];
    dataTX[0] = CMD_RDATA;
    uint8_t byteLengthRX = DATA_LENGTH;
    uint8_t i;
    CRCWORD vCRC;

    // Determine the length of data for result based on conversion count and data integrity mode
    if(COUNT_ENABLED)
    {
        if(CRC_ENABLED)
        {
            byteLengthRX += (COUNT_LENGTH + CRC_LENGTH);
        }
        else if(INVERTED_ENABLED)
        {
            byteLengthRX += ((2 * COUNT_LENGTH) + INV_LENGTH);
        }
        else
        {
            byteLengthRX += COUNT_LENGTH;
        }
    }
    else
    {
        if(CRC_ENABLED)
        {
            byteLengthRX += CRC_LENGTH;
        }
        else if(INVERTED_ENABLED)
        {
            byteLengthRX += INV_LENGTH;
        }
    }
    // Get the data for the desired length
    retStatus = I2C_SendReceive(dataTX, COMMAND_BYTE_LENGTH, dataRX, byteLengthRX);
    if(retStatus != false)
    {
        /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

        return 1;
    }

    // If conversion counter is enabled
    if(COUNT_ENABLED)
        {
            // If CRC is enabled
            if(CRC_ENABLED)
            {
                dCount[0] = dataRX[0];
                for(i = 0; i < DATA_LENGTH; i++)
                {
                    dTemp[i] = dataRX[i + 1];
                }
                dValue = signExtend(dTemp);
                vCRC = getCRC(dataRX, (COUNT_LENGTH + DATA_LENGTH) , CRC_INITIAL_SEED);

                dCRC[0] = combineBytes(dataRX[COUNT_LENGTH + DATA_LENGTH], dataRX[COUNT_LENGTH + DATA_LENGTH + 1]);

                if((vCRC - (CRCWORD) dCRC[0]) != 0)
                {

                    /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */
                }
            }
            // If inverted data mode is enabled
            else if(INVERTED_ENABLED)
            {
                dCount[0] = dataRX[0];
                for(i = 0; i < DATA_LENGTH; i++)
                {
                    dTemp[i] = dataRX[i + 1];
                }
                dValue = signExtend(dTemp);
                dInvCount[0] = dataRX[COUNT_LENGTH + DATA_LENGTH];
                for(i = 0; i < DATA_LENGTH; i++)
                {
                    dTemp[i] = dataRX[i + ((2 * COUNT_LENGTH) + DATA_LENGTH)];
                }
                dCRC[0] = signExtend(dTemp);

                if((dCount[0] ^ dInvCount[0]) != 0xFF)
                {
                    /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

                }

                if((dValue ^ dCRC[0]) != 0xFFFFFFFF)
                {
                    /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

                }
            }
            // Otherwise just send the conversion count and conversion data
            else
            {
                dCount[0] = dataRX[0];
                for(i = 0; i < DATA_LENGTH; i++)
                {
                    dTemp[i] = dataRX[i + COUNT_LENGTH];
                }
                dValue = signExtend(dTemp);
            }
        }
        else
        // Otherwise don't send the conversion count
        {
            // If CRC is enabled
            if(CRC_ENABLED)
            {
                vCRC = getCRC(dataRX, DATA_LENGTH , CRC_INITIAL_SEED);

                dCRC[0] = combineBytes(dataRX[DATA_LENGTH], dataRX[DATA_LENGTH + 1]);

                if((vCRC - (CRCWORD) dCRC[0]) != 0)
                {
                    /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

                }
            }
            // If inverted data mode is enabled
            else if(INVERTED_ENABLED)
            {
                dValue = signExtend(dataRX);
                for(i = 0; i < DATA_LENGTH; i++)
                {
                    dTemp[i] = dataRX[i + (DATA_LENGTH)];
                }
                dCRC[0] = signExtend(dTemp);

                if((dValue ^ dCRC[0]) != 0xFFFFFFFF)
                {
                    /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */


                }
            }
            else
            // Otherwise just send the conversion data
            {
                dValue = signExtend(dataRX);
            }
        }

    return dValue;
}

/**
 *
 * @brief sendStartSync()
 * Sends the START/SYNC command to the ADC.
 *
 * @return  0 if successful, and 1 if unsuccessful
 *
 */
uint8_t sendStartSync(void)
{
    return sendCommand(CMD_START_SYNC);
}

/**
 *
 * @brief sendPowerdown()
 * Sends the POWERDOWN command to the ADC.
 *
 * @return  0 if successful, and 1 if unsuccessful
 *
 */
uint8_t sendPowerdown(void)
{
    return sendCommand(CMD_POWERDOWN);
}

/**
 *
 * @brief startContConversion()
 * Sets the register setting to continuous mode and starts the ADC conversion.
 *
 * @return  uint8_t regValue of updated register
 *
 */
uint8_t startContConversion(void)
{
    uint8_t regValue = getRegisterValue(CONFIG1_ADDRESS);
    uint8_t retStatus;
    if(!(regValue & CONFIG1_CM_CONTINUOUS))
    {
        // Set ADC to continuous conversion mode
        regValue |= CONFIG1_CM_CONTINUOUS;
        retStatus = registerWrite(CONFIG1_ADDRESS, regValue);

        if(retStatus != false)
        {
            /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

            return 0;

        }
    }

    // Send a START_SYNC command
    sendStartSync();
    return regValue;
}

/**
 *
 * @brief startSingleShotConversion()
 * Sets the register setting to single-shot mode and starts conversion.
 *
 * @return  uint8_t regValue of updated register
 *
 */
uint8_t startSingleShotConversion(void)
{
    uint8_t regValue = getRegisterValue(CONFIG1_ADDRESS);
    uint8_t retStatus;

    if(regValue & CONFIG1_CM_CONTINUOUS)
    {
        // Set ADC to single-shot conversion mode
        regValue &= ~CONFIG1_CM_MASK;
        regValue |= CONFIG1_CM_SINGLE_SHOT;
        retStatus = registerWrite(CONFIG1_ADDRESS, regValue);

        if(retStatus != false)
        {
            /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

            return 0;
        }
    }

    // Send a START_SYNC command
    sendStartSync();
    return regValue;
}

/**
 *
 * @brief checkDRDYforEOC()
 * Looks for EOC by checking DRDY bit in register.
 *
 * @param[in] timeout_ms is time to wait while checking to prevent endless loop
 *
 * @return  timeout as boolean with 0 showing timeout has occurred
 *
 */
bool checkDRDYbitForEOC(const uint32_t timeout_ms)
{
    /* --- INSERT YOUR CODE HERE ---
     * Poll the DRDY register bit. To avoid potential infinite
     * loops, you may also want to implement a timer interrupt to occur after
     * the specified timeout period, in case the DRDY bit is inactive.
     * Return a boolean to indicate if nDRDY went low or if a timeout occurred.
     */


    // Convert ms to a # of loop iterations, OR even better use a timer here...

     uint32_t timeout = timeout_ms * CHECK_DRDY_LOOP;   // convert to # of loop iterations 72us per loop at 1MHz SCL
    // Wait for DRDY finished bit or timeout
    do {
        timeout--;
    } while (!(registerRead(CONFIG2_ADDRESS) & CONFIG2_DRDY_NEW) && (timeout > 0));

    return (timeout > 0);
}

/**
 *
 * @brief checkInternalTemperature()
 * Sets the register setting to internal temperature and starts conversion.
 *
 * @param[in] dFormat Is temperature format to return 0 is deg C, 1 is deg F
 *
 * @return  float value Representing the temperature
 *
 */
float checkInternalTemperature(bool dFormat)
{
    uint8_t regValue = getRegisterValue(CONFIG1_ADDRESS);
    int32_t retValue = 0;
    uint8_t dCount = 0;
    uint8_t dInvCount = 0;
    uint32_t dCRC = 0;
    uint8_t retStatus;

    // Enable the temperature sensor for measurement
    retStatus = registerWrite(CONFIG1_ADDRESS, (regValue | CONFIG1_TS_ENABLED));

    if(retStatus != false)
    {
        /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

        return 0;
    }

    // Start a conversion
    sendStartSync();

    // Wait for the conversion to end, then read result
    if(waitForDRDYinterrupt(1000))
    {
        // Read back the conversion result
        retValue = dataRead(&dCount, &dInvCount, &dCRC);
    }

    // After the temperature sensor measurement return back to the previous setting
    retStatus = registerWrite(CONFIG1_ADDRESS, regValue);

    if(retStatus != false)
    {
        /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

        return 0;
    }

    // Return the correct value for the desired format
    if (dFormat == true) return (float) ((convertCodeToTemperature(retValue) * 9 / 5) + 32 );
    else return (float) (convertCodeToTemperature(retValue) );
}

/************************************************************************************//**
 *
 * @brief convertCodeToTemperature()
 *          Convert ADC code to temperature for on-chip temperature sensor
 *          Code is signed and magnitude number of 14 bits
 *              For positive numbers, temp = 0.03125 * code
 *              For negative numbers, temp = -0.03125 * ~(code - 1)
 *
 * @param[in]   dCodes ADC result
 *
 * @return      temperature in floating point in degrees C
 *
 */
float convertCodeToTemperature( int32_t dCodes )
{
    float temp;

    if ( dCodes & (1 << (BIT_RESOLUTION - 1)) ) {
        // Negative value
        temp = -0.03125 * ((~(dCodes - 1) & TEMP_MASK) >> (BIT_RESOLUTION - TEMP_BITRES));
    } else {
        // Positive value
        temp = 0.03125 * ( dCodes >> (BIT_RESOLUTION - TEMP_BITRES));
    }
    return temp;
}

/*****************************************************************************
 *
 * Getter functions
 *
 ****************************************************************************/

/**
 *
 * @brief getRegisterValue()
 * Getter function to access ADC_RegisterMap array from outside of this module.
 *
 * @param[in] address Of desired register contents to return
 *
 * NOTE: The internal registerMap arrays stores the last known register value,
 * since the last read or write operation to that register. This function
 * does not communicate with the device to retrieve the current register value.
 * For the most up-to-date register data or retrieving the value of a hardware
 * controlled register, it is recommend to use registerRead() to read the
 * current register value.
 *
 * @return unsigned 8-bit register value.
 */
uint8_t getRegisterValue(uint8_t address)
{
    assert(address <= MAX_REGISTER_ADDRESS);
    return ADC_RegisterMap[address];
}

/*****************************************************************************
 *
 * Helper functions
 *
 ****************************************************************************/

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
 * @brief signExtend()
 * Combines ADC data bytes into a single signed 32-bit word.
 *
 * @param[in] dataBytes Is a pointer to uint8_t[] where the first element is the MSB.
 *
 * @return Returns the signed-extend 32-bit result.
 */
int32_t signExtend(const uint8_t dataBytes[])
{
#ifdef ADS122C04
    int32_t upperByte   = ((int32_t) dataBytes[0] << 24);
    int32_t middleByte  = ((int32_t) dataBytes[1] << 16);
    int32_t lowerByte   = ((int32_t) dataBytes[2] << 8);

    return (((int32_t) (upperByte | middleByte | lowerByte)) >> 8);     // Right-shift of signed data maintains signed bit
#else
    int32_t upperByte   = ((int32_t) dataBytes[0] << 24);
    int32_t lowerByte   = ((int32_t) dataBytes[1] << 16);

    return (((int32_t) (upperByte | lowerByte)) >> 16);                 // Right-shift of signed data maintains signed bit
#endif
}
