/* --COPYRIGHT--,BSD
 * Copyright (c) 2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#include "unit_tests.h"


/* NOTES:
 * - Unless specified, tests should be self-contained (results should not depend on the testing sequence).
 * - Tests that modify register configurations should restore the device to its default state.
 * - All tests should return a boolean data type.
 * - All test functions and conditions should be documented to explain their significance.
 * - Avoid polling nDRDY without a timeout to avoid the possibility of software entering into an infinite loop.
 */



///////////////////////////////////   MAIN   ///////////////////////////////////

// This function will run all test cases and return true if all tests PASS
bool run_self_tests(void)
{
    bool allTestsPassed = true;

    delay_ms(50);   // Delay in case we are calling this function while the power supplies are still settling.
    toggleRESET();  // NOTE: We are assuming that this function is working before we've tested it.


    /// GPIO Functionality ///////////////////////////////////////

    // NOTE: All tests assume GPIO pins are controlled by MCU.
    // If pins are tied, update the tests to check for desired state.

    allTestsPassed &= test_PWDN_pin();
    allTestsPassed &= test_START_pin();
    allTestsPassed &= test_RESET_pin();
    allTestsPassed &= test_CS_pin();
    allTestsPassed &= test_DRDY_interrupt();


    /// SPI Functionality ////////////////////////////////////////

    allTestsPassed &= test_read_register();
    allTestsPassed &= test_write_register();
    allTestsPassed &= test_reset_command();
    allTestsPassed &= test_multiple_read_write();
    allTestsPassed &= test_read_data();


    /// ADC Performance  /////////////////////////////////////////

    allTestsPassed &= test_noise();


    return allTestsPassed;
}



////////////////////////////////   TEST CASES   ////////////////////////////////


/// GPIO Functionality ///////////////////////////////////////

// bool test_PWDN_pin(void)
// Tests if the GPIO pin connected to /PWDN is functioning
// NOTE: This test will NOT catch broken connections between the MCU and ADC!
bool test_PWDN_pin(void)
{
    bool b_pass = true;
    bool setting;

    // Set nCS LOW and read it back
    setPWDN(LOW);
    setting = getPWDN();
    b_pass &= (setting == LOW);

    // Set nPWDN HIGH and read it back
    setPWDN(HIGH);
    setting = getPWDN();
    b_pass &= (setting == HIGH);

    return b_pass;
}

// bool test_START_pin(void)
// Tests if the GPIO pin connected to START is functioning
// NOTE: This test will NOT catch broken connections between the MCU and ADC!
bool test_START_pin(void)
{
    bool b_pass = true;
    bool setting;

    // Set nCS LOW and read it back
    setSTART(LOW);
    setting = getSTART();
    b_pass &= (setting == LOW);

    // Set START HIGH and read it back
    setSTART(HIGH);
    setting = getSTART();
    b_pass &= (setting == HIGH);

    return b_pass;
}

// bool test_RESET_pin(void)
// Tests if the GPIO pin connected to /RESET is functioning
// NOTE: This test will NOT catch broken connections between the MCU and ADC!
bool test_RESET_pin(void)
{
    bool b_pass = true;
    bool setting;

    // Set nRESET LOW and read it back
    setRESET(LOW);
    setting = getRESET();
    b_pass &= (setting == LOW);

    // Set nRESET HIGH and read it back
    setRESET(HIGH);
    setting = getRESET();
    b_pass &= (setting == HIGH);

    return b_pass;
}

// bool test_CS_pin(void)
// Tests if the GPIO pin connected to /CS is functioning
// NOTE: This test will NOT catch broken connections between the MCU and ADC!
bool test_CS_pin(void)
{
    bool b_pass = true;
    bool setting;

    // Set nCS LOW and read it back
    setCS(LOW);
    setting = getCS();
    b_pass &= (setting == LOW);

    // Set nCS HIGH and read it back
    setCS(HIGH);
    setting = getCS();
    b_pass &= (setting == HIGH);

    return b_pass;
}

// bool test_DRDY_interrupt(void)
// Tests if the /DRDY interrupt and interrupt handler is functioning
bool test_DRDY_interrupt(void)
{
    bool b_pass = true;

    // Set START and nPWDN pins high and check if nDRDY is active and interrupt is functioning
    setPWDN(HIGH);
    setSTART(HIGH);
    toggleRESET();  // Abort current conversion
    b_pass &= waitForDRDYinterrupt(50);

    return b_pass;
}


/// SPI Functionality ////////////////////////////////////////

// bool test_read_register(void)
// Tests if reading a (non-zero) register returns the expected result.
bool test_read_register(void)
{
    bool b_pass = true;
    uint8_t value;

    toggleRESET();

    value = readSingleRegister(REG_ADDR_POWER);
    b_pass &= (value == POWER_DEFAULT);

    return b_pass;
}

// bool test_write_register(void)
// Tests if writing to and then reading back a register returns the expected result.
bool test_write_register(void)
{
    bool b_pass = true;
    uint8_t writeValue, readValue;

    writeValue = (~POWER_DEFAULT) & (0x13);
    writeSingleRegister(REG_ADDR_POWER, writeValue);

    readValue = readSingleRegister(REG_ADDR_POWER);
    b_pass &= (readValue == writeValue);
	
	// Restore default register state
    sendCommand(OPCODE_RESET);

    return b_pass;
}

// bool test_reset_command(void)
// Tests if the RESET SPI command causes a register to return to its default value.
bool test_reset_command(void)
{
    bool b_pass = true;
    uint8_t writeValue, readValue;

    // Write non-default value to register
    writeValue = (~POWER_DEFAULT) & (0x13);
    writeSingleRegister(REG_ADDR_POWER, writeValue);

    // Issue reset command
    sendCommand(OPCODE_RESET);

    // Check that internal array was automatically updated
    b_pass &= (getRegisterValue(REG_ADDR_POWER) == POWER_DEFAULT);

    // Check if read register command returns default value
    readValue = readSingleRegister(REG_ADDR_POWER);
    b_pass &= (readValue == POWER_DEFAULT);

    return b_pass;
}

// bool test_multiple_read_write(void)
// Tests if writing to and then reading back multiple registers returns the expected results.
bool test_multiple_read_write(void)
{
    bool b_pass = true;
    uint8_t i, writeValues[13];

    // Test data - register bits are inverted where reasonable
    writeValues[REG_ADDR_ID]            = 0x00u;    // NOTE: Cannot write to ID register
    writeValues[REG_ADDR_POWER]         = (uint8_t) (~REG_ADDR_POWER & 0x13u);
    writeValues[REG_ADDR_INTERFACE]     = (uint8_t) (~INTERFACE_DEFAULT & 0x0Fu);
    writeValues[REG_ADDR_MODE0]         = (uint8_t) (~MODE0_DEFAULT & 0xFFu);
    writeValues[REG_ADDR_MODE1]         = (uint8_t) (~MODE1_DEFAULT & 0xFFu);
    writeValues[REG_ADDR_MODE2]         = (uint8_t) (~MODE2_DEFAULT & 0xFFu);
    writeValues[REG_ADDR_INPMUX]        = (uint8_t) (~INPMUX_DEFAULT & 0xFFu);
    writeValues[REG_ADDR_OFCAL0]        = (uint8_t) (~OFCAL0_DEFAULT & 0xFFu);
    writeValues[REG_ADDR_OFCAL1]        = (uint8_t) (~OFCAL1_DEFAULT & 0xFFu);
    writeValues[REG_ADDR_OFCAL2]        = (uint8_t) (~OFCAL2_DEFAULT & 0xFFu);
    writeValues[REG_ADDR_FSCAL0]        = (uint8_t) (~FSCAL0_DEFAULT & 0xFFu);
    writeValues[REG_ADDR_FSCAL1]        = (uint8_t) (~FSCAL1_DEFAULT & 0xFFu);
    writeValues[REG_ADDR_FSCAL2]        = (uint8_t) (~FSCAL2_DEFAULT & 0xFFu);

    // Write registers (skipping ID register)
    writeMultipleRegisters(REG_ADDR_POWER, 13 - 1, &writeValues[1]);

    // Check that internal array was automatically updated
    for (i = REG_ADDR_POWER; i <= REG_ADDR_FSCAL2; i++)
    {
        b_pass &= (getRegisterValue(i) == writeValues[i]);
    }

    // Read back registers
    readMultipleRegisters(REG_ADDR_POWER, 13 - 1);

    // Check that read back values match the written values
    for (i = REG_ADDR_POWER; i <= REG_ADDR_FSCAL2; i++)
    {
        b_pass &= (getRegisterValue(i) == writeValues[i]);
    }
	
	// Restore default register state
    sendCommand(OPCODE_RESET);

    return b_pass;
}

// bool test_read1_data(void)
// Tests if reading data from the internal monitoring channels on ADC1 returns data within an expected range.
bool test_read1_data(void)
{
    bool b_pass = true;
    uint8_t i, writeValue, statusByte, dataBytes[4], crcByte;
    int32_t dataValue, upperLimit, lowerLimit;

    // Ensure device is powered, converting, and reset to default values
    startConversions();
    toggleRESET();


    //
    // Write registers
    //

    // Clear reset flag
    writeValue = POWER_DEFAULT & ~POWER_RESET_MASK;
    writeSingleRegister(REG_ADDR_POWER, writeValue);

    // Disable data checksum & status bytes
    writeValue = INTERFACE_CRC_OFF;
    writeSingleRegister(REG_ADDR_INTERFACE, writeValue);


    //
    // AVDD
    //

    // Configure input MUX to measure AVDD/4
    writeValue = INPMUX_MUXP_AVDD_P | INPMUX_MUXN_AVDD_N;
    writeSingleRegister(REG_ADDR_INPMUX, writeValue);

    // Wait and check that /DRDY interrupt occurred
    b_pass &= waitForDRDYinterrupt(100);

    //  Read data in read direct mode
    dataValue = readData(NULL, NULL, NULL);

    // Check AVDD/4 measurement data
    lowerLimit = 0x3CCCCCCC;    // 4.75 V
    upperLimit = 0x43333333;    // 5.25 V
    b_pass &= ((dataValue >= lowerLimit) & (dataValue <= upperLimit));


    //
    // DVDD
    //

    // Enable status byte
    writeValue = INTERFACE_STATUS_MASK;
    writeSingleRegister(REG_ADDR_INTERFACE, writeValue);

    // Configure input MUX to measure DVDD/4
    writeValue = INPMUX_MUXP_DVDD_P | INPMUX_MUXN_DVDD_N;
    writeSingleRegister(REG_ADDR_INPMUX, writeValue);

    // Wait and check that /DRDY interrupt occurred
    b_pass &= waitForDRDYinterrupt(100);

    //  Read data in read direct mode
    dataValue = readData(&statusByte, NULL, NULL);

    // Check status byte
    b_pass &= (statusByte & ~STATUS_EXTCLK & ~STATUS_ADC2) == STATUS_ADC1;

    // Check DVDD measurement data
    lowerLimit = 0x228F5C28;    // 2.70 V
    upperLimit = 0x43333333;    // 5.25 V
    b_pass &= ((dataValue >= lowerLimit) & (dataValue <= upperLimit));


    //
    // Temperature Sensor
    //

    // Enable checksum and status bytes
    writeValue = INTERFACE_STATUS_MASK | INTERFACE_CRC_CHKSUM;
    writeSingleRegister(REG_ADDR_INTERFACE, writeValue);

    // Configure input MUX to measure INT TEMP
    writeValue = INPMUX_MUXP_TEMP_P | INPMUX_MUXN_TEMP_N;
    writeSingleRegister(REG_ADDR_INPMUX, writeValue);

    // Wait and check that /DRDY interrupt occurred
    b_pass &= waitForDRDYinterrupt(100);

    // Read data in read direct mode
    dataValue = readData(&statusByte, &dataBytes, &crcByte);

    // Check STATUS byte
    b_pass &= (statusByte & ~STATUS_EXTCLK & ~STATUS_ADC2) == STATUS_ADC1;

    // Check TEMP measurement data
    lowerLimit = 0x05BAB218;    // 0 deg. C
    upperLimit = 0x06CDF267;    // 50 deg. C
    b_pass &= ((dataValue >= lowerLimit) & (dataValue <= upperLimit));

    // Check checksum byte
    b_pass &= (crcByte == calculateChecksum(dataBytes, 4));


    //
    // Inputs shorted
    //

    // Clear reset flag
    writeValue = POWER_VBIAS_MASK | POWER_INTREF_MASK;
    writeSingleRegister(REG_ADDR_POWER, writeValue);

    // Enable CRC and STATUS bytes
    writeValue = INTERFACE_STATUS_MASK | INTERFACE_CRC_ON;
    writeSingleRegister(REG_ADDR_INTERFACE, writeValue);

    // Configure input MUX to measure INT TEMP
    writeValue = INPMUX_MUXP_AINCOM | INPMUX_MUXN_AINCOM;
    writeSingleRegister(REG_ADDR_INPMUX, writeValue);

    // Wait and check that /DRDY interrupt occurred
    b_pass &= waitForDRDYinterrupt(100);

    // Read data in read direct mode
    dataValue = readData(&statusByte, &dataBytes[0], &crcByte);

    // Check status byte
    b_pass &= (statusByte & ~STATUS_EXTCLK & ~STATUS_ADC2) == STATUS_ADC1;

    // Check TEMP measurement data
    lowerLimit = 0x05BAB218;    // 0 deg. C
    upperLimit = 0x06CDF267;    // 50 deg. C
    b_pass &= ((dataValue >= lowerLimit) & (dataValue <= upperLimit));

    // Check checksum byte
    b_pass &= (crcByte == calculateChecksum(dataBytes, 4));


    // Enable input chop to minimize offset
    //writeValue = MODE0_DEFAULT | MODE0_CHOP_MASK;
    //writeSingleRegister(REG_ADDR_MODE0, writeValue);

    // Enable data checksum & status bytes
    //writeValue = INTERFACE_STATUS_MASK | INTERFACE_CRC_CHKSUM;
    //writeSingleRegister(REG_ADDR_INTERFACE, writeValue);




    // Read all data monitor channels
    for (i = 0; i < 5; i++)
    {
        // Not required, but added to allow for debugger to halt code execution
        setSTART(LOW);

        // Wait and check that /DRDY interrupt occurred
        b_pass &= waitForDRDYinterrupt(10);

        //  Read data in read direct mode
        dataValues[i] = readData(&statusBytes[i], NULL, DIRECT);

        // Not required, but added to allow for debugger to halt code execution
        setSTART(HIGH);
    }

    // Check status bytes
    statusGood = STATUS_ADC1 & ~STATUS_REF_ALM & ~STATUS_PGAL_ALM;
    b_pass &= (statusBytes[0] == (statusGood | STATUS_CHID_OFFSET)  );
    b_pass &= (statusBytes[1] == (statusGood | STATUS_CHID_VCC)     );
    b_pass &= (statusBytes[2] == (statusGood | STATUS_CHID_TEMP)    );
    b_pass &= (statusBytes[3] == (statusGood | STATUS_CHID_GAIN)    );
    b_pass &= (statusBytes[4] == (statusGood | STATUS_CHID_REF)     );

    /* STATUS byte field masks */
    #define STATUS_ADC1                         ((uint8_t) 0x40)            /* Indicates new ADC1 data */
    #define STATUS_EXTCLK                       ((uint8_t) 0x20)            /* Indicates ADC clock source */
    #define STATUS_REF_ALM                      ((uint8_t) 0x10)            /* Low Reference Alarm   - Only used with ADC1 */
    #define STATUS_PGAL_ALM                     ((uint8_t) 0x08)            /* PGA Output Low Alarm  - Only used with ADC1 */
    #define STATUS_PGAH_ALM                     ((uint8_t) 0x04)            /* PGA Output High Alarm - Only used with ADC1 */
    #define STATUS_PGAD_ALM                     ((uint8_t) 0x02)            /* PGA Diff Output Alarm - Only used with ADC1 */
    #define STATUS_RESET                        ((uint8_t) 0x01)            /* Indicates device reset (re-named to avoid conflict) */


    // Check OFFSET data
    lowerLimit = 0xFFFFFEE0;    // -91.5 uV
    upperLimit = 0x00000120;    // +91.5 uV
    b_pass &= ((dataValues[0] >= lowerLimit) & (dataValues[0] <= upperLimit));

    // Check VCC data
    lowerLimit = 0x00390000;    // 4.75 V
    upperLimit = 0x003F0000;    // 5.25 V
    b_pass &= ((dataValues[1] >= lowerLimit) & (dataValues[1] <= upperLimit));

    // Check TEMP data
    lowerLimit = 0x000384AE;   // 0 deg. C, with 5.25V reference
    upperLimit = 0x002BB2B0;   // 50 deg. C with 0.50V reference
    b_pass &= ((dataValues[2] >= lowerLimit) & (dataValues[2] <= upperLimit));

    // Check GAIN data
    lowerLimit = 0x0070CCCC;    // -0.6% V/V
    upperLimit = 0x007F3333;    // +0.6% V/V

    b_pass &= ((dataValues[3] >= lowerLimit) & (dataValues[3] <= upperLimit));

    // Check REF data
    // TODO: Update these values based on your expected reference voltage accuracy!
    lowerLimit = 0x00060000;    // 0.50 V
    upperLimit = 0x003F0000;    // 5.25 V
    b_pass &= ((dataValues[4] >= lowerLimit) & (dataValues[4] <= upperLimit));

    return b_pass;
}



test_read2_data

/// ADC Performance  /////////////////////////////////////////

// bool test_noise(void)
// Tests if reading data with ADC input shorted returns expected data variance.
// NOTE: The test limit needs to be updated for different voltage reference values. 
bool test_noise(void)
{
    bool b_pass = true;
    uint8_t i, writeValues[7], statusByte, statusGood, numSamples;
    int32_t dataValue, ppNoiseLimit, ppMin, ppMax;


    // Test configuration
    statusGood = STATUS_NEW_MASK & ~STATUS_OVF_MASK & ~STATUS_SUPPLY_MASK;

    writeValues[0]   = CONFIG0_DEFAULT & ~CONFIG0_MUXMOD_MASK;  // Auto-scan mode
    writeValues[1]   = CONFIG1_DEFAULT & CONFIG1_DRATE_1953SPS; // Slowest data rate
    writeValues[2]   = MUXSCH_DEFAULT;
    writeValues[3]   = MUXDIF_DEFAULT;                          // Differential channels off
    writeValues[4]   = 0x00;                                    // Single-ended channels off
    writeValues[5]   = 0x00;
    writeValues[6]   = SYSRED_OFFSET_ENABLE;                    // Enable offset monitor

    // Ensure device is powered, converting, and reset to default values
    startConversions();
    toggleRESET();

    // Write registers
    writeMultipleRegisters(REG_ADDR_CONFIG0, 7, writeValues);

    // Read all data monitor channels
    numSamples = 50;
    for (i = 0; i < numSamples; i++)
    {
        // Not required, but added to allow for debugger to halt code execution
        setSTART(LOW);

        // Wait and check that /DRDY interrupt occurred
        b_pass &= waitForDRDYinterrupt(10);

        // Read data with read command
        dataValue = readData(&statusByte, NULL, NULL);

        // Check status byte
        b_pass &= (statusByte == (statusGood | STATUS_CHID_OFFSET));

        // Keep track of MIN and MAX conversion results
        if (i == 0)
        {
            ppMin = dataValue;
            ppMax = dataValue;
        }
        else if (dataValue < ppMin)
        {
            ppMin = dataValue;
        }
        else if (dataValue > ppMax)
        {
            ppMax = dataValue;
        }

        // Not required, but added to allow for debugger to halt code execution
        setSTART(HIGH);
    }

    // Check if we exceeded expected PP noise limit
    ppNoiseLimit = 6 * 3 * 4;   // ppNoiseLimit = Crest factor * uVrms (for selected data rate) * codes/uV (VREF dependent)
    b_pass &= ( (ppMax - ppMin) < ppNoiseLimit );
	
	// Restore default register state
    sendCommand(OPCODE_RESET);

    return b_pass;
}
