/**
 * \copyright Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
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

#include "unit_tests.h"


// Number of samples to collect for noise testing
#define NUM_SAMPLES_NOISE   (20)

// Helper functions
static void     readSingleChannelPPNoise(int32_t *dataArry, uint8_t channelNum);


/* Unit-tests
 *
 *
 * NOTES:
 * - Unless specified, tests should be self-contained (results should not depend on the testing sequence)!
 * - Tests that modify register configurations should restore the device to its default state.
 * - All test should return a boolean data type!
 * - All test conditions should be a constant data type and defined within the specific test function.
 * - All test functions and conditions should be documented to explain their significance.
 *
 * - In case the device is not working, avoid polling nDRDY to avoid the possibility of software entering into an infinite loop!
 * - Avoid writing tests that verify your code's functionality. Check code functionality elsewhere (through unit/functional tests)!
 */



///////////////////////////////////   MAIN   ///////////////////////////////////

// This function will run all test cases and print results to the console
bool run_self_tests(void)
{
    bool allTestsPassed = true;

    delay_ms(100);   // Delay in case we are calling this function while supply rails are still settling.
    toggleRESET();  // NOTE: We are assuming that this function is currently working.


    /// GPIO Functionality ///////////////////////////////////////

    // NOTE: All tests assume GPIO pins are controlled by MCU.
    // If pins are tied, update the tests to check for desired state.

    allTestsPassed &= test_SYNC_RESET_pin();
    allTestsPassed &= test_CS_pin();
    allTestsPassed &= test_DRDY_interrupt();


    /// SPI Functionality ////////////////////////////////////////

    allTestsPassed &= test_read_register();
    allTestsPassed &= test_write_register();
    allTestsPassed &= test_reset_command();
    //allTestsPassed &= test_multiple_read_write();
    //allTestsPassed &= test_read_data();


    /// ADC Performance  /////////////////////////////////////////

    allTestsPassed &= test_noise();


    return allTestsPassed;
}




////////////////////////////////   TEST CASES   ////////////////////////////////


/// GPIO Functionality ///////////////////////////////////////

bool test_SYNC_RESET_pin(void)
{
    bool b_pass = true;
    bool setting;

    // Set nSYNC/nRESET LOW and read it back
    setSYNC_RESET(LOW);
    setting = getSYNC_RESET();
    b_pass &= (setting == LOW);

    // Set nSYNC/nRESET HIGH and read it back
    setSYNC_RESET(HIGH);
    setting = getSYNC_RESET();
    b_pass = (setting == HIGH);

    return b_pass;
}

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
    b_pass = (setting == HIGH);

    return b_pass;
}


bool test_DRDY_interrupt(void)
{
    bool b_pass = true;

    // Set START and nPWDN pins high and check if nDRDY is active and interrupt is functioning
    toggleSYNC();  // Abort current conversion
    b_pass &= waitForDRDYinterrupt(100);

    return b_pass;
}


/// SPI Functionality ////////////////////////////////////////

bool test_read_register(void)
{
    bool b_pass = true;
    uint16_t value;

    toggleRESET();

    value = readSingleRegister(CLOCK_ADDRESS);
    b_pass &= (value == CLOCK_DEFAULT);

    return b_pass;
}

bool test_write_register(void)
{
    bool b_pass = true;
    uint16_t writeValue, readValue;

    writeValue = GAIN1_PGAGAIN3_128 | GAIN1_PGAGAIN2_128 | GAIN1_PGAGAIN1_128 | GAIN1_PGAGAIN0_128;
    writeSingleRegister(GAIN1_ADDRESS, writeValue);

    readValue = readSingleRegister(GAIN1_ADDRESS);
    b_pass &= (readValue == writeValue);

    return b_pass;
}

bool test_reset_command(void)
{
    bool b_pass = true;
    uint16_t writeValue, readValue;

    // Write non-default value to register
    writeValue = GAIN1_PGAGAIN3_128 | GAIN1_PGAGAIN2_128 | GAIN1_PGAGAIN1_128 | GAIN1_PGAGAIN0_128;
    writeSingleRegister(GAIN1_ADDRESS, writeValue);

    // Issue reset command
    resetDevice();

    // Check that internal array was automatically updated
    b_pass &= (getRegisterValue(GAIN1_ADDRESS) == GAIN1_DEFAULT);

    // Check if read register command returns default value
    readValue = readSingleRegister(GAIN1_ADDRESS);
    b_pass &= (readValue == GAIN1_DEFAULT);

    return b_pass;
}

#if 0
bool test_multiple_read_write(void)
{
    bool b_pass = true;
    uint16_t i, writeValues[7];

    // Test data - when reasonable, register bits where inverted
    writeValues[0]   = 0x74u;   // CONFIG0
    writeValues[1]   = 0x7Cu;   // CONFIG1
    writeValues[2]   = 0xFFu;   // MUXSCH
    writeValues[3]   = 0xFFu;   // MUXDIF
    writeValues[4]   = 0x00u;   // MUXSG0
    writeValues[5]   = 0x00u;   // MUXSG1
    writeValues[6]   = 0xFFu;   // SYSRED

    // In case of external GPIO connections, do not test GPIO pins here.
    // This should be done in a separate test case.

    // Write registers
    writeMultipleRegisters(REG_ADDR_CONFIG0, 7, writeValues);

    // Check that internal array was automatically updated
    for (i = 0; i < 7; i++)
    {
        b_pass &= (getRegisterValue(i) == writeValues[i]);
    }

    // Read back registers
    readMultipleRegisters(REG_ADDR_CONFIG0, 7);

    // Check that read back values match the written values
    for (i = 0; i < 7; i++)
    {
        b_pass &= (getRegisterValue(i) == writeValues[i]);
    }

    return b_pass;
}
#endif

/// ADC Performance  /////////////////////////////////////////

bool test_noise(void)
{
    bool b_pass = true;
    int i, j;
    uint16_t writeValue; //, statusByte, statusGood;
    int32_t maxValue, minValue;
    int32_t channelData[NUM_SAMPLES_NOISE];

    const int32_t pp_limit = 140;    // 20 uVpp

    // Ensure device is powered, converting, and reset to default values
    toggleRESET();

    // Test configuration
    //writeValue = (CLOCK_DEFAULT & ~CLOCK_OSR_MASK) | CLOCK_OSR_16384;
    //writeSingleRegister(CLOCK_ADDRESS, writeValue);

    writeValue = (CLOCK_DEFAULT & ~CLOCK_OSR_MASK) | CLOCK_OSR_16384;
    writeSingleRegister(CLOCK_ADDRESS, writeValue);

    writeValue = CH0_CFG_MUX0_ADC_INPUT_SHORT;
    writeSingleRegister(CH0_CFG_ADDRESS, writeValue);

#if (CHANNEL_COUNT > 1)
    writeValue = CH1_CFG_MUX1_ADC_INPUT_SHORT;
    writeSingleRegister(CH1_CFG_ADDRESS, writeValue);
#endif
#if (CHANNEL_COUNT > 2)
    writeValue = CH2_CFG_MUX2_ADC_INPUT_SHORT;
    writeSingleRegister(CH2_CFG_ADDRESS, writeValue);
#endif
#if (CHANNEL_COUNT > 3)
    writeValue = CH3_CFG_MUX3_ADC_INPUT_SHORT;
    writeSingleRegister(CH3_CFG_ADDRESS, writeValue);
#endif
#if (CHANNEL_COUNT > 4)
    writeValue = CH4_CFG_MUX4_ADC_INPUT_SHORT;
    writeSingleRegister(CH4_CFG_ADDRESS, writeValue);
#endif
#if (CHANNEL_COUNT > 5)
    writeValue  = CH5_CFG_MUX5_ADC_INPUT_SHORT;
    writeSingleRegister(CH5_CFG_ADDRESS, writeValue);
#endif
#if (CHANNEL_COUNT > 6)
    writeValue = CH6_CFG_MUX6_ADC_INPUT_SHORT;
    writeSingleRegister(CH6_CFG_ADDRESS, writeValue);
#endif
#if (CHANNEL_COUNT > 7)
    writeValue = CH7_CFG_MUX7_ADC_INPUT_SHORT;
    writeSingleRegister(CH7_CFG_ADDRESS, writeValue);
#endif

    // Send a NULL command to ensure that the next response is STATUS
    sendCommand(OPCODE_NULL);

    // Loop through all channels
    for(j = 1; j <= CHANNEL_COUNT; j++)
    {
        // Collect data
        readSingleChannelPPNoise(channelData, j);

        // Calculate PP Noise
        minValue = channelData[0];
        maxValue = channelData[0];
        for(i = 1; i < NUM_SAMPLES_NOISE; i++)
        {
            if (channelData[i] < minValue)      { minValue = channelData[i]; }
            else if (channelData[i] > maxValue) { maxValue = channelData[i]; }
        }

        // Check PP Noise performance
        b_pass &= (maxValue - minValue) <= pp_limit;
    }

    return b_pass;
}


static void readSingleChannelPPNoise(int32_t *dataArry, uint8_t channelNum)
{
    int i;

    // Read data
    for(i = 1; i <= NUM_SAMPLES_NOISE; i++)
    {
        waitForDRDYinterrupt(100);

        switch(channelNum)
        {
//            case 1:
//                readData(&dataArry[i], 0, 0, 0, 0, 0, 0, 0); TODO: Fix
//                break;
//            case 2:
//                readData(0, &dataArry[i], 0, 0, 0, 0, 0, 0);
//                break;
//            case 3:
//                readData(0, 0, &dataArry[i], 0, 0, 0, 0, 0);
//                break;
//            case 4:
//                readData(0, 0, 0, &dataArry[i], 0, 0, 0, 0);
//                break;
//            case 5:
//                readData(0, 0, 0, 0, &dataArry[i], 0, 0, 0);
//                break;
//            case 6:
//                readData(0, 0, 0, 0, 0, &dataArry[i], 0, 0);
//                break;
//            case 7:
//                readData(0, 0, 0, 0, 0, 0, &dataArry[i], 0);
//                break;
//            case 8:
//                readData(0, 0, 0, 0, 0, 0, 0, &dataArry[i]);
//                break;
        }
    }
}
