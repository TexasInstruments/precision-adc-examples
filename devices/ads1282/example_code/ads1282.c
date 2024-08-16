/**
 * @file ads1282.c
 *
 * @brief ADS1282 Example Code
 *
 * @copyright Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
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

#include "ads1282.h"


//****************************************************************************
//
// Internal variables
//
//****************************************************************************

/// Internal register map array (to recall current configuration)
static uint8_t registerMap[NUM_REGISTERS];

/// Flag to track if device is in SDATAC mode (false) or RDATAC mode (true)
static bool readContinuousMode = false;


//****************************************************************************
//
// Internal function prototypes
//
//****************************************************************************

static int32_t _signExtendData(const uint8_t dataBytes[]);


//****************************************************************************
//
// Functions
//
//****************************************************************************

//*****************************************************************************
//
//! Getter function to access registerMap array from outside of this module.
//!
//! \fn uint8_t getRegisterValue(const uint8_t address)
//!
//! NOTE: The internal registerMap arrays stores the last know register value,
//! since the last read or write operation to that register. This function
//! does not communicate with the device to retrieve the current register value.
//! For the most up-to-date register data or retrieving the value of a hardware
//! controlled register use readSingleRegister().
//!
//! \return unsigned 8-bit register value.
//
//*****************************************************************************
uint8_t getRegisterValue(const uint8_t address)
{
    assert(address < NUM_REGISTERS);
    return registerMap[address];
}


//*****************************************************************************
//
//! Example start up sequence.
//!
//! \fn void adcStartupRoutine(void)
//!
//! Before calling this function, the device must be powered,
//! the SPI/GPIO pins of the MCU must have already been configured,
//! and the external clock source provided to CLKIN.
//!
//! \return None.
//
//*****************************************************************************
void adcStartupRoutine(void)
{
    /* (OPTIONAL) Provide additional delay time for power supply settling */
    delay_us(50000);

    /* (REQUIRED) Set nPWDN and nRESET pins high to activate ADC */
    setPWDN(HIGH);
    setRESET(HIGH);

    /* (RECOMMENDED) Toggle nRESET pin to assure default register settings
     * and to reset the SPI clock in case of any glitches during initialization */
    toggleRESET();

    /* Ensure internal register array is initialized */
    _restoreRegisterDefaults();

    /* (OPTIONAL) Configure initial device register settings here */

    /* (OPTIONAL) Read back all registers */
    //readMultipleRegisters(ID_ADDRESS, NUM_REGISTERS);

    /* (OPTIONAL) Synchronize readings (with other ADCs) */
}


//*****************************************************************************
//
//! Reads the contents of a single register at the specified address.
//!
//! \fn uint8_t readSingleRegister(const uint8_t address)
//!
//! \param address is the 8-bit address of the register to read.
//!
//! \return Returns the 8-bit register read result.
//
//*****************************************************************************
uint8_t readSingleRegister(const uint8_t address)
{
    // Check that the register address is in range
    assert(address < NUM_REGISTERS);

    uint8_t dataTx[3];    // Holds data to transmit on DIN pin
    uint8_t dataRx[3];    // Placeholder for DOUT received data

    // Put device in "SDATAC" mode to enable register data on DOUT
    if (readContinuousMode) { sendCommand(OPCODE_SDATAC); }

    // Send command
    dataTx[0] = OPCODE_RREG | (address & OPCODE_REG_ADDR_MASK); // 1st command byte
    dataTx[1] = 0x00;                                           // 2nd command byte
    spiSendReceive(dataTx, dataRx, 2);

    // Read data
    dataTx[2] = OPCODE_NOP;
    delay_us(DELAY_T_DLY);                                      // tDLY: wait for DOUT to update
    spiSendReceive(&dataTx[2], &dataRx[2], 1);                  // Clock out register data

    // Update internal register array and return register value
    const uint8_t registerValue = registerMap[address] = dataRx[2];
    return registerValue;
}


//*****************************************************************************
//
//! Reads the contents of multiple registers starting at the specified address.
//!
//! \fn void readMultipleRegisters(const uint8_t startAddress, const uint8_t count)
//!
//! \param startAddress is the 8-bit starting address of the first register to read.
//! \param count total number of register to read
//!
//! \return None. Register data can retrieved later by calling getRegisterValue().
//
//*****************************************************************************
void readMultipleRegisters(const uint8_t startAddress, const uint8_t count)
{
    // Check that the register address and count are in range
    assert((startAddress + (count - 1)) < NUM_REGISTERS);

    uint8_t i;            // Loop index
    uint8_t dataTx[3];    // Holds data to transmit on DIN pin
    uint8_t dataRx[3];    // Placeholder for DOUT received data

    // Put device in "SDATAC" mode to enable register data on DOUT
    if (readContinuousMode) { sendCommand(OPCODE_SDATAC); }

    // Send command
    dataTx[0] = OPCODE_RREG | (startAddress & OPCODE_REG_ADDR_MASK);    // 1st command byte
    dataTx[1] = (count - 1) & OPCODE_REG_COUNT_MASK;                    // 2nd command byte
    spiSendReceive(dataTx, dataRx, 2);

    // Read register data
    dataTx[2] = OPCODE_NOP;                                 // This byte is sent multiple times
    for (i = 0; i < count; i++)
    {
        delay_us(DELAY_T_DLY);                              // tDLY: wait for DOUT to update
        spiSendReceive(&dataTx[2], &dataRx[2], 1);          // Clock out register data
        registerMap[i] = dataRx[2];                         // Update internal register array
    }
}


//*****************************************************************************
//
//! Writes data to a single register.
//!
//! \fn void writeSingleRegister(const uint8_t address, const uint8_t data)
//!
//! \param address is the address of the register to write to.
//! \param data is the value to write.
//!
//! \return None.
//
//*****************************************************************************
void writeSingleRegister(const uint8_t address, const uint8_t data)
{
    // Check that the register address is in range
    assert(address < NUM_REGISTERS);

    uint8_t dataTx[3];   // Holds data to transmit on DIN pin
    uint8_t dataRx[3];   // Placeholder for DOUT received data

    // Send command
    dataTx[0] = OPCODE_WREG | (address & OPCODE_REG_ADDR_MASK); // 1st command byte
    dataTx[1] = 0x00;                                           // 2nd command byte
    spiSendReceive(dataTx, dataRx, 2);

    // Write data
    // NOTE: tDLY is not required between bytes during WREG command
    dataTx[2] = data;
    spiSendReceive(&dataTx[2], &dataRx[2], 1);                  // Clock in register data, RX data is ignored

    // Update internal register array
    // NOTE: This implementation assumes the user is not attempting to modify reserved values.
    registerMap[address] = data;

    // (OPTIONAL) Read back register to verify register write was successful
    // Take note that read-only bits may not be modified!
    //readSingleRegister(address);
}


//*****************************************************************************
//
//! Writes data to a multiple registers starting at the specified address.
//!
//! \fn void writeMultipleRegisters(const uint8_t startAddress, const uint8_t count, const uint8_t dataArray[])
//!
//! \param startAddress is the 8-bit starting address of the first register to begin writing.
//! \param count is the total number of registers to write.
//! \param dataArray[] is the data array containing the new register values,
//! where dataArray[0] is the value to be written to the 'startAddress' register.
//!
//! \return None.
//
//*****************************************************************************
void writeMultipleRegisters(const uint8_t startAddress, const uint8_t count, const uint8_t dataArray[])
{
    // Check that the register address and count are in range
    assert((startAddress + (count - 1)) < NUM_REGISTERS);

    // Check that dataArray is not a NULL pointer
    assert(dataArray);

    uint8_t i;                      // Loop index
    uint8_t dataTx[3];    // Holds data to transmit on DIN pin
    uint8_t dataRx[3];    // Placeholder for DOUT received data

    // Send command
    dataTx[0] = OPCODE_WREG | (startAddress & OPCODE_REG_ADDR_MASK);    // 1st command byte
    dataTx[1] = (count - 1) & OPCODE_REG_COUNT_MASK;                    // 2nd command byte
    spiSendReceive(dataTx, dataRx, 2);

    // Write register data
    // NOTE: tDLY is not required between bytes during WREG command
    for (i = 0; i < count; i++)
    {
        // Clock in register data, RX data is ignored
        spiSendReceive(&dataArray[i], &dataRx[2], 1);

        // Update internal register array
        // NOTE: This implementation assumes the user is not attempting to modify reserved values.
        registerMap[startAddress + i] = dataArray[i];
    }
}


//*****************************************************************************
//
//! Function for sending single byte SPI commands to the ADC
//!
//! \fn uint8_t sendCommand(const uint8_t op_code)
//!
//! \param op_code the command byte/opcode.
//!
//! NOTE: Multi-byte SPI commands have their own dedicated functions.
//!
//! \return None.
//
//*****************************************************************************
void sendCommand(const uint8_t op_code)
{
    // Assert if this function is used to send any of the following commands
    assert(OPCODE_RREG != op_code);    /* Use "readSingleRegister()"  or "readMultipleRegisters()"  */
    assert(OPCODE_WREG != op_code);    /* Use "writeSingleRegister()" or "writeMultipleRegisters()" */

    uint8_t dataTx[1] = { op_code };    // Command byte
    uint8_t dataRx[1];

    // Send command
    spiSendReceive(dataTx, dataRx, 1);  // Returned data is ignored

    // Update internal state variables
    switch (op_code)
    {
        case OPCODE_RESET:
            _restoreRegisterDefaults();
            break;
        case OPCODE_RDATAC:
            readContinuousMode = true;
            break;
        case OPCODE_SDATAC:
            readContinuousMode = false;
            break;
    }
}


//*****************************************************************************
//
//! Function for retrieving ADC conversion results
//!
//! \fn int32_t readData(void)
//!
//! \return 31-bit ADC data (signed-extended to 32-bits).
//!
//! Example of reading data continuously:
//! \code
//!     // Device should be in RDATAC mode
//!     sendCommand(OPCODE_RDATAC);
//!
//!     for (i = 0; i < NUM_SAMPLES; i++)
//!     {
//!         // Wait for nDRDY to go low (or timeout)
//!         waitForDRDYinterrupt(TIMEOUT_uSEC);
//!
//!         // Read data
//!         int32_t data = readData();
//!     }
//! \endcode
//!
//! Example of reading data by command:
//! \code
//!     // Device should be in SDATAC mode
//!     sendCommand(OPCODE_SDATAC);
//!
//!     for (i = 0; i < NUM_SAMPLES; i++)
//!     {
//!         // Send RDATA command
//!         sendCommand(OPCODE_RDATA);
//!
//!         // Wait for nDRDY to go low (or timeout)
//!         waitForDRDYinterrupt(TIMEOUT_uSEC);
//!
//!         // Read data
//!         int32_t data = readData();
//!     }
//! \endcode
//
//*****************************************************************************
int32_t readData(void)
{
    uint8_t dataTx[4] = { 0 };                  // DIN held low
    uint8_t dataRx[4];                          // Placeholder for DOUT received data

    const uint8_t number_bytes_to_read = 4;     // To capture only the 24 most significant
                                                // bits, reduce this value to 3.
    // Read data and sign-extend to 32-bits
    spiSendReceive(dataTx, dataRx, number_bytes_to_read);
    return _signExtendData(dataRx);
}


//*****************************************************************************
//
//! Reverts internal variables to default state after a reset.
//!
//! \warning This function should only be called if the ADS1282 is reset by
//! some external method (other than calling "sendCommand(OPCODE_RESET)" or
//! "adcStartupRoutine()", such as a hardware reset OR power-cycle).
//!
//! \fn void _restoreRegisterDefaults(void)
//!
//! NOTES:
//! - If the MCU keeps a copy of the ADC register settings in memory,
//! then it is important to ensure that these values remain in sync with the
//! actual hardware settings. In order to help facilitate this, this function
//! should be called after powering up or resetting the device (either by
//! toggling the nRESET pin or sending the SPI "RESET" command). Whenever
//! possible, the "ads1282.c" module automatically handles this requirement.
//!
//! \return None.
//
//*****************************************************************************
void _restoreRegisterDefaults(void)
{
    // The ADS1282 defaults to the "SDATAC" (i.e. read by command) state after a reset.
    readContinuousMode = false;

    // Restore default register configurations, except the
    // previously read ID[3:0] field as this value does not change.
    registerMap[ID_ADDRESS]         =   ID_DEFAULT & ID_RESET_MASK;
    registerMap[CONFIG0_ADDRESS]    =   CONFIG0_DEFAULT;
    registerMap[CONFIG1_ADDRESS]    =   CONFIG1_DEFAULT;
    registerMap[HPF0_ADDRESS]       =   HPF0_DEFAULT;
    registerMap[HPF1_ADDRESS]       =   HPF1_DEFAULT;
    registerMap[OFC0_ADDRESS]       =   OFC0_DEFAULT;
    registerMap[OFC1_ADDRESS]       =   OFC1_DEFAULT;
    registerMap[OFC2_ADDRESS]       =   OFC2_DEFAULT;
    registerMap[FSC0_ADDRESS]       =   FSC0_DEFAULT;
    registerMap[FSC1_ADDRESS]       =   FSC1_DEFAULT;
    registerMap[FSC2_ADDRESS]       =   FSC2_DEFAULT;
}


//*****************************************************************************
//
//! Combines ADC data bytes into a single signed 32-bit word
//! To convert from an integer value to a voltage, multiply the return value of
//! this function by the "LSB size" = FSR / 2^31, where:
//!  Full-scale range (FSR) = VREF / PGA
//!  2^31 = total number of ADC output codes
//!
//! \fn int32_t _signExtendData(const uint8_t dataBytes[])
//!
//! \param[in] dataBytes is a pointer to uint8_t[] where the first element is the MSB.
//!
//! \return Returns the signed-extend 32-bit result.
//
//*****************************************************************************
static int32_t _signExtendData(const uint8_t dataBytes[])
{
    int32_t concatenated = ((int32_t) dataBytes[0] << 24);
    concatenated = concatenated | ((int32_t) dataBytes[1] << 16);
    concatenated = concatenated | ((int32_t) dataBytes[2] << 8);
    concatenated = concatenated | ((int32_t) dataBytes[3] << 0);

    if (FILTR_SINC_ONLY)
    {
        // NOTE: In SINC filter mode, 31-bit data is already right-aligned
        // in the 32-bit word and sign-extended by the ADS1282 (output codes
        // will not clip at +/- full-scale in SINC filter mode).
        return concatenated;
    }
    else
    {
        // NOTE: In FIR filter mode 31-bit data is left-aligned in 32-bit word
        // and the LSB is a redundant sign bit that matches the MSB (unless the
        // ADC was over-ranged and the output code is clipped to +/- full-scale).

        // (OPTIONAL) Remove redundant (LSB) sign bit while preserving (MSB) sign.
        // This simplifies the code-to-voltage conversion by providing a similar
        // LSB size for both "SINC only" and "SINC + FIR" filter modes.
        return (concatenated >> 1);
    }
}
