/*
 * \copyright Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
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

#include "ads127l21.h"


//****************************************************************************
//
// Macros
//
//****************************************************************************
#define UPPER_BYTE(x)           ((uint8_t)(((x) >> 8) & 0x00FFu))
#define LOWER_BYTE(x)           ((uint8_t)(((x) & 0x00FFu)))


//****************************************************************************
//
// Internal variables & functions
//
//****************************************************************************

// Shadow register array used to recall device configurations
static uint8_t registerMap[NUM_REGISTERS];

// Arrays used to send and receive data over SPI
static uint8_t dataTx[SPI_BUFFER_SIZE];
static uint8_t dataRx[SPI_BUFFER_SIZE];

// Helper functions
static uint8_t buildSPIarray(uint8_t byte1, uint8_t byte2);
static void sendNOPcommand(void);
static void sendRREGcommand(uint8_t address);
static void sendWREGcommand(uint8_t address, uint8_t data);
static void writeCoefficientBank(int8_t address, int32_t coeffs[], uint8_t coeff_length);


//*****************************************************************************
//
//! Getter function for accessing (cached) shadow register values.
//!
//! \fn uint8_t getRegisterValue(uint8_t address)
//!
//! NOTE: The internal registerMap arrays stores the last know register value,
//! since the last read or write operation to that register. This function
//! does not communicate with the device to retrieve the current register value.
//! For the most up-to-date register data or retrieving the value of a hardware
//! controlled register, it is recommend to use readSingleRegister() to acquire
//! the current register value.
//!
//! \return unsigned 8-bit register value.
//
//*****************************************************************************
uint8_t getRegisterValue(uint8_t address)
{
    assert(address < NUM_REGISTERS);
    assert((address != FIR_BANK_ADDRESS) && (address != IIR_BANK_ADDRESS));
    return registerMap[address];
}


//*****************************************************************************
//
//! Example ADC start up sequence
//!
//! \fn void adcStartup(void)
//!
//! Before calling this function, the device must be powered,
//! the SPI/GPIO pins of the MCU must have already been configured,
//! and (if applicable) an external clock source provided.
//
//*****************************************************************************
void adcStartup(void)
{
    // Initial CRC module
    initCRC8(CRC8_POLYNOMIAL);

    // (OPTIONAL) Provide additional delay time for power supply settling
    delay_ms(50);

    // (REQUIRED) Set nRESET pin high for ADC operation
    setRESET(HIGH);

    // (REQUIRED) Set START pin high to begin conversions
    setSTART(HIGH);

    // (OPTIONAL) Toggle nRESET pin to ensure default register settings
    toggleRESET();

    // (REQUIRED) Initialize shadow register array with device defaults
    restoreRegisterDefaults();

    // (OPTIONAL) Read Revision ID
    readSingleRegister(REV_ID_ADDRESS);

    // (OPTIONAL) Clears any error flags triggered during power-up (e.g. POR_FLAG)
    clearSTATUSflags();

    // (OPTIONAL) Define your initial register settings here
    // Example: Enable input & reference buffers and VCM output
    uint8_t regVal = CONFIG1_VCM_MASK | CONFIG1_REFP_BUF_MASK | CONFIG1_AINP_BUF_MASK | CONFIG1_AINN_BUF_MASK;
    writeSingleRegister(CONFIG1_ADDRESS, regVal);

    // (OPTIONAL) Enable main memory map CRC
    enableRegisterMapCrc(true);
}


//*****************************************************************************
//
//! Clears STATUS1 and STATUS2 register error flags
//!
//! \fn void clearSTATUSflags(void)
//
//*****************************************************************************
void clearSTATUSflags(void)
{
    // Clear ALV_FLAG, POR_FLAG, and SPI_ERR bits (write 1 to clear)
    writeSingleRegister(STATUS1_ADDRESS, STATUS1_CLEAR_ERRORS);

    // Clear M_CRC_ERR bit (write 1 to clear)
    writeSingleRegister(STATUS2_ADDRESS, STATUS2_M_CRC_ERR_MASK);
}


//*****************************************************************************
//
//! Writes data to a single register
//!
//! \fn void writeSingleRegister(uint8_t address, uint8_t data)
//! \param address is the address of the register to write to
//! \param data is the value to write
//!
//! NOTES:
//! 1) Recommended to call clearSTATUSflags() first to ensure WREG commands are not blocked
//! 2) Call this function in a loop to write to multiple registers
//
//*****************************************************************************
void writeSingleRegister(uint8_t address, uint8_t data)
{
    /* Check that the register address is in range */
    assert(address < NUM_REGISTERS);

    // [FRAME 1] Send WREG command
    sendWREGcommand(address, data);

    // Update shadow register
    registerMap[address] = data;

    // Check if write operation will result in a reset
    // and restore defaults to ensure register map stays in sync
    if ((address == CONTROL_ADDRESS) & (data == CONTROL_RESET_MASK))
    {
        restoreRegisterDefaults();
        delay_ms(1);    // tSRLRST delay
    }
}


//*****************************************************************************
//
//! Reads the contents of a single register at the specified address
//!
//! \fn uint8_t readSingleRegister(uint8_t address)
//! \param address is the 8-bit address of the register to read
//! \return Returns the 8-bit register read result
//
//*****************************************************************************
uint8_t readSingleRegister(uint8_t address)
{
    /* Check that the register address is in range */
    assert(address < NUM_REGISTERS);

    // [FRAME 1] Send RREG command
    sendRREGcommand(address);

    // [FRAME 2] Send NOP command to retrieve the register data
    sendNOPcommand();

    // Parse dataRx for register value
    uint8_t registerValue = dataRx[(STATUS_ENABLED ? 1 : 0)];

    // Update shadow register
    registerMap[address] = registerValue;

    return registerValue;
}


//*****************************************************************************
//
//! Reads the contents of multiple registers
//!
//! \fn void readMultipleRegisters(uint8_t startAddress, uint8_t count)
//! \param startAddress is the 8-bit address of the starting register to read
//! \param count is the total number of registers to read (between 1 and 32)
//!
//! NOTES:
//! 1) This function does NOT return register data. Use getRegisterValue()
//!     to access the updated register data!

//! 2) This function is not suitable for reading the FIR_BANK and IIR_BANK
//!     registers since it increments the register address after each read.
//
//*****************************************************************************
void readMultipleRegisters(uint8_t startAddress, uint8_t count)
{
    assert(count > 0);  // Must read at least one register
    assert((startAddress + count) <= NUM_REGISTERS); // Avoid out-of-range address

    // Loop through registers
    uint8_t i;
    for (i = 0; i <= count; i++)
    {
        // [FRAME 0 to count-1] Send WREG command
        if (i < count) { sendRREGcommand(startAddress + i); }
        else { sendNOPcommand(); }  // [LAST FRAME] Send NOP instead

        // Update shadow register
        registerMap[i + startAddress] = dataRx[(STATUS_ENABLED ? 1 : 0)];
    }
}


//*****************************************************************************
//
//! Reads ADC data and returns signed-extended value
//!
//! \fn int32_t readData(adc_channel_t *dataStruct)
//!
//! NOTES:
//! 1) Provide a adc_channel_t pointer To retrieve STATUS and CRC bytes, otherwise
//!     provide a NULL pointer to only retrieve ADC data
//!
//! 2) This function can be used to read 24-bit or 16-bit data; however, if you
//!     use 16-bit data mode consider creating a modified version of this function
//!     that returns a int16_t instead of a int32_t
//
//*****************************************************************************
int32_t readData(adc_channel_t *dataStruct)
{
    // [FRAME 1] Send NOP command
    sendNOPcommand();

    // Parse return data
    int32_t data = signExtend(&dataRx[(STATUS_ENABLED ? 1 : 0)]);

    if (dataStruct)
    {
        uint8_t i = 0;
        if (STATUS_ENABLED) { dataStruct->status = dataRx[i++]; }
        dataStruct->data = data;
        if (SPI_CRC_ENABLED)
        {
            i += (RESOLUTION_IS_16_BIT ? 2 : 3);
            dataStruct->crc = dataRx[i];
        }
    }

    return data;
}


//*****************************************************************************
//
//! Resets the device by writing to the CONTROL register
//!
//! \fn void resetDeviceByCommand(void)
//!
//! NOTES:
//! 1) Recommended to call clearSTATUSflags() first to ensure WREG commands are not blocked     // TODO: verify!
//
//*****************************************************************************
void resetDeviceByCommand(void)
{
    // Send command
    writeSingleRegister(CONTROL_ADDRESS, CONTROL_RESET_COMMAND);

    // Update register setting array to keep software in sync with device
    restoreRegisterDefaults();

    // tSRLRST delay
    delay_ms(1);
}


//*****************************************************************************
//
//! Resets the device by SPI input pattern
//!
//! \fn void resetDeviceByPattern(void)
//
//*****************************************************************************
void resetDeviceByPattern(void)
{
    int i;

    if (SPI_3_WIRE_MODE)
    {
        // (PATTERN 1) Send at least 1023 ones followed by a zero (3-wire or 4-wire SPI mode)
        for (i = 0; i < 127; i++) { spiSendReceiveByte(0xFF); }
        spiSendReceiveByte(0xFE);
    }
    else
    {
        // (PATTERN 2) Send at least 1024 ones followed by taking /CS HIGH (4-wire SPI or daisy-chain mode)
        setCS(LOW);
        for (i = 0; i < 128; i++) { spiSendReceiveByte(0xFF); }
        setCS(HIGH);
    }

    // Update register setting array to keep software in sync with device
    restoreRegisterDefaults();

    // tSRLRST delay
    delay_ms(1);
}


//*****************************************************************************
//
//! Resets SPI frame in 3-wire mode
//!
//! \fn void resetSPIframe(void)
//
//*****************************************************************************
void resetSPIframe(void)
{
    if (SPI_3_WIRE_MODE)
    {
        // (PATTERN) Send at least 63 ones followed by a zero (3-wire SPI mode)
        int i;
        for (i = 0; i < 7; i++) { spiSendReceiveByte(0xFF); }
        spiSendReceiveByte(0xFE);
    }
    else { setCS(HIGH); }
}


//*****************************************************************************
//
//! Checks if CRC-OUT is valid
//!
//! \fn bool validateCrcOut(void)
//! \return Returns true if CRC-OUT is valid or if SPI_CRC mode is disabled
//!
//! NOTES:
//! 1) This function is intended to be called after performing a SPI command
//!     when SPI_CRC is enabled.
//! 2) When including the CRC-OUT byte in the CRC calculation, a results of
//!     zero indicates that the
//
//*****************************************************************************
bool isValidCrcOut(void)
{
    bool crc_valid = true;
    if (SPI_CRC_ENABLED)
    {
        uint8_t crcIndex = (STATUS_ENABLED ? 1 : 0) + (RESOLUTION_IS_16_BIT ? 2 : 3);

        // (OPTION 1) Comparing bytes
        //crc_valid = (dataTx[crcIndex] == getCRC8(dataTx, crcIndex, CRC8_INITIAL_SEED));

        // (OPTION 2) Including CRC byte
        crc_valid = !(bool) getCRC8(dataTx, crcIndex + 1, CRC8_INITIAL_SEED);
    }
    return crc_valid;
}


//*****************************************************************************
//
//! Updates shadow register array to device's default values.
//!
//! \fn void restoreRegisterDefaults(void)
//!
//! NOTES:
//! - If the MCU keeps a copy of the ADC register settings in memory, then
//! it is important to ensure that these values remain in sync with the actual
//! hardware settings. In order to help facilitate this, this function should
//! be called after powering up or resetting the device (either by hardware
//! pin control or SPI software command).
//!
//! - Reading back all of the registers after resetting the device can
//! accomplish the same result; however, this might be problematic if the
//! device was previously in CRC mode or STATUS output mode, since
//! resetting the device exits these modes. If the MCU is not aware of this
//! mode change, then read register commands will return invalid data due to
//! the expectation of data appearing in a different byte position.
//
//*****************************************************************************
void restoreRegisterDefaults(void)
{
    registerMap[DEV_ID_ADDRESS]             =   DEV_ID_DEFAULT;
    //registerMap[REV_ID_ADDRESS]           =   0x00;               // Read to update
    registerMap[STATUS1_ADDRESS]            =   STATUS1_DEFAULT;    // Contains hardware controlled bits
    registerMap[STATUS2_ADDRESS]            =   STATUS2_DEFAULT;    // Contains hardware controlled bits
    registerMap[CONTROL_ADDRESS]            =   CONTROL_DEFAULT;
    registerMap[MUX_ADDRESS]                =   MUX_DEFAULT;
    registerMap[CONFIG1_ADDRESS]            =   CONFIG1_DEFAULT;
    registerMap[CONFIG2_ADDRESS]            =   CONFIG2_DEFAULT;
    registerMap[CONFIG3_ADDRESS]            =   CONFIG3_DEFAULT;
    registerMap[FILTER1_ADDRESS]            =   FILTER1_DEFAULT;
    registerMap[FILTER2_ADDRESS]            =   FILTER2_DEFAULT;
    registerMap[FILTER3_ADDRESS]            =   FILTER3_DEFAULT;
    registerMap[OFFSET2_ADDRESS]            =   OFFSET2_DEFAULT;
    registerMap[OFFSET1_ADDRESS]            =   OFFSET1_DEFAULT;
    registerMap[OFFSET0_ADDRESS]            =   OFFSET0_DEFAULT;
    registerMap[GAIN2_ADDRESS]              =   GAIN2_DEFAULT;
    registerMap[GAIN1_ADDRESS]              =   GAIN1_DEFAULT;
    registerMap[GAIN0_ADDRESS]              =   GAIN0_DEFAULT;
    registerMap[MAIN_CRC_ADDRESS]           =   MAIN_CRC_DEFAULT;
    //registerMap[FIR_BANK_ADDRESS]         =   0x00;   // Reset value undefined
    //registerMap[FIR_CRC1_ADDRESS]         =   0x00;   // Reset value undefined
    //registerMap[FIR_CRC0_ADDRESS]         =   0x00;   // Reset value undefined
    //registerMap[IIR_BANK_ADDRESS]         =   0x00;   // Reset value undefined
    //registerMap[IIR_CRC_ADDRESS]          =   0x00;   // Reset value undefined
}


//*****************************************************************************
//
//! Enable/Disable register map CRC
//!
//! \fn void enableRegisterMapCrc(bool enable)
//!
//! NOTES:
//! 1) Recommended to call clearSTATUSflags() first to ensure WREG commands are not blocked
//
//*****************************************************************************
void enableRegisterMapCrc(bool enable)
{
    // Disable the REG_CRC bit
    readSingleRegister(CONFIG3_ADDRESS);    // update shadow register
    writeSingleRegister(CONFIG3_ADDRESS, registerMap[CONFIG3_ADDRESS] & ~CONFIG3_REG_CRC_MASK);

    if (!enable) { return; }

    // Compute and update MAIN_CRC
    uint8_t crc = CRC8_INITIAL_SEED;
    crc = getCRC8(&registerMap[DEV_ID_ADDRESS], 2, crc);    // registers 0h and 1h
    crc = getCRC8(&registerMap[MUX_ADDRESS], 13, crc);      // registers 5h to 11h (skip 2h to 4h)
    writeSingleRegister(MAIN_CRC_ADDRESS, crc);

    // Enable the REG_CRC bit
    writeSingleRegister(CONFIG3_ADDRESS, registerMap[CONFIG3_ADDRESS] | CONFIG3_REG_CRC_MASK);
}


//*****************************************************************************
//
//! Updates the FIR coefficients.
//! User must synchronize the digital filter after changing the coefficients
//!
//! \fn void setFIRcoeffs(int32_t fir_coeffs[], uint8_t fir_coeff_length, uint16_t fir_coeff_crc)
//! \param crc is the expected 16-bit crc word.
//! \param filter_coeff[] is the array of new values for the FIR filter
//!
//! NOTES:
//! 1) Recommended to call clearSTATUSflags() first to ensure WREG commands are not blocked
//! 2) It is recommended to restart ADC conversions after updating filter coefficients
//
//*****************************************************************************
void setFIRcoeffs(int32_t fir_coeffs[], uint8_t fir_coeff_length, uint16_t fir_coeff_crc)
{
    // Disable the REG_CRC bit
    readSingleRegister(CONFIG3_ADDRESS);    // update shadow register
    writeSingleRegister(CONFIG3_ADDRESS, registerMap[CONFIG3_ADDRESS] & ~CONFIG3_REG_CRC_MASK);

    // TODO: Handle zero padding if fir_coeff_length < 128...
    assert(fir_coeff_length == 128);

    // Write coefficients to FIR bank
    writeCoefficientBank(FIR_BANK_ADDRESS, fir_coeffs, fir_coeff_length);

    // Update FIR_CRC1 and FIR_CRC0 registers
    writeSingleRegister(FIR_CRC1_ADDRESS, UPPER_BYTE(fir_coeff_crc));
    writeSingleRegister(FIR_CRC0_ADDRESS, LOWER_BYTE(fir_coeff_crc));

    // Enable the REG_CRC bit
    writeSingleRegister(CONFIG3_ADDRESS, registerMap[CONFIG3_ADDRESS] | CONFIG3_REG_CRC_MASK);
}


//*****************************************************************************
//
//! Updates the IIR coefficients.
//! User must synchronize the digital filter after changing the coefficients
//!
//! \fn void setIIRcoeffs(int32_t iir_coeffs[], uint8_t iir_coeff_length, uint8_t iir_coeff_crc)
//! \param crc is the expected 16-bit crc word.
//! \param filter_coeff[] is the array of new values for the IIR filter
//!     This variable MUST have 25 elements. Due to the nature of the IIR,
//!     the user must select all coeff. Example is provided in iir_coeff.h.
//!
//! NOTES:
//! 1) Recommended to call clearSTATUSflags() first to ensure WREG commands are not blocked
//! 2) It is recommended to restart ADC conversions after updating filter coefficients
//
//*****************************************************************************
void setIIRcoeffs(int32_t iir_coeffs[], uint8_t iir_coeff_length, uint8_t iir_coeff_crc)
{
    // Disable the REG_CRC bit
    readSingleRegister(CONFIG3_ADDRESS);    // update shadow register
    writeSingleRegister(CONFIG3_ADDRESS, registerMap[CONFIG3_ADDRESS] & ~CONFIG3_REG_CRC_MASK);

    // TODO: Handle zero padding if iir_coeff_length < 25...
    assert(iir_coeff_length == 25);

    // Write coefficients to IIR bank
    writeCoefficientBank(IIR_BANK_ADDRESS, iir_coeffs, iir_coeff_length);

    // Update IIR_CRC register
    writeSingleRegister(IIR_CRC_ADDRESS, iir_coeff_crc);

    // Enable the REG_CRC bit
    writeSingleRegister(CONFIG3_ADDRESS, registerMap[CONFIG3_ADDRESS] | CONFIG3_REG_CRC_MASK);
}


//*****************************************************************************
//
//! Combines ADC data bytes into a single signed 32-bit word.
//!
//! \fn int32_t combineDataBytes(const uint8_t dataBytes[])
//! \param dataBytes pointer to uint8_t[] where the first byte is the MSB
//! \return Returns the signed-extend 32-bit result.
//
//*****************************************************************************
int32_t signExtend(const uint8_t dataBytes[])
{
    int32_t upperByte, middleByte=0, lowerByte, shift;

    if (RESOLUTION_IS_16_BIT)
    {
        upperByte   = (int32_t) dataBytes[0] << 24;
        lowerByte   = (int32_t) dataBytes[1] << 16;
        shift       = 16;
    }
    else
    {
        upperByte   = (int32_t) dataBytes[0] << 24;
        middleByte  = (int32_t) dataBytes[1] << 16;
        lowerByte   = (int32_t) dataBytes[2] << 8;
        shift       = 8;
    }

    // Right-shift of signed data maintains sign bit
    return (upperByte | middleByte | lowerByte) >> shift;
}


//****************************************************************************
//
// Helper functions
//
//****************************************************************************


//*****************************************************************************
//
//! Helper function for inserting SPI command bytes into dataTx array
//!
//! \fn uint8_t buildSPIarray(uint8_t byte1, uint8_t byte2)
//! \param byte1 Command byte 1 (OPCODE_NOP, OPCODE_RREG, OPCODE_WREG)
//! \param byte2 Command byte 2 (arbitrary value or don't care)
//! \return number of bytes added to byteArray[] (expected values: 2, 3, 4, 5)
//
//*****************************************************************************
static uint8_t buildSPIarray(uint8_t byte1, uint8_t byte2)
{
    uint8_t i = 0;  // byte index

    if (STATUS_ENABLED) { dataTx[i++] = 0x00; }         // don't care
    if (!RESOLUTION_IS_16_BIT) { dataTx[i++] = 0x00; }   // don't care
    dataTx[i++] = byte1;    // command byte 1
    dataTx[i++] = byte2;    // command byte 2
    if (SPI_CRC_ENABLED)
    {
        // Compute CRC-IN, ignoring 'don't care' bytes
        dataTx[i++] = getCRC8(&dataTx[i - 2], 2, CRC8_INITIAL_SEED);
    }
    return i;
}


//*****************************************************************************
//
//! Sends the ADC's NOP command
//!
//! \fn static void sendNOPcommand(void)
//!
//! NOTES:
//! 1) This function doesn't perform any data validation checks. The calling
//! function is expected to handle this.
//! 2) The calling function is expected to provide the appropriate word count
//! for clocking out sufficient data words or avoiding a SCLK_COUNT_FAULTn flag.
//
//*****************************************************************************
static void sendNOPcommand(void)
{
    // Build SPI TX array
    const uint8_t byteCount = buildSPIarray(OPCODE_NOP, OPCODE_NOP);

    // [FRAME 1] Send NOP command
    spiSendReceiveArrays(dataTx, dataRx, byteCount);

    // NOTE: No dataRx[] validation is performed here so that calling functions
    // can have full control over what types of validation tests to perform.
}


//*****************************************************************************
//
//! Sends RREG command (low-level)
//!
//! \fn static void sendRREGcommand(uint8_t address)
//! \param address 8-bit register address to read
//
//*****************************************************************************
static void sendRREGcommand(uint8_t address)
{
    // Build SPI TX array
    const uint8_t command = OPCODE_RREG | address;
    const uint8_t arbitrary = 0x00;
    uint8_t numberOfBytes = buildSPIarray(command, arbitrary);

    // [FRAME 1] Send RREG command
    spiSendReceiveArrays(dataTx, dataRx, numberOfBytes);
}


//*****************************************************************************
//
//! Sends WREG command (low-level)
//!
//! \fn static void sendWREGcommand(uint8_t address, uint8_t data)
//!
//! \param address is the address of the register to write to.
//! \param data is the value to write.
//!
//! IMPORTANT: The SPI_ERR bit must be cleared (if set) otherwise
//! register write operations will be blocked by the device.
//
//*****************************************************************************
static void sendWREGcommand(uint8_t address, uint8_t data)
{
    // Build SPI TX array
    const uint8_t command = OPCODE_WREG | address;
    const uint8_t numberOfBytes = buildSPIarray(command, data);

    // [FRAME 1] Send WREG command
    spiSendReceiveArrays(dataTx, dataRx, numberOfBytes);
}


//*****************************************************************************
//
//! Write multiple 32-bit FIR or IIR coefficients to register bank
//!
//! \fn static void writeCoefficientBank(int8_t address, int32_t coeffs[], uint8_t coeff_length)
//! \param address register address (FIR_BANK or IIR_BANK)
//! \param coeffs pointer to coefficient array
//! \param coeff_length number of elements in coefficient array
//
//*****************************************************************************
static void writeCoefficientBank(int8_t address, int32_t coeffs[], uint8_t coeff_length)
{
    assert((address == FIR_BANK_ADDRESS) || (address == IIR_BANK_ADDRESS));

    // write coefficients to register bank
    uint8_t i, bytes[4];
    for(i = 0; i < coeff_length; i++)
    {
        bytes[0] = (coeffs[i] >> 24) & 0xFF;
        bytes[1] = (coeffs[i] >> 16) & 0xFF;
        bytes[2] = (coeffs[i] >> 8) & 0xFF;
        bytes[3] = coeffs[i] & 0xFF;

        sendWREGcommand(address, bytes[3]);
        sendWREGcommand(address, bytes[2]);
        sendWREGcommand(address, bytes[1]);
        sendWREGcommand(address, bytes[0]);
    }
}
