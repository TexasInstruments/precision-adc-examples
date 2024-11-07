/**
 * \copyright Copyright (C) 2019-2020 Texas Instruments Incorporated - http://www.ti.com/
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

#include "ads7066.h"


//****************************************************************************
//
// Internal variables
//
//****************************************************************************

/* Array used to recall device register map configurations */
static uint8_t      registerMap[NUM_REGISTERS];


//****************************************************************************
//
// Internal Function prototypes
//
//****************************************************************************

static void         restoreRegisterDefaults(void);
static int32_t      signExtend(const uint8_t dataBytes[]);


//****************************************************************************
//
// Function Definitions
//
//****************************************************************************

//*****************************************************************************
//
//! Example start up sequence for the ADS7066.
//!
//! \fn void adcStartup(void)
//!
//! Before calling this function, the device must be powered,
//! the SPI/GPIO pins of the MCU must have already been configured,
//! and (if applicable) the external clock source should be provided to CLKIN.
//!
//! \return None.
//
//*****************************************************************************
void initADS7066(void)
{
    // (OPTIONAL) Provide additional delay time for power supply settling
    delay_ms(50);

    // Initialize internal 'registerMap' array with device default settings
    restoreRegisterDefaults();

    // Clear BOR flag
    writeSingleRegister(SYSTEM_STATUS_ADDRESS, SYSTEM_STATUS_BOR_MASK);
}


//*****************************************************************************
//
//! Starts periodic ADC conversions
//!
//! \fn void startConversions(uint32_t samplesPerSecond, uint8_t OSR)
//! \param CHID channel number (0-7) selected as analog input
//! \param samplesPerSecond desired sampling rate (specified as an integer number of SPS)
//! \return None.
//
//*****************************************************************************
void startManualConversions(uint8_t channelID, uint32_t samplesPerSecond)
{
    // Select manual mode
    writeSingleRegister(SEQUENCE_CFG_ADDRESS, SEQUENCE_CFG_SEQ_MODE_MANUAL);

    // Configure pin as analog input
    setChannelAsAnalogInput(channelID);

    // Select channel as MUX input
    writeSingleRegister(CHANNEL_SEL_ADDRESS, channelID);

    // Set nCS pin LOW, next rising edge will trigger start of conversion
    setCS(LOW);

    // Start conversion timer
    startTimer(samplesPerSecond);
}


//*****************************************************************************
//
//! Stops ADC conversions
//!
//! \fn void stopConversions(void)
//!
//! \return None.
//
//*****************************************************************************
void stopConversions(void)
{
    // Stop conversion timer
    stopTimer();

    // Set nCS pin HIGH, allows MCU to communicate with other devices on SPI bus
    setCS(HIGH);
}


//*****************************************************************************
//
//! Reads 4 bytes of ADC data.
//!
//! \fn void readData(uint8_t dataRx[])
//!
//! \param *dataRx points to receive data byte array
//!
//! \return int32_t sign-extended ADC data.
//
//*****************************************************************************
int32_t readData(uint8_t dataRx[])
{
    uint8_t dataTx[4] = { 0 };
    uint8_t numberOfBytes = SPI_CRC_ENABLED ? 4 : 3;

    // NULL command
    dataTx[0] = OPCODE_NULL;
    dataTx[1] = OPCODE_NULL;
    dataTx[2] = OPCODE_NULL;
    if (SPI_CRC_ENABLED) { dataTx[3] = calculateCRC(&dataTx[0], 3, 0xFF); }
    else { dataTx[3] = OPCODE_NULL; }

    // Set the nCS pin LOW
    setCS(LOW);

    // Send all dataTx[] bytes on MOSI, and capture all MISO bytes in dataRx[]
    int i;
    for (i = 0; i < numberOfBytes; ++i)
    {
        dataRx[i] = spiSendReceiveByte(dataTx[i]);
    }

    return signExtend(dataRx);
}


//*****************************************************************************
//
//! Reads the contents of a single register at the specified address.
//!
//! \fn uint8_t readSingleRegister(uint8_t address)
//!
//! \param address is the 8-bit address of the register to read.
//!
//! \return Returns the 8-bit register read result.
//
//*****************************************************************************
uint8_t readSingleRegister(uint8_t address)
{
    uint8_t dataTx[4] = { 0 };
    uint8_t dataRx[4] = { 0 };
    uint8_t numberOfBytes = SPI_CRC_ENABLED ? 4 : 3;
    bool crcOK;

    //
    // Check that the register address is in range
    //
    assert(address < NUM_REGISTERS);

    //
    // [FRAME 1] RREG command
    //
    dataTx[0] = OPCODE_RREG;
    dataTx[1] = address;
    dataTx[2] = OPCODE_NULL;
    if (SPI_CRC_ENABLED) { dataTx[3] = calculateCRC(&dataTx[0], 3, 0xFF); }
    spiSendReceiveArray(dataTx, dataRx, numberOfBytes);

    //
    // [FRAME 2] NULL command
    //
    dataTx[0] = OPCODE_NULL;
    dataTx[1] = OPCODE_NULL;
    dataTx[2] = OPCODE_NULL;
    if (SPI_CRC_ENABLED) { dataTx[3] = calculateCRC(&dataTx[0], 3, 0xFF); }
    spiSendReceiveArray(dataTx, dataRx, numberOfBytes);

    if (SPI_CRC_ENABLED)
    {
        //
        // Calculate CRC for received data and validate it. For register read
        // command CRC will always be at second byte (i.e at dataRx[1])
        //
        crcOK = (dataRx[1] == calculateCRC(dataRx, 1, 0xFF));
        if (crcOK) { registerMap[address] = dataRx[0]; }

        // (OPTIONAL) Consider setting a  flag or returning the value of crcOK
        // to notify the application if a CRC error occurred.
    }

    return registerMap[address];
}


//*****************************************************************************
//
//! Getter function to access registerMap array from outside of this module.
//!
//! \fn uint8_t getRegisterValue(uint8_t address)
//!
//! NOTE: The internal registerMap arrays stores the last know register value,
//! since the last read or write operation to that register. This function
//! does not communicate with the device to retrieve the current register value.
//! For the most up-to-date register data or retrieving the value of a hardware
//! controlled register, it is recommend to use readSingleRegister() to read the
//! current register value.
//!
//! \return unsigned 8-bit register value.
//
//*****************************************************************************

uint8_t getRegisterValue(uint8_t address)
{
    assert(address < NUM_REGISTERS);
    return registerMap[address];
}


//*****************************************************************************
//
//! Writes data to a single register and also read it back for internal confirmation
//!
//! \fn void writeSingleRegister(uint8_t address, uint8_t data)
//!
//! \param address is the address of the register to write to.
//! \param data is the value to write.
//!
//! \return None.
//
//*****************************************************************************
void writeSingleRegister(uint8_t address, uint8_t data)
{
    uint8_t dataTx[4] = { 0 };
    uint8_t dataRx[4] = { 0 };
    uint8_t numberOfBytes = SPI_CRC_ENABLED ? 4 : 3;

    //
    // Check that the register address is in range
    //
    assert(address < NUM_REGISTERS);

    //
    // (OPTIONAL) Read STATUS register to check if we can perform a register write.
    //
    if (SPI_CRC_ENABLED) { readSingleRegister(SYSTEM_STATUS_ADDRESS); }

    //
    // If CRC error has occurred, write is allowed only for registers SYSTEM_STATUS and GENERAL_CFG
    //
    if (SPI_CRC_ERROR && ((address != SYSTEM_STATUS_ADDRESS) && (address != GENERAL_CFG_ADDRESS)))
    {
       // (OPTIONAL) Consider clearing the CRC error here or else make sure the application handles
       // this error elsewhere
       return;
    }

    //
    // WREG command
    //
    dataTx[0] = OPCODE_WREG;
    dataTx[1] = address;
    dataTx[2] = data;
    if (SPI_CRC_ENABLED) { dataTx[3] = calculateCRC(&dataTx[0], 3, 0xFF); }
    spiSendReceiveArray(dataTx, dataRx, numberOfBytes);

    //
    // (RECOMMENDED) Read back register to confirm register write was successful
    //
    //registerMap[address] = data;
    readSingleRegister(address);

    //
    // (OPTIONAL) Check if a CRC error occurred. Consider handling it here.
    //
    if (SPI_CRC_ENABLED) { readSingleRegister(SYSTEM_STATUS_ADDRESS); }
}


//*****************************************************************************
//
//!
//! \fn void setRegisterBits(uint8_t address, uint8_t bitMask)
//!
//! \param address is the address of the register to write to.
//! \param bitMask indicates which bits in the register to set.
//!
//! \return None.
//
//*****************************************************************************
void setRegisterBits(uint8_t address, uint8_t bitMask)
{
    uint8_t dataTx[4] = { 0 };
    uint8_t dataRx[4] = { 0 };
    uint8_t numberOfBytes = SPI_CRC_ENABLED ? 4 : 3;

    // Check that the register address is in range
    assert(address < NUM_REGISTERS);

    // SETBIT command
    dataTx[0] = OPCODE_SETBIT;
    dataTx[1] = address;
    dataTx[2] = bitMask;
    if (SPI_CRC_ENABLED) { dataTx[3] = calculateCRC(&dataTx[0], 3, 0xFF); }
    spiSendReceiveArray(dataTx, dataRx, numberOfBytes);

    // (RECOMMENDED) Read back register to confirm register write was successful
    //registerMap[address] = data;
    readSingleRegister(address);
}


//*****************************************************************************
//
//!
//! \fn void clearRegisterBits(uint8_t address, uint8_t bitMask)
//!
//! \param address is the address of the register to write to.
//! \param bitMask indicates which bits in the register to clear.
//!
//! \return None.
//
//*****************************************************************************
void clearRegisterBits(uint8_t address, uint8_t bitMask)
{
    uint8_t dataTx[4] = { 0 };
    uint8_t dataRx[4] = { 0 };
    uint8_t numberOfBytes = SPI_CRC_ENABLED ? 4 : 3;

    // Check that the register address is in range
    assert(address < NUM_REGISTERS);

    // SETBIT command
    dataTx[0] = OPCODE_CLRBIT;
    dataTx[1] = address;
    dataTx[2] = bitMask;
    if (SPI_CRC_ENABLED) { dataTx[3] = calculateCRC(&dataTx[0], 3, 0xFF); }
    spiSendReceiveArray(dataTx, dataRx, numberOfBytes);

    // (RECOMMENDED) Read back register to confirm register write was successful
    //registerMap[address] = data;
    readSingleRegister(address);

    // (OPTIONAL) Check if a CRC error occurred. Consider handling it here.
}


//*****************************************************************************
//
//! Configure the selected channel as an analog Input
//!
//! \fn void setChannelAsAnalogInput(uint8_t channelID)
//!
//! \param ChannelID is the channel number.
//!
//! \return None
//
//*****************************************************************************
void setChannelAsAnalogInput(uint8_t channelID)
{
    // Check that channel ID is in range.
    assert(channelID < 8);

    // Clear the corresponding channel bit to set the channel  as analog input
    clearRegisterBits(PIN_CFG_ADDRESS, (1 << channelID));
}


//*****************************************************************************
//
//! Calculates the 8-bit CRC for the selected CRC polynomial.
//!
//! \fn uint8_t calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint8_t initialValue)
//!
//! \param dataBytes[] pointer to first element in the data byte array
//! \param numberBytes number of bytes to be used in CRC calculation
//! \param initialValue the seed value (or partial crc calculation), use 0xFF when beginning a new CRC computation
//!
//! NOTE: This calculation is shown as an example and is not optimized for speed.
//!
//! \return 8-bit calculated CRC word
//
//*****************************************************************************
uint8_t calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint8_t initialValue)
{
    int bitIndex, byteIndex;
    bool dataMSb, crcMSb;

    //
    // Check that "dataBytes" is not a null pointer
    //
    assert(dataBytes != 0x00);

    //
    // Initial value of crc register
    // NOTE: The ADS7066 defaults to 0xFF,
    // but can be set at function call to continue an on-going calculation
    //
    uint8_t crc = initialValue;

    //
    // ANSI CRC polynomial = x^8 + x^2 + x^1 + 1
    //
    const uint8_t poly = 0x07;

    //
    // Loop through all bytes in the dataBytes[] array
    //
    for (byteIndex = 0; byteIndex < numberBytes; byteIndex++)
    {
        //
        // Loop through all bits in the current byte
        //
        bitIndex = 0x80u;
        while (bitIndex > 0)
        {
            // Get the MSB's of data and crc
            dataMSb = (bool) (dataBytes[byteIndex] & bitIndex);
            crcMSb  = (bool) (crc & 0x80u);

            // Left-shift crc register
            crc <<= 1;

            // XOR crc register with polynomial?
            if (dataMSb ^ crcMSb) { crc ^= poly; }

            // Update MSB pointer to the next data bit
            bitIndex >>= 1;
        }
    }

    return crc;
}


//*****************************************************************************
//
//! Updates the registerMap[] array to its default values.
//!
//! \fn static void restoreRegisterDefaults(void)
//!
//! NOTES:
//! - If the MCU keeps a copy of the ADC register settings in memory,
//! then it is important to ensure that these values remain in sync with the
//! actual hardware settings. In order to help facilitate this, this function
//! should be called after powering up or resetting the device (either by
//! hardware pin control or SPI software command).
//!
//! - Reading back all of the registers after resetting the device can
//! accomplish the same result; however, this might be problematic if the
//! device was previously in CRC mode , since resetting the device exits
//! these modes. If the MCU is not aware of this mode change, then read
//! register commands will fail.
//!
//! \return None.
//
//*****************************************************************************
static void restoreRegisterDefaults(void)
{
    registerMap[SYSTEM_STATUS_ADDRESS]          =   SYSTEM_STATUS_DEFAULT;    /* NOTE: This a read-only register */
    registerMap[GENERAL_CFG_ADDRESS]            =   GENERAL_CFG_DEFAULT;
    registerMap[DATA_CFG_ADDRESS]               =   DATA_CFG_DEFAULT;
    registerMap[OSR_CFG_ADDRESS]                =   OSR_CFG_DEFAULT;
    registerMap[OPMODE_CFG_ADDRESS]             =   OPMODE_CFG_DEFAULT;
    registerMap[PIN_CFG_ADDRESS]                =   PIN_CFG_DEFAULT;
    registerMap[GPIO_CFG_ADDRESS]               =   GPIO_CFG_DEFAULT;
    registerMap[GPO_DRIVE_CFG_ADDRESS]          =   GPO_DRIVE_CFG_DEFAULT;
    registerMap[GPO_OUTPUT_VALUE_ADDRESS]       =   GPO_OUTPUT_VALUE_DEFAULT;
    registerMap[GPI_VALUE_ADDRESS]              =   GPI_VALUE_DEFAULT;
    registerMap[SEQUENCE_CFG_ADDRESS]           =   SEQUENCE_CFG_DEFAULT;
    registerMap[CHANNEL_SEL_ADDRESS]            =   CHANNEL_SEL_DEFAULT;
    registerMap[AUTO_SEQ_CHSEL_ADDRESS]         =   AUTO_SEQ_CHSEL_DEFAULT;
    registerMap[DIAGNOSTICS_KEY_ADDRESS]        =   DIAGNOSTICS_KEY_DEFAULT;
    registerMap[BIT_WALK_ADDRESS]               =   BIT_WALK_DEFAULT;
    registerMap[BIT_SAMPLE_LSB_ADDRESS]         =   BIT_SAMPLE_LSB_DEFAULT;
    registerMap[BIT_SAMPLE_MSB_ADDRESS]         =   BIT_SAMPLE_MSB_DEFAULT;
}


//****************************************************************************
//
// Helper functions
//
//****************************************************************************


//*****************************************************************************
//
//! Internal function used by readData() to convert ADC data from multiple unsigned
//! bytes into a single signed 32-bit word.
//!
//! \fn int32_t signExtend(const uint8_t dataBytes[])
//!
//! \param dataBytes is a pointer to uint8_t[] where the first element is the MSB.
//!
//! \return Returns the signed-extend 32-bit result.
//
//*****************************************************************************
static int32_t signExtend(const uint8_t dataBytes[])
{
    uint8_t shiftDistance = AVERAGING_ENABLED ? 12 : 16;

    int32_t upperByte   = ((int32_t) dataBytes[0] << 24);
    int32_t middleByte  = ((int32_t) dataBytes[1] << 16);
    int32_t lowerByte   = ((int32_t) dataBytes[2] << 8);

    // NOTE: This right-shift operation on signed data maintains the signed bit,
    // and provides for the sign-extension to to 32 bits.
    return (((int32_t) (upperByte | middleByte | lowerByte)) >> shiftDistance);
}
