/**
 * \copyright Copyright (C) 2019-2021 Texas Instruments Incorporated - http://www.ti.com/
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

#include "ads7038.h"


//****************************************************************************
//
// Internal variables
//
//****************************************************************************

/** Array used to recall device register map configurations */
static uint8_t      registerMap[MAX_REGISTER_ADDRESS + 1];


//****************************************************************************
//
// Internal Function prototypes
//
//****************************************************************************
static void         restoreRegisterDefaults(void);
static int16_t      signExtend(const uint8_t dataBytes[]);


//****************************************************************************
//
// Function Definitions
//
//****************************************************************************

//*****************************************************************************
//
//! \brief Example start up sequence for the ADS7038.
//!
//! \fn void initADS7038(void)
//!
//! Before calling this function, the device must be powered,
//! the SPI pins of the MCU must have already been configured.
//!
//! \return None.
//
//*****************************************************************************
void initADS7038(void)
{
    // (OPTIONAL) Provide additional delay time for power supply settling
    delay_ms(50);

    // Reset device
    resetDevice();

    // Clear BOR flag
    setRegisterBits(SYSTEM_STATUS_ADDRESS, SYSTEM_STATUS_BOR_MASK);

    // (OPTIONAL) Configure initial register settings here

    // (RECOMMENDED) If you plan to modify the CRC_EN or CPOL_CPHA bits,
    // do so here (and only here) to be simplify the code implementation.

    // (OPTIONAL) Read back registers and check STATUS register for faults

}


//*****************************************************************************
//
//! \brief  Resets the device and reinitializes the register map array
//!         maintained in firmware to default values.
//!
//! \fn     void resetDevice()
//!
//! \return None
//
//*****************************************************************************
void resetDevice()
{
    // Set the RST bit high to reset the device
    setRegisterBits(GENERAL_CFG_ADDRESS, GENERAL_CFG_RST_MASK);

    // Update internal register map array
    restoreRegisterDefaults();
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
//! \brief  Reads ADC conversion result and returns 16-bit sign-extended value.
//!
//! \fn void readData(uint8_t dataRx[])
//!
//! \param *dataRx points to receive data byte array
//!
//! \return int16_t (sign-extended data).
//
//*****************************************************************************
int16_t readData(uint8_t dataRx[])
{
    uint8_t numberOfBytes = SPI_CRC_ENABLED ? 4 : 3;

    // NULL command
    uint8_t dataTx[4] = { 0 };
    if (SPI_CRC_ENABLED)
    {
        dataTx[3] = calculateCRC(dataTx, numberOfBytes - 1, CRC_INITIAL_SEED);
    }
    spiSendReceiveArray(dataTx, dataRx, numberOfBytes);

    return signExtend(dataRx);
}


//*****************************************************************************
//
//! \brief  Reads the contents of a single register at the specified address.
//!
//! \fn     uint8_t readSingleRegister(uint8_t address)
//!
//! \param  address is the 8-bit address of the register to read.
//!
//! \return Returns the 8-bit register read result.
//
//*****************************************************************************
uint8_t readSingleRegister(uint8_t address)
{
    // Check that the register address is in range
    assert(address <= MAX_REGISTER_ADDRESS);

    uint8_t dataTx[4] = {0};
    uint8_t dataRx[4] = {0};
    uint8_t numberOfBytes = SPI_CRC_ENABLED ? 4 : 3;
    bool crcError = false;

    //
    // [FRAME 1] RREG command
    //
    dataTx[0] = OPCODE_RREG;
    dataTx[1] = address;
    dataTx[2] = OPCODE_NULL;
    if (SPI_CRC_ENABLED)
    {
        dataTx[3] = calculateCRC(dataTx, numberOfBytes - 1, CRC_INITIAL_SEED);
    }
    spiSendReceiveArray(dataTx, dataRx, numberOfBytes);

    //
    // [FRAME 2] NULL command
    //
    dataTx[0] = OPCODE_NULL;
    dataTx[1] = OPCODE_NULL;
    dataTx[2] = OPCODE_NULL;
    if (SPI_CRC_ENABLED)
    {
        dataTx[3] = calculateCRC(dataTx, numberOfBytes - 1, CRC_INITIAL_SEED);
    }
    spiSendReceiveArray(dataTx, dataRx, numberOfBytes);

    // Check for CRC error
    if (SPI_CRC_ENABLED)
    {
        // To check the CRC validity you can test either of the following conditions:
        // 1) "dataRx[1] == calculateCRC(dataRx, 1, CRC_INITIAL_SEED)" - true means no CRC error occurred.
        // 2) "0x00 == calculateCRC(dataRx, 2, CRC_INITIAL_SEED) - including the CRC byte in the calculation should return 0x00.
        crcError = (bool) calculateCRC(dataRx, 2, CRC_INITIAL_SEED);
    }
    if (crcError)
    {
        // Update internal register array
        registerMap[SYSTEM_STATUS_ADDRESS] = registerMap[SYSTEM_STATUS_ADDRESS] || SYSTEM_STATUS_CRCERR_IN_MASK;

        // (OPTIONAL) Consider notifying the system of the error and repeating the previous command.
    }
    else
    {
        registerMap[address] = dataRx[0];
    }

    return registerMap[address];
}


//*****************************************************************************
//
//! Getter function to access registerMap array from outside of this module.
//!
//! \fn uint8_t getRegisterValue(uint8_t address)
//!
//! NOTE: The internal registerMap array stores the last known register value,
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
    assert(address <= MAX_REGISTER_ADDRESS);
    return registerMap[address];
}


//*****************************************************************************
//
//! \brief  Writes data to a single register and reads it back for confirmation.
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
    // Check that the register address is in range
    assert(address <= MAX_REGISTER_ADDRESS);

    uint8_t dataTx[4] = { 0 };
    uint8_t dataRx[4] = { 0 };
    uint8_t numberOfBytes = SPI_CRC_ENABLED ? 4 : 3;

    // (OPTIONAL) Check for and clear CRC error to proceed with register write.
    // Once a CRC error has occurred, writes are only allowed to the SYSTEM_STATUS and GENERAL_CFG registers
    if (SPI_CRC_ENABLED && (address > GENERAL_CFG_ADDRESS))
    {
        // Read STATUS register to check whether CRC error has occurred or not.
        readSingleRegister(SYSTEM_STATUS_ADDRESS);
        if (SPI_CRCERR_IN)
        {
            // (OPTIONAL) Clear the CRC error by writing 1b to CRCERR_IN bit
            setRegisterBits(SYSTEM_STATUS_ADDRESS, SYSTEM_STATUS_CRCERR_IN_MASK);

            // (OPTIONAL) Consider notifying the system of the error and repeating the previous command.
        }
    }

    // WREG command
    dataTx[0] = OPCODE_WREG;
    dataTx[1] = address;
    dataTx[2] = data;
    if (SPI_CRC_ENABLED)
    {
        dataTx[3] = calculateCRC(dataTx, numberOfBytes - 1, CRC_INITIAL_SEED);
    }
    spiSendReceiveArray(dataTx, dataRx, numberOfBytes);

    // Update internal register map array (assume command was successful).
    // NOTE: This is required for writing to the CRC_EN bit to ensure read back uses the correct mode.
    registerMap[address] = data;

    // NOTE: If you modify the CPOL_CPHA bits in the DATA_CFG register, the SPI perhiperal will need to be reconfigured here.

    // (RECOMMENDED) Read back register to confirm register write was successful
    registerMap[address] = readSingleRegister(address);
}


//*****************************************************************************
//
//! \fn void setRegisterBits(uint8_t address, uint8_t bitMask)
//!
//! \param address is the address of the register to write to.
//! \param bitMask indicates which bit(s) in the register to set.
//!
//! This function does not perform a read back of the register value.
//!
//! \return None.
//
//*****************************************************************************
void setRegisterBits(uint8_t address, uint8_t bitMask)
{
    // Check that the register address is in range
    assert(address <= MAX_REGISTER_ADDRESS);

    uint8_t dataTx[4] = {0};
    uint8_t dataRx[4] = {0};
    uint8_t numberOfBytes = SPI_CRC_ENABLED ? 4 : 3;

    // SETBIT command
    dataTx[0] = OPCODE_SETBIT;
    dataTx[1] = address;
    dataTx[2] = bitMask;
    if (SPI_CRC_ENABLED)
    {
        dataTx[3] = calculateCRC(dataTx, numberOfBytes - 1, CRC_INITIAL_SEED);
    }
    spiSendReceiveArray(dataTx, dataRx, numberOfBytes);

    // Update internal register map array (assume command was successful).
    // NOTE: This is required for writing to the CRC_EN bit to ensure read back uses the correct mode.
    registerMap[address] = registerMap[address] | bitMask;

    // (OPTIONAL) Check if a CRC error occurred
}


//*****************************************************************************
//
//!
//! \fn void clearRegisterBits(uint8_t address, uint8_t bitMask)
//!
//! \param address is the address of the register to write to.
//! \param bitMask indicates which bit(s) in the register to clear.
//!
//! This function does not perform a read back of the register value.
//!
//! \return None.
//
//*****************************************************************************
void clearRegisterBits(uint8_t address, uint8_t bitMask)
{
    // Check that the register address is in range
    assert(address <= MAX_REGISTER_ADDRESS);

    uint8_t dataTx[4] = {0};
    uint8_t dataRx[4] = {0};
    uint8_t numberOfBytes = SPI_CRC_ENABLED ? 4 : 3;

    // CLRBIT command
    dataTx[0] = OPCODE_CLRBIT;
    dataTx[1] = address;
    dataTx[2] = bitMask;
    if (SPI_CRC_ENABLED)
    {
        dataTx[3] = calculateCRC(dataTx, numberOfBytes - 1, CRC_INITIAL_SEED);
    }
    spiSendReceiveArray(dataTx, dataRx, numberOfBytes);

    // Update internal register map array (assume command was successful).
    // NOTE: This is required for writing to the CRC_EN bit to ensure read back uses the correct mode.
    registerMap[address] = registerMap[address] & ~bitMask;

    // (OPTIONAL) Check if a CRC error occurred
}


//*****************************************************************************
//
//! \brief  Calculates the 8-bit CRC for the selected CRC polynomial.
//!
//! \fn uint8_t calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint8_t initialValue)
//!
//! \param dataBytes[] pointer to first element in the data byte array
//! \param numberBytes number of bytes to be used in CRC calculation
//! \param initialValue the seed value (or partial crc calculation), use CRC_INITIAL_SEED when beginning a new CRC computation
//!
//! NOTE: This calculation is shown as an example and is not optimized for speed.
//!
//! \return 8-bit calculated CRC word
//
//*****************************************************************************
uint8_t calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint8_t initialValue)
{
    // Check that "dataBytes" is not a null pointer
    assert(dataBytes != 0x00);

    int         bitIndex, byteIndex;
    bool        dataMSb;                        /* Most significant bit of data byte */
    bool        crcMSb;                         /* Most significant bit of crc byte  */

    // Initial value of crc register
    // Use 0x00 when starting a new computation OR provide result of previous CRC calculation when continuing an on-going calculation.
    uint8_t crc = initialValue;

    // CRC polynomial = x^8 + x^2 + x^1 + 1
    const uint8_t poly = 0x07;

    /* CRC algorithm */

    // Loop through all bytes in the dataBytes[] array
    for (byteIndex = 0; byteIndex < numberBytes; byteIndex++)
    {
        // Point to MSb in byte
        bitIndex = 0x80u;

        // Loop through all bits in the current byte
        while (bitIndex > 0)
        {
            // Check MSB's of data and crc
            dataMSb = (bool) (dataBytes[byteIndex] & bitIndex);
            crcMSb  = (bool) (crc & 0x80u);

            // Update crc register
            crc <<= 1;
            if (dataMSb ^ crcMSb) { crc ^= poly; }

            // Shift MSb pointer to the next data bit
            bitIndex >>= 1;
        }
    }

    return crc;
}


//*****************************************************************************
//
//! Configure the selected channel as an analog Input
//!
//! \fn void setChannelAsAnalogInput(uint8_t channelID)
//!
//! \param channelID is the channel number.
//!
//! \return None
//
//*****************************************************************************
void setChannelAsAnalogInput(uint8_t channelID)
{
    // Check that channel ID is in range.
    assert(channelID < 8);

    // Clear the corresponding channel bit to configure channel as an analog input
    clearRegisterBits(PIN_CFG_ADDRESS, (1 << channelID));
}


//****************************************************************************
//
// Helper functions
//
//****************************************************************************

//*****************************************************************************
//
//! \brief  Updates the registerMap[] array to its default values.
//!
//! \fn static void restoreRegisterDefaults(void)
//!
//! NOTES:
//! - If the MCU keeps a copy of the ADS7038 register settings in memory,
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
    registerMap[SYSTEM_STATUS_ADDRESS]          = SYSTEM_STATUS_DEFAULT;
    registerMap[GENERAL_CFG_ADDRESS]            = GENERAL_CFG_DEFAULT;

    registerMap[DATA_CFG_ADDRESS]               = DATA_CFG_DEFAULT;
    registerMap[OSR_CFG_ADDRESS]                = OSR_CFG_DEFAULT;
    registerMap[OPMODE_CFG_ADDRESS]             = OPMODE_CFG_DEFAULT;
    registerMap[PIN_CFG_ADDRESS]                = PIN_CFG_DEFAULT;

    registerMap[GPIO_CFG_ADDRESS]               = GPIO_CFG_DEFAULT;
    registerMap[GPO_DRIVE_CFG_ADDRESS]          = GPO_DRIVE_CFG_DEFAULT;
    registerMap[GPO_OUTPUT_VALUE_ADDRESS]       = GPO_OUTPUT_VALUE_DEFAULT;
    registerMap[GPI_VALUE_ADDRESS]              = GPI_VALUE_DEFAULT;

    registerMap[SEQUENCE_CFG_ADDRESS]           = SEQUENCE_CFG_DEFAULT;
    registerMap[CHANNEL_SEL_ADDRESS]            = CHANNEL_SEL_DEFAULT;
    registerMap[AUTO_SEQ_CHSEL_ADDRESS]         = AUTO_SEQ_CHSEL_DEFAULT;

    registerMap[ALERT_CH_SEL_ADDRESS]           = ALERT_CH_SEL_DEFAULT;
    registerMap[ALERT_MAP_ADDRESS]              = ALERT_MAP_DEFAULT;
    registerMap[ALERT_PIN_CFG_ADDRESS]          = ALERT_PIN_CFG_DEFAULT;

    registerMap[EVENT_FLAG_ADDRESS]             = EVENT_FLAG_DEFAULT;
    registerMap[EVENT_HIGH_FLAG_ADDRESS]        = EVENT_HIGH_FLAG_DEFAULT;
    registerMap[EVENT_LOW_FLAG_ADDRESS]         = EVENT_LOW_FLAG_DEFAULT;
    registerMap[EVENT_RGN_ADDRESS]              = EVENT_RGN_DEFAULT;

    registerMap[HYSTERESIS_CH0_ADDRESS]         = HYSTERESIS_CHx_DEFAULT;
    registerMap[HYSTERESIS_CH1_ADDRESS]         = HYSTERESIS_CHx_DEFAULT;
    registerMap[HYSTERESIS_CH2_ADDRESS]         = HYSTERESIS_CHx_DEFAULT;
    registerMap[HYSTERESIS_CH3_ADDRESS]         = HYSTERESIS_CHx_DEFAULT;
    registerMap[HYSTERESIS_CH4_ADDRESS]         = HYSTERESIS_CHx_DEFAULT;
    registerMap[HYSTERESIS_CH5_ADDRESS]         = HYSTERESIS_CHx_DEFAULT;
    registerMap[HYSTERESIS_CH6_ADDRESS]         = HYSTERESIS_CHx_DEFAULT;
    registerMap[HYSTERESIS_CH7_ADDRESS]         = HYSTERESIS_CHx_DEFAULT;

    registerMap[EVENT_COUNT_CH0_ADDRESS]        = EVENT_COUNT_CHx_DEFAULT;
    registerMap[EVENT_COUNT_CH1_ADDRESS]        = EVENT_COUNT_CHx_DEFAULT;
    registerMap[EVENT_COUNT_CH2_ADDRESS]        = EVENT_COUNT_CHx_DEFAULT;
    registerMap[EVENT_COUNT_CH3_ADDRESS]        = EVENT_COUNT_CHx_DEFAULT;
    registerMap[EVENT_COUNT_CH4_ADDRESS]        = EVENT_COUNT_CHx_DEFAULT;
    registerMap[EVENT_COUNT_CH5_ADDRESS]        = EVENT_COUNT_CHx_DEFAULT;
    registerMap[EVENT_COUNT_CH6_ADDRESS]        = EVENT_COUNT_CHx_DEFAULT;
    registerMap[EVENT_COUNT_CH7_ADDRESS]        = EVENT_COUNT_CHx_DEFAULT;

    registerMap[HIGH_TH_CH0_ADDRESS]            = HIGH_TH_CHx_DEFAULT;
    registerMap[HIGH_TH_CH1_ADDRESS]            = HIGH_TH_CHx_DEFAULT;
    registerMap[HIGH_TH_CH2_ADDRESS]            = HIGH_TH_CHx_DEFAULT;
    registerMap[HIGH_TH_CH3_ADDRESS]            = HIGH_TH_CHx_DEFAULT;
    registerMap[HIGH_TH_CH4_ADDRESS]            = HIGH_TH_CHx_DEFAULT;
    registerMap[HIGH_TH_CH5_ADDRESS]            = HIGH_TH_CHx_DEFAULT;
    registerMap[HIGH_TH_CH6_ADDRESS]            = HIGH_TH_CHx_DEFAULT;
    registerMap[HIGH_TH_CH7_ADDRESS]            = HIGH_TH_CHx_DEFAULT;

    registerMap[LOW_TH_CH0_ADDRESS]             = LOW_TH_CHx_DEFAULT;
    registerMap[LOW_TH_CH1_ADDRESS]             = LOW_TH_CHx_DEFAULT;
    registerMap[LOW_TH_CH2_ADDRESS]             = LOW_TH_CHx_DEFAULT;
    registerMap[LOW_TH_CH3_ADDRESS]             = LOW_TH_CHx_DEFAULT;
    registerMap[LOW_TH_CH4_ADDRESS]             = LOW_TH_CHx_DEFAULT;
    registerMap[LOW_TH_CH5_ADDRESS]             = LOW_TH_CHx_DEFAULT;
    registerMap[LOW_TH_CH6_ADDRESS]             = LOW_TH_CHx_DEFAULT;
    registerMap[LOW_TH_CH7_ADDRESS]             = LOW_TH_CHx_DEFAULT;

    registerMap[MAX_CH0_LSB_ADDRESS]            = MAX_CHx_LSB_DEFAULT;
    registerMap[MAX_CH1_LSB_ADDRESS]            = MAX_CHx_LSB_DEFAULT;
    registerMap[MAX_CH2_LSB_ADDRESS]            = MAX_CHx_LSB_DEFAULT;
    registerMap[MAX_CH3_LSB_ADDRESS]            = MAX_CHx_LSB_DEFAULT;
    registerMap[MAX_CH4_LSB_ADDRESS]            = MAX_CHx_LSB_DEFAULT;
    registerMap[MAX_CH5_LSB_ADDRESS]            = MAX_CHx_LSB_DEFAULT;
    registerMap[MAX_CH6_LSB_ADDRESS]            = MAX_CHx_LSB_DEFAULT;
    registerMap[MAX_CH7_LSB_ADDRESS]            = MAX_CHx_LSB_DEFAULT;

    registerMap[MAX_CH0_MSB_ADDRESS]            = MAX_CHx_MSB_DEFAULT;
    registerMap[MAX_CH1_MSB_ADDRESS]            = MAX_CHx_MSB_DEFAULT;
    registerMap[MAX_CH2_MSB_ADDRESS]            = MAX_CHx_MSB_DEFAULT;
    registerMap[MAX_CH3_MSB_ADDRESS]            = MAX_CHx_MSB_DEFAULT;
    registerMap[MAX_CH4_MSB_ADDRESS]            = MAX_CHx_MSB_DEFAULT;
    registerMap[MAX_CH5_MSB_ADDRESS]            = MAX_CHx_MSB_DEFAULT;
    registerMap[MAX_CH6_MSB_ADDRESS]            = MAX_CHx_MSB_DEFAULT;
    registerMap[MAX_CH7_MSB_ADDRESS]            = MAX_CHx_MSB_DEFAULT;

    registerMap[MIN_CH0_LSB_ADDRESS]            = MIN_CHx_LSB_DEFAULT;
    registerMap[MIN_CH1_LSB_ADDRESS]            = MIN_CHx_LSB_DEFAULT;
    registerMap[MIN_CH2_LSB_ADDRESS]            = MIN_CHx_LSB_DEFAULT;
    registerMap[MIN_CH3_LSB_ADDRESS]            = MIN_CHx_LSB_DEFAULT;
    registerMap[MIN_CH4_LSB_ADDRESS]            = MIN_CHx_LSB_DEFAULT;
    registerMap[MIN_CH5_LSB_ADDRESS]            = MIN_CHx_LSB_DEFAULT;
    registerMap[MIN_CH6_LSB_ADDRESS]            = MIN_CHx_LSB_DEFAULT;
    registerMap[MIN_CH7_LSB_ADDRESS]            = MIN_CHx_LSB_DEFAULT;

    registerMap[MIN_CH0_MSB_ADDRESS]            = MIN_CHx_MSB_DEFAULT;
    registerMap[MIN_CH1_MSB_ADDRESS]            = MIN_CHx_MSB_DEFAULT;
    registerMap[MIN_CH2_MSB_ADDRESS]            = MIN_CHx_MSB_DEFAULT;
    registerMap[MIN_CH3_MSB_ADDRESS]            = MIN_CHx_MSB_DEFAULT;
    registerMap[MIN_CH4_MSB_ADDRESS]            = MIN_CHx_MSB_DEFAULT;
    registerMap[MIN_CH5_MSB_ADDRESS]            = MIN_CHx_MSB_DEFAULT;
    registerMap[MIN_CH6_MSB_ADDRESS]            = MIN_CHx_MSB_DEFAULT;
    registerMap[MIN_CH7_MSB_ADDRESS]            = MIN_CHx_MSB_DEFAULT;

    registerMap[RECENT_CH0_LSB_ADDRESS]         = RECENT_CHx_LSB_DEFAULT;
    registerMap[RECENT_CH1_LSB_ADDRESS]         = RECENT_CHx_LSB_DEFAULT;
    registerMap[RECENT_CH2_LSB_ADDRESS]         = RECENT_CHx_LSB_DEFAULT;
    registerMap[RECENT_CH3_LSB_ADDRESS]         = RECENT_CHx_LSB_DEFAULT;
    registerMap[RECENT_CH4_LSB_ADDRESS]         = RECENT_CHx_LSB_DEFAULT;
    registerMap[RECENT_CH5_LSB_ADDRESS]         = RECENT_CHx_LSB_DEFAULT;
    registerMap[RECENT_CH6_LSB_ADDRESS]         = RECENT_CHx_LSB_DEFAULT;
    registerMap[RECENT_CH7_LSB_ADDRESS]         = RECENT_CHx_LSB_DEFAULT;

    registerMap[RECENT_CH0_MSB_ADDRESS]         = RECENT_CHx_MSB_DEFAULT;
    registerMap[RECENT_CH1_MSB_ADDRESS]         = RECENT_CHx_MSB_DEFAULT;
    registerMap[RECENT_CH2_MSB_ADDRESS]         = RECENT_CHx_MSB_DEFAULT;
    registerMap[RECENT_CH3_MSB_ADDRESS]         = RECENT_CHx_MSB_DEFAULT;
    registerMap[RECENT_CH4_MSB_ADDRESS]         = RECENT_CHx_MSB_DEFAULT;
    registerMap[RECENT_CH5_MSB_ADDRESS]         = RECENT_CHx_MSB_DEFAULT;
    registerMap[RECENT_CH6_MSB_ADDRESS]         = RECENT_CHx_MSB_DEFAULT;
    registerMap[RECENT_CH7_MSB_ADDRESS]         = RECENT_CHx_MSB_DEFAULT;

    registerMap[GPO0_TRIG_EVENT_SEL_ADDRESS]    = GPOx_TRIG_EVENT_SEL_DEFAULT;
    registerMap[GPO1_TRIG_EVENT_SEL_ADDRESS]    = GPOx_TRIG_EVENT_SEL_DEFAULT;
    registerMap[GPO2_TRIG_EVENT_SEL_ADDRESS]    = GPOx_TRIG_EVENT_SEL_DEFAULT;
    registerMap[GPO3_TRIG_EVENT_SEL_ADDRESS]    = GPOx_TRIG_EVENT_SEL_DEFAULT;
    registerMap[GPO4_TRIG_EVENT_SEL_ADDRESS]    = GPOx_TRIG_EVENT_SEL_DEFAULT;
    registerMap[GPO5_TRIG_EVENT_SEL_ADDRESS]    = GPOx_TRIG_EVENT_SEL_DEFAULT;
    registerMap[GPO6_TRIG_EVENT_SEL_ADDRESS]    = GPOx_TRIG_EVENT_SEL_DEFAULT;
    registerMap[GPO7_TRIG_EVENT_SEL_ADDRESS]    = GPOx_TRIG_EVENT_SEL_DEFAULT;

    registerMap[GPO_TRIGGER_CFG_ADDRESS]        = GPO_TRIGGER_CFG_DEFAULT;
    registerMap[GPO_VALUE_TRIG_ADDRESS]         = GPO_VALUE_TRIG_DEFAULT;
}


//*****************************************************************************
//
//! Called by readData() to convert ADC data from multiple unsigned
//! bytes into a single, signed 16-bit word.
//!
//! \fn int16_t signExtend(const uint8_t dataBytes[])
//!
//! \param dataBytes pointer to data array (big-endian).
//!
//! \return Returns the signed-extend 16-bit result.
//
//*****************************************************************************
static int16_t signExtend(const uint8_t dataBytes[])
{
    int16_t upperByte = ((int32_t)dataBytes[0] << 8);
    int16_t lowerByte = ((int32_t)dataBytes[1] << 0);

    // NOTE: This right-shift operation on signed data maintains the sign bit
    uint8_t shiftDistance = AVERAGING_ENABLED ? 0 : 4;
    return (((int16_t)(upperByte | lowerByte)) >> shiftDistance);
}

