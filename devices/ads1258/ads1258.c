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

#include "ads1258.h"



//****************************************************************************
//
// Macros
//
//****************************************************************************
/** Alias used for setting GPIOs pins to the logic "high" state */
#define HIGH                ((bool) true)

/** Alias used for setting GPIOs pins to the logic "low" state */
#define LOW                 ((bool) false)



//****************************************************************************
//
// Internal variables
//
//****************************************************************************

/* Internal register map array (to recall current configuration) */
static uint8_t registerMap[NUM_REGISTERS];



//****************************************************************************
//
// Functions
//
//****************************************************************************

/**
 * \fn uint8_t getRegisterValue(uint8_t address)
 * \brief Getter function to access the registerMap array outside of this module
 * \param address The 8-bit register address
 * \return The 8-bit register value
 */
uint8_t getRegisterValue(uint8_t address)
{
    assert(address < NUM_REGISTERS);
    return registerMap[address];
}



/**
 * \fn void adcStartupRoutine(void)
 * \brief Startup function to be called before communicating with the ADC
 */
void adcStartupRoutine(void)
{
    /* (OPTIONAL) Provide additional delay time for power supply settling */
    delay_ms(50);

    /* (REQUIRED) Set nRESET/nPWDN pin high for ADC operation */
    setPWDN(HIGH);

    /* (OPTIONAL) Start ADC conversions with HW pin control  */
    setSTART(HIGH);

    /* (REQUIRED) tWAKE delay */
    delay_ms(5);

    /* (OPTIONAL) Toggle nRESET pin to assure default register settings. */
    /* NOTE: This also ensures that the device registers are unlocked.   */
    toggleRESET();

    /* Ensure internal register array is initialized */
    restoreRegisterDefaults();

    /* (OPTIONAL) Configure initial device register settings here */
    uint8_t initRegisterMap[NUM_REGISTERS];
    initRegisterMap[REG_ADDR_CONFIG0]   =   CONFIG0_DEFAULT;
    initRegisterMap[REG_ADDR_CONFIG1]   =   CONFIG1_DLY_64us | CONFIG1_DRATE_7813SPS;
    initRegisterMap[REG_ADDR_MUXSCH]    =   MUXSCH_DEFAULT;
    initRegisterMap[REG_ADDR_MUXDIF]    =   MUXDIF_DEFAULT;
    initRegisterMap[REG_ADDR_MUXSG0]    =   MUXSG0_DEFAULT;
    initRegisterMap[REG_ADDR_MUXSG1]    =   MUXSG1_DEFAULT;
    initRegisterMap[REG_ADDR_SYSRED]    =   SYSRED_DEFAULT;
    initRegisterMap[REG_ADDR_GPIOC]     =   GPIOC_DEFAULT;
    initRegisterMap[REG_ADDR_GPIOD]     =   GPIOD_DEFAULT;
    initRegisterMap[REG_ADDR_ID]        =   0x00;           // Read-only register

    /* (OPTIONAL) Write to all (writable) registers */
    writeMultipleRegisters(REG_ADDR_CONFIG0, NUM_REGISTERS - 1, initRegisterMap);

    /* (OPTIONAL) Read back all registers */
    readMultipleRegisters(REG_ADDR_CONFIG0, NUM_REGISTERS);

    /* (OPTIONAL) Start ADC conversions with the SPI command.
     * Not needed if the START pin has already been set HIGH.
     *
     * sendCommand(START_OPCODE);
     */
}



/**
 * \fn uint8_t readSingleRegister(uint8_t addr)
 * \brief Reads contents of a single register at the specified address
 * \param addr address of the register to read
 * \return 8-bit register read result
 */
uint8_t readSingleRegister(uint8_t address)
{
    /* Check that the register address is in range */
    assert(address < NUM_REGISTERS);

    /* Initialize arrays */
    uint8_t DataTx[2] = { 0 };
    uint8_t DataRx[2] = { 0 };

    /* Build TX array and send it */
    DataTx[0] = OPCODE_RREG | (address & OPCODE_A_MASK);
    spiSendReceiveArrays(DataTx, DataRx, 2);

    /* Update register array and return read result*/
    registerMap[address] = DataRx[1];
    return DataRx[1];
}



/**
 * \fn void readMultipleRegisters(uint8_t startAddress, uint8_t count)
 * \brief Reads a group of registers starting at the specified address
 * \param startAddress register address from which we start reading
 * \param count number of registers we want to read
 * NOTE: Use getRegisterValue() to retrieve the read values
 */
void readMultipleRegisters(uint8_t startAddress, uint8_t count)
{
    /* Check that the register address and count are in range */
    assert(startAddress + count <= NUM_REGISTERS);


    //
    // SPI communication
    //

    setCS(LOW);

    uint8_t dataTx = OPCODE_RREG | OPCODE_MUL_MASK | (startAddress & OPCODE_A_MASK);
    spiSendReceiveByte(dataTx);

    uint8_t i;
    for (i = startAddress; i < startAddress + count; i++)
    {
        // Read register data bytes
        registerMap[i] = spiSendReceiveByte(0x00);
    }

    setCS(HIGH);
}



/**
 * \fn void writeSingleRegister(uint8_t address, uint8_t data)
 * \brief Write data to a single register at the specified address
 * \param address The address of the register to write
 * \param data The 8-bit data to write to the register
 */
void writeSingleRegister(uint8_t address, uint8_t data)
{
    /* Check that the register address is in range */
    assert(address < NUM_REGISTERS);

    /* Initialize arrays */
    uint8_t DataTx[2];
    uint8_t DataRx[2] = { 0 };

    /* Build TX array and send it */
    DataTx[0] = ( OPCODE_WREG | (address & OPCODE_A_MASK) );
    DataTx[1] = data;
    spiSendReceiveArrays(DataTx, DataRx, 2);

    /* Update register array */
    registerMap[address] = DataTx[1];
}



/**
 * \fn void writeMultipleRegisters(uint8_t startAddress, uint8_t count, const uint8_t regData[])
 * \brief Writes data to a group of registers
 * \param startAddress register address from which we start write
 * \param count number of registers we want to write to
 * \param regData Array that holds the data to write, where element zero is the data to write to the starting address.
 * NOTES:
 * - Use getRegisterValue() to retrieve the written values.
 * - Registers should be re-read after a write operaiton to ensure proper configuration.
 */
void writeMultipleRegisters(uint8_t startAddress, uint8_t count, const uint8_t regData[])
{
    /* Check that the register address and count are in range */
    assert(startAddress + count <= NUM_REGISTERS);

    /* Check that regData is not a NULL pointer */
    assert(regData);


    //
    // SPI communication
    //

    setCS(LOW);

    uint8_t dataTx = OPCODE_WREG | OPCODE_MUL_MASK | (startAddress & OPCODE_A_MASK);
    spiSendReceiveByte(dataTx);

    uint8_t i;
    for (i = startAddress; i < startAddress + count; i++)
    {
        // write register data bytes
        spiSendReceiveByte(regData[i]);

        /* Update register array */
        registerMap[i] = regData[i];
    }

    setCS(HIGH);
}



/**
 * \fn void sendCommand(uint8_t op_code)
 * \brief Sends the specified SPI command to the ADC
 * \param op_code SPI command byte
 */
void sendCommand(uint8_t op_code)
{
    /* Assert if this function is used to send any of the following commands */
    assert(OPCODE_RREG         != op_code);    /* Use "readSingleRegister()"  or "readMultipleRegisters()"  */
    assert(OPCODE_WREG         != op_code);    /* Use "writeSingleRegister()" or "writeMultipleRegisters()" */
    assert(OPCODE_READ_DIRECT  != op_code);    /* Use "readData()" */
    assert(OPCODE_READ_COMMAND != op_code);    /* Use "readData()" */

    /* SPI communication */
    setCS(LOW);
    spiSendReceiveByte(op_code);
    setCS(HIGH);

    // Check for RESET command
    if (OPCODE_RESET == op_code)
    {
        /* Update register array to keep software in sync with device */
        restoreRegisterDefaults();
    }
}



/**
 * \fn void startConversions()
 * \brief Wakes the device from power-down and starts continuous conversions
 */
void startConversions(void)
{
    /* Ensure device is not in PWDN mode */
    setPWDN(HIGH);

    /* Begin continuous conversions */
    setSTART(HIGH);
}



/* Retrieves the ADC conversion result 
 * NOTE: Call this function after /DRDY goes low and specify the 
 * the number of bytes to read and the starting position of data
 * (based on whe)
 *
 * This function implements the read command
*/

/**
 * \fn int32_t readData(uint8_t status[], uint8_t data[], readMode mode)
 * \brief Sends the read command and retrieves STATUS (if enabled) and data
 * \param status[] pointer to address where STATUS byte will be stored
 * \param data[] pointer to starting address where data bytes will be stored
 * \param mode readMode typedef to select which read operation to use
 * \return 32-bit sign-extended conversion result (data only)
 */
int32_t readData(uint8_t status[], uint8_t data[], readMode mode)
{
    uint8_t DataTx[5] = { 0 };    // Initialize all array elements to 0
    uint8_t DataRx[5] = { 0 };    // Relies on C99 [$6.7.8/10], [$6.7.8/21]
    uint8_t byteLength;
    uint8_t dataPosition;
    bool    status_byte_enabled;

    /* Build TX array and send it */
    if (mode == DIRECT)
    {
        DataTx[0] = OPCODE_READ_DIRECT;

        /* STATUS byte enable depends on MUXMOD and STAT (see table 17 in datasheet) */
        status_byte_enabled = IS_STAT_SET & !IS_MUXMOD_SET;
        byteLength = (status_byte_enabled ? 4 : 3);
        dataPosition = (status_byte_enabled ? 1 : 0);
    }
    else if (mode == COMMAND)
    {
        DataTx[0] = OPCODE_READ_COMMAND | OPCODE_MUL_MASK;

        /* STATUS byte always enabled, but undefined in fixed-channel mode (see table 17 in datasheet) */
        status_byte_enabled = true;
        byteLength = 5;
        dataPosition = 2;
    }
    spiSendReceiveArrays(DataTx, DataRx, byteLength);



    //
    // Parse returned SPI data
    //

    /* Check if STATUS byte is enabled and if we have a valid "status" memory pointer */
    if (status_byte_enabled && status)
    {
        status[0] = DataRx[dataPosition - 1];
    }

    /* Check for "data" pointer before making assignments */
    if (data)
    {
        data[0] = DataRx[dataPosition + 0];
        data[1] = DataRx[dataPosition + 1];
        data[2] = DataRx[dataPosition + 2];
    }

    /* Return the 32-bit sign-extended conversion result */
	int32_t signByte;
    if (DataRx[dataPosition] & 0x80u)   { signByte = 0xFF000000; }
    else                                { signByte = 0x00000000; }

	int32_t upperByte 	= ((int32_t) DataRx[dataPosition + 0] & 0xFF) << 16;
	int32_t middleByte  = ((int32_t) DataRx[dataPosition + 1] & 0xFF) << 8;
	int32_t lowerByte	= ((int32_t) DataRx[dataPosition + 2] & 0xFF) << 0;

	return (signByte | upperByte | middleByte | lowerByte);
}



/**
 * \fn void restoreRegisterDefaults(void)
 * \brief Updates the registerMap[] array to its default values.
 *
 * NOTES:
 * - If the MCU keeps a copy of the ADC register settings in memory,
 * then it is important to ensure that these values remain in sync with the
 * actual hardware settings. In order to help facilitate this, this function
 * should be called after powering up or resetting the device (either by
 * hardware pin control or SPI software command).
 *
 * - Reading back all of the registers after resetting the device will
 * accomplish the same result.
 */
void restoreRegisterDefaults(void)
{
    registerMap[REG_ADDR_CONFIG0]   =   CONFIG0_DEFAULT;
    registerMap[REG_ADDR_CONFIG1]   =   CONFIG1_DEFAULT;
    registerMap[REG_ADDR_MUXSCH]    =   MUXSCH_DEFAULT;
    registerMap[REG_ADDR_MUXDIF]    =   MUXDIF_DEFAULT;
    registerMap[REG_ADDR_MUXSG0]    =   MUXSG0_DEFAULT;
    registerMap[REG_ADDR_MUXSG1]    =   MUXSG1_DEFAULT;
    registerMap[REG_ADDR_SYSRED]    =   SYSRED_DEFAULT;
    registerMap[REG_ADDR_GPIOC]     =   GPIOC_DEFAULT;
    registerMap[REG_ADDR_GPIOD]     =   GPIOD_DEFAULT;
    registerMap[REG_ADDR_ID]        =   0x00;               // Value of 0x00 indicates that we have not yet read the ID register
}
