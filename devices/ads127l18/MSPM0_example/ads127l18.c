/*
 * @file ads127l18.c
 *
 * @brief ADS127L18 Descriptor
 *
 * @copyright Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
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

#include "ads127l18.h"
#include "ti/driverlib/m0p/dl_core.h"

//****************************************************************************
//
// Internal variables
//
//****************************************************************************

// Array used to recall device register map configurations */
static uint8_t             registerMap[NUM_REGISTERS]; 

//****************************************************************************
//
// Internal function prototypes
//
//****************************************************************************

uint8_t     buildSPIarray(const uint8_t opcodeArray[], uint8_t byteArray[]);
void        BB_SPI_transmitData8(uint8_t value);

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
uint16_t getRegisterValue(uint8_t address)
{
    assert(address < NUM_REGISTERS);
    return registerMap[address];
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
	/* Check that the register address is in range */
	assert(address < NUM_REGISTERS);

	// Build TX and RX byte array
    uint8_t dataTx[SPI_BUFFER_SIZE] = { 0 };
    uint8_t dataRx[SPI_BUFFER_SIZE] = { 0 };

    // Build opcode and SPI TX array
    const uint8_t       arbitrary = 0x00;
    uint8_t             opcode[2] = {address, 0x00};
    uint8_t         numberOfBytes = buildSPIarray(opcode, dataTx);

	// [FRAME 1] Send RREG command
	spiSendReceiveArrays(dataTx, dataRx, numberOfBytes);

	// [FRAME 2] Send NULL command to retrieve the register data
	uint8_t nullopcode[2] = {OPCODE_NULL, OPCODE_NULL};
	sendCommand(nullopcode);

	return registerMap[address];
}



//*****************************************************************************
//
//! Writes data to a single register.
//!
//! \fn void writeSingleRegister(uint8_t address, uint8_t data)
//!
//! \param address is the address of the register to write to.
//! \param data is the value to write.
//!
//!
//! \return None.
//
//*****************************************************************************
void writeSingleRegister(uint8_t address, uint8_t data)
{
    /* Check that the register address is in range */
    assert(address < NUM_REGISTERS);

    // Build TX and RX byte array
    uint8_t dataTx[SPI_BUFFER_SIZE] = { 0 };
    uint8_t dataRx[SPI_BUFFER_SIZE] = { 0 };

    // Build opcode and SPI TX array
    uint8_t   opcode[2]     = {(OPCODE_WREG | address), data};
    uint8_t numberOfBytes   = buildSPIarray(opcode, dataTx);

    // Send command
    spiSendReceiveArrays(dataTx, dataRx, numberOfBytes);

    // Update internal array
    registerMap[address] = data;

    // (RECOMMENDED) Read back register to confirm register write was successful
    readSingleRegister(address);
}



//*****************************************************************************
//
//! Sends the specified SPI command to the ADC (NULL, STANDBY, or WAKEUP).
//!
//! \fn uint8_t sendCommand(uint8_t opcode[])
//!
//! \param opcode SPI command byte.
//!
//! NOTE: Other commands have their own dedicated functions to support
//! additional functionality.
//!
//! \return ADC response byte (typically the STATUS byte).
//
//*****************************************************************************
void sendCommand(uint8_t opcode[])
{
    /* Assert if this function is used to send any of the following opcodes */
    assert(OPCODE_RREG != opcode[0]);      /* Use "readSingleRegister()"   */
    assert(OPCODE_WREG != opcode[0]);      /* Use "writeSingleRegister()"  */

    // Build TX byte array
    uint8_t dataTx[SPI_BUFFER_SIZE] = { 0 };

    // set CS low
    DL_GPIO_clearPins(GPIO_SPI_PORT, GPIO_SPI_CS_PIN); 

    // Send the null opcode
    BB_SPI_transmitData8(dataTx[0]);
    BB_SPI_transmitData8(dataTx[1]);

    // set CS high
    DL_GPIO_setPins(GPIO_SPI_PORT, GPIO_SPI_CS_PIN);
}



//****************************************************************************
//
// Internal functions
//
//****************************************************************************


//*****************************************************************************
//
//! Builds SPI TX data arrays according to number of opcodes provided and
//! currently programmed device word length.
//!
//! \fn uint8_t buildSPIarray(const uint16_t opcodeArray[], uint8_t byteArray[])
//!
//! \param opcodeArray[] pointer to an array of 8-bit opcodes to use in the SPI command.
//! \param byteArray[] pointer to an array of 8-bit SPI bytes to send to the device.
//!
//! NOTE: The calling function must ensure it reserves sufficient memory for byteArray[]!
//!
//! \return number of bytes added to byteArray[].
//
//*****************************************************************************
uint8_t buildSPIarray(const uint8_t opcodeArray[], uint8_t byteArray[])
{
    
    // const uint8_t numberOfBytes  = numberOpcodes + frontPadBytes;//Determine total number of bytes to clock
    const uint8_t numberOfBytes  = 2;

    //Place the command byte/data in the correct place in the array
    byteArray[0] = opcodeArray[0];
    byteArray[1] = opcodeArray[1];

    return numberOfBytes;
}

//*****************************************************************************
//
//! This function replicates the SPI procotol by toggling on/off
//! GPIO pins that represent each SPI signal
//!
//! \fn void BB_SPI_transmitData8(uint8_t data)
//!
//! \param data is an 8-bit value that will be sent
//!
//! NOTES:
//! The timing of the SCLK signal is not perfect, but the timing
//! is interpretable to the ADS127L18
//!
//! \return None.
//
//*****************************************************************************
void BB_SPI_transmitData8(uint8_t data) {
    uint8_t i;

    // send 8 bits
    for (i = 0; i < 8; i++) {
        // consider leftmost bit
        // set gpio signal high if bit is 1, low if bit is 0
        if (data & 0x80) {
        DL_GPIO_setPins(GPIO_SPI_PORT, GPIO_SPI_MOSI_PIN);
        } else {
        DL_GPIO_clearPins(GPIO_SPI_PORT, GPIO_SPI_MOSI_PIN);
        }

        // pulse the clock state to indicate that bit value should be read
        // Set SCLK High
        DL_GPIO_setPins(GPIO_SPI_PORT, GPIO_SPI_SCLK_PIN);
        delay_cycles(2); // The timing for the delay could be better

        // Set SCLK low
        DL_GPIO_clearPins(GPIO_SPI_PORT, GPIO_SPI_SCLK_PIN);

        // shift byte left so next bit will be leftmost
        data <<= 1;
    }
}

//*****************************************************************************
//
//! Updates the registerMap[] array to its default values.
//!
//! \fn void restoreRegisterDefaults(void)
//!
//! NOTES:
//! - If the MCU keeps a copy of the ADS127L18 register settings in memory,
//! then it is important to ensure that these values remain in sync with the
//! actual hardware settings. In order to help facilitate this, this function
//! should be called after powering up or resetting the device (either by
//! hardware pin control or SPI software command).
//!
//! - Reading back all of the registers after resetting the device can
//! accomplish the same result; however, this might be problematic if the
//! device was previously in CRC mode or STATUS output mode, since
//! resetting the device exits these modes. If the MCU is not aware of this
//! mode change, then read register commands will return invalid data due to
//! the expectation of data appearing in a different byte position.
//!
//! \return None.
//
//*****************************************************************************
void restoreRegisterDefaults(void)
{

    registerMap[DEV_ID_REG_ADDRESS]         =   DEV_ID_REG_DEFAULT;
    registerMap[REV_ID_REG_ADDRESS]         =   REV_ID_REG_DEFAULT;
    registerMap[STATUS_REG_ADDRESS]         =   STATUS_REG_DEFAULT;
    registerMap[CLK_CNT_REG_ADDRESS]        =   CLK_CNT_REG_DEFAULT;
    registerMap[GPIO_RD_REG_ADDRESS]        =   GPIO_RD_REG_DEFAULT;
    registerMap[CRC_MSB_REG_ADDRESS]        =   CRC_MSB_REG_DEFAULT;
    registerMap[CRC_LSB_REG_ADDRESS]        =   CRC_LSB_REG_DEFAULT;
    registerMap[CONTROL_REG_ADDRESS]        =   CONTROL_REG_DEFAULT;
    registerMap[GEN_CFG1_REG_ADDRESS]       =   GEN_CFG1_REG_DEFAULT;
    registerMap[GEN_CFG2_REG_ADDRESS]       =   GEN_CFG2_REG_DEFAULT;
    registerMap[GEN_CFG3_REG_ADDRESS]       =   GEN_CFG3_REG_DEFAULT;
    registerMap[DP_CFG1_REG_ADDRESS]        =   DP_CFG1_REG_DEFAULT;
    registerMap[DP_CFG2_REG_ADDRESS]        =   DP_CFG2_REG_DEFAULT;
    registerMap[CLK_CFG_REG_ADDRESS]        =   CLK_CFG_REG_DEFAULT;
    registerMap[GPIO_WR_REG_ADDRESS]        =   GPIO_WR_REG_DEFAULT;
    registerMap[GPIO_DIR_REG_ADDRESS]       =   GPIO_DIR_REG_DEFAULT;
    registerMap[GPIO_EN_REG_ADDRESS]        =   GPIO_EN_REG_DEFAULT;
    registerMap[CH0_CFG1_REG_ADDRESS]       =   CH0_CFG1_REG_DEFAULT;
    registerMap[CH0_CFG2_REG_ADDRESS]       =   CH0_CFG2_REG_DEFAULT;
    registerMap[CH0_OFS_MSB_REG_ADDRESS]    =   CH0_OFS_MSB_REG_DEFAULT;
    registerMap[CH0_OFS_MID_REG_ADDRESS]    =   CH0_OFS_MID_REG_DEFAULT;
    registerMap[CH0_OFS_LSB_REG_ADDRESS]    =   CH0_OFS_LSB_REG_DEFAULT;
    registerMap[CH0_GAN_MSB_REG_ADDRESS]    =   CH0_GAN_MSB_REG_DEFAULT;
    registerMap[CH0_GAN_MID_REG_ADDRESS]    =   CH0_GAN_MID_REG_DEFAULT;
    registerMap[CH0_GAN_MID_REG_ADDRESS]    =   CH0_GAN_MID_REG_DEFAULT;
    registerMap[CH1_CFG1_REG_ADDRESS]       =   CH1_CFG1_REG_DEFAULT;
    registerMap[CH1_CFG2_REG_ADDRESS]       =   CH1_CFG2_REG_DEFAULT;
    registerMap[CH1_OFS_MSB_REG_ADDRESS]    =   CH1_OFS_MSB_REG_DEFAULT;
    registerMap[CH1_OFS_MID_REG_ADDRESS]    =   CH1_OFS_MID_REG_DEFAULT;
    registerMap[CH1_OFS_LSB_REG_ADDRESS]    =   CH1_OFS_LSB_REG_DEFAULT;
    registerMap[CH1_GAN_MSB_REG_ADDRESS]    =   CH1_GAN_MSB_REG_DEFAULT;
    registerMap[CH1_GAN_MID_REG_ADDRESS]    =   CH1_GAN_MID_REG_DEFAULT;
    registerMap[CH1_GAN_MID_REG_ADDRESS]    =   CH1_GAN_MID_REG_DEFAULT;
    registerMap[CH2_CFG1_REG_ADDRESS]       =   CH2_CFG1_REG_DEFAULT;
    registerMap[CH2_CFG2_REG_ADDRESS]       =   CH2_CFG2_REG_DEFAULT;
    registerMap[CH2_OFS_MSB_REG_ADDRESS]    =   CH2_OFS_MSB_REG_DEFAULT;
    registerMap[CH2_OFS_MID_REG_ADDRESS]    =   CH2_OFS_MID_REG_DEFAULT;
    registerMap[CH2_OFS_LSB_REG_ADDRESS]    =   CH2_OFS_LSB_REG_DEFAULT;
    registerMap[CH2_GAN_MSB_REG_ADDRESS]    =   CH2_GAN_MSB_REG_DEFAULT;
    registerMap[CH2_GAN_MID_REG_ADDRESS]    =   CH2_GAN_MID_REG_DEFAULT;
    registerMap[CH2_GAN_LSB_REG_ADDRESS]    =   CH2_GAN_LSB_REG_DEFAULT;
    registerMap[CH3_CFG1_REG_ADDRESS]       =   CH3_CFG1_REG_DEFAULT;
    registerMap[CH3_CFG2_REG_ADDRESS]       =   CH3_CFG2_REG_DEFAULT;
    registerMap[CH3_OFS_MSB_REG_ADDRESS]    =   CH3_OFS_MSB_REG_DEFAULT;
    registerMap[CH3_OFS_MID_REG_ADDRESS]    =   CH3_OFS_MID_REG_DEFAULT;
    registerMap[CH3_OFS_LSB_REG_ADDRESS]    =   CH3_OFS_LSB_REG_DEFAULT;
    registerMap[CH3_GAN_MSB_REG_ADDRESS]    =   CH3_GAN_MSB_REG_DEFAULT;
    registerMap[CH3_GAN_MID_REG_ADDRESS]    =   CH3_GAN_MID_REG_DEFAULT;
    registerMap[CH3_GAN_LSB_REG_ADDRESS]    =   CH3_GAN_LSB_REG_DEFAULT;
    registerMap[CH4_CFG1_REG_ADDRESS]       =   CH4_CFG1_REG_DEFAULT;
    registerMap[CH4_CFG2_REG_ADDRESS]       =   CH4_CFG2_REG_DEFAULT;
    registerMap[CH4_OFS_MSB_REG_ADDRESS]    =   CH4_OFS_MSB_REG_DEFAULT;
    registerMap[CH4_OFS_MID_REG_ADDRESS]    =   CH4_OFS_MID_REG_DEFAULT;
    registerMap[CH4_OFS_LSB_REG_ADDRESS]    =   CH4_OFS_LSB_REG_DEFAULT;
    registerMap[CH4_GAN_MSB_REG_ADDRESS]    =   CH4_GAN_MSB_REG_DEFAULT;
    registerMap[CH4_GAN_MID_REG_ADDRESS]    =   CH4_GAN_MID_REG_DEFAULT;
    registerMap[CH4_GAN_LSB_REG_ADDRESS]    =   CH4_GAN_LSB_REG_DEFAULT;
    registerMap[CH5_CFG1_REG_ADDRESS]       =   CH5_CFG1_REG_DEFAULT;
    registerMap[CH5_CFG2_REG_ADDRESS]       =   CH5_CFG2_REG_DEFAULT;
    registerMap[CH5_OFS_MSB_REG_ADDRESS]    =   CH5_OFS_MSB_REG_DEFAULT;
    registerMap[CH5_OFS_MID_REG_ADDRESS]    =   CH5_OFS_MID_REG_DEFAULT;
    registerMap[CH5_OFS_LSB_REG_ADDRESS]    =   CH5_OFS_LSB_REG_DEFAULT;
    registerMap[CH5_GAN_MSB_REG_ADDRESS]    =   CH5_GAN_MSB_REG_DEFAULT;
    registerMap[CH5_GAN_MID_REG_ADDRESS]    =   CH5_GAN_MID_REG_DEFAULT;
    registerMap[CH5_GAN_LSB_REG_ADDRESS]    =   CH5_GAN_LSB_REG_DEFAULT;
    registerMap[CH6_CFG1_REG_ADDRESS]       =   CH6_CFG1_REG_DEFAULT;
    registerMap[CH6_CFG2_REG_ADDRESS]       =   CH6_CFG2_REG_DEFAULT;
    registerMap[CH6_OFS_MSB_REG_ADDRESS]    =   CH6_OFS_MSB_REG_DEFAULT;
    registerMap[CH6_OFS_MID_REG_ADDRESS]    =   CH6_OFS_MID_REG_DEFAULT;
    registerMap[CH6_OFS_LSB_REG_ADDRESS]    =   CH6_OFS_LSB_REG_DEFAULT; 
    registerMap[CH6_GAN_MSB_REG_ADDRESS]    =   CH6_GAN_MSB_REG_DEFAULT;
    registerMap[CH6_GAN_MID_REG_ADDRESS]    =   CH6_GAN_MID_REG_DEFAULT;
    registerMap[CH6_GAN_LSB_REG_ADDRESS]    =   CH6_GAN_LSB_REG_DEFAULT;
    registerMap[CH7_CFG1_REG_ADDRESS]       =   CH7_CFG1_REG_DEFAULT;
    registerMap[CH7_CFG2_REG_ADDRESS]       =   CH7_CFG2_REG_DEFAULT;
    registerMap[CH7_OFS_MSB_REG_ADDRESS]    =   CH7_OFS_MSB_REG_DEFAULT;
    registerMap[CH7_OFS_MID_REG_ADDRESS]    =   CH7_OFS_MID_REG_DEFAULT;
    registerMap[CH7_OFS_LSB_REG_ADDRESS]    =   CH7_OFS_LSB_REG_DEFAULT; 
    registerMap[CH7_GAN_MSB_REG_ADDRESS]    =   CH7_GAN_MSB_REG_DEFAULT;
    registerMap[CH7_GAN_MID_REG_ADDRESS]    =   CH7_GAN_MID_REG_DEFAULT;
    registerMap[CH7_GAN_LSB_REG_ADDRESS]    =   CH7_GAN_LSB_REG_DEFAULT;

//  end
}