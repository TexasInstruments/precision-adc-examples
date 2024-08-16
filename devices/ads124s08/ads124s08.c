/**
 * 
 * @file ads124s08.c
 *
 * @brief  ADS124S08 Low level routines using TI Drivers
 * 
 * @copyright Copyright (C) 2019-22 Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved. 
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
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <assert.h>

#include "hal.h"
#include "ADS124S08.h"
#include "crc.h"

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

/************************************************************************************//**
 *
 * @brief getRegisterValue()
 *          Getter function to access the registerMap array outside of this module
 *
 * @param[in]	address	The 8-bit register address
 *
 * @return 		The 8-bit register value
 */
uint8_t getRegisterValue( uint8_t address )
{
    assert( address < NUM_REGISTERS );
    return registerMap[address];
}

/************************************************************************************//**
 *
 * @brief adcStartupRoutine()
 *          Startup function to be called before communicating with the ADC
 *
 * @param[in]   *spiHdl    SPI_Handle pointer for TI Drivers
 *
 * @return      true for successful initialization
 *              false for unsuccessful initialization
 */
 bool adcStartupRoutine( SPI_Handle spiHdl )
{
    uint8_t initRegisterMap[NUM_REGISTERS] = { 0 };
    uint8_t status, i;

    // Provide additional delay time for power supply settling
     delay_us( DELAY_2p2MS );

    // Toggle nRESET pin to assure default register settings.
    toggleRESET();

    // Must wait 4096 tCLK after reset
    delay_us( DELAY_4096TCLK );

    status = readSingleRegister( spiHdl, REG_ADDR_STATUS );
    if ( (status & ADS_nRDY_MASK) ) {
        return( false );                      // Device not ready
    }

    // Ensure internal register array is initialized
    restoreRegisterDefaults();

    // Configure initial device register settings here
    writeSingleRegister( spiHdl, REG_ADDR_STATUS, 0x00 );    // Reset POR event

    // Create temporary array based on desired configuration
    for( i = 0 ; i < NUM_REGISTERS ; i++) {
        initRegisterMap[i] = registerMap[i];
    }

    // Read back all registers, except for status register.
    readMultipleRegisters( spiHdl, REG_ADDR_ID, NUM_REGISTERS );
    for ( i = REG_ADDR_STATUS; i < REG_ADDR_SYS - REG_ADDR_STATUS + 1; i++ ) {
        if ( i == REG_ADDR_STATUS )
            continue;
        if ( initRegisterMap[i] != registerMap[i] )
            return( false );
    }
    return( true );
}

/************************************************************************************//**
 *
 * @brief readSingleRegister()
 *          Reads contents of a single register at the specified address
 *
 * @param[in]   spiHdl  SPI_Handle from TI Drivers
 * @param[in]	address Address of the register to be read
 *
 * @return 		8-bit register contents
 */
uint8_t readSingleRegister( SPI_Handle spiHdl, uint8_t address )
{
    /* Initialize arrays */
    uint8_t DataTx[COMMAND_LENGTH + 1] = { OPCODE_RREG | (address & OPCODE_RWREG_MASK), 0, 0 };
    uint8_t DataRx[COMMAND_LENGTH + 1] = { 0, 0, 0 };


    /* Check that the register address is in range */
    assert( address < NUM_REGISTERS );

    /* Build TX array and send it */
    spiSendReceiveArrays( spiHdl, DataTx, DataRx, COMMAND_LENGTH + 1);

    /* Update register array and return read result*/
    registerMap[address] = DataRx[COMMAND_LENGTH];
    return DataRx[COMMAND_LENGTH];
}

/************************************************************************************//**
 *
 * @brief readMultipleRegisters()
 *          Reads a group of registers starting at the specified address
 *          NOTE: Use getRegisterValue() to retrieve the read values
 *
 * @param[in]   spiHdl          SPI_Handle from TI Drivers
 * @param[in]	startAddress	Register address to start reading
 * @param[in]	count 			Number of registers to read
 *
 * @return 		None
 */
void readMultipleRegisters( SPI_Handle spiHdl, uint8_t startAddress, uint8_t count )
{
    uint8_t DataTx[COMMAND_LENGTH + NUM_REGISTERS] = { 0 };
    uint8_t DataRx[COMMAND_LENGTH + NUM_REGISTERS] = { 0 };
    uint8_t i;

    /* Check that the register address and count are in range */
    assert( startAddress + count <= NUM_REGISTERS );

    // Read register data bytes
    DataTx[0] = OPCODE_RREG | (startAddress & OPCODE_RWREG_MASK);
    DataTx[1] = count - 1;

    spiSendReceiveArrays( spiHdl, DataTx, DataRx, COMMAND_LENGTH + count);

    for ( i = 0; i < count; i++ ) {
        // Store received register data into internal registerMap copy
        registerMap[i+startAddress] = DataRx[COMMAND_LENGTH + i];
    }
}

/************************************************************************************//**
 *
 * @brief writeSingleRegister()
 *          Write data to a single register at the specified address
 *
 * @param[in]   spiHdl      SPI_Handle from TI Drivers
 * @param[in]	address 	Register address to write
 * @param[in]	data 		8-bit data to write
 *
 * @return 		None
 */
void writeSingleRegister( SPI_Handle spiHdl, uint8_t address, uint8_t data )
{
    /* Initialize arrays */
    uint8_t DataTx[COMMAND_LENGTH + 1] = { OPCODE_WREG | (address & OPCODE_RWREG_MASK), 0, data};
    uint8_t DataRx[COMMAND_LENGTH + 1] = { 0 };

    /* Check that the register address is in range */
    assert( address < NUM_REGISTERS );

    /* Build TX array and send it */
    spiSendReceiveArrays( spiHdl, DataTx, DataRx, COMMAND_LENGTH + 1 );

    /* Update register array */
    registerMap[address] = DataTx[COMMAND_LENGTH];
}

/************************************************************************************//**
 *
 * @brief writeMultipleRegisters()
 *          Write data to a group of registers
 *          NOTES: Use getRegisterValue() to retrieve the written values.
 *          Registers should be re-read after a write operation to ensure proper configuration.
 *
 * @param[in]   spiHdl          SPI_Handle from TI Drivers
 * @param[in]	startAddress 	Register address to start writing
 * @param[in]	count 			Number of registers to write
 * @param[in]	regData			Array that holds the data to write, where element zero 
 *      						is the data to write to the starting address.
 *
 * @return 		None
 */
void writeMultipleRegisters( SPI_Handle spiHdl, uint8_t startAddress, uint8_t count, uint8_t regData[] )
{
    uint8_t DataTx[COMMAND_LENGTH + NUM_REGISTERS] = { 0 };
    uint8_t DataRx[COMMAND_LENGTH + NUM_REGISTERS] = { 0 };
    uint8_t i, j = 0;

    /* Check that the register address and count are in range */
    assert( startAddress + count <= NUM_REGISTERS );

    /* Check that regData is not a NULL pointer */
    assert( regData );

    DataTx[0] = OPCODE_WREG | (startAddress & OPCODE_RWREG_MASK);
    DataTx[1] = count - 1;
    for ( i = startAddress; i < startAddress + count; i++ ) {
        DataTx[COMMAND_LENGTH + j++] = regData[i];
        registerMap[i] = regData[i];
    }

    // SPI communication
    spiSendReceiveArrays( spiHdl, DataTx, DataRx, COMMAND_LENGTH + count );
}

/************************************************************************************//**
 *
 * @brief sendCommand()
 *          Sends the specified SPI command to the ADC
 *
 * @param[in]   spiHdl      SPI_Handle from TI Drivers
 * @param[in]	op_code 	SPI command byte
 *
 * @return 		None
 */
void sendCommand(SPI_Handle spiHdl, uint8_t op_code)
{
    /* Assert if this function is used to send any of the following commands */
    assert( OPCODE_RREG         != op_code );    /* Use "readSingleRegister()"  or "readMultipleRegisters()"  */
    assert( OPCODE_WREG         != op_code );    /* Use "writeSingleRegister()" or "writeMultipleRegisters()" */

    /* SPI communication */
    spiSendReceiveByte( spiHdl, op_code );

    // Check for RESET command
    if (OPCODE_RESET == op_code)
    {
        // Must wait 4096 tCLK after reset
        delay_us( DELAY_4096TCLK );

        /* Update register array to keep software in sync with device */
        restoreRegisterDefaults();
    }
}

/************************************************************************************//**
 *
 * @brief startConversions()
 *        	Wakes the device from power-down and starts continuous conversions
 *            by setting START pin high or sending START Command
 *
 * @param[in]   spiHdl      SPI_Handle from TI Drivers
 *
 * @return 		None
 */
void startConversions( SPI_Handle spiHdl )
{
	// Wakeup device if in POWERDOWN
    sendWakeup( spiHdl );

#ifdef START_PIN_CONTROLLED     // If defined in ADS124S08.h
     /* Begin continuous conversions */
    setSTART( HIGH );
#else
    sendSTART( spiHdl );
#endif    
}

/************************************************************************************//**
 *
 * @brief stopConversions()
 *          Stops continuous conversions by setting START pin low or sending STOP Command
 *
 * @param[in]   spiHdl      SPI_Handle from TI Drivers
 *
 * @return      None
 */
void stopConversions( SPI_Handle spiHdl )
{
     /* Stop continuous conversions */
#ifdef START_PIN_CONTROLLED     // If defined in ADS124S08.h
    setSTART( LOW );
#else
    sendSTOP( spiHdl );
#endif    
}

/************************************************************************************//**
 *
 * @brief resetADC()
 *          Resets ADC by setting RESET pin low or sending RESET Command
 *
 * @param[in]   spiHdl      SPI_Handle from TI Drivers
 *
 * @return      None
 */
void resetADC( SPI_Handle spiHdl )
{
     /* Reset ADC */
#ifdef RESET_PIN_CONTROLLED     // If defined in ADS124S08.h
    toggleRESET();
#else
    sendRESET( spiHdl );
#endif
    // Must wait 4096 tCLK after reset
    delay_us( DELAY_4096TCLK );

    // Update the local copy to say in sync
    restoreRegisterDefaults();
}

/************************************************************************************//**
 *
 * @brief readConvertedData()
 *          Sends the read command and retrieves STATUS (if enabled) and data
 *          NOTE: Call this function after /DRDY goes low and specify the 
 *          the number of bytes to read and the starting position of data
 *          
 * @param[in]   spiHdl      SPI_Handle from TI Drivers
 * @param[in]	status[] 	Pointer to location where STATUS byte will be stored
 * @param[in]	mode 		Direct or Command read mode
 * 
 * @return 		32-bit sign-extended conversion result (data only)
 */
int32_t readConvertedData( SPI_Handle spiHdl, uint8_t status[], readMode mode )
{
    uint8_t DataTx[RDATA_COMMAND_LENGTH + STATUS_LENGTH + DATA_LENGTH + CRC_LENGTH] = { 0 };    // Initialize all array elements to 0
    uint8_t DataRx[RDATA_COMMAND_LENGTH + STATUS_LENGTH + DATA_LENGTH + CRC_LENGTH] = { 0 };    
    uint8_t byteLength;
    uint8_t dataPosition;
    uint8_t byte_options;
    uint8_t data[5];
    bool    status_byte_enabled = 0;
	int32_t signByte, upperByte, middleByte, lowerByte;

    // Status Byte is sent if SENDSTAT bit of SYS register is set
    byte_options = IS_SENDSTAT_SET << 1 | IS_CRC_SET;
    switch ( byte_options ) {
    	case 0:							// No STATUS and no CRC
			byteLength   = DATA_LENGTH;
			dataPosition = 0;
    		break;
    	case 1:							// No STATUS and CRC
    		byteLength 	 = DATA_LENGTH + CRC_LENGTH;
			dataPosition = 0;
   			break;
    	case 2:							// STATUS and no CRC
    		byteLength   = STATUS_LENGTH + DATA_LENGTH;
			dataPosition = 1;
			status_byte_enabled = 1;
    		break;
    	case 3:							// STATUS and CRC
    		byteLength   = STATUS_LENGTH + DATA_LENGTH + CRC_LENGTH;
			dataPosition = 1;
			status_byte_enabled = 1;
   			break;
    }
	
	if ( mode == COMMAND ) {
        DataTx[0]     = OPCODE_RDATA;
        byteLength   += 1;
        dataPosition += 1;
	}
    spiSendReceiveArrays( spiHdl, DataTx, DataRx, byteLength );

    // Parse returned SPI data
    /* Check if STATUS byte is enabled and if we have a valid "status" memory pointer */
    if ( status_byte_enabled && status ) {
        status[0] = DataRx[dataPosition - 1];
    }

    /* Return the 32-bit sign-extended conversion result */
    if ( DataRx[dataPosition] & 0x80u ) {
    	signByte = 0xFF000000; 
    } else { 
    	signByte = 0x00000000; 
    }

    if ( IS_CRC_SET ){
        if(IS_SENDSTAT_SET){
            data[0] = DataRx[dataPosition - 1]; // status
            data[1] = DataRx[dataPosition];     // msb
            data[2] = DataRx[dataPosition + 1]; // mid
            data[3] = DataRx[dataPosition + 2]; // lsb
            data[4] = DataRx[dataPosition + 3]; // crc
            bool error = (bool) getCRC(data, 5, CRC_INITIAL_SEED);

            if ( error ) {
                // if error, report and handle the error
                while (1);
            }
        }
        else {
            data[0] = DataRx[dataPosition];     // msb
            data[1] = DataRx[dataPosition + 1]; // mid
            data[2] = DataRx[dataPosition + 2]; // lsb
            data[3] = DataRx[dataPosition + 3]; // crc
            bool error = (bool) getCRC(data, 4, CRC_INITIAL_SEED);

            if ( error ) {
                // if error, report and handle the error
                while (1);
            }
        }
    }
	upperByte 	= ((int32_t) DataRx[dataPosition] & 0xFF) << 16;
	middleByte  = ((int32_t) DataRx[dataPosition + 1] & 0xFF) << 8;
	lowerByte	= ((int32_t) DataRx[dataPosition + 2] & 0xFF);

	return ( signByte + upperByte + middleByte + lowerByte );
}

/************************************************************************************//**
 *
 * @brief restoreRegisterDefaults()
 *          Updates the registerMap[] array to its default values
 *          NOTES: If the MCU keeps a copy of the ADC register settings in memory,
 *          then it is important to ensure that these values remain in sync with the
 *          actual hardware settings. In order to help facilitate this, this function
 *          should be called after powering up or resetting the device (either by
 *          hardware pin control or SPI software command).
 *          Reading back all of the registers after resetting the device will
 *          accomplish the same result.
 *
 * @return 		None
 */
 void restoreRegisterDefaults( void )
{
	/* Default register settings */
	registerMap[REG_ADDR_ID]       = ID_DEFAULT;
	registerMap[REG_ADDR_STATUS]   = STATUS_DEFAULT;
	registerMap[REG_ADDR_INPMUX]   = INPMUX_DEFAULT;
	registerMap[REG_ADDR_PGA]      = PGA_DEFAULT;
	registerMap[REG_ADDR_DATARATE] = DATARATE_DEFAULT;
	registerMap[REG_ADDR_REF]      = REF_DEFAULT;
	registerMap[REG_ADDR_IDACMAG]  = IDACMAG_DEFAULT;
	registerMap[REG_ADDR_IDACMUX]  = IDACMUX_DEFAULT;
	registerMap[REG_ADDR_VBIAS]    = VBIAS_DEFAULT;
	registerMap[REG_ADDR_SYS]      = SYS_DEFAULT;
	registerMap[REG_ADDR_OFCAL0]   = OFCAL0_DEFAULT;
	registerMap[REG_ADDR_OFCAL1]   = OFCAL1_DEFAULT;
	registerMap[REG_ADDR_OFCAL2]   = OFCAL2_DEFAULT;
	registerMap[REG_ADDR_FSCAL0]   = FSCAL0_DEFAULT;
	registerMap[REG_ADDR_FSCAL1]   = FSCAL1_DEFAULT;
	registerMap[REG_ADDR_FSCAL2]   = FSCAL2_DEFAULT;
	registerMap[REG_ADDR_GPIODAT]  = GPIODAT_DEFAULT;
	registerMap[REG_ADDR_GPIOCON]  = GPIOCON_DEFAULT;
}
