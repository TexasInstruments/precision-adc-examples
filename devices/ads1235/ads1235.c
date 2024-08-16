/**
 * @file ads1235.c
 *
 * @brief This file contains all basic communication and device setup for the ADS1235.
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

#include "ADS1235.h"

/* Initialize global variables */
uint8_t ADC_RegisterMap[NUM_REGISTERS];
uint8_t ADC_DontCare = 0x00;


/**
 * \fn void adcStartupRoutine(void)
 * \brief Example start up sequence for the ADS1235
 *
 * NOTE: Before calling this function, the device must be powered,
 * the SPI/GPIO pins of the MCU must have already been configured,
 * and (if applicable) the external clock source should be provided to CLKIN.
 */
void adcStartupRoutine(void)
{
	/* (OPTIONAL) Provide additional delay time for power supply settling */
	delay_ms(50);

	/* (REQUIRED) Set nPWDN and nRESET pins high for ADC operation */
	setPWDN(HIGH);
	setRESET();

	/* (OPTIONAL) Start ADC conversions with HW pin control.
	 * NOTE: Using the HW pin control here to monitor the nDRDY falling edge.
	 */
	setSTART(HIGH);

	/* (REQUIRED) NO SPI COMMANDS ARE ALLOWED PRIOR TO nDRDY RISING EDGE!
	 * In case the MCU cannot monitor the nDRDY rising edge during startup,
	 * for example if the MCU is performing other startup tasks at this time,
	 * use the nDRDY falling edge as the indicator that the device is ready
	 * for communication as is done here.
	 *
	 * Alternatively, insert a 50 ms delay here if nDRDY pin is not monitored.
	 */
	pollForDRDY(50);

	/* (OPTIONAL) Start ADC conversions with the SPI command.
	 * This can occur any time after the nDRDY rising edge,
	 * but is not needed if the START pin has already been set HIGH.
	 *
	 * sendCommand(START_OPCODE);
	 */

	/* (OPTIONAL) Toggle nRESET pin to assure default register settings. */
	/* NOTE: This also ensures that the device registers are unlocked.	 */
	toggleRESET();

	/* (OPTIONAL) Configure initial device register settings here */
	uint8_t initRegisterMap[NUM_REGISTERS];
	initRegisterMap[REG_ADDR_ID] 			= 	0x00;           /* NOTE: This a read-only register */
	initRegisterMap[REG_ADDR_STATUS] 		= 	STATUS_CLEAR;	/* NOTE: This a non-default setting */
	initRegisterMap[REG_ADDR_MODE0]			= 	MODE0_DEFAULT;
	initRegisterMap[REG_ADDR_MODE1] 		= 	MODE1_DEFAULT;
	initRegisterMap[REG_ADDR_MODE2] 		= 	MODE2_DEFAULT;
	initRegisterMap[REG_ADDR_MODE3] 		= 	MODE3_DEFAULT;
	initRegisterMap[REG_ADDR_REF] 			= 	REF_DEFAULT;
	initRegisterMap[REG_ADDR_OFCAL0] 		= 	OFCAL0_DEFAULT;
	initRegisterMap[REG_ADDR_OFCAL1] 		= 	OFCAL1_DEFAULT;
	initRegisterMap[REG_ADDR_OFCAL2] 		= 	OFCAL2_DEFAULT;
	initRegisterMap[REG_ADDR_FSCAL0] 		= 	FSCAL0_DEFAULT;
	initRegisterMap[REG_ADDR_FSCAL1] 		= 	FSCAL1_DEFAULT;
	initRegisterMap[REG_ADDR_FSCAL2] 		= 	FSCAL2_DEFAULT;
	initRegisterMap[REG_ADDR_RES_0] 		= 	RES_0_DEFAULT;
	initRegisterMap[REG_ADDR_RES_1] 		= 	RES_1_DEFAULT;
	initRegisterMap[REG_ADDR_RESERVED] 		= 	RESERVED_DEFAULT;
	initRegisterMap[REG_ADDR_PGA] 			= 	PGA_DEFAULT;
	initRegisterMap[REG_ADDR_INPMUX]		= 	INPMUX_DEFAULT;

	/* (OPTIONAL) Write to all registers */
	uint8_t startIndex = REG_ADDR_STATUS;
	uint8_t count = NUM_REGISTERS - startIndex;
	writeMultipleRegisters(startIndex, count, &initRegisterMap[startIndex]);

	/* (OPTIONAL) Read back all registers */
	readMultipleRegisters(startIndex, count, &ADC_RegisterMap[startIndex]);

	/* (OPTIONAL) Check STATUS register for faults */
	pollForDRDY(50);    /* Avoids data not new STATUS flag */
	uint8_t status_register = readSingleRegister(REG_ADDR_STATUS);
	checkStatus(status_register);
}


/**
 * \fn uint8_t readSingleRegister(uint8_t addr)
 * \brief Reads contents of a single register at the specified address
 * \param addr address of the register to read
 * \return 8-bit register read result
 */
uint8_t readSingleRegister(uint8_t addr)
{
	/* Check that the register address is in range */
	assert(addr < NUM_REGISTERS);

	uint8_t DataTx[6];
	uint8_t DataRx[6] = { 0 };
	uint8_t byteLength = (CRC_BYTE_ENABLED ? 6 : 3);
	uint8_t dataPosition = (CRC_BYTE_ENABLED ? 4 : 2);

	/* Build TX array and send it */
	DataTx[0] = OPCODE_RREG + (addr & 0x1F);
	DataTx[1] = ADC_DontCare;
	DataTx[2] = ADC_DontCare;
	if (CRC_BYTE_ENABLED)
	{
		DataTx[2] = calculateCRC(&DataTx[0], 2);	/* Compute CRC-2 */
		DataTx[3] = ADC_DontCare;
		DataTx[4] = ADC_DontCare;
		DataTx[5] = ADC_DontCare;
	}
	SPI_SendReceive(DataTx, DataRx, byteLength);

	/* Validate command response */
	if (validateSPI(DataTx, DataRx, OPCODE_RREG))
	{
		/* Handle SPI error */
		handleSPIerror(DataTx, DataRx, byteLength, "RREG");
	}
	else
	{
		/* Update register array */
		ADC_RegisterMap[addr] = DataRx[dataPosition];
	}

	return DataRx[dataPosition];
}


/**
 * \fn void readMultipleRegisters(uint8_t addr, uint8_t count, uint8_t *data)
 * \brief Reads a group of registers starting at the specified address
 *
 * NOTE: For simplicity, this implementation calls
 * "readSingleRegister()" in a for loop and will toggle /CS
 * between each RREG command. This is slightly slower than
 * holding /CS low but ensures that the SPI gets reset between commands.
 *
 * \param addr register address from which we start reading
 * \param count is the number of registers we want to read
 * \param *data points to a location in memory to store the register data
 */
void readMultipleRegisters(uint8_t addr, uint8_t count, uint8_t data[])
{
	/* Validate input parameters */
	assert(count > 0);
	assert((addr + count) <= NUM_REGISTERS);

	uint8_t i;
	for(i = addr; i < (addr + count); i++)
	{
		data[i] = readSingleRegister(i);
	}
}


/**
 * \fn void writeSingleRegister(uint8_t addr, uint8_t data)
 * \brief Writes data to a single register
 * NOTE: This command will be ignored if device registers are locked.
 *
 * \param addr 8-bit address of the register to which we start writing
 * \param data to be written
 */
void writeSingleRegister(uint8_t addr, uint8_t data)
{
	/* Check that the register address is in range */
	assert(addr < NUM_REGISTERS);

	uint8_t DataTx[4];
	uint8_t DataRx[4] = { 0 };
	uint8_t byteLength = (CRC_BYTE_ENABLED ? 4 : 2);

	/* Build TX array and send it */
	DataTx[0] = OPCODE_WREG + (addr & 0x1F);
	DataTx[1] = data;
	if (CRC_BYTE_ENABLED)
	{
		DataTx[2] = calculateCRC(&DataTx[0], 2);	/* Compute CRC-2 */
		DataTx[3] = ADC_DontCare;
	}
	SPI_SendReceive(DataTx, DataRx, byteLength);

	/* Handle SPI errors */
	if (validateSPI(DataTx, DataRx, OPCODE_WREG))
	{
		handleSPIerror(DataTx, DataRx, byteLength, "WREG");
	}
	else if (!REGISTER_LOCK)
	{
		/* If registers are unlocked, update the global register map variable */
	    ADC_RegisterMap[addr] = data;
	}
}


/**
 * \fn void writeMultipleRegisters(uint8_t addr, uint8_t count, uint8_t *data)
 * \brief Writes to a group of registers starting at the specified address
 *
 * NOTE: For simplicity, this implementation calls
 * "writeSingleRegister()" in a for loop and will toggle /CS
 * between each WREG command. This is slightly slower than
 * holding /CS low but ensures that the SPI gets reset between commands.
 *
 * \param addr starting register address byte
 * \param count total number of register to write
 * \param *data pointer to array of values to write (array must map 1-to-1 with the device registers)
 */
void writeMultipleRegisters(uint8_t addr, uint8_t count, const uint8_t data[])
{
	/* Check that register map address range is not exceeded */
	assert( (addr + count) <= NUM_REGISTERS );

	uint8_t i;
	for (i = addr; i < (addr + count); i++)
	{
		writeSingleRegister(addr + i, data[i]);
	}
}


/**
 * \fn bool sendCommand(uint8_t op_code)
 * \brief Sends the specified SPI command to the ADS1235
 * \param op_code SPI command byte
 * \returns boolean which indicates if an error occurred
 */
bool sendCommand(uint8_t op_code)
{
	/* Assert if this function is used to send any of the following commands */
	assert(OPCODE_RREG != op_code);		/* Use "readSingleRegister()" 	*/
	assert(OPCODE_WREG != op_code);		/* Use "writeSingleRegister()" 	*/
	assert(OPCODE_RDATA != op_code);	/* Use "readData()" 			*/
	assert(OPCODE_LOCK != op_code);		/* Use "lockRegisters()" 		*/
	assert(OPCODE_UNLOCK != op_code);	/* Use "unlockRegisters()" 		*/

	bool spiError;
	uint8_t DataTx[4];
	uint8_t DataRx[4] = { 0 };
	uint8_t byteLength = (CRC_BYTE_ENABLED ? 4 : 2);

	/* Build TX array and send it */
	DataTx[0] = op_code;
	DataTx[1] = ADC_DontCare;
	if(CRC_BYTE_ENABLED)
	{
		DataTx[2] = calculateCRC(&DataTx[0], 2);
		DataTx[3] = ADC_DontCare;
	}
	SPI_SendReceive(DataTx, DataRx, byteLength);

	/* Handle SPI errors */
	spiError = validateSPI(DataTx, DataRx, op_code);
	if (spiError)
	{
		handleSPIerror(DataTx, DataRx, byteLength, "");
	}
	else if (OPCODE_RESET == op_code)
	{
		/* Update register setting array to keep software in sync with device */
		restoreRegisterDefaults();
	}

	return spiError;
}


/**
 * \fn int32_t readData(uint8_t *dStatus, uint8_t *dData, uint8_t *dCRC)
 * \brief Sends the RDATA command and retrieves STATUS, DATA, & CRC bytes
 * \param *dStatus pointer to address where STATUS byte will be stored
 * \param *dData pointer to starting address where data bytes will be stored
 * \param *dCRC pointer to address where CRC byte will be stored
 * \return 32-bit sign-extended conversion result (data only)
 */
int32_t readData(uint8_t status[], uint8_t data[], uint8_t crc[])
{
	uint8_t i;
	uint8_t DataTx[9];
	uint8_t DataRx[9] = { 0 };

	/* Byte length and data position of the RDATA command depends on the mode:
	 * /---------------------------------------------------------\
	 * |  CRC mode	| Status enabled | byteLength | dataPosition |
	 * |---------------------------------------------------------|
	 * |	false   |	  false		 |	   5	  |		 2		 |
	 * |	false   |     true 		 |	   6	  |		 2		 |
	 * |	true    |	  false 	 |	   8	  |		 4		 |
	 * |	true    |	  true		 |	   9	  |		 4		 |
	 * \---------------------------------------------------------/
	 */


	/* Build TX array and send it */

	uint8_t byteLength = 5 + (CRC_BYTE_ENABLED ? 3 : 0) + \
									(STATUS_BYTE_ENABLED ? 1 : 0);

	DataTx[0] = OPCODE_RDATA;

	for (i = 1; i < byteLength; i++)
	{
		DataTx[i] = ADC_DontCare;
	}

	if (CRC_BYTE_ENABLED)
	{
		DataTx[2] = calculateCRC(&DataTx[0], 2);	/* Compute 2-byte CRC */
	}

	SPI_SendReceive(DataTx, DataRx, byteLength);


	/* Handle SPI errors */
	if (validateSPI(DataTx, DataRx, OPCODE_RDATA))
	{
		handleSPIerror(DataTx, DataRx, byteLength, "RDATA");
	}


	/* Parse returned SPI data */

	uint8_t dataPosition = (CRC_BYTE_ENABLED ? 4 : 2);

	if (STATUS_BYTE_ENABLED)
	{
		/* Store STATUS byte to memory? */
		if (status != NULL)	{ status[0] = DataRx[dataPosition]; }

		/* Check for STATUS byte alarms */
		checkStatus(DataRx[dataPosition]);

		/* Increment data position counter */
		dataPosition++;
	}

	/* Store data bytes to memory? */
	if (data != NULL)
	{
		data[0] = DataRx[dataPosition];
		data[1] = DataRx[dataPosition + 1];
		data[2] = DataRx[dataPosition + 2];
	}

	/* Store CRC byte to memory? */
	if (CRC_BYTE_ENABLED && (crc != NULL)) { crc[0] = DataRx[dataPosition + 3]; }


	/* Return the 32-bit sign-extended conversion result */
	int32_t signByte;
	if (DataRx[dataPosition] & 0x80u)	{ signByte = 0xFF000000; }
	else								{ signByte = 0x00000000; }

	int32_t upperByte 	= ((int32_t) DataRx[dataPosition + 0] & 0xFF) << 16;
	int32_t middleByte 	= ((int32_t) DataRx[dataPosition + 1] & 0xFF) << 8;
	int32_t lowerByte	= ((int32_t) DataRx[dataPosition + 2] & 0xFF) << 0;

	return (signByte | upperByte | middleByte | lowerByte);
}


/**
 * \fn bool lockRegisters()
 * \brief Sends the LOCK command and verifies that registers are locked.
 * \return boolean to indicate if an error occurred (0 = no error; 1 = error)
 */
bool lockRegisters(void)
{
	bool b_lock_error;
	uint8_t DataTx[4];
	uint8_t DataRx[4] = { 0 };
	uint8_t byteLength = (CRC_BYTE_ENABLED ? 4 : 2);

	/* Build TX array and send it */
	DataTx[0] = OPCODE_LOCK;
	DataTx[1] = ADC_DontCare;
	if (CRC_BYTE_ENABLED)
	{
		DataTx[2] = calculateCRC(&DataTx[0], 2);
		DataTx[3] = ADC_DontCare;
	}
	SPI_SendReceive(DataTx, DataRx, byteLength);

	/* (OPTIONAL) Check for SPI errors */
	b_lock_error = validateSPI(DataTx, DataRx, OPCODE_LOCK);

	/* (OPTIONAL) Read back the STATUS register and check if LOCK bit is set...
     * readSingleRegister(REG_ADDR_STATUS);
     * if (!REGISTER_LOCK) { b_lock_error = true; }
     */

    /* If the STATUS register is NOT read back,
     * then make sure to manually update the global register map variable... */
	ADC_RegisterMap[REG_ADDR_STATUS]  |= STATUS_LOCK_MASK;

	/* (OPTIONAL) Error handler */
	if (b_lock_error) { handleSPIerror(DataTx, DataRx, byteLength, "LOCK"); }

	return b_lock_error;
}


/**
 * \fn bool unlockRegisters()
 * \brief Sends the UNLOCK command and verifies that registers are unlocked
 * \return boolean that indicates if an SPI communication error occurred
 */
bool unlockRegisters(void)
{
	bool b_unlock_error;
	uint8_t DataTx[4];
	uint8_t DataRx[4] = { 0 };
	uint8_t byteLength = (CRC_BYTE_ENABLED ? 4 : 2);

	/* Build TX array and send it */
	DataTx[0] = OPCODE_UNLOCK;
	DataTx[1] = ADC_DontCare;
	if (CRC_BYTE_ENABLED)
	{
		DataTx[2] = calculateCRC(&DataTx[0], 2);
		DataTx[3] = ADC_DontCare;
	}
	SPI_SendReceive(DataTx, DataRx, byteLength);

	/* (OPTIONAL) Check for SPI errors */
	b_unlock_error = validateSPI(DataTx, DataRx, OPCODE_UNLOCK);

	/* (OPTIONAL) Read the STATUS register and check if LOCK bit is cleared...
	 * readSingleRegister(REG_ADDR_STATUS);
     * if (REGISTER_LOCK) { b_unlock_error = true; }
	 */

    /* If the STATUS register is NOT read back,
     * then make sure to manually update the global register map variable... */
    ADC_RegisterMap[REG_ADDR_STATUS]  &= ~STATUS_LOCK_MASK;

	/* (OPTIONAL) Error handler */
	if (b_unlock_error)
	{
		handleSPIerror(DataTx, DataRx, byteLength, "UNLOCK");
	}

	return b_unlock_error;
}


/**
 * \fn void ads1235_startConversions()
 * \brief Wakes the device from HW & SW power-down modes and starts conversions
 */
void ads1235_startConversions(void)
{
	/* Ensure that the PWDN pin is high before beginning conversions */
	if (check_nPWDN_PIN_LOW) 	{ setPWDN(HIGH); }

	/* Wake the device from software power-down mode, if in this state */
	if (SW_PWDN_MODE)
	{
		uint8_t current_reg_data = ADC_RegisterMap[REG_ADDR_MODE3];
		uint8_t modified_reg_data = current_reg_data & !MODE3_PWDN_MASK;

		writeSingleRegister(REG_ADDR_MODE3, modified_reg_data);
	}

	/* If the START pin is low, send the START command to begin conversions */
	if (check_START_PIN_LOW) 	{ sendCommand(OPCODE_START); }

	/* Otherwise, toggle the START pin to restart ADC conversions */
	else                            { toggleSTART(); }
}


/**
 * \fn uint8_t calculateCRC(uint8_t *dataByte, uint8_t numBytes)
 * \brief Calculates the 8-bit CRC for data words, up to 4 bytes
 *
 * NOTE: This calculation is shown as an example and is not optimized for speed.
 * On a TM4C1294NCPDT, operating with a 120 MHz system clock, this function's
 * execution time is between 6-10 us.
 *
 *
 * --- CRC Calculation Description ---
 * The CRC shift register is initialized to 0xFF and all data is shifted in MSB first, as shown in the
 * diagram below. 'Bit 7' of the CRC shifted register is XOR'd with the next data bit and the result
 * is placed into 'Bit 0' of shift register and also used in additional XOR operations. 'Bit 1' takes
 * the value of 'Bit 0' XOR'd with the result of the first XOR operation. Similarly, 'Bit 2' takes
 * the value of 'Bit 1' XOR'd with the result of the first XOR operation. All other bits in the CRC
 * shift register are shifted, such that 'Bit N' takes on the previous value of 'Bit N-1'.
 *
 * NOTE: If the first XOR operation results in a '0', all other XOR operations retain the value of the
 * previous bit; and as such, can be ignored.
 *
 *                       Bit    Bit    Bit    Bit    Bit    Bit           Bit           Bit
 * CRC Shift register:   |7| << |6| << |5| << |4| << |3| << |2| << XOR << |1| << XOR << |0|
 *                        |                                         ^             ^      ^
 *                        V                                         |             |      |
 * Data IN (MSB 1st) ->> XOR -------------------------------------------------------------
 *
 *
 * \param *dataByte pointer to first element in the data byte array
 * \param numBytes number of bytes (between 1 and 4) used in CRC calculation
 * \return calculated CRC byte
 */
uint8_t calculateCRC(const uint8_t dataBytes[], uint8_t numBytes)
{
	/* Check that "numBytes" is between 1 and 4 */
	assert((numBytes >= 1) && (numBytes <= 4));

	/* Check that "dataBytes" is not a null pointer */
	assert(dataBytes != NULL);

	/* NOTE:
	 * Using "uint_fast8_t" types here instead of "uint8_t" to avoid unnecessary
	 * implicit type conversions. Reference this E2E thread for additional info:
	 * https://e2e.ti.com/support/microcontrollers/msp430/f/166/t/679200
	 */
	uint_fast8_t i;
	uint_fast8_t crc 		= 0xFFu;		/* Initial value of crc register 	 */
    uint_fast8_t crcMSb;                    /* Most significant bit of crc byte  */
	uint_fast8_t poly 		= 0x07u;	    /* CRC polynomial byte               */
	uint_fast8_t shift_by   = 0u;           /* Intermediate variable             */
	uint32_t data 		    = 0u;			/* Data storage variable			 */
	uint32_t msbMask 	    = 0x80000000u;  /* Points to the next data bit 		 */
	uint32_t dataMSb;						/* Most significant bit of data int  */

	/* Construct data word from data bytes */
	for (i = 0; i < numBytes; i++)
	{
	    shift_by = 8 * (numBytes - i - 1);
		data |= (((uint32_t) dataBytes[i]) << shift_by);
	}

	/* Determine the location of the first data byte */
	shift_by = 8 * (4 - numBytes);
	msbMask >>= shift_by;

	/* CRC algorithm */
	while (msbMask > 0)
	{
		// Check MSB's of data and crc
	    dataMSb = data & msbMask;
		crcMSb 	= crc & 0x80u;

		// Shift crc byte
		crc <<= 1;

		// Check if XOR operation of MSbs results in additional XOR operation
		if (dataMSb ^ crcMSb)   { crc ^= poly; }

		/* Shift MSb pointer */
		msbMask >>= 1;
	}

	return crc;
}


/**
 * \fn bool validateSPI(uint8_t *DataTx, uint8_t *DataRx, uint8_t opcode)
 * \brief Checks if returned SPI bytes match their expected values
 *
 * NOTE: This function can be used to help validate SPI communication,
 * even when CRC mode is not enabled.
 *
 * \param *DataTx pointer TX byte array
 * \param *DataRx pointer RX byte array
 * \param opcode The command byte that was sent
 * \return boolean that indicates if an SPI communication error occurred
 */
bool validateSPI(const uint8_t DataTx[], const uint8_t DataRx[], uint8_t opcode)
{
	/* This variable keeps track of whether or not an SPI error was detected */
	bool spiError = false;

	/* DOUT byte 1 should always be 0xFF */
	if (0xFF != DataRx[0]) { spiError = true; }

	/* DOUT byte 2 should always be the repeated command byte */
	if (DataRx[1] != DataTx[0]) { spiError = true; }

	if (!CRC_BYTE_ENABLED)
	{
		return spiError;	/* No additional bytes to check */
	}


	/*
	 * Additional DOUT bytes only occur in CRC mode...
	 */

	/* DOUT byte 3 should always be the repeated "don't care" byte */
	if (DataRx[2] != DataTx[1]) { spiError = true; }

	/* DOUT byte 4 should always be the repeated CRC-2 byte */
	if (DataRx[3] != DataTx[2]) { spiError = true; }

	/* Additional DOUT bytes depend on which SPI command was sent... */
	switch (opcode)
	{
		case OPCODE_RREG:
			/* DOUT byte 5 of RREG command is register data */
			/* DOUT byte 6 of RREG command is  CRC-1 of the register data */
			if (calculateCRC(&DataRx[4], 1) != DataRx[5])
			{
				spiError = true;
			}
		break;

		case OPCODE_RDATA:
			if (!STATUS_BYTE_ENABLED)		/* Status byte disabled */
			{
				/* DOUT bytes 5-7 of RDATA command are data */
				/* DOUT byte 8 of RDATA command is the CRC-3 of the data */
				if (calculateCRC(&DataRx[4], 3) != DataRx[7])
				{
					spiError = true;
				}
			}
			else 							/* Status byte enabled */
			{
				/* DOUT bytes 5-8 of RDATA command are status + data */
				/* DOUT byte 9 of RDATA command is CRC-4 of status + data */
				if (calculateCRC(&DataRx[4], 4) != DataRx[8])
				{
					spiError = true;
				}
			}
		break;

		default:
		break;
	}

	return spiError;
}


/**
 * \fn void restoreRegisterDefaults(void)
 * \brief Updates the ADC_RegisterMap[] array to its default values.
 *
 * NOTES:
 * - If the MCU keeps a copy of the ADS1235 register settings in memory,
 * then it is important to ensure that these values remain in sync with the
 * actual hardware settings. In order to help facilitate this, this function
 * should be called after powering up or resetting the device (either by
 * hardware pin control or SPI software command).
 *
 * - Reading back all of the registers after resetting the device can
 * accomplish the same result; however, this might be problematic if the
 * device was previously in CRC mode since resetting the device exits this mode.
 * If the MCU is not aware of this mode change, then read register commands
 * will return invalid data due to the expectation of data appearing in a
 * different byte position.
 */
void restoreRegisterDefaults(void)
{
	ADC_RegisterMap[REG_ADDR_ID] 			= 	ID_DEV_ADS1235 | ID_REV_A;
	ADC_RegisterMap[REG_ADDR_STATUS] 		= 	STATUS_DEFAULT;
	ADC_RegisterMap[REG_ADDR_MODE0]			= 	MODE0_DEFAULT;
	ADC_RegisterMap[REG_ADDR_MODE1] 		= 	MODE1_DEFAULT;
	ADC_RegisterMap[REG_ADDR_MODE2] 		= 	MODE2_DEFAULT;
	ADC_RegisterMap[REG_ADDR_MODE3] 		= 	MODE3_DEFAULT;
	ADC_RegisterMap[REG_ADDR_REF] 			= 	REF_DEFAULT;
	ADC_RegisterMap[REG_ADDR_OFCAL0] 		= 	OFCAL0_DEFAULT;
	ADC_RegisterMap[REG_ADDR_OFCAL1] 		= 	OFCAL1_DEFAULT;
	ADC_RegisterMap[REG_ADDR_OFCAL2] 		= 	OFCAL2_DEFAULT;
	ADC_RegisterMap[REG_ADDR_FSCAL0] 		= 	FSCAL0_DEFAULT;
	ADC_RegisterMap[REG_ADDR_FSCAL1] 		= 	FSCAL1_DEFAULT;
	ADC_RegisterMap[REG_ADDR_FSCAL2] 		= 	FSCAL2_DEFAULT;
	ADC_RegisterMap[REG_ADDR_RES_0] 		= 	RES_0_DEFAULT;
	ADC_RegisterMap[REG_ADDR_RES_1] 		= 	RES_1_DEFAULT;
	ADC_RegisterMap[REG_ADDR_PGA] 			= 	PGA_DEFAULT;
	ADC_RegisterMap[REG_ADDR_INPMUX]		= 	INPMUX_DEFAULT;
}


/*
 * Use Case Examples...
 */

/**
 * \fn void ads1235_measure_internal_temperature_example(void)
 * \brief Measures and computes the internal temperature in degrees Celsius
 * \return the _________TODO:
 */
float ads1235_measure_internal_temperature_example(void)
{
	/* Configure PGA register: default settings  */
	writeSingleRegister(REG_ADDR_PGA, PGA_DEFAULT);

	/* Configure input MUX: Internal temperature monitor  */
	uint8_t regData = (INPMUX_MUXP_TEMP_P | INPMUX_MUXN_TEMP_N);
	writeSingleRegister(REG_ADDR_INPMUX, regData);

	/* Collect data */
	ads1235_startConversions();
	pollForDRDY(100);
	int32_t adcData = readData(NULL, NULL, NULL);

	/* Convert to temperature (deg C) */
	float uVolts = (float) adcData * 0.298023224;
	float temp_C = 25.0 + ((uVolts - 122400.0) / 420.0);

	return temp_C;
}
