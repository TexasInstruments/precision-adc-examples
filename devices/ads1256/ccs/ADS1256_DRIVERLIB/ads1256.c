/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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

#include "ADS1255_7.h"

// Global Variables
uint8_t RegData_LR[NUM_REGISTERS];		// Stores "Last Read" register data
uint8_t RegData_LW[NUM_REGISTERS];		// Stores "Last Write" register data
bool RDATACmode = false;

extern void DRDY_int(void);


/* Sets the state of the ADC's /CS pin:
*		0 = LOW 	(active)
*		1 = HIGH	(not selected)
*
*	Asserts:
*		'state' must be '0' or '1'
*/
void set_adc_CS(uint8_t logicLevel)
{
	if (0 == logicLevel)
	{
		GPIO_setOutputLowOnPin(CS_PORT, CS_PIN);
			__delay_cycles(TD_CSSC_DELAY);					//td(CSSC) delay
	}
	else if (1 == logicLevel)
	{
			__delay_cycles(TD_SCCS_DELAY);					//td(SCCS) delay
		GPIO_setOutputHighOnPin(CS_PORT, CS_PIN);

	}
	else assert(0);										//Aborts program if invalid argument is used
}


/*
 * Initializes device for use in the ADS1257BRD.
 *
 * \param *device The shadow instance of the device to initialize
 * \param SPIBASE Address of the Tiva SSI module which will be used to
 * communicate with the device
 *
 * \return True if device is in correct hardware defaults and is connected
 *
 */
uint8_t InitDevice(void)
{

	__delay_cycles(TD_STARTUP_DELAY);	// Startup delay

	// TODO: Set GPIO pin logic levels (if different than initial settings configured in "hal.c")
	// TODO: Check if DRDY is toggling - if not notify user that an external clock must be usred for ADC CLK

	readRegs(0, NUM_REGISTERS);				// Read all registers (results are stored in "RegData_LR")

	return 1;
}



// receive byte, simultaneously send data
unsigned char adcXferByte(unsigned char cData)
{
		while(USCI_B_SPI_isBusy(USCI_B0_BASE));
	USCI_B_SPI_transmitData(USCI_B0_BASE, cData);
	while(USCI_B_SPI_isBusy(USCI_B0_BASE));
	return USCI_B_SPI_receiveData(USCI_B0_BASE);
}

void reset_adc_hw(void)
{
	GPIO_setOutputLowOnPin(RST_PORT, RST_PIN);
	__delay_cycles(TW_PDL_DELAY);						//tw(PDL) delay
	GPIO_setOutputHighOnPin(RST_PORT, RST_PIN);
}

void reset_adc_sclk(void)
{
	set_adc_CS(0);
	GPIO_setOutputLowOnPin(SPI_PORT, SCLK_PIN);
    GPIO_setAsOutputPin(SPI_PORT, SCLK_PIN);
    GPIO_setOutputHighOnPin(SPI_PORT, SCLK_PIN);
    	__delay_cycles(T12_DELAY);						//tw(PDL) delay
    GPIO_setOutputLowOnPin(SPI_PORT, SCLK_PIN);
    	__delay_cycles(T13_DELAY);						//tw(PDL) delay
	GPIO_setOutputHighOnPin(SPI_PORT, SCLK_PIN);
		__delay_cycles(T14_DELAY);						//tw(PDL) delay
	GPIO_setOutputLowOnPin(SPI_PORT, SCLK_PIN);
		__delay_cycles(T13_DELAY);
	GPIO_setOutputHighOnPin(SPI_PORT, SCLK_PIN);
		__delay_cycles(T15_DELAY);
	GPIO_setOutputLowOnPin(SPI_PORT, SCLK_PIN);
    GPIO_setAsPeripheralModuleFunctionOutputPin(SPI_PORT, SCLK_PIN);
    set_adc_CS(1);
}

void reset_adc_sw(void)
{
	set_adc_CS(0);
	adcXferByte(RESET_OPCODE);
	set_adc_CS(1);
}

////9.4.5 Power-Down Mode
////Holding the SYNC/PDWN pin low for 20 DRDY cycles activates the Power-Down mode. During Power-Down
////mode, all circuitry is disabled including the clock output.
////To exit Power-Down mode, take the SYNC/PDWN pin high. Upon exiting from Power-Down mode 8192 t(CLKIN)
////cycles are needed before conversions begin.
//void pwdn_adc_hw(void)
//{
//	GPIO_setOutputLowOnPin(PWDN_PORT, PWDN_PIN);
//	assert(0); 	// TODO: wait for 20 DRDY cycles
//	GPIO_setOutputHighOnPin(RST_PORT, RST_PIN);
//}


/*
 * Reads a register contents from the specified address and places the result in memory address
 *
 * \param regnum identifies which address to read
 * \param *pdata identifies memory address to place read value
 *
 */
void regRead(uint8_t regnum, uint8_t *p_data)
{
	uint8_t DataTx[3];

	assert(regnum < NUM_REGISTERS);					//Asserts when "regnum" is out of range

	DataTx[0] = RREG_OPCODE + (regnum & 0x0F);
	DataTx[1] = 0;
	DataTx[2] = 0x00;

	set_adc_CS(0);


	//WaitForDRDY();
	if (RDATACmode == true)
	{
		adcXferByte(SDATAC_OPCODE);							// Enter SDATAC mode prior to reading or writing registers to prevent data corruption
		__delay_cycles(TD_SCSC_DELAY_24CLK);
	}

	adcXferByte(DataTx[0]);
	__delay_cycles(TD_SCSC_DELAY_4CLK);

	adcXferByte(DataTx[1]);
	__delay_cycles(TD_DIDO_DELAY_T6);
	p_data[0] = adcXferByte(DataTx[2]);


	set_adc_CS(1);

	if(regnum < NUM_REGISTERS)
		RegData_LR[regnum] = p_data[0];

	return;
}

/*
 * Reads a group of registers starting at the specified address
 *
 * \param regnum identifies SPIBASE Address of the Tiva SSI module which will be used to
 * communicate with the device
 * \param addr_mask 16-bit mask of the register from which we start reading
 * \param num The number of registers we wish to read
 * \param *location pointer to the location in memory to write the data
 *
 */
void readRegs(uint8_t regnum, uint8_t count)
{
	assert( (regnum + count - 1) < NUM_REGISTERS);			// Asserts when false, i.e. "regnum + count - 1" is >= "NUM_REGISTERS"

	uint8_t DataTx[2];

	DataTx[0] = RREG_OPCODE + (regnum & 0x0F);
	DataTx[1] = count - 1;

	set_adc_CS(0);

	if (RDATACmode == true)
	{
		adcXferByte(SDATAC_OPCODE);							// Enter SDATAC mode prior to reading or writing registers to prevent data corruption
		__delay_cycles(TD_SCSC_DELAY_24CLK);
	}

	adcXferByte(DataTx[0]);
	__delay_cycles(TD_SCSC_DELAY_4CLK);
	adcXferByte(DataTx[1]);
	__delay_cycles(TD_DIDO_DELAY_T6);

	for(int i = 0; i < count; i++)
		RegData_LR[regnum + i] = adcXferByte(NOP_OPCODE);	// Store register value in "RegData_LR" array

	set_adc_CS(1);
}


/*
 * Writes a group of registers with the specified data
 *
 * \param SPIBASE Address of the Tiva SSI module which will be used to
 * communicate with the device
 * \param addr_mask 16-bit mask of the register to which we start writing
 * \param num The number of registers we wish to write
 * \param *data pointer to the location in memory from which data is written
 *
 */

void regWrite(unsigned int regnum, unsigned char *pdata)
{
	unsigned long DataTx[3];

	DataTx[0] = WREG_OPCODE | (regnum & 0x0F);
	DataTx[1] = 0x00;
	DataTx[2] = pdata[0];

	set_adc_CS(0);

	if (RDATACmode == true)
	{
		adcXferByte(SDATAC_OPCODE);							// Enter SDATAC mode prior to reading or writing registers to prevent data corruption
		__delay_cycles(TD_SCSC_DELAY_24CLK);
	}

	adcXferByte(DataTx[0]);
	__delay_cycles(TD_SCSC_DELAY_4CLK);					// TODO: Check if necessary
	adcXferByte(DataTx[1]);
	adcXferByte(DataTx[2]);

	if(regnum < NUM_REGISTERS)
		RegData_LW[regnum] = DataTx[2];

	set_adc_CS(1);

	return;
}

void  writeRegs(unsigned int regnum, unsigned int count, unsigned char *data)
{
	int i;
	unsigned long DataTx[2];

	DataTx[0] = WREG_OPCODE + (regnum & 0x0F);
	DataTx[1] = count - 1;

	set_adc_CS(0);

	if (RDATACmode == true)
	{
		adcXferByte(SDATAC_OPCODE);							// Enter SDATAC mode prior to reading or writing registers to prevent data corruption
		__delay_cycles(TD_SCSC_DELAY_24CLK);
	}

	adcXferByte(DataTx[0]);
	__delay_cycles(TD_SCSC_DELAY_4CLK);					// TODO: Check if necessary
	adcXferByte(DataTx[1]);

	for(i = 0; i < count; i++)
	{
		adcXferByte((unsigned long)data[i]);
		if(regnum + i < NUM_REGISTERS)
			RegData_LW[regnum + i] = data[i];
	}

	set_adc_CS(1);

	return;
}

/*
 * Sends a command to the ADS1257
 *
 * \param SPIBASE Address of the Tiva SSI module which will be used to
 * communicate with the device
 * \param op_code is the command being issued
 */
void sendCommand(uint8_t op_code)
{
	switch(op_code)						// Wait for DRDY before sending opcode?
	{
		case RDATAC_OPCODE:
			RDATACmode = true;
		case SDATAC_OPCODE:
			RDATACmode = false;
		case RESET_OPCODE:
		case STANDBY_OPCODE:
		case SELFOCAL_OPCODE:
		case SYSOCAL_OPCODE:
		case SELFGCAL_OPCODE:
		case SYSGCAL_OPCODE:
		case SELFCAL_OPCODE:
			WaitForDRDY();
			break;

		default:
			break;
	}

	set_adc_CS(0);
	adcXferByte(op_code);				// Send opcode
	set_adc_CS(1);


	switch(op_code)						// Insert opcode dependent delay after sending opcode?
	{
		case RREG_OPCODE:
		case WREG_OPCODE:
		case RDATA_OPCODE:
			__delay_cycles(TD_SCSC_DELAY_4CLK);
			break;

		case RDATAC_OPCODE:
		case SDATAC_OPCODE:
		case SYNC_OPCODE:
			__delay_cycles(TD_SCSC_DELAY_24CLK);
			break;

		case RESET_OPCODE:
		case STANDBY_OPCODE:
		case SELFOCAL_OPCODE:
		case SYSOCAL_OPCODE:
		case SELFGCAL_OPCODE:
		case SYSGCAL_OPCODE:
		case SELFCAL_OPCODE:
			WaitForDRDY();
			break;

		default:
			break;
	}


	return;
}


// Sends RDATA command to read the data
// Wait for DRDY low before calling this function
int32_t dataRead_byCommand(void)
{
	uint8_t dataRx[DATA_LENGTH];

	set_adc_CS(0);
	adcXferByte(RDATA_OPCODE);
	__delay_cycles(TD_DIDO_DELAY_T6);
	dataRx[0] = adcXferByte(0x00);
	dataRx[1] = adcXferByte(0x00);
	dataRx[2] = adcXferByte(0x00);
	set_adc_CS(1);

	return (int32_t)( ( (dataRx[0] & 0x80) ? (0xFF000000) : (0x00000000) ) |			// Sign extend and return result
									  ((int32_t) (dataRx[0] & 0xFF) << 16) |
									  ((int32_t) (dataRx[1] & 0xFF) << 8 ) |
									  ((int32_t) (dataRx[2] & 0xFF) << 0 ) );
}


// Reads data directly (must be in RDATAC mode)
extern int32_t dataRead_direct(void)
{
	uint8_t dataRx[DATA_LENGTH];

	set_adc_CS(0);

	dataRx[0] = adcXferByte(0x00);
	dataRx[1] = adcXferByte(0x00);
	dataRx[2] = adcXferByte(0x00);

	set_adc_CS(1);

	return (int32_t) (( (dataRx[0] & 0x80) ? (0xFF000000) : (0x00000000) ) |
									  ((int32_t) (dataRx[0] & 0xFF) << 16) |
									  ((int32_t) (dataRx[1] & 0xFF) << 8 ) |
									  ((int32_t) (dataRx[2] & 0xFF) << 0 ) );
}


