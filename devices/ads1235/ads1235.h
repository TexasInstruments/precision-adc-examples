/**
 * @file ads1235.h
 *
 * @brief This header file contains all register map definitions for the ADS1235.
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

#ifndef ADS1235_H_
#define ADS1235_H_

#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include "hal.h"

/*----------------------------------------------------------------------------*/
/* Constants */
/*----------------------------------------------------------------------------*/

/** Total number of device registers */
#define NUM_REGISTERS 		((uint8_t) 18)

/** Alias used for setting GPIOs pins to the logic "high" state */
#define HIGH                (true)

/** Alias used for setting GPIOs pins to the logic "low" state */
#define LOW                 (false)

/*----------------------------------------------------------------------------*/
/* Global variables */
/*----------------------------------------------------------------------------*/

/** Array used to recall device register map configurations */
extern uint8_t 				ADC_RegisterMap[NUM_REGISTERS];

/** Value used as the "don't care" byte when sending SPI commands */
extern uint8_t 				ADC_DontCare;


/*----------------------------------------------------------------------------*/
/* Helpful Macros */
/*----------------------------------------------------------------------------*/

/** Returns true if STATUS byte enable bit is set */
#define STATUS_BYTE_ENABLED ((bool) (ADC_RegisterMap[REG_ADDR_MODE3] & MODE3_STATEN_MASK))

/** Returns true if CRC byte enable bit is set */
#define CRC_BYTE_ENABLED	((bool) (ADC_RegisterMap[REG_ADDR_MODE3] & MODE3_CRCEN_MASK))

/** Returns true if PGA bypass bit is set */
#define PGA_BYPASSED		((bool) (ADC_RegisterMap[REG_ADDR_PGA] & PGA_BYPASS_MASK))

/** Returns true if pulse convert mode bit is set */
#define PULSE_CONVERT_MODE	((bool) (ADC_RegisterMap[REG_ADDR_MODE1] & MODE1_CONVRT_PULSE))

/** Returns true if software power-down pin is set */
#define SW_PWDN_MODE		((bool) (ADC_RegisterMap[REG_ADDR_MODE3] & MODE3_PWDN_MASK))

/** Gain register field setting */
#define GAIN_INDEX			((uint8_t) ((ADC_RegisterMap[REG_ADDR_PGA] & PGA_GAIN_MASK) >> 0))

/** Data rate register field setting */
#define DATARATE_INDEX		((uint8_t) ((ADC_RegisterMap[REG_ADDR_MODE0] & MODE0_DR_MASK) >> 3))

/** Device ID bits */
#define DEVICE_ID			((uint8_t) ((ADC_RegisterMap[REG_ADDR_ID] & ID_DEV_MASK) >> 4))

/** Revision ID bits */
#define REVISION_ID			((uint8_t) ((ADC_RegisterMap[REG_ADDR_ID] & ID_REV_MASK) >> 0))

/** Returns true if the STATUS register LOCK bit is set */
#define REGISTER_LOCK		((bool) (ADC_RegisterMap[REG_ADDR_STATUS] & STATUS_LOCK_MASK))

/** Returns true if the STATUS register CRCERR bit is set */
#define CRC_ERROR			((bool) (ADC_RegisterMap[REG_ADDR_STATUS] & STATUS_CRC_ERR_MASK))

/** Returns true if the STATUS register PGAL_ALM bit is set */
#define PGAL_ALARM			((bool) (ADC_RegisterMap[REG_ADDR_STATUS] & STATUS_PGAL_ALM_MASK))

/** Returns true if the STATUS register PGAH_ALM bit is set */
#define PGAH_ALARM			((bool) (ADC_RegisterMap[REG_ADDR_STATUS] & STATUS_PGAH_ALM_MASK))

/** Returns true if the STATUS register REFL_ALM bit is set */
#define REFL_ALARM			((bool) (ADC_RegisterMap[REG_ADDR_STATUS] & STATUS_REFL_ALM_MASK))

/** Returns true if the STATUS register DRDY bit is set */
#define DRDY_STATUS			((bool) (ADC_RegisterMap[REG_ADDR_STATUS] & STATUS_DRDY_MASK))

/** Returns true if the STATUS register CLOCK bit is set */
#define EXT_CLOCK			((bool) (ADC_RegisterMap[REG_ADDR_STATUS] & STATUS_CLOCK_MASK))

/** Returns true if the STATUS register RESET bit is set */
#define RESET_ALARM			((bool) (ADC_RegisterMap[REG_ADDR_STATUS] & STATUS_RESET_MASK))


/*----------------------------------------------------------------------------*/
/* SPI command opcodes */
/*----------------------------------------------------------------------------*/

/** "NOP" SPI command */
#define OPCODE_NOP								((uint8_t) 0x00)

/** "RESET" SPI command */
#define OPCODE_RESET							((uint8_t) 0x06)

/** "START" SPI command */
#define OPCODE_START							((uint8_t) 0x08)

/** "STOP" SPI command */
#define OPCODE_STOP								((uint8_t) 0x0A)

/** "RDATA" SPI command */
#define OPCODE_RDATA							((uint8_t) 0x12)

/** "SYOCAL" SPI command */
#define OPCODE_SYOCAL							((uint8_t) 0x16)

/** "SYGCAL" SPI command */
#define OPCODE_SYGCAL							((uint8_t) 0x17)

/** "SFOCAL" SPI command */
#define OPCODE_SFOCAL							((uint8_t) 0x19)

/** "RREG" SPI command */
#define OPCODE_RREG								((uint8_t) 0x20)

/** "WREG" SPI command */
#define OPCODE_WREG								((uint8_t) 0x40)

/** "LOCK" SPI command */
#define OPCODE_LOCK								((uint8_t) 0xF2)

/** "UNLOCK" SPI command */
#define OPCODE_UNLOCK							((uint8_t) 0xF5)


/*----------------------------------------------------------------------------*/
/* Function prototypes */
/*----------------------------------------------------------------------------*/
void    adcStartupRoutine(void);
uint8_t readSingleRegister(uint8_t addr);
void    readMultipleRegisters(uint8_t addr, uint8_t count, uint8_t data[]);
void    writeSingleRegister(uint8_t addr, uint8_t data);
void    writeMultipleRegisters(uint8_t addr, uint8_t count, const uint8_t data[]);
bool    sendCommand(uint8_t op_code);
bool    lockRegisters(void);
bool    unlockRegisters(void);
void    ads1235_startConversions(void);
int32_t readData(uint8_t status[], uint8_t data[], uint8_t crc[]);
uint8_t calculateCRC(const uint8_t dataBytes[], uint8_t numBytes);
bool    validateSPI(const uint8_t DataTx[], const uint8_t DataRx[], uint8_t opcode);
void    restoreRegisterDefaults(void);

float ads1225_measure_internal_temperature_example(void);

/*----------------------------------------------------------------------------*/
/* Register definitions */
/*----------------------------------------------------------------------------*/

/* NOTE: Whenever possible, macro names (defined below) were derived from
 * datasheet defined names; however, updates to documentation may cause
 * mismatches between names defined here in example code from those shown
 * in the device datasheet.
 */


/* Register 0x00 (ID) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |        		DEV_ID[3:0]      	   |  	     	  REV_ID[3:0]		       |
 * ---------------------------------------------------------------------------------
 */

	/** ID register address */
	#define REG_ADDR_ID							((uint8_t) 0x00)

	/* Define DEV_ID (device) */
	#define ID_DEV_ADS1235						((uint8_t) 0xC0)
	#define ID_DEV_MASK							((uint8_t) 0xF0)

	/* Define REV_ID (revision) */
    /* Note: Revision ID can change without notification */
	#define ID_REV_A							((uint8_t) 0x00)
    #define ID_REV_B                            ((uint8_t) 0x01)
	#define ID_REV_MASK							((uint8_t) 0x0F)


/* Register 0x01 (STATUS) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |  LOCK   | CRC_ERR |PGAL_ALM |PGAH_ALM |REFL_ALM |  DRDY   |  CLOCK  |  RESET  |
 * ---------------------------------------------------------------------------------
 */

	/** STATUS register address */
	#define REG_ADDR_STATUS						((uint8_t) 0x01)

	/** STATUS default (reset) value */
	#define STATUS_DEFAULT						((uint8_t) 0x01)

	/* Define the STATUS byte bit masks */
	#define STATUS_LOCK_MASK					((uint8_t) 0x80)
	#define STATUS_CRC_ERR_MASK					((uint8_t) 0x40)
	#define STATUS_PGAL_ALM_MASK				((uint8_t) 0x20)
	#define STATUS_PGAH_ALM_MASK				((uint8_t) 0x10)
	#define STATUS_REFL_ALM_MASK				((uint8_t) 0x08)
	#define STATUS_DRDY_MASK					((uint8_t) 0x04)
	#define STATUS_CLOCK_MASK					((uint8_t) 0x02)
	#define STATUS_RESET_MASK					((uint8_t) 0x01)

	/** Write this value to clear the STATUS register */
	#define STATUS_CLEAR						((uint8_t) 0x00)


/* Register 0x02 (MODE0) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    0    |			 DR[3:0]   				 	 |       	FILTER[2:0]		   |
 * ---------------------------------------------------------------------------------
 */

	/** MODE0 register address */
	#define REG_ADDR_MODE0						((uint8_t) 0x02)

	/** MODE0 default (reset) value */
	#define MODE0_DEFAULT						((uint8_t) 0x24)

	/* Define the data rates */
	#define MODE0_DR_2_5_SPS					((uint8_t) 0x00)
	#define MODE0_DR_5_SPS						((uint8_t) 0x08)
	#define MODE0_DR_10_SPS						((uint8_t) 0x10)
	#define MODE0_DR_16_6_SPS					((uint8_t) 0x18)
	#define MODE0_DR_20_SPS						((uint8_t) 0x20)
	#define MODE0_DR_50_SPS						((uint8_t) 0x28)
	#define MODE0_DR_60_SPS						((uint8_t) 0x30)
	#define MODE0_DR_100_SPS					((uint8_t) 0x38)
	#define MODE0_DR_400_SPS					((uint8_t) 0x40)
	#define MODE0_DR_1200_SPS					((uint8_t) 0x48)
	#define MODE0_DR_2400_SPS					((uint8_t) 0x50)
	#define MODE0_DR_4800_SPS					((uint8_t) 0x58)
	#define MODE0_DR_7200_SPS					((uint8_t) 0x60)
	#define MODE0_DR_14400_SPS					((uint8_t) 0x68)
	#define MODE0_DR_MASK						((uint8_t) 0x78)

	/** Define the filter modes */
	#define MODE0_SINC1							((uint8_t) 0x00)
	#define MODE0_SINC2							((uint8_t) 0x01)
	#define MODE0_SINC3							((uint8_t) 0x02)
	#define MODE0_SINC4							((uint8_t) 0x03)
	#define MODE0_FIR							((uint8_t) 0x04)


/* Register 0x03 (MODE1) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    0    |     CHOP[1:0]	 | CONVRT  |               DELAY[3:0]     	  	   |
 * ---------------------------------------------------------------------------------
 */

	/** MODE1 register address */
	#define REG_ADDR_MODE1						((uint8_t) 0x03)

	/** MODE1 default (reset) value */
	#define MODE1_DEFAULT						((uint8_t) 0x01)

	/* Define chop and AC-excitation modes */
	#define MODE1_CHOP_OFF    					((uint8_t) 0x00)
	#define MODE1_CHOP_ON						((uint8_t) 0x20)
	#define MODE1_CHOP_2WIRE_ACX				((uint8_t) 0x40)
	#define MODE1_CHOP_4WIRE_ACX				((uint8_t) 0x60)

	/* Define ADC conversion modes */
	#define MODE1_CONVRT_CONTINUOUS				((uint8_t) 0x00)
	#define MODE1_CONVRT_PULSE 					((uint8_t) 0x10)

	/* Define conversion start delays */
	#define MODE1_DELAY_0_uS					((uint8_t) 0x00)
	#define MODE1_DELAY_50_uS					((uint8_t) 0x01)
	#define MODE1_DELAY_59_uS					((uint8_t) 0x02)
	#define MODE1_DELAY_67_uS					((uint8_t) 0x03)
	#define MODE1_DELAY_85_uS					((uint8_t) 0x04)
	#define MODE1_DELAY_119_uS					((uint8_t) 0x05)
	#define MODE1_DELAY_189_uS					((uint8_t) 0x06)
	#define MODE1_DELAY_328_uS					((uint8_t) 0x07)
	#define MODE1_DELAY_605_uS					((uint8_t) 0x08)
	#define MODE1_DELAY_1160_uS					((uint8_t) 0x09)
	#define MODE1_DELAY_2270_uS					((uint8_t) 0x0A)
	#define MODE1_DELAY_4490_uS					((uint8_t) 0x0B)
	#define MODE1_DELAY_8930_uS					((uint8_t) 0x0C)
	#define MODE1_DELAY_17800_uS				((uint8_t) 0x0D)


/* Register 0x04 (MODE2) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |   			GPIO_CON[3:0]			   |			  GPIO_DIR[3:0]  		   |
 * ---------------------------------------------------------------------------------
 */

	/** MODE2 register address */
	#define REG_ADDR_MODE2						((uint8_t) 0x04)

	/** MODE2 default (reset) value */
	#define MODE2_DEFAULT						((uint8_t) 0x00)

	/* Define the GPIO pin connection masks (0-Analog Input; 1-GPIO) */
	#define MODE2_GPIOCON_AIN3_ENABLE_MASK		((uint8_t) 0x80)
	#define MODE2_GPIOCON_AIN2_ENABLE_MASK		((uint8_t) 0x40)
	#define MODE2_GPIOCON_AIN1_ENABLE_MASK		((uint8_t) 0x20)
	#define MODE2_GPIOCON_AIN0_ENABLE_MASK		((uint8_t) 0x10)

	/* Define the GPIO pin direction masks (0-Output; 1-Input) */
	#define MODE2_GPIODIR_AIN3_INPUT_MASK		((uint8_t) 0x08)
	#define MODE2_GPIODIR_AIN2_INPUT_MASK		((uint8_t) 0x04)
	#define MODE2_GPIODIR_AIN1_INPUT_MASK		((uint8_t) 0x02)
	#define MODE2_GPIODIR_AIN0_INPUT_MASK		((uint8_t) 0x01)


/* Register 0x05 (MODE3) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |  PWDN	 | STATENB | CRCENB  | SPITIM  |		     GPIO_DAT[3:0]  		   |
 * ---------------------------------------------------------------------------------
 */

	/** MODE3 register address */
	#define REG_ADDR_MODE3						((uint8_t) 0x05)

	/** MODE3 default (reset) value */
	#define MODE3_DEFAULT						((uint8_t) 0x00)

	/* Define the MODE3 upper nibble bit masks */
	#define MODE3_PWDN_MASK						((uint8_t) 0x80)
	#define MODE3_STATEN_MASK					((uint8_t) 0x40)
	#define MODE3_CRCEN_MASK					((uint8_t) 0x20)
	#define MODE3_SPITIM_MASK					((uint8_t) 0x10)

	/* Define the GPIO data setting masks (0-Low; 1-High)*/
	#define MODE3_GPIODAT_AIN3_DATA_MASK		((uint8_t) 0x08)
	#define MODE3_GPIODAT_AIN2_DATA_MASK		((uint8_t) 0x04)
	#define MODE3_GPIODAT_AIN1_DATA_MASK		((uint8_t) 0x02)
	#define MODE3_GPIODAT_AIN0_DATA_MASK		((uint8_t) 0x01)


/* Register 0x06 (REF) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    0    |    0    |    0    |    0    | 	RMUXP[1:0]**   |	 RMUXN[1:0]**  |
 * ---------------------------------------------------------------------------------
 */

	/** REF register address */
	#define REG_ADDR_REF						((uint8_t) 0x06)

	/** REF default (reset) value */
	#define REF_DEFAULT							((uint8_t) 0x05)

	/* Define the positive reference inputs */
	#define REF_RMUXP_AVDD						((uint8_t) 0x04)
	#define REF_RMUXP_REFP0						((uint8_t) 0x08)
	#define REF_RMUXP_AIN0					    ((uint8_t) 0x0C)

	/* Define the negative reference inputs */
	#define REF_RMUXN_AVSS						((uint8_t) 0x01)
	#define REF_RMUXN_REFN0						((uint8_t) 0x02)
    #define REF_RMUXN_AIN1                      ((uint8_t) 0x03)


/* Register 0x07 (OFCAL0) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    								OFC[7:0]								   |
 * ---------------------------------------------------------------------------------
 */

	/** OFCAL0 register address */
	#define REG_ADDR_OFCAL0						((uint8_t) 0x07)

	/** OFCAL0 default (reset) value */
	#define OFCAL0_DEFAULT						((uint8_t) 0x00)


/* Register 0x08 (OFCAL1) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    								OFC[15:8]								   |
 * ---------------------------------------------------------------------------------
 */
	/** OFCAL1 register address */
	#define REG_ADDR_OFCAL1						((uint8_t) 0x08)

	/** OFCAL1 default (reset) value */
	#define OFCAL1_DEFAULT						((uint8_t) 0x00)


/* Register 0x09 (OFCAL2) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    							   OFC[23:16]								   |
 * ---------------------------------------------------------------------------------
 */
	/** OFCAL2 register address */
	#define REG_ADDR_OFCAL2						((uint8_t) 0x09)

	/** OFCAL2 default (reset) value */
	#define OFCAL2_DEFAULT						((uint8_t) 0x00)


/* Register 0x0A (FSCAL0) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    								FSC[7:0]								   |
 * ---------------------------------------------------------------------------------
 */

	/** FSCAL0 register address */
	#define REG_ADDR_FSCAL0						((uint8_t) 0x0A)

	/** FSCAL0 default (reset) value */
	#define FSCAL0_DEFAULT						((uint8_t) 0x00)


/* Register 0x0B (FSCAL1) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    							   FSC[15:8]								   |
 * ---------------------------------------------------------------------------------
 */

	/** FSCAL1 register address */
	#define REG_ADDR_FSCAL1						((uint8_t) 0x0B)

	/** FSCAL1 default (reset) value */
	#define FSCAL1_DEFAULT						((uint8_t) 0x00)


/* Register 0x0C (FSCAL2) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    							   FSC[23:16]								   |
 * ---------------------------------------------------------------------------------
 */

	/** FSCAL2 register address */
	#define REG_ADDR_FSCAL2						((uint8_t) 0x0C)

	/** FSCAL2 default (reset) value */
	#define FSCAL2_DEFAULT						((uint8_t) 0x40)


/* Register 0x0D (RESERVED_0) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    1	 |	  1    |    1    |	  1	   | 	1	 |	  1    |    1    | 	  1    |
 * ---------------------------------------------------------------------------------
 */

	/** RESERVED 0 register address */
	#define REG_ADDR_RES_0						((uint8_t) 0x0D)

	/** RESERVED 0 default (reset) value */
	#define RES_0_DEFAULT						((uint8_t) 0xFF)


/* Register 0x0E (RESERVED_1) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    1	 |	  1    |    1    |	  1	   | 	1	 |	  1    |    1    | 	  1    |
 * ---------------------------------------------------------------------------------
 */

	/** RESERVED 1 register address */
	#define REG_ADDR_RES_1						((uint8_t) 0x0E)

	/** RESERVED 1 default (reset) value */
	#define RES_1_DEFAULT						((uint8_t) 0x00)


/* Register 0x0F (RESERVED) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    0    |    0    |    0    |    0    |    0    |    0    |    0    |    0    |
 * ---------------------------------------------------------------------------------
 */

	/** RESERVED register address */
	#define REG_ADDR_RESERVED					((uint8_t) 0x0F)

	/** RESERVED default (reset) value */
	#define RESERVED_DEFAULT					((uint8_t) 0x00)


/* Register 0x10 (PGA) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |  BYPASS |    0    |    0    |    0    |    0    |    		GAIN[2:0]	       |
 * ---------------------------------------------------------------------------------
 */

	/** PGA register address */
	#define REG_ADDR_PGA						((uint8_t) 0x10)

	/** PGA default (reset) value */
	#define	PGA_DEFAULT							((uint8_t) 0x00)

	/** Define the PGA Bypass mask (0=PGA normal mode; 1=PGA bypass mode) */
	#define PGA_BYPASS_MASK						((uint8_t) 0x80)

	/* Define the PGA gain definitions */
	#define PGA_GAIN_1							((uint8_t) 0x00)
	#define PGA_GAIN_64							((uint8_t) 0x06)
	#define PGA_GAIN_128						((uint8_t) 0x07)


/* Register 0x11 (INPMUX) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |   			   MUXP[3:0]**			   |     			MUXN[3:0]**		       |
 * ---------------------------------------------------------------------------------
 */

	/** INPMUX register address */
	#define REG_ADDR_INPMUX						((uint8_t) 0x11)

	/** INPMUX default (reset) value */
	#define	INPMUX_DEFAULT						((uint8_t) 0xFF)

	/* Define the positive input MUX connections */
	#define INPMUX_MUXP_AIN0					((uint8_t) 0x30)
	#define INPMUX_MUXP_AIN1					((uint8_t) 0x40)
	#define INPMUX_MUXP_AIN2					((uint8_t) 0x50)
	#define INPMUX_MUXP_AIN3					((uint8_t) 0x60)
	#define INPMUX_MUXP_AIN4					((uint8_t) 0x70)
	#define INPMUX_MUXP_AIN5					((uint8_t) 0x80)
	#define INPMUX_MUXP_TEMP_P					((uint8_t) 0xB0)
	#define INPMUX_MUXP_OPEN					((uint8_t) 0xE0)
	#define INPMUX_MUXP_VCOM					((uint8_t) 0xF0)

	/* Define the negative input MUX connections */
	#define INPMUX_MUXN_AIN0					((uint8_t) 0x03)
	#define INPMUX_MUXN_AIN1					((uint8_t) 0x04)
	#define INPMUX_MUXN_AIN2					((uint8_t) 0x05)
	#define INPMUX_MUXN_AIN3					((uint8_t) 0x06)
	#define INPMUX_MUXN_AIN4					((uint8_t) 0x07)
	#define INPMUX_MUXN_AIN5					((uint8_t) 0x08)
	#define INPMUX_MUXN_TEMP_N					((uint8_t) 0x0B)
	#define INPMUX_MUXN_OPEN					((uint8_t) 0x0E)
	#define INPMUX_MUXN_VCOM					((uint8_t) 0x0F)



/*----------------------------------------------------------------------------*/
/* END OF REGISTER DEFINITIONS */
/*----------------------------------------------------------------------------*/

#endif /* ADS1235_H_ */
