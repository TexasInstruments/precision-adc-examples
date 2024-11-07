/**
 * \file ads125H02.h
 *
 * \brief This header file contains all register map definitions for the ADS125H01 and ADS125H02.
 * 
 * \note Macro naming conventions try to follow ADS125H01 and ADS125H02 datasheet naming definitions;
 *  however, future datasheet revisions may cause macro names to differ from the datasheet.
 *
 * \date 1/11/2019
 *
 * \copyright Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef ADS125H02_H_
#define ADS125H02_H_

#include <assert.h>
#include <stdint.h>
#include <stdbool.h>

#include "hal.h"


/** Remove this macro definition to use this header file with the ADS125H01 */
#define ADS125H02_ONLY_FEATURES

/* Disable assertions when not in the CCS "Debug" configuration */
#ifndef _DEBUG
	#define NDEBUG
#endif


/*----------------------------------------------------------------------------*/
/* Constants */
/*----------------------------------------------------------------------------*/

/** Total number of device registers */
#define NUM_REGISTERS 		((uint8_t) 19)

/** Alias used for setting GPIOs pins to the logic "high" state */
#define HIGH                (true)

/** Alias used for setting GPIOs pins to the logic "low" state */
#define LOW                 (false)


/*----------------------------------------------------------------------------*/
/* Global variables */
/*----------------------------------------------------------------------------*/

/** Array used to recall device register map configurations */
extern uint8_t              ADC_RegisterMap[NUM_REGISTERS];

/** Value used as the "don't care" byte when sending SPI commands */
extern uint8_t 				ADC_DontCare;


/*----------------------------------------------------------------------------*/
/* Helpful Macros */
/*----------------------------------------------------------------------------*/

/** Returns true if STATUS byte enable bit is set */
#define STATUS_BYTE_ENABLED ((bool) (ADC_RegisterMap[REG_ADDR_MODE3] & MODE3_STATEN_MASK))

/** Returns true if pulse convert mode bit is set */
#define PULSE_CONVERT_MODE	((bool) (ADC_RegisterMap[REG_ADDR_MODE1] & MODE1_CONVRT_PULSE))

/** Gain register field setting */
#define GAIN_INDEX			((uint8_t) ((ADC_RegisterMap[REG_ADDR_MODE4] & MODE4_GAIN_MASK) >> 0))

/** Data rate register field setting */
#define DATARATE_INDEX		((uint8_t) ((ADC_RegisterMap[REG_ADDR_MODE0] & MODE0_DR_MASK) >> 3))

/** Device ID bits */
#define DEVICE_ID			((uint8_t) ((ADC_RegisterMap[REG_ADDR_ID] & ID_DEV_MASK) >> 4))

/** Revision ID1 bits */
#define REVISION_ID1        ((uint8_t) ((ADC_RegisterMap[REG_ADDR_ID] & ID1_REV_MASK) >> 0))

/** Revision ID2 bits */
#define REVISION_ID2        ((uint8_t) ((ADC_RegisterMap[REG_ADDR_STATUS2] & ID2_REV_MASK) >> 0))

/** Returns true if the STATUS0 register LOCK1 bit is set */
#define REGISTER_LOCK1      ((bool) (ADC_RegisterMap[REG_ADDR_STATUS0] & STATUS0_LOCK1_MASK))

/** Returns true if the STATUS2 register LOCK2 bit is set */
#define REGISTER_LOCK2      ((bool) (ADC_RegisterMap[REG_ADDR_STATUS2] & STATUS2_LOCK2_MASK))

/** Returns true if the LOCK1 or LOCK2 bit is set */
#define REGISTERS_LOCKED    ((bool) (REGISTER_LOCK1 | REGISTER_LOCK2))


/*----------------------------------------------------------------------------*/
/* SPI command opcodes */
/*----------------------------------------------------------------------------*/

#define OPCODE_NOP								((uint8_t) 0x00)
#define OPCODE_RESET							((uint8_t) 0x06)
#define OPCODE_START							((uint8_t) 0x08)
#define OPCODE_STOP								((uint8_t) 0x0A)
#define OPCODE_RDATA							((uint8_t) 0x12)
#define OPCODE_OFSCAL							((uint8_t) 0x16)
#define OPCODE_GANCAL							((uint8_t) 0x17)
#define OPCODE_RREG								((uint8_t) 0x20)
#define OPCODE_WREG								((uint8_t) 0x40)
#define OPCODE_LOCK								((uint8_t) 0xF2)
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
void    startConversions(void);
int32_t readData(uint8_t status[], uint8_t data[], uint8_t crc[]);
uint8_t calculateCRC(const uint8_t dataBytes[], uint8_t numBytes);
bool    validateSPI(const uint8_t DataTx[], const uint8_t DataRx[], uint8_t opcode);
void    restoreRegisterDefaults(void);


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
 * |        		DEV_ID[3:0]      	   |  	     	 REV_ID1[3:0]**		       |
 * ---------------------------------------------------------------------------------
 * ** Revision ID may change on future devices, without notification!
 */

	/** ID register address */
	#define REG_ADDR_ID							((uint8_t) 0x00)

    /** ID register field masks */
    #define ID_DEV_ID_MASK                      ((uint8_t) 0xF0)
    #define ID_REV_ID1_MASK                     ((uint8_t) 0x0F)

    /** DEV_ID field values */
    #define ID_DEV_ADS125H01                    ((uint8_t) 0x40)
    #define ID_DEV_ADS125H02                    ((uint8_t) 0x60)

	/** REV_ID1 field values */
	#define ID1_REVA							((uint8_t) 0x00)
    #define ID1_REVB                            ((uint8_t) 0x01)
    #define ID1_REVC                            ((uint8_t) 0x02)
    #define ID1_REVD                            ((uint8_t) 0x03)


/* Register 0x01 (STATUS0) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |  LOCK1  | CRC1ERR |    0    | STAT12  | REFALM  |  DRDY   |  CLOCK  |  RESET  |
 * ---------------------------------------------------------------------------------
 */

	/** STATUS0 register address */
	#define REG_ADDR_STATUS0					((uint8_t) 0x01)

	/** STATUS0 default (reset) value */
	#define STATUS0_DEFAULT						((uint8_t) 0x01)

	/** STATUS0 bit masks */
	#define STATUS0_LOCK1_MASK					((uint8_t) 0x80)
	#define STATUS0_CRC1ERR_MASK				((uint8_t) 0x40)
	#define STATUS0_STAT12_MASK					((uint8_t) 0x10)
	#define STATUS0_REFALM_MASK				    ((uint8_t) 0x08)
	#define STATUS0_DRDY_MASK					((uint8_t) 0x04)
	#define STATUS0_CLOCK_MASK					((uint8_t) 0x02)
	#define STATUS0_RESET_MASK					((uint8_t) 0x01)

	/** Write value to clear the STATUS0 register */
	#define STATUS0_CLEAR						((uint8_t) 0x00)


/* Register 0x02 (MODE0) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |         			  DR[4:0]   			 	 |         FILTER[2:0]		   |
 * ---------------------------------------------------------------------------------
 */

	/** MODE0 register address */
	#define REG_ADDR_MODE0						((uint8_t) 0x02)

	/** MODE0 default (reset) value */
	#define MODE0_DEFAULT						((uint8_t) 0x24)

    /** MODE0 register field masks */
    #define MODE0_DR_MASK                       ((uint8_t) 0xF8)
    #define MODE0_FILTER_MASK                   ((uint8_t) 0x07)

	/** DR field values */
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
	#define MODE0_DR_19200_SPS					((uint8_t) 0x70)
	#define MODE0_DR_25600_SPS					((uint8_t) 0x78)
	#define MODE0_DR_40000_SPS					((uint8_t) 0x80)

	/** FILTER field values */
	#define MODE0_SINC1							((uint8_t) 0x00)
	#define MODE0_SINC2							((uint8_t) 0x01)
	#define MODE0_SINC3							((uint8_t) 0x02)
	#define MODE0_SINC4							((uint8_t) 0x03)
	#define MODE0_FIR							((uint8_t) 0x04)


/* Register 0x03 (MODE1) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    0    |    CHOP[1:0]**	 | CONVRT  |               DELAY[3:0]     	  	   |
 * ---------------------------------------------------------------------------------
 * ** ADS125H01: ACX modes are not available!
 */

	/** MODE1 register address */
	#define REG_ADDR_MODE1						((uint8_t) 0x03)

	/** MODE1 default (reset) value */
	#define MODE1_DEFAULT						((uint8_t) 0x01)

    /** MODE1 register field masks */
    #define MODE1_CHOP_MASK                     ((uint8_t) 0x60)
    #define MODE1_CONVRT_MASK                   ((uint8_t) 0x10)
    #define MODE1_DELAY_MASK                    ((uint8_t) 0x0F)

    /** CHOP field values */
	#define MODE1_CHOP_OFF    					((uint8_t) 0x00)
	#define MODE1_CHOP_ON						((uint8_t) 0x20)
#ifdef ADS125H02_ONLY_FEATURES
	#define MODE1_CHOP_2WIRE_ACX				((uint8_t) 0x40)
	#define MODE1_CHOP_4WIRE_ACX				((uint8_t) 0x60)
#endif

    /** CONVRT field values */
	#define MODE1_CONVRT_CONTINUOUS				((uint8_t) 0x00)
	#define MODE1_CONVRT_PULSE 					((uint8_t) 0x10)

    /** DELAY field values */
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
 * |   			GPIO_CON[3:0]**			   |			  GPIO_DIR[3:0]**		   |
 * ---------------------------------------------------------------------------------
 */

	/** MODE2 register address */
	#define REG_ADDR_MODE2						((uint8_t) 0x04)

	/** MODE2 default (reset) value */
	#define MODE2_DEFAULT						((uint8_t) 0x00)

#ifdef ADS125H02_ONLY_FEATURES
	/** GPIO_CON field values */
	#define MODE2_GPIOCON_GPIO3_ENABLE_MASK		((uint8_t) 0x80)
	#define MODE2_GPIOCON_GPIO2_ENABLE_MASK		((uint8_t) 0x40)
	#define MODE2_GPIOCON_GPIO1_ENABLE_MASK		((uint8_t) 0x20)
	#define MODE2_GPIOCON_GPIO0_ENABLE_MASK		((uint8_t) 0x10)

    /** GPIO_DIR field values */
	#define MODE2_GPIODIR_GPIO3_INPUT_MASK		((uint8_t) 0x08)
	#define MODE2_GPIODIR_GPIO2_INPUT_MASK		((uint8_t) 0x04)
	#define MODE2_GPIODIR_GPIO1_INPUT_MASK		((uint8_t) 0x02)
	#define MODE2_GPIODIR_GPIO0_INPUT_MASK		((uint8_t) 0x01)
#endif


/* Register 0x05 (MODE3) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    0 	 | STATENB |         0         |		     GPIO_DAT[3:0]**		   |
 * ---------------------------------------------------------------------------------
 * ** ADS125H01: GPIO_DAT field is reserved
 */

	/** MODE3 register address */
	#define REG_ADDR_MODE3						((uint8_t) 0x05)

	/** MODE3 default (reset) value */
	#define MODE3_DEFAULT						((uint8_t) 0x00)
	
	/** MODE3 field masks */
	#define MODE3_STATEN_MASK					((uint8_t) 0x40)
#ifdef ADS125H02_ONLY_FEATURES
	#define MODE3_GPIODAT_GPIO3_DATA_MASK		((uint8_t) 0x08)
	#define MODE3_GPIODAT_GPIO2_DATA_MASK		((uint8_t) 0x04)
	#define MODE3_GPIODAT_GPIO1_DATA_MASK		((uint8_t) 0x02)
	#define MODE3_GPIODAT_GPIO0_DATA_MASK		((uint8_t) 0x01)
#endif


/* Register 0x06 (REF) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |              0              |  REFENB | 	RMUXP[1:0]**   |	 RMUXN[1:0]**  |
 * ---------------------------------------------------------------------------------
 * ** ADS125H01: REFP1/REFN1 pins are not available for external reference input!
 */

	/** REF register address */
	#define REG_ADDR_REF						((uint8_t) 0x06)

	/** REF default (reset) value */
	#define REF_DEFAULT							((uint8_t) 0x05)

	/** REF field masks */
	#define REF_REFENB_MASK						((uint8_t) 0x10)
    #define REF_RMUXP_MASK                      ((uint8_t) 0x0C)
    #define REF_RMUXN_MASK                      ((uint8_t) 0x03)

	/** RMUXP field values */
	#define REF_RMUXP_INT_P						((uint8_t) 0x00)
	#define REF_RMUXP_AVDD						((uint8_t) 0x04)
	#define REF_RMUXP_REFP0						((uint8_t) 0x08)
#ifdef ADS125H02_ONLY_FEATURES
	#define REF_RMUXP_REFP1					    ((uint8_t) 0x0C)
#endif

    /** RMUXP field values */
	#define REF_RMUXN_INT_N						((uint8_t) 0x00)
	#define REF_RMUXN_AGND						((uint8_t) 0x01)
	#define REF_RMUXN_REFN0						((uint8_t) 0x02)
#ifdef ADS125H02_ONLY_FEATURES
    #define REF_RMUXN_REFN1                     ((uint8_t) 0x03)
#endif


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


/* Register 0x0D (IMUX) definition**
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |              IMUX2[3:0]**    		   | 			  IMUX1[3:0]**			   |
 * ---------------------------------------------------------------------------------
 * ** Register settings only affect ADS125H02, write default value with ADS125H01
 */

	/** IMUX register address */
	#define REG_ADDR_IMUX						((uint8_t) 0x0D)

	/** IMUX default (reset) value */
	#define IMUX_DEFAULT						((uint8_t) 0xFF)

#ifdef ADS125H02_ONLY_FEATURES
    /** IMUX field masks */
    #define IMUX_IMUX2_MASK                     ((uint8_t) 0xF0)
    #define IMUX_IMUX1_MASK                     ((uint8_t) 0x0F)

	/** IMUX2 field values */
	#define IMUX_IMUX2_NOCONNECT			    ((uint8_t) 0xF0)
	#define IMUX_IMUX2_IDAC1					((uint8_t) 0x80)
    #define IMUX_IMUX2_IDAC2                    ((uint8_t) 0x90)

	/** IMUX1 field values */
	#define IMUX_IMUX1_NOCONNECT			    ((uint8_t) 0x0F)
	#define IMUX_IMUX1_IDAC1					((uint8_t) 0x08)
	#define IMUX_IMUX1_IDAC2				    ((uint8_t) 0x09)
#endif


/* Register 0x0E (IMAG) definition**
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |              IMAG2[3:0]**    		   | 			  IMAG1[3:0]**			   |
 * ---------------------------------------------------------------------------------
 * ** Register settings only affect ADS125H02, write default value with ADS125H01
 */

	/** IMAG register address */
	#define REG_ADDR_IMAG						((uint8_t) 0x0E)

	/** IMAG default (reset) value */
	#define IMAG_DEFAULT						((uint8_t) 0x00)

#ifdef ADS125H02_ONLY_FEATURES
    /** IMAG field masks */
    #define IMAG_IMAG2_MASK                     ((uint8_t) 0xF0)
    #define IMAG_IMAG1_MASK                     ((uint8_t) 0x0F)

	/** IMAG2 field values */
	#define IMAG_IMAG2_OFF						((uint8_t) 0x00)
	#define IMAG_IMAG2_50_uA					((uint8_t) 0x10)
	#define IMAG_IMAG2_100_uA					((uint8_t) 0x20)
	#define IMAG_IMAG2_250_uA					((uint8_t) 0x30)
	#define IMAG_IMAG2_500_uA					((uint8_t) 0x40)
	#define IMAG_IMAG2_750_uA					((uint8_t) 0x50)
	#define IMAG_IMAG2_1000_uA					((uint8_t) 0x60)
	#define IMAG_IMAG2_1500_uA					((uint8_t) 0x70)
	#define IMAG_IMAG2_2000_uA					((uint8_t) 0x80)
	#define IMAG_IMAG2_2500_uA					((uint8_t) 0x90)
	#define IMAG_IMAG2_3000_uA					((uint8_t) 0xA0)

	/** IMAG1 field values */
	#define IMAG_IMAG1_OFF						((uint8_t) 0x00)
	#define IMAG_IMAG1_50_uA					((uint8_t) 0x01)
	#define IMAG_IMAG1_100_uA					((uint8_t) 0x02)
	#define IMAG_IMAG1_250_uA					((uint8_t) 0x03)
	#define IMAG_IMAG1_500_uA					((uint8_t) 0x04)
	#define IMAG_IMAG1_750_uA					((uint8_t) 0x05)
	#define IMAG_IMAG1_1000_uA					((uint8_t) 0x06)
	#define IMAG_IMAG1_1500_uA					((uint8_t) 0x07)
	#define IMAG_IMAG1_2000_uA					((uint8_t) 0x08)
	#define IMAG_IMAG1_2500_uA					((uint8_t) 0x09)
	#define IMAG_IMAG1_3000_uA					((uint8_t) 0x0A)
#endif


/* Register 0x0F (RESERVED) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                                       0                                       |
 * ---------------------------------------------------------------------------------
 */

	/** RESERVED register address */
	#define REG_ADDR_RESERVED					((uint8_t) 0x0F)

	/** RESERVED default (reset) value */
	#define RESERVED_DEFAULT					((uint8_t) 0x00)


/* Register 0x10 (MODE4) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    0    | 		     MUX[2:0]		   |    			GAIN[3:0]		       |
 * ---------------------------------------------------------------------------------
 */

	/** MODE4 register address */
	#define REG_ADDR_MODE4						((uint8_t) 0x10)

	/** MODE4 default (reset) value */
	#define	MODE4_DEFAULT						((uint8_t) 0x50)

	/** MODE4 field masks */
	#define MODE4_MUX_MASK						((uint8_t) 0x70)
    #define MODE4_GAIN_MASK                    ((uint8_t) 0x0F)

	/** MUX field values */
	#define MODE4_MUX_AIN1_AIN0				    ((uint8_t) 0x00)
	#define MODE4_MUX_AIN0_AIN1				    ((uint8_t) 0x10)
	#define MODE4_MUX_AIN1_AINCOM			    ((uint8_t) 0x20)
	#define MODE4_MUX_AIN0_AINCOM			    ((uint8_t) 0x30)
	#define MODE4_MUX_HV_SUPPLY					((uint8_t) 0x40)
	#define MODE4_MUX_INT_SHORT				    ((uint8_t) 0x50)
	#define MODE4_MUX_TEMP_SENSE			    ((uint8_t) 0x60)

	/** GAIN field values */
	#define MODE4_GAIN_0P125			        ((uint8_t) 0x00)
	#define MODE4_GAIN_0P1875					((uint8_t) 0x01)
	#define MODE4_GAIN_0P25						((uint8_t) 0x02)
	#define MODE4_GAIN_0P5						((uint8_t) 0x03)
	#define MODE4_GAIN_1						((uint8_t) 0x04)
	#define MODE4_GAIN_2						((uint8_t) 0x05)
	#define MODE4_GAIN_4						((uint8_t) 0x06)
	#define MODE4_GAIN_8						((uint8_t) 0x07)
	#define MODE4_GAIN_16						((uint8_t) 0x08)
	#define MODE4_GAIN_32						((uint8_t) 0x09)
	#define MODE4_GAIN_64						((uint8_t) 0x0A)
	#define MODE4_GAIN_128						((uint8_t) 0x0B)


/* Register 0x11 (STATUS1) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * | PGA_ONL | PGA_ONH | PGA_OPL | PGA_OPH | PGA_INL | PGA_INH | PGA_IPL | PGA_IPH |
 * ---------------------------------------------------------------------------------
 */

	/** STATUS1 register address */
	#define REG_ADDR_STATUS1					((uint8_t) 0x11)

	/** STATUS1 default (reset) value */
	#define STATUS1_DEFAULT						((uint8_t) 0x00)

	/** STATUS1 bit field masks */
	#define STATUS1_PGA_ONL						((uint8_t) 0x80)
	#define STATUS1_PGA_ONH						((uint8_t) 0x40)
	#define STATUS1_PGA_OPL						((uint8_t) 0x20)
	#define STATUS1_PGA_OPH						((uint8_t) 0x10)
	#define STATUS1_PGA_INL						((uint8_t) 0x08)
	#define STATUS1_PGA_INH						((uint8_t) 0x04)
	#define STATUS1_PGA_IPL						((uint8_t) 0x02)
	#define STATUS1_PGA_IPH						((uint8_t) 0x01)

	/** Write this value to clear the STATUS1 register */
	#define STATUS1_CLEAR						((uint8_t) 0x00)


/* Register 0x12 (STATUS2) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |		 0		   |  LOCK2  | CRC2ERR |			 REV_ID2[3:0]**			   |
 * ---------------------------------------------------------------------------------
 * ** Revision ID may change on future devices, without notification!
 */

    /** STATUS2 register address */
	#define REG_ADDR_STATUS2					((uint8_t) 0x12)

    /** STATUS2 default (reset) value */
    #define STATUS2_DEFAULT                     ((uint8_t) 0x01)

	/** STATUS2 field masks */
	#define STATUS2_LOCK2_MASK					((uint8_t) 0x20)
	#define STATUS2_CRC2ERR_MASK				((uint8_t) 0x10)
    #define STATUS2_REV_ID2_MASK                ((uint8_t) 0x0F)

	/** REV_ID2 field values */
	#define ID2_REVA							((uint8_t) 0x00)
    #define ID2_REVB                            ((uint8_t) 0x01)
	#define ID2_REVC                            ((uint8_t) 0x02)
	#define ID2_REVD                            ((uint8_t) 0x03)

    /** Write this value to clear the STATUS2 register */
    #define STATUS2_CLEAR                       ((uint8_t) 0x00)


/*----------------------------------------------------------------------------*/
/* END OF REGISTER DEFINITIONS */
/*----------------------------------------------------------------------------*/

#endif /* ADS125H02_H_ */
