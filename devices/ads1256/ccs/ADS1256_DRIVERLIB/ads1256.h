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

#ifndef ADS1255_7_H_
#define ADS1255_7_H_

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// TODO:  Remove or rename User Library
#include "hal.h"

// Device command prototypes
extern uint8_t InitDevice(void);
extern void regRead(uint8_t regnum, uint8_t *pdata);
extern void readRegs(uint8_t regnum, uint8_t count);
extern void regWrite(unsigned int regnum, unsigned char *pdata);
extern void writeRegs(unsigned int regnum, unsigned int count, unsigned char *data);
extern void sendCommand(uint8_t op_code);
extern void set_adc_CS(uint8_t logicLevel);							// nCS pin control
extern void reset_adc_hw(void);
extern void reset_adc_sclk(void);
extern void reset_adc_sw(void);
extern int32_t dataRead_byCommand(void);
extern int32_t dataRead_direct(void);


#define ADS1256_NUM_REGISTERS 		11

// Global Variables
extern uint8_t RegData_LR[];	// Array that stores the last read register values
extern uint8_t RegData_LW[];	// Array that stores the last written register values
extern bool RDATACmode;			// Boolean that keeps track of the ADC read data mode

// Lengths of conversion data components
#define ADS1256_DATA_LENGTH			3					// 24-bit data (3 bytes)

// Flag to signal that we are in the process of collecting data
extern bool converting;
//#define DATA_MODE (registers[SYSCTRL_ADDR_MASK] & 0x03)
#define DATA_MODE_NORMAL	0x00
#define DATA_MODE_STATUS	0x01
#define DATA_MODE_CRC		0x02


// SPI TIMING REQUIREMENTS									//TODO: Update delay timing if SYSFREQ or MCLK frequency is modified
// MCLK			MIN: 0.1 MHz    TYP: 7.68 MHz	MAX: 10 MHz
// SCLK			MIN: 4/MCLK
//																  			CALLING FUNCTION					TIMING REQUIREMENT			EFFECTIVE DELAY			REFERENCE
#define TD_STARTUP_DELAY		750000						// Called as "__delay_cycles(TD_STARTUP_DELAY);"	... >= 30ms for XTAL, 8192 CLKS for EXTCLK 			(See pg. 27 in PDS)
#define TD_CSSC_DELAY			1							// Called as "__delay_cycles(TD_CSSC_DELAY);" 		... N/A
#define TD_SCCS_DELAY			26							// Called as "__delay_cycles(TD_SCCS_DELAY);" 		... (8/MCLK) ~= (26/SYSFREQ) ~= 1 us
#define TD_SCSC_DELAY_4CLK		14							// Called as "__delay_cycles(TD_SCSC_DELAY_4CLK);" 	... (4/MCLK) ~= (14/SYSFREQ) ~= 0.56 us
#define TD_SCSC_DELAY_24CLK		80							// Called as "__delay_cycles(TD_SCSC_DELAY_24CLK);" ... (24/MCLK) ~= (80/SYSFREQ) ~= 3.2 us				(See "t11" on pg. 8)
#define TW_PDL_DELAY			13							// Called as "__delay_cycles(TW_PDL_DELAY);" 		... (4/MCLK) ~= (13/SYSFREQ) ~= 0.5 us
#define TD_DIDO_DELAY_T6		170							// Called as "__delay_cycles(TD_DIDO_DELAY_T6);" ... (50/MCLK) ~= (170/SYSFREQ) ~= 6.8 us

// SCLK Reset Timing
#define T12_DELAY				1000						// Called as "__delay_cycles(T12_DELAY);"			... (307/MCLK) ~= (1000/SYSFREQ) ~= 40 us	 		(See pg. 8 in PDS)
#define T13_DELAY				17							// Called as "__delay_cycles(T13_DELAY);"			... (5/MCLK) ~= (17/SYSFREQ) ~= 1 us
#define T14_DELAY				1790						// Called as "__delay_cycles(T14_DELAY);"			... (550/MCLK) ~= (1790/SYSFREQ) ~= 72 us
#define T15_DELAY				3418						// Called as "__delay_cycles(T15_DELAY);"			... (1050/MCLK) ~= (3418/SYSFREQ) ~= 137 us


/*----------------------------------------------------------------------------*/
/* SPI command opcodes */
/*----------------------------------------------------------------------------*/
#define NOP_OPCODE				0x00
#define RDATA_OPCODE			0x01
#define RDATAC_OPCODE			0x03
#define SDATAC_OPCODE			0x0F
#define RREG_OPCODE				0x10
#define WREG_OPCODE				0x50
#define SELFCAL_OPCODE			0xF0
#define SELFOCAL_OPCODE			0xF1
#define SELFGCAL_OPCODE			0xF2
#define SYSOCAL_OPCODE			0xF3
#define SYSGCAL_OPCODE			0xF4
#define SYNC_OPCODE				0xFC
#define STANDBY_OPCODE			0xFD
#define RESET_OPCODE			0xFE
#define WAKEUP_OPCODE			0xFF



// Register address bytes used with RREG and WREG commands
#define MUX_REG_ADDR			0x01
#define ADCON_REG_ADDR			0x02
#define DRATE_REG_ADDR			0x03
#define IO_REG_ADDR				0x04
#define OFC0_REG_ADDR			0x05
#define OFC1_REG_ADDR			0x06
#define OFC2_REG_ADDR			0x07
#define FSC0_REG_ADDR			0x08
#define FSC1_REG_ADDR			0x09
#define FSC2_REG_ADDR			0x0A


/*----------------------------------------------------------------------------*/
/* Register definitions */
/*----------------------------------------------------------------------------*/

/* Register 0x00 (STATUS) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                ID[3:0]                |  ORDER  |   ACAL  |  BUFEN  |   DRDY  |
 * ---------------------------------------------------------------------------------
 */

    /* STATUS register address */
    #define ADS1256_REG_ADDR_STATUS									0x00

    /* STATUS register default */
    #define ADS1256_REG_DEFAULT_STATUS                              0x01

    /* STATUS register field masks */
    #define ADS1256_STATUS_ID_MASK                                  0xF0
    #define ADS1256_ORDER_MASK                                      0xF0
    #define ADS1256_STATUS_ACAL_MASK                                0xF0
    #define ADS1256_STATUS_BUFEN_MASK                               0xF0
    #define ADS1256_STATUS_DRDY_MASK                                0xF0

    /* FIELD DEFINITION: ORDER (Bit 3) 
     * DESCRIPTION: ORDER: Data Output Bit Order
     * 0 = Most Significant Bit First (default)
     * 1 = Least Significant Bit First
     * Input data is always shifted in most significant byte and bit first. Output data is always shifted out most significant
     * byte first. The ORDER bit only controls the bit order of the output data within the byte.
     */
    #define ADS1256_STATUS_ORDER_MSBFIRST		                                0x00
    #define ADS1256_STATUS_ORDER_LSBFIRST		                                0x08

    /* ACAL (auto-calibration)
#define ADS_ACAL_DISABLED		0x00
#define ADS_ACAL_ENABLED		0x04
// Define BUFEN (buffer)
#define ADS_BUFEN_DISABLED		0x00
#define ADS_BUFEN_ENABLED		0x02
// Define DRDY (data ready) (READ-ONLY)
#define ADS_DRDY_MASK			0x01


/* ADS1257 Register 1 (MUX) Definition */
//|   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
//-------------------------------------------------------------------------------------------------
//| RESERVED  |  RESERVED |       PSEL[1:0]       | RESERVED  |  RESERVED |       NSEL[1:0]       |
//
// Reset register value (0000 0001b)
#define MUX_RESET_VALUE			0x01
// Define the ADC positive input channels
#define ADS_PSEL_AIN0			0x00
#define ADS_PSEL_AIN1			0x10
#define ADS_PSEL_AIN2			0x20
#define ADS_PSEL_AIN3			0x30
//  Define the ADC negative input channels
#define ADS_NSEL_AIN0			0x00
#define ADS_NSEL_AIN1			0x01
#define ADS_NSEL_AIN2			0x02
#define ADS_NSEL_AIN3			0x03


/* ADS1257 Register 2 (ADCON) Definition */
//|   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
//-------------------------------------------------------------------------------------------------
//|  RESERVED |       CLK[1:0]        |       SDCS[1:0]       |       		PGA[2:0]              |
//
// Reset register value (0010 0000b)
#define ADCON_RESET_VALUE		0x20
// Define DO/CLKOUT clock output rate setting
#define ADS_CLK_OFF				0x00
#define ADS_CLK_1				0x20
#define ADS_CLK_0p5				0x40
#define ADS_CLK_0p25			0x60
// Define sensor detect current source setting
#define ADS_SDCS_OFF			0x00
#define ADS_SDCS_0p5uA			0x08
#define ADS_SDCS_2uA			0x10
#define ADS_SDCS_10uA			0x18
// Define PGA gain setting
#define ADS_PGA_1				0x00
#define ADS_PGA_2				0x01
#define ADS_PGA_4				0x02
#define ADS_PGA_8				0x03
#define ADS_PGA_16				0x04
#define ADS_PGA_32				0x05
#define ADS_PGA_64				0x06


/* ADS1257 Register 3 (DRATE) Definition */
//|   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
//-------------------------------------------------------------------------------------------------
//|   										DR[7:0]												  |
//
// Reset register value (1111 0000b)
#define DRATE_RESET_VALUE		0xF0
// Define ADC data rate setting
#define ADS_DR_2P5				0x03
#define ADS_DR_5				0x13
#define ADS_DR_10				0x23
#define ADS_DR_15				0x33
#define ADS_DR_25				0x43
#define ADS_DR_30				0x53
#define ADS_DR_50				0x63
#define ADS_DR_60				0x72
#define ADS_DR_100				0x82
#define ADS_DR_500				0x92
#define ADS_DR_1000				0xA1
#define ADS_DR_2000				0xB0
#define ADS_DR_3750				0xC0
#define ADS_DR_7500				0xD0
#define ADS_DR_15000			0xE0
#define ADS_DR_30000			0xF0


/* ADS1257 Register 4 (GPIO) Definition */
//|   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
//-------------------------------------------------------------------------------------------------
//| RESERVED  | RESERVED  |   	  DIR[1:0]	      | RESERVED  | RESERVED  |       DIO[2:0]        |
//
// Reset register value (1110 0000b)
#define GPIO_RESET_VALUE		0xE0
// Define gpio I/O directions
#define ADS_DIR_D0_OUTPUT		0xC0
#define ADS_DIR_D0_INPUT		0xD0
#define ADS_DIR_D1_OUTPUT		0xC0
#define ADS_DIR_D1_INPUT		0xE0
// Define gpio status masks
#define ADS_D0_MASK				0x01
#define ADS_D1_MASK				0x02


/* ADS1257 Register 5 (OFC0) Definition */
//|   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
//-------------------------------------------------------------------------------------------------
//                                         OFC0[7:0] (OFC[7:0] LSB)
// Reset register value depends on calibration results
#define OFC0_IDEAL_VALUE		0x00


/* ADS1257 Register 6 (OFC1) Definition */
//|   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
//-------------------------------------------------------------------------------------------------
//                                         OFC1[7:0] (OFC[15:8])
// Reset register value depends on calibration results
#define OFC1_IDEAL_VALUE		0x00

/* ADS1257 Register 7 (OFC2) Definition */
//|   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
//-------------------------------------------------------------------------------------------------
//                                         OFC2[7:0] (MSB OFC[23:16])
// Reset register value depends on calibration results
#define OFC2_IDEAL_VALUE		0x00

/* ADS1257 Register 8 (FSC0) Definition */
//|   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
//-------------------------------------------------------------------------------------------------
//                                         FSC0[7:0] (FSC[7:0] LSB)
// Reset register value depends on calibration results
#define FSC0_IDEAL_VALUE_10				0x4C
#define FSC0_IDEAL_VALUE_60				0xF3
//#define FSC0_IDEAL_VALUE_3K_TO_30K		0x08
#define FSC0_IDEAL_VALUE_15000			0x08


/* ADS1257 Register 9 (FSC1) Definition */
//|   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
//-------------------------------------------------------------------------------------------------
//                                         FSC1[7:0] (FSC[15:8])
// Reset register value depends on calibration results
#define FSC1_IDEAL_VALUE_10				0xE1
#define FSC1_IDEAL_VALUE_60				0x51
//#define FSC1_IDEAL_VALUE_3K_TO_30K		0xAC
#define FSC1_IDEAL_VALUE_15000			0xAC

/* ADS1257 Register A (FSC2) Definition */
//|   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
//-------------------------------------------------------------------------------------------------
//                                         FSC2[7:0] (MSB FSC[23:16])
// Reset register value depends on calibration results
#define FSC2_IDEAL_VALUE_10				0x2E
#define FSC2_IDEAL_VALUE_60				0x46
//#define FSC2_IDEAL_VALUE_3K_TO_30K		0x44
#define FSC2_IDEAL_VALUE_15000			0x44


#endif /* ADS1257_H_ */
