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


#ifndef ADS1256_H_
#define ADS1256_H_

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "hal.h"

// Device command prototypes
extern uint8_t InitDevice(void);
extern void readRegs(uint8_t regnum, uint8_t count);
extern void writeRegs(unsigned int regnum, unsigned int count, unsigned char *data);
extern void reset_adc_sclk(void);
extern void reset_adc_sw(void);



#define NUM_REGISTERS 		        ((uint8_t) 11)
#define DATA_LENGTH_BYTES           ((uint8_t) 3)

// Lengths of conversion data components
#define ADS1256_DATA_LENGTH			3					// 24-bit data (3 bytes)

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



//**********************************************************************************
//
// Device commands
//
//**********************************************************************************

#define OPCODE_NOP				((uint8_t) 0x00)
#define OPCODE_RDATA			((uint8_t) 0x01)
#define OPCODE_RDATAC			((uint8_t) 0x03)
#define OPCODE_SDATAC			((uint8_t) 0x0F)
#define OPCODE_RREG				((uint8_t) 0x10)
#define OPCODE_WREG				((uint8_t) 0x50)
#define OPCODE_SELFCAL			((uint8_t) 0xF0)
#define OPCODE_SELFOCAL			((uint8_t) 0xF1)
#define OPCODE_SELFGCAL			((uint8_t) 0xF2)
#define OPCODE_SYSOCAL			((uint8_t) 0xF3)
#define OPCODE_SYSGCAL			((uint8_t) 0xF4)
#define OPCODE_SYNC				((uint8_t) 0xFC)
#define OPCODE_STANDBY			((uint8_t) 0xFD)
#define OPCODE_RESET			((uint8_t) 0xFE)
#define OPCODE_WAKEUP			((uint8_t) 0xFF)

/* Read mode enum */
typedef enum { DIRECT, COMMAND } readMode;



//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

void adcStartup(void);
uint8_t readSingleRegister(uint8_t address);
//void    readMultipleRegisters(uint8_t startAddress, uint8_t count);
void    writeSingleRegister(uint8_t address, uint8_t data);
//void    writeMultipleRegisters(uint8_t startAddress, uint8_t count, const uint8_t regData[]);
void    sendCommand(uint8_t op_code);
int32_t readData(uint8_t status[], uint8_t data[], readMode mode);

void    startConversions(void);

// Internal variable getters
uint8_t getRegisterValue(uint8_t address);

// Internal variable setters
void    restoreRegisterDefaults(void);



//**********************************************************************************
//
// Register definitions
//
//**********************************************************************************


/* Register 0x00 (STATUS) definition
 * |---------------------------------------------------------------------------------------|
 * |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * |---------------------------------------------------------------------------------------|
 * |                  ID[3:0]                  |   ORDER  |   ACAL   |   BUFEN  |   NDRDY  |
 * |---------------------------------------------------------------------------------------|
 */

    /* STATUS register address */
    #define STATUS_ADDRESS													((uint8_t) 0x00)

    /* STATUS default (reset) value */
    #define STATUS_DEFAULT													((uint8_t) 0x01)
    #define STATUS_DEFAULT_MASK												((uint8_t) 0x0E)

    /* STATUS register field masks */
    #define STATUS_ID_MASK													((uint8_t) 0xF0)
    #define STATUS_ORDER_MASK												((uint8_t) 0x08)
    #define STATUS_ACAL_MASK												((uint8_t) 0x04)
    #define STATUS_BUFEN_MASK												((uint8_t) 0x02)
    #define STATUS_NDRDY_MASK												((uint8_t) 0x01)

    /* ORDER field values */
    #define STATUS_ORDER_MSB_FIRST											((uint8_t) 0x00)
    #define STATUS_ORDER_LSB_FIRST											((uint8_t) 0x08)

    /* ACAL field values */
    #define STATUS_ACAL_DISABLED											((uint8_t) 0x00)
    #define STATUS_ACAL_ENABLED												((uint8_t) 0x04)

    /* BUFEN field values */
    #define STATUS_BUFEN_DISABLED											((uint8_t) 0x00)
    #define STATUS_BUFEN_ENABLED											((uint8_t) 0x02)



/* Register 0x01 (MUX) definition
 * |---------------------------------------------------------------------------------------|
 * |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * |---------------------------------------------------------------------------------------|
 * |                 PSEL[3:0]                 |                 NSEL[3:0]                 |
 * |---------------------------------------------------------------------------------------|
 */

    /* MUX register address */
    #define MUX_ADDRESS														((uint8_t) 0x01)

    /* MUX default (reset) value */
    #define MUX_DEFAULT														((uint8_t) 0x01)

    /* MUX register field masks */
    #define MUX_PSEL_MASK													((uint8_t) 0xF0)
    #define MUX_NSEL_MASK													((uint8_t) 0x0F)

    /* PSEL field values */
    #define MUX_PSEL_AIN0													((uint8_t) 0x00)
    #define MUX_PSEL_AIN1													((uint8_t) 0x10)
    #define MUX_PSEL_AIN2													((uint8_t) 0x20)
    #define MUX_PSEL_AIN3													((uint8_t) 0x30)
    #define MUX_PSEL_AIN4													((uint8_t) 0x40)
    #define MUX_PSEL_AIN5													((uint8_t) 0x50)
    #define MUX_PSEL_AIN6													((uint8_t) 0x60)
    #define MUX_PSEL_AIN7													((uint8_t) 0x70)
    #define MUX_PSEL_AINCOM													((uint8_t) 0x80)

    /* NSEL field values */
    #define MUX_NSEL_AIN0													((uint8_t) 0x00)
    #define MUX_NSEL_AIN1													((uint8_t) 0x01)
    #define MUX_NSEL_AIN2													((uint8_t) 0x02)
    #define MUX_NSEL_AIN3													((uint8_t) 0x03)
    #define MUX_NSEL_AIN4													((uint8_t) 0x04)
    #define MUX_NSEL_AIN5													((uint8_t) 0x05)
    #define MUX_NSEL_AIN6													((uint8_t) 0x06)
    #define MUX_NSEL_AIN7													((uint8_t) 0x07)
    #define MUX_NSEL_AINCOM													((uint8_t) 0x08)



/* Register 0x02 (ADCON) definition
 * |---------------------------------------------------------------------------------------|
 * |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * |---------------------------------------------------------------------------------------|
 * |     0    |       CLK[1:0]      |      SDCS[1:0]      |            PGA[2:0]            |
 * |---------------------------------------------------------------------------------------|
 */

    /* ADCON register address */
    #define ADCON_ADDRESS													((uint8_t) 0x02)

    /* ADCON default (reset) value */
    #define ADCON_DEFAULT													((uint8_t) 0xA0)

    /* ADCON register field masks */
    #define ADCON_0_MASK													((uint8_t) 0x80)
    #define ADCON_CLK_MASK													((uint8_t) 0x60)
    #define ADCON_SDCS_MASK													((uint8_t) 0x18)
    #define ADCON_PGA_MASK													((uint8_t) 0x07)

    /* CLK field values */
    #define ADCON_CLK_CLK_OFF												((uint8_t) 0x00)
    #define ADCON_CLK_CLK_DIV1												((uint8_t) 0x20)
    #define ADCON_CLK_CLK_DIV2												((uint8_t) 0x40)
    #define ADCON_CLK_CLK_DIV4												((uint8_t) 0x60)

    /* SDCS field values */
    #define ADCON_SDCS_SDCS_OFF												((uint8_t) 0x00)
    #define ADCON_SDCS_SDCS_0p5uA											((uint8_t) 0x08)
    #define ADCON_SDCS_SDCS_2uA												((uint8_t) 0x10)
    #define ADCON_SDCS_SDCS_10uA											((uint8_t) 0x18)

    /* PGA field values */
    #define ADCON_PGA_1														((uint8_t) 0x00)
    #define ADCON_PGA_2														((uint8_t) 0x01)
    #define ADCON_PGA_4														((uint8_t) 0x02)
    #define ADCON_PGA_8														((uint8_t) 0x03)
    #define ADCON_PGA_16													((uint8_t) 0x04)
    #define ADCON_PGA_32													((uint8_t) 0x05)
    #define ADCON_PGA_64													((uint8_t) 0x06)



/* Register 0x03 (DRATE) definition
 * |---------------------------------------------------------------------------------------|
 * |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * |---------------------------------------------------------------------------------------|
 * |                                        DR[7:0]                                        |
 * |---------------------------------------------------------------------------------------|
 */

    /* DRATE register address */
    #define DRATE_ADDRESS													((uint8_t) 0x03)

    /* DRATE default (reset) value */
    #define DRATE_DEFAULT													((uint8_t) 0xF0)

    /* DRATE register field masks */
    #define DRATE_DR_MASK													((uint8_t) 0xFF)

    /* DR field values */
    #define DRATE_DR_2p5_SPS												((uint8_t) 0x00)
    #define DRATE_DR_5_SPS													((uint8_t) 0x01)
    #define DRATE_DR_10_SPS													((uint8_t) 0x02)
    #define DRATE_DR_15_SPS													((uint8_t) 0x03)
    #define DRATE_DR_25_SPS													((uint8_t) 0x04)
    #define DRATE_DR_30_SPS													((uint8_t) 0x05)
    #define DRATE_DR_50_SPS													((uint8_t) 0x06)
    #define DRATE_DR_60_SPS													((uint8_t) 0x07)
    #define DRATE_DR_100_SPS												((uint8_t) 0x08)
    #define DRATE_DR_500_SPS												((uint8_t) 0x09)
    #define DRATE_DR_1000_SPS												((uint8_t) 0x0A)
    #define DRATE_DR_2000_SPS												((uint8_t) 0x0B)
    #define DRATE_DR_3750_SPS												((uint8_t) 0x0C)
    #define DRATE_DR_7500_SPS												((uint8_t) 0x0D)
    #define DRATE_DR_15000_SPS												((uint8_t) 0x0E)
    #define DRATE_DR_30000_SPS												((uint8_t) 0x0F)



/* Register 0x04 (IO) definition
 * |---------------------------------------------------------------------------------------|
 * |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * |---------------------------------------------------------------------------------------|
 * |   DIR3   |   DIR2   |   DIR1   |   DIR0   |   DIO3   |   DIO2   |   DIO1   |   DIO0   |
 * |---------------------------------------------------------------------------------------|
 */

    /* IO register address */
    #define IO_ADDRESS														((uint8_t) 0x04)

    /* IO default (reset) value */
    #define IO_DEFAULT														((uint8_t) 0xE0)

    /* IO register field masks */
    #define IO_DIR3_MASK													((uint8_t) 0x80)
    #define IO_DIR2_MASK													((uint8_t) 0x40)
    #define IO_DIR1_MASK													((uint8_t) 0x20)
    #define IO_DIR0_MASK													((uint8_t) 0x10)
    #define IO_DIO3_MASK													((uint8_t) 0x08)
    #define IO_DIO2_MASK													((uint8_t) 0x04)
    #define IO_DIO1_MASK													((uint8_t) 0x02)
    #define IO_DIO0_MASK													((uint8_t) 0x01)

    /* DIR3 field values */
    #define IO_DIR3_OUTPUT													((uint8_t) 0x00)
    #define IO_DIR3_INPUT													((uint8_t) 0x80)

    /* DIR2 field values */
    #define IO_DIR2_OUTPUT													((uint8_t) 0x00)
    #define IO_DIR2_INPUT													((uint8_t) 0x40)

    /* DIR1 field values */
    #define IO_DIR1_OUTPUT													((uint8_t) 0x00)
    #define IO_DIR1_INPUT													((uint8_t) 0x20)

    /* DIR0 field values */
    #define IO_DIR0_CLKOUT_OUTPUT											((uint8_t) 0x00)
    #define IO_DIR0_CLKOUT_INPUT											((uint8_t) 0x10)

    /* DIO3 field values */
    #define IO_DIO3_HIGH													((uint8_t) 0x00)
    #define IO_DIO3_LOW														((uint8_t) 0x08)

    /* DIO2 field values */
    #define IO_DIO2_HIGH													((uint8_t) 0x00)
    #define IO_DIO2_LOW														((uint8_t) 0x04)

    /* DIO1 field values */
    #define IO_DIO1_HIGH													((uint8_t) 0x00)
    #define IO_DIO1_LOW														((uint8_t) 0x02)

    /* DIO0 field values */
    #define IO_DIO0_HIGH													((uint8_t) 0x00)
    #define IO_DIO0_LOW														((uint8_t) 0x01)



/* Register 0x05 (OFC0) definition
 * |---------------------------------------------------------------------------------------|
 * |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * |---------------------------------------------------------------------------------------|
 * |                                        OFC[7:0]                                       |
 * |---------------------------------------------------------------------------------------|
 */

    /* OFC0 register address */
    #define OFC0_ADDRESS													((uint8_t) 0x05)

    /* OFC0 default (reset) value */
    #define OFC0_DEFAULT													((uint8_t) 0x00)

    /* OFC0 register field masks */
    #define OFC0_OFC_MASK													((uint8_t) 0xFF)



/* Register 0x06 (OFC1) definition
 * |---------------------------------------------------------------------------------------|
 * |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * |---------------------------------------------------------------------------------------|
 * |                                        OFC[7:0]                                       |
 * |---------------------------------------------------------------------------------------|
 */

    /* OFC1 register address */
    #define OFC1_ADDRESS													((uint8_t) 0x06)

    /* OFC1 default (reset) value */
    #define OFC1_DEFAULT													((uint8_t) 0x00)

    /* OFC1 register field masks */
    #define OFC1_OFC_MASK													((uint8_t) 0xFF)



/* Register 0x07 (OFC2) definition
 * |---------------------------------------------------------------------------------------|
 * |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * |---------------------------------------------------------------------------------------|
 * |                                        OFC[7:0]                                       |
 * |---------------------------------------------------------------------------------------|
 */

    /* OFC2 register address */
    #define OFC2_ADDRESS													((uint8_t) 0x07)

    /* OFC2 default (reset) value */
    #define OFC2_DEFAULT													((uint8_t) 0x00)

    /* OFC2 register field masks */
    #define OFC2_OFC_MASK													((uint8_t) 0xFF)



/* Register 0x08 (FSC0) definition
 * |---------------------------------------------------------------------------------------|
 * |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * |---------------------------------------------------------------------------------------|
 * |                                        FSC[7:0]                                       |
 * |---------------------------------------------------------------------------------------|
 */

    /* FSC0 register address */
    #define FSC0_ADDRESS													((uint8_t) 0x08)

    /* FSC0 default (reset) value */
    #define FSC0_DEFAULT													((uint8_t) 0x00)

    /* FSC0 register field masks */
    #define FSC0_FSC_MASK													((uint8_t) 0xFF)



/* Register 0x09 (FSC1) definition
 * |---------------------------------------------------------------------------------------|
 * |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * |---------------------------------------------------------------------------------------|
 * |                                        FSC[7:0]                                       |
 * |---------------------------------------------------------------------------------------|
 */

    /* FSC1 register address */
    #define FSC1_ADDRESS													((uint8_t) 0x09)

    /* FSC1 default (reset) value */
    #define FSC1_DEFAULT													((uint8_t) 0x00)

    /* FSC1 register field masks */
    #define FSC1_FSC_MASK													((uint8_t) 0xFF)



/* Register 0x0A (FSC2) definition
 * |---------------------------------------------------------------------------------------|
 * |   Bit 7  |   Bit 6  |   Bit 5  |   Bit 4  |   Bit 3  |   Bit 2  |   Bit 1  |   Bit 0  |
 * |---------------------------------------------------------------------------------------|
 * |                                        FSC[7:0]                                       |
 * |---------------------------------------------------------------------------------------|
 */

    /* FSC2 register address */
    #define FSC2_ADDRESS													((uint8_t) 0x0A)

    /* FSC2 default (reset) value */
    #define FSC2_DEFAULT													((uint8_t) 0x00)

    /* FSC2 register field masks */
    #define FSC2_FSC_MASK													((uint8_t) 0xFF)



//*****************************************************************************
//
// Macros       // TODO: Update all macros...
//
//*****************************************************************************

/** Register bit checking macros...
 *  Return true if register bit is set (since last read or write).
 */
#define IS_MUXMOD_SET       ((bool) (getRegisterValue(REG_ADDR_CONFIG0) & CONFIG0_MUXMOD_MASK))
#define IS_STAT_SET         ((bool) (getRegisterValue(REG_ADDR_CONFIG0) & CONFIG0_STAT_MASK))

#endif /* ADS1256_H_ */
