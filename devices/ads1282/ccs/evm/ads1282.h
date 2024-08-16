/**
 * @file ads1282.h
 *
 * @brief ADS1282 Descriptor
 *
 * @copyright Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef ADS1282_H_
#define ADS1282_H_

// Standard libraries
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

// Custom libraries
#include "hal.h"


//*****************************************************************************
//
// Constants
//
//*****************************************************************************

/// Total number of device registers
#define NUM_REGISTERS 					        ((uint8_t) 11)

/// Alias for setting GPIOs pins to the logic "high" state
#define HIGH                ((bool) true)

/// Alias for setting GPIOs pins to the logic "low" state
#define LOW                 ((bool) false)

/// TIMING REQUIREMENT: "tDLY" (time for DOUT to update during RREG command)
///              "tDLY" = 24/fCLK = 5.89 us @ 4.096 MHz
/// NOTE: This delay value may need to be increased to account for rounding errors
/// when using an external clock frequency other than 4.096 MHz. However, most
/// micro-controllers will have inherent delays between functions calls that
/// will add to this delay and in many cases more than account for the rounding
/// error. If that is the case, this value may be reduced instead.
#define DELAY_T_DLY         ((uint32_t) 6 * (4096000/FCLK_FREQ_HZ))


//*****************************************************************************
//
// Commands
//
//*****************************************************************************

#define OPCODE_NOP                          ((uint8_t) 0x00)
#define OPCODE_WAKEUP				        ((uint8_t) 0x00)    // Use 0x00 or 0x01
#define OPCODE_STANDBY				        ((uint8_t) 0x03)    // Use 0x02 or 0x03
#define OPCODE_SYNC							((uint8_t) 0x05)    // Use 0x04 or 0x05
#define OPCODE_RESET					    ((uint8_t) 0x07)    // Use 0x06 or 0x07
#define OPCODE_RDATAC				        ((uint8_t) 0x10)
#define OPCODE_SDATAC						((uint8_t) 0x11)
#define OPCODE_RDATA                        ((uint8_t) 0x12)
#define OPCODE_RREG                         ((uint8_t) 0x20)    // OR'd with register address
#define OPCODE_WREG                         ((uint8_t) 0x40)    // OR'd with register address
#define OPCODE_REG_ADDR_MASK                ((uint8_t) 0x1F)    // Location of address bits in RREG/WREG commands (1st byte)
#define OPCODE_REG_COUNT_MASK               ((uint8_t) 0x1F)    // Location of count bits in RREG/WREG commands (2nd byte)
#define OPCODE_OFSCAL                       ((uint8_t) 0x60)
#define OPCODE_GANCAL                       ((uint8_t) 0x61)

 /* Read mode enum */
typedef enum { DIRECT, COMMAND } readMode;


//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

void    adcStartupRoutine(void);
uint8_t readSingleRegister(const uint8_t address);
void    readMultipleRegisters(const uint8_t startAddress, const uint8_t count);
void    writeSingleRegister(const uint8_t address, const uint8_t data);
void    writeMultipleRegisters(const uint8_t startAddress, const uint8_t count, const uint8_t regData[]);
int32_t readData(void);
void    sendCommand(const uint8_t op_code);

// Getters...
uint8_t getRegisterValue(const uint8_t address);

// Internal Use Functions...
// NOTE: Avoid calling these functions in the application code except when absolutely needed
// as these functions affect the internal state variables in the ads1282.c module.
void    _restoreRegisterDefaults(void);
#ifndef EXAMPLE_CODE
// These functions to be shwon in doxygen instead of example code source code...
// We keep a reference to them here to allow us to test this code.
int32_t example_readDataByCommand(void);
int32_t example_readDataContinuously(void);
#endif


//*****************************************************************************
//
// Register macros
//
//*****************************************************************************

#define FILTR_SETTING       ((uint8_t) (getRegisterValue(CONFIG0_ADDRESS) & CONFIG0_FILTR_MASK))
#define FILTR_BYPASSED      ((bool) (FILTR_SETTING == CONFIG0_FILTR_MODMODE))
#define FILTR_SINC_ONLY     ((bool) (FILTR_SETTING == CONFIG0_FILTR_SINC))
#ifndef EXAMPLE_CODE
    #define DATA_RATE           ((uint8_t) (getRegisterValue(CONFIG0_ADDRESS) & CONFIG0_DR_MASK))
    #define DECIMATION_RATIO    (((uint16_t) (128 / ((DATA_RATE >> 3) + 1)) * FILTR_SINC_ONLY ? 1 : 32))
    //#define OSR             ((uint8_t) CONFIG0_DR_MASK)
#endif


//*****************************************************************************
//
// Register definitions
//
//*****************************************************************************

/* Register 0x00 (ID) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |                ID[3:0]                |                   0                   |
 * ---------------------------------------------------------------------------------
 */

    /* ID register */
    #define ID_ADDRESS                          ((uint8_t) 0x00)
    #define ID_RESET_MASK                       ((uint8_t) 0x0F)    // Only lower nibble is determinate after rest
    #define ID_DEFAULT                          ((uint8_t) 0x00)

    /* Factory-programmed identification bits (read-only) */
    #define ID_ID_MASK                          ((uint8_t) 0xF0)


/* Register 0x01 (CONFIG0) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |   SYNC  |    1    |            DR[2:0]          |  PHASE  |     FILTR[0:1]    |
* ---------------------------------------------------------------------------------
*/

    /* CONFIG0 register */
    #define CONFIG0_ADDRESS                     ((uint8_t) 0x01)
    #define CONFIG0_DEFAULT                     ((uint8_t) 0x52)

    /* Synchronization mode */
    #define CONFIG0_SYNC_MASK                   ((uint8_t) 0x80)
    #define CONFIG0_SYNC_PULSE                  ((uint8_t) 0x00 << 7)   // default
    #define CONFIG0_SYNC_CONTINUOUS             ((uint8_t) 0x01 << 7)

    /* Data Rate Select */
    // NOTE: Specified data rates valid only in FIR filter mode with nominal 4.096 MHz FCLK
    #define CONFIG0_DR_MASK                     ((uint8_t) 0x38)
    #define CONFIG0_DR_250SPS                   ((uint8_t) 0x00 << 3)
    #define CONFIG0_DR_500SPS                   ((uint8_t) 0x01 << 3)
    #define CONFIG0_DR_1000SPS                  ((uint8_t) 0x02 << 3)   // default
    #define CONFIG0_DR_2000SPS                  ((uint8_t) 0x03 << 3)
    #define CONFIG0_DR_4000SPS                  ((uint8_t) 0x04 << 3)

    /* FIR Phase Response */
    #define CONFIG0_PHASE_MASK                  ((uint8_t) 0x04)
    #define CONFIG0_PHASE_LINEAR                ((uint8_t) 0x00 << 2)   // default
    #define CONFIG0_PHASE_MINIMUM               ((uint8_t) 0x01 << 2)

    /* Digital Filter Select */
    #define CONFIG0_FILTR_MASK                  ((uint8_t) 0x03)
    #define CONFIG0_FILTR_MODMODE               ((uint8_t) 0x00 << 0)
    #define CONFIG0_FILTR_SINC                  ((uint8_t) 0x01 << 0)
    #define CONFIG0_FILTR_SINC_LPF              ((uint8_t) 0x02 << 0)   // default
    #define CONFIG0_FILTR_SINC_LPF_HPF          ((uint8_t) 0x03 << 0)


/* Register 0x02 (CONFIG1) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |    0    |           MUX[2:0]          |   CHOP  |           PGA[3:0]          |
* ---------------------------------------------------------------------------------
*/

    /* CONFIG1 register */
    #define CONFIG1_ADDRESS                     ((uint8_t) 0x02)
    #define CONFIG1_DEFAULT                     ((uint8_t) 0x08)

    /* MUX Select */
    #define CONFIG1_MUX_MASK                    ((uint8_t) 0x70)
    #define CONFIG1_MUX_AINP1_AINN1             ((uint8_t) 0x00 << 4)   // default
    #define CONFIG1_MUX_AINP2_AINN2             ((uint8_t) 0x01 << 4)
    #define CONFIG1_MUX_INT_400OHM_SHORT        ((uint8_t) 0x02 << 4)
    #define CONFIG1_MUX_AINX1_AINX2             ((uint8_t) 0x03 << 4)
    #define CONFIG1_MUX_EXT_SHORT_AINN2         ((uint8_t) 0x04 << 4)

    /* PGA Chopping Enable */
    #define CONFIG1_CHOP_MASK                   ((uint8_t) 0x08)
    #define CONFIG1_CHOP_DISABLED               ((uint8_t) 0x00 << 3)
    #define CONFIG1_CHOP_ENDABLED               ((uint8_t) 0x01 << 3)   // default

    /* PGA Gain Select */
    #define CONFIG1_PGA_MASK                    ((uint8_t) 0x07)
    #define CONFIG1_PGA_1                       ((uint8_t) 0x00 << 0)
    #define CONFIG1_PGA_2                       ((uint8_t) 0x01 << 0)
    #define CONFIG1_PGA_4                       ((uint8_t) 0x02 << 0)
    #define CONFIG1_PGA_8                       ((uint8_t) 0x03 << 0)
    #define CONFIG1_PGA_16                      ((uint8_t) 0x04 << 0)
    #define CONFIG1_PGA_32                      ((uint8_t) 0x05 << 0)
    #define CONFIG1_PGA_64                      ((uint8_t) 0x06 << 0)


/* Register 0x03 (HPF0) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |                                   HP[07:00]                                   |
* ---------------------------------------------------------------------------------
*/

    /* HPF0 register */
    #define HPF0_ADDRESS                        ((uint8_t) 0x03)
    #define HPF0_DEFAULT                        ((uint8_t) 0x32)


/* Register 0x04 (HPF1) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |                                   HP[15:08]                                   |
* ---------------------------------------------------------------------------------
*/

    /* HPF1 register */
    #define HPF1_ADDRESS                        ((uint8_t) 0x04)
    #define HPF1_DEFAULT                        ((uint8_t) 0x03)


/* Register 0x05 (OFC0) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |                                   OC[07:00]                                   |
* ---------------------------------------------------------------------------------
*/

    /* OFC0 register */
    #define OFC0_ADDRESS                        ((uint8_t) 0x05)
    #define OFC0_DEFAULT                        ((uint8_t) 0x00)


/* Register 0x06 (OFC1) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |                                   OC[15:08]                                   |
* ---------------------------------------------------------------------------------
*/

    /* OFC1 register */
    #define OFC1_ADDRESS                        ((uint8_t) 0x06)
    #define OFC1_DEFAULT                        ((uint8_t) 0x00)


/* Register 0x07 (OFC2) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |                                   OC[23:16]                                   |
* ---------------------------------------------------------------------------------
*/

    /* OFC2 register */
    #define OFC2_ADDRESS                        ((uint8_t) 0x07)
    #define OFC2_DEFAULT                        ((uint8_t) 0x00)


/* Register 0x08 (FSC0) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |                                  FSC[07:00]                                   |
* ---------------------------------------------------------------------------------
*/

    /* FSC0 register */
    #define FSC0_ADDRESS                        ((uint8_t) 0x08)
    #define FSC0_DEFAULT                        ((uint8_t) 0x00)


/* Register 0x09 (FSC1) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |                                  FSC[15:08]                                   |
* ---------------------------------------------------------------------------------
*/

    /* FSC1 register */
    #define FSC1_ADDRESS                        ((uint8_t) 0x09)
    #define FSC1_DEFAULT                        ((uint8_t) 0x00)


/* Register 0x0A (FSC2) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |                                  FSC[23:16]                                   |
* ---------------------------------------------------------------------------------
*/

    /* FSC2 register */
    #define FSC2_ADDRESS                        ((uint8_t) 0x0A)
    #define FSC2_DEFAULT                        ((uint8_t) 0x40)


#endif /* ADS1282_H_ */
