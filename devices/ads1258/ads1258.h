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

#ifndef ADS1258_H_
#define ADS1258_H_


#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "hal.h"

#ifdef EXAMPLE_CODE
#else

// TODO: Either remove this section or add defines to ads1258.c to allow for compatibility with ADS1158!!

//*****************************************************************************
//
// TODO: Select which device you are using...
//
//*****************************************************************************
//#define ADS1158
#define ADS1258
#endif


//*****************************************************************************
//
// Constants
//
//*****************************************************************************
#define NUM_REGISTERS 					        ((uint8_t) 10)



//*****************************************************************************
//
// Command byte formatting
//
//*****************************************************************************

/* Command byte definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |            C[2:0]           |   MUL   |                A[3:0]                 |
 * ---------------------------------------------------------------------------------
 */

    /* SPI Commands */
    #define OPCODE_READ_DIRECT				    ((uint8_t) 0x00)
    #define OPCODE_READ_COMMAND					((uint8_t) 0x30)    // Includes MUL bit
    #define OPCODE_RREG							((uint8_t) 0x40)
    #define OPCODE_WREG							((uint8_t) 0x60)
    #define OPCODE_PULSE_CONVERT				((uint8_t) 0x80)
    #define OPCODE_RESET						((uint8_t) 0xC0)

    /* Commands byte masks */
    #define OPCODE_C_MASK                       ((uint8_t) 0xE0)
    #define OPCODE_MUL_MASK                     ((uint8_t) 0x10)
    #define OPCODE_A_MASK                       ((uint8_t) 0x0F)

    /* Read mode enum */
    typedef enum { DIRECT, COMMAND } readMode;



//*****************************************************************************
//
// Status byte formatting
//
//*****************************************************************************

/* STATUS byte definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |   NEW   |   OVF   |  SUPPLY |                    CHID[4:0]                    |
 * ---------------------------------------------------------------------------------
 */

    /* STATUS byte field masks */
    #define STATUS_NEW_MASK                     ((uint8_t) 0x80)  /* Indicates new data */
    #define STATUS_OVF_MASK                     ((uint8_t) 0x40)  /* Indicates differential over-range condition */
    #define STATUS_SUPPLY_MASK                  ((uint8_t) 0x20)  /* Indicates low analog power-supply condition */
    #define STATUS_CHID_MASK                    ((uint8_t) 0x1F)  /* Channel ID bits */

    /* CHID field values */
    #define STATUS_CHID_DIFF0                   ((uint8_t) 0x00)
    #define STATUS_CHID_DIFF1                   ((uint8_t) 0x01)
    #define STATUS_CHID_DIFF2                   ((uint8_t) 0x02)
    #define STATUS_CHID_DIFF3                   ((uint8_t) 0x03)
    #define STATUS_CHID_DIFF4                   ((uint8_t) 0x04)
    #define STATUS_CHID_DIFF5                   ((uint8_t) 0x05)
    #define STATUS_CHID_DIFF6                   ((uint8_t) 0x06)
    #define STATUS_CHID_DIFF7                   ((uint8_t) 0x07)
    #define STATUS_CHID_AIN0                    ((uint8_t) 0x08)
    #define STATUS_CHID_AIN1                    ((uint8_t) 0x09)
    #define STATUS_CHID_AIN2                    ((uint8_t) 0x0A)
    #define STATUS_CHID_AIN3                    ((uint8_t) 0x0B)
    #define STATUS_CHID_AIN4                    ((uint8_t) 0x0C)
    #define STATUS_CHID_AIN5                    ((uint8_t) 0x0D)
    #define STATUS_CHID_AIN6                    ((uint8_t) 0x0E)
    #define STATUS_CHID_AIN7                    ((uint8_t) 0x0F)
    #define STATUS_CHID_AIN8                    ((uint8_t) 0x10)
    #define STATUS_CHID_AIN9                    ((uint8_t) 0x11)
    #define STATUS_CHID_AIN10                   ((uint8_t) 0x12)
    #define STATUS_CHID_AIN11                   ((uint8_t) 0x13)
    #define STATUS_CHID_AIN12                   ((uint8_t) 0x14)
    #define STATUS_CHID_AIN13                   ((uint8_t) 0x15)
    #define STATUS_CHID_AIN14                   ((uint8_t) 0x16)
    #define STATUS_CHID_AIN15                   ((uint8_t) 0x17)
    #define STATUS_CHID_OFFSET                  ((uint8_t) 0x18)
    #define STATUS_CHID_VCC                     ((uint8_t) 0x1A)
    #define STATUS_CHID_TEMP                    ((uint8_t) 0x1B)
    #define STATUS_CHID_GAIN                    ((uint8_t) 0x1C)
    #define STATUS_CHID_REF                     ((uint8_t) 0x1D)
    #define STATUS_CHID_FIXEDCHMODE             ((uint8_t) 0x1F)  /* ID for fixed-channel mode */



//*****************************************************************************
//
// Register definitions
//
//*****************************************************************************


/* Register 0x00 (CONFIG0) definition
 * ---------------------------------------------------------------------------------
 * |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
 * ---------------------------------------------------------------------------------
 * |    0    |  SPIRST |  MUXMOD |  BYPAS  |  CLKENB |   CHOP  |   STAT  |    0    |
 * ---------------------------------------------------------------------------------
 */

    /** CONFIG0 register address */
    #define REG_ADDR_CONFIG0                    ((uint8_t) 0x00)

    /** CONFIG0 default (reset) value */
    #define CONFIG0_DEFAULT                     ((uint8_t) 0x0A)

    /* CONFIG0 register field masks */
    #define CONFIG0_SPIRST_MASK                 ((uint8_t) 0x40)
    #define CONFIG0_MUXMOD_MASK                 ((uint8_t) 0x20)
    #define CONFIG0_BYPAS_MASK                  ((uint8_t) 0x10)
    #define CONFIG0_CLKENB_MASK                 ((uint8_t) 0x08)
    #define CONFIG0_CHOP_MASK                   ((uint8_t) 0x04)
    #define CONFIG0_STAT_MASK                   ((uint8_t) 0x02)



/* Register 0x01 (CONFIG1) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |  IDLMOD |           DLY[2:0]          |     SCBCS[1:0]    |     DRATE[0:1]    |
* ---------------------------------------------------------------------------------
*/

    /** CONFIG1 register address */
    #define REG_ADDR_CONFIG1                    ((uint8_t) 0x01)

    /** CONFIG1 default (reset) value */
    #define CONFIG1_DEFAULT                     ((uint8_t) 0x83)

    /* CONFIG1 register field masks */
    #define CONFIG1_IDLMOD_MASK                 ((uint8_t) 0x80)
    #define CONFIG1_DLY_MASK                    ((uint8_t) 0x70)
    #define CONFIG1_SCBCS_MASK                  ((uint8_t) 0x0C)
    #define CONFIG1_DRATE_MASK                  ((uint8_t) 0x03)

    /* DLY field values */
    #define CONFIG1_DLY_0us                     ((uint8_t) 0x00)
    #define CONFIG1_DLY_8us                     ((uint8_t) 0x10)
    #define CONFIG1_DLY_16us                    ((uint8_t) 0x20)
    #define CONFIG1_DLY_32us                    ((uint8_t) 0x30)
    #define CONFIG1_DLY_64us                    ((uint8_t) 0x40)
    #define CONFIG1_DLY_128us                   ((uint8_t) 0x50)
    #define CONFIG1_DLY_256us                   ((uint8_t) 0x60)
    #define CONFIG1_DLY_384us                   ((uint8_t) 0x70)

    /* SCBCS field values */
    #define CONFIG1_SCBCS_OFF                   ((uint8_t) 0x00)
    #define CONFIG1_SCBCS_1_5uA                 ((uint8_t) 0x40)
    #define CONFIG1_SCBCS_24uA                  ((uint8_t) 0xC0)

    /* DRATE field values (fixed-channel DRs shown) */
    #define CONFIG1_DRATE_1953SPS               ((uint8_t) 0x00)
    #define CONFIG1_DRATE_7813SPS               ((uint8_t) 0x01)
    #define CONFIG1_DRATE_31250SPS              ((uint8_t) 0x02)
    #define CONFIG1_DRATE_125000SPS             ((uint8_t) 0x03)



/* Register 0x02 (MUXSCH) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |               AINP[3:0]               |               AINN[3:0]               |
* ---------------------------------------------------------------------------------
*/

    /** MUXSCH register address */
    #define REG_ADDR_MUXSCH                     ((uint8_t) 0x02)

    /** MUXSCH default (reset) value */
    #define MUXSCH_DEFAULT                      ((uint8_t) 0x00)

    /* MUXSCH register field masks */
    #define MUXSCH_AINP_MASK                    ((uint8_t) 0xF0)
    #define MUXSCH_AINN_MASK                    ((uint8_t) 0x0F)

    /* AINP field values */
    #define MUXSCH_AINP_AIN0                    ((uint8_t) 0x00)
    #define MUXSCH_AINP_AIN1                    ((uint8_t) 0x10)
    #define MUXSCH_AINP_AIN2                    ((uint8_t) 0x20)
    #define MUXSCH_AINP_AIN3                    ((uint8_t) 0x30)
    #define MUXSCH_AINP_AIN4                    ((uint8_t) 0x40)
    #define MUXSCH_AINP_AIN5                    ((uint8_t) 0x50)
    #define MUXSCH_AINP_AIN6                    ((uint8_t) 0x60)
    #define MUXSCH_AINP_AIN7                    ((uint8_t) 0x70)
    #define MUXSCH_AINP_AIN8                    ((uint8_t) 0x80)
    #define MUXSCH_AINP_AIN9                    ((uint8_t) 0x90)
    #define MUXSCH_AINP_AIN10                   ((uint8_t) 0xA0)
    #define MUXSCH_AINP_AIN11                   ((uint8_t) 0xB0)
    #define MUXSCH_AINP_AIN12                   ((uint8_t) 0xC0)
    #define MUXSCH_AINP_AIN13                   ((uint8_t) 0xD0)
    #define MUXSCH_AINP_AIN14                   ((uint8_t) 0xE0)
    #define MUXSCH_AINP_AIN15                   ((uint8_t) 0xF0)

    /* AINN field values */
    #define MUXSCH_AINN_AIN0                    ((uint8_t) 0x00)
    #define MUXSCH_AINN_AIN1                    ((uint8_t) 0x01)
    #define MUXSCH_AINN_AIN2                    ((uint8_t) 0x02)
    #define MUXSCH_AINN_AIN3                    ((uint8_t) 0x03)
    #define MUXSCH_AINN_AIN4                    ((uint8_t) 0x04)
    #define MUXSCH_AINN_AIN5                    ((uint8_t) 0x05)
    #define MUXSCH_AINN_AIN6                    ((uint8_t) 0x06)
    #define MUXSCH_AINN_AIN7                    ((uint8_t) 0x07)
    #define MUXSCH_AINN_AIN8                    ((uint8_t) 0x08)
    #define MUXSCH_AINN_AIN9                    ((uint8_t) 0x09)
    #define MUXSCH_AINN_AIN10                   ((uint8_t) 0x0A)
    #define MUXSCH_AINN_AIN11                   ((uint8_t) 0x0B)
    #define MUXSCH_AINN_AIN12                   ((uint8_t) 0x0C)
    #define MUXSCH_AINN_AIN13                   ((uint8_t) 0x0D)
    #define MUXSCH_AINN_AIN14                   ((uint8_t) 0x0E)
    #define MUXSCH_AINN_AIN15                   ((uint8_t) 0x0F)



/* Register 0x03 (MUXDIF) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |  DIFF7  |  DIFF6  |  DIFF5  |  DIFF4  |  DIFF3  |  DIFF2  |  DIFF1  |  DIFF0  |
* ---------------------------------------------------------------------------------
*/

    /** MUXDIF register address */
    #define REG_ADDR_MUXDIF                     ((uint8_t) 0x03)

    /** MUXDIF default (reset) value */
    #define MUXDIF_DEFAULT                      ((uint8_t) 0x00)

    /* MUXDIF register field masks */
    #define MUXDIF_DIFF7_ENABLE                 ((uint8_t) 0x80)
    #define MUXDIF_DIFF6_ENABLE                 ((uint8_t) 0x40)
    #define MUXDIF_DIFF5_ENABLE                 ((uint8_t) 0x20)
    #define MUXDIF_DIFF4_ENABLE                 ((uint8_t) 0x10)
    #define MUXDIF_DIFF3_ENABLE                 ((uint8_t) 0x08)
    #define MUXDIF_DIFF2_ENABLE                 ((uint8_t) 0x04)
    #define MUXDIF_DIFF1_ENABLE                 ((uint8_t) 0x02)
    #define MUXDIF_DIFF0_ENABLE                 ((uint8_t) 0x01)



/* Register 0x04 (MUXSG0) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |   AIN7  |   AIN6  |   AIN5  |   AIN4  |   AIN3  |   AIN2  |   AIN1  |   AIN0  |
* ---------------------------------------------------------------------------------
*/

    /** MUXSG0 register address */
    #define REG_ADDR_MUXSG0                     ((uint8_t) 0x04)

    /** MUXSG0 default (reset) value */
    #define MUXSG0_DEFAULT                      ((uint8_t) 0xFF)

    /* MUXSG0 register field masks */
    #define MUXSG0_AIN7_ENABLE                  ((uint8_t) 0x80)
    #define MUXSG0_AIN6_ENABLE                  ((uint8_t) 0x40)
    #define MUXSG0_AIN5_ENABLE                  ((uint8_t) 0x20)
    #define MUXSG0_AIN4_ENABLE                  ((uint8_t) 0x10)
    #define MUXSG0_AIN3_ENABLE                  ((uint8_t) 0x08)
    #define MUXSG0_AIN2_ENABLE                  ((uint8_t) 0x04)
    #define MUXSG0_AIN1_ENABLE                  ((uint8_t) 0x02)
    #define MUXSG0_AIN0_ENABLE                  ((uint8_t) 0x01)



/* Register 0x05 (MUXSG1) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |  AIN15  |  AIN14  |  AIN13  |  AIN12  |  AIN11  |  AIN10  |   AIN9  |   AIN8  |
* ---------------------------------------------------------------------------------
*/

    /** MUXSG1 register address */
    #define REG_ADDR_MUXSG1                     ((uint8_t) 0x05)

    /** MUXSG1 default (reset) value */
    #define MUXSG1_DEFAULT                      ((uint8_t) 0xFF)

    /* MUXSG1 register field masks */
    #define MUXSG1_AIN15_ENABLE                 ((uint8_t) 0x80)
    #define MUXSG1_AIN14_ENABLE                 ((uint8_t) 0x40)
    #define MUXSG1_AIN13_ENABLE                 ((uint8_t) 0x20)
    #define MUXSG1_AIN12_ENABLE                 ((uint8_t) 0x10)
    #define MUXSG1_AIN11_ENABLE                 ((uint8_t) 0x08)
    #define MUXSG1_AIN10_ENABLE                 ((uint8_t) 0x04)
    #define MUXSG1_AIN9_ENABLE                  ((uint8_t) 0x02)
    #define MUXSG1_AIN8_ENABLE                  ((uint8_t) 0x01)



/* Register 0x06 (SYSRED) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |    0    |    0    |   REF   |   GAIN  |   TEMP  |   VCC   |     0   |  OFFSET |
* ---------------------------------------------------------------------------------
*/

    /** SYSRED register address */
    #define REG_ADDR_SYSRED                     ((uint8_t) 0x06)

    /** SYSRED default (reset) value */
    #define SYSRED_DEFAULT                      ((uint8_t) 0x00)

    /* SYSRED register field masks */
    #define SYSRED_REF_ENABLE                   ((uint8_t) 0x20)
    #define SYSRED_GAIN_ENABLE                  ((uint8_t) 0x10)
    #define SYSRED_TEMP_ENABLE                  ((uint8_t) 0x08)
    #define SYSRED_VCC_ENABLE                   ((uint8_t) 0x04)
    #define SYSRED_OFFSET_ENABLE                ((uint8_t) 0x01)



/* Register 0x07 (GPIOC) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |                                    CIO[7:0]                                   |
* ---------------------------------------------------------------------------------
*/

    /** GPIOC register address */
    #define REG_ADDR_GPIOC                      ((uint8_t) 0x07)

    /** GPIOC default (reset) value */
    #define GPIOC_DEFAULT                       ((uint8_t) 0xFF)

    /* GPIOC register field masks */
    #define GPIOC_GPIO7_INPUT                   ((uint8_t) 0x80)
    #define GPIOC_GPIO6_INPUT                   ((uint8_t) 0x40)
    #define GPIOC_GPIO5_INPUT                   ((uint8_t) 0x20)
    #define GPIOC_GPIO4_INPUT                   ((uint8_t) 0x10)
    #define GPIOC_GPIO3_INPUT                   ((uint8_t) 0x08)
    #define GPIOC_GPIO2_INPUT                   ((uint8_t) 0x04)
    #define GPIOC_GPIO1_INPUT                   ((uint8_t) 0x02)
    #define GPIOC_GPIO0_INPUT                   ((uint8_t) 0x01)



/* Register 0x08 (GPIOD) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |                                    DIO[7:0]                                   |
* ---------------------------------------------------------------------------------
*/

    /** GPIOD register address */
    #define REG_ADDR_GPIOD                      ((uint8_t) 0x08)

    /** GPIOD default (reset) value */
    #define GPIOD_DEFAULT                       ((uint8_t) 0x00)

    /* GPIOD register field masks */
    #define GPIOD_GPIO7_HIGH                    ((uint8_t) 0x80)
    #define GPIOD_GPIO6_HIGH                    ((uint8_t) 0x40)
    #define GPIOD_GPIO5_HIGH                    ((uint8_t) 0x20)
    #define GPIOD_GPIO4_HIGH                    ((uint8_t) 0x10)
    #define GPIOD_GPIO3_HIGH                    ((uint8_t) 0x08)
    #define GPIOD_GPIO2_HIGH                    ((uint8_t) 0x04)
    #define GPIOD_GPIO1_HIGH                    ((uint8_t) 0x02)
    #define GPIOD_GPIO0_HIGH                    ((uint8_t) 0x01)



/* Register 0x09 (ID) definition
* ---------------------------------------------------------------------------------
* |  Bit 7  |  Bit 6  |  Bit 5  |  Bit 4  |  Bit 3  |  Bit 2  |  Bit 1  |  Bit 0  |
* ---------------------------------------------------------------------------------
* |                                    ID[7:0]                                    |
* ---------------------------------------------------------------------------------
*/

    /** ID register address */
    #define REG_ADDR_ID                         ((uint8_t) 0x09)

    /* ID register field masks */
    #define ID_ID4_MASK                         ((uint8_t) 0x10)

    /* ID4 field values */
    #define ID_ID4_ADS1258                      ((uint8_t) 0x00)
    #define ID_ID4_ADS1158                      ((uint8_t) 0x10)



//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

void    adcStartupRoutine(void);
int32_t readData(uint8_t status[], uint8_t data[], readMode mode);
uint8_t readSingleRegister(uint8_t address);
void    readMultipleRegisters(uint8_t startAddress, uint8_t count);
void    sendCommand(uint8_t op_code);
void    startConversions(void);
void    writeSingleRegister(uint8_t address, uint8_t data);
void    writeMultipleRegisters(uint8_t startAddress, uint8_t count, const uint8_t regData[]);

// Internal variable getters
uint8_t getRegisterValue(uint8_t address);

// Internal variable setters
void    restoreRegisterDefaults(void);



//*****************************************************************************
//
// Macros
//
//*****************************************************************************

/** Register bit checking macros...
 *  Return true if register bit is set (since last read or write).
 */
#define IS_MUXMOD_SET       ((bool) (getRegisterValue(REG_ADDR_CONFIG0) & CONFIG0_MUXMOD_MASK))
#define IS_STAT_SET         ((bool) (getRegisterValue(REG_ADDR_CONFIG0) & CONFIG0_STAT_MASK))



#endif /* ADS1258_H_ */
