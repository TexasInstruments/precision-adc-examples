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

#ifndef PCA9535_H_
#define PCA9535_H_

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

#define PCA9535_I2C_ADDRESS         (0x20)
#define PCA9535_NUM_REGISTERS       (8)
#define PCA9535_CONTROL_BIT_MASK    (0x07)




//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

uint8_t PCA9535_readRegister(const uint8_t address);
void PCA9535_writeRegister(const uint8_t address, const uint8_t value);



//*****************************************************************************
//
// Register macros
//
//*****************************************************************************

/* Returns true of digital filter is bypassed */
//#define MOD_MODE        ((bool) ((getRegisterValue(REG_ADDR_CONFIG0) & CONFIG0_FILTR_MASK) == CONFIG0_FILTR_MODMODE)


//*****************************************************************************
//
// Register definitions
//
//*****************************************************************************




#endif /* PCA9535_H_ */
