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

#include "pca9535.h"


//****************************************************************************
//
// Internal variables
//
//****************************************************************************

/* Internal register map array (to recall current configuration) */
static uint8_t registerMap[PCA9535_NUM_REGISTERS];


//****************************************************************************
//
// Functions
//
//****************************************************************************




//*****************************************************************************
//
//! Reads the contents of a single register at the specified address.
//!
//! \fn uint8_t readSingleRegister(uint8_t address)
//!
//! \param address is the 8-bit address of the register to read.
//!
//! \return Returns the 8-bit register read result.
//
//*****************************************************************************
uint8_t PCA9535_readRegister(const uint8_t address)
{
    // Check that the register address is in range
    assert(address < PCA9535_NUM_REGISTERS);

    // Initialize RX and TX buffers
    uint8_t dataTx[1] = { address & PCA9535_CONTROL_BIT_MASK };
    uint8_t dataRx[1] = { 0 };   // Placeholder for received data

    i2cWrite(PCA9535_I2C_ADDRESS, dataTx, 1);
    i2cRead(PCA9535_I2C_ADDRESS, dataRx, 1);

    // Update internal register array and return register value
    const uint8_t registerValue = registerMap[address] = dataRx[0];
    return registerValue;
}

void PCA9535_writeRegister(const uint8_t address, const uint8_t value)
{
    // Check that the register address is in range
    assert(address < PCA9535_NUM_REGISTERS);

    // Initialize RX and TX buffers
    uint8_t dataTx[2] = { (address & PCA9535_CONTROL_BIT_MASK), value };
    i2cWrite(PCA9535_I2C_ADDRESS, dataTx, 2);

    // Update internal register array
    registerMap[address] = value;
}

