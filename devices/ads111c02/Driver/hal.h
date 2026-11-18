/*
 * \copyright Copyright (C) 2026 Texas Instruments Incorporated - http://www.ti.com/
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
 */

#include "ti_msp_dl_config.h"
#include "ti/driverlib/m0p/dl_core.h"


#define HIGH ((bool)true)
#define LOW ((bool)false)

//I2C Address
#define I2C_ADDRESS 0x40

// Global Variables 
extern uint8_t g_i2c_addr;

// Function Prototypes

/**
 * @brief Drives the SPI chip select (CS_n) line.
 *
 * @param[in] state HIGH to deassert chip select, LOW to assert it.
 */
void    setCS(bool state);

/**
 * @brief Writes bytes to the I2C target at g_i2c_addr.
 *
 * Blocks until the controller transfer has completed.
 *
 * @param[in] dataTx  Bytes to transmit.
 * @param[in] wLength Number of bytes to transmit.
 */
void    I2C_Write (uint8_t dataTx[], uint8_t wLength);

/**
 * @brief Reads bytes from the I2C target at g_i2c_addr.
 *
 * Blocks until the controller transfer has completed.
 *
 * @param[out] dataRx  Buffer that receives the bytes read.
 * @param[in]  rLength Number of bytes to read.
 */
void    I2C_Read(uint8_t dataRx[], uint8_t rLength);

/**
 * @brief Transmits and receives a frame over SPI.
 *
 * Asserts chip select, shifts out bufferLength bytes from dataTx while
 * capturing the same number of bytes into dataRx, then deasserts chip select.
 *
 * @param[in]  dataTx       Bytes to transmit.
 * @param[out] dataRx       Buffer that receives the bytes shifted in.
 * @param[in]  bufferLength Number of bytes in the frame.
 */
void    spiSendReceiveArrays(uint8_t dataTx[], uint8_t dataRx[], uint8_t bufferLength);

/**
 * @brief Blocking delay in milliseconds (cycle-count based).
 *
 * @param[in] delay_time_ms Delay in milliseconds.
 */
void    delay_ms(const uint32_t delay_time_ms);

/**
 * @brief Blocking delay in microseconds (cycle-count based).
 *
 * @param[in] delay_time_us Delay in microseconds.
 */
void    delay_us(const uint32_t delay_time_us);

/**
 * @brief Waits for nDRDY to go low, which signals a new conversion result.
 *
 * Asserts chip select, then polls the nDRDY pin until it goes low or the
 * timeout expires. The timeout is a polling-iteration count scaled from
 * timeout_ms, so it is approximate rather than a precise time.
 *
 * @param[in] timeout_ms Approximate maximum time to wait in milliseconds.
 *
 * @return true if nDRDY was detected before the timeout, false otherwise.
 */
bool    waitForDRDY(uint32_t timeout_ms);