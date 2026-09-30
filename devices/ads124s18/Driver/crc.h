/**
 * @file crc.h
 * @brief CRC8 calculation and validation for data integrity checking.
 *
 * Implements CRC8 checksum computation for validating data integrity in
 * SPI communication and register memory protection. Supports both lookup
 * table and calculation modes for flexibility vs. performance tradeoffs.
 *
 * CRC Algorithm:
 * - Type: CRC8 (8-bit polynomial)
 * - Polynomial: 0x07
 * - Initial seed: 0xFF
 * - Output XOR: None (raw CRC value returned)
 *
 * Usage modes (select one at compile time):
 * 1. CRC_LOOKUP: Fast lookup table mode (256 bytes RAM, ~5µs per byte)
 * 2. CRC_CALCULATION: Slow bit-by-bit mode (requires initCRC() first)
 *
 * Typical applications in ADS124S18 driver:
 * - SPI_CRC: CRC on each SPI transaction data (1-3 bytes of payload)
 * - REGMAP_CRC: CRC over all register memory pages (detect corruption)
 *
 * Reference Implementation:
 * APP NOTE: Communication Methods for Data Integrity Using Delta-Sigma Data Converters
 * URL: https://www.ti.com/lit/an/sbaa106/sbaa106.pdf
 *
 * \copyright Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef CRC_H_
#define CRC_H_

// Standard libraries
#include <stdbool.h>
#include <stdint.h>


//****************************************************************************
//
// Constants
//
//****************************************************************************

//*****************************************************************************
//
// Configuration Constants
//
//*****************************************************************************

//
// CRC Calculation Mode Selection (choose ONE):
// CRC_LOOKUP    - Uses 256-byte lookup table for fast calculation (~1-5µs per byte)
// CRC_CALCULATION - Uses bit-by-bit computation (slower, minimal RAM)
//
#define CRC_LOOKUP                      //!< Use lookup table mode for performance
//#define CRC_CALCULATION                // Use calculation mode for minimal RAM

//
// CRC Word Length Selection (choose ONE):
// CRC8         - 8-bit CRC (typical for SPI validation)
// CRC16        - 16-bit CRC (higher error detection capability)
//
#define CRC8                            //!< 8-bit CRC (used by ADS124S18)
//#define CRC16                          // 16-bit CRC

// Adjust "CRCWORD" to match selected word length
#ifdef CRC8
    #define CRCWORD                     uint8_t     //!< 8-bit CRC type
#elif defined(CRC16)
    #define CRCWORD                     uint16_t    //!< 16-bit CRC type
#endif

//
// CRC Algorithm Parameters
//

//!< Starting value for CRC computation (resets accumulator for each frame)
#define CRC_INITIAL_SEED                ((CRCWORD) 0xFF)

//!< Polynomial for CRC8 (0x07 = x^7 + x^2 + x^0)
#define CRC_POLYNOMIAL                  ((CRCWORD) 0x07)


//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

/**
 * @brief Initialize CRC engine (required for CRC_CALCULATION mode).
 *
 * For CRC_LOOKUP mode: Generates the 256-entry lookup table (one-time cost)
 * For CRC_CALCULATION mode: Performs any required setup
 *
 * Call this once during initialization before any getCRC() calls.
 * Required by ADS124S18 driver during resetDevice().
 *
 * @note No return value - performs setup internally
 */
void initCRC(void);

/**
 * @brief Compute CRC8 over a data buffer.
 * @param dataBytes[] Pointer to data buffer to checksum
 * @param numberBytes Number of bytes in buffer
 * @param initialValue Starting CRC value (typically CRC_INITIAL_SEED)
 * @return 8-bit CRC checksum
 *
 * Calculates CRC8 using polynomial 0x07 over the provided data.
 *
 * Typical usage patterns:
 *
 * 1. SPI CRC validation (1-3 byte payload):
 *    uint8_t received_crc = spiBuffer[3];
 *    uint8_t computed_crc = getCRC(spiBuffer, 3, CRC_INITIAL_SEED);
 *    bool valid = (received_crc == computed_crc);
 *
 * 2. Register map CRC (sequential page blocks):
 *    CRCWORD crc = CRC_INITIAL_SEED;
 *    crc = getCRC(page_data, 21, crc);     // Accumulate across pages
 *    crc = getCRC(next_page, 16, crc);     // Continue with new seed
 *
 * Performance:
 * - CRC_LOOKUP mode: ~1-5µs per byte (depends on SRAM access speed)
 * - CRC_CALCULATION mode: ~20-50µs per byte (instruction heavy)
 *
 * @note For CRC_LOOKUP mode, call initCRC() first to build lookup table.
 */
CRCWORD getCRC(const uint8_t dataBytes[], uint8_t numberBytes, CRCWORD initialValue);


#endif /* CRC_H_ */
