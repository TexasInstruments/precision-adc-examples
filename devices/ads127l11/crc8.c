/**
 * \copyright Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
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

#include "crc8.h"


//*****************************************************************************
//
// Internal variables and function prototypes
//
//*****************************************************************************
static uint8_t poly;
static bool initialized = false;
#ifdef CRC8_LOOKUP
    static uint8_t crcLookupTable[256];
#endif



//*****************************************************************************
//
// Functions
//
//*****************************************************************************


//*****************************************************************************
//
//! \brief Calculates the CRC for the selected CRC polynomial.
//!
//! \fn static uint8_t calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint8_t initialValue)
//! \param dataBytes[] pointer to first element in the data byte array
//! \param numberBytes number of bytes to be used in CRC calculation
//! \param initialValue the seed value (or partial crc calculation), use CRC_INITIAL_SEED when beginning a new CRC computation
//!
//! NOTE: This calculation is not optimized for speed.
//!
//! \return Resulting CRC value.
//
//*****************************************************************************
static uint8_t calculateCRC8(const uint8_t dataBytes[], uint8_t numberBytes, uint8_t initialValue)
{
    int         bitIndex, byteIndex;
    bool        dataMSb;                        /* Most significant bit of data byte */
    bool        crcMSb;                         /* Most significant bit of crc byte  */

    // Initial value of crc register
    uint8_t crc = initialValue;

    // Loop through all bytes in the dataBytes[] array
    for (byteIndex = 0; byteIndex < numberBytes; byteIndex++)
    {
        // Point to most significant bit
        bitIndex = 0x80u;

        // Loop through all bits in the current byte
        while (bitIndex > 0)
        {
            // Check MSB's of data and crc
            dataMSb = (bool) (dataBytes[byteIndex] & bitIndex);
            crcMSb  = (bool) (crc & 0x80u);

            // Update crc register
            crc <<= 1;
            if (dataMSb ^ crcMSb) { crc ^= poly; }

            // Shift MSb pointer to the next data bit
            bitIndex >>= 1;
        }
    }

    return crc;
}


#ifdef CRC8_LOOKUP
//*****************************************************************************
//
//! \brief Performs CRC lookup operation
//!
//! \fn static uint8_t lookupCRC8(const uint8_t dataBytes[], uint8_t numberBytes, uint8_t initialValue)
//! \param dataBytes pointer to the data array
//! \param numberBytes number of bytes in array to process
//! \param initialValue CRC initial seed value or partial result of previous CRC calculation
//!
//! \return Resulting CRC value.
//
//*****************************************************************************
static uint8_t lookupCRC8(const uint8_t dataBytes[], uint8_t numberBytes, uint8_t initialValue)
{
    uint8_t crc = initialValue;
    unsigned int i;
    for (i = 0; i < numberBytes; ++i)
    {
        crc = crcLookupTable[crc ^ dataBytes[i]];      
    }
    return crc;
}


//*****************************************************************************
//
//! \brief Creates lookup table
//!
//! \fn static void initLUT(void)
//!
//! \return None.
//
//*****************************************************************************
static void initLUT(void)
{
    uint8_t value;
    unsigned int i;
    for (i = 0; i < 256; ++i)
    {
        value = (uint8_t) i;
        crcLookupTable[i] = calculateCRC8(&value, 1, 0x00);
    }
}
#endif


//*****************************************************************************
//
//! \brief Initializes CRC module and creates lookup table (if using lookup method)
//!
//! \fn void initCRC8(uint8_t polynomial)
//! \param polynomial CRC polynomial value
//!
//! \return None.
//
//*****************************************************************************
void initCRC8(uint8_t polynomial)
{
    poly = polynomial;  // calculateCRC8() will recall this value later

#ifdef CRC8_LOOKUP
    initLUT();
#endif

    initialized = true;
}


//*****************************************************************************
//
//! \brief Performs CRC lookup or calculation
//!
//! \fn uint8_t getCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint8_t initialValue)
//! \param dataBytes pointer to the data array
//! \param numberBytes number of bytes in array to process
//! \param initialValue CRC initial seed value or partial result of previous CRC calculation
//!
//! To calculate the CRC of 3-bytes of data:
//! uint8_t crc = getCRC(data, 3, CRC8_INITIAL_SEED);
//!
//! To validate a 4-byte message with a CRC byte included:
//! bool error = (bool) getCRC(data, 4, CRC8_INITIAL_SEED);
//!
//! \return Resulting CRC value.
//
//*****************************************************************************
uint8_t getCRC8(const uint8_t dataBytes[], uint8_t numberBytes, uint8_t initialValue)
{
    assert(initialized);    // Check that user ran initCRC8() and provided a polynomial

#ifdef CRC8_LOOKUP
    return lookupCRC8(dataBytes, numberBytes, initialValue);
#else
    return calculateCRC8(dataBytes, numberBytes, initialValue);
#endif
}