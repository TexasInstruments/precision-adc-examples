/**
 * @file crc.c
 *
 * @brief Library of functions for CRC implementations
 *
 * \copyright Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
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

#include "crc.h"


//*****************************************************************************
//
// Internal variables and function prototypes
//
//*****************************************************************************

static CRCWORD calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, CRCWORD initialValue);

#ifdef CRC_LOOKUP
    static bool initialized = false;
    static CRCWORD crcLookupTable[256];
    static void initTable(void);
    static CRCWORD lookupCRC(const uint8_t dataBytes[], uint8_t numberBytes, CRCWORD initialValue);
#endif


//*****************************************************************************
//
// Functions
//
//*****************************************************************************

/************************************************************************************//**
 *
 * @brief initCRC()
 *             Initializes CRC module and creates lookup table (if using lookup method)
 *
 *
 * @return     None
 */
void initCRC(void)
{
#ifdef CRC_LOOKUP
    initTable();
    initialized = true;
#endif
}

/************************************************************************************//**
 *
 * @brief getCRC()
 *             Performs CRC lookup or calculation
 *
 * @param[in]   dataBytes    pointer to the data array
 * @param[in]   numberBytes  number of bytes in array to process
 * @param[in]   initialValue CRC initial seed value or partial result of previous CRC calculation
 *
 * @return     CRCWORD       of the calculation
 *
 * @code
 *      // To calculate the CRC of a 3-byte message call:
 *      CRCWORD crc = getCRC(data, 3, CRC_INITIAL_SEED);
 *
 *      // To test a 4-byte message with a CRC byte call:
 *      bool error = (bool) getCRC(data, 4, CRC_INITIAL_SEED);
 *
 * @endcode
 */
CRCWORD getCRC(const uint8_t dataBytes[], uint8_t numberBytes, CRCWORD initialValue)
{
#ifdef CRC_CALCULATION
    return calculateCRC(dataBytes, numberBytes, initialValue);
#endif

#ifdef CRC_LOOKUP
    if (!initialized) { initTable(); }
    return lookupCRC(dataBytes, numberBytes, initialValue);
#endif
}


#ifdef CRC_LOOKUP
/************************************************************************************//**
 *
 * @brief initTable()
 *             Creates lookup table in memory using byte wide computation in a 256 element array
 *
 * @return      None
 *
 */
static void initTable(void)
{
    uint8_t value;
    unsigned int i;
    for (i = 0; i < 256; ++i)
    {
        value = (uint8_t) i;
        crcLookupTable[i] = calculateCRC(&value, 1, 0x00);
    }
}

/************************************************************************************//**
 *
 * @brief lookupCRC(()
 *             Performs CRC lookup operation in byte increments
 *
 * @param[in]   dataBytes    pointer to the data array (little endian)
 * @param[in]   numberBytes  number of bytes in array to process
 * @param[in]   initialValue CRC initial seed value or partial result of previous CRC calculation
 *
 * @return     CRCWORD       of the calculation
 *
 */
static CRCWORD lookupCRC(const uint8_t dataBytes[], uint8_t numberBytes, CRCWORD initialValue)
{
    CRCWORD crc = initialValue;
    unsigned int i;
    for (i = 0; i < numberBytes; ++i)
    {

        crc = crcLookupTable[crc ^ dataBytes[i]];

    }
    return crc;
}
#endif

/************************************************************************************//**
 *
 * @brief calculateCRC()
 *             Calculates the CRC for the selected CRC polynomial
 *
 * @param[in]   dataBytes    pointer to first element in the data byte array
 * @param[in]   numberBytes  number of bytes to be used in CRC calculation
 * @param[in]   initialValue the seed value (or partial crc calculation), use CRC_INITIAL_SEED when beginning a new CRC computation
 *
 * @return     CRCWORD       of the calculation
 *
 */
static CRCWORD calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, CRCWORD initialValue)
{
    int         bitIndex, byteIndex;
    bool        dataMSb;                        /* Most significant bit of data byte */
    bool        crcMSb;                         /* Most significant bit of crc byte  */

    // Initial value of crc register
    CRCWORD crc = initialValue;

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
            if (dataMSb ^ crcMSb) { crc ^= CRC_POLYNOMIAL; }

            // Shift MSb pointer to the next data bit
            bitIndex >>= 1;
        }
    }

    return crc;
}

