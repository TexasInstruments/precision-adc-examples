/**
 * @file crc.c
 *
 * @brief Example of calculating and verifying CRC
 *
 * @copyright Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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


/******************************************************************************
 *
 * Internal variables and function prototypes
 *
 ******************************************************************************/


#ifdef CRC_LOOKUP
    static bool initialized = false;
    static CRCWORD crcLookupTable[256];
    static void initTable();
    static CRCWORD lookupCRC(const uint8_t dataBytes[], uint8_t numberBytes, CRCWORD initialValue);
#else
    static CRCWORD calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, CRCWORD initialValue);

#endif


/******************************************************************************
 *
 * Functions
 *
 ******************************************************************************/

/**
 * @brief initCRC()
 * Initializes CRC module and creates lookup table (if using lookup method).
 *
 * @return None.
 *
 */
void initCRC()
{
#ifdef CRC_LOOKUP
    initTable();
    initialized = true;
#endif
}

/**
 * @brief getCRC()
 * Performs CRC lookup or calculation
 *
 * @param[in] dataBytes Is a pointer to the data array.
 * @param[in] numberBytes Is the number of bytes in the array to process.
 * @param[in] initialValue Is the CRC initial seed value or partial result of previous CRC calculation.
 *
 * Note: To calculate the CRC of a 3-byte message call:
 * CRCWORD crc = getCRC(data, 3, CRC_INITIAL_SEED);
 *
 * @return CRCWORD Value from calculation.
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
/**
 * @brief initTable()
 * Creates a lookup table.
 *
 * @return None.
 */
static void initTable()
{
    CRCWORD value;
    uint32_t i, bit;
    for (i = 0; i < 256; ++i)
    {
        value = (CRCWORD) i << (WIDTH - 8);

        for(bit = 8; bit > 0; bit--)                       // For each bit in the remainder
        {
            if(value & (1 << (WIDTH - 1)))
                value = (value << 1) ^ CRC_POLYNOMIAL;     // If the top bit is set, left shift
                                                           // the remainder and XOR it with the
            else                                           // divisor and store the result in
                                                           // the remainder
                value = (value << 1);                      // If the top bit is clear, left shift
                                                           // the remainder
         }
        crcLookupTable[i] = value;
    }
}

/**
 * @brief lookupCRC()
 * Performs CRC lookup operation.
 *
 * @param[in] dataBytes Is the pointer to the data array.
 * @param[in] numberBytes Is the number of bytes in the array to process.
 * @param[in] initialValue Is the CRC initial seed value or partial result of previous CRC calculation.
 *
 * @return CRCWORD Of the calculated value.
 */
static CRCWORD lookupCRC(const uint8_t dataBytes[], uint8_t numberBytes, CRCWORD initialValue)
{
    CRCWORD crc = initialValue;
    uint32_t i, byte;
    for (i = 0; i < numberBytes; ++i)
    {
        byte = dataBytes[i] ^ (crc >> (WIDTH -8));
        crc = crcLookupTable[byte] ^ (crc << 8);
    }
    return crc;
}
#else

/**
 * @brief calculateCRC()
 * Calculates the CRC for the selected CRC polynomial.
 *
 * @param[in] dataByte Is the pointer to first element in the data byte array.
 * @param[in] numberBytes number of bytes to be used in CRC calculation.
 * @param[in] initialValue Is the CRC initial seed value or partial result of previous CRC calculation.
 * Use CRC_INITIAL_SEED when beginning a new CRC computation.
 *
 * NOTE: This calculation is not optimized for speed.
 *
 * @return CRCWORD Of the calculated value.
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
        // Point to MSb in byte
        bitIndex = 0x80u;

        // Loop through all bits in the current byte
        while (bitIndex > 0)
        {
            // Check MSB's of data and crc
            dataMSb = (bool) (dataBytes[byteIndex] & bitIndex);
#ifdef CRC8
            crcMSb  = (bool) (crc & 0x80u);
#else
            crcMSb  = (bool) (crc & 0x8000u);
#endif
            // Update crc register
            crc <<= 1;
            if (dataMSb ^ crcMSb) { crc ^= CRC_POLYNOMIAL; }

            // Shift MSb pointer to the next data bit
            bitIndex >>= 1;
        }
    }

    return crc;
}
#endif
