/**
 * @file crc.h
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

// APP NOTE: Communication Methods for Data Integrity Using Delta-Sigma Data Converters
// URL: https://www.ti.com/lit/an/sbaa106/sbaa106.pdf

#ifndef CRC_H_
#define CRC_H_

// Standard libraries
#include <stdbool.h>
#include <stdint.h>


/*****************************************************************************
 *
 * Constants
 *
 ****************************************************************************/


//
// Select CRC calculation mode...
//
//#define CRC_LOOKUP
#define CRC_CALCULATION

//
// Select CRC word length...
//
//#define CRC8            // CRC word is 8-bits wide
#define CRC16         // CRC word is 16-bits wide

// Adjust "CRCWORD" to CRC word length
#ifdef CRC8
    #define CRCWORD                             uint8_t
#endif
#ifdef CRC16
    #define CRCWORD                             uint16_t
#endif

#define WIDTH (8 * sizeof(CRCWORD))

//
// Initial seed value for CRC calculation
//
#define CRC_INITIAL_SEED                        ((CRCWORD) 0xFFFF)

//
// Initial seed value for CRC calculation
//
#define CRC_POLYNOMIAL                          ((CRCWORD) 0x1021)


/*****************************************************************************
 *
 * Function Prototypes
 *
 ****************************************************************************/
void initCRC();
CRCWORD getCRC(const uint8_t dataBytes[], uint8_t numberBytes, CRCWORD initialValue);


#endif /* CRC_H_ */
