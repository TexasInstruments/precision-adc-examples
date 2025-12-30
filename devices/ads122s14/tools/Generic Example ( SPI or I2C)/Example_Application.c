/**
 * @file Example_Application.c
 *
 * @brief This file contains all basic communication and device setup for the ADS122y14 device family.
 * @warning This software utilizes TI Drivers
 *
 * @copyright Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
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


//-----------------------------EXAMPLE APPLICATION------------------------
/*
This program will show basic usage of the ADS122y14 as an example application.
In this program we will use the basic example code functions available from
<ADS122S14.c> and <hal.c> in order to initialize the part, set a configuration,
measure some internal signals, and then enter a low power state.

Use this simple example to explore the functions provided in the example code,
as well as to learn how to operate the ADC to make precision measurements.

** This example will use the SPI variant of the device.

*/

#include "evm/ads122s1x.h"
#include "evm/hal.h"
#include <stdbool.h>
#include "evm/crc.h"


uint32_t AdcReading[32] = {0};
uint32_t numSamples;
uint8_t zeros[4] = {0};
bool DRDY = false;

void exampleApplication(void)
{
    initCRC();
    resetDevice();      // all 3 of these functions are defined in ADS122S14.c
    initAdcConfig();
    readSingleRegister(DEVICE_CFG_ADDRESS);
    startAdcConversion();

    while(numSamples < 32)
    {
        // wait for data sample to be ready
        if(DRDY)
        {
            AdcReading[numSamples] = readDataDirect();
            numSamples++;
            DRDY = false;
        }else{

            delay_us(100);
            DRDY = STATUS_MSB_DRDY_MASK & readSingleRegister(STATUS_MSB_ADDRESS);
        }

    }

// next we will read a register with CRC

  writeSingleRegister(DIGITAL_CFG_ADDRESS, DIGITAL_CFG_SPI_CRC_EN_ENABLED);


    //initialize variables used for the HAL SPI transaction
    uint8_t dataTx[4] = {0};
    uint8_t dataRx[4] = {0};
    uint8_t crcBuffer[4] = {0};
    uint8_t bufferLength = 4;

    // read register 0x05
    dataTx[0] = 0x00;
    dataTx[1] = 0x45;               // '01'+address[5:0] = command 0x45
    dataTx[2] = 0x00;

    crcBuffer[0] = dataTx[1];
    crcBuffer[1] = dataTx[2];
    dataTx[3] = getCRC(crcBuffer, 2, CRC_INITIAL_SEED);    // crc_in value for transaction




    if(dataRx[3] != getCRC(dataRx,3, CRC_INITIAL_SEED))
            {json_console("CRC ERROR");}



    // write register to now dis-able CRC
    dataTx[0] = 0x8A;
    dataTx[1] = 0x00;
    dataTx[2] = getCRC(dataTx, 2, CRC_INITIAL_SEED);
    bufferLength = 3;


}




// show SS mode
// show CRC example
// Show STATUS byte example
// polling for data with DRDY pin

