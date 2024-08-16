/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
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
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>

//#include <ti/drivers/Power.h>
//#include <ti/drivers/power/PowerMSP432E4.h>

/* Example/Board Header files */
#include "Board.h"


#define MSGSIZE  (15)

// SPI
static SPI_Handle       spi;
static SPI_Params       spiParams;
static SPI_Transaction  spiTransaction;
static uint8_t          transmitBuffer[MSGSIZE] = {0};
static uint8_t          receiveBuffer[MSGSIZE+1] = {0};
static bool             transferOK;

static bool flag_nDRDY = false;


/*
 *  ======== SPI_readData ========
 *  Callback function for the nDRDY falling-edge interrupt.
 */
void SPI_readData()
{
    //
    // NOTE: SPI_transfer() cannot be called inside of this callback function.
    //  (the MCU will enter a fault loop).
    //

    // Check if nDRDY flag is still set from a previous interrupt...
    if (flag_nDRDY)
    {
        GPIO_write(GPIO_LED1_ERROR, 1);
        while (1);  // ERROR: MCU was unable to keep up with the ADC's output data rate
    }
    flag_nDRDY = true;
}


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init function(s) */
    GPIO_init();
    SPI_init();


    //
    // Configure SPI
    //
    SPI_Params_init(&spiParams);  // Initialize SPI parameters
    spiParams.frameFormat = SPI_POL0_PHA1;
    spiParams.transferMode = SPI_MODE_BLOCKING;
    spiParams.dataSize = 8;       // 8-bit data size
    spiParams.bitRate = 25000000;   // 25 MHz SCLK - Max allowed by ADS131A04

    spi = SPI_open(ADC_SPI3, &spiParams);
    if (spi == NULL) {
        GPIO_write(GPIO_LED1_ERROR, 1);
        while (1);  // SPI_open() failed
    }

    // Fill in transmitBuffer
    spiTransaction.count = MSGSIZE;
    spiTransaction.txBuf = (void *)transmitBuffer;
    spiTransaction.rxBuf = (void *)receiveBuffer;

    /* Turn on LED0 */
    GPIO_write(GPIO_LED0, 1);

    /* Set nRESET high */
    GPIO_write(ADC_nRESET, 1);

    /* Enable /DRDY interrupt */
    GPIO_enableInt(ADC_nDRDY);


    // INITIALIZE the ADC here!


    // Infinite loop - Example of reading ADC data
    while (1)
    {
        // Wait for /DRDY to go low before reading data
        if (flag_nDRDY)
        {
            // Read ADC data
            transferOK = SPI_transfer(spi, &spiTransaction);

            // Check for SPI driver error
            if (!transferOK)
            {
                // Error in SPI or transfer already in progress.
                GPIO_write(GPIO_LED1_ERROR, 1);
                while (1);  // SPI_transfer() failed
            }

            flag_nDRDY = false;

        }
    }
}
