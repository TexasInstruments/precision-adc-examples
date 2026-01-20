/*
 * Copyright (c) 2024-2026, Texas Instruments Incorporated
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

#include "Driver/hal.h"
#include "ti/devices/msp/peripherals/hw_spi.h"
#include "ti/driverlib/dl_spi.h"
#include "ti_msp_dl_config.h"

volatile bool flag_nDRDY_INTERRUPT = false;

//****************************************************************************/
//
// Timing functions
//
/*****************************************************************************/
//delay (100) = 100 * (1/M0 clock) = 100 * (1/32,000,000) = 3.125uS

void delay_ms(const uint32_t delay_time_ms)
{
    uint32_t delayTime = 31e+6 / (1000u);
    delayTime = delayTime * delay_time_ms;
    delay_cycles( delayTime );
}

void delay_us(const uint32_t delay_time_us)
{
    uint32_t delayTime = 31e+6 / (1000000u);
    delayTime = delayTime * delay_time_us;
    delay_cycles( delayTime );
}

bool waitForDRDYinterrupt(uint32_t timeout_ms)
{
    uint32_t timeout = 31e+6 / (1000u); 
    timeout =  timeout_ms * timeout;
    DL_GPIO_clearInterruptStatus(GPIOA, GPIO_GRP_0_DRDYn_PIN);
    flag_nDRDY_INTERRUPT = false;
    DL_GPIO_enableInterrupt(GPIOA, GPIO_GRP_0_DRDYn_PIN);
    // Wait for nDRDY interrupt or timeout
    do {
        timeout--;
    } while (!flag_nDRDY_INTERRUPT );
        return (timeout > 0);
}

//****************************************************************************/
//
// Interrupts
//
/*****************************************************************************/

void GROUP1_IRQHandler(uint32_t numCaptured, uint32_t streamRx[])
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case GPIO_GRP_0_INT_IIDX:
        flag_nDRDY_INTERRUPT = true;
        DL_GPIO_disableInterrupt(GPIOA, GPIO_GRP_0_DRDYn_PIN);
        break;
    }
}

//****************************************************************************/
//
// SPI functions
//
/*****************************************************************************/

void spiSendReceiveArrays(uint8_t dataTx[], uint8_t dataRx[], uint8_t bufferLength)
// this funciton will operate the MSPM0 SPI port.  Because the SPI FIFO on M0 is 16-bit,
// the dataTX and dataRX buffers are packed into 16-bit elements before being tranfered to/from the SPI FIFO. 
// this transformation improves data readback when SPI frames include SPI and CRC words ( 6-byte frame )

{
    DL_SPI_drainRXFIFO8(SPI_0_INST, dataRx, 4);


    uint8_t i = 0;
    uint8_t j = 0;


    DL_GPIO_clearPins(GPIOA, GPIO_LEDS_USER_LED_1_PIN | GPIO_GRP_0_CSn_PIN);

    while (i < bufferLength) {
        
        if(!DL_SPI_isTXFIFOFull(SPI_0_INST)) {      // add to fifo if not full
            DL_SPI_transmitData8(SPI_0_INST, dataTx[i++]);
        }
        if(!DL_SPI_isRXFIFOEmpty(SPI_0_INST)) {     // read from fifo if not empty
            dataRx[j++] = DL_SPI_receiveData8(SPI_0_INST);    
        }
            
    }
    
    while (i != j)  {
        if(!DL_SPI_isRXFIFOEmpty(SPI_0_INST)) {
            dataRx[j++] = DL_SPI_receiveData8(SPI_0_INST);    
        }
        
    }

    DL_GPIO_setPins(GPIOA, GPIO_LEDS_USER_LED_1_PIN | GPIO_GRP_0_CSn_PIN);


     
}