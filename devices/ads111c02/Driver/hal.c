/**
 * \copyright Copyright (C) 2026 Texas Instruments Incorporated - http://www.ti.com/
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


/**
 * @file hal.c
 * @brief Hardware abstraction layer (I2C, SPI, GPIO and delays) for the MSPM0.
 */

#include "Driver/hal.h"
#include "ti/devices/msp/peripherals/hw_spi.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>

volatile bool flag_nDRDY_INTERRUPT = false;

//*****************************************************************************
//
//                   I2C Communications
//
//*****************************************************************************

void I2C_Write (uint8_t dataTx[], uint8_t wLength) 
{     
    DL_I2C_resetControllerTransfer(I2C_0_INST);    // reset I2C controller settings  
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, dataTx, wLength);        // fill the Tx FIFO
   
    DL_I2C_startControllerTransfer(I2C_0_INST, g_i2c_addr, DL_I2C_CONTROLLER_DIRECTION_TX, wLength);

    do
    {
        delay_cycles(10);
    } while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY));
      
    // Flush Tx Buffer afterward
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);   

    delay_us(50);                                               
}

void I2C_Read(uint8_t dataRx[], uint8_t rLength) 
{
    // reset I2C controller settings    
    DL_I2C_resetControllerTransfer(I2C_0_INST);                                                     
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);       

    // Begin reading
    DL_I2C_startControllerTransfer(I2C_0_INST, g_i2c_addr, DL_I2C_CONTROLLER_DIRECTION_RX,
        rLength); 

    // wait for idle.
    do
    {
        delay_cycles(10);
    } while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY));
   
   
       for (int i = 0; i < rLength; i++)
    {
        dataRx[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    }
    
    // Flush Rx Buffer afterward
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);                          
    delay_us(50);
}

//*****************************************************************************
//
//                   SPI Communications
//
//*****************************************************************************


void spiSendReceiveArrays(uint8_t dataTx[], uint8_t dataRx[], uint8_t bufferLength)
// this function will operate the MSPM0 SPI port.  Because the SPI FIFO on M0 is 16-bit,
// the dataTX and dataRX buffers are packed into 16-bit elements before being transferred to/from the SPI FIFO. 
// this transformation improves data readback when SPI frames include SPI and CRC words ( 6-byte frame )

{
    DL_GPIO_initPeripheralOutputFunction(GPIO_SPI_0_IOMUX_PICO, GPIO_SPI_0_IOMUX_PICO_FUNC);
    DL_SPI_enable(SPI_0_INST);
    
    DL_SPI_drainRXFIFO8(SPI_0_INST, dataRx, 4);


    uint8_t i = 0;
    uint8_t j = 0;

   setCS(LOW);

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

     setCS(HIGH);
}

    void setCS(bool state)
    {

        if (state)
        {
            DL_GPIO_setPins(GPIOA, GPIO_GRP_0_CS_n_PIN);
        }
        else
        {
            DL_GPIO_clearPins(GPIOA, GPIO_GRP_0_CS_n_PIN);
        }
    }

//****************************************************************************/
//
// Timing functions
//
/*****************************************************************************/
//delay_cycles (100) = 100 * (1/M0 clock) = 100 * (1/32,000,000) = 3.125uS

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

bool waitForDRDY(uint32_t timeout_ms)
{

    uint32_t timeout = 31e+6 / (1000u); 
    timeout =  timeout_ms * timeout;
    flag_nDRDY_INTERRUPT = false;
    setCS(LOW);

    // Wait for nDRDY interrupt or timeout
     while (DL_GPIO_readPins(GPIOA,GPIO_GRP_0_DRDYn_PIN) && timeout != 0)
        {
        timeout--;

        } 
    
    
    return (timeout > 0);
  
}