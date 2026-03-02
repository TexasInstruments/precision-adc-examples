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


#include "Driver/hal.h"
#include "ti/devices/msp/peripherals/hw_spi.h"
#include "ti_msp_dl_config.h"


/*
 *  Helper function to transmit data from the Controller to the Peripheral.
 *  This function assumes that the command has already been transmitted, so
 *  the CD line should already be set HIGH indicating that data is being
 *  transmitted.
 *
 *  data         The data to send to the Peripheral.
 *               Example: DATA_TYPE_0
 *  dataLength   The number of data bytes to send.
 *               Example: DATA_TYPE_0_LENGTH
 */


//*****************************************************************************
//
//                   I2C Communications
//
//*****************************************************************************

void I2C_Write (uint8_t dataTx[], uint8_t wLength) 
{     
    DL_I2C_resetControllerTransfer(I2C_0_INST);    // reset I2C controller settings  
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, dataTx, wLength);        // fill the Tx FIFO
   
    DL_I2C_startControllerTransfer(I2C_0_INST, I2C_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_TX, wLength);

    do
    {
        delay_cycles(10);
    } while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY));
      
    // Flush Tx Buffer afterwords
    DL_I2C_flushControllerTXFIFO(I2C_0_INST);                                                  
}

void I2C_Read(uint8_t dataRx[], uint8_t rLength) 
{
    // reset I2C controller settings    
    DL_I2C_resetControllerTransfer(I2C_0_INST);                                                     
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);       

    // Begin reading
    DL_I2C_startControllerTransfer(I2C_0_INST, I2C_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_RX,
        rLength); 

    // wait for idle.
    do
    {
        delay_cycles(10);
    } while ((DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY));
   
   
   
    //while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));  

    for (int i = 0; i < rLength; i++)
    {
        dataRx[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    }
    
    // Flush Rx Buffer afterwords
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);                          
  
}