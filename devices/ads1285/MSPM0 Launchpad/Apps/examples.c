/*
 * Copyright (c) 2026, Texas Instruments Incorporated
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

#include "Driver/ads1285.h"
#include "Driver/hal.h"
#include "ti/driverlib/dl_gpio.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>



uint8_t regData;  
#define numSamples  512             // allocated number of samples for M0
uint32_t ADC_readings[numSamples];  // 32 bit ADC readings array.
bool g_collectData = true;          // a global variable used to begin/end continuous data collection.

//**********************************************************************************
//  Simple ADC Example:  In this example the ADC is setup in a simple configuration, 
//  measuring input 1. After initialization, this function will enter a while loop
//  to collect numSamples of ADC readings, then enter a low power mode and wait 10ms.   
//**********************************************************************************

void simpleADC(void)
{
    
    initADC();   // set ADC, set OSR and set speed mode 
    
    uint8_t ADC_ID = readSingleRegister(ID_SYNC_ADDRESS); // Read ADC ID. 
 
    //Perform an offset calibration
    //Erase any prior Cal data, and begin autocal
    writeSingleRegister(OFFSET0_ADDRESS, 0x00);
    writeSingleRegister(OFFSET1_ADDRESS, 0x00);
    writeSingleRegister(OFFSET2_ADDRESS, 0x00);
    writeSingleRegister(CONFIG1_ADDRESS,CONFIG1_MUX_INTERNALSHORTWITHA0ORESISTOR);
    //begin Offset cal operation
    sendCommand(OFSCAL_CMD);
    
    //Wait for calibration to complete
    //Get data rate
    double_t drdyPeriod = calcFdataPeriod((getRegisterValue(CONFIG0_ADDRESS) & CONFIG0_DR_MASK));
    
    //Multiply by 81 and convert to ms
    delay_ms((uint32_t)(ceil(81 * drdyPeriod * 1000)) +1 );
    
    //Save offset calibration value for uC to store (not necessary)
    readMultipleRegisters(OFFSET0_ADDRESS, 3);
    uint32_t offsetCal = getRegisterValue(OFFSET2_ADDRESS)<<16 |
                            getRegisterValue(OFFSET1_ADDRESS)<<8  |
                            getRegisterValue(OFFSET2_ADDRESS);
    
    //Restore mux to original input setting after Cal operation. 
    writeSingleRegister(CONFIG1_ADDRESS, CONFIG1_MUX_INPUT1);     
    
    //Wait some time before beginning ADC sampling.
    delay_ms(10);   

    while(g_collectData)
    {
        for(uint16_t i = 0; i<numSamples; i++) 
        {
            ADC_readings[i] = readDataDirect();
        }
        sendCommand(STANDBY_CMD);
        delay_ms(100);   
        sendCommand(WAKEUP_CMD);
    }   

}

