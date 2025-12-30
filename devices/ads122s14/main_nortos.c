/*
 * Copyright (c) 2025, Texas Instruments Incorporated
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

//------------Demonstration Program for ADS122S14 with MSPM0-----------------//
//  1) initalize M0
//  2) initalize ADC
//  3) set ADC configuration for single-shot conversion mode
//  4) poll for when conversion is ready
//  5) store conversion result 
//  6) change to continuous conversion mode and start
//  7) poll for new samples
//  8) collect adcNumSamples # of samples
//  9) stop conversions and powerdown. 
//---------------------------------------------------------------------------//

#include "evm/ads122y1x.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include "evm/hal.h"
#include "evm/crc.h"

#define adcNumSamples 64 //number of samples to collect



adc_channel_t adcSingleSample; 
adc_channel_t adcReadings[adcNumSamples] = {0};
uint16_t i = 0;

int main(void)
{
SYSCFG_DL_init();       // init M0 via syscfg driver
adcStartup();

// ADC init
// set reference scaling(2.5V, internal ref), mux setting A, 25 SPS data rate, single-shot conversion mode, status word enabled 
writeSingleRegister(REFERENCE_CFG_ADDRESS, REFERENCE_CFG_REF_VAL_2 | REFERENCE_CFG_REF_SEL_INTERNALVOLTAGEREFERENCE);
writeSingleRegister(MUX_CFG_ADDRESS, MUX_CFG_AINP_AIN0 | MUX_CFG_AINN_GND);
writeSingleRegister(DATA_RATE_CFG_ADDRESS, DATA_RATE_CFG_FLTR_OSR_FSUBDATASUB25SPS);
writeSingleRegister(DEVICE_CFG_ADDRESS, DEVICE_CFG_CONV_MODE_SINGLESHOTCONVERSIONMODE); 
writeSingleRegister(DIGITAL_CFG_ADDRESS, DIGITAL_CFG_STATUS_EN_ENABLED); 

// start conversion
writeSingleRegister(CONVERSION_CTRL_ADDRESS, CONVERSION_CTRL_START_START);
delay_cycles(150);          // optional delay.  Adjust based on your sample latency. 

// poll the ADC for new data
do { adcSingleSample = readData(); }
    while (!(adcSingleSample.status_msb & STATUS_MSB_DRDY_MASK));

delay_cycles(2048);

// Change to continuous mode and collect many samples  
writeSingleRegister(DEVICE_CFG_ADDRESS, DEVICE_CFG_CONV_MODE_CONTINUOUSCONVERSIONMODE);
writeSingleRegister(CONVERSION_CTRL_ADDRESS, CONVERSION_CTRL_START_START);
delay_cycles(150);          // optional delay.  Adjust based on your first sample latency. 

while (i < adcNumSamples)
    { 
        do { adcReadings[i] = readData(); }
    while (!(adcReadings[i].status_msb & STATUS_MSB_DRDY_MASK));
       delay_cycles(50);    // optional delay.  Adjust how frequently the ADC is polled for new samples
        i++;
    }

// stop conversions and enter powerdown mode
writeSingleRegister(CONVERSION_CTRL_ADDRESS, CONVERSION_CTRL_STOP_STOP);    // Stop CMD not required here. 
writeSingleRegister(DEVICE_CFG_ADDRESS,DEVICE_CFG_PWDN_POWERDOWNMODE);

while (1)
{
  __WFI(); // wait forever
}

}