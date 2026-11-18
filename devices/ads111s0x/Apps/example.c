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



//---------------------------------------------------------------------------//
/******************************************
[3.3V Power Supply]
                  |
           [1,000 Ω Resistor]
                  |
        /---------+---------\
        |                   |
        [K-Type Thermocouple|
        |                   |
        \---------+---------/
                  |
               [Ground]
******************************************/


#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include "Apps/Sensor/inc/thermocouple.h"
#include "example.h"
#include"/Driver/Inc/ads_device.h"

//uint8_t g_i2c_addr = ADDR_1;

//****Temperature Sensor Library Settings*****/
TC_Type TCtype = K;
//********************************************/


float internalCJCtemp;
float internalCJTvoltage;
float ADCmV;
float TCtemp;


ADCchar_Set ADCsettings;

void applicationExample(void)
{

adcStartup();


// set initial conditions for ADC
#if defined (ADS_DEVICE_S02)
    writeSingleRegister(CONFIG_ADDRESS, CONFIG_MUX_AINP_AIN0_AINN_GND | CONFIG_PGA_GAIN_1 | CONFIG_TS_MUX_SELECTED_FOR_MEASUREMENT |
        CONFIG_MODE_SINGLE_SHOT_CONVERSION_MODE | CONFIG_WRITE_EN_VALID_DATA);   
#elif defined (ADS_DEVICE_S04)
    writeSingleRegister(CONFIG_ADDRESS, CONFIG_MUX_AINP_AIN0_AINN_GND | CONFIG_PGA_GAIN_1 | CONFIG_TS_MUX_SELECTED_FOR_MEASUREMENT |
        CONFIG_MODE_SINGLE_SHOT_CONVERSION_MODE | CONFIG_WRITE_EN_VALID_DATA);   
#elif defined (ADS_DEVICE_C02)
    writeSingleRegister(CONFIG_ADDRESS, CONFIG_MUX_AINP_AIN0_AINN_GND | CONFIG_PGA_GAIN_1 | CONFIG_TS_MUX_SELECTED_FOR_MEASUREMENT |
        CONFIG_MODE_SINGLE_SHOT_CONVERSION_MODE);   
    writeSingleRegister(HI_THRESH_ADDRESS, 0x8000); // enable the DRDY pin
#elif defined (ADS_DEVICE_C04)
    writeSingleRegister(CONFIG_ADDRESS, CONFIG_MUX_AINP_AIN0_AINN_GND | CONFIG_PGA_GAIN_1 | CONFIG_TS_MUX_SELECTED_FOR_MEASUREMENT |
        CONFIG_MODE_SINGLE_SHOT_CONVERSION_MODE);   
    
#endif


ADCsettings.resolution = RESOLUTION_IS_16_BIT? 16 : 12;
ADCsettings.pgaGain = 1;
ADCsettings.Vref = 1.024;


while (1)
    {

    // Start a single shot converison
    startAdcConversion();   
  
    // Save an reading from the TC sensor
    ADCsettings.adcValue = readData();

    // Convert ADC reading to a mV value
    ADCmV = ((1000.0 * ADCsettings.adcValue * (ADCsettings.Vref)) / pow(2, ((ADCsettings.resolution) - 1)));
   
    // Take a measurement from the internal temp sensor
    internalCJCtemp = measureInternalTemperature();

    //using the Sensor Lib, compute a Temp measurement from the TC. 
    TCtemp = Thermocouple_Linearization_and_CJC(TCtype, ADCmV, internalCJCtemp);
    
    // wait then repeat. 
    delay_ms(100);
    }
}


