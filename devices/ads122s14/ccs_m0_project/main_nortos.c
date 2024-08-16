/*
 * Copyright (c) 2024, Texas Instruments Incorporated
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
//  3) start continuous conversion
//  4) set mux configuration A
//  5) poll for when conversion is ready
//  6) store conversion result in array, loop to collect desired # of samples
//  7) when # samples collected, exit loop and set configuration B
//  8) configuration B poll and collect desired # of samples
//  9) stop conversions
//  10) enter power down mode
//  11) delay (here other M0 process could take place while ADC is off and consuming low power)
//  12) exit power down mode
//  13) loop back to take next set of data


//---------------------------------------------------------------------------//



#include "evm/ads122s1x.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include "evm/hal.h"

#define adcNumSamplesA 5 //number of samples to collect for config A
#define adcNumSamplesB 10 //number of samples to collect for config B
#define FSR 2.5        //internal reference 2.5V 

//delay explained
//delay (100) = 100 * (1/M0 clock) = 100 * (1/32,000,000) = 3.125uS
//extra delay time in logic analyzer comes from additional 10,000 clocks SPI delays implemented (SPI_DELAY)

uint8_t STATUS_MSB = 0; 
bool drdy_flag = false;
bool collect = true;

int j = 0; //for loop count
uint32_t adcReadingsConfigA[adcNumSamplesA]; // space to save 5 readings of mux config A, AIN0-GND, need to initialize or doesn't mater?
uint32_t adcReadingsConfigB[adcNumSamplesB]; //space to save 10 readings of mux config B, AIN1-AIN2

//test variables
uint8_t mux_cfgA = 0; //should = 08h
uint8_t mux_cfgB = 0; //should read 12h

int main(void)
{
SYSCFG_DL_init();       // init M0 via syscfg driver

resetDevice();          // issue an SPI reset command
delay_cycles(4000);     // delay ~100us to allow the ADC to reset 

initADC();              // set reference scaling(2.5V, internal ref), mux setting A, 250 SPS data rate, can set to single-shot mode TODO: activate line in ADS122s14.c file


//While(1){              //add while(1) loop here to loop around program infinitely

startAdcConversion();   // start continuous conversions

configurationA(); //set mux to config A

mux_cfgA = readSingleRegister(MUX_CFG_ADDRESS);// should read 08h

for(j = 0; j < adcNumSamplesA; j++)
{
    //read
    while (collect)
    {
        if (drdy_flag)
        {
            adcReadingsConfigA[j] =readData();     // read 24'b ADC Data
            collect = false;            // deassert flag for exit
        }
        else 
        {
            STATUS_MSB = readSingleRegister(STATUS_MSB_ADDRESS);        // read STATUS_MSB and check for new data
            drdy_flag  = STATUS_MSB & STATUS_MSB_DRDY_MASK;             // if bit it set, enable drdy_flag
        }
    }
    drdy_flag = false;
    collect = true;

}

delay_cycles(8000);

configurationB(); //set mux configuration B

mux_cfgB = readSingleRegister(MUX_CFG_ADDRESS);//should read 12h

for(j = 0; j < adcNumSamplesB; j++)
{
    //read
    while (collect)
    {
        if (drdy_flag)
        {
            adcReadingsConfigB[j] =readData();     // read 24'b ADC Data
            collect = false;            // deassert flag for exit
        }
        else 
        {
            STATUS_MSB = readSingleRegister(STATUS_MSB_ADDRESS);        // read STATUS_MSB and check for new data
            drdy_flag  = STATUS_MSB & STATUS_MSB_DRDY_MASK;             // if bit it set, enable drdy_flag
        }
    }
    drdy_flag = false;
    collect = true;

}


stopAdcConversion();       //84h 01h             stop conversions
powerDownMode(); //85h 80h, power down adc
//convert codes to voltage example? other M0 processes could take place here
delay_cycles(160000); //delay 0.5 seconds
activeMode(); //85h 00h, power up adc

//}      //end while(1) loop here

}

// TODO: add while(1) loop to loop around process infinitely 


