/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/

// NOTE: This CCS project was compiled in CCS 9.3.0.00012,
// using the TI v18.12.4.LTS ARM Compiler,
// the SimpleLink MSP432E4 SDK v3.20.0.10 library,
// and targeting the MSP-EXP432E401Y hardware.


// Standard Libraries
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Custom Libraries
#include "hal.h"            // For ADC initialization
#include "interrupts.h"         // For state machine flags


//****************************************************************************
//
// This is the application entry point.
//
//****************************************************************************
int main(void)
{
    //
    // MSP432E LaunchPad initializations
    //
    INIT_LAUNCHPAD();


    //
    // Turn on LED1
    //
    ROM_GPIOPinWrite(LED1_PORT, LED1_PIN, LED1_PIN);


    //
    // Device initializations
    //
    InitADC();


    //
    // Read data... Time from /CS low to /CS high is 22.12us -> allows for up to ~32 kSPS
    //
    adc_data_struct data;
    while(1)
    {
        waitForDRDYinterrupt(1000);
        bool crcError = readData(&data);
        if (crcError) { ROM_GPIOPinWrite(LED2_PORT, LED2_PIN, LED2_PIN); }

        // For debugging, read the ERROR_CNT register...
        //uint8_t val = readSingleRegister(ERROR_CNT_ADDRESS);
    }
}
