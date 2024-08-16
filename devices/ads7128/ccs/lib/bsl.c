/* --COPYRIGHT--,BSD
 * Copyright (c) 2019, Texas Instruments Incorporated
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


#include "bsl.h"

// Internal function prototypes
static void blink_leds(void);



//*****************************************************************************
//
// Function to erase flash memory
//
//*****************************************************************************
#define FLASH_SECTOR_SIZE (1024)
void erase_flash(void)
{
    blink_leds();

    uint32_t junk;

    /* Enter a loop to erase all the requested flash pages */
    for (junk = 0x00; junk < 1024000; junk += FLASH_SECTOR_SIZE)
    {
        ROM_FlashErase(junk);
    }
}



//*****************************************************************************
//
// Function call to jump to the boot loader.
//
//*****************************************************************************
void JumpToBootLoader(void)
{
    /* Blink LEDs to notify that we about to jump boot loader mode */
    blink_leds();

    // Disconnect USB device
    USBDevDisconnect(USB0_BASE);

    // Set USB MUX to default for DFU mode
    GPIOPinWrite(MUXSEL_PORT, MUXSEL_PIN, MUXSEL_PIN);

    // Enable Supervisor Watchdog timer
    GPIOPinWrite(WD_PORT, WD_PIN, WD_PIN);

    // Software reset the micro
    SysCtlReset();
}



//*****************************************************************************
//
// Internal functions
//
//*****************************************************************************
static void blink_leds(void)
{
    uint_fast8_t i;
    for(i = 0; i < 3; i++)
    {
        /* Delay cycles for ~0.15 second */
        if (0 != i) { SysCtlDelay(6000000); }

        /* All ON */
        GPIOPinWrite(LED1_PORT, LED1_PIN, LED1_PIN);
        GPIOPinWrite(LED2_PORT, LED2_PIN, LED2_PIN);
        GPIOPinWrite(LED3_PORT, LED3_PIN, LED3_PIN);
        GPIOPinWrite(LED4_PORT, LED4_PIN, LED4_PIN);

        /* Delay cycles for ~0.15 second */
        SysCtlDelay(6000000);

        /* All OFF */
        GPIOPinWrite(LED1_PORT, LED1_PIN, 0);
        GPIOPinWrite(LED2_PORT, LED2_PIN, 0);
        GPIOPinWrite(LED3_PORT, LED3_PIN, 0);
        GPIOPinWrite(LED4_PORT, LED4_PIN, 0);
    }
}
