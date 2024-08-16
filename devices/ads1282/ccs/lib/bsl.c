/* --COPYRIGHT--,BSD
 * Copyright (c) 2021, Texas Instruments Incorporated
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
#include "evm\hal.h"
#include "..\settings.h"

// Internal function prototypes
static void blink_leds(void);



//*****************************************************************************
//
// Function to erase flash memory
//
//*****************************************************************************
// TODO: Need to consider the importance in keeping....may be a good failsafe
/******************************************************************************
 *
 *  @brief A macro used to MSP432E FLASH sector size when erasing the flash contents.
 *
 *****************************************************************************/
#define FLASH_SECTOR_SIZE (1024)

void erase_flash(void)
{
    blink_leds();

    uint32_t junk;
    // TODO: technically all that needs to be erased is the first page so that the application space is empty at reset
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
    /* Blink LEDs to notify that we about to jump boot loader mode...
	 *
	 * NOTE: This indication is optional, but I (Chris) recommend including it as a sanity
	 * check and/or in the event that we need to walk a customer through this process.
	 */
    blink_leds();

    // Disconnect USB device
    USBDevDisconnect(USB0_BASE);
    GPIO_write(MUXSEL_CONST, 1);
    GPIO_write(WD_CONST, 1);
    // Software reset the micro
    SysCtlReset();
}


//*****************************************************************************
//
// Internal functions
//
//*****************************************************************************

/****************************************************************************
 *
 * @brief Blinks the four primary LEDs on PAMB.
 *
 * @return none
 *
 ***************************************************************************/
static void blink_leds(void)
{
    uint_fast8_t i;
    for(i = 0; i < 3; i++)
    {
        /* Delay cycles for ~0.15 second */
        if (0 != i) { SysCtlDelay(6000000); }
        /* All ON */
        GPIO_write(LED1_CONST, 1);
        GPIO_write(LED2_CONST, 1);
        GPIO_write(LED3_CONST, 1);
        GPIO_write(LED4_CONST, 1);

        /* Delay cycles for ~0.15 second */
        SysCtlDelay(6000000);

        /* All OFF */
        GPIO_write(LED1_CONST, 0);
        GPIO_write(LED2_CONST, 0);
        GPIO_write(LED3_CONST, 0);
        GPIO_write(LED4_CONST, 0);
    }
}
