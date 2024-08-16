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

// NOTE: This CCS project was compiled in CCS 8.3.1.00004,
// using the TI v18.1.3.LTS ARM Compiler.

// This code was written for PAMB revision A!

// TODO: Remove unneeded include paths
// TODO: Update CCS project to CCS 9.x.x
// TODO: Update/test the latest ARM compiler

// Standard Libraries
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Custom Libraries
#include "evm/hal.h"            // For ADC initialization
#include "evm/json.h"           // For outputting message to GUI
#include "interrupts.h"         // For state machine flags
#include "lib/bsl.h"            // For BSL caller
#include "lib/cmdline.h"        // For processing of ASCII commands


//****************************************************************************
//
// This is the application entry point.
//
//****************************************************************************
int main(void)
{
    //
    // MSP432E initializations
    //
    INIT_PAMB();

    //
    // Wait for USB device to enumerate
    //
    while (!g_bUSBConfigured);

    //
    // Enable power to BoosterPack
    //
    ROM_GPIOPinWrite(BOOST_EN_PORT, BOOST_EN_PIN, BOOST_EN_PIN);

    //
    // Turn on LED1
    //
    ROM_GPIOPinWrite(LED1_PORT, LED1_PIN, LED1_PIN);

    //
    // Device initializations
    //
    InitADC();

    //
    // State machine loop...
    //
    while(1)
    {
        // TODO: Use function pointers to run current state machine state and abstract some of this code away...

        //
        // Check if we have a command to process
        //
        if(g_ui32Flags & COMMAND_RECEIVED)
        {
            // Clear the flag
            g_ui32Flags &= ~COMMAND_RECEIVED;

            // For debugging with a terminal we need to delete the return character at the end of a command. TODO: Consider building this into the command parsing routine.
            // NOTE: This is not needed when communicating with GUI Composer, unless you intentionally append a '\n' character to the end of every command string.
            //delReturn();

            // Acknowledge the command and the the GUI we are BUSY (if not in terminal mode)
            json_acknowledge(g_pcCmdBuf);

            // Process the command line.
            int32_t i32Status = CmdLineProcess(g_pcCmdBuf);

            // Handle command line processing errors and notify GUI that we are IDLE (if not in terminal mode)
            switch(i32Status)
            {
                case CMDLINE_BAD_CMD:
                    json_console("Error: '%s' is not a valid command", g_pcCmdBuf);
                    break;

                case CMDLINE_TOO_MANY_ARGS:
                    json_console("Error: Command exceeded maximum number of arguments (%d)", CMDLINE_MAX_ARGS);
                    break;
            }
            if (i32Status) { json_error(ERROR_BAD_COMMAND); }

            // Clear the command buffer
            clearCommandBuffer((i32Status == 0));

            // Notify GUI that firmware is no longer busy
            json_evmState(IDLE);
        }


        //
        // Read in new USB RX data (only after COMMAND_RECIEVED has completed)
        //
        if(g_ui32Flags & CDC_RX_RECEIVED)
        {
            // Clear the flag
            g_ui32Flags &= ~CDC_RX_RECEIVED;

            uint32_t numBytesAvailable = RxBytesAvailable();

            if (numBytesAvailable)
            {
                // Read entire command into buffer - TODO: Read up until separator
                ReadCdcRxBuffer(numBytesAvailable);

                // Check if separator (i.e. complete command) was found,
                // if so, set COMMAND_RECEIVED flag, otherwise continue

                // Indicate that UI command has been received
                g_ui32Flags |= COMMAND_RECEIVED;
            }
        }


        //
        // Collect data?
        //
        if (g_ui32Flags & COLLECT_MODE)
        {
            //json_evmState(COLLECTING);

            //TODO: EVM state = collecting (here or in collect command)

            //TODO: Implement

            //TODO: EVM state = idle (here or in stop command)
        }


        //
        // Run boot loader application...
        //
        // NOTE: These modes won't execute if we are in collect mode;
        // GUI should exit collect mode before calling these commands!
        //
        else if (g_ui32Flags & BOOTLOADER_MODE)
        {
            g_ui32Flags &= ~BOOTLOADER_MODE;
            JumpToBootLoader();
        }
        else if (g_ui32Flags & ERASE_MODE)
        {
            // NOTE: Application will no longer run after erasing FLASH,
            // and a reset will be required to enter ROM boot loader mode!
            erase_flash();
        }
    }
}
