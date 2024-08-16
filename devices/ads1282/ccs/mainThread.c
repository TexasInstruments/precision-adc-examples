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

// NOTE: This CCS project was compiled in CCS 10.1.0.00010,
// using the TI v20.2.1.LTS ARM Compiler.
// using SysConfig v1.5
// using SimpleLink v4.20.0.12

// This code was written for PAMB revision A!

// Standard Libraries
#include <lib/json.h>           // For outputting message to GUI
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

// Custom Libraries
#include "evm/hal.h"            // For ADC initialization
#include "interrupts.h"         // For state machine flags
#include "lib/bsl.h"            // For BSL caller
#include "lib/cmdline.h"        // For processing of ASCII commands
#include "settings.h"

/************************************************************************************************************
 *
 *   @brief mainThread()
 *          This is the application entry point.
 *
 *   @details This is a template project for the PAMB motherboard. Changes to the firmware are required for proper setup and communication to a specific EVM.
 *
 *   @return Application runs in continuous loop and never returns
 */


/* Driver Header files */
#include "ti/drivers/GPIO.h"
#include <ti/drivers/I2C.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include "settings.h"
#include "lib/eeprom.h"
/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    INIT_PAMB();                        // Configure PAMB peripherals

    while (!g_bUSBConfigured);          // Wait for USB device to enumerate

    GPIO_write(BOOST_EN_CONST, 1);      // Enable power to BoosterPack
    GPIO_write(LED1_CONST, 1);          // Turn on LED1 to indicate PAMB is now operational
    initADCperhiperhals();              // Configure ADC peripherals

    // Check communication with EEPROM to see if it is available and programmed
//    if (EEPROMcheck())
//    {
//        g_bEEPROM = true;
//        InitADC();                      // Device initializations
//    }

    // Variables used for data collection
//    uint8_t outpacket[30];
//    uint8_t dLength = 2;
//    uint8_t regTXdata[4] = {0};
//    uint8_t regRXdata[4] = {0};
//    uint16_t regConfig;
    /* in this implementation the frame id is always 0 */
//    outpacket[0] = 0x00;

    // State machine loop...
    while(1)
    {
        // TODO: Use function pointers to run current state machine state and abstract some of this code away...

        // Check if we have a command to process
        if(g_ui32Flags & COMMAND_RECEIVED)
        {
            // Clear the flag
            g_ui32Flags &= ~COMMAND_RECEIVED;

            // For debugging with a terminal we need to delete the return character at the end of a command. TODO: Consider building this into the command parsing routine.
            // NOTE: This is not needed when communicating with GUI Composer, unless you intentionally append a '\n' character to the end of every command string.
            delReturn();

            // Acknowledge the command and the the GUI we are BUSY (if not in terminal mode)
            json_acknowledge(g_pcCmdBuf);

            // Process the command line.
            int32_t i32Status = CmdLineProcess(g_pcCmdBuf);

            // Handle command line processing errors and notify GUI that we are IDLE (if not in terminal mode)
            switch(i32Status)
            {
                case CMDLINE_BAD_CMD:
                    json_error(ERROR_BAD_COMMAND, "'%s' is not a valid command", g_pcCmdBuf);
                    break;

                case CMDLINE_TOO_MANY_ARGS:
                    json_error(ERROR_BAD_COMMAND, "Command exceeded maximum number of arguments (%d)", CMDLINE_MAX_ARGS);
                    break;
            }

            // Clear the command buffer
            clearCommandBuffer((i32Status == 0));

            // Notify GUI that firmware is no longer busy
            json_evmState((g_ui32Flags & COLLECT_MODE) ? COLLECTING : IDLE);
        }


        // Read in new USB RX data (after COMMAND_RECIEVED has completed)
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


        // Collect data
        if (g_ui32Flags & COLLECT_MODE)
        {
//            // Check for a completed conversion
//            if(getDRDYinterruptStatus())
//            {
//
//                regConfig = getRegisterValue(CONFIG_ADDRESS);
//                // Check conversion mode and if SS need to start conversion
//                if(regConfig & CONFIG_MODE_MASK)
//                {
//                    // Start conversion for SS mode
//                    regConfig |= CONFIG_SS_MASK;
//                    regTXdata[1] = (uint8_t) (regConfig & 0xFF);
//                    regTXdata[0] = (uint8_t) ((regConfig >> 8) & 0xFF);
//                }
//                else
//                {
//                    // Do not change configuration in Continuous mode
//                    regTXdata[1] = 0;
//                    regTXdata[0] = 0;
//                }
//
//                // Go and retrieve the data and reset conversion ready flag
//                spiSendReceiveArrays(regTXdata, regRXdata, dLength);
//                setCS(LOW);
//                setDRDYinterruptStatus(false);
//                outpacket[1] = regRXdata[0];
//                outpacket[2] = regRXdata[1];
//                // Print out the conversion result to USB_BULK
//                USBBufferWrite(BULK_TX_BUFFER, outpacket, 3);
//
//                // Decrease the sample count
//                g_num_samples--;
//                // If the count has reached zero, then stop conversions
//                if(g_num_samples == 0)
//                {
//                    enableDRDYinterrupt(false);
//                    g_ui32Flags &= !(COLLECT_MODE);
//                    // Wait for USB transmit buffer to empty
//                    while (USBBufferSpaceAvailable(BULK_TX_BUFFER) < (BULK_TX_BUFFER_SIZE - 1));
//                    // Delay 1ms for peripheral to kick out last of the data
//                    SysCtlDelay(40000);
//                    json_evmState(IDLE);
//                }
//
//            }

        }

    }
}
