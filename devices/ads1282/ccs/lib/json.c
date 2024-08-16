/**
 * \copyright Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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

#include <lib/json.h>

//TODO: Need to add function documentation
// Select function to use for printing to IO
#define PRINT USBprintf
//****************************************************************************
//
// Internal variables
//
//****************************************************************************

//****************************************************************************
//
// Function Definitions
//
//****************************************************************************

//
// NOTE: Make sure each of the following functions aligns with a defined case
// in pambCustomCodec.js's "decode()" function!
//

// IMPORTANT: GUI composer is expecting ALL JSON strings to end with '\r\n' in order
// in to differentiate between the end of one JSON string and the beginning of another!



// Sent in response to a received command, to acknowledge the command
void json_acknowledge(const char *commandBuffer)
{
    PRINT("{\"acknowledge\":\"%s\"}\r\n", commandBuffer);
}

// Prints a message to the GUI's console
void json_console(const char *pcString, ...)
{
    va_list vaArgP;

    //
    // Start the vargs processing.
    //
    va_start(vaArgP, pcString);

    const char *json_console_start = "{\"console\":\"";
    USBBufferWrite(CDC_TX_BUFFER, (uint8_t*)json_console_start, strlen(json_console_start));

    USBvprintf(pcString, vaArgP);

    const char *json_console_end = "\"}\r\n";
    USBBufferWrite(CDC_TX_BUFFER, (uint8_t*)json_console_end, strlen(json_console_end));

    //
    // We're finished with the vargs now.
    //
    va_end(vaArgP);
}

// Sends generic data to GUI's
// NOTE: You must provide the structure to the JSON string!
void json_data(const char *pcString, ...)
{
    va_list vaArgP;

    // Start the vargs processing.
    va_start(vaArgP, pcString);

    // Print formatted string
    USBvprintf(pcString, vaArgP);

    // Print delimiter
    const char *json_console_end = "\r\n";
    USBBufferWrite(CDC_TX_BUFFER, (uint8_t*)json_console_end, strlen(json_console_end));

    // We're finished with the vargs now.
    va_end(vaArgP);
}
// Sends an error notification to the GUI
void json_error(const int error_code, const char *msg, ...)
{
    // Start the vargs processing.
    va_list vaArgP;
    va_start(vaArgP, msg);

    PRINT("{\"error\":{\"no\":%d,\"msg\":\"", error_code);
    USBvprintf(msg, vaArgP);    // Insert message string

    const char *json_error_end = "\"}}\r\n";
    USBBufferWrite(CDC_TX_BUFFER, (uint8_t*)json_error_end, strlen(json_error_end));

    // We're finished with the vargs now.
    va_end(vaArgP);
}

// Updates the GUI of the current firmware state
void json_evmState(const evmState_t state)
{

    PRINT("{\"evm_state\":\"");
    switch(state)
    {
        case IDLE:
            PRINT("idle");
            break;

        case BUSY:
            PRINT("busy");
            break;

        case COLLECTING:
            PRINT("collecting");
            break;
    }
    PRINT("\"}\r\n");
}


void json_id(const char *name, const char *version, const char *date, const char *time)
{
    PRINT("{\"id\":{\"name\":\"%s\",\"version\":\"%s\",\"date\":\"%s\",\"time\":\"%s\"}}\r\n", name, version, date, time);
}

// TODO: Make register size adjustable, update GUI to recognize hex format
void json_register(const uint8_t regAddr, const  uint8_t regVal)
{
    PRINT("{\"register\":{\"address\":%d,\"value\":0x%02X}}\r\n", regAddr, regVal);
}

void json_command(const char *cmdStr, const char *descriptionStr)
{
    PRINT("{\"command\":{\"string\":\"%s\",\"description\":\"%s\"}}\r\n", cmdStr, descriptionStr);
}



