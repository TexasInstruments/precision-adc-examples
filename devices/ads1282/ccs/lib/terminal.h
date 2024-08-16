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

#ifndef LIB_TERMINAL_H_
#define LIB_TERMINAL_H_

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <lib/json.h> // TODO: Remove this dependence
#include "interrupts.h"

// Define to command history
//#define ENABLE_HISTORY_BUFFER

// Define to enable single character terminal parsing
//#define ENABLE_TERMINAL_PROCESSING

//#define UNIT_TESTING

// Return values
//#define BUFFER_OVERFLOW         ((int8_t) -3)       // Print error message
//#define DELETE_CHARACTER        ((int8_t) -2)       // Delete current position and right shift
//#define BACKSPACE_CHARACTER     ((int8_t) -1)       // Backspace
//#define IGNORE_CHARACTER        ((int8_t)  0)       // No echo
//#define ECHO_CHARACTER          ((int8_t)  1)       // Echo single character
//#define ECHO_STRING             ((int8_t)  2)       // Echo entire string (when recalling a past command)
//#define EXECUTE_COMMAND         ((int8_t)  3)       // Call command parsing function
//#define LEFT_ARROW              ((int8_t)  4)       // Move cursor left
//#define RIGHT_ARROW             ((int8_t)  5)       // Move cursor right




//****************************************************************************
//
// The buffer that holds the command line.
//
//****************************************************************************

// Defines the size of the buffer that holds the command line.
// NOTE: We may run into errors if we try to send more than 16 characters at a time through the UART FIFO runs out of memory,
// However, since we are using USB CDC we can use a larger buffer
#define CMD_BUF_SIZE    (256 )
extern char g_pcCmdBuf[CMD_BUF_SIZE];

//for debugging remove return
void delReturn(void);
uint32_t RxBytesAvailable(void);
void ReadCdcRxBuffer(uint32_t numBytesToRead);     // Used in UI mode
void clearCommandBuffer(bool validCommand);

#ifdef UNIT_TESTING
bool TerminalProcess_test(void);
#endif

#ifdef ENABLE_TERMINAL_PROCESSING
void TerminalProcess(void);
#endif

#endif /* LIB_TERMINAL_H_ */
