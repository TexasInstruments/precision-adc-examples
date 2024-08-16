/*
 * terminal.h
 *
 *  Created on: Nov 9, 2018
 *      Author: a0282860
 */

#ifndef TERMINAL_H_
#define TERMINAL_H_

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <evm/json.h> // TODO: Remove this dependence

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
#define CMD_BUF_SIZE    64
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

#endif /* TERMINAL_H_ */
