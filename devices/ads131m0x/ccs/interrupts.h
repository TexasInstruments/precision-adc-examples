/*
 * interrupts.h
 *
 *  Created on: Jan 17, 2019
 *      Author: a0282860
 */

#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "settings.h"



//****************************************************************************
//
// Flags used to pass commands from interrupt context to the main loop.
//
//****************************************************************************

// USB Events
#define CDC_RX_RECEIVED             (0x00000001)        // CDC
#define COMMAND_RECEIVED            (0x00000002)
//#define PLACEHOLDER               (0x00000004)
//#define PLACEHOLDER               (0x00000008)

#define TRANSMIT_COMPLETE           (0x00000010)
#define COMMAND_STATUS_UPDATE       (0x00000020)
//#define PLACEHOLDER               (0x00000040)
//#define PLACEHOLDER               (0x00000080)

//#define PLACEHOLDER               (0x00000100)
//#define PLACEHOLDER               (0x00000200)
//#define PLACEHOLDER               (0x00000400)
//#define PLACEHOLDER               (0x00000800)

//#define PLACEHOLDER               (0x00001000)
//#define PLACEHOLDER               (0x00002000)
//#define PLACEHOLDER               (0x00004000)
//#define PLACEHOLDER               (0x00008000)

//#define PLACEHOLDER               (0x00010000)
//#define PLACEHOLDER               (0x00020000)
//#define PLACEHOLDER               (0x00040000)
//#define PLACEHOLDER               (0x00080000)

//#define PLACEHOLDER               (0x00100000)
//#define PLACEHOLDER               (0x00200000)
//#define PLACEHOLDER               (0x00400000)
//#define PLACEHOLDER               (0x00800000)

#define COLLECT_MODE                (0x01000000)
//#define STREAM_MODE                 (0x02000000)
//#define PLACEHOLDER               (0x04000000)
//#define PLACEHOLDER               (0x08000000)

//#define PLACEHOLDER               (0x10000000)
//#define PLACEHOLDER               (0x20000000)
#define ERASE_MODE                  (0x40000000)
#define BOOTLOADER_MODE             (0x80000000)



//****************************************************************************
//
// Data collection
//
//****************************************************************************
extern uint32_t g_num_samples;
void readComplete(void);



//****************************************************************************
//
// Default line coding settings for the redirected UART.
//
//****************************************************************************
#define DEFAULT_BIT_RATE        (115200)
#define DEFAULT_UART_CONFIG     (UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | \
                                 UART_CONFIG_STOP_ONE)


//****************************************************************************
//
// Globals
//
//****************************************************************************
extern volatile uint32_t g_ui32Flags;
//static bool g_bSendingBreak = false;

// Global flag indicating that a USB configuration has been set.
extern volatile bool g_bUSBConfigured;

// Interrupt Handlers
extern void USB0_IRQDeviceHandler(void);
#ifdef BSL_INTERRUPT
    extern void GPION_IRQHandler(void);
#endif

#endif /* INTERRUPTS_H_ */
