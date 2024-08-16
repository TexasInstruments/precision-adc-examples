/*
 *
 *  \copyright Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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
/*****************************************************************************
 *
 * @brief Event flag indicating a CDC command communication is in process.
 *
 ****************************************************************************/
#define CDC_RX_RECEIVED             (0x00000001)        // CDC

/*****************************************************************************
 *
 * @brief Event flag indicating complete command has been received and ready to process.
 *
 ****************************************************************************/
#define COMMAND_RECEIVED            (0x00000002)
//#define PLACEHOLDER               (0x00000004)
//#define PLACEHOLDER               (0x00000008)
/*****************************************************************************
 *
 * @brief Event flag indicating a USB transmission is complete.
 *
 ****************************************************************************/
#define TRANSMIT_COMPLETE           (0x00000010)

/*****************************************************************************
 *
 * @brief Event flag indicating an update in command status.
 *
 ****************************************************************************/
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

/*****************************************************************************
 *
 * @brief Command event flag for data collection.
 *
 ****************************************************************************/
#define COLLECT_MODE                (0x01000000)
#define STREAM_MODE                 (0x02000000)
//#define PLACEHOLDER               (0x04000000)
//#define PLACEHOLDER               (0x08000000)

//#define PLACEHOLDER               (0x10000000)
//#define PLACEHOLDER               (0x20000000)
//#define PLACEHOLDER               (0x40000000)
//#define PLACEHOLDER               (0x80000000)

//****************************************************************************
//
// Data collection
//
//****************************************************************************
extern uint32_t g_num_samples;

//****************************************************************************
//
// Default line coding settings for the redirected UART.
//
//****************************************************************************
#define DEFAULT_BIT_RATE        (115200)
// TODO: Verify if this is necessary for PAMB as CDC is being used (maybe if IsUART?)
#define DEFAULT_UART_CONFIG     (UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | \
                                 UART_CONFIG_STOP_ONE)


//****************************************************************************
//
// Globals
//
//****************************************************************************
extern volatile uint32_t g_ui32Flags;

// Global flag indicating that a USB configuration has been set.
extern volatile bool g_bUSBConfigured;
// Global flag indicating that an EEPROM is available
extern volatile bool g_bEEPROM;
// Interrupt Handlers
/*****************************************************************************
 *
 * @brief Handler routine for the USB interface.
 *
 * @param none
 *
 * @return none
 *
 ****************************************************************************/
extern void USB0_IRQDeviceHandler(void);
// TODO: BSL interrupt can be removed as there is no way to invoke via hardware
#ifdef BSL_INTERRUPT
    extern void GPION_IRQHandler(void);
#endif

#endif /* INTERRUPTS_H_ */
