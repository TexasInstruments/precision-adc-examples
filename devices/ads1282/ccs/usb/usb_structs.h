/**
 * \copyright Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef USB_USB_STRUCTS_H_
#define USB_USB_STRUCTS_H_

#include "ti/usblib/msp432e4/device/usbdevice.h"
#include "ti/usblib/msp432e4/device/usbdcdc.h"
#include "ti/usblib/msp432e4/device/usbdbulk.h"
#include "ti/usblib/msp432e4/device/usbdcomp.h"
#include "ti/usblib/msp432e4/device/usbdevicepriv.h"

//*****************************************************************************
//
// The size of the transmit and receive buffers.
// This number should be a power of 2 for best performance.  Size is chosen
// pretty much at random, though the buffer should be at least twice the size of
// a maximum-sized USB packet.
//
//*****************************************************************************





/////////////////////////////////////////////////////////////////////////////////////
/// COMPOSITE DEVICE

extern tUSBDCompositeDevice g_sCompDevice;

// Number of devices defined in composite USB structure
#define NUM_USB_DEVICES         (2)
extern tCompositeEntry g_psCompEntries[NUM_USB_DEVICES];

// Size of USB descriptor data
#define DESCRIPTOR_DATA_SIZE    (COMPOSITE_DCDC_SIZE + COMPOSITE_DBULK_SIZE)
extern uint8_t g_pui8DescriptorData[DESCRIPTOR_DATA_SIZE];


/// BULK Device (#1) ///
                                            // Configure as needed...
#define BULK_RX_BUFFER_SIZE      (8)        // Not used
#define BULK_TX_BUFFER_SIZE      (8192)     // For sending binary data to PC

extern tUSBDBulkDevice g_psBULKDevice;
extern tUSBBuffer g_psBulkTxBuffer;
extern tUSBBuffer g_psBulkRxBuffer;

// Event handlers (defined in interrupts.c)
extern uint32_t RxHandlerBULK(void *pvCBData, uint32_t ui32Event,
                          uint32_t ui32MsgValue, void *pvMsgData);
extern uint32_t TxHandlerBULK(void *pvi32CBData, uint32_t ui32Event,
                          uint32_t ui32MsgValue, void *pvMsgData);


/// CDC Device (#2) ///
                                            // Configure as needed...
#define CDC_RX_BUFFER_SIZE      (256)       // For short ASCII commands
#define CDC_TX_BUFFER_SIZE      (8192)      // For longer JSON responses

extern tUSBDCDCDevice g_psCDCDevice;
extern tUSBBuffer g_psCdcTxBuffer;
extern tUSBBuffer g_psCdcRxBuffer;

// Event handlers (defined in interrupts.c)
extern uint32_t RxHandlerCmd(void *pvCBData, uint32_t ui32Event,
                             uint32_t ui32MsgValue, void *pvMsgData);
extern uint32_t TxHandlerCmd(void *pvlCBData, uint32_t ui32Event,
                             uint32_t ui32MsgValue, void *pvMsgData);
extern uint32_t ControlHandler(void *pvCBData, uint32_t ui32Event,
                             uint32_t ui32MsgValue, void *pvMsgData);


/////////////////////////////////////////////////////////////////////////////////////
/// MACROS (for use outside of this module)
#define BULK_TX_BUFFER  ((tUSBBuffer *)&g_psBulkTxBuffer)
#define BULK_RX_BUFFER  ((tUSBBuffer *)&g_psBulkRxBuffer)
#define CDC_TX_BUFFER   ((tUSBBuffer *)&g_psCdcTxBuffer)
#define CDC_RX_BUFFER   ((tUSBBuffer *)&g_psCdcRxBuffer)


#endif // USB_USB_STRUCTS_H_
