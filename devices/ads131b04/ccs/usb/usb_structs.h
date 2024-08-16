//*****************************************************************************
//
// usb_serial_structs.h - Data structures defining this Composite USB device.
//
// Copyright (c) 2009-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
//
//*****************************************************************************

#ifndef __USB_SERIAL_STRUCTS_H__
#define __USB_SERIAL_STRUCTS_H__

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


#endif // __USB_SERIAL_STRUCTS_H__
