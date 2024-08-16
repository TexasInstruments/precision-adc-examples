/*
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

#include "interrupts.h"

//****************************************************************************
//
// Global variables
//
//****************************************************************************
/*****************************************************************************
 *
 * @brief Flag evaluating the state of the USB configuration. Defaults to false
 *      (not configured) at startup and is set true once configured for operation.
 *
 ****************************************************************************/
volatile bool       g_bUSBConfigured    = false;        // Configured or connected?

/*****************************************************************************
 *
 * @brief Event/command flag indicating current device state.
 *
 ****************************************************************************/
volatile uint32_t   g_ui32Flags         = 0;

/*****************************************************************************
 *
 * @brief Variable setting the number of data collections samples required.
 *
 ****************************************************************************/
uint32_t            g_num_samples       = 0;
/*****************************************************************************
 *
 * @brief Variable used to identify if EEPROM or EVM (pullups) is missing.
 *
 ****************************************************************************/
volatile bool       g_bEEPROM    = false;        // Connected or missing EVM?
// TODO: BSL interrupt can be removed as there is currently no way to invoke via hardware
#ifdef BSL_INTERRUPT
//****************************************************************************
//
// GPIO N interrupt handler for BSL button
//
//****************************************************************************
void GPION_IRQHandler(void)
{
    /* Get the interrupt status from the GPIO and clear the status */
    uint32_t getIntStatus = MAP_GPIOIntStatus(BSL_PORT, true);

    if((getIntStatus & BSL_PIN) == BSL_PIN)
    {
        MAP_GPIOIntClear(BSL_PORT, getIntStatus);
        g_ui32Flags |= BOOTLOADER_MODE;
    }
}

#endif

/*****************************************************************************
 *
 * @brief Default USB interrupt handler assigned in the CCS startup file
 *      \a startup_msp432e401y_ccs.c.
 *
 * @details This dummy handler created as a hack for USB interrupt in the
        startup file.  When the USB interrupt has attribute of 'weak'
        and usb.lib is linked to project, the interrupt handler in
        ../device/usbdhandler.c file is routed to default handler.  The
        'weak' attribute only works for dynamic libraries and not a static library
        if a static library were to be created.
 *
 * @return none
 *
 ****************************************************************************/
void USB0_IRQHandler(void)
{
    USB0_IRQDeviceHandler();
}



/*****************************************************************************
 *
 * @brief Sets the control line state for RS232.
 *
 * @param usState
 *
 * @details Set the state of the RS232 RTS and DTR signals.  Handshaking is not
 *      supported for this implementation of CDC on USB so this request will be ignored.
 *
 * @return none
 *
 ****************************************************************************/
static void SetControlLineState(unsigned short usState)
{
}

/*****************************************************************************
 *
 * @brief Set the communication parameters to use on the UART.
 *
 * @param *psLineCoding points to a structure containing parity, number of data bits, baud rate and stop bits.
 *
 * @details Returns a Boolean of the resulting action to set the line coding for RS232 communication.
 *      UART communication on RS232 type devices require baud rate, parity info, number of data bits and
 *      number of stop bits used in the communication. An example: 115k baud, N-8-1.  CDC communication
 *      ignores these values so the function is empty and returns a false (no error) as opposed to true (error).
 *
 * @return 0 for no error
 *
 ****************************************************************************/
static bool SetLineCoding(tLineCoding *psLineCoding)
{
    //
    //user to enter customized data here
    //
    //
    // Let the caller know if we had a problem or not.
    //
    return(0);
}

/*****************************************************************************
 *
 * @brief Get the communication parameters in use on the CDC terminal.
 *
 * @param *psLineCoding points to a structure containing parity, number of data bits, baud rate and stop bits.
 *
 * @details Communication structure of the line coding used for RS232 communication.
 *      UART communication on RS232 type devices require baud rate, parity info, number of data bits and
 *      number of stop bits used in the communication. An example: 115k baud, N-8-1.
 *
 * @return none
 *
 ****************************************************************************/
static void GetLineCoding(tLineCoding *psLineCoding)
{
    // Get the current line coding set for the CDC terminal.
    psLineCoding->ui32Rate = DEFAULT_BIT_RATE;
    psLineCoding->ui8Databits = 8;
    psLineCoding->ui8Parity = USB_CDC_PARITY_NONE;
    psLineCoding->ui8Stop = USB_CDC_STOP_BITS_1;
}


/****************************************************************************
 *
 * @brief Handles CDC driver notifications related to control and setup of the
 * device.
 *
 * @param pvCBData is the client-supplied callback pointer for this channel.
 * @param ui32Event identifies the event we are being notified about.
 * @param ui32MsgValue is an event-specific value.
 * @param pvMsgData is an event-specific pointer.
 *
 * @details This function is called by the CDC driver to perform control-related
 * operations on behalf of the USB host.  These functions include setting
 * and querying the serial communication parameters, setting handshake line
 * states and sending break conditions.
 *
 * @return The return value is event-specific.
 *
 ****************************************************************************/
uint32_t ControlHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData)
{
    // Which event are we being asked to process?
    switch(ui32Event)
    {
        // We are connected to a host and communication is now possible.
        case USB_EVENT_CONNECTED:
        {
            g_bUSBConfigured = true;

            // Turn on LED?

            // Flush our buffers.
            USBBufferFlush(CDC_TX_BUFFER);
            USBBufferFlush(CDC_RX_BUFFER);

            break;
        }

        // The host has disconnected.
        case USB_EVENT_DISCONNECTED:
        {
            g_bUSBConfigured = false;
            break;
        }

        // Return the current serial communication parameters.
        case USBD_CDC_EVENT_GET_LINE_CODING:
        {
            GetLineCoding(pvMsgData);
            break;
        }

        // Set the current serial communication parameters.
        case USBD_CDC_EVENT_SET_LINE_CODING:
        {
            SetLineCoding(pvMsgData);
            break;
        }

        // Set the current serial communication parameters.
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
        {
            SetControlLineState((unsigned short)ui32MsgValue);
            break;
        }

        // Send a break condition on the serial line.
        case USBD_CDC_EVENT_SEND_BREAK:
        {
            break;
        }

        // Clear the break condition on the serial line.
        case USBD_CDC_EVENT_CLEAR_BREAK:
        {
            break;
        }

        // Ignore SUSPEND and RESUME for now.
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
        {
            break;
        }

        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        default:
        {
            break;
        }
    }

    return(0);
}



/*****************************************************************************
 *
 * @brief Handles CDC driver notifications related to the transmit channel (data to
 * the USB host).
 *
 * @param ui32CBData is the client-supplied callback pointer for this channel.
 * @param ui32Event identifies the event we are being notified about.
 * @param ui32MsgValue is an event-specific value.
 * @param pvMsgData is an event-specific pointer.
 *
 * @details This function is called by the CDC driver to notify us of any events
 * related to operation of the transmit data channel (the IN channel carrying
 * data to the USB host).
 *
 * @return The return value is event-specific.
 *
 *****************************************************************************/
uint32_t TxHandlerCmd(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData)
{
    // Which event have we been sent?
    switch(ui32Event)
    {
        case USB_EVENT_TX_COMPLETE:
        {
            // Clear the flag
            g_ui32Flags |= TRANSMIT_COMPLETE;
            break;
        }

        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        default:
        {
            break;
        }
    }
    return(0);
}



/*****************************************************************************
 *
 * @brief Handles CDC driver notifications related to the receive channel (data from
 * the USB host).
 *
 * @param ui32CBData is the client-supplied callback data value for this
 * channel.
 * @param ui32Event identifies the event we are being notified about.
 * @param ui32MsgValue is an event-specific value.
 * @param pvMsgData is an event-specific pointer.
 *
 * @details This function is called by the CDC driver to notify us of any events
 * related to operation of the receive data channel (the OUT channel carrying
 * data from the USB host).
 *
 * @return The return value is event-specific.
 *
 *****************************************************************************/
uint32_t RxHandlerCmd(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData)
{
    switch(ui32Event)
    {
        // A new packet has been received.
        case USB_EVENT_RX_AVAILABLE:
        {
            // Indicate CRC RX event occurred
            g_ui32Flags |= CDC_RX_RECEIVED;
            break;
        }

        //
        // We are being asked how much unprocessed data we have still to
        // process. We return 0 if the UART is currently idle or 1 if it is
        // in the process of transmitting something. The actual number of
        // bytes in the UART FIFO is not important here, merely whether or
        // not everything previously sent to us has been transmitted.
        //
        case USB_EVENT_DATA_REMAINING:
        {
            //
            // Get the number of bytes in the buffer and add 1 if some data
            // still has to clear the transmitter.
            //
            break;
        }

        //
        // We are being asked to provide a buffer into which the next packet
        // can be read. We do not support this mode of receiving data so let
        // the driver know by returning 0. The CDC driver should not be
        // sending this message but this is included just for illustration and
        // completeness.
        //
        case USB_EVENT_REQUEST_BUFFER:
        {
            break;
        }

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
        {
            break;
        }
    }

    return(0);
}


////////////////////////////////////////////////////////
/* The following functions are for BULK communication */
////////////////////////////////////////////////////////


/******************************************************************************
 *
 * @brief Handles bulk driver notifications related to the transmit channel (data to
 * the USB host).
 *
 * @param pvCBData is the client-supplied callback pointer for this channel.
 * @param ulEvent identifies the event we are being notified about.
 * @param ulMsgValue is an event-specific value.
 * @param pvMsgData is an event-specific pointer.
 *
 * @details This function is called by the bulk driver to notify us of any events
 * related to operation of the transmit data channel (the IN channel carrying
 * data to the USB host).
 *
 * @return The return value is event-specific.
 *
 ******************************************************************************/
uint32_t TxHandlerBULK(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData)
{
    //
    // We are not required to do anything in response to any transmit event
    // in this example. All we do is update our transmit counter.
    //
    if (ui32Event == USB_EVENT_TX_COMPLETE)
    {
        //gb_bulkTXcompleted = true; //soliton add
        //g_ui32TxCount += ui32MsgValue;
    }
    return(0);
}

/******************************************************************************
 *
 * @brief Handles bulk driver notifications related to the receive channel (data from
 * the USB host).
 *
 * @param pvCBData is the client-supplied callback pointer for this channel.
 * @param ui32Event identifies the event we are being notified about.
 * @param ui32MsgValue is an event-specific value.
 * @param pvMsgData is an event-specific pointer.
 *
 * @details This function is called by the bulk driver to notify us of any events
 * related to operation of the receive data channel (the OUT channel carrying
 * data from the USB host).
 *
 * @return The return value is event-specific.
 *
 ******************************************************************************/
uint32_t RxHandlerBULK(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue, void *pvMsgData)
{
    // Which event are we being sent?
    switch(ui32Event)
    {
        // We are connected to a host and communication is now possible.
        case USB_EVENT_CONNECTED:
        {
            g_bUSBConfigured = true;
            g_ui32Flags |= COMMAND_STATUS_UPDATE;

            // Flush our buffers.
            USBBufferFlush(BULK_TX_BUFFER);
            USBBufferFlush(BULK_RX_BUFFER);

            break;
        }

        // The host has disconnected.
        case USB_EVENT_DISCONNECTED:
        {
            //g_bUSBConfigured = false;
            g_ui32Flags |= COMMAND_STATUS_UPDATE;
            break;
        }

        // A new packet has been received.
        case USB_EVENT_RX_AVAILABLE:
        {
#if 0       // NOTE: This implementation will not receive BULK data from the host!

            tUSBDBulkDevice *psDevice;

            //
            // Get a pointer to our instance data from the callback data
            // parameter.
            //
            psDevice = (tUSBDBulkDevice *)pvCBData;

            //
            // Read the new packet and echo it back to the host.
            //
            return(EchoNewDataToHost(psDevice, pvMsgData, ui32MsgValue));
#else
            break;
#endif
        }

        // Ignore SUSPEND and RESUME for now.
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        // Ignore all other events and return 0.
        default:
            break;
    }

    return(0);
}
