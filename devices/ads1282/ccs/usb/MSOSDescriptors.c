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
//
// MS OS Descriptors
//

#include "MSOSDescriptors.h"


//
// Macros
//
#ifndef min
    #define min(a, b)               (((a) < (b)) ? (a) : (b))
#endif


enum {
    USB_BULK_INTERFACE_NUM,
    USB_CDC_INTERFACE_NUM,
    USB_NUM_INTERFACES       // number of interfaces
};


//#define USB_WINUSB_INTERFACE_NUM        (0x00)
#define USB_MS_OS_VENDOR_CODE             (0x01)
#define USB_TI_USB_DEV_BULK_IF_GUID       L"{6E45736A-2B1B-4078-B772-B3AF2B6FDE1C}"
#define USB_TI_USB_CDC_IF_GUID            L"{D17C772B-AF45-4041-9979-AAFE96BF6398}"

// Microsoft OS String Descriptor
tUSB_MS_OS_STRING_DESCRIPTOR g_USBMsftOSStringDesc = 
    USB_MS_OS_STRING_DESCRIPTOR( USB_MS_OS_VENDOR_CODE );


// MS Extended Compat ID OS Feature Descriptor

//#define MS_EXTENDED_COMPAT_FUNCTION_NUM    1

typedef struct _MS_EXTENDED_COMPAT_ID_DESCRIPTOR
{
    tMS_EXTENDED_COMPAT_ID_HEADER   header;
    tMS_EXTENDED_COMPAT_ID_FUNCTION function[ USB_NUM_INTERFACES ];
} PACKED tMS_EXTENDED_COMPAT_ID_DESCRIPTOR;

#ifdef USE_MS_OS_10 
static tMS_EXTENDED_COMPAT_ID_DESCRIPTOR g_USBExtendedCompatIDDesc = 
{
    .header   = MS_EXTENDED_COMPAT_ID_HEADER( sizeof( tMS_EXTENDED_COMPAT_ID_DESCRIPTOR ), USB_NUM_INTERFACES ),
	.function = { MS_EXTENDED_COMPAT_ID_FUNCTION( USB_BULK_INTERFACE_NUM, USB_COMPATID_WINUSB, USB_SUBCOMPATID_NONE ),
	    MS_EXTENDED_COMPAT_ID_FUNCTION( USB_CDC_INTERFACE_NUM,  USB_COMPATID_NONE, USB_SUBCOMPATID_NONE )
	}
};

static tMS_EXTENDED_PROPERTY_DESCRIPTOR g_USBExtendedPropIfGUIDDesc0 = 
{
    .header    = MS_EXTENDED_PROPERTY_HEADER( sizeof( tMS_EXTENDED_PROPERTY_DESCRIPTOR ), 0x0001 ),
    .property0 = MS_EXTENDED_PROPERTY_INTERFACE_GUID( USB_TI_USB_DEV_BULK_IF_GUID )
};

// static tMS_EXTENDED_PROPERTY_DESCRIPTOR g_USBExtendedPropIfGUIDDesc1 = 
// {
//     .header    = MS_EXTENDED_PROPERTY_HEADER( sizeof( tMS_EXTENDED_PROPERTY_DESCRIPTOR ), 0x0001 ),
//     .property0 = MS_EXTENDED_PROPERTY_INTERFACE_GUID( USB_TI_USB_CDC_IF_GUID )
// };
#endif


// MS Extended Property OS Feature Descriptor
typedef struct _MS_EXTENDED_PROPERTY_DESCRIPTOR
{
    tMS_EXTENDED_PROPERTY_HEADER         header;
    tMS_EXTENDED_PROPERTY_INTERFACE_GUID property0;
} PACKED tMS_EXTENDED_PROPERTY_DESCRIPTOR;




bool MSOSdescHandleStringRequest( tUSBRequest * psUSBRequest, uint8_t ** ppui8EP0Data, volatile uint32_t * pui32EP0DataRemain )
{
#ifdef USE_MS_OS_10
    if ( USB_MS_OS_STRING_DESC_IDX == (psUSBRequest->wValue & 0xFF) )
    {
        *ppui8EP0Data       = (uint8_t *)&g_USBMsftOSStringDesc;
        *pui32EP0DataRemain = sizeof( tUSB_MS_OS_STRING_DESCRIPTOR );
        return true;
    }
#endif

#ifdef USE_MS_OS_20
    if ( USB_MS_OS_STRING_DESC_IDX == (psUSBRequest->wValue & 0x0f00) )
    {
        *ppui8EP0Data       = (uint8_t *)&g_USBbosDescriptor;
        *pui32EP0DataRemain = sizeof(g_USBbosDescriptor);
        return true;
    }
#endif

    return false;
}


bool MSOSdescHandleVendorRequests(tUSBRequest *pUSBRequest)
{
    uint32_t len;
    
#ifdef USE_MS_OS_10

    if ( (pUSBRequest->bmRequestType == (USB_RTYPE_DIR_IN | USB_RTYPE_VENDOR) )
      && (pUSBRequest->bRequest      == USB_MS_OS_VENDOR_CODE)
//      && (pUSBRequest->wValue        == USB_WINUSB_INTERFACE_NUM)  // should be ignored
      && (pUSBRequest->wIndex        == USB_MS_EXTENDED_COMPAT_ID_TYPE)
       ) {
        len = min( pUSBRequest->wLength, sizeof(g_USBExtendedCompatIDDesc) );
        USBDCDSendDataEP0( 0, (uint8_t *)&g_USBExtendedCompatIDDesc, len );
        return true;
    }

   
    if ( ( (pUSBRequest->bmRequestType == (USB_RTYPE_DIR_IN | USB_RTYPE_VENDOR | USB_RTYPE_INTERFACE) ) 
        || (pUSBRequest->bmRequestType == (USB_RTYPE_DIR_IN | USB_RTYPE_VENDOR) ) ) // workaround for Renesus USB3.0 driver
      && (pUSBRequest->bRequest      == USB_MS_OS_VENDOR_CODE)
      && (pUSBRequest->wIndex        == USB_MS_EXTENDED_PROPERTY_TYPE) 
       ) {/*
           if( pUSBRequest->wValue      == USB_BULK_INTERFACE_NUM ) {
            len = min( pUSBRequest->wLength, sizeof(g_USBExtendedPropIfGUIDDesc0) );
            USBDCDSendDataEP0( 0, (uint8_t *)& g_USBExtendedPropIfGUIDDesc0, len );
            return true;
           }
           if( pUSBRequest->wValue      == USB_CDC_INTERFACE_NUM ) {
            len = min( pUSBRequest->wLength, sizeof(g_USBExtendedPropIfGUIDDesc1) );
            USBDCDSendDataEP0( 0, (uint8_t *)& g_USBExtendedPropIfGUIDDesc1, len );
            return true;
           }*/
           // this above is incorrect as the extended functions are never called for Bulk or CDC 
           // below doesnt work either
            len = min( pUSBRequest->wLength, sizeof(g_USBExtendedPropIfGUIDDesc0) );
            USBDCDSendDataEP0( 0, (uint8_t *)& g_USBExtendedPropIfGUIDDesc0, len );
            return true;
       
    }

#endif

#ifdef USE_MS_OS_20
    if ( (pUSBRequest->bmRequestType == (USB_RTYPE_DIR_IN | USB_RTYPE_VENDOR) )
    && (pUSBRequest->wIndex        == MS_OS_20_DESCRIPTOR_INDEX)
       ) {
          len = min( pUSBRequest->wLength, sizeof(g_USBmsOs20DescriptorSet) );
            USBDCDSendDataEP0( 0, (uint8_t *)& g_USBmsOs20DescriptorSet, len );
            return true; 
       }

#endif

    return false;
    // STALL, if no one processes this request
    //USBDCDStallEP0(0);
}
