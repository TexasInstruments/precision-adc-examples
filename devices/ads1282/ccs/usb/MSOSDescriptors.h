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

//*****************************************************************************
//
// MSOSDescriptors.h - Definitions for MS OS descriptors
//
//    based on : Microsoft OS 1.0 & 2.0 Descriptors Specification
//    https://msdn.microsoft.com/en-us/windows/hardware/gg463179
//    
//*****************************************************************************

#ifndef USB_MSOSDESCRIPTORS_H_
#define USB_MSOSDESCRIPTORS_H_

#ifdef __cplusplus
extern "C"
{
#endif


#include <stdbool.h>
#include <stdint.h>

#include "ti/usblib/msp432e4/usblib.h"
#include "ti/usblib/msp432e4/device/usbdevice.h"
#include "settings.h"


//
// Microsoft OS String Descriptor 1.0
//

#ifdef USE_MS_OS_10
#define USB_MS_OS_STRING_DESC_IDX         0xEE
#endif

#ifdef USE_MS_OS_20
#define USB_MS_OS_STRING_DESC_IDX         0x0F00
#endif

#define USB_MS_OS_STRING_DESC_SIGNATURE { \
    'M', 0, 'S', 0, 'F', 0, 'T', 0, '1', 0, '0', 0, '0', 0 }

typedef struct _USB_MS_OS_STRING_DESCRIPTOR
{
    uint8_t  bLength;               // Length of the descriptor
    uint8_t  bDescriptorType;       // Descriptor type
    uint8_t  qwSignature[14];       // Signature field
    uint8_t  bMS_VendorCode;        // Vendor code
    uint8_t  bPad;                  // Pad field
} PACKED tUSB_MS_OS_STRING_DESCRIPTOR;

#define USB_MS_OS_STRING_DESCRIPTOR( vendor_code ) { \
    .bLength         = sizeof(tUSB_MS_OS_STRING_DESCRIPTOR), \
    .bDescriptorType = USB_DTYPE_STRING, \
    .qwSignature     = USB_MS_OS_STRING_DESC_SIGNATURE, \
    .bMS_VendorCode  = USB_MS_OS_VENDOR_CODE, \
    .bPad            = 0x00 \
}

//
// Microsoft Extended Compat ID OS Feature Descriptor
//
#define USB_MS_EXTENDED_COMPAT_ID_VERSION 0x0100
#define USB_MS_EXTENDED_COMPAT_ID_TYPE    0x04

#define USB_COMPATID_NONE                {0}
#define USB_SUBCOMPATID_NONE             {0}
#define USB_COMPATID_WINUSB              "WINUSB\0"
#define USB_COMPATID_RNDIS               "RNDIS\0\0"
#define USB_COMPATID_PTP                 "PTP\0\0\0\0"
#define USB_COMPATID_MTP                 "MTP\0\0\0\0"
#define USB_COMPATID_BLUETOOTH           "BLUTUTH"
#define USB_SUBCOMPATID_BT_V11           "11\0\0\0\0\0"
#define USB_SUBCOMPATID_BT_V12           "12\0\0\0\0\0"
#define USB_SUBCOMPATID_BT_V20EDR        "EDR\0\0\0\0"

//
// Microsoft OS String Descriptor 2.0
//
#define REQUEST_GET_MS_DESCRIPTOR    0x20
//#define MS_OS_20_LENGTH 0xB2 // Adjust for exact length of msOs20DescriptorSet

#define USB_DESCRIPTOR_TYPE_BOS 15
#define USB_DESCRIPTOR_TYPE_DEVICE_CAPABILITY 16

// Microsoft OS 2.0 Descriptors, Table 1
#define USB_DEVICE_CAPABILITY_TYPE_PLATFORM 5

// Microsoft OS 2.0 Descriptors, Table 8
#define MS_OS_20_DESCRIPTOR_INDEX 7
#define MS_OS_20_SET_ALT_ENUMERATION 8

// Microsoft OS 2.0 Descriptors, Table 9
#define MS_OS_20_SET_HEADER_DESCRIPTOR 0x00
#define MS_OS_20_SUBSET_HEADER_CONFIGURATION 0x01
#define MS_OS_20_SUBSET_HEADER_FUNCTION 0x02
#define MS_OS_20_FEATURE_COMPATIBLE_ID 0x03
#define MS_OS_20_FEATURE_REG_PROPERTY 0x04
#define MS_OS_20_FEATURE_MIN_RESUME_TIME 0x05
#define MS_OS_20_FEATURE_MODEL_ID 0x06
#define MS_OS_20_FEATURE_CCGP_DEVICE 0x07



typedef struct _MS_EXTENDED_COMPAT_ID_HEADER
{
    uint32_t dwLength;              // length of the complete descriptors set
    uint16_t bcdVersion;            // descriptor's version number (BCD: 0x0100)
    uint16_t wIndex;                // type number of extended compat descriptor (0x04)
    uint8_t  bCount;                // number of custom property sections
    uint8_t  rgbReserved[7];        // reserved
} PACKED tMS_EXTENDED_COMPAT_ID_HEADER;

#define MS_EXTENDED_COMPAT_ID_HEADER( size, count ) { \
    .dwLength       = size, \
    .bcdVersion     = USB_MS_EXTENDED_COMPAT_ID_VERSION, \
    .wIndex         = USB_MS_EXTENDED_COMPAT_ID_TYPE, \
    .bCount         = count, \
    .rgbReserved = {0} \
}

typedef struct _MS_EXTENDED_COMPAT_ID_FUNCTION
{
    uint8_t  bFirstInterfaceNumber; // interface or function number
    uint8_t  bReserved;             // reserved
    uint8_t  compatibleID[8];       // function's compatible ID
    uint8_t  subCompatibleID[8];    // function's subcompatible ID
    uint8_t  rgbReserved[6];        // reserved
} PACKED tMS_EXTENDED_COMPAT_ID_FUNCTION;

#define MS_EXTENDED_COMPAT_ID_FUNCTION( interfaceNo, id, subID ) { \
    .bFirstInterfaceNumber = interfaceNo, \
    .bReserved             = 0x01, \
    .compatibleID          = id, \
    .subCompatibleID       = subID, \
    .rgbReserved           = {0} \
}

//
// Microsoft Extended Properties OS Feature Descriptor
//
#define USB_MS_EXTENDED_PROPERTY_VERSION  0x0100
#define USB_MS_EXTENDED_PROPERTY_TYPE     0x05

#define USB_EX_PROPERTY_REG_SZ            0x00000001  // NULL-terminated Unicode String
#define USB_EX_PROPERTY_REG_EXPAND_SZ     0x00000002  // NULL-terminated Unicode String (environment var)
#define USB_EX_PROPERTY_REG_BINARY        0x00000003  // Binary
#define USB_EX_PROPERTY_REG_DWORD_LE      0x00000004  // little-endian 32-bit integer
#define USB_EX_PROPERTY_REG_DWORD_BE      0x00000005  // big-endian 32-bit integer
#define USB_EX_PROPERTY_REG_LINK          0x00000006  // NULL-terminated Unicode String (symbolic link)
#define USB_EX_PROPERTY_REG_MULTI_SZ      0x00000007  // Multi NULL-terminated Unicode String

#define USB_EX_PROPERTY_DATA_LEN_DWORD            4

typedef struct _MS_EXTENDED_PROPERTY_HEADER
{
    uint32_t dwLength;              // length of the complete descriptors set
    uint16_t bcdVersion;            // descriptor's version number (BCD: 0x0100)
    uint16_t wIndex;                // type number of extended property desc (0x05)
    uint16_t wCount;                // number of custom property sections
} PACKED tMS_EXTENDED_PROPERTY_HEADER;

#define MS_EXTENDED_PROPERTY_HEADER( size, count ) { \
    .dwLength   = size, \
    .bcdVersion = USB_MS_EXTENDED_PROPERTY_VERSION, \
    .wIndex     = USB_MS_EXTENDED_PROPERTY_TYPE, \
    .wCount     = count \
}

// SelectiveSuspendEnabled - for HID device

#define USB_EX_PROPERTY_NAME_LENGTH_SELECTIVESUSPEND   0x30

typedef struct _MS_EXTENDED_PROPERTY_SELECTIVESUSPEND
{
    uint32_t dwSize;                // size of this section
    uint32_t dwPropertyDataType;    // property data format
    uint16_t wPropertyNameLength;   // property name length
    uint16_t bPropertyName[USB_EX_PROPERTY_NAME_LENGTH_SELECTIVESUSPEND / sizeof(uint16_t)];   // property name
    uint32_t dwPropertyDataLength;  // length of property data
    uint32_t bPropertyData;         // property data
} PACKED tMS_EXTENDED_PROPERTY_SELECTIVESUSPEND;

#define MS_EXTENDED_PROPERTY_SELECTIVESUSPEND( sw ) { \
    .dwSize               = sizeof( tMS_EXTENDED_PROPERTY_SELECTIVESUSPEND ), \
    .dwPropertyDataType   = USB_EX_PROPERTY_REG_DWORD_LE, \
    .wPropertyNameLength  = USB_EX_PROPERTY_NAME_LENGTH_SELECTIVESUSPEND, \
    .bPropertyName        = L"SelectiveSuspendEnabled", \
    .dwPropertyDataLength = USB_EX_PROPERTY_DATA_LEN_DWORD, \
    .bPropertyData        = sw \
}

// DeviceInterfaceGUID - interface GUID for WinUSB

#define USB_EX_PROPERTY_NAME_LENGTH_IFGUID   0x0028
#define USB_EX_PROPERTY_DATA_LEN_IFGUID      0x0000004E

typedef struct _MS_EXTENDED_PROPERTY_INTERFACE_GUID
{
    uint32_t dwSize;                // size of this section
    uint32_t dwPropertyDataType;    // property data format
    uint16_t wPropertyNameLength;   // property name length
    uint16_t bPropertyName[USB_EX_PROPERTY_NAME_LENGTH_IFGUID / sizeof(uint16_t)];   // property name
    uint32_t dwPropertyDataLength;  // length of property data
    uint16_t bPropertyData[USB_EX_PROPERTY_DATA_LEN_IFGUID / sizeof(uint16_t)];      // property data
} PACKED tMS_EXTENDED_PROPERTY_INTERFACE_GUID;

#define MS_EXTENDED_PROPERTY_INTERFACE_GUID( guid ) { \
    .dwSize               = sizeof( tMS_EXTENDED_PROPERTY_INTERFACE_GUID ), \
    .dwPropertyDataType   = USB_EX_PROPERTY_REG_SZ, \
    .wPropertyNameLength  = USB_EX_PROPERTY_NAME_LENGTH_IFGUID, \
    .bPropertyName        = L"DeviceInterfaceGUID", \
    .dwPropertyDataLength = USB_EX_PROPERTY_DATA_LEN_IFGUID, \
    .bPropertyData        = guid \
}

// WinUSB Power Management
// https://msdn.microsoft.com/en-us/library/windows/hardware/ff728834(v=vs.85).aspx

// SystemWakeEnabled - device should be allowed to wake the system from a low power state.

#define USB_EX_PROPERTY_NAME_LENGTH_WAKEENABLED   36

typedef struct _MS_EXTENDED_PROPERTY_WAKEENABLED
{
    uint32_t dwSize;                // size of this section
    uint32_t dwPropertyDataType;    // property data format
    uint16_t wPropertyNameLength;   // property name length
    uint16_t bPropertyName[USB_EX_PROPERTY_NAME_LENGTH_WAKEENABLED / sizeof(uint16_t)];   // property name
    uint32_t dwPropertyDataLength;  // length of property data
    uint32_t bPropertyData;         // property data
} PACKED tMS_EXTENDED_PROPERTY_WAKEENABLED;

#define MS_EXTENDED_PROPERTY_WAKEENABLED( sw ) { \
    .dwSize               = sizeof( tMS_EXTENDED_PROPERTY_WAKEENABLED ), \
    .dwPropertyDataType   = USB_EX_PROPERTY_REG_DWORD_LE, \
    .wPropertyNameLength  = USB_EX_PROPERTY_NAME_LENGTH_WAKEENABLED, \
    .bPropertyName        = L"SystemWakeEnabled", \
    .dwPropertyDataLength = USB_EX_PROPERTY_DATA_LEN_DWORD, \
    .bPropertyData        = sw \
}

// DeviceIdleEnabled - device is capable of being powered down when idle (Selective Suspend)

#define USB_EX_PROPERTY_NAME_LENGTH_IDLEENABLED   36
#define USB_EX_PROPERTY_DATA_LEN_DWORD            4

typedef struct _MS_EXTENDED_PROPERTY_IDLEENABLED
{
    uint32_t dwSize;                // size of this section
    uint32_t dwPropertyDataType;    // property data format
    uint16_t wPropertyNameLength;   // property name length
    uint16_t bPropertyName[USB_EX_PROPERTY_NAME_LENGTH_IDLEENABLED / sizeof(uint16_t)];   // property name
    uint32_t dwPropertyDataLength;  // length of property data
    uint32_t bPropertyData;         // property data
} PACKED tMS_EXTENDED_PROPERTY_IDLEENABLED;

#define MS_EXTENDED_PROPERTY_IDLEENABLED( sw ) { \
    .dwSize               = sizeof( tMS_EXTENDED_PROPERTY_IDLEENABLED ), \
    .dwPropertyDataType   = USB_EX_PROPERTY_REG_DWORD_LE, \
    .wPropertyNameLength  = USB_EX_PROPERTY_NAME_LENGTH_IDLEENABLED, \
    .bPropertyName        = L"DeviceIdleEnabled", \
    .dwPropertyDataLength = USB_EX_PROPERTY_DATA_LEN_DWORD, \
    .bPropertyData        = sw \
}

// DeviceIdleIgnoreWakeEnable - suspends the device even if it does not support RemoteWake

#define USB_EX_PROPERTY_NAME_LENGTH_IGNOREWAKE   54

typedef struct _MS_EXTENDED_PROPERTY_IGNOREWAKE
{
    uint32_t dwSize;                // size of this section
    uint32_t dwPropertyDataType;    // property data format
    uint16_t wPropertyNameLength;   // property name length
    uint16_t bPropertyName[USB_EX_PROPERTY_NAME_LENGTH_IGNOREWAKE / sizeof(uint16_t)];   // property name
    uint32_t dwPropertyDataLength;  // length of property data
    uint32_t bPropertyData;         // property data
} PACKED tMS_EXTENDED_PROPERTY_IGNOREWAKE;

#define MS_EXTENDED_PROPERTY_IGNOREWAKE( sw ) { \
    .dwSize               = sizeof( tMS_EXTENDED_PROPERTY_IGNOREWAKE ), \
    .dwPropertyDataType   = USB_EX_PROPERTY_REG_DWORD_LE, \
    .wPropertyNameLength  = USB_EX_PROPERTY_NAME_LENGTH_IGNOREWAKE, \
    .bPropertyName        = L"DeviceIdleIgnoreWakeEnable", \
    .dwPropertyDataLength = USB_EX_PROPERTY_DATA_LEN_DWORD, \
    .bPropertyData        = sw \
}

// UserSetDeviceIdleEnabled - check box should be enabled in the device Properties page that allows a user to override the idle defaults

#define USB_EX_PROPERTY_NAME_LENGTH_SETIDLEEN   50

typedef struct _MS_EXTENDED_PROPERTY_SETIDLEEN
{
    uint32_t dwSize;                // size of this section
    uint32_t dwPropertyDataType;    // property data format
    uint16_t wPropertyNameLength;   // property name length
    uint16_t bPropertyName[USB_EX_PROPERTY_NAME_LENGTH_SETIDLEEN / sizeof(uint16_t)];   // property name
    uint32_t dwPropertyDataLength;  // length of property data
    uint32_t bPropertyData;         // property data
} PACKED tMS_EXTENDED_PROPERTY_SETIDLEEN;

#define MS_EXTENDED_PROPERTY_SETIDLEEN( sw ) { \
    .dwSize               = sizeof( tMS_EXTENDED_PROPERTY_SETIDLEEN ), \
    .dwPropertyDataType   = USB_EX_PROPERTY_REG_DWORD_LE, \
    .wPropertyNameLength  = USB_EX_PROPERTY_NAME_LENGTH_SETIDLEEN, \
    .bPropertyName        = L"UserSetDeviceIdleEnabled", \
    .dwPropertyDataLength = USB_EX_PROPERTY_DATA_LEN_DWORD, \
    .bPropertyData        = sw \
}

// DefaultIdleState - default value of the AUTO_SUSPEND power policy setting

#define USB_EX_PROPERTY_NAME_LENGTH_DEFAULTIDLESTATE   34

typedef struct _MS_EXTENDED_PROPERTY_DEFAULTIDLESTATE
{
    uint32_t dwSize;                // size of this section
    uint32_t dwPropertyDataType;    // property data format
    uint16_t wPropertyNameLength;   // property name length
    uint16_t bPropertyName[USB_EX_PROPERTY_NAME_LENGTH_DEFAULTIDLESTATE / sizeof(uint16_t)];   // property name
    uint32_t dwPropertyDataLength;  // length of property data
    uint32_t bPropertyData;         // property data
} PACKED tMS_EXTENDED_PROPERTY_DEFAULTIDLESTATE;

#define MS_EXTENDED_PROPERTY_DEFAULTIDLESTATE( sw ) { \
    .dwSize               = sizeof( tMS_EXTENDED_PROPERTY_DEFAULTIDLESTATE ), \
    .dwPropertyDataType   = USB_EX_PROPERTY_REG_DWORD_LE, \
    .wPropertyNameLength  = USB_EX_PROPERTY_NAME_LENGTH_DEFAULTIDLESTATE, \
    .bPropertyName        = L"DefaultIdleState", \
    .dwPropertyDataLength = USB_EX_PROPERTY_DATA_LEN_DWORD, \
    .bPropertyData        = sw \
}

// DefaultIdleTimeout - sets the default state of the SUSPEND_DELAY power policy setting (milli-seconds).

#define USB_EX_PROPERTY_NAME_LENGTH_DEFAULTIDLETIMEOUT   38

typedef struct _MS_EXTENDED_PROPERTY_DEFAULTIDLETIMEOUT
{
    uint32_t dwSize;                // size of this section
    uint32_t dwPropertyDataType;    // property data format
    uint16_t wPropertyNameLength;   // property name length
    uint16_t bPropertyName[USB_EX_PROPERTY_NAME_LENGTH_DEFAULTIDLETIMEOUT / sizeof(uint16_t)];   // property name
    uint32_t dwPropertyDataLength;  // length of property data
    uint32_t bPropertyData;         // property data
} PACKED tMS_EXTENDED_PROPERTY_DEFAULTIDLETIMEOUT;

#define MS_EXTENDED_PROPERTY_DEFAULTIDLETIMEOUT( ms ) { \
    .dwSize               = sizeof( tMS_EXTENDED_PROPERTY_DEFAULTIDLETIMEOUT ), \
    .dwPropertyDataType   = USB_EX_PROPERTY_REG_DWORD_LE, \
    .wPropertyNameLength  = USB_EX_PROPERTY_NAME_LENGTH_DEFAULTIDLETIMEOUT, \
    .bPropertyName        = L"DefaultIdleTimeout", \
    .dwPropertyDataLength = USB_EX_PROPERTY_DATA_LEN_DWORD, \
    .bPropertyData        = ms \
}

////BOS stuff


#ifdef USE_COMPOSITE
#define MS_OS_20_LENGTH 0xB2


// Micrsoft OS 2.0 Descriptor Set for a composite device.
uint8_t g_USBmsOs20DescriptorSet[MS_OS_20_LENGTH] =
{
    // Microsoft OS 2.0 Descriptor Set header (Table 10)
    0x0A, 0x00,  // wLength
    MS_OS_20_SET_HEADER_DESCRIPTOR, 0x00,
    0x00, 0x00, 0x03, 0x06,  // dwWindowsVersion: Windows 8.1 (NTDDI_WINBLUE)
    MS_OS_20_LENGTH, 0x00,  // wTotalLength

//  // Microsoft OS 2.0 registry property descriptor (Table 14)
//     0x20, 0x00,   // wLength
//     MS_OS_20_FEATURE_REG_PROPERTY, 0x00,
//     0x07, 0x00,   // wPropertyDataType: REG_MULTI_SZ
//     0x0C, 0x00,   // wPropertyNameLength
//     'H',0,'a',0,'p',0,'p',0,'y',0,0,0,
//     0x0A, 0x00,   // wPropertyDataLength
//     '3',0,0,0,'5',0,0,0,0,0,


    // Microsoft OS 2.0 configuration subset (Table 11)
    0x08, 0x00,  // wLength of this header
    MS_OS_20_SUBSET_HEADER_CONFIGURATION, 0x00,  // wDescriptorType
    0,            // configuration index
    0x00,         // bReserved
    0xA8, 0x00,   // wTotalLength of this subset

    // Microsoft OS 2.0 function subset header (Table 12)
    0x08, 0x00,  // wLength
    MS_OS_20_SUBSET_HEADER_FUNCTION, 0x00,  // wDescriptorType
    0,           // bFirstInterface
    0x00,        // bReserved,
    0xA0, 0x00,  // wSubsetLength

    // Microsoft OS 2.0 compatible ID descriptor (Table 13)
    0x14, 0x00,                                      // wLength
    MS_OS_20_FEATURE_COMPATIBLE_ID, 0x00,            // wDescriptorType
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,        // compatibleID
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // subCompatibleID

    // Microsoft OS 2.0 registry property descriptor (Table 14)
    0x84, 0x00,   // wLength
    MS_OS_20_FEATURE_REG_PROPERTY, 0x00,
    0x07, 0x00,   // wPropertyDataType: REG_MULTI_SZ
    0x2a, 0x00,   // wPropertyNameLength
    'D',0,'e',0,'v',0,'i',0,'c',0,'e',0,'I',0,'n',0,'t',0,'e',0,'r',0,
    'f',0,'a',0,'c',0,'e',0,'G',0,'U',0,'I',0,'D',0,'s',0,0,0,
    0x50, 0x00,   // wPropertyDataLength
    '{',0,'6',0,'E',0,'4',0,'5',0,'7',0,'3',0,'6',0,'A',0,'-',0,
    '2',0,'B',0,'1',0,'B',0,'-',0,'4',0,'0',0,'7',0,'8',0,'-',0,
    'B',0,'7',0,'7',0,'2',0,'-',0,'B',0,'3',0,'A',0,'F',0,'2',0,
    'B',0,'6',0,'F',0,'D',0,'E',0,'1',0,'C',0,'}',0,0,0,0,0,
};

#else
#define MS_OS_20_LENGTH 0xA2

// Micrsoft OS 2.0 Descriptor Set for a non-composite device.
uint8_t g_USBmsOs20DescriptorSet[MS_OS_20_LENGTH] =
{
    // Microsoft OS 2.0 Descriptor Set header (Table 10)
    0x0A, 0x00,  // wLength
    MS_OS_20_SET_HEADER_DESCRIPTOR, 0x00,
    0x00, 0x00, 0x03, 0x06,  // dwWindowsVersion: Windows 8.1 (NTDDI_WINBLUE)
    MS_OS_20_LENGTH, 0x00,  // wTotalLength

    // Microsoft OS 2.0 compatible ID descriptor (Table 13)
    0x14, 0x00,                                      // wLength
    MS_OS_20_FEATURE_COMPATIBLE_ID, 0x00,            // wDescriptorType
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,        // compatibleID
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // subCompatibleID

    // Microsoft OS 2.0 registry property descriptor (Table 14)
    0x84, 0x00,   // wLength
    MS_OS_20_FEATURE_REG_PROPERTY, 0x00,
    0x07, 0x00,   // wPropertyDataType: REG_MULTI_SZ
    0x2a, 0x00,   // wPropertyNameLength
    'D',0,'e',0,'v',0,'i',0,'c',0,'e',0,'I',0,'n',0,'t',0,'e',0,'r',0,
    'f',0,'a',0,'c',0,'e',0,'G',0,'U',0,'I',0,'D',0,'s',0,0,0,
    0x50, 0x00,   // wPropertyDataLength
    '{',0,'6',0,'E',0,'4',0,'5',0,'7',0,'3',0,'6',0,'A',0,'-',0,
    '2',0,'B',0,'1',0,'B',0,'-',0,'4',0,'0',0,'7',0,'8',0,'-',0,
    'B',0,'7',0,'7',0,'2',0,'-',0,'B',0,'3',0,'A',0,'F',0,'2',0,
    'B',0,'6',0,'F',0,'D',0,'E',0,'1',0,'C',0,'}',0,0,0,0,0,
};
#endif

uint8_t	g_USBbosDescriptor[0x21] =
{
    0x05,       // bLength of this descriptor
    USB_DESCRIPTOR_TYPE_BOS,
    0x21, 0x00, // wLength
    0x01,       // bNumDeviceCaps
	0x1C,       // bLength of this first device capability descriptor
    USB_DESCRIPTOR_TYPE_DEVICE_CAPABILITY,
    USB_DEVICE_CAPABILITY_TYPE_PLATFORM,
    0x00,       // bReserved
    // Microsoft OS 2.0 descriptor platform capability UUID
    // from Microsoft OS 2.0 Descriptors,  Table 3.
    0xDF, 0x60, 0xDD, 0xD8, 0x89, 0x45, 0xC7, 0x4C,
    0x9C, 0xD2, 0x65, 0x9D, 0x9E, 0x64, 0x8A, 0x9F,
	0x00, 0x00, 0x03, 0x06,  // dwWindowsVersion: Windows 8.1 (NTDDI_WINBLUE)
    MS_OS_20_LENGTH, 0x00,    // wMSOSDescriptorSetTotalLength
    REQUEST_GET_MS_DESCRIPTOR,
    0,                       // bAltEnumCode
};


/////END OF BOS STUFF



#ifdef __cplusplus
}
#endif

#endif // USB_MSOSDESCRIPTORS_H_
