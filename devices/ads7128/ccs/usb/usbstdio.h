/*
 * usbstdio.h
 *
 *  Created on: Mar 20, 2019
 *      Author: a0282860
 */

#ifndef LIB_USBSTDIO_H_
#define LIB_USBSTDIO_H_

#include <stdarg.h>
#include <stdint.h>

#include "ti/devices/msp432e4/driverlib/driverlib.h"
#include "ti/usblib/msp432e4/usblib.h"
#include "ti/usblib/msp432e4/usbcdc.h"
#include "ti/usblib/msp432e4/usb-ids.h"
#include "ti/usblib/msp432e4/device/usbdevice.h"
//#include "ti/usblib/msp432e4/device/usbdcdc.h"
#include "usb_structs.h"

void USBprintf(const char *pcString, ...);
void USBvprintf(const char *pcString, va_list vaArgP);


#endif /* LIB_USBSTDIO_H_ */
