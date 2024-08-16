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
#ifndef SETTINGS_H_
#define SETTINGS_H_

// Standard libraries
#include <stdbool.h>

// Custom libraries
#include "ti/devices/msp432e4/driverlib/driverlib.h"
#include "ti/devices/msp432e4/driverlib/inc/hw_gpio.h"
#include "ti/usblib/msp432e4/usblib.h"
#include "ti/usblib/msp432e4/usbcdc.h"
#include "usb/usb_structs.h"

/* Driver Header files */
#include "ti/drivers/GPIO.h"
#include "ti_drivers_config.h"


/******************************************************************************
 *
 *  @brief A type definition used to identify errors using a value of unsigned 32-bit.
 *
 *****************************************************************************/
typedef uint32_t   ERRCODE;

//*****************************************************************************
//
// Button Configuration Options
//
//*****************************************************************************
// TODO: Not used...SW2 not populated on PAMB
// Enable this macro to call JumpToBootLoader() when the BSL button is pressed.
//#define BSL_INTERRUPT

// TODO: Add option to change between PAMB and LP pinout
//#define LAUNCHPAD_PINOUT

//*****************************************************************************
//
// Definitions for calling code segments (Primary usage in HAL)
//
//*****************************************************************************
//#define USE_SAR
#define USE_DS
//#define USE_UART
//#define USE_I2C
#define USE_I2C_PULLUP
#define USE_SPI
//#define USE_ALERT
//#define USE_LOCK
//#define USE_RESET
//#define USE_SYNC
//#define USE_SLEEP
//#define USE_INTEGRITY
//#define USE_EXTCLK
#define USE_SYSCONFIG
#define USE_CS

//*****************************************************************************
//
// USB Configuration Options
//
//*****************************************************************************

// TODO: Test all of the permutations of these configurations ...
/******************************************************************************
 *
 *  @brief A macro used to set configuration for MSP432E to use the ULPI peripheral for high-speed USB 2.0.
 *
 *  @note When disabled, code will configure to full-speed USB peripheral.
 *
 *****************************************************************************/
#define USE_ULPI

/******************************************************************************
 *
 *  @brief A macro used to enable the Microsoft OS 2.0 Descriptor for automatic (driver-less) enumeration.
 *
 *  @note When disabled, auto enumeration will not take place and device driver file(s) must be included.
 *
 ******************************************************************************/
#define USE_MS_OS_20

/******************************************************************************
 *
 *  @brief A macro used to configure the USB device descriptors to describe a composite (CDC + BULK) USB device.
 *
 *  @note When disabled, USB enumerates as CDC device only.
 *
 *****************************************************************************/
#define USE_COMPOSITE


//*****************************************************************************
//
// Pinouts (for PAM board only)
//
//*****************************************************************************


//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************
/************************************************************************************
 *
 * @brief Basic initialization and configuration of the PAMB motherboard.
 *
 * @details Initializes minimum required PAMB peripherals.
 *          BoosterPack peripherals are initialized in hal.c
 *
 * @return none
 */
void        INIT_PAMB(void);
uint32_t    getSysClockHz(void);
uint32_t    getSysTickCount(void);
void        resetSysTickCount(void);
#ifdef USE_ULPI
/****************************************************************************
 *
 * @brief Configures the desired USB interface.
 *
 * @param enableULPI is a boolean passed to use the ULPI interface when true,
 *      or the standard default USB interface on the MSP432E when false.
 *
 * @details PAMB can be configured to use the standard MSP432E USB interface,
 *      or the externally provided ULPI interface for transmitting at higher
 *      speeds.  Also, the desired USB host communication must be directed via
 *      a mux to the correct communication input. The function configures the
 *      desired interface and properly connects the USB mux to the interface.
 *
 * @return none
 *
 ***************************************************************************/
    extern void     ConfigureULPI(const bool enableULPI);
#endif



#endif /* SETTINGS_H_ */
