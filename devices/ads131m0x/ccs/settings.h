#ifndef SETTINGS_H_
#define SETTINGS_H_

// Standard libraries
#include <stdbool.h>

// Custom libraries
#include "ti/devices/msp432e4/driverlib/driverlib.h"
#include "ti/usblib/msp432e4/usblib.h"
#include "ti/usblib/msp432e4/usbcdc.h"
#include "usb/usb_structs.h"



//*****************************************************************************
//
// Button Configuration Options
//
//*****************************************************************************

// Enable this macro to call JumpToBootLoader() when the BSL button is pressed.
//#define BSL_INTERRUPT

// TODO: Add option to change between PAMB and LP pinout
//#define LAUNCHPAD_PINOUT



//*****************************************************************************
//
// USB Configuration Options
//
//*****************************************************************************

// TODO: Test all of the permutations of these configurations ...
#define USE_ULPI                // Configures MSP432E to use ULPI peripheral for high-speed USB 2.0.
#define USE_MS_OS_20            // Enables the Microsoft OS 2.0 Descriptor for automatic (driver-less) enumeration.
#define USE_COMPOSITE           // Configures the USB device descriptors to describe a composite (CDC + BULK) USB device.

// NOTE: When disabled, code will configure full-speed USB peripheral, not respond to MS OS descriptor, and enumerate as CDC device only.



//*****************************************************************************
//
// Pinouts (for PAM board only)
//
//*****************************************************************************

// MSP432E LaunchPad does not have a USB UPLI, so make sure to disable it when using the LaunchPad pinout
#ifdef LAUNCHPAD_PINOUT
#undef USE_ULPI
#endif

#define LED1_PORT           (GPIO_PORTN_BASE)
#define LED1_PIN            (GPIO_PIN_1)

#define LED2_PORT           (GPIO_PORTN_BASE)
#define LED2_PIN            (GPIO_PIN_0)

#define LED3_PORT           (GPIO_PORTF_BASE)
#define LED3_PIN            (GPIO_PIN_4)

#define LED4_PORT           (GPIO_PORTF_BASE)
#define LED4_PIN            (GPIO_PIN_0)

#define BSL_PORT            (GPIO_PORTA_BASE)
#define BSL_PIN             (GPIO_PIN_1)
#define BSL_INT             (INT_GPIOA)

// USB MUX select pin (PAM PB0, Tiva PD4?)
#define MUXSEL_PORT         (GPIO_PORTB_BASE)
#define MUXSEL_PIN          (GPIO_PIN_0)

// ULPI reset pin (PAM PB1, Tiva PK0)
#define ULPI_nRESET_PORT    (GPIO_PORTB_BASE)
#define ULPI_nRESET_PIN     (GPIO_PIN_1)

// BOOST_EN pin (PAM PG0, Tiva ?)
#define BOOST_EN_PORT       (GPIO_PORTA_BASE)
#define BOOST_EN_PIN        (GPIO_PIN_0)



//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************

extern void         INIT_PAMB(void);
extern uint32_t     getSysClockHz(void);
#ifdef USE_ULPI
    extern void     ConfigureULPI(const bool enableULPI);
#endif



#endif /* SETTINGS_H_ */
