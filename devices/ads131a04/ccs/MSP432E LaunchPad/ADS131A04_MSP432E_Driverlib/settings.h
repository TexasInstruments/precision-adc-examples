#ifndef SETTINGS_H_
#define SETTINGS_H_

// Standard libraries
#include <stdbool.h>

// Custom libraries
#include "ti/devices/msp432e4/driverlib/driverlib.h"



//*****************************************************************************
//
// Pinouts
//
//*****************************************************************************

#define LED1_PORT           (GPIO_PORTN_BASE)
#define LED1_PIN            (GPIO_PIN_1)

#define LED2_PORT           (GPIO_PORTN_BASE)
#define LED2_PIN            (GPIO_PIN_0)

#define LED3_PORT           (GPIO_PORTF_BASE)
#define LED3_PIN            (GPIO_PIN_4)

#define LED4_PORT           (GPIO_PORTF_BASE)
#define LED4_PIN            (GPIO_PIN_0)



//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************

extern void         INIT_LAUNCHPAD(void);
extern uint32_t     getSysClockHz(void);



#endif /* SETTINGS_H_ */
