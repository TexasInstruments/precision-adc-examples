/* --COPYRIGHT--,BSD
 * Copyright (c) 2018, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#ifndef HAL_H_
#define HAL_H_


//****************************************************************************
//
// Standard libraries
//
//****************************************************************************
#include <stdbool.h>
#include <stdint.h>


//****************************************************************************
//
// Insert processor specific header file(s) here
//
//****************************************************************************

#ifdef EXAMPLE_CODE
/*  --- TODO: INSERT YOUR CODE HERE --- */
#include "ti/devices/msp432e4/driverlib/driverlib.h"
#else
#include "settings.h"
#endif

#include "ads1263.h"


//*****************************************************************************
//
// Pin definitions (MSP432E)
//
//*****************************************************************************

#define START_PORT          (GPIO_PORTP_BASE)
#define START_PIN           (GPIO_PIN_3)

#define nDRDY_PORT          (GPIO_PORTP_BASE)
#define nDRDY_PIN           (GPIO_PIN_5)
#define nDRDY_INT           (INT_GPIOP5)

#define nCS_PORT            (GPIO_PORTM_BASE)
#define nCS_PIN             (GPIO_PIN_0)

#define nRST_nPWDN_PORT     (GPIO_PORTM_BASE)
#define nRST_nPWDN_PIN      (GPIO_PIN_6)




//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

void    InitADCPeripherals(void);
void    delay_ms(uint32_t delay_time_ms);
void    delay_ns(uint32_t delay_time_us);
void    setCS(bool state);
void    setSTART(bool state);
void    setPWDN(bool state);
void    toggleRESET(void);
void    spiSendReceiveArrays(uint8_t DataTx[], uint8_t DataRx[], uint8_t byteLength);
uint8_t spiSendReceiveByte(uint8_t dataTx);
bool    waitForDRDYinterrupt(uint32_t timeout_ms);

// Functions used for testing only
bool    getCS(void);
bool    getPWDN(void);
bool    getRESET(void);
bool    getSTART(void);
void    setRESET(bool state);



//*****************************************************************************
//
// Macros
//
//*****************************************************************************
/** Alias used for setting GPIOs pins to the logic "high" state */
#define HIGH                ((bool) true)

/** Alias used for setting GPIOs pins to the logic "low" state */
#define LOW                 ((bool) false)


#endif /* HAL_H_ */
