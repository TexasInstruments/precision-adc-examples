/**
 * @file hal.h
 *
 * @brief Hardware abstraction layer (HAL) descriptor
 *
 * @copyright Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
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


#ifndef HAL_H_
#define HAL_H_

/* Standard libraries */
#include <stdbool.h>
#include <stdint.h>

/* Custom Includes */
#include "ads1282.h"

//****************************************************************************
//
// Insert processor specific header file(s) here
//
//****************************************************************************

/*  --- INSERT YOUR CODE HERE --- */
#include "ti/drivers/GPIO.h"
#include "ti/drivers/SPI.h"
#include "ti_drivers_config.h"      // SysConfig auto-generated file
#include "ti/devices/msp432e4/driverlib/driverlib.h"
#ifndef EXAMPLE_CODE
    #include "ti/drivers/I2C.h"
    #include "pca9535.h"
    #include "..\settings.h"
#endif


//*****************************************************************************
//
// Constants
//
//*****************************************************************************

/// Alias used for setting GPIOs pins to the logic "high" state
#define HIGH                ((bool) true)

/// Alias used for setting GPIOs pins to the logic "low" state
#define LOW                 ((bool) false)

/// Sets SPI clock frequency in hal.c.
#define SCLK_FREQ_HZ        ((uint32_t) 1000000)

/// ADC system clock frequency in Hz. This value has no effect on the hardware,
/// it is used here to scale timing delays.
#define FCLK_FREQ_HZ        ((uint32_t) 4096000)


//****************************************************************************
//
// Pin definitions (MSP-EXP432E401Y BoosterPack #2)
//
//****************************************************************************
//
//                  LEFT                                RIGHT
//               /--------\                          /--------\
//        +3.3V -|3V3  +5V|- +5V                    -|PG1  GND|- GND
//              -|PD2  GND|- GND                    -|PK4  PM7|- SYNC
//              -|PP0  PB4|-                        -|PK5  PP5|-
//              -|PP1  PB5|-                        -|PM0  PA7|-
//              -|PD4* PK0|-                  nDRDY -|PM1  RST|-
//              -|PD5* PK1|-                        -|PM2  PQ2|- DIN
//         SCLK -|PQ0  PK2|-                        -|PH0  PQ3|- DOUT
//              -|PP4  PK3|-                  nPWDN -|PH1  PP3|-
//              -|PN5  PA4|-                        -|PK6  PQ1|-
//              -|PN4  PA5|-                        -|PK7  PM6|- nRESET
//               \--------/                          \--------/
//
// * Pin connection depends on jumper setting
//
//****************************************************************************

#ifndef EXAMPLE_CODE
    #if 0
    /* --- INSERT YOUR CODE HERE --- */
    /* The following code is based on a TI Drivers implementation */

    #define nDRDY_PORT          (GPIO_PORTM_BASE)
    #define nDRDY_PIN           (GPIO_PIN_1)
    #define nDRDY_INTERRUPT     (INT_GPIOM)

    #define nPWDN_PORT          (GPIO_PORTH_BASE)
    #define nPWDN_PIN           (GPIO_PIN_1)


    #define nRESET_PORT         (GPIO_PORTM_BASE)
    #define nRESET_PIN          (GPIO_PIN_6)

    #define SYNC_PORT           (GPIO_PORTM_BASE)
    #define SYNC_PIN            (GPIO_PIN_7)

    // NOTE: The ADS1282EVM uses an I2C pin expander (PCA9535) to control the nRESET, SYNC, MFLAG, & PINMOD pins!

    // NOTE: If J3 on the ADS1282EVM is set to DVDD then we do not need control the CLKSEL pin!
    #define CLKSEL_PORT         (GPIO_PORTKBASE)
    #define CLKSEL_PIN          (GPIO_PIN_6)

    #endif
#endif

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

void    initADCperhiperhals(void);
void    delay_us(const uint32_t delay_time_us);
void    delay_ms(const uint32_t delay_time_ms);
void    setPWDN(const bool state);
void    setRESET(const bool state);
void    toggleRESET(void);
bool    spiSendReceive(const uint8_t transmitBuffer[], uint8_t receiveBuffer[], const uint8_t byteLength);
bool    waitForDRDYinterrupt(uint32_t timeout_ms);

#ifndef EXAMPLE_CODE

    void i2cWrite(uint8_t address, uint8_t dataTx[], uint8_t byteLength);
    void i2cRead(uint8_t address, uint8_t dataRx[], uint8_t byteLength);

    // Not supported
    bool    getMFLAG(void);
    //void    toggleSYNC(void);

#endif

#endif /* HAL_H_ */
