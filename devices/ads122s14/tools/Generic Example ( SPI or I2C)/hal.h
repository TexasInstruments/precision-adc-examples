/**
 * \copyright Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
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
#ifndef EVM_HAL_H_
#define EVM_HAL_H_
#include <evm/ads122s1x.h>
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
/*  --- INSERT YOUR CODE HERE --- */
#include "ti/devices/msp432e4/driverlib/driverlib.h"
#else
#include "..\settings.h"
#endif
//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************
void delay_ms(const uint32_t delay_time_ms);
void delay_us(const uint32_t delay_time_us);
void setSTART( bool state );
void toggleSTART( void );
void toggleRESET(void);
bool getDRDYstatus(void);
void setDRDYstatus(bool DRDYflag);
void GPIO_DRDYn_IRQHandler(uint_least8_t index);
bool waitForDRDYinterrupt(const uint32_t timeout_ms);
bool getDRDYninterruptStatus(void);
void setDRDYninterruptStatus(const bool value);
void enableDRDYninterrupt(const bool intEnable);
int8_t transferI2CData( uint8_t *writeBuff, uint8_t wLength, uint8_t *readBuff, uint8_t rLength);

#ifdef SPI_COMMS
bool getCS(void);
void setCS(const bool state);
void enableCSpullup(void);
void enableCSpulldown(void);
bool spiSendReceiveArrays(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t bufferLength);
#endif

//*****************************************************************************
//
// Macros
//
//*****************************************************************************
/** Alias used for setting GPIOs pins to the logic "high" state */
#define HIGH                ((bool) true)

/** Alias used for setting GPIOs pins to the logic "low" state */
#define LOW                 ((bool) false)

#endif /* EVM_HAL_H_ */
#ifdef EXAMPLE_CODE
//****************************************************************************
//
// BoosterPack pinout...LP
//
//****************************************************************************
//
//                  LEFT                                RIGHT
//               /--------\                          /--------\
//        +3.3V -|3V3  +5V|- +5V                    -|PG1  GND|- GND
//              -|PD2  GND|- GND                    -|PK4  PM7|-
//              -|PP0  PB4|-                        -|PK5 *PP5|-
//              -|PP1  PB5|-                        -|PM0  PA7|-
//              -|PD4  PK0|-                        -|PM1  RST|-
//              -|PD5  PK1|-                        -|PM2  PQ2|-
//              -|PQ0  PK2|-                        -|PH0  PQ3|-
//              -|PP4* PK3|-                        -|PH1 *PP3|-
//          SCL -|PN5  PA4|-                        -|PK6  PQ1|-
//          SDA -|PN4  PA5|-              ALERT/RDY -|PK7  PM6|-
//               \--------/                          \--------/
//
//****************************************************************************
#else
// TODO: Modify the correct template version and delete non used version
// NOTE: pins marked with '*' differ between PAMB and LP
//****************************************************************************
//
// BoosterPack pinout...LP
//
//****************************************************************************
//
//                  LEFT                                RIGHT
//               /--------\                          /--------\
//        +3.3V -|3V3  +5V|- +5V                CLK -|PG1  GND|- GND
//              -|PD2  GND|- GND                    -|PK4  PM7|- CONVST
//              -|PP0  PB4|-                        -|PK5 *PP5|-
//              -|PP1  PB5|-                 nRESET -|PM0  PA7|-
//              -|PD4  PK0|-                        -|PM1  RST|-
//              -|PD5  PK1|-                        -|PM2  PQ2|- DIN
//         SCLK -|PQ0  PK2|-                        -|PH0  PQ3|- DOUT
//     EEPROMWP -|PP4* PK3|-                        -|PH1 *PP3|-
//          SCL -|PN5  PA4|-                        -|PK6  PQ1|- nCS
//          SDA -|PN4  PA5|-                  nDRDY -|PK7  PM6|-
//               \--------/                          \--------/
//
//****************************************************************************
//
// BoosterPack pinout...PAMB
//
//****************************************************************************
//
//                  LEFT                                RIGHT
//               /--------\                          /--------\
//        +3.3V -|3V3  +5V|- +5V                CLK -|PG1  GND|- GND
//              -|PD2  GND|- GND                    -|PK4  PM7|- CONVST
//              -|PP0  PB4|-                        -|PK5 *PN3|-
//              -|PP1  PB5|-                 nRESET -|PM0  PA7|-
//              -|PD4  PK0|-                        -|PM1  RST|-
//              -|PD5  PK1|-                        -|PM2  PQ2|- DIN
//         SCLK -|PQ0  PK2|-                        -|PH0  PQ3|- DOUT
//     EEPROMWP -|PH3* PK3|-                        -|PH1 *PN2|-
//          SCL -|PN5  PA4|-                        -|PK6  PQ1|- nCS
//          SDA -|PN4  PA5|-                  nDRDY -|PK7  PM6|-
//               \--------/                          \--------/
//
//****************************************************************************
//
// * NOTE: These pins differ from the MSP-EXP432E401Y LaunchPad!
//  MSP432E LP:  PP4, PP5, PP3
//  PAMB (REV1): PA6, PN1, PN0
//  PAMB (REV2): PH3, PN3, PN2
//  PAMB (A)   : PH3, PN3, PN2
//
#endif
//*****************************************************************************
//
// Pin definitions (MSP432E401Y)
//
//*****************************************************************************
#ifdef EXAMPLE_CODE
#define ALERT_RDY_PORT          (GPIO_PORTK_BASE)
#define ALERT_RDY_PIN           (GPIO_PIN_7)
#define ALERT_RDY_INT           (INT_GPIOK7)
#define I2Cbus 0    // Used in TI Drivers implementation
#else
// TODO: Make sure pin definitions match EVM layout (and pictorial above)  as not all connections available
/* Optional pin control or input signals */
// (OPTIONAL) External reset source
#endif
