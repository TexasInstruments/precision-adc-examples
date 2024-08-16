/*
 *  Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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

#ifndef LIB_EEPROM_H_
#define LIB_EEPROM_H_

#include <stdint.h>
#include <string.h>
#include <ti/devices/msp432e4/inc/msp432.h>
#include <ti/devices/msp432e4/driverlib/sysctl.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CMSP432E4.h>
#include "usb/usbstdio.h"
#include "json.h"
#include "settings.h"

// Define the slave address for the EEPROM
/******************************************************************************
 *
 *  @brief A macro used to describe the address location of the I2C EEPROM used on the EVM.
 *
 *****************************************************************************/
/* I2C Peripheral Macros */
// TODO: Make sure previous references to driverlib are converted to TI Drivers for I2C
//#define I2C_BIT_RATE_Kbps                       400  /* Give only 100kbps or 400kbps */
// I2C slave address for EEPROM on EVM
#define I2C_SLAVE_ADDRESS                       0x51

/* I2C EEPROM Macros */
#define I2C_EEPROM_PAGE_SIZE                    0x20
#define I2C_EEPROM_MEMORY_SIZE                  0x1000
#define I2C_EEPROM_SOE_LEN                      0x05
#define I2C_EEPROM_EOE_LEN                      0x04

/****************************************************************************
 *
 * @brief Reads the data stored on the I2C EEPROM of the EVM.
 *
 * @return int8_t of possible error, with '0' as no error.
 *
 ***************************************************************************/
int8_t readEEPROM(void);

/****************************************************************************
 *
 * @brief Writes data to the I2C EEPROM on the EVM
 *
 * @param *data pointer to a string array of data.
 * @param dLen unsigned byte of the data array length.
 *
 * @details I2C EEPROM can be written as pages with a maximum of 32 bytes per
 *      page.
 *
 * @return int8_t of possible error, with '0' as no error.
 *
 ***************************************************************************/
int8_t writeEEPROM(char *data, uint8_t dLen);

/****************************************************************************
 *
 * @brief Writes data to the EEPROM on the MSP432E
 *
 * @param *data pointer to a string array of data.
 * @param dLen unsigned byte of the data array length.
 *
 * @details EEPROM data of the MSP432E is not byte frame size, but rather 32-bit.
 *      Data is pieced together to write the ASCII data as an unsigned 32-bit value.
 *
 * @return int8_t of possible error, with '0' as no error.
 *
 ***************************************************************************/
int8_t writeInternalEEPROM(char *data, uint8_t dLen);

/****************************************************************************
 *
 * @brief Reads data from the EEPROM on the MSP432E
 *
 * @details EEPROM data of the MSP432E is not byte frame size, but rather 32-bit.
 *      Data is split into character bytes from the unsigned 32-bit value.
 *
 * @return int8_t of possible error, with '0' as no error.
 *
 ***************************************************************************/
int8_t readInternalEEPROM(void);

/****************************************************************************
 *
 * @brief Erases data from the EEPROM on the EVM
 *
 *
 * @return int8_t of possible error, with '0' as no error.
 *
 ***************************************************************************/
int8_t clearEEPROM(void);

/****************************************************************************
 *
 * @brief Sets the control pin state for the EEPROM WE
 *
 * @param enable boolean used for setting the control state.
 *
 * @details The WE (write enable) pin of the EEPROM is active low.  A logic
 *     low enables writing to the EEPROM and a logic high disables writing.
 *
 * @return int8_t of possible error, with '0' as no error.
 *
 ***************************************************************************/
int8_t writeProtectEnable(bool enable);

/****************************************************************************
 *
 * @brief Checks the state of EEPROM or if EVM is not connected.
 *
 *
 * @return bool, true EEPROM found or EVM connected or false not found.
 *
 ***************************************************************************/
bool EEPROMcheck(void);
#endif /* LIB_EEPROM_H_ */
