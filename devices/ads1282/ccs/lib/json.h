/**
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

#ifndef LIB_JSON_H_
#define LIB_JSON_H_

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "lib/terminal.h"
#include "usb/usbstdio.h"

// EVM states
typedef enum
{
    IDLE,
    BUSY,
    COLLECTING

} evmState_t;

// List of status values
#define ERROR_NONE                      (0)
#define ERROR_DEVICE_UNRESPONSIVE       (1)
#define ERROR_BAD_COMMAND               (2)
#define ERROR_I2C_READ_WRITE_FAILED     (3)
#define ERROR_REG_READ_WRITE_FAILED     (4)
#define ERROR_BAD_CRC_BYTE              (5)

void json_acknowledge(const char *commandBuffer);
void json_console(const char *pcString, ...);
void json_data(const char *pcString, ...);
void json_error(const int error_code, const char *msg, ...);
void json_evmState(const evmState_t state);
void json_id(const char *name, const char *version, const char *date, const char *time);
void json_register(const uint8_t regAddr, const  uint8_t regVal);
void json_command(const char *cmdStr, const char *descriptionStr);


#endif /* LIB_JSON_H_ */
