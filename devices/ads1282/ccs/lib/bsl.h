/* --COPYRIGHT--,BSD
 * Copyright (c) 2021, Texas Instruments Incorporated
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

#ifndef LIB_BSL_H_
#define LIB_BSL_H_

#include <stdint.h>
#include <stdbool.h>
#include "ti/devices/msp432e4/driverlib/driverlib.h"
#include "settings.h"

// Function prototypes
/****************************************************************************
 *
 * @brief Erases the MSP432E application flash.
 *
 * @details One method of forcing the MSP432E into Device Firmware Update (DFU)
 *      is to erase the application flash currently in the device.  With no
 *      application available the device defaults to enumerate as a DFU class device.
 *
 * @return none
 *
 ***************************************************************************/
void erase_flash(void);

/****************************************************************************
 *
 * @brief Force the device to enter DFU mode for firmware application update.
 *
 * @details If the firmware application needs to be updated, the MSP432E can
 *      be set to restart the device into DFU mode by hardware configuration on
 *      the PAMB motherboard.  On software reset a specific pin configuration
 *      allows for the device to restart as DFU instead of the normal application
 *      enumeration.
 *
 * @return none
 *
 ***************************************************************************/
void JumpToBootLoader(void);

#endif /* LIB_BSL_H_ */
