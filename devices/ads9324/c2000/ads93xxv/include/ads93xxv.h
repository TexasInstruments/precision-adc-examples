/*
 * @copyright Copyright (C) 2025-2026 Texas Instruments Incorporated - http://www.ti.com/
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
 */

#ifndef ADS93XXV_H_
#define ADS93XXV_H_

#ifdef __cplusplus

extern "C" {
#endif

//
//=============================================================================
// Includes
//=============================================================================
//
#include "driverlib.h"
#include "device.h"
#include <ads93xxv_hal.h>

//
// ADS93XXV ADC Functions
//
void ADS93XXV_regBankSel(uint8_t bank);
void ADS93XXV_swResetAdc();
void ADS93XXV_initalization();
void ADS93XXV_setupAdcOutOnSdoutSize_16b(void);
void ADS93XXV_setupAdcOutOnSdoutSize_24b(void);
void ADS93XXV_setupRegReadOutOnSdout(void);

void ADS93XXV_setRangeVsns_6V25(void);

void ADS93XXV_disableExtAdcTestPattern();
void ADS93XXV_setupTestPattern();
void ADS93XXV_initExtAdc();
void ADS93XXV_setOsr(void);


#ifdef __cplusplus
}
#endif  

#endif /* ADS93XXV_H_ */
