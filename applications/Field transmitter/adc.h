/*
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
 */

#ifndef ADC_H_
#define ADC_H_

#include "main.h"
#include <stdint.h>

/**
 * @brief This function gets called regularly by the main function, do not block here
 *
 * @return status code
 */
enum status_enum (*adc_cyclic)(void); //gets called regularly in main loop whenever there is time


/**
 * @brief Called once at startup to initialize the ADC system
 *
 * @param config pointer to configuration array, this is a separate flash section
 * and can be used arbitrarily for ever ADC implementation, this array might contain no or invalid data
 * @return status code, return STATUS_CFG_CHANGED if config memory was changed and should be written to flash
 */
enum status_enum (*adc_init)(void *adc_config); //gets called once during init, expect config memory pointer, returns status

/**
 * @brief read adc reading and return it
 *
 * @param adc_raw pointer to store the adc reading to
 * @return status code
 */
enum status_enum (*adc_get_reading)(int32_t *adc_raw); //return status, and raw value

/**
 * @brief handle uart commands and execute them
 *
 * @param cmd pointer of received command from UART interface
 * @return status code
 */
enum status_enum (*adc_cmd)(char *cmd);

/** stores the pointer to the name of the selected driver **/
extern char *gAdcDriverName;

/** stores number of drivers **/
extern int AdcDriverCnt;

#endif
