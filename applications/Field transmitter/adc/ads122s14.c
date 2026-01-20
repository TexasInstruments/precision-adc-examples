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

#include "main.h"
#include <stdint.h>

/**
 * @brief     Cyclic function of empty ADC driver framework
 *
 * @details  This function gets called regularly by the main loop. Up to the user what to do at this time.
 *
 * @returns   STATUS_OK always, indicating that the function executed successfully
 */
enum status_enum ads122s14_cyclic(void){

    return STATUS_OK;
}

/**
 * @brief     Initializes and configures ADC.
 *
 * @details  Empty driver framework
 *
 * @param[in ] config        A pointer to an initialization configuration structure
 *
 * @return    This function returns a status code indicating the result of the operation
 *
 * @retval   STATUS_OK       Successful initialization
 * @retval   STATUS_CFG_CHANGED     Configuration was modified and reset to a default state
 */
enum status_enum ads122s14_init(void *config){

    return STATUS_OK;
}

/**
 * @brief     Retrieves the current ADC reading, and returns a status code indicating whether the value was successfully retrieved or not.
 *
 * @details  Empty driver framework
 *
 * @param[out] adc_raw        A pointer to an integer variable where the cached ADC reading will be stored
 *
 * @return    This function returns a status code indicating the result of the operation
 *
 * @retval   STATUS_OK       The ADC reading was successfully retrieved and stored to adc_raw
 * @retval  STATUS_NOCHANGE    No new data available, does not change adc_raw pointer
 */
enum status_enum ads122s14_get_reading(int32_t *adc_raw){

    return STATUS_OK;
}

/**
 * @brief gets called by UART handler to process a command
 *
 * @details  Empty driver framework
 *
 * @param[in] cmd The full command string to be executed (e.g. "adc get"), including any optional arguments.
 * It will check commands structure and execute the function if a command is listed there.
 *
 * @return This function returns a status code indicating the result of executing the command.
 * Will return the return value of called command function.
 *
 * @retval  STATUS_CMDNOTFOUND    Command not found or unrecognized
 */
enum status_enum ads122s14_cmd(char *cmd){

    return STATUS_OK;
}

