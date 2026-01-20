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

#ifndef OUTPUT_H_
#define OUTPUT_H_

#include "main.h"
#include <stdint.h>

/**
 * @brief Called once at startup
 *
 * @param config pointer to configuration array, this is a separate flash section  this array might contain
 * no or invalid data
 * @return status code, return STATUS_CFG_CHANGED if config memory was changed and should be written to flash
 */
enum status_enum (*output_init)(void *output_config); //gets called once during init, expect config memory pointer, returns status

/**
 * @brief This function gets called regularly by the main function, do not block here
 *
 * @return status code
 */
enum status_enum (*output_cyclic)(void); //gets called regularly in main loop whenever there is time

/**
 * @brief gets called to set output value
 *
 * @param raw raw output reading to set interface to
 * @return status code
 */
enum status_enum (*output_set_reading)(int32_t raw);


/**
 * @brief handle uart commands and execute them
 *
 * @param cmd pointer of received command from UART interface
 * @return status code
 */
enum status_enum (*output_cmd)(char *cmd);

/** stores the pointer to the name of the selected driver **/
extern char *OutputDriverName;

/** stores number of supported output drivers **/
extern int OutputDriverCnt;

#endif /* OUTPUT_H_ */
