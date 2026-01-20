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

#ifndef IDENTIFICATION_H_
#define IDENTIFICATION_H_

#include "main.h"
#include "stdint.h"

/** ADC functions */
typedef enum status_enum (*adc_init_func)(void *adc_config);
typedef enum status_enum (*adc_cyclic_func)(void);
typedef enum status_enum (*adc_get_reading_func)(int32_t *adc_raw);
typedef enum status_enum (*adc_cmd_func)(char *cmd);

/** adc conditioner */
typedef enum status_enum (*condition_adc_init_func)(void *condition_adc_config);
typedef enum status_enum (*condition_adc_cyclic_func)(void);
typedef enum status_enum (*condition_adc_process_func)(int32_t *in, float *out);
typedef enum status_enum (*condition_adc_cmd_func)(char *cmd);

/** output conditioner **/
typedef enum status_enum (*condition_out_init_func)(void *condition_out_config);
typedef enum status_enum (*condition_out_cyclic_func)(void);
typedef enum status_enum (*condition_out_process_func)(float in, int32_t *out);
typedef enum status_enum (*condition_out_cmd_func)(char *cmd);

/** output functions **/
typedef enum status_enum (*output_init_func)(void *condition_out_config);
typedef enum status_enum (*output_cyclic_func)(void);
typedef enum status_enum (*output_set_reading_func)(int32_t raw);
typedef enum status_enum (*output_cmd_func)(char*cmd);

/** other functions **/
typedef enum status_enum (*cpu_clock_init_func)(void);


/**
 * @brief This structure is used to map hardware specific functions to generic function pointers
 *
 */

struct id_func_input_map_struct{
    /** adc initialisation function */
    adc_init_func adc_init;
    /** This function gets called regularly by the main function, do not block here */
    adc_cyclic_func adc_cyclic;
    /**  read adc reading and return it */
    adc_get_reading_func adc_get_reading;
    /** handle uart commands and execute them */
    adc_cmd_func adc_cmd;
    /** stores the name of the adc driver **/
    char *adc_driver_name;

    /** adc conditioning init function **/
    condition_adc_init_func condition_adc_init;
    /** adc conditioning cyclic function **/
    condition_adc_cyclic_func condition_adc_cyclic;
    /** adc conditioning processing function **/
    condition_adc_process_func condition_adc_process;
    /** adc conditioning uart cmd handler **/
    condition_adc_cmd_func condition_adc_cmd;
};

struct id_func_output_map_struct{
    /** output conditioning init **/
    condition_out_init_func condition_out_init;
    /** output conditioning cyclic function **/
    condition_out_cyclic_func condition_out_cyclic;
    /** output conditioning set the output value **/
    condition_out_process_func condition_out_process;
    /** output conditioning uart cmd handler **/
    condition_out_cmd_func condition_out_cmd;

    /** output stage init **/
    output_init_func output_init;
    /** output cyclic function **/
    output_cyclic_func output_cyclic;
    /** output setter **/
    output_set_reading_func output_set_reading;
    /** output uart handler **/
    output_cmd_func output_cmd;
    /** stores the name of the output driver **/
    char *output_driver_name;

    /** init cpu clock **/
    cpu_clock_init_func cpu_clock_init;
};

/**
 * @brief Loads the functions into generic function pointers matching the actual hardware id
 *
 */
void load_id(void);

#endif /* IDENTIFICATION_H_ */
