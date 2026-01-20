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
#include "system/uart.h"
#include "system/flash.h"
#include "system/system.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

static enum status_enum help_cmd(char *cmd);
static enum status_enum sim_cmd(char *cmd);
static enum status_enum get_cmd(char *cmd);

struct command
{
    char *name;
    char *help;
    enum status_enum (*function) (char *cmd);
};

static struct command commands[] = {
    {"sim", "simulate adc values sim (static|up) value {step interval}", sim_cmd},
    {"get", "return raw adc reading", get_cmd},
    {"help", "show this message", help_cmd},
};

enum sim_mode_enum {sim_disabled, sim_static, sim_counting_up};

struct adc_config_struct{
    uint8_t config_length;
    uint8_t config_chksum;
    enum sim_mode_enum sim_mode;
    uint32_t sim_interval;
    uint32_t sim_last;
    uint32_t sim_step;
    uint32_t sim_value;
};

static int32_t gAdcRawReading;

static struct adc_config_struct *adc_config;

/**
 * @brief Prints help information about available commands.
 *
 * @param [in] cmd Pointer to the input command string (currently not used)
 * @return Information if completed successfully
 * @retval STATUS_OK
 */
static enum status_enum help_cmd(char *cmd){
    int i;
    uart_printf("All commands must start with adc \r\n");
    for(i=0; i<sizeof(commands)/sizeof(commands[0]); i++){
        uart_printf("%-10s - 0x%x - %s\r\n", commands[i].name, commands[i].function, commands[i].help);
    }
    return STATUS_OK;
}

/**
 * @brief     Processes a simulation command.
 *
 * @details  This function parses a command string and triggers simulation mode or counting up sequences based on its contents. The following sub-commands are supported:
 *           - "static <value>" : Sets the ADC value to the specified value.
 *           - "up  <value> <step> <interval>" : Starts an incrementing counter with the specified step value in each interval.
 *
 * @param[in] cmd        Pointer to the input command string, split into sub-commands and arguments using space characters as delimiters.
 *
 * @returns   STATUS_OK if the function execution was successful
 *           - STATUS_CMDERROR if a syntax error occurred while parsing the input command.
 */
static enum status_enum sim_cmd(char *cmd){
    int cnt;
    char *subcmd = strtok(cmd, " ");
    if(!subcmd) return STATUS_CMDERROR;
    if(!strcmp("static", subcmd)){
        adc_config->sim_mode = sim_static;
        subcmd = strtok(NULL, " "); /* this should contain the value that was sent */
        if(!subcmd) return STATUS_CMDERROR;
        adc_config->sim_value = atoi(subcmd);
        gAdcRawReading = adc_config->sim_value;
    }else if(!strcmp("up", subcmd)){
        cnt = sscanf((subcmd+strlen("up")+1), "%d %d %d", &adc_config->sim_value, &adc_config->sim_step, &adc_config->sim_interval);
        adc_config->sim_last = 0;
        adc_config->sim_mode = sim_counting_up;
        if(cnt != 3) return STATUS_CMDERROR;
    }
    return STATUS_OK;
}

/**
 * @brief     Prints the current ADC raw reading value as a hexadecimal string.
 *
 * @details  This function simply prints the contents of `adc_raw_reading` to the UART console and returns immediately.
 *
 * @param[in ] cmd        Pointer to an unused command string (this parameter is not used by this function)
 *
 * @returns   STATUS_OK always, indicating that the function executed successfully
 */
static enum status_enum get_cmd(char *cmd){
    uart_printf("0x%x\r\n", gAdcRawReading);
    return STATUS_OK;
}

/**
 * @brief     Updates the simulated ADC raw reading based on cyclic counting simulation values.
 *
 * @details  If the current simulation mode is "counting up", this function checks whether the time since last update exceeds
 *  the specified interval `sim_interval`. If so, it updates the last update timestamp, increments the simulation value by
 *  a specified step, and assigns the new value to `adc_raw_reading`.
 *
 * @returns   STATUS_OK always, indicating that the function executed successfully
 */
enum status_enum adc_example_cyclic(void){
    if(adc_config->sim_mode == sim_counting_up){
        if(adc_config->sim_last+adc_config->sim_interval <= gSysTick){
            adc_config->sim_last = gSysTick;
            adc_config->sim_value += adc_config->sim_step;
            gAdcRawReading = adc_config->sim_value;
        }
    }
    return STATUS_OK;
}

/**
 * @brief     Initializes and configures the Example ADC system.
 *
 * @details  This function sets up the initial state of the simulated ADC.
 * Checks the configuration and optionally resets the configuration if it was empty before.
 * It also seeds the raw reading value based on the chosen simulation mode.
 *
 * @param[in ] config        A pointer to an initialization configuration structure
 *
 * @return    This function returns a status code indicating the result of the operation
 *
 * @retval   STATUS_OK       Successful initialization
 * @retval   STATUS_CFG_CHANGED     Configuration was modified and reset to a default state
 */
enum status_enum adc_example_init(void *config){
    enum status_enum ret = STATUS_OK;
    adc_config = config;
    uart_printf("Using Example ADC\r\n");

    adc_config->config_length = sizeof(struct adc_config_struct)-2;
    /* on first start, the configuration structure is empty, the crc should be wrong, so initialize config structure */
    if(crc8(config+2,adc_config->config_length) != adc_config->config_chksum){
        ERR_LOG("ADC config reset");
        adc_config->sim_mode = sim_disabled;
        adc_config->config_chksum = crc8(config+2, adc_config->config_length);
        ret = STATUS_CFG_CHANGED;
    }

    if(adc_config->sim_mode == sim_static){
        gAdcRawReading = adc_config->sim_value;
    }
    if(adc_config->sim_mode == sim_counting_up){
        adc_config->sim_last = 0;
    }

    return ret;
}

/**
 * @brief     Retrieves the current ADC reading, caches it, and returns a status code indicating whether the value was successfully retrieved or not.
 *
 * @param[out] adc_raw        A pointer to an integer variable where the cached ADC reading will be stored
 *
 * @return    This function returns a status code indicating the result of the operation
 *
 * @retval   STATUS_OK       The ADC reading was successfully retrieved and stored to adc_raw
 * @retval  STATUS_NOCHANGE    No new data available, does not change adc_raw pointer
 */
enum status_enum adc_example_get_reading(int32_t *adc_raw){
    static int32_t adc_raw_reading_old;

    if(adc_raw_reading_old != gAdcRawReading){
        adc_raw[0] = gAdcRawReading;
        adc_raw_reading_old = gAdcRawReading;
        return STATUS_OK;
    }
    return STATUS_NOCHANGE;
}

/**
 * @brief Executes a command from the UART interface, passing any arguments along.
 *
 * @param[in] cmd The full command string to be executed (e.g. "adc get"), including any optional arguments.
 * It will check commands structure and execute the function if a command is listed there.
 *
 * @return This function returns a status code indicating the result of executing the command.
 * Will return the return value of called command function.
 *
 * @retval  STATUS_CMDNOTFOUND    Command not found or unrecognized
 */
enum status_enum adc_example_cmd(char *cmd){
    int i;
    char *subcmd = strtok(cmd, " ");
    if(!subcmd) return STATUS_CMDNOTFOUND;
    bool found=false;
    for(i=0; i<sizeof(commands)/sizeof(commands[0]); i++){
        if(!strcmp(commands[i].name, subcmd)){
            /* command found, call it */
            return(commands[i].function(cmd+strlen(commands[i].name)+1));
            found = true;
            break;
        }
    }
    if(found == false) { /* unknown command*/
        uart_printf("command not found - try adc help\r\n");
        return STATUS_CMDNOTFOUND;
    }
    return STATUS_ERR; /* should never get here */
}

