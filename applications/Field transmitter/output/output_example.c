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
#include <stdint.h>
#include <string.h>

static enum status_enum help_cmd(char *cmd);
static enum status_enum get_cmd(char *cmd);

static int32_t gOutRawReading;

struct command
{
    char *name;
    char *help;
    enum status_enum (*function) (char *cmd);
};

static struct command commands[] = {
    {"help", "show this message", help_cmd},
    {"get", "return raw reading", get_cmd},
};

/**
 * @brief Prints help information about available commands.
 *
 * @param [in] cmd Pointer to the input command string (currently not used)
 * @return Information if completed successfully
 * @retval STATUS_OK if the function execution was successful
 */
static enum status_enum help_cmd(char *cmd){
    int i;
    uart_printf("All commands must start with out \r\n");
    for(i=0; i<sizeof(commands)/sizeof(commands[0]); i++){
        uart_printf("%-10s - 0x%x - %s\r\n", commands[i].name, commands[i].function, commands[i].help);
    }
    return STATUS_OK;
}

/**
 * @brief Handles the "get" command from command line
 *
 * This function prints the current raw input reading value to the UART
 *
 * @param[in]  cmd The incoming command string (only examined for "get" prefix in this implementation).
 *
 * @return STATUS_OK always
 */
static enum status_enum get_cmd(char *cmd){
    uart_printf("0x%x\r\n", gOutRawReading);
    return STATUS_OK;
}

/**
 * @brief Cyclic function not used.
 *
 * @return STATUS_OK
 */
enum status_enum output_example_cyclic(void){
    return STATUS_OK;
}

/**
 * @brief init function of example driver. Does nothing
 *
 * @param[in] config pointer where config can be stored
 * @return STATUS_OK if everything went well
 */
enum status_enum output_example_init(void *config){
    return STATUS_OK;
}

/**
 * @brief called by main loop to set new DAC value
 *7
 * @param[in] raw value to set the DAC to
 * @return STATUS_OK always
 */
enum status_enum output_example_set_reading(int32_t raw){
    gOutRawReading = raw;
    return STATUS_OK;
}

/**
 * @brief handles UART commands
 *
 * @param[in] cmd pointer of UART input
 * @return return value of command function, STATUS_CMDNOTFOUND if command is not available, STATUS_ERR in case of generic error
 */
enum status_enum output_example_cmd(char *cmd){
    int i;
    char *subcmd = strtok(cmd, " ");
    bool found=false;
    for(i=0; i<sizeof(commands)/sizeof(commands[0]); i++){
        if(!strcmp(commands[i].name, subcmd)){
            // command found, call it
            return(commands[i].function(cmd+strlen(commands[i].name)+1));
            found = true;
            break;
        }
    }
    if(found == false) { //unknown command
        uart_printf("command not found - try out help\r\n");
        return STATUS_CMDNOTFOUND;
    }
    return STATUS_ERR; //should never get here
}
