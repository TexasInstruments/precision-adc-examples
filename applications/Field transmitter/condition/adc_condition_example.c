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

struct command
{
    char *name;
    char *help;
    enum status_enum (*function) (char *cmd);
};

static struct command commands[] = {
    {"help", "show this message", help_cmd},
};

/**
 * @brief Prints help information about available commands.
 *
 * @param [in] cmd Pointer to the input command string (currently not used)
 * @return Information if completed successfully
 * @retval STATUS_OK
 */
static enum status_enum help_cmd(char *cmd){
    int i;
    uart_printf("all command must start with cin \r\n");
    for(i=0; i<sizeof(commands)/sizeof(commands[0]); i++){
        uart_printf("%-10s - 0x%x - %s\r\n", commands[i].name, commands[i].function, commands[i].help);
    }
    return STATUS_OK;
}

/**
 * @brief     placeholder
 *
 *
 * @returns   STATUS_OK always, indicating that the function executed successfully
 */
enum status_enum condition_example_adc_cyclic(void){
    return STATUS_OK;
}

/**
 * @brief     placeholder
 *
 *
 * @returns   STATUS_OK always, indicating that the function executed successfully
 */
enum status_enum condition_example_adc_init(void *config){
    return STATUS_OK;
}

/**
 * @brief     placeholder
 *
 *
 * @returns   STATUS_OK always, indicating that the function executed successfully
 */
enum status_enum condition_example_adc_process(int32_t *in, float *out){
    *out = in[0];
    return STATUS_OK;
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
enum status_enum condition_example_adc_cmd(char *cmd){
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
        uart_printf("command not found - try cin help\r\n");
        return STATUS_CMDNOTFOUND;
    }
    return STATUS_ERR; //should never get here
}


