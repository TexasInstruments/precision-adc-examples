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
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

static enum status_enum help_cmd(char *cmd);
static enum status_enum offset_cmd(char *cmd);
static enum status_enum slope_cmd(char *cmd);


struct cout_offset_slope_struct{
    uint8_t config_length;
    uint8_t config_chksum;
    float offset;
    float slope;
};
static struct cout_offset_slope_struct *cout_config;

struct command
{
    char *name;
    char *help;
    enum status_enum (*function) (char *cmd);
};

static struct command commands[] = {
    {"help", "show this message", help_cmd},
    {"offset", "read/set offset", offset_cmd},
    {"slope", "read/set slope", slope_cmd},
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
    uart_printf("All commands must start with cout \r\n");
    for(i=0; i<sizeof(commands)/sizeof(commands[0]); i++){
        uart_printf("%-10s - 0x%x - %s\r\n", commands[i].name, commands[i].function, commands[i].help);
    }
    return STATUS_OK;
}

/**
 * @brief Read or set offset of scaler
 *
 * @param [in] cmd Pointer to the input command string
 * @return Information if completed successfully
 * @retval STATUS_OK
 */
static enum status_enum offset_cmd(char *cmd){
    char *subcmd = strtok(cmd, " ");
    if(!subcmd){ // nothing given, reading offset
        uart_printf("%f\r\n", cout_config->offset);
        return STATUS_OK;
    }else{
        cout_config->offset = atof(subcmd);
        cout_config->config_chksum = crc8((void*)cout_config+2, cout_config->config_length);
        return STATUS_OK;
    }
}

/**
 * @brief Read or set slope of scaler
 *
 * @param [in] cmd Pointer to the input command string
 * @return Information if completed successfully
 * @retval STATUS_OK
 */
static enum status_enum slope_cmd(char *cmd){
    char *subcmd = strtok(cmd, " ");
    if(!subcmd){ // nothing given, reading slope
        uart_printf("%f\r\n", cout_config->slope);
        return STATUS_OK;
    }else{
        cout_config->slope = atof(subcmd);
        cout_config->config_chksum = crc8((void*)cout_config+2, cout_config->config_length);
        return STATUS_OK;
    }
}

/**
 * @brief     placeholder
 *
 *
 * @returns   STATUS_OK always, indicating that the function executed successfully
 */
enum status_enum condition_offset_slope_uint16_out_cyclic(void){
    return STATUS_OK;
}

/**
 * @brief initializes scaler to default values
 *
 * @param[in]  config pointer to memory to store configuration in
 * @return STATUS_OK if everthing wen well
 */
enum status_enum condition_offset_slope_uint16_out_init(void *config){
    enum status_enum ret = STATUS_OK;

    cout_config = config;
    cout_config->config_length = sizeof(struct cout_offset_slope_struct)-2;
    if(crc8(config+2, cout_config->config_length) != cout_config->config_chksum){
        ERR_LOG("output conditioner reset");
        cout_config->config_chksum = crc8(config+2, cout_config->config_length);
        ret = STATUS_CFG_CHANGED;
        cout_config->offset = 0;
        cout_config->slope = 1;
        cout_config->config_chksum = crc8(config+2, cout_config->config_length);
    }

    return ret;
}

/**
 * @brief     scales input data with slope and offset
 *
 *
 * @returns   STATUS_OK always, indicating that the function executed successfully
 */
enum status_enum condition_offset_slope_uint16_out_process(float in, int32_t *out){
    *out = (int32_t) (in*cout_config->slope + cout_config->offset);
    if(*out > 65535) *out = 65535;
    if(*out < 0) *out = 0;
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
enum status_enum condition_offset_slope_uint16_out_cmd(char *cmd){
    int i;
    char *subcmd = strtok(cmd, " ");
    if(!subcmd) return STATUS_CMDNOTFOUND;
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
        uart_printf("command not found - try cout help\r\n");
        return STATUS_CMDNOTFOUND;
    }
    return STATUS_ERR; //should never get here
}


