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

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <string.h>
#include "system.h"
#include "uart.h"
#include "flash.h"

uint32_t gSysTick=0;
bool gMainLoop;

static uint32_t gLastCycleDuration=0;
static uint32_t gLastCycleTimestamp=0;
static enum status_enum help_cmd(char *cmd);
static enum status_enum load_cmd(char *cmd);
static enum status_enum reset_cmd(char *cmd);

struct command
{
    char *name;
    char *help;
    enum status_enum (*function) (char *cmd);
};

static struct command commands[] = {
    {"help", "show this message", help_cmd},
    {"saveenv", "writes current configuration to flash", (enum status_enum (*)(char *)) (config_save)},
    {"dumpenv", "dumps current configuration to screen", (enum status_enum (*)(char *)) (config_dump)},
    {"resetenv", "dumps current configuration to screen", (enum status_enum (*)(char *)) (config_reset)},
    {"load", "read processing time of signal chain", (enum status_enum (*)(char *)) (load_cmd)},
    {"reset", "restart the cpu", (enum status_enum (*)(char *)) (reset_cmd)},

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
    uart_printf("All commands must start with sys \r\n");
    for(i=0; i<sizeof(commands)/sizeof(commands[0]); i++){
        uart_printf("%-10s - 0x%x - %s\r\n", commands[i].name, commands[i].function, commands[i].help);
    }
    return STATUS_OK;
}

/**
 * @brief prints how long last processing cycle took
 *
 * @param [in] cmd Pointer to the input command string (currently not used)
 * @return Information if completed successfully
 * @retval STATUS_OK
 */
static enum status_enum load_cmd(char *cmd){
    uart_printf("%d ms\r\n", gLastCycleDuration);
    return STATUS_OK;
}

/**
 * @brief function handling reset command, resets the device
 *
 * @param [in] cmd Pointer to the input command string (currently not used)
 * @return Information if completed successfully
 * @retval STATUS_OK
 */
static enum status_enum reset_cmd(char *cmd){
    DL_SYSCTL_resetDevice(DL_SYSCTL_RESET_POR);
    return STATUS_OK;
}

/**
 * @brief regularly called in main loop to handle uart command line and calculates how long a processing cycle took
 *
 * @return Information if completed successfully
 * @retval STATUS_OK
 */
enum status_enum  system_cyclic(void){
    if(UartFlag){
        uart_handle_cmd();
        UartFlag = false;
    }

    gLastCycleDuration = gSysTick - gLastCycleTimestamp;
    gLastCycleTimestamp = gSysTick;

    return STATUS_OK;
}

/**
 * @brief Executes a command from the UART interface, passing any arguments along.
 *
 * @param[in] cmd The full command string to be executed (e.g. "sys reset"), including any optional arguments.
 * It will check commands structure and execute the function if a command is listed there.
 *
 * @return This function returns a status code indicating the result of executing the command.
 * Will return the return value of called command function.
 *
 * @retval  STATUS_CMDNOTFOUND    Command not found or unrecognized
 */
enum status_enum system_cmd(char *cmd){
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
        uart_printf("command not found - try sys help\r\n");
        return STATUS_CMDNOTFOUND;
    }

    return STATUS_ERR; //should never get here
}

/**
 * @brief handles systick IRQ, incrementing counter and unlocking main loop to run
 *
 */
void SysTick_Handler(void){
    gSysTick++;
    gMainLoop = true;
}

