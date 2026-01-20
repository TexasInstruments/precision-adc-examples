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

#ifdef USEIOLINK

#include "main.h"
#include "uart.h"
#include "flash.h"
#include "IOL_Types.h"
#include <stdint.h>
#include <string.h>
#include <strings.h>
#include <ti_msp_dl_config.h>
#include "output_io-link.h"

extern void IOLD_API_Init(struct iolink_config_struct *config, struct iolink_process_struct *process);
extern void IOLD_API_vRun (void);

static void pin2_init(void);

static struct iolink_config_struct *gIolinkConfig;
static struct iolink_process_struct gIolinkProcess;

static enum status_enum help_cmd(char *cmd);
static enum status_enum cycle_cmd(char *cmd);
static enum status_enum com_cmd(char *cmd);
static enum status_enum status_cmd(char *cmd);
static enum status_enum pin2mode_cmd(char *cmd);
static enum status_enum pin2value_cmd(char *cmd);

struct command
{
    char *name;
    char *help;
    enum status_enum (*function) (char *cmd);
};

static struct command commands[] = {
    {"help", "show this message", help_cmd},
    {"cycle", "get/set cycle time in ms", cycle_cmd},
    {"com", "get/set com", com_cmd},
    {"pin2mode", "set pin 2 mode [in|npn|pnp]", pin2mode_cmd},
    {"pin2value", "set pin 2 value [0|1]", pin2value_cmd},
    {"status", "get IO-Link status", status_cmd},
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
    uart_printf("all command must start with out \r\n");
    for(i=0; i<sizeof(commands)/sizeof(commands[0]); i++){
        uart_printf("%-10s - 0x%x - %s\r\n", commands[i].name, commands[i].function, commands[i].help);
    }
    return STATUS_OK;
}

/**
 * @brief Handles command line input to read/update cycle time
 *
 * @param [in] cmd Pointer to the input command string
 *
 * @return Information if completed successfully
 * @retval STATUS_OK if the function execution was successful
 * @retval STATUS_CMDERROR otherwise
 */
static enum status_enum cycle_cmd(char *cmd){
    enum status_enum ret = STATUS_CMDERROR;
    int cnt;
    float cycle;
    char *subcmd = strtok(cmd, " ");
    if(!subcmd){
        uart_printf("%.1f ms\r\n", gIolinkConfig->cycle_time);
        ret = STATUS_OK;
    }else{
        sscanf(subcmd, "%f", &cycle);
        if((cycle >= 0.4) && (cycle <= 132.8)){
            gIolinkConfig->cycle_time = cycle;
            gIolinkConfig->config_chksum = crc8((void*)(gIolinkConfig)+2, gIolinkConfig->config_length);
            uart_printf("cycle time changed, enter sys saveenv and sys reset to apply changes\r\n");
            ret = STATUS_OK;
        }
    }
    return ret;
}

/**
 * @brief Handles command line input to read/update COM rate
 *
 * @param [in] cmd Pointer to the input command string
 *
 * @return Information if completed successfully
 * @retval STATUS_OK if the function execution was successful
 * @retval STATUS_CMDERROR otherwise
 */
static enum status_enum com_cmd(char *cmd){
    enum status_enum ret = STATUS_CMDERROR;
    int cnt;
    int mode;
    char *subcmd = strtok(cmd, " ");
    if(!subcmd){
        uart_printf("COM%d\r\n", (int)(gIolinkConfig->TransferMode));
        ret = STATUS_OK;
    }else{
        sscanf(subcmd, "%d", &mode);
        if((mode >= 1) && (mode <= 3)){
            gIolinkConfig->TransferMode = mode;
            gIolinkConfig->config_chksum = crc8((void*)(gIolinkConfig)+2, gIolinkConfig->config_length);
            uart_printf("com mode changed, enter sys saveenv and sys reset to apply changes\r\n");
            ret = STATUS_OK;
        }
    }
    return ret;
}

/**
 * @brief Handles command line input to update Pin 2 mode
 *
 * @param [in] cmd Pointer to the input command string
 *
 * @return Information if completed successfully
 * @retval STATUS_OK if the function execution was successful
 * @retval STATUS_CMDERROR otherwise
 */
static enum status_enum pin2mode_cmd(char *cmd){
    enum status_enum ret = STATUS_CMDERROR;
    int mode;
    char *subcmd = strtok(cmd, " ");
    if(!subcmd){
        return STATUS_CMDERROR;
    }else if(!strcasecmp(subcmd, "di")){
        gIolinkConfig->pin2_mode = IN;
        ret = STATUS_OK;
    }
    else if(!strcasecmp(subcmd, "npn")){
        gIolinkConfig->pin2_mode = NPN;
        ret = STATUS_OK;
    }else if(!strcasecmp(subcmd, "PNP")){
        gIolinkConfig->pin2_mode = PNP;
        ret = STATUS_OK;
    }
    else if(!strcasecmp(subcmd, "PP")){
        gIolinkConfig->pin2_mode = PP;
        ret = STATUS_OK;
    }

    gIolinkConfig->config_chksum = crc8((void*)(gIolinkConfig)+2, gIolinkConfig->config_length);
    pin2_init();

    return ret;
}

/**
 * @brief Handles command line input to update Pin 2 status
 *
 * @param [in] cmd Pointer to the input command string
 *
 * @return Information if completed successfully
 * @retval STATUS_OK if the function execution was successful
 * @retval STATUS_CMDERROR otherwise
 */
static enum status_enum pin2value_cmd(char *cmd){
    enum status_enum ret = STATUS_CMDERROR;
    int mode;
    char *subcmd = strtok(cmd, " ");
    if(!subcmd){
        return STATUS_CMDERROR;
    }else if(!strcasecmp(subcmd, "1")){
        gIolinkConfig->pin2_state = true;
        ret = STATUS_OK;
    }else if(!strcasecmp(subcmd, "true")){
        gIolinkConfig->pin2_state = true;
        ret = STATUS_OK;
    }else if(!strcasecmp(subcmd, "0")){
        gIolinkConfig->pin2_state = false;
        ret = STATUS_OK;
    }else if(!strcasecmp(subcmd, "false")){
        gIolinkConfig->pin2_state = false;
        ret = STATUS_OK;
    }

    gIolinkConfig->config_chksum = crc8((void*)(gIolinkConfig)+2, gIolinkConfig->config_length);

    pin2_init();
    return ret;
}

/**
 * @brief Handles command line input to print a IO-Link Status output
 *
 * @param [in] cmd Pointer to the input command string
 *
 * @return Information if completed successfully
 * @retval STATUS_OK if the function execution was successful
 * @retval STATUS_CMDERROR otherwise
 */
static enum status_enum status_cmd(char *cmd){
    uart_printf("COM%d ", (int)(gIolinkConfig->TransferMode));
    uart_printf("%.1f ms\r\n", gIolinkConfig->cycle_time);
    uart_printf("IO-Link Mode: %d - %s\r\n", gIolinkProcess.DeviceMode, gIolinkProcess.DeviceModeName);
    uart_printf("Pin 2: %s Level: %s\r\n", (gIolinkConfig->pin2_mode==IN)?"DI":(gIolinkConfig->pin2_mode==NPN)?"NPN":(gIolinkConfig->pin2_mode==PNP)?"PNP":"PP",
            (DL_GPIO_readPins(GPIOA, GPIO_GRP_IOLink_RX2_PIN)?"Low":"High"));
    return STATUS_OK;
}

/**
 * @brief Initializes Pin 2 according to global configuration
 *
 */
static void pin2_init(void){
    if(gIolinkConfig->pin2_mode == IN){
        DL_GPIO_clearPins(GPIOA, GPIO_GRP_IOLink_EN2_PIN);
    }else if(gIolinkConfig->pin2_mode == NPN){
        DL_GPIO_setPins(GPIOA, GPIO_GRP_IOLink_TX2_PIN);
    }else if(gIolinkConfig->pin2_mode == PNP){
        DL_GPIO_clearPins(GPIOA, GPIO_GRP_IOLink_TX2_PIN);
    }else if(gIolinkConfig->pin2_mode == PP){
        DL_GPIO_setPins(GPIOA, GPIO_GRP_IOLink_EN2_PIN);
    }

    if(gIolinkConfig->pin2_mode == NPN | gIolinkConfig->pin2_mode == PNP){
        if(gIolinkConfig->pin2_state == true){
            DL_GPIO_setPins(GPIOA, GPIO_GRP_IOLink_EN2_PIN);
        }else{
            DL_GPIO_clearPins(GPIOA, GPIO_GRP_IOLink_EN2_PIN);
        }
    }else if(gIolinkConfig->pin2_mode == PP){
        DL_GPIO_setPins(GPIOA, GPIO_GRP_IOLink_EN2_PIN);
        if(gIolinkConfig->pin2_state == true){
            DL_GPIO_clearPins(GPIOA, GPIO_GRP_IOLink_TX2_PIN);
        }else{
            DL_GPIO_setPins(GPIOA, GPIO_GRP_IOLink_TX2_PIN);
        }
    }
}

/**
 * @brief     Cyclic function to process IO-Link stack
 *
 * @returns   STATUS_OK always, indicating that the function executed successfully
 */
enum status_enum output_iolink_cyclic(void){
    IOLD_API_vRun();
    return STATUS_OK;
}

/**
 * @brief     Initializes and configures the IO-Link driver
 *
 *
 * @param[in] config        A pointer to an initialization configuration structure
 *
 * @return    This function returns a status code indicating the result of the operation
 *
 * @retval   STATUS_OK       Successful initialization
 * @retval   STATUS_CFG_CHANGED     Configuration was modified and reset to a default state
 */
enum status_enum output_iolink_init(void *config){
    enum status_enum ret = STATUS_OK;
    gIolinkConfig = config;
    gIolinkConfig->config_length = sizeof(struct iolink_config_struct)-2;

    if(crc8(config+2, gIolinkConfig->config_length) != gIolinkConfig->config_chksum){
        ERR_LOG("IO-Link config reset");
        gIolinkConfig->cycle_time = 0.4;
        gIolinkConfig->TransferMode = IOL_eTransferMode_COM3;
        gIolinkConfig->pin2_mode = PP;
        gIolinkConfig->pin2_state = true;
        gIolinkConfig->config_chksum = crc8(config+2, gIolinkConfig->config_length);
        ret = STATUS_CFG_CHANGED;
    }

    /* configure GPIOs of second channel TIOL112 Pin 2 */
    DL_GPIO_initDigitalOutput(GPIO_GRP_IOLink_EN2_IOMUX);
    DL_GPIO_enableOutput(GPIOA, GPIO_GRP_IOLink_EN2_PIN);
    DL_GPIO_initDigitalOutput(GPIO_GRP_IOLink_TX2_IOMUX);
    DL_GPIO_enableOutput(GPIOA, GPIO_GRP_IOLink_TX2_PIN);
    DL_GPIO_initDigitalInput(GPIO_GRP_IOLink_RX2_IOMUX);

    pin2_init();

    IOLD_API_Init(gIolinkConfig, &gIolinkProcess);
    NVIC_EnableIRQ(GPIO_GRP_IOLink_INT_IRQN);
    DL_UART_Main_disableInterrupt(UART_IO_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_IO_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_IO_INST_INT_IRQN);
    return ret;

}

/**
 * @brief     Updates IO-Link process data
 *
 * @param[in] raw        New process data
 *
 * @return    This function returns a status code indicating the result of the operation
 *
 * @retval   STATUS_OK       Always
 */
enum status_enum output_iolink_set_reading(int32_t raw){
    gIolinkProcess.processData = raw;
    return STATUS_OK;
}

/**
 * @brief Executes a command from the UART interface
 *
 * @param[in] cmd The full command string to be executed (e.g. "out help"), including any optional arguments.
 * It will check commands structure and execute the function if a command is listed there.
 *
 * @return This function returns a status code indicating the result of executing the command.
 * Will return the return value of called command function.
 *
 * @retval  STATUS_CMDNOTFOUND    Command not found or unrecognized
 */
enum status_enum output_iolink_cmd(char *cmd){
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


/**
 * @brief     IRQ handler for UART connected to IO-Link transceiver
 */
extern void ioLink_UART_IO_INST_IRQHandler(void);
void UART_IO_INST_IRQHandler(void){
    ioLink_UART_IO_INST_IRQHandler();
}

#endif
