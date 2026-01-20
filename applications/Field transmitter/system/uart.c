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

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "ti_msp_dl_config.h"

#include "main.h"
#include "uart.h"
#include "system.h"
#include "adc.h"
#include "output.h"
#include "out_condition.h"
#include "adc_condition.h"
#include "identification.h"


static char gUartRxBuffer[32];
static char UartRxPos = 0;
static char gUartTxBuffer[128];
static char UartTxPos = 0;
static char UartTxLen = 0;
bool UartFlag = false;
static enum status_enum help_cmd(char *cmd);
static enum status_enum version_cmd(char *cmd);
static bool rxStringAccumulate(char byte);

struct command
{
    char *name;
    char *help;
    enum status_enum (*function) (char *cmd);
};

static struct command commands[7];
#if 0
static void uart_puts(char *str){
    while(*str != 0)
        DL_UART_Main_transmitDataBlocking(UART_CMD_INST, *(str++)); //todo: write non blocking with wfi
}
#endif

/**
 * @brief fills the command list structure with correct function pointers
 *
 */
void uart_cmd_init(void){
    int i=0;

    commands[i].name = "sys";
    commands[i].function = system_cmd;
    commands[i++].help = "configure system";
    commands[i].name = "adc";
    commands[i].function = adc_cmd;
    commands[i++].help = "configure adc subsystem";
    commands[i].name = "out";
    commands[i].function = output_cmd;
    commands[i++].help = "configure output subsystem";
    commands[i].name = "cin";
    commands[i].function = condition_adc_cmd;
    commands[i++].help = "configure adc data conditioning system";
    commands[i].name = "cout";
    commands[i].function = condition_out_cmd;
    commands[i++].help = "configure output data conditioning system";
    commands[i].name = "help";
    commands[i].function = help_cmd;
    commands[i++].help = "show this message";
    commands[i].name = "version";
    commands[i].function = version_cmd;
    commands[i++].help = "show information about software and hardware version";
}

/**
 * @brief handles help command, showing available commands
 *
 * @param[in] cmd pointer of UART input
 * @return STATUS_OK always
 */
static enum status_enum help_cmd(char *cmd){
    int i;
    for(i=0; i<sizeof(commands)/sizeof(commands[0]); i++){
        uart_printf("%-7s - 0x%x - %s\r\n", commands[i].name, commands[i].function, commands[i].help);
    }
    return STATUS_OK;
}

/**
 * @brief handles version command, showing current version
 *
 * @param[in] cmd pointer of UART input
 * @return STATUS_OK always
 */
static enum status_enum version_cmd(char *cmd){
    int i;
    uart_printf("FT Platform compile date/time %s %s\r\n", __DATE__, __TIME__);
    uart_printf("Version %s\r\n", VERSION_STR);

    uart_printf("ADC driver in use: %s\r\n", gAdcDriverName);
    uart_printf("Output driver in use: %s\r\n", OutputDriverName);

    extern struct id_func_input_map_struct gIdFuncInputMap[];
    extern struct id_func_output_map_struct gIdFuncOutputMap[];

    uart_printf("Supported ADC drivers:\r\n");
    for(i=0;i<AdcDriverCnt; i++){
        uart_printf("  %s\r\n", gIdFuncInputMap[i].adc_driver_name);
    }

    uart_printf("Supported Output drivers:\r\n");
    for(i=0;i<OutputDriverCnt; i++){
        uart_printf("  %s\r\n", gIdFuncOutputMap[i].output_driver_name);
    }

    return STATUS_OK;
}

void uart_handle_cmd(void){
    int i;
    bool found=false;
    for(i=0; i<sizeof(commands)/sizeof(commands[0]); i++){
        if(!strcmp(commands[i].name, strtok(gUartRxBuffer, " "))){
            // command found, call it
            uart_printf("ret: %d\r\n", commands[i].function(gUartRxBuffer+strlen(commands[i].name)+1));
            found = true;
            break;
        }
    }
    if(found == false) { //unknown command
        uart_printf("command not found - try help\r\n");
    }
    memset(gUartRxBuffer, 0, sizeof(gUartRxBuffer));
}

//! \brief Callback function for GUI HAL. Called when byte is received.
//!
//! \param[in] byte is the byte received from GUI Comm interface.
//!
//! \return true if end of command string was reached
static bool rxStringAccumulate(char byte)
{
    static uint16_t charCnt = 0;
    bool end = false;
    // gUartRxBuffer[UartRxPos++]
    if (charCnt >= MAX_STR_LEN_RX) { charCnt = 0; } // overflow
    switch (byte)
    {
        case '\b':  // backspace  (0x08)
            if (charCnt > 0) {
                gUartRxBuffer[UartRxPos--] = '\0';
                gUartRxBuffer[UartRxPos] = '\0';
            }
            break;

        default:
            
            break;
    }
    return end;
}

/**
 * Printf to UART
 * @param format format to print
 */
void uart_printf(const char * format, ...){
    //char buffer[128];
    va_list args;

    while(UartTxLen != 0) // there is still something sending, wait...
        asm(" wfi");

    va_start(args, format);
    UartTxLen = vsnprintf(gUartTxBuffer, sizeof(gUartTxBuffer), format, args);
    UartTxPos = 0;
    DL_UART_Main_enableInterrupt(UART_CMD_INST, DL_UART_MAIN_INTERRUPT_TX);
    va_end(args);
}

void UART_CMD_INST_IRQHandler(void)
{
    char data;
    switch (DL_UART_Main_getPendingInterrupt(UART_CMD_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            data = DL_UART_Main_receiveData(UART_CMD_INST);
            rxStringAccumulate(data);   // handling \b chars
            DL_UART_Main_transmitData(UART_CMD_INST, data);
            if(UartRxPos<sizeof(gUartRxBuffer)){
                gUartRxBuffer[UartRxPos++] = data;
            }
            if(data == '\r' || data == '\n'){
                while (DL_UART_isTXFIFOFull(UART_CMD_INST));
                DL_UART_Main_transmitData(UART_CMD_INST, '\n');
                gUartRxBuffer[UartRxPos-1] = 0;
                UartFlag = true; // this flag is checked outside of IRQ context in system_cyclic and triggers uart_handle_cmd
                UartRxPos = 0;
            }
            break;
        case DL_UART_MAIN_IIDX_TX:
            DL_UART_Main_transmitData(UART_CMD_INST, gUartTxBuffer[UartTxPos++]);
            if(UartTxPos == UartTxLen){
                DL_UART_Main_disableInterrupt(UART_CMD_INST, DL_UART_MAIN_INTERRUPT_TX);
                UartTxLen = 0;
            }
        default:
            break;
    }
}

void uart_init(void){
    NVIC_ClearPendingIRQ(UART_CMD_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_CMD_INST_INT_IRQN);
}
