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
static enum status_enum stream_cmd(char *cmd);

static bool gStreaming = false;


/*
 * calibration coeffients from Coeffient_Generation.xlsx
 * Should be moved to config memory later
 */
static int32_t h0 = 27340;
static int32_t h1 = 0;
static int32_t h2 = 0;
static int32_t h3 = 0;
static int32_t g0 = 18135484;
static int32_t g1 = 0;
static int32_t g2 = 0;
static int32_t g3 = 0;
static int32_t n0 = 421023099;
static int32_t n1 = 0;
static int32_t n2 = 0;
static int32_t n3 = 0;
static int32_t m0 = 0;
static int32_t m1 = 0;
static int32_t m2 = 0;
static int32_t m3 = 0;
static int32_t P_offset = -1769446;
static int32_t T_offset = 0;

static float h0n;
static float h1n;
static float h2n;
static float h3n;
static float g0n;
static float g1n;
static float g2n;
static float g3n;
static float n0n;
static float n1n;
static float n2n;
static float n3n;
static float m0n;
static float m1n;
static float m2n;
static float m3n;

struct command
{
    char *name;
    char *help;
    enum status_enum (*function) (char *cmd);
};


static struct command commands[] = {
    {"help", "show this message", help_cmd},
    {"stream", "starts/stops streaming raw readings", stream_cmd},
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
    uart_printf("All commands must start with cin \r\n");
    for(i=0; i<sizeof(commands)/sizeof(commands[0]); i++){
        uart_printf("%-10s - 0x%x - %s\r\n", commands[i].name, commands[i].function, commands[i].help);
    }
    return STATUS_OK;
}

/**
 * @brief Toggles gStreaming state.
 *
 * This function flips the `gStreaming` flag when called, turning it on or off.
 * The current value of `gStreaming` is used to stream the ADC readings to UART as well or not
 *
 * @param[in] cmd Unused command parameter; kept for consistency with other functions.
 *
 * @return STATUS_OK regardless of the new gStreaming state.
 */
static enum status_enum stream_cmd(char *cmd) {
    if (gStreaming)
        gStreaming = false;
    else
        gStreaming = true;
    return STATUS_OK;
}

/**
 * @brief Not used
 *
 * @return
 */
enum status_enum condition_press_temp_adc_cyclic(void){
    return STATUS_OK;
}

/**
 * @brief initializes internal coefficients to to the compensation
 *
 * @param config not used at the moment
 * @return STATUS_OK alwys
 */
enum status_enum condition_press_temp_adc_init(void *config){
    /* Normalize compensation coefficents */
    h0n = h0 / 1073741824.0; /* divide by 2^30 or right shift >>30 */
    h1n = h1 / 1073741824.0;
    h2n = h2 / 1073741824.0;
    h3n = h3 / 1073741824.0;
    g0n = g0 / 1073741824.0;
    g1n = g1 / 1073741824.0;
    g2n = g2 / 1073741824.0;
    g3n = g3 / 1073741824.0;
    n0n = n0 / 1073741824.0;
    n1n = n1 / 1073741824.0;
    n2n = n2 / 1073741824.0;
    n3n = n3 / 1073741824.0;
    m0n = m0 / 1073741824.0;
    m1n = m1 / 1073741824.0;
    m2n = m2 / 1073741824.0;
    m3n = m3 / 1073741824.0;

    return STATUS_OK;
}

/**
 * @brief Pressure sensor temperature compensation and scaling for DAC
 *
 * @param [in] in pointer to input values, raw temperature and pressure
 * @param [out]  out pointer where to stare calculated value to
 * @return STATUS_OK always
 */
enum status_enum condition_press_temp_adc_process(int32_t *in, float *out){
    uint32_t DAC = 0;
    int32_t PADC = in[0];
    int32_t TADC = in[1];

    float PADCn = 0;
    float TADCn = 0;
    float PADCn_sq = 0;
    float TADCn_sq = 0;
    float PADCn_cb = 0;
    float TADCn_cb = 0;

    float term1 = 0;
    float term2 = 0;
    float term3 = 0;
    float term4 = 0;

    float DACn = 0;

    /*convert PADC and TADC from unsigned 32 bit integer to float, add offset and normalize */
    PADCn = (PADC + P_offset) / 1073741824.0; /* scale */
    TADCn = (TADC + T_offset) / 1073741824.0;

    /* calculate PADC^2 and TADC^2 */
    PADCn_sq = PADCn * PADCn;
    TADCn_sq = TADCn * TADCn;

    /* calculate PADC^3 and TADC^3*/
    PADCn_cb = PADCn_sq * PADCn;
    TADCn_cb = TADCn_sq * TADCn;

    /* calculate polynomial terms */
    term1 = h0n + h1n * TADCn + h2n * TADCn_sq + h3n * TADCn_cb;
    term2 = (g0n + g1n * TADCn + g2n * TADCn_sq + g3n * TADCn_cb) * PADCn;
    term3 = (n0n + n1n * TADCn + n2n * TADCn_sq + n3n * TADCn_cb) * PADCn_sq;
    term4 = (m0n + m1n * TADCn + m2n * TADCn_sq + m3n * TADCn_cb) * PADCn_cb;

    /* combine terms to calculate DACn */
    DACn = term1 + term2 + term3 + term4;

    /* Multiply normalized DACn */
    DAC = DACn*1073741824.0 ;

    *out = DAC;

    if(gStreaming)
        uart_printf("%d\r\n", (int)(*out));

    return STATUS_OK;
}

/**
 * @brief handles UART commands
 *
 * @param[in] cmd pointer of UART input
 * @return return value of command function, STATUS_CMDNOTFOUND if command is not available, STATUS_ERR in case of generic error
 */
enum status_enum condition_press_temp_adc_cmd(char *cmd){
    int i;
    char *subcmd = strtok(cmd, " ");
    bool found=false;
    for(i=0; i<sizeof(commands)/sizeof(commands[0]); i++){
        if(!strcmp(commands[i].name, subcmd)){
            /* command found, call it */
            return(commands[i].function(cmd+strlen(commands[i].name)+1));
            found = true;
            break;
        }
    }
    if(found == false) { /* unknown command */
        uart_printf("command not found - try cin help\r\n");
        return STATUS_CMDNOTFOUND;
    }
    return STATUS_ERR; /* should never get here */
}


