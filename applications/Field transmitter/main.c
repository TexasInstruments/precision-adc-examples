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

#include <ti/driverlib/m0p/dl_interrupt.h>
#include "ti_msp_dl_config.h"

#include "adc.h"
#include "adc_condition.h"
#include "out_condition.h"
#include "output.h"
#include "system/flash.h"
#include "system/uart.h"
#include "identification.h"
#include "system/system.h"

#include <string.h>

/**
 * @brief The main entry point of the program.
 *
 * This function initializes the system, sets up the communication interfaces,
 * and enters an infinite loop where it periodically checks the ADC status
 * and processes sensor data to update outputs.
 */
int main(void)
{
    int32_t adc_raw[2];
    float sensor_value;
    int32_t out_raw;
    enum status_enum status;

    SYSCFG_DL_init();

    while(gSysTick < 10)
        asm("wfi");

    uart_init();
    uart_printf("FT Platform %s %s\r\n", __DATE__, __TIME__);
    load_id();
    uart_cmd_init();
    config_init();

    status = adc_init(&gAdcConfigRam);
    if(status == STATUS_CFG_CHANGED) config_save();
    else if (status != STATUS_OK) ERR_LOG("fault %d", status);

    status = condition_adc_init(&aAdcCondConfigRam);
    if(status == STATUS_CFG_CHANGED) config_save();
    else if (status != STATUS_OK) ERR_LOG("fault %d", status);

    status = condition_out_init(&gOutCondConfigRam);
    if(status == STATUS_CFG_CHANGED) config_save();
    else if (status != STATUS_OK) ERR_LOG("fault %d", status);

    status = output_init(&gOutputConfigRam);
    if(status == STATUS_CFG_CHANGED) config_save();
    else if (status != STATUS_OK) ERR_LOG("fault %d", status);

    uart_printf("initialized...\r\n");

    while (1) {
        if(gMainLoop){
            status = system_cyclic();
            if(status != STATUS_OK) ERR_LOG("fault %d", status);

            status = adc_cyclic();
            if(status != STATUS_OK) ERR_LOG("fault %d", status);

            status = condition_adc_cyclic();
            if(status != STATUS_OK) ERR_LOG("fault %d", status);

            status = condition_out_cyclic();
            if(status != STATUS_OK) ERR_LOG("fault %d", status);

            status = output_cyclic();
            if(status != STATUS_OK) ERR_LOG("fault %d", status);

            status = adc_get_reading(&adc_raw[0]);
            if(status == STATUS_OK){
                status = condition_adc_process(&adc_raw[0], &sensor_value);
                if(status != STATUS_OK) ERR_LOG("fault %d", status);

                status = condition_out_process(sensor_value, &out_raw);
                if(status != STATUS_OK) ERR_LOG("fault %d", status);

                status = output_set_reading(out_raw);
                if(status != STATUS_OK) ERR_LOG("fault %d", status);

            }else if (status != STATUS_NOCHANGE){
                ERR_LOG("fault %d", status);
            }
            gMainLoop = false;
        }
        asm("wfi");
    }
}
