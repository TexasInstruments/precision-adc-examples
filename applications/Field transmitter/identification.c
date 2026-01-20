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

#include "identification.h"
#include "system/clock.h"

#include "adc.h"
#include "adc/ads122s14.h"
#include "adc/ads122s14_ptx_daisychain.h"
#include "adc/adc_example.h"

#include "adc_condition.h"
#include "condition/adc_condition_example.h"
#include "condition/pressure_temp_adc_condition.h"

#include "out_condition.h"
#include "condition/out_condition_example.h"
#include "condition/out_condition_offset_slope_uint16.h"

#include "output.h"
#include "output/output_example.h"
#ifdef USEIOLINK
#include "output/output_io-link.h"
#endif
#include "output/output_afe881.h"
#include "output/output_afe881.h"


struct id_idx_map_struct{
    int id;
    int idx;
};

/** stores the pointer to the name of the selected driver **/
char *gAdcDriverName;

/** stores number of drivers **/
int AdcDriverCnt;

/** stores the pointer to the name of the selected driver **/
char *OutputDriverName;

/** stores number of supported output drivers **/
int OutputDriverCnt;

struct id_func_input_map_struct gIdFuncInputMap[] = {
    {
     .adc_init = adc_example_init,
     .adc_cyclic = adc_example_cyclic,
     .adc_get_reading = adc_example_get_reading,
     .adc_cmd = adc_example_cmd,
     .adc_driver_name = "Example driver",

     .condition_adc_init = condition_example_adc_init,
     .condition_adc_cyclic = condition_example_adc_cyclic,
     .condition_adc_process = condition_example_adc_process,
     .condition_adc_cmd = condition_example_adc_cmd,
    },
    {
     .adc_init = ads122s14_ptx_daisychain_init,
     .adc_cyclic = ads122s14_ptx_daisychain_cyclic,
     .adc_get_reading = ads122s14_ptx_daisychain_get_reading,
     .adc_cmd = ads122s14_ptx_daisychain_cmd,
     .adc_driver_name = "ADS122S14 daisy chain driver",


     .condition_adc_init = condition_press_temp_adc_init,
     .condition_adc_cyclic = condition_press_temp_adc_cyclic,
     .condition_adc_process = condition_press_temp_adc_process,
     .condition_adc_cmd = condition_press_temp_adc_cmd,
    }
};

#ifdef TEST
struct id_idx_map_struct gIdIdxInputMap[] = {
       {0x00, 0}, // nothing connected
       {0x40, 0}, // nothing
};
#else
struct id_idx_map_struct gIdIdxInputMap[] = {
       {0x00, 0}, // nothing connected
       {0x40, 1}, // ads122s14 pressure sensing daisy chain
};
#endif
struct id_func_output_map_struct gIdFuncOutputMap[] = {
    { /* nothing */
     .condition_out_init = condition_example_out_init,
     .condition_out_cyclic = condition_example_out_cyclic,
     .condition_out_process = condition_example_out_process,
     .condition_out_cmd = condition_example_out_cmd,

     .output_init = output_example_init,
     .output_cyclic = output_example_cyclic,
     .output_set_reading = output_example_set_reading,
     .output_cmd = output_example_cmd,
     .output_driver_name = "Example driver",

     .cpu_clock_init = cpu_clock_init_32m,
    },
    { /* AFE88x */
    .condition_out_init = condition_offset_slope_uint16_out_init,
    .condition_out_cyclic = condition_offset_slope_uint16_out_cyclic,
    .condition_out_process = condition_offset_slope_uint16_out_process,
    .condition_out_cmd = condition_offset_slope_uint16_out_cmd,

    .output_init = output_afe881h1_init,
    .output_cyclic = output_afe881h1_cyclic,
    .output_set_reading = output_afe881h1_set_reading,
    .output_cmd = output_afe881h1_cmd,
    .output_driver_name = "AFE881/2H1 driver",

    .cpu_clock_init = cpu_clock_init_32m,
    },
    { /* TIOL112 */
    .condition_out_init = condition_example_out_init,
    .condition_out_cyclic = condition_example_out_cyclic,
    .condition_out_process = condition_example_out_process,
    .condition_out_cmd = condition_example_out_cmd,
#ifdef USEIOLINK
    .output_init = output_iolink_init,
    .output_cyclic = output_iolink_cyclic,
    .output_set_reading = output_iolink_set_reading,
    .output_cmd = output_iolink_cmd,
    .output_driver_name = "IO-Link driver",
#else
    .output_init = output_example_init,
    .output_cyclic = output_example_cyclic,
    .output_set_reading = output_example_set_reading,
    .output_cmd = output_example_cmd,
    .output_driver_name = "Example driver",
#endif
    .cpu_clock_init = cpu_clock_init_80m,
    }
};

struct id_idx_map_struct gIdIdxOutputMap[] = {
       {0x00, 0}, //nothing connected
       {0x40, 1}, //AFE88x
       {0x80, 2}, //TIOL
};

/**
 * @brief converts board id to array index
 *
 * @param map[in] pointer to mapping structure
 * @param id[in] id read out from hardware
 * @return index odf mapping array correlating to hardware
 */
int get_idx(struct id_idx_map_struct *map, int id){
    int i;

    for(i=0;i<sizeof(gIdIdxOutputMap)/sizeof(gIdIdxOutputMap[0]); i++){
        if(map[i].id == id)
            return map[i].idx;
    }
    return 0;
}

/**
 * @brief reads hardware id from input and output board and fill function pointers for all processing function with the funtions correlating the connected hardware
 *
 */
void load_id(void){
    static const DL_SPI_Config gSPI_0_config = {
        .mode        = DL_SPI_MODE_CONTROLLER,
        .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA0,
        .parity      = DL_SPI_PARITY_NONE,
        .dataSize    = DL_SPI_DATA_SIZE_8,
        .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
        .chipSelectPin = DL_SPI_CHIP_SELECT_0,
    };

    static const DL_SPI_ClockConfig gSPI_0_clockConfig = {
        .clockSel    = DL_SPI_CLOCK_BUSCLK,
        .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
    };

    DL_SPI_setClockConfig(SPI_0_INST, (DL_SPI_ClockConfig *) &gSPI_0_clockConfig);
    DL_SPI_init(SPI_0_INST, (DL_SPI_Config *) &gSPI_0_config);
    DL_SPI_setBitRateSerialClockDivider(SPI_0_INST, 3);
    DL_SPI_setFIFOThreshold(SPI_0_INST, DL_SPI_RX_FIFO_LEVEL_1_2_FULL, DL_SPI_TX_FIFO_LEVEL_1_2_EMPTY);
    DL_SPI_enable(SPI_0_INST);

    DL_GPIO_clearPins(GPIO_ID_PORT, GPIO_ID_ID0_PIN); // load id into shift reg
    DL_GPIO_setPins(GPIO_ID_PORT, GPIO_ID_ID0_PIN); // and set to shift
    DL_SPI_transmitDataBlocking8(SPI_0_INST, 0x00);
    int adcid = DL_SPI_receiveDataBlocking8(SPI_0_INST);

    static const DL_SPI_Config gSPI_1_config = {
        .mode        = DL_SPI_MODE_CONTROLLER,
        .frameFormat = DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA0,
        .parity      = DL_SPI_PARITY_NONE,
        .dataSize    = DL_SPI_DATA_SIZE_8,
        .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
        .chipSelectPin = DL_SPI_CHIP_SELECT_0,
    };

    static const DL_SPI_ClockConfig gSPI_1_clockConfig = {
        .clockSel    = DL_SPI_CLOCK_BUSCLK,
        .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
    };

    DL_SPI_setClockConfig(SPI_1_INST, (DL_SPI_ClockConfig *) &gSPI_1_clockConfig);
    DL_SPI_init(SPI_1_INST, (DL_SPI_Config *) &gSPI_1_config);
    DL_SPI_setBitRateSerialClockDivider(SPI_1_INST, 3);
    DL_SPI_setFIFOThreshold(SPI_1_INST, DL_SPI_RX_FIFO_LEVEL_1_2_FULL, DL_SPI_TX_FIFO_LEVEL_1_2_EMPTY);
    DL_SPI_enable(SPI_1_INST);

    DL_GPIO_clearPins(GPIO_ID_PORT, GPIO_ID_ID0_PIN); // load id into shift reg
    DL_GPIO_setPins(GPIO_ID_PORT, GPIO_ID_ID0_PIN); // and set to shift
    DL_SPI_transmitDataBlocking8(SPI_1_INST, 0x00);
    int outputid = DL_SPI_receiveDataBlocking8(SPI_1_INST);

    adc_init = gIdFuncInputMap[get_idx(gIdIdxInputMap, adcid)].adc_init;
    adc_cyclic = gIdFuncInputMap[get_idx(gIdIdxInputMap, adcid)].adc_cyclic;
    adc_get_reading = gIdFuncInputMap[get_idx(gIdIdxInputMap, adcid)].adc_get_reading;
    adc_cmd = gIdFuncInputMap[get_idx(gIdIdxInputMap, adcid)].adc_cmd;
    gAdcDriverName = gIdFuncInputMap[get_idx(gIdIdxInputMap, adcid)].adc_driver_name;
    AdcDriverCnt = sizeof(gIdFuncInputMap)/sizeof(gIdFuncInputMap[0]);

    condition_adc_init = gIdFuncInputMap[get_idx(gIdIdxInputMap, adcid)].condition_adc_init;
    condition_adc_cyclic = gIdFuncInputMap[get_idx(gIdIdxInputMap, adcid)].condition_adc_cyclic;
    condition_adc_process = gIdFuncInputMap[get_idx(gIdIdxInputMap, adcid)].condition_adc_process;
    condition_adc_cmd = gIdFuncInputMap[get_idx(gIdIdxInputMap, adcid)].condition_adc_cmd;

    condition_out_init = gIdFuncOutputMap[get_idx(gIdIdxOutputMap, outputid)].condition_out_init;
    condition_out_cyclic = gIdFuncOutputMap[get_idx(gIdIdxOutputMap, outputid)].condition_out_cyclic;
    condition_out_process = gIdFuncOutputMap[get_idx(gIdIdxOutputMap, outputid)].condition_out_process;
    condition_out_cmd = gIdFuncOutputMap[get_idx(gIdIdxOutputMap, outputid)].condition_out_cmd;

    output_init = gIdFuncOutputMap[get_idx(gIdIdxOutputMap, outputid)].output_init;
    output_cyclic = gIdFuncOutputMap[get_idx(gIdIdxOutputMap, outputid)].output_cyclic;
    output_set_reading = gIdFuncOutputMap[get_idx(gIdIdxOutputMap, outputid)].output_set_reading;
    output_cmd = gIdFuncOutputMap[get_idx(gIdIdxOutputMap, outputid)].output_cmd;
    OutputDriverName = gIdFuncOutputMap[get_idx(gIdIdxOutputMap, outputid)].output_driver_name;
    OutputDriverCnt = sizeof(gIdFuncOutputMap)/sizeof(gIdFuncOutputMap[0]);

    gIdFuncOutputMap[get_idx(gIdIdxOutputMap, outputid)].cpu_clock_init();

    DL_GPIO_clearPins(GPIO_ID_PORT, GPIO_ID_ID0_PIN); // configure SPI mux to communicate with connected devices

}
