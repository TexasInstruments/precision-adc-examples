/**
 * \copyright Copyright (C) 2026 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef CS_FORWARDING_H_
#define CS_FORWARDING_H_
#include <stdbool.h>
#include <stdint.h>
#include "Driver/ads125p08.h"

//**********************************************************************************
//
// Typedefs
//
//**********************************************************************************

typedef enum {
    DEVICE = 0,
    GPIO0  = 1,
    GPIO1  = 2,
    GPIO2  = 3,
    GPIO3  = 4,
    AGPIO0 = 5,
    AGPIO1 = 6,
    AGPIO2 = 7,
    AGPIO3 = 8,
    AGPIO4 = 9,
    AGPIO5 = 10,
    AGPIO6 = 11,
    AGPIO7 = 12
} GPIO_Pins;

typedef enum {
    DEVICE_ANY,
    ADS125H18,
    ADS125P08
} CS_Fwd_Device;

typedef struct {
    uint8_t device;
    uint8_t global_gpio_addr;
    uint8_t global_gpio_bits;
    uint8_t cs_fwd_addr;
    uint8_t cs_fwd_bits;
    uint8_t step_gpio_addr;
    uint8_t step_gpio_bits;
} GPIO_LookupEntry;

typedef struct {
    uint8_t *gpio_array;
    uint8_t array_size;
    CS_Fwd_Device device_type;
} CSfwdConfig;

typedef struct {
    uint8_t index_spi;
    uint8_t num_of_frames;
    uint8_t byte_crc;
    uint8_t *data_tx;
    uint8_t *data_rx;
    uint8_t data_array_size;
    uint8_t row_size;
    uint8_t col_size;
} CSfwdSendFrames;

//**********************************************************************************
//
// GPIO Lookup Table
//
//**********************************************************************************

static const GPIO_LookupEntry gpio_lookup_table[] = {
            // {device     reg address         reg mask         fwd address            fwd mask            step GPIO address             step GPIO mask           }
    /*DEVICE*/ {ADS125P08, 0,                  0,               0,                     0,                  0,                            0,                       },
    /*GPIO0 */ {ADS125P08, GPIO_CFG_ADDRESS,   GPIO0_CFG_MASK,  GPIO_FWD_CFG_ADDRESS,  GPIO0_FWD_EN_MASK,  STEPX_GPIO_DATA_OUT_ADDRESS,  STEPX_GPIO0_DAT_OUT_MASK },
    /*GPIO1 */ {ADS125P08, GPIO_CFG_ADDRESS,   GPIO1_CFG_MASK,  GPIO_FWD_CFG_ADDRESS,  GPIO1_FWD_EN_MASK,  STEPX_GPIO_DATA_OUT_ADDRESS,  STEPX_GPIO1_DAT_OUT_MASK },
    /*GPIO2 */ {ADS125P08, GPIO_CFG_ADDRESS,   GPIO2_CFG_MASK,  GPIO_FWD_CFG_ADDRESS,  GPIO2_FWD_EN_MASK,  STEPX_GPIO_DATA_OUT_ADDRESS,  STEPX_GPIO2_DAT_OUT_MASK },
    /*GPIO3 */ {ADS125P08, GPIO_CFG_ADDRESS,   GPIO3_CFG_MASK,  GPIO_FWD_CFG_ADDRESS,  GPIO3_FWD_EN_MASK,  STEPX_GPIO_DATA_OUT_ADDRESS,  STEPX_GPIO3_DAT_OUT_MASK },
    /*AGPIO0*/ {ADS125P08,  AGPIO_CFG0_ADDRESS, AGPIO0_CFG_MASK, AGPIO_FWD_CFG_ADDRESS, AGPIO0_FWD_EN_MASK, STEPX_AGPIO_DATA_OUT_ADDRESS, STEPX_AGPIO0_DAT_OUT_MASK},
    /*AGPIO1*/ {ADS125P08,  AGPIO_CFG0_ADDRESS, AGPIO1_CFG_MASK, AGPIO_FWD_CFG_ADDRESS, AGPIO1_FWD_EN_MASK, STEPX_AGPIO_DATA_OUT_ADDRESS, STEPX_AGPIO1_DAT_OUT_MASK},
    /*AGPIO2*/ {ADS125P08,  AGPIO_CFG0_ADDRESS, AGPIO2_CFG_MASK, AGPIO_FWD_CFG_ADDRESS, AGPIO2_FWD_EN_MASK, STEPX_AGPIO_DATA_OUT_ADDRESS, STEPX_AGPIO2_DAT_OUT_MASK},
    /*AGPIO3*/ {ADS125P08,  AGPIO_CFG0_ADDRESS, AGPIO3_CFG_MASK, AGPIO_FWD_CFG_ADDRESS, AGPIO3_FWD_EN_MASK, STEPX_AGPIO_DATA_OUT_ADDRESS, STEPX_AGPIO3_DAT_OUT_MASK},
    /*AGPIO4*/ {ADS125P08,  AGPIO_CFG1_ADDRESS, AGPIO4_CFG_MASK, AGPIO_FWD_CFG_ADDRESS, AGPIO4_FWD_EN_MASK, STEPX_AGPIO_DATA_OUT_ADDRESS, STEPX_AGPIO4_DAT_OUT_MASK},
    /*AGPIO5*/ {ADS125P08,  AGPIO_CFG1_ADDRESS, AGPIO5_CFG_MASK, AGPIO_FWD_CFG_ADDRESS, AGPIO5_FWD_EN_MASK, STEPX_AGPIO_DATA_OUT_ADDRESS, STEPX_AGPIO5_DAT_OUT_MASK},
    /*AGPIO6*/ {ADS125P08,  AGPIO_CFG1_ADDRESS, AGPIO6_CFG_MASK, AGPIO_FWD_CFG_ADDRESS, AGPIO6_FWD_EN_MASK, STEPX_AGPIO_DATA_OUT_ADDRESS, STEPX_AGPIO6_DAT_OUT_MASK},
    /*AGPIO7*/ {ADS125P08,  AGPIO_CFG1_ADDRESS, AGPIO7_CFG_MASK, AGPIO_FWD_CFG_ADDRESS, AGPIO7_FWD_EN_MASK, STEPX_AGPIO_DATA_OUT_ADDRESS, STEPX_AGPIO7_DAT_OUT_MASK},
};

//**********************************************************************************
//
// Example Command Arrays
//
//**********************************************************************************

/*
adc_init

*/


// initialize the ADC
static const uint8_t adc_init_rows = 12;
static const uint8_t adc_init_cols = 2;
static const uint8_t adc_init[12][2] = {
    {0xBF, 0x00},   // change to global page (page 0)
    {0xA0, 0x82},   // configure the sequencer for single sequence mode
                    // stop after complete sequence
                    // DRDY transitions once after each completed sequence
    {0xA1, 0x03},   // enable steps 0 and 1
    {0xA5, 0x01},   // enable the FIFO

    {0xBF, 0x01},   // change to step 0 (page 1)
    {0x80, 0x08},   // set AINx = AIN8-RESN
    {0x82, 0x1C},   // set VREF = internal
                    // 64 conversions
    {0x83, 0x08},   // set ODR = sinc4, OSR 1024

    {0xBF, 0x02},   // change to step 1 (page 2)
    {0x80, 0x09},   // set AINx = AIN9-RESN
    {0x82, 0x17},   // set VREF = internal
                    // 12 conversions
    {0x83, 0x09}    // set ODR = sinc4, OSR 2048
   
};
static uint8_t adc_init_out[12][2];


// start conversions
static const uint8_t adc_start_single_rows = 2;
static const uint8_t adc_start_single_cols = 2;
static const uint8_t adc_start_single[2][2] = {
    {0xBF, 0x00},   // reset to global page (page 0)
    {0x90, 0x80}    // start conversions, step INIT = 0
};
static uint8_t adc_start_single_out[2][2];


// stop conversions
static const uint8_t adc_stop_rows = 2;
static const uint8_t adc_stop_cols = 2;
static const uint8_t adc_stop[2][2] = {
    {0xBF, 0x00},   // reset to global page (page 0)
    {0x90, 0x01}    // stop conversions, step INIT = 0
};
static uint8_t adc_stop_out[2][2];


// read 24-bit data, no CRC, no STATUS
static const uint8_t adc_read_data_rows = 1;
static const uint8_t adc_read_data_cols = 3;
static const uint8_t adc_read_data[1][3] = {
    {0x00, 0x00, 0x00}
};
static uint8_t adc_read_data_out[1][3];


// read 24-bit FIFO data, no CRC, no STATUS
static const uint8_t adc_read_FIFO_rows = 1;
static const uint8_t adc_read_FIFO_cols = 3;
static const uint8_t adc_read_FIFO[1][3] = {
    {0x00, 0x0F, 0x00}
};
static uint8_t adc_read_FIFO_out[1][3];

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************

void csFwdExample(void);
void csFwdGPIOvalidate(CSfwdConfig *gpio_array);
void csFwdSetup(CSfwdConfig *gpio_array);
void csFwdEnable(void);
void csFwdSendCompleteFrame(CSfwdSendFrames *frame_data);
void csFwdSendHeaderFrame(uint8_t SPI_index, uint8_t num_frames, uint8_t CRC_byte);
void csFwdSendSingleFrame(uint8_t *CS_FWD_data_Tx, uint8_t num_bytes, uint8_t *CS_FWD_data_Rx);
void csFwdDisable(void);


#endif /* CS_FORWARDING_H_ */