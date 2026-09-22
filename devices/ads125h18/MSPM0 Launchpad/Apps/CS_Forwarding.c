/**

 *
 * @brief This file contains all basic communication and device setup.
 * @warning This software utilizes TI Drivers
 *
 * @copyright Copyright (C) 2026 Texas Instruments Incorporated - http://www.ti.com/
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


#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "Apps/CS_Forwarding.h"
#include "Driver/ads125h18.h"
#include "Driver/hal.h"

/**
 * @brief Example to demonstrate CS FWD mode using two ADCs
 * @details Demonstrate CS FWD mode operation by using GPIO2 from one ADC as the CS for a second ADC
            Communicates with both ADCs using CS FWD mode. Sequentially (ADC1 then ADC2) performs initialization,
            conversion start, and data output (user must collect data e.g. using logic analyzer)
 * @note This is an example implementation; arrays and configurations should be customized per application
 * @return void
 */
void csFwdExample(void)
{

    resetDevice();       // issue an SPI reset command and wait for reset.
    
    /* select GPIOs used for CS forwarding:
       ADS125H18 options = {DEVICE, GPIO0, GPIO1, GPIO2, GPIO3} */

    uint8_t cs_fwd_array[] = {GPIO2};   // CLKIN pin on ADS125H18
    uint8_t array_size = sizeof(cs_fwd_array) / sizeof(cs_fwd_array[0]);

    // configure the GPIOs for the selected device with the CS FWD array
    CSfwdConfig config = {
        .gpio_array = cs_fwd_array,
        .array_size = array_size,
        .device_type = ADS125H18
    };

    // struct to initialize the ADC
    CSfwdSendFrames init_frame = {
        .index_spi = DEVICE,                        // local device
        .num_of_frames = adc_init_rows,             // number of frames
        .byte_crc = 0x00,                           // 0x00 if CRC disabled
        .data_tx = (uint8_t *)adc_init,             // transmit data
        .data_rx = (uint8_t *)adc_init_out,         // receive buffer
        .row_size = adc_init_rows,                  // number of rows in the array
        .col_size = adc_init_cols                   // number of columns in the array
    };

    // struct to start conversions (single or multiple, depends on sequencer mode)
    CSfwdSendFrames start_single = {
        .index_spi = DEVICE,                        // local device
        .num_of_frames = adc_start_single_rows,     // number of frames
        .byte_crc = 0x00,                           // 0x00 if CRC disabled
        .data_tx = (uint8_t *)adc_start_single,     // transmit data
        .data_rx = (uint8_t *)adc_start_single_out, // receive buffer
        .row_size = adc_start_single_rows,          // number of columns in the array
        .col_size = adc_start_single_cols           
    };
    
    // struct to read data from the ADC using NOP
    CSfwdSendFrames read_data = {
        .index_spi = DEVICE,                        // local device
        .num_of_frames = adc_read_data_rows,        // number of frames
        .byte_crc = 0x00,                           // 0x00 if CRC disabled
        .data_tx = (uint8_t *)adc_read_data,        // transmit data
        .data_rx = (uint8_t *)adc_read_data_out,    // receive buffer
        .row_size = adc_read_data_rows,             // number of rows in the array
        .col_size = adc_read_data_cols              // number of columns in the array
    };

    // struct to read from the FIFO (different command = 0x0F)
    CSfwdSendFrames read_FIFO = {
        .index_spi = DEVICE,                        // local device
        .num_of_frames = adc_read_FIFO_rows,        // number of frames
        .byte_crc = 0x00,                           // 0x00 if CRC disabled
        .data_tx = (uint8_t *)adc_read_FIFO,        // transmit data
        .data_rx = (uint8_t *)adc_read_FIFO_out,    // receive buffer
        .row_size = adc_read_FIFO_rows,             // number of rows in the array
        .col_size = adc_read_FIFO_cols              // number of columns in the array
    };

    
    /*use one of these two functions to either setup or validate CS FWD */
    csFwdSetup(&config);            // Use this function to manually configure GPIO pins for CS FWD mode operation
    // csFwdGPIOvalidate(&config);  // Use this function to validate the CS FWD configuration registers programmed by the _RegisterDetails.h file
    
    // enable CS FWD mode
    csFwdEnable();

    // initialize the first ADC, start conversions, read data
    csFwdSendCompleteFrame(&init_frame);    // initialize ADC
    csFwdSendCompleteFrame(&start_single);  // start conversions
    
    delay_ms(10); // about how long this sequence takes to finish <-- adjust accordingly to your settings

    // read FIFO data from the local device
    for (uint8_t i = 0; i <= 76; i++) { // 76 is specific to this example sequence, where we read 64 conversions in step 0 and and 12 conversions in step 1
         csFwdSendCompleteFrame(&read_FIFO);
    }
    
    // Use GPIO2 to control CS on the second ADC
    init_frame.index_spi = GPIO2;
    start_single.index_spi = GPIO2;
    read_FIFO.index_spi = GPIO2;
    
    // initialize the second ADC, start conversions, read data
    csFwdSendCompleteFrame(&init_frame);    // initialize ADC
    csFwdSendCompleteFrame(&start_single);  // start conversions

    delay_ms(10); // about how long this sequence takes to finish <-- adjust accordingly to your settings

    // read FIFO data from the local device
    for (uint8_t i = 0; i <= 76; i++) { // 76 is specific to this example sequence, where we read 64 conversions in step 0 and and 12 conversions in step 1
         csFwdSendCompleteFrame(&read_FIFO);
    }
    
    csFwdDisable();

    return;
}

/**
 * @brief Use this function to validate the CS FWD configuration registers programmed by the _RegisterDetails.h file
 * @param gpio_pin_id GPIO pin identifier to validate
 * @return void
 * @note Checks global GPIO, CS FWD enable, and step GPIO register bits
 */
void csFwdGPIOvalidate(CSfwdConfig *gpio_array)
{

    // determine which step is currently set to STEP INIT
    uint8_t stepInit = ((registerMap[GENERAL_SETTINGS][CONVERSION_CTRL_ADDRESS] << 1) >> 3) + 1;    
    
    // loop through all elements in the gpio_array
    for (uint8_t i = 0; i < gpio_array->array_size; i++) {

        // Look up the GPIO entry in the lookup table
        GPIO_LookupEntry entry = gpio_lookup_table[gpio_array->gpio_array[i]];

        // Read values from the three register addresses associated with this GPIO
        uint8_t global_gpio_value = readSingleRegister(PAGE_POINTER_DEFAULT, entry.global_gpio_addr).Register_data;
        uint8_t cs_fwd_value = readSingleRegister(PAGE_POINTER_DEFAULT, entry.cs_fwd_addr).Register_data;
        uint8_t step_gpio_value = readSingleRegister(stepInit, entry.step_gpio_addr).Register_data;

        // Confirm that the masked bits equal 10b (highest bit set, lower bits clear)
        uint8_t masked_value = global_gpio_value & entry.global_gpio_bits;
        uint8_t expected_value = (entry.global_gpio_bits & -entry.global_gpio_bits) << 1;
        if (masked_value != expected_value) {
            // USER TODO: what to do if global_gpio register validation fails
            return;
        }

        if ((cs_fwd_value & entry.cs_fwd_bits) != entry.cs_fwd_bits) {
            // USER TODO: what to do if cs_fwd register validation fails
            return;
        }

        if ((step_gpio_value & entry.step_gpio_bits) != entry.step_gpio_bits) {
            // USER TODO: what to do if step_gpio register validation fails
            return;
        }
    }
    
    // GPIO validation passed - all registers have expected bit values set
    return;
}

/**
 * @brief Use this function to manually configure GPIO pins for CS FWD mode operation
 * @param gpio_array Pointer to CSfwdConfig structure containing GPIO array and device type
 * @return void
 * @note Sets GPIO output state high on the INIT step, configures as digital output (10b), and sets the desired GPIOs to CS FWD mode
 */
void csFwdSetup(CSfwdConfig *gpio_array)
{
           
    // determine which step is currently set to STEP INIT
    uint8_t stepInit = ((registerMap[GENERAL_SETTINGS][CONVERSION_CTRL_ADDRESS] << 1) >> 3) + 1;
    
    // loop through all elements in the gpio_array
    for (uint8_t i = 0; i < gpio_array->array_size; i++) {

        // Look up the GPIO entry in the lookup table
        GPIO_LookupEntry entry = gpio_lookup_table[gpio_array->gpio_array[i]];
        
        // make sure that we are not trying to write to an out-of-bounds address for that specific device
        if (gpio_array->device_type != entry.device && entry.device != 0)
        {
            continue;
        }
        
        // Set GPIOn output state high on the step page pointed to by STEP_INIT by setting STEPx_GPIOn_DAT_OUT = 1b
        uint8_t gpio_current_step = registerMap[stepInit][entry.step_gpio_addr] | entry.step_gpio_bits;
        writeSingleRegister(stepInit, entry.step_gpio_addr, gpio_current_step);

        // Configure GPIOn as digital output by setting GPIO_CFG = 10b on global page
        uint8_t gpio_current_cfg = (registerMap[GENERAL_SETTINGS][entry.global_gpio_addr] & ~entry.global_gpio_bits) | ((entry.global_gpio_bits & -entry.global_gpio_bits) << 1);
        writeSingleRegister(PAGE_POINTER_DEFAULT, entry.global_gpio_addr, gpio_current_cfg);
        
        // Select GPIO to be CS-FWD by setting GPIO_FWD_EN = 1b on global page
        uint8_t gpio_current_fwd_cfg = registerMap[GENERAL_SETTINGS][entry.cs_fwd_addr] | entry.cs_fwd_bits;
        writeSingleRegister(PAGE_POINTER_DEFAULT, entry.cs_fwd_addr, gpio_current_fwd_cfg);
    }

    return;
}

/**
 * @brief Enable CS FWD mode on the ADC
 * @return void
 * @note Preserves existing timeout settings in CS_FWD_CFG register
 */
void csFwdEnable(void)
{

    //enable the CS FWD mode using 0x010111 and preserve the user-selected timeout
    writeSingleRegister(PAGE_POINTER_DEFAULT, CS_FWD_CFG_ADDRESS, 0x5C | registerMap[GENERAL_SETTINGS][CS_FWD_CFG_ADDRESS]);
    
    return;
}

void csFwdSendCompleteFrame(CSfwdSendFrames *frame_data)
{

    // send the header frame
    csFwdSendHeaderFrame(frame_data->index_spi, frame_data->num_of_frames, frame_data->byte_crc);
    
    // send all of the data
    for (uint8_t i = 0; i < frame_data->row_size; i++) {

        csFwdSendSingleFrame(&frame_data->data_tx[i * frame_data->col_size], frame_data->col_size, &frame_data->data_rx[i * frame_data->col_size]);
    }
}

/**
 * @brief Send the header frame to initiate CS FWD communication sequence
 * @param SPI_index Device to address (0 = local device)
 * @param num_frames Number of frames - 1 to send e.g. write 011b (3 in decimal) to send 4 frames
 * @param CRC_byte CRC byte for validation, or 0x00 if CRC disabled
 * @return void
 * @note Validates that transmitted and received header bytes match
 */
void csFwdSendHeaderFrame(uint8_t SPI_index, uint8_t num_frames, uint8_t CRC_byte)
{
    uint8_t headerTx[3] = {0};
    uint8_t headerRx[3] = {0};

    // header frame is always 3x bytes
    headerTx[0] = (SPI_index << 4) | (num_frames - 1);      // prepare the first byte = SPI index (bits 7:4) + number of frames (bits 3:0)
    headerTx[1] = CRC_byte;                                 // if device CRC is enabled, otherwise this is a don't care
    headerTx[2] = 0x00;                                     // don't care

    // always sending and receiving 3x bytes
    spiSendReceiveArrays(headerTx, headerRx, 3);
    
    // If the first byte on the header does not match the second byte on the received data, there is an error (this is independent of CRC check)
    if (headerTx[0] != headerRx[1])
    {
        // USER TODO: what to do if this validation step fails
        return; 
    }

    return;
}

/**
 * @brief Send a single data frame via CS FWD mode
 * @param CS_FWD_data_Tx Pointer to data array to transmit
 * @param num_bytes Number of bytes in data array
 * @param CS_FWD_data_Rx Pointer to receive buffer (caller allocated, must be at least num_bytes)
 * @return void
 * @note Caller is responsible for allocating and managing receive buffer
 */
void csFwdSendSingleFrame(uint8_t *CS_FWD_data_Tx, uint8_t num_bytes, uint8_t *CS_FWD_data_Rx)
{
    if (CS_FWD_data_Rx == NULL)
    {
        // USER TODO: what to do if this validation step fails
        return;
    }

    spiSendReceiveArrays(CS_FWD_data_Tx, CS_FWD_data_Rx, num_bytes);
    return;
}

/**
 * @brief Disable CS FWD mode on the ADC
 * @return void
 * @note Clears CS_FWD_CFG register and resets timeout to 0
 */
void csFwdDisable(void)
{

    static const uint8_t disable_cs_fwd_rows = 2;
    static const uint8_t disable_cs_fwd_cols = 2;
    static const uint8_t disable_cs_fwd[2][2] = {
        {PAGE_POINTER_ADDRESS + 0x80, PAGE_POINTER_DEFAULT},
        {CS_FWD_CFG_ADDRESS + 0x80, 0x00}
    };
    static uint8_t disable_cs_fwd_out[2][2];
    
    csFwdSendHeaderFrame(0x00, 0x02, 0x00);            
    csFwdSendSingleFrame((uint8_t *)&disable_cs_fwd[0][0], 2, &disable_cs_fwd_out[0][0]);  // row 0
    csFwdSendSingleFrame((uint8_t *)&disable_cs_fwd[1][0], 2, &disable_cs_fwd_out[1][0]);  // row 1

    return;
 }
