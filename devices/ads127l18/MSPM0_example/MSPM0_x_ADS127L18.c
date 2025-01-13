/*
 * Copyright (c) 2024, Texas Instruments Incorporated
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

#include "hal.h"
#include "ti_msp_dl_config.h"
#include "ads127l18.h"
//#include "stdlib.h"
//#include <cassert>
//#include <cstdint>

/************* USER INPUTS (Start) *************/
/*
 * Number of bytes allocated for the ADS127L18 Channel Data
 * The packet will be transmitted by the SPI peripheral using DMA
 * This value should be divisible by 32
 */
#define M0_MEMORY_BYTES (512)

// Speed mode selection:
// 0 = Low-speed mode (f_CLK = 3.2 MHz)
// 1 = Mid-speed mode (f_CLK = 12.8 MHz)
// 2 = High-speed mode (f_CLK = 25.6 MHz)
// 3 = Max-speed mode (f_CLK = 32.768 MHz)
#define speed_mode (2)

// Data port time division multiplexing (TDM): 
// Number of DOUT pins = 1, 2, 4, 8
#define DP_TDM (1)

// Data port DCLK frequency divider: 1, 2, 4, 8
#define DCLK_DIV (2)

// ADC clock selection: 
// 0 = Internal Oscillator (25.6 MHz)
// 1 = External Clock 
#define ADC_CLK_SEL (1)

// Main clock frequency can be any value from 500000 to 33660000, or up to the Speed mode limit (refer to datasheet)
#if(ADC_CLK_SEL == 0) 
#define f_CLK 25600000 // Internal Oscillator Clock Frequency
#else 
#define f_CLK 32768000  //External CLK freq
#endif

// ADC clock divider: 1, 2, 3, 4, 8
#define ADC_CLK_DIV 8
 
// Oversampling Ratio: 32, 64, 128, 256, 512, 1024, 2048, 4096
// All channels set to same OSR value
// Only Wideband filter supported, but code can be easily modified to support all filter modes if needed.
#define OSR 64

/************* USER INPUTS (End) *************/

// M0 memory array
#if DP_TDM == 1
#define TDM_factor 8
#define RX_BUFFER_SIZE (M0_MEMORY_BYTES)
uint8_t gRxPacket[RX_BUFFER_SIZE];
uint8_t total_channelBytes = 24;     //channel outputs a 24-bit data packet (3 * 8 bytes)
#endif

#if DP_TDM == 2
#define TDM_factor 4
#define RX_BUFFER_SIZE (M0_MEMORY_BYTES / 2)
uint8_t gRxPacket[RX_BUFFER_SIZE];
uint8_t gRxPacket2[RX_BUFFER_SIZE];
uint8_t total_channelBytes = 12;     //channel outputs a 24-bit data packet (3 * 4 bytes)
#endif


// Flag to break out of while loop when the ADC conversion data has been received
bool data_received_flag = false;

// Indexes for gRxPacket
uint16_t gRxPacket_index = 0;
uint16_t gRxPacket2_index = 0;

// variables for DMA
volatile bool gDMARXDataTransferred1, gDMARXDataTransferred2;

// Helper array for 24-bit to 32-bit conversion function
uint8_t save_buffer[3];

// helper functions
void SPI_receive(void);
void left_shift_one_bit(uint8_t[], uint16_t);
uint16_t parse_null_data(uint8_t[], uint16_t, uint16_t, uint8_t);
void insert_ch_id(uint8_t[], uint16_t, uint16_t, uint8_t, uint8_t);
void memory_cleaning(uint8_t[], uint16_t, uint16_t);
void insertionSort(uint8_t[], uint16_t);
void signExtend(uint8_t[], uint16_t);

int main(void)
{

    /* Data Rate */
    uint32_t f_DATA = f_CLK / (ADC_CLK_DIV * 2 * OSR);

    /* Data Port Clock */
    uint32_t f_DCLK = f_CLK / DCLK_DIV;

    /* Number of bytes in one FSYNC period */
    uint16_t numBytes_in_one_FSYNC_period = (ADC_CLK_DIV * OSR) / (DCLK_DIV * 4);


/********check that the data port configuration is valid*******/
    /* ADC Clock Frequency */    
    switch (speed_mode) 
    {
        case 0:
            assert(f_CLK/ADC_CLK_DIV <= 3290000);
            break;
        case 1:
            assert(f_CLK/ADC_CLK_DIV <= 13150000);
            break;
        case 2:
            assert(f_CLK/ADC_CLK_DIV <= 26300000);
            break;
        default:
            assert(f_CLK/ADC_CLK_DIV <= 33660000);
            break;
    }
   
     

    // If the DCLK frequency is not faster than the RHS product, then data will be lost
    assert(f_DCLK >= f_DATA * TDM_factor * 24);

    /************* Assert MSPM0G3507's Limitations  *************/

    //DCLK is limited by the MSPM0G3507's Max SPI Clock Freq operating at 1.8 V
    assert(f_DCLK <= 24000000);

    // DP_TDM == 1 or 2; the number of DOUT pins is limited by the MSPM0G3507's number of SPI ports
    assert(DP_TDM == 1 | DP_TDM == 2);
   
   


   /***************** Initalize M0 ***********************************/
    SYSCFG_DL_init();
    DL_SYSCTL_disableSleepOnExit();
  
    DL_GPIO_setPins(GPIO_L18_PORT, GPIO_L18_RESET_PIN);
    delay_cycles(50000);
    
    NVIC_ClearPendingIRQ(SPI_PERIPHERAL_INST_INT_IRQN);
    NVIC_EnableIRQ(SPI_PERIPHERAL_INST_INT_IRQN);
     
    NVIC_ClearPendingIRQ(SPI_PERIPHERAL2_INST_INT_IRQN);
    NVIC_EnableIRQ(SPI_PERIPHERAL2_INST_INT_IRQN);



    /***************************** Initalize ADC********************/
  
    initalizeRegisters(OSR, speed_mode, DP_TDM, DCLK_DIV, ADC_CLK_SEL, ADC_CLK_DIV);
  
    // sets L18 START PIN HIGH
    DL_GPIO_setPins(GPIO_L18_PORT, GPIO_L18_START_PIN);


    /************************ Begin data collection ***************/

    while (1) {
        
        // This function utilizes the SPI Peripheral mode and DMA to collect data from the ADS127L18
        SPI_receive();
        
        // Once data has been received break out of while loop
        if (data_received_flag) {
            // sets L18 START PIN LOW
            DL_GPIO_clearPins(GPIO_L18_PORT, GPIO_L18_START_PIN);
            break;
        }
    }

    /************* Parsing Routine (Start) *************/
    #if DP_TDM == 1    
    // gRxPacket
    // shift all of the data over one bit
    left_shift_one_bit(gRxPacket, RX_BUFFER_SIZE);

    // Parse null Data (Do Math)
    uint16_t valid_data_current_buffer_size = parse_null_data(gRxPacket, RX_BUFFER_SIZE, numBytes_in_one_FSYNC_period, total_channelBytes);
    uint16_t valid_data_final_buffer_size = valid_data_current_buffer_size / 3 * 4;

    //24-bit to 32-bit conversion routine (extra byte is used to denote the channel number)
    insert_ch_id(gRxPacket, valid_data_current_buffer_size, valid_data_final_buffer_size, 0, 7);

    // Clean up memory (zero out the null data)
    memory_cleaning(gRxPacket, valid_data_final_buffer_size, RX_BUFFER_SIZE);

    // sort the channels
    insertionSort(gRxPacket, valid_data_final_buffer_size);

    // Sign-extend each data conversion sample
   signExtend(gRxPacket, valid_data_final_buffer_size);



    #endif

    #if DP_TDM == 2
    // gRxPacket
    // shift all of the data over one bit
    left_shift_one_bit(gRxPacket, RX_BUFFER_SIZE);

    // Parse null Data (Do Math)
    uint16_t valid_data_current_buffer_size = parse_null_data(gRxPacket, RX_BUFFER_SIZE, numBytes_in_one_FSYNC_period, total_channelBytes);
    uint16_t valid_data_final_buffer_size = valid_data_current_buffer_size / 3 * 4;

    // 24-bit to 32-bit conversion routine (extra byte is used to denote the channel number)
    insert_ch_id(gRxPacket, valid_data_current_buffer_size, valid_data_final_buffer_size, 0, 3);

    // Clean up memory (zero out the null data)
    memory_cleaning(gRxPacket, valid_data_final_buffer_size, RX_BUFFER_SIZE);

    // sort the channels
    insertionSort(gRxPacket, valid_data_final_buffer_size);

    // Sign-extend each data conversion sample
    signExtend(gRxPacket, valid_data_final_buffer_size);

    // gRxPacket2
    // shift all of the data over one bit
    left_shift_one_bit(gRxPacket2, RX_BUFFER_SIZE);

    // Parse null Data (Do Math)
    uint16_t valid_data_current_buffer_size2 = parse_null_data(gRxPacket2, RX_BUFFER_SIZE, numBytes_in_one_FSYNC_period, total_channelBytes);
    uint16_t valid_data_final_buffer_size2 = valid_data_current_buffer_size2 / 3 * 4;

    // 24-bit to 32-bit conversion routine (extra byte is used to denote the channel number)
    convert_to_32_bits(gRxPacket2, valid_data_current_buffer_size2, valid_data_final_buffer_size2, 4, 7);

    // Clean up memory (zero out the null data)
    memory_cleaning(gRxPacket2, valid_data_final_buffer_size2, RX_BUFFER_SIZE);

    // sort the channels
    insertionSort(gRxPacket2, valid_data_final_buffer_size2);

    // Sign-extend each data conversion sample
    signExtend(gRxPacket2, valid_data_final_buffer_size2);
    #endif

    /************* Parsing Routine (End) *************/

    while (1) {}    // wait forever. 
           
}

void SPI_PERIPHERAL_INST_IRQHandler(void)
{
    switch (DL_SPI_getPendingInterrupt(SPI_PERIPHERAL_INST)) {
        case DL_SPI_IIDX_DMA_DONE_RX:
            /* DMA is done transferring data from RXFIFO to gRxPacket*/
            gDMARXDataTransferred1 = true;
            break;

        default:
            break;
    }
}

void SPI_PERIPHERAL2_INST_IRQHandler(void)
{
    switch (DL_SPI_getPendingInterrupt(SPI_PERIPHERAL2_INST)) {
        case DL_SPI_IIDX_DMA_DONE_RX:
            /* DMA is done transferring data from RXFIFO to gRxPacket*/
            gDMARXDataTransferred2 = true;
            break;

        default:
            break;
    }
}


void SPI_receive(void)
{
    //FIXME: reference the example code that I used for these functions

    /*
     * Configure DMA source, destination and size from RXDATA to gRxPacket.
     * The DMA transfer will start when the RX interrupt is set, which happens
     * when the device receives data.
     */
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&SPI_PERIPHERAL_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &gRxPacket[gRxPacket_index]);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, RX_BUFFER_SIZE); // RX_BUFFER_SIZE 
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
    gRxPacket_index = gRxPacket_index + RX_BUFFER_SIZE; // RX_BUFFER_SIZE  

#if DP_TDM == 2
    DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)(&SPI_PERIPHERAL2_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t) &gRxPacket2[gRxPacket2_index]);
    DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, RX_BUFFER_SIZE); // RX_BUFFER_SIZE 
    DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);
    gRxPacket2_index = gRxPacket2_index + RX_BUFFER_SIZE; // RX_BUFFER_SIZE
#endif    

    /*
     * Wait in SLEEP mode until data is received and DMA interrupt
     * is triggered
     */
    while (false == gDMARXDataTransferred1) {
        __WFE();
    }

    gDMARXDataTransferred1 = false; 

#if DP_TDM == 2
    /*
     * Wait in SLEEP mode until data is received and DMA interrupt
     * is triggered
     */
    while (false == gDMARXDataTransferred2) {
        __WFE();
    }

    gDMARXDataTransferred2 = false;
#endif 

    // When transfer finishes, bring start pin low and set data_flag to true
    if (gRxPacket_index == RX_BUFFER_SIZE) {
        // sets L18 START PIN LOW
        DL_GPIO_clearPins(GPIO_L18_PORT, GPIO_L18_START_PIN);
        data_received_flag = true;
    }
}




//*****************************************************************************
//
//! This function left shifts all of the data in arr by 1 bit
//!
//! \fn void left_shift_one_bit(uint8_t arr[], uint16_t n)
//!
//! \param arr is the array representing the M0's memory.
//! \param n is the number of bytes allocated to arr.
//!
//! \return None.
//
//*****************************************************************************
void left_shift_one_bit(uint8_t arr[], uint16_t n)
{
    for(uint16_t i = 0; i < n-1; i++)
    {
        // left shift one bit (after shifting, the least significant bit will be a 0b)
        arr[i] = arr[i] << 1;

        // if the first bit of the next byte is a 1, then set the least significant bit as 1b to the current byte
        if(arr[i + 1] & 0x80)
        {
            arr[i] = arr[i] | 0x01;
        }

    }
}

//*****************************************************************************
//
//! This function parses out the null data in the M0's memory array.
//!
//! \fn uint16_t parse_null_data(uint8_t arr[], uint16_t buffer_size, 
//!                                 uint16_t numBytes_in_one_FSYNC_period, 
//!                                 uint8_t total_channelBytes)
//!
//! \param arr is the array representing the M0's memory.
//! \param buffer_size is the number of bytes allocated to arr.
//! \param numBytes_in_one_FSYNC_period is the number of bytes in one FSYNC period.
//! \param total_channelBytes is the number of bytes that make up the valid channel
//!                           data in one FSYNC period.
//!
//! NOTE: this function has not accounted for channel power-down mode
//! EX: Say CH0 - CH3 data are on one DOUT pin, but CH1 is powered down
//! Therefore you will still see the 0s representing CH1 data in the M0 memory
//!
//! \return Returns the number of valid channel bytes.
//
//*****************************************************************************
uint16_t parse_null_data(uint8_t arr[], uint16_t buffer_size, uint16_t numBytes_in_one_FSYNC_period, uint8_t total_channelBytes)
{
    // replace_index starts at the (index of the) first instance of null data in the M0's memory
    // (think of it as a pointer) After replacing each value replace_index will increment by 1
    uint16_t replace_index = total_channelBytes;

    // channelData_index "points" to the valid channel data of the M0's memory,
    // and will skip the null data with the help of number_of_valid_bytes_replaced
    // we start at numBytes_in_one_FSYNC_period because the first set of valid channel data
    // is already sorted at the beginning of the M0's memory
    uint16_t channelData_index = numBytes_in_one_FSYNC_period;
    uint16_t number_of_valid_bytes_replaced = 0;

    // Calculates the number of null bytes in one FSYNC period
    uint16_t number_null_bytes = numBytes_in_one_FSYNC_period - total_channelBytes;

    // If there are no null bytes, return the maximum size of 24-bit data packets
    // that will fit the allocated M0's memory once converted to 32-bits  
    if(number_null_bytes == 0) {return (buffer_size / 4 * 3);}

    while(channelData_index < buffer_size)
    {
        // replace value at 'replace_index' with value at 'channelData_index'
        arr[replace_index] = arr[channelData_index];

        // increment "pointer" positions
        replace_index++;
        channelData_index++;
        number_of_valid_bytes_replaced++;

        // if the number of valid channel byte replacements equals the total number 'total_channelBytes'
        // update 'channelData_index' to "point" to the next valid channel data in the array
        if(number_of_valid_bytes_replaced == total_channelBytes)
        {
            number_of_valid_bytes_replaced = 0;
            channelData_index = channelData_index + number_null_bytes;
        }
    }

    // return the number of valid channel bytes
    uint16_t valid_data_current_buffer_size = buffer_size / numBytes_in_one_FSYNC_period * total_channelBytes;
    //return replace_index;
    return valid_data_current_buffer_size;
}

//*****************************************************************************
//
//! This function converts all of the 24-bit data packets in arr into 32-bit.
//! The extra byte added to each data packet will represent the corresponding
//! channel data (This will be useful in the sorting function)
//!
//! \fn void convert_to_32_bits(uint8_t arr[], uint16_t before_conversion_size, 
//!                             uint16_t after_conversion_size, 
//!                             uint8_t first_channel_number, 
//!                             uint8_t last_channel_number) 
//!
//! \param arr is the array representing the M0's memory.
//! \param before_conversion_size is is the number of bytes allocated to arr.
//! \param after_conversion_size is the number of bytes in one FSYNC period.
//! \param first_channel_number is the number of bytes that make up the valid channel
//! \param last_channel_number is the number of bytes that make up the valid channel
//!
//! \return None.
//
//*****************************************************************************
void insert_ch_id(uint8_t arr[], uint16_t before_conversion_size, uint16_t after_conversion_size, uint8_t first_channel_number, uint8_t last_channel_number)
{
    long val = 0;
    uint8_t save_buffer[3];
    uint16_t before_conversion_index = before_conversion_size - 3;
    uint8_t channel_num = last_channel_number;
    
    for(uint16_t after_conversion_index = after_conversion_size - 4; after_conversion_index >= 0; after_conversion_index = after_conversion_index - 4){

        // save initial values
        save_buffer[0] = arr[before_conversion_index];
        save_buffer[1] = arr[before_conversion_index + 1];
        save_buffer[2] = arr[before_conversion_index + 2];

        // 24-bit to 32-bit conversion
        // Extra byte is used to denote the channel number (this will be helpful for the sorting algorithm)
        arr[after_conversion_index] = channel_num;

        val = (val << 8) | save_buffer[0];
        arr[after_conversion_index + 1] = val;

        val = (val << 8) | save_buffer[1];
        arr[after_conversion_index + 2] = val;

        val = (val << 8) | save_buffer[2];
        arr[after_conversion_index + 3] = val;

        // After converting the last 24-bit data packet to 32-bit, break out of for loop
        // (We need a force break because unsigned integer can never be negative)
        if(after_conversion_index == 0) {break;} 

        // update val
        val = 0;
        
        // update channel_num
        if(channel_num == first_channel_number){
            channel_num = last_channel_number;
        } else {
            channel_num--;
        }

        // update before_conversion_index
        before_conversion_index = before_conversion_index - 3;
    }
}

// This function zeros out all of the leftover space after the 24-bit to 32-bit conversion
void memory_cleaning(uint8_t arr[], uint16_t after_conversion_size, uint16_t full_buffer_size)
{
    // replace leftover the null data with zeros
    for(uint16_t i = after_conversion_size; i < full_buffer_size; i++)
    {
        arr[i] = 0;
    }

}

// This function sorts arr using insertion sort
// Note: 'Stable' and 'In-Place' Sorting Algorithm
void insertionSort(uint8_t arr[], uint16_t n)
{
    for (uint16_t i = 4; i < n; i = i + 4) {
        uint8_t key0 = arr[i];
        uint8_t key1 = arr[i + 1];
        uint8_t key2 = arr[i + 2];
        uint8_t key3 = arr[i + 3];
        uint16_t j = i - 4;

        /* Move elements of arr[0..i-1], that are
        greater than key, to one position ahead
        of their current position */
        while (j >= 0 && arr[j] > key0) {
            arr[j + 4] = arr[j];
            arr[j + 5] = arr[j + 1];
            arr[j + 6] = arr[j + 2];
            arr[j + 7] = arr[j + 3];
            j = j - 4;
        }
        arr[j + 4] = key0;
        arr[j + 5] = key1;
        arr[j + 6] = key2;
        arr[j + 7] = key3;
    }
}

// This function sign extends each 32-bit data packet in arr
void signExtend(uint8_t arr[], uint16_t n)
{

// now re-arrange endiness of and data to respect channel id
    int8_t temp[4];
    for (uint16_t j = 0; j < n; j = j + 4)
    {
        temp[0] = arr[j + 3];
        temp[1] = arr[j + 2];
        temp[2] = arr[j + 1];
        temp[3] = arr[j];

        arr[j] = temp[0];
        arr[j + 1] = temp[1];
        arr[j + 2] = temp[2];
        arr[j + 3] = temp[3];

    }
}

