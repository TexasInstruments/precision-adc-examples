/*
 * Copyright (c) 2024-2025, Texas Instruments Incorporated
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

#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti/driverlib/dl_dma.h"
#include "ti/driverlib/dl_gpio.h"
#include "ti/driverlib/dl_spi.h"
#include "ti/driverlib/m0p/dl_core.h"
#include "ti_msp_dl_config.h"
#include <assert.h>

/* 48-bit 3-wire SPI frame for accessing dual channel ADC data on D3 lane with manual CS control */
/* Use 16-bit SPI and repeat 3 times for 48-bit transmission */
#define ADC_DATA_SPI_FRAME_SIZE (DL_SPI_DATA_SIZE_16)
#define ADC_DATA_SPI_POLARITY (DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA0)
#define ADC_DATA_DMA_WIDTH DL_DMA_WIDTH_HALF_WORD

/* 24-bit 3-wire SPI frame for register access with manual CS control */
/* Use 8-bit SPI and repeat 3 times for 24-bit transmission */
#define ADC_REGISTERS_SPI_FRAME_SIZE (DL_SPI_DATA_SIZE_8)
#define ADC_REGISTERS_SPI_POLARITY (DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA0)
#define ADC_REGISTERS_DMA_WIDTH DL_DMA_WIDTH_BYTE

/* Data for SPI to receive for register access */
uint16_t gRxData[4];
uint16_t gADCByte;

/* Variables to record ADC channel data */
uint16_t adc_a;
uint16_t adc_b;

volatile bool gSPIDataTransmitted, gDMATXDataTransferred,
    gDMARXDataTransferred, gTransmitReady;

// functions to configure MSPM0 SPI module
void config_SPI_24bit_frames(void);
void config_SPI_48bit_frames(void);

// functions to acess the ADC register read. register write and data read
void adc_reg_write(uint8_t  addr, uint16_t data, uint8_t clks);
uint8_t adc_reg_read(uint8_t  addr, bool bank, uint8_t clks);
void read_adc_1sdo(void);
void initialize_ads9327(void);

uint8_t calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint8_t initialValue);

uint16_t adc_48bit_spi_transfer(uint8_t addr, uint16_t data);
uint16_t adc_24bit_spi_transfer(uint8_t addr, uint16_t data);

int main(void)
{
    gTransmitReady        = false;
    gSPIDataTransmitted   = false;
    gDMATXDataTransferred = false;
    gDMARXDataTransferred = false;

    SYSCFG_DL_init();
    DL_SYSCTL_disableSleepOnExit();

    NVIC_EnableIRQ(SPI_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* CONVST = 1; idle */
    DL_GPIO_setPins(GPIOB, ADS9327_CONVST_PIN);

    initialize_ads9327(); //sets up the ADC in 1-SDO mode; use 48-bit transfers
    gADCByte = adc_reg_read(0x10, true, 48);

    while (1) {
        if (gTransmitReady == true) {
            gTransmitReady = false;

            /* read 1 conversion result from 2 ADC channels */
            read_adc_1sdo();            
            
        } else {
            __WFI();
        }
    }
}

void SPI_0_INST_IRQHandler(void)
{
    switch (DL_SPI_getPendingInterrupt(SPI_0_INST)) {
        case DL_SPI_IIDX_DMA_DONE_TX:
            /* DMA is done transferring data from gTxPacket to TXFIFO */
            gDMATXDataTransferred = true;
            break;
        case DL_SPI_IIDX_TX_EMPTY:
            /* SPI is done transmitting data and TXFIFO is empty */
            gSPIDataTransmitted = true;
            break;
        case DL_SPI_IIDX_DMA_DONE_RX:
            /* DMA is done transferring data from RXFIFO to gRxPacket*/
            gDMARXDataTransferred = true;
        default:
            break;
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMER_IIDX_ZERO:
            gTransmitReady = true;
            DL_GPIO_togglePins(GPIO_LEDS_PORT,
                GPIO_LEDS_USER_LED_1_PIN);
            break;
        default:
            break;
    }
}

void adc_reg_write(uint8_t addr, uint16_t data, uint8_t clks)
{    
    if (clks == 24)
    {
        config_SPI_24bit_frames();
        adc_24bit_spi_transfer(addr, data);
    }
    else
    {
        config_SPI_48bit_frames();
        adc_48bit_spi_transfer(addr, data);
    }    
}

uint8_t adc_reg_read(uint8_t addr, bool bank, uint8_t clks)
{
    uint16_t reg_data;

    if (clks == 24) { config_SPI_24bit_frames(); }
    else { config_SPI_48bit_frames(); }

    /* 
    * This funtion unlocks the register map, reads a register, and locks the register map.
    * Multiple registers can be read after register map is unlocked.
    */

    // unlock the register map
    adc_reg_write(0xFE, 0xB38F, clks);
    adc_reg_write(0xFE, 0xABCD, clks);

    // select register bank
    if (bank == 0) { adc_reg_write(0x02, 0x0000, clks); } // register bank 0
    else           { adc_reg_write(0x02, 0x0002, clks); } // register bank 1

    // write the addr of the register to be read
    adc_reg_write(0x01, ((addr << 8) + 0x0001), clks); 

    // drain the RX FIFO
    DL_SPI_drainRXFIFO16(SPI_0_INST, gRxData, 4);

    // read register data in this frame and also clear addr 0x01
    if (clks == 24) { reg_data = adc_24bit_spi_transfer(0x01, 0x0000); }
    else { reg_data = adc_48bit_spi_transfer(0x01, 0x0000); }

    // lock the register map with any value 
    adc_reg_write(0xFE, 0x1234, clks);

    return reg_data;
}

/* transmit and receive 24-bit frames */
uint16_t adc_24bit_spi_transfer(uint8_t addr, uint16_t data)
{
    /* 
    * This function requires that SPI controller is configured in 3 wire mode 
    * and CS line is configured for manual control. See void config_SPI_3wire(void).
    */
    uint16_t spi_readback;

    /* CSn = 0 */
    DL_GPIO_clearPins(GPIOB, GPIO_SPI_0_CS0_PIN);

    /* transmit #1: 8-bit address */
    DL_SPI_transmitDataBlocking8(SPI_0_INST, addr);

    /* wait for all SPI bytes to be transmitted */
    while (DL_SPI_isBusy(SPI_0_INST));
    spi_readback = (DL_SPI_receiveData8(SPI_0_INST) << 8);  // ADC register reads return in the MSB 8 bits after CSn falling
    
    /* transmit #2: data[15:8] */
    DL_SPI_transmitDataBlocking8(SPI_0_INST, (data & 0xFF00) >> 8 );

    /* wait for all SPI bytes to be transmitted */
    while (DL_SPI_isBusy(SPI_0_INST));
    spi_readback += DL_SPI_receiveData8(SPI_0_INST);  // ADC register reads return in the LSB 8 bits

    /* transmit #3: data[7:0] */
    DL_SPI_transmitDataBlocking8(SPI_0_INST, (data & 0x00FF));

    /* wait for all SPI bytes to be transmitted */
    while (DL_SPI_isBusy(SPI_0_INST));

    /* CSn = 1 */
    DL_GPIO_setPins(GPIOB, GPIO_SPI_0_CS0_PIN);
    return spi_readback;
}

uint16_t adc_48bit_spi_transfer(uint8_t addr, uint16_t data)
{
    uint16_t reg_data;
    /* 
    * This function requires that SPI controller is configured transmit 16-bit frames.
    * See adc_48bit_spi_transfer().
    */
    
    /* CSn = 0 */
    DL_GPIO_clearPins(GPIOB, GPIO_SPI_0_CS0_PIN);

    /* transmit #1 */
    DL_SPI_transmitDataBlocking16(SPI_0_INST, ((addr << 8) + ((data & 0xFF00) >> 8)));  

    /* wait for all SPI bytes to be transmitted */
    while (DL_SPI_isBusy(SPI_0_INST));
    reg_data = DL_SPI_receiveDataBlocking16(SPI_0_INST);  

    DL_SPI_transmitDataBlocking16(SPI_0_INST, ((data & 0x00FF) << 8));  
    
    /* wait for all SPI bytes to be transmitted */
    while (DL_SPI_isBusy(SPI_0_INST));
    gRxData[1] = DL_SPI_receiveData16(SPI_0_INST);  

    /* transmit #2 */
    DL_SPI_transmitDataBlocking16(SPI_0_INST, 0x0000);

    /* wait for all SPI bytes to be transmitted */
    while (DL_SPI_isBusy(SPI_0_INST));
    gRxData[2] = DL_SPI_receiveData16(SPI_0_INST);  

   /* wait for all SPI bytes to be transmitted */
    while (DL_SPI_isBusy(SPI_0_INST));

    /* CSn = 1 */
    DL_GPIO_setPins(GPIOB, GPIO_SPI_0_CS0_PIN);

    return reg_data;
}


void read_adc_1sdo(void)
{
    /* 
    * This function requires that SPI controller is configured in 3 wire mode 
    * and CS line is configured for manual control. See config_SPI_48bit_frames().
    */

    /* CONVST = 0; sampling instant */
    DL_GPIO_clearPins(GPIOB, ADS9327_CONVST_PIN);

    delay_cycles(5); //remove from rev1.1 onwards

    /* CSn = 0 */
    DL_GPIO_clearPins(GPIOB, GPIO_SPI_0_CS0_PIN);

    /* transmit 48 SCLKs */
    DL_SPI_transmitDataBlocking16(SPI_0_INST, 0x0000);   

    /* wait for all SPI bytes to be transmitted */
    while (DL_SPI_isBusy(SPI_0_INST));
    gRxData[0] = DL_SPI_receiveData16(SPI_0_INST);  

    DL_SPI_transmitDataBlocking16(SPI_0_INST, 0x0000);     

    /* wait for all SPI bytes to be transmitted */
    while (DL_SPI_isBusy(SPI_0_INST));
    gRxData[1] = DL_SPI_receiveData16(SPI_0_INST);  

     DL_SPI_transmitDataBlocking16(SPI_0_INST, 0x0000);     

    /* wait for all SPI bytes to be transmitted */
    while (DL_SPI_isBusy(SPI_0_INST));
    gRxData[2] = DL_SPI_receiveData16(SPI_0_INST);  

   /* wait for all SPI bytes to be transmitted */
    while (DL_SPI_isBusy(SPI_0_INST));

    /* CSn = 1 */
    DL_GPIO_setPins(GPIOB, GPIO_SPI_0_CS0_PIN);
    
    /* extract ADC data from the 48-bit data received */
    adc_a = gRxData[0];
    adc_b = ((gRxData[1] & 0x00FF) << 8) + ((gRxData[2] & 0xFF00) >> 8);

    /* CONVST = 1 */
    DL_GPIO_setPins(GPIOB, ADS9327_CONVST_PIN);
}

//*****************************************************************************
//
//! Calculates the 8-bit CRC for the selected CRC polynomial.
//!
//! \fn uint8_t calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint8_t initialValue)
//!
//! \param dataBytes[] pointer to first element in the data byte array
//! \param numberBytes number of bytes to be used in CRC calculation
//! \param initialValue the seed value (or partial crc calculation), use 0xFF when beginning a new CRC computation
//!
//! NOTE: This calculation is shown as an example and is not optimized for speed.
//!
//! \return 8-bit calculated CRC word
//
//*****************************************************************************
uint8_t calculateCRC(const uint8_t dataBytes[], uint8_t numberBytes, uint8_t initialValue)
{
    int bitIndex, byteIndex;
    bool dataMSb, crcMSb;

    //
    // Check that "dataBytes" is not a null pointer
    //
    assert(dataBytes != 0x00);

    //
    // Initial value of crc register
    // NOTE: The ADS9327 defaults to 0xFF,
    // but can be set at function call to continue an on-going calculation
    //
    uint8_t crc = initialValue;

    //
    // ANSI CRC polynomial = x^8 + x^2 + x^1 + 1
    //
    const uint8_t poly = 0x07;

    //
    // Loop through all bytes in the dataBytes[] array
    //
    for (byteIndex = 0; byteIndex < numberBytes; byteIndex++)
    {
        //
        // Loop through all bits in the current byte
        //
        bitIndex = 0x80u;
        while (bitIndex > 0)
        {
            // Get the MSB's of data and crc
            dataMSb = (bool) (dataBytes[byteIndex] & bitIndex);
            crcMSb  = (bool) (crc & 0x80u);

            // Left-shift crc register
            crc <<= 1;

            // XOR crc register with polynomial?
            if (dataMSb ^ crcMSb) { crc ^= poly; }

            // Update MSB pointer to the next data bit
            bitIndex >>= 1;
        }
    }

    return crc;
}

/*
* This function initializes the ADS9327.
* See datasheet for more details on initialization.
* Configure ADC in 1-SDO mode
*/
void initialize_ads9327(void)
{
    // adc reset
    adc_reg_write(0x01, 0x0002, 48); //because the ADC could be in 48bit mode
    adc_reg_write(0x01, 0x0002, 24);
    delay_cycles(1000000);
    
    // clear reset
    adc_reg_write(0x01, 0x0000, 24);
    
    // unlock register map
    adc_reg_write(0xFE, 0xB38F, 24);
    adc_reg_write(0xFE, 0xABCD, 24);
    
    // select register bank 1
    adc_reg_write(0x02, 0x0002, 24);

    //enable test pattern ramp + 1024 inc
    adc_reg_write(0x0F, 0x0005, 24);

    // select 1-SDO mode; data on D3; register read/writes after this need to be 48 SCLK
    // low-latency mode
    // adc_reg_write(0x09, 0x0460, 24);

    // select 1-SDO mode; data on D3; register read/writes after this need to be 48 SCLK
    // n+1 mode
    adc_reg_write(0x09, 0x0060, 24);

    config_SPI_48bit_frames();

    // write to tes pattern register to test readback function in main()
    adc_reg_write(0x10, 0xFAFA, 48);

    // lock the register map
    adc_reg_write(0xFE, 0x1234, 48);
}

/* Configure SPI controller for register access using 3 wire mode for 24-bit frame */
void config_SPI_24bit_frames(void)
{
    DL_SPI_disable(SPI_0_INST);
    DL_SPI_setDataSize(SPI_0_INST, ADC_REGISTERS_SPI_FRAME_SIZE);
    DL_SPI_setFrameFormat(SPI_0_INST, ADC_REGISTERS_SPI_POLARITY);
    DL_SPI_setChipSelect(SPI_0_INST, DL_SPI_CHIP_SELECT_NONE);

    DL_GPIO_initPeripheralOutputFunctionFeatures(
		 GPIO_SPI_0_IOMUX_CS0, GPIO_SPI_0_IOMUX_CS0_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initPeripheralOutputFunctionFeatures(
		 GPIO_SPI_0_IOMUX_PICO, GPIO_SPI_0_IOMUX_PICO_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    /* set SPI0_CS1 pin as a digital output for direct control as a GPIO */
    DL_GPIO_initDigitalOutput(GPIO_SPI_0_IOMUX_CS0);
    DL_GPIO_setPins(GPIOB, GPIO_SPI_0_CS0_PIN);
    DL_GPIO_enableOutput(GPIOB, GPIO_SPI_0_CS0_PIN);

    DL_SPI_enable(SPI_0_INST);
}

/* Configure SPI controller for reading ADC data using 3 wire mode for 16-bit frame */
void config_SPI_48bit_frames(void)
{
    DL_SPI_disable(SPI_0_INST);
    DL_SPI_setDataSize(SPI_0_INST, ADC_DATA_SPI_FRAME_SIZE);
    DL_SPI_setFrameFormat(SPI_0_INST, ADC_DATA_SPI_POLARITY);
    DL_SPI_setChipSelect(SPI_0_INST, DL_SPI_CHIP_SELECT_NONE);

    DL_GPIO_initPeripheralOutputFunctionFeatures(
		 GPIO_SPI_0_IOMUX_CS0, GPIO_SPI_0_IOMUX_CS0_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initPeripheralOutputFunctionFeatures(
		 GPIO_SPI_0_IOMUX_PICO, GPIO_SPI_0_IOMUX_PICO_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    /* set SPI0_CS1 pin as a digital output for direct control as a GPIO */
    DL_GPIO_initDigitalOutput(GPIO_SPI_0_IOMUX_CS0);
    DL_GPIO_setPins(GPIOB, GPIO_SPI_0_CS0_PIN);
    DL_GPIO_enableOutput(GPIOB, GPIO_SPI_0_CS0_PIN);

    DL_SPI_enable(SPI_0_INST);
}