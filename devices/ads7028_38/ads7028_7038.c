/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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

/* 12-bit 4-wire SPI frame for reading ADC data */
#define ADC_DATA_SPI_FRAME_SIZE (DL_SPI_DATA_SIZE_12)
#define ADC_DATA_SPI_POLARITY (DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0)
#define ADC_DATA_DMA_WIDTH DL_DMA_WIDTH_HALF_WORD

/* 24-bit 3-wire SPI frame for register access with manual CS control */
#define ADC_REGISTERS_SPI_FRAME_SIZE (DL_SPI_DATA_SIZE_8)
#define ADC_REGISTERS_SPI_POLARITY ( DL_SPI_FRAME_FORMAT_MOTO3_POL0_PHA0)
#define ADC_REGISTERS_DMA_WIDTH DL_DMA_WIDTH_BYTE

/* ADS7028 / 38 register access OPCODES from datasheet*/
#define OPCODE_WRITE    0x08 
#define OPCODE_READ     0x10 
#define OPCODE_NOP      0x00 


#define SPI_PACKET_SIZE_REG_ACCESS (3) /* Byte 1: OPCODE | Byte 2: address | Byte 3: Data */
#define SPI_PACKET_SIZE_DATA (8)

/* Data for SPI to transmit for register access */
uint8_t gTxPacket[SPI_PACKET_SIZE_REG_ACCESS];

/* Data for SPI to receive for register access */
uint32_t gRxByte [8];
uint16_t gCount;

/* Data for receiving ADC result */
uint16_t gRxData[SPI_PACKET_SIZE_DATA];
uint16_t gTxData[SPI_PACKET_SIZE_DATA];

volatile bool gSPIDataTransmitted, gDMATXDataTransferred,
    gDMARXDataTransferred, gTransmitReady;

// functions to configure MSPM0 SPI module
void config_SPI_3wire(void);
void config_SPI_4wire(void);

// functions to acess the ADC register read. register write and data read
void adc_reg_write(uint8_t  addr, uint8_t data);
uint8_t adc_reg_read(uint8_t  addr);
uint8_t adc_24bit_spi_transfer(uint8_t opcode, uint8_t addr, uint8_t data);
void adc_12bit_spi_transfer(void);

//demo function
void clear_BOR_bit(void);

int main(void)
{
    gTransmitReady        = false;
    gSPIDataTransmitted   = false;
    gDMATXDataTransferred = false;
    gDMARXDataTransferred = false;


    SYSCFG_DL_init();
    DL_SYSCTL_disableSleepOnExit();

    // DL_GPIO_initDigitalInput(IOMUX_PINCM29);
    NVIC_EnableIRQ(SPI_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    // reset the ADC
    adc_reg_write(0x01, 0x01);
    delay_cycles(1000);

    //call the demo function to demonstrate register read - write
    clear_BOR_bit();
    
    //select AIN3 for manual channel select in address 0x11
    adc_reg_write(0x11, 0x03);
    
    // configure ADS7028 / ADS7038 channels
    // AIN2 = digital output, push-pull (this pin voltage is measured by AIN3; install jumper between J5-5 and J5-6 on ADS7038EVM-PDK)
    // AIN3 = analog input
    // AIN7 = digital output, push-pull, logic 0
    adc_reg_write(0x05, 0x84); //PIN_CFG (addr = 0x05); set AIN2 as GPIO
    adc_reg_write(0x07, 0x84); //GPIO_CFG (addr = 0x07); set AIN2 as digital output
    adc_reg_write(0x09, 0x84); //GPO_CFG (addr = 0x09); set AIN2 as push-pull output

    adc_reg_write(0x0B, 0x04); //GPO_OUTPUT_VALUE (addr = 0x0B); set AIN2 as logic 1
    
    while (1) {
        if (gTransmitReady == true) {
            gTransmitReady = false;
            
            //read AIN3
            adc_12bit_spi_transfer();
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

/* Configure SPI controller for register access using 3 wire mode for 24-bit frame */
void config_SPI_3wire(void)
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

/* Configure SPI controller for reading ADC data using 4 wire mode for 16-bit frame */
void config_SPI_4wire(void)
{
    DL_SPI_disable(SPI_0_INST);
    DL_SPI_setDataSize(SPI_0_INST, ADC_DATA_SPI_FRAME_SIZE);
    DL_SPI_setFrameFormat(SPI_0_INST, ADC_DATA_SPI_POLARITY);
    
    /* set SPI0_CS1 pin as a peripheral CS line */
    DL_GPIO_initPeripheralOutputFunctionFeatures(
		 GPIO_SPI_0_IOMUX_CS0, GPIO_SPI_0_IOMUX_CS0_FUNC,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_SPI_enable(SPI_0_INST);
}

void adc_reg_write(uint8_t addr, uint8_t data)
{    
    config_SPI_3wire();
    adc_24bit_spi_transfer(OPCODE_WRITE, addr, data);
}

uint8_t adc_reg_read(uint8_t addr)
{
    config_SPI_3wire();
    gCount = 0;
    adc_24bit_spi_transfer(OPCODE_READ, addr, 0x00);
    delay_cycles(10);
    return adc_24bit_spi_transfer(OPCODE_NOP, 0x00, 0x00);
}

/* transmit and receive 24-bit frames */
uint8_t adc_24bit_spi_transfer(uint8_t opcode, uint8_t addr, uint8_t data)
{
    /* 
    * This function requires that SPI controller is configured in 3 wire mode 
    * and CS line is configured for manual control. See void config_SPI_3wire(void).
    */
    
    /* CSn = 0 */
    DL_GPIO_clearPins(GPIOB, GPIO_SPI_0_CS0_PIN);

    /* transmit 24-bit data */
    DL_SPI_transmitDataBlocking8(SPI_0_INST, opcode);   // transmit opcode
    delay_cycles(100);
    /* wait for all SPI bytes to be transmitted */
    while (DL_SPI_isBusy(SPI_0_INST));
    gRxByte[gCount] = DL_SPI_receiveData8(SPI_0_INST);  // ADC register reads return in the first 8 bits after CSn falling

    DL_SPI_transmitDataBlocking8(SPI_0_INST, addr);     //transmit address
    delay_cycles(100);
    /* wait for all SPI bytes to be transmitted */
    while (DL_SPI_isBusy(SPI_0_INST));
    gRxByte[gCount] = DL_SPI_receiveData8(SPI_0_INST);  // ADC register reads return in the first 8 bits after CSn falling

    DL_SPI_transmitDataBlocking8(SPI_0_INST, data);     //transmit data
    delay_cycles(100);
    /* wait for all SPI bytes to be transmitted */
    while (DL_SPI_isBusy(SPI_0_INST));
    gRxByte[gCount] = DL_SPI_receiveData8(SPI_0_INST);  // ADC register reads return in the first 8 bits after CSn falling

    /* wait for all SPI bytes to be transmitted */
    while (DL_SPI_isBusy(SPI_0_INST));

    /* CSn = 1 */
    DL_GPIO_setPins(GPIOB, GPIO_SPI_0_CS0_PIN);
    return gRxByte[4];

}

void adc_12bit_spi_transfer(void)
{
    config_SPI_4wire();

    /* transmit 16-bit data */
    DL_SPI_transmitDataBlocking16(SPI_0_INST, 0x00);   // transmit dummy byte
    /* wait for all SPI bytes to be transmitted */
    while (DL_SPI_isBusy(SPI_0_INST));
    gRxData[0] = DL_SPI_receiveDataBlocking16(SPI_0_INST);
}
/*
* This function reads the BOR bit in address 0x00. The BOR bit is set to 1b after a reset or power-up.
* The host can clear this bit by writing 1b to it. 
* The host can monitor this bit to detect if the ADC was reset by an external event.
* This function is intended as a demo for register read and write operations.
*/
void clear_BOR_bit(void)
{
    // read address 0x00; on power-up this should return 0x81
    adc_reg_read(0x00);

    //clear the BOR bit in address 0x00
    adc_reg_write(0x00, 0x01); 

    //read BOR bit in address 0x00; this should return 0x80
    adc_reg_read(0x00);

}