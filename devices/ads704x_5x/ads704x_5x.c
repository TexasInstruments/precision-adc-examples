/*
 * Copyright (c) 2025, Texas Instruments Incorporated
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

#include "ti/driverlib/dl_dma.h"
#include "ti/driverlib/dl_spi.h"
#include "ti_msp_dl_config.h"


/* selection list for support ADC part numbers */
#define ADS7040 (0)     /* 8bit 1MSPS single-ended inputs*/
#define ADS7041 (1)     /* 10bit 1MSPS single-ended inputs*/
#define ADS7042 (2)     /* 12bit 1MSPS single-ended inputs*/
#define ADS7043 (3)     /* 12bit 1MSPS pseudo-differential inputs*/
#define ADS7044 (4)     /* 12bit 1MSPS fully-differential inputs*/
#define ADS7046 (5)     /* 12bit 3MSPS single-ended inputs*/
#define ADS7047 (6)     /* 12bit 1MSPS fully-differential inputs*/
#define ADS7052 (7)     /* 14bit 1MSPS single-ended inputs*/
#define ADS7054 (8)     /* 14bit 1MSPS fully-differential inputs*/
#define ADS7056 (9)     /* 14bit 2.5MSPS single-ended inputs*/
#define ADS7057 (10)    /* 14bit 2.5MSPS fully-differential inputs*/

/* select the target ADC */
#define ADC_PART_NUMBER ADS7056

    
#if ADC_PART_NUMBER == ADS7040
    #define ADC_SPI_FRAME_SIZE (DL_SPI_DATA_SIZE_10)
    #define ADC_SPI_POLARITY (DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0)
    #define DMA_WIDTH DL_DMA_WIDTH_HALF_WORD
    #define SPI_PACKET_SIZE (4)
#elif (ADC_PART_NUMBER == ADS7041) 
    #define ADC_SPI_FRAME_SIZE (DL_SPI_DATA_SIZE_12)
    #define ADC_SPI_POLARITY (DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0)
    #define DMA_WIDTH DL_DMA_WIDTH_HALF_WORD
    #define SPI_PACKET_SIZE (4)
#elif (ADC_PART_NUMBER == ADS7042) || (ADC_PART_NUMBER == ADS7043) || (ADC_PART_NUMBER == ADS7044)
    #define ADC_SPI_FRAME_SIZE (DL_SPI_DATA_SIZE_14)
    #define ADC_SPI_POLARITY (DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0)
    #define DMA_WIDTH DL_DMA_WIDTH_HALF_WORD
    #define SPI_PACKET_SIZE (4)
#elif (ADC_PART_NUMBER == ADS7046) || (ADC_PART_NUMBER == ADS7047)
    #define ADC_SPI_FRAME_SIZE (DL_SPI_DATA_SIZE_15)
    #define ADC_SPI_POLARITY (DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA1)
    #define DMA_WIDTH DL_DMA_WIDTH_HALF_WORD
    #define SPI_PACKET_SIZE (1)
#elif (ADC_PART_NUMBER == ADS7052) || (ADC_PART_NUMBER == ADS7054) || (ADC_PART_NUMBER == ADS7056) || (ADC_PART_NUMBER == ADS7057)
    #define ADC_SPI_FRAME_SIZE (DL_SPI_DATA_SIZE_9) // SPI packing will be used to get 18bit output
    #define ADC_SPI_POLARITY (DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA1)
    #define DMA_WIDTH DL_DMA_WIDTH_WORD
    #define SPI_PACKET_SIZE (2)
#endif

/* Data for SPI to transmit */
uint16_t gTxPacket[SPI_PACKET_SIZE];

/* Data for SPI to receive */
uint16_t gRxPacket[SPI_PACKET_SIZE];

volatile bool gSPIDataTransmitted, gDMATXDataTransferred,
    gDMARXDataTransferred, gTransmitReady;

void SPI_send(void);

int main(void)
{
    gTransmitReady        = false;
    gSPIDataTransmitted   = false;
    gDMATXDataTransferred = false;
    gDMARXDataTransferred = false;


    SYSCFG_DL_init();
    DL_SYSCTL_disableSleepOnExit();

    /* reconfigure SPI controller as per ADC_PART_NUMBER */

    DL_SPI_disable(SPI_0_INST);
    DL_SPI_setDataSize(SPI_0_INST, ADC_SPI_FRAME_SIZE);
    DL_SPI_setFrameFormat(SPI_0_INST, ADC_SPI_POLARITY);
    DL_SPI_enable(SPI_0_INST);
    
    NVIC_EnableIRQ(SPI_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    while (1) {
        if (gTransmitReady == true) {
            gTransmitReady = false;
            SPI_send();
        } else {
            __WFI();
        }
    }
}

void SPI_send(void)
{
    /*
     * Configure DMA source, destination and size to transfer data from
     * gTxPacket to TXDATA. The DMA transfer will start when TX interrupt is
     * set, which it should already be set (which indicates that the SPI is
     * ready to transfer) so the DMA will begin this transfer when the channel
     * is enabled.
     */
    DL_DMA_setSrcWidth(DMA, DMA_CH0_CHAN_ID, DMA_WIDTH);
    DL_DMA_setDestWidth(DMA, DMA_CH0_CHAN_ID, DMA_WIDTH);
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &gTxPacket[0]);
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&SPI_0_INST->TXDATA));
    DL_DMA_setTransferSize(
        DMA, DMA_CH0_CHAN_ID, sizeof(gTxPacket) / sizeof(gTxPacket[0]));

    /*
     * Configure DMA source, destination and size from RXDATA to gRxPacket.
     * The DMA transfer will start when the RX interrupt is set, which happens
     * when the device receives data.
     */
    DL_DMA_setSrcWidth(DMA, DMA_CH1_CHAN_ID, DMA_WIDTH);
    DL_DMA_setDestWidth(DMA, DMA_CH1_CHAN_ID, DMA_WIDTH);
    DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)(&SPI_0_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t) &gRxPacket[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, sizeof(gTxPacket) / sizeof(gTxPacket[0]));

    /*
     * The SPI TX interrupt is already set, indicating the SPI is ready to
     * transmit data, so enabling the DMA will start the transfer
     */
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
    DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);
    /*
     * Wait in SLEEP mode until SPI_PACKET_SIZE bytes have been transferred
     * from gTxPacket to the SPI TXFIFO, and the DMA interrupt is triggered
     */
    while (false == gDMATXDataTransferred) {
        __WFE();
    }

    /*
     * Wait until the SPI has transmitted all the data and the TXFIFO
     * is empty
     */
    while (false == gSPIDataTransmitted) {
        __WFE();
    }

    /*
     * Wait in SLEEP mode until SPI_PACKET_SIZE bytes have been transferred
     * from SPI TXFIFO to gRxPacket, and the DMA interrupt is triggered
     */
    while (false == gDMARXDataTransferred) {
        __WFE();
    }
    /*
     * Optional SW breakpoint to check results.
     * If this example is used with the
     * spi_peripheral_repeated_fifo_dma_interrupts example,
     * the expected data that will be received in gRxPacket is
     * {'x', 0x2, 0x3, 0x4}, where 'x' starts at 0x0 and
     * should increment every time the Peripheral example
     * sends a new data packet.
     */
    //        __BKPT(0);

    gSPIDataTransmitted   = false;
    gDMATXDataTransferred = false;
    gDMARXDataTransferred = false;
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
                GPIO_LEDS_USER_LED_1_PIN | GPIO_LEDS_USER_TEST_PIN);
            break;
        default:
            break;
    }
}
