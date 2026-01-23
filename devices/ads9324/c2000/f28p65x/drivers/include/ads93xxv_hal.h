/*
 * @brief: Hardware abstraction
 *
 * @copyright Copyright (C) 2025-2026 Texas Instruments Incorporated - http://www.ti.com/
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
 */

#ifndef ADS93XXV_HAL_H_
#define ADS93XXV_HAL_H_

#ifdef __cplusplus

extern "C" {
#endif


//
// Includes
//

#include "driverlib.h"
#include "device.h"
#include <ads93xxv_settings.h>
#include <ads93xxv_regbank0.h>
#include <ads93xxv_regbank1.h>
#include <ads93xxv_regbank2.h>


//
// Global variables
//
typedef struct
{
    volatile int16_t channel0;
    volatile int16_t channel1;
    volatile int16_t channel2;
    volatile int16_t channel3;
    volatile int16_t channel4;
    volatile int16_t channel5;
    volatile int16_t channel6;
    volatile int16_t channel7;
    volatile int16_t channel8;
    volatile int16_t channel9;
    volatile int16_t channel10;
    volatile int16_t channel11;
    volatile int16_t channel12;
    volatile int16_t channel13;
    volatile int16_t channel14;
    volatile int16_t channel15;
} ads93xxv_data;

extern ads93xxv_data ADS93XXV_HAL_adcCapture1;

#if(ADS93XXV_BUILD_NUM==ADS93XXV_16B_DMA ||ADS93XXV_BUILD_NUM==ADS93XXV_24B_DMA)
extern const void *destAddr;
extern const void *srcAddr;
#endif


//
// Function prototypes
//

//
// Device initialization prototypes
//
void ADS93XXV_HAL_setupDevice(void);
void ADS93XXV_HAL_DeviceInit();
void ADS93XXV_HAL_initPeripherals();
void ADS93XXV_HAL_MEMCFG_init();
void ADS93XXV_HAL_initSysctl();

void ADS93XXV_HAL_initAnalogPinMux(void);
void ADS93XXV_HAL_initAsysctl(void);
void ADS93XXV_HAL_enableGlobalInterrupt(void);


//
// ISR function prototypes
//
interrupt void ISR1(void);
interrupt void ISR2(void);
interrupt void ISR3(void);

//
// ADC functions
//
void ADS93XXV_HAL_setupGpioExtAdcSpi();
void ADS93XXV_HAL_setupGpioExtAdc();
void ADS93XXV_HAL_setupGpioExtAdcDrdy();
void ADS93XXV_HAL_setupGpioIntDrdy();
void ADS93XXV_HAL_setupGpioExtAdcRst();
void ADS93XXV_HAL_setupGpioExtAdcConvst();

void ADS93XXV_HAL_setupSpiExtAdcRegReadWrite(void);
void ADS93XXV_HAL_setupSpiExtAdcDataRead(void);
void ADS93XXV_HAL_resetExtAdc(void);
void ADS93XXV_initExtAdc();

void ADS93XXV_HAL_setupExtAdcInterrupt(void);
void ADS93XXV_HAL_enableExtAdcInterrupt(void);

void ADS93XXV_HAL_enableEpwmCounting();
void ADS93XXV_HAL_setupEpwmExtAdcConvst();
void ADS93XXV_HAL_disableEpwmCounting();

void ADS93XXV_HAL_setupSpiAdcRegReadWrite();
void ADS93XXV_HAL_setupSpiExtAdcDataRead();

#if(ADS93XXV_BUILD_NUM==ADS93XXV_16B_DMA || ADS93XXV_BUILD_NUM==ADS93XXV_24B_DMA)

void ADS93XXV_HAL_initDmaSpiTx();
void ADS93XXV_HAL_initDmaSpiRx();
void ADS93XXV_HAL_initDmaSpi();
void ADS93XXV_HAL_setupInterruptDmaRx(void);

#endif

void ADS93XXV_HAL_initUserVariables(void );

void ADS93XXV_HAL_delay_ms(uint16_t milliseconds);
void ADS93XXV_HAL_delay_us(uint16_t milliseconds);




//
// GPIO Configurations
//

void ADS93XXV_HAL_initGpio0();
void ADS93XXV_HAL_initGpio1();



//
//==============================================================================
// ADS93XXV_HAL_ADS93XXV_HAL_transmitAdcDataReadFrame -
// - Interrupt function DRDY risings edge
//==============================================================================
//
#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_ADS93XXV_HAL_transmitAdcDataReadFrame)
static inline void ADS93XXV_HAL_ADS93XXV_HAL_transmitAdcDataReadFrame(void){
    
    SPI_writeDataNonBlocking(ADS93XXV_SPI_BASE, 0x0000);
    SPI_writeDataNonBlocking(ADS93XXV_SPI_BASE, 0x0000);
    SPI_writeDataNonBlocking(ADS93XXV_SPI_BASE, 0x0000);
    SPI_writeDataNonBlocking(ADS93XXV_SPI_BASE, 0x0000);
    SPI_writeDataNonBlocking(ADS93XXV_SPI_BASE, 0x0000);
    SPI_writeDataNonBlocking(ADS93XXV_SPI_BASE, 0x0000);
    SPI_writeDataNonBlocking(ADS93XXV_SPI_BASE, 0x0000);
    SPI_writeDataNonBlocking(ADS93XXV_SPI_BASE, 0x0000);
    SPI_writeDataNonBlocking(ADS93XXV_SPI_BASE, 0x0000);
    SPI_writeDataNonBlocking(ADS93XXV_SPI_BASE, 0x0000);
    SPI_writeDataNonBlocking(ADS93XXV_SPI_BASE, 0x0000);
    SPI_writeDataNonBlocking(ADS93XXV_SPI_BASE, 0x0000);
    SPI_writeDataNonBlocking(ADS93XXV_SPI_BASE, 0x0000);
    SPI_writeDataNonBlocking(ADS93XXV_SPI_BASE, 0x0000);
    SPI_writeDataNonBlocking(ADS93XXV_SPI_BASE, 0x0000);
    SPI_writeDataNonBlocking(ADS93XXV_SPI_BASE, 0x0000);

}

//
//==============================================================================
// ADS93XXV_HAL_readSpiFifoAdcData - Interrupt function ISR2 for 1 Lane Mode
//==============================================================================
//
#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_readSpiFifoAdcData)
static inline void ADS93XXV_HAL_readSpiFifoAdcData(void){
    

    ADS93XXV_HAL_adcCapture1.channel0 =
            SPI_readDataNonBlocking(ADS93XXV_SPI_BASE);
    ADS93XXV_HAL_adcCapture1.channel1 =
            SPI_readDataNonBlocking(ADS93XXV_SPI_BASE);
    ADS93XXV_HAL_adcCapture1.channel2 =
            SPI_readDataNonBlocking(ADS93XXV_SPI_BASE);
    ADS93XXV_HAL_adcCapture1.channel3 =
            SPI_readDataNonBlocking(ADS93XXV_SPI_BASE);
    ADS93XXV_HAL_adcCapture1.channel4 =
            SPI_readDataNonBlocking(ADS93XXV_SPI_BASE);
    ADS93XXV_HAL_adcCapture1.channel5 =
            SPI_readDataNonBlocking(ADS93XXV_SPI_BASE);
    ADS93XXV_HAL_adcCapture1.channel6 =
            SPI_readDataNonBlocking(ADS93XXV_SPI_BASE);
    ADS93XXV_HAL_adcCapture1.channel7 =
            SPI_readDataNonBlocking(ADS93XXV_SPI_BASE);


    ADS93XXV_HAL_adcCapture1.channel8 =
            SPI_readDataNonBlocking(ADS93XXV_SPI_BASE);
    ADS93XXV_HAL_adcCapture1.channel9 =
            SPI_readDataNonBlocking(ADS93XXV_SPI_BASE);
    ADS93XXV_HAL_adcCapture1.channel10 =
            SPI_readDataNonBlocking(ADS93XXV_SPI_BASE);
    ADS93XXV_HAL_adcCapture1.channel11 =
            SPI_readDataNonBlocking(ADS93XXV_SPI_BASE);
    ADS93XXV_HAL_adcCapture1.channel12 =
            SPI_readDataNonBlocking(ADS93XXV_SPI_BASE);
    ADS93XXV_HAL_adcCapture1.channel13 =
            SPI_readDataNonBlocking(ADS93XXV_SPI_BASE);
    ADS93XXV_HAL_adcCapture1.channel14 =
            SPI_readDataNonBlocking(ADS93XXV_SPI_BASE);
    ADS93XXV_HAL_adcCapture1.channel15 =
            SPI_readDataNonBlocking(ADS93XXV_SPI_BASE);

}

//
//==============================================================================
// ADS93XXV_HAL_clearAckGroup1 - Clear interrupts ACK for DRDY
//==============================================================================
//
#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_clearAckGroup1)
static inline void ADS93XXV_HAL_clearAckGroup1(void){

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//
//==============================================================================
// ADS93XXV_HAL_clearAckGroupSpiRxFull - Clear interrupts ACK for ISR2
//==============================================================================
//
#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_clearAckGroupSpiRxFull)
static inline void ADS93XXV_HAL_clearAckGroupSpiRxFull(void){

    SPI_clearInterruptStatus(ADS93XXV_SPI_BASE, SPI_INT_RXFF);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP6);

}


//
//==============================================================================
// ADS93XXV_HAL_clearAckDmaTxEnd - Clear interrupts ACK for DMA TX
//==============================================================================
//
#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_clearAckDmaTxEnd)
static inline void ADS93XXV_HAL_clearAckDmaTxEnd(void){

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);

}

//
//==============================================================================
// ADS93XXV_HAL_clearAckDmaRxEnd - Clear interrupts ACK for DMA RX
//==============================================================================
//
#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_clearAckDmaRxEnd)
static inline void ADS93XXV_HAL_clearAckDmaRxEnd(void){

    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);
}

#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_startDmaTx)
static inline void ADS93XXV_HAL_startDmaTx(uint32_t base){
    DMA_startChannel(base);

}

#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_startDmaRx)
static inline void ADS93XXV_HAL_startDmaRx(uint32_t base){
    DMA_startChannel(base);

}

#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_stopDmaTx)
static inline void ADS93XXV_HAL_stopDmaTx(uint32_t base){
    DMA_stopChannel(base);

}

#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_stopDmaRx)
static inline void ADS93XXV_HAL_stopDmaRx(uint32_t base){
    DMA_stopChannel(base);

}


#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_sendTxFrameExtAdc)
static inline void ADS93XXV_HAL_sendTxFrameExtAdc(uint32_t base){

    //
    // send 16 null words for 16bit 16 channels data
    //
    SPI_writeDataNonBlocking(base, 0x0000);
    SPI_writeDataNonBlocking(base, 0x0000);
    SPI_writeDataNonBlocking(base, 0x0000);
    SPI_writeDataNonBlocking(base, 0x0000);
    SPI_writeDataNonBlocking(base, 0x0000);
    SPI_writeDataNonBlocking(base, 0x0000);
    SPI_writeDataNonBlocking(base, 0x0000);
    SPI_writeDataNonBlocking(base, 0x0000);

    SPI_writeDataNonBlocking(base, 0x0000);
    SPI_writeDataNonBlocking(base, 0x0000);
    SPI_writeDataNonBlocking(base, 0x0000);
    SPI_writeDataNonBlocking(base, 0x0000);
    SPI_writeDataNonBlocking(base, 0x0000);
    SPI_writeDataNonBlocking(base, 0x0000);
    SPI_writeDataNonBlocking(base, 0x0000);
    SPI_writeDataNonBlocking(base, 0x0000);

}


static inline void ADS93XXV_HAL_ExtAdcRxframe(ads93xxv_data *DataStruct,
                                           uint32_t base){
    //
    // Read from the FIFO buffer
    //
    //ADS93XXV_HAL_writeRegister(0x02, 0x0002);
    DataStruct->channel0=SPI_readDataNonBlocking(base);
    DataStruct->channel1=SPI_readDataNonBlocking(base);
    DataStruct->channel2=SPI_readDataNonBlocking(base);
    DataStruct->channel3=SPI_readDataNonBlocking(base);
    DataStruct->channel4=SPI_readDataNonBlocking(base);
    DataStruct->channel5=SPI_readDataNonBlocking(base);
    DataStruct->channel6=SPI_readDataNonBlocking(base);
    DataStruct->channel7=SPI_readDataNonBlocking(base);
    DataStruct->channel8=SPI_readDataNonBlocking(base);
    DataStruct->channel9=SPI_readDataNonBlocking(base);
    DataStruct->channel10=SPI_readDataNonBlocking(base);
    DataStruct->channel11=SPI_readDataNonBlocking(base);
    DataStruct->channel12=SPI_readDataNonBlocking(base);
    DataStruct->channel13=SPI_readDataNonBlocking(base);
    DataStruct->channel14=SPI_readDataNonBlocking(base);
    DataStruct->channel15=SPI_readDataNonBlocking(base);

}


#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_ackHwIntDrdy)
static inline void ADS93XXV_HAL_ackHwIntDrdy(void){
    Interrupt_clearACKGroup(ADS93XXV_INT_DRDY_INTERRUPT_ACK_GROUP);
}


//
//==============================================================================
// ADS93XXV_HAL_writeRegister - Function to write ADC registers using SPI
//==============================================================================
//
#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_writeRegister)
static inline void ADS93XXV_HAL_writeRegister(uint8_t reg_addr, uint16_t reg_value)
{
    //
    // configure 8-bit + 16-bit data into 24-bit SPI frame
    //
    uint32_t reg_addr_value = ((uint32_t)reg_addr << 16) | (reg_value & 0xFFFF);

    //
    // Transmit 24-bit frame
    //
    SPI_transmit24Bits(ADS93XXV_SPI_BASE, reg_addr_value,0);

}


//
//==============================================================================
// ADS93XXV_HAL_readRegister - Function to read ADC registers using SPI
//==============================================================================
//
#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_readRegister)
static inline uint32_t ADS93XXV_HAL_readRegister(uint8_t reg_addr)
{
    //
    // configure 8-bit + 16-bit data into 24-bit SPI frame
    //
    uint32_t spiFrame = ((uint32_t)0x010000) | ((uint32_t)reg_addr <<8) | 0x000001;

    //
    // Transmit 24-bit register read command
    //
    SPI_transmit24Bits(ADS93XXV_SPI_BASE, spiFrame,0);

    //
    // Optional delay
    //
    ADS93XXV_HAL_delay_us(1);

    //
    // Transmit 24 SCLK to read the register output
    //
    spiFrame=(SPI_receive24Bits(ADS93XXV_SPI_BASE,SPI_DATA_BIG_ENDIAN, 0x00,0))>>8U;

    //
    // Optional delay
    //
    ADS93XXV_HAL_delay_us(1);

    return(spiFrame);


}

//
//==============================================================================
// ADS93XXV_HAL_readRegister - Function to read ADC registers using SPI
//==============================================================================
//
#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_readModiFyWriteRegister)
static inline uint32_t ADS93XXV_HAL_readModiFyWriteRegister(uint8_t reg_addr, uint16_t reg_value, uint16_t mask)
{
    //
    // configure 8-bit + 16-bit data into 24-bit SPI frame
    //
    uint32_t spiFrame = ((uint32_t)0x010000) | ((uint32_t)reg_addr <<8) | 0x000001;

    //
    // Transmit 24-bit register read command
    //
    SPI_transmit24Bits(ADS93XXV_SPI_BASE, spiFrame,0);

    //
    // Optional delay
    //
    ADS93XXV_HAL_delay_us(10);

    //
    // Transmit 24 SCLK to read the register output
    //
    spiFrame=(SPI_receive24Bits(ADS93XXV_SPI_BASE,SPI_DATA_BIG_ENDIAN, 0x00,0))>>8U;

    //
    // Configure 8-bit + 16-bit data into 24-bit SPI frame
    //
    uint32_t registerWrite;
    registerWrite = spiFrame & ~mask; // reset the bit field
    registerWrite = registerWrite | (reg_value & mask); // update the new value
    registerWrite = ((uint32_t)reg_addr << 16) | (registerWrite); // append the address

    //
    // Transmit 24-bit frame
    //
    SPI_transmit24Bits(ADS93XXV_SPI_BASE, registerWrite,0);

    //
    // Optional delay
    //
    ADS93XXV_HAL_delay_us(10);

    return(spiFrame); // return previous value of the register

}



#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_readExtAdc)
static inline void ADS93XXV_HAL_readExtAdc(){

    //
    // Make CS high
    //
    //GPIO_writePin(ADS93XXV_CS_GPIO_NUM, 1);
    ADS93XXV_HAL_ExtAdcRxframe(&ADS93XXV_HAL_adcCapture1,ADS93XXV_SPI_BASE);

}

#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_ackInterruptSpiRxFifo)
static inline void ADS93XXV_HAL_ackInterruptSpiRxFifo(){
    SPI_clearInterruptStatus(ADS93XXV_SPI_BASE, SPI_INT_RXFF );
    Interrupt_clearACKGroup(ADS93XXV_SPI_RXFIFO_INTERRUPT_ACK_GROUP);
}


//
//=============================================================================
// Static inline functions
//=============================================================================
//
#pragma FUNC_ALWAYS_INLINE(ADS93XXV_HAL_getLowSpeedClock)
static inline uint32_t ADS93XXV_HAL_getLowSpeedClock(void)
{
    #ifdef USE_INTOSC
        return( SysCtl_getLowSpeedClock((uint32_t)BT4CH_INTOSC_HZ) );
    #else
        return( SysCtl_getLowSpeedClock(DEVICE_OSCSRC_FREQ) );
    #endif
}

#ifdef __cplusplus
}
#endif  

#endif /* ADS93XXV_HAL_H_ */
