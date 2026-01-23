/*
 * @brief: Board level settings
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

//
// Include files
//
#include <ads93xxv.h>
#include <ads93xxv_settings.h>

uint16_t regReadValue=0;

//
// Main
//
void main(void)
{

    //
    // Initialize device clock and peripherals
    // Disable pin locks and enable internal pull-ups.
    // Initialize PIE and clear PIE registers. Disables CPU interrupts
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //

    ADS93XXV_HAL_setupDevice();

    EALLOW;

    //
    //  Initialize analog GPIO
    //
    ADS93XXV_HAL_initAnalogPinMux();

    //
    // Configure voltage reference for the internal ADC
    // External voltage reference is selected
    // Voltage reference is 2.5V
    //
    ADS93XXV_HAL_initAsysctl();

    //
    // Configure I/O for external ADC
    // Initialize SPI for register read and write
    //
    ADS93XXV_HAL_setupGpioExtAdc();

    //
    // Hard reset the ADC
    //
    ADS93XXV_HAL_resetExtAdc();

    ADS93XXV_HAL_setupSpiAdcRegReadWrite();

    ADS93XXV_swResetAdc();

    ADS93XXV_initalization();

    //
    // Set OSR to 8
    //
    ADS93XXV_setOsr();
    ADS93XXV_HAL_writeRegister(0x02, 0x01);
    regReadValue=ADS93XXV_HAL_readRegister(0x14);

    //
    // Enable register read
    //
    ADS93XXV_setupRegReadOutOnSdout();

    ADS93XXV_HAL_writeRegister(0x02, 0x02);
    ADS93XXV_HAL_writeRegister(0x2F,0x1234);
    regReadValue=ADS93XXV_HAL_readRegister(0x2F);
    ADS93XXV_HAL_writeRegister(0x02,0x01);

    //
    // Throw an error if SPI is not working
    //
    if(regReadValue!=0x1234){
        ESTOP0;
    }

    ADS93XXV_HAL_writeRegister(0x02, 0x01);
    regReadValue=ADS93XXV_HAL_readRegister(0x14);

    //
#if(ADS93XXV_BUILD_NUM==ADS93XXV_16B_DMA || ADS93XXV_BUILD_NUM==ADS93XXV_16B_SPI_FIFO)
    ADS93XXV_setupAdcOutOnSdoutSize_16b();
#elif(ADS93XXV_BUILD_NUM==ADS93XXV_24B_DMA)
    ADS93XXV_setupAdcOutOnSdoutSize_24b();
#endif

    //
    // Optional: Check the interface by enable test pattern output on the ADC
    //
#if(ADS93XXV_TEST_PATT_EN)
    ADS93XXV_setupTestPattern();
#endif

    //
    // Setup SPI for data read
    //
    ADS93XXV_HAL_setupSpiExtAdcDataRead();

    //
    // Start conversion start clock for the ADC
    //
    ADS93XXV_HAL_enableEpwmCounting();
    ADS93XXV_HAL_setupEpwmExtAdcConvst();
    ADS93XXV_HAL_disableEpwmCounting();


    //
    // Init DMA
    //
#if(ADS93XXV_BUILD_NUM==ADS93XXV_16B_DMA || ADS93XXV_BUILD_NUM==ADS93XXV_24B_DMA)
    ADS93XXV_HAL_initDmaSpi();
#endif

    //
    // Init user variables
    //
    ADS93XXV_HAL_initUserVariables();


    //
    // Configure and enable External ADC data ready and SPI interrupt
    //
    ADS93XXV_HAL_setupExtAdcInterrupt();

    ADS93XXV_HAL_enableExtAdcInterrupt();

    //
    // enable global interrupts
    //
    ADS93XXV_HAL_enableGlobalInterrupt();
    EDIS;


    //
    // Background loop with periodic branches to state-machine tasks.
    // Frequency of task branching is configured in setupDevice() routine.
    //
    while(1)
    {
         // jump to an Alpha state (A0,B0,...)
    }
}

//
// ISR2() interrupt function
//
#pragma CODE_SECTION(ISR1,"isrfunc");
__interrupt void ISR1(void)
{
    //
    // ISR is triggered by BUSY/DRDY
    //
#if(ADS93XXV_BUILD_NUM==ADS93XXV_16B_SPI_FIFO)
    ADS93XXV_HAL_sendTxFrameExtAdc(ADS93XXV_SPI_BASE);
#elif(ADS93XXV_BUILD_NUM==ADS93XXV_16B_DMA || ADS93XXV_BUILD_NUM==ADS93XXV_24B_DMA)
    ADS93XXV_HAL_startDmaTx(ADS93XXV_HAL_SPI_TX_DMA_BASE);
    ADS93XXV_HAL_startDmaRx(ADS93XXV_HAL_SPI_RX_DMA_BASE);
#endif

    //
    // Acknowledge the interrupt
    //
    ADS93XXV_HAL_ackHwIntDrdy();

}

//
// ISR2() interrupt function
//
#pragma CODE_SECTION(ISR2,"isrfunc");
__interrupt void ISR2(void)
{
    //
    // ISR is triggered by the SPIx_RX
    //

    ADS93XXV_HAL_readExtAdc();

    ADS93XXV_HAL_ackInterruptSpiRxFifo();

}

//
// ISR3() interrupt function
//
#pragma CODE_SECTION(ISR3,"isrfunc");
__interrupt void ISR3(void)
{
    //
    // ISR is triggered by the DMA Rx
    //

#if(ADS93XXV_BUILD_NUM==ADS93XXV_16B_DMA || ADS93XXV_BUILD_NUM==ADS93XXV_24B_DMA)
    //ADS93XXV_HAL_stopDmaRx(ADS93XXV_HAL_SPI_RX_DMA_BASE);
#endif


    ADS93XXV_HAL_clearAckDmaRxEnd();
}


//
// End of ADS93XXV_main.c
//
