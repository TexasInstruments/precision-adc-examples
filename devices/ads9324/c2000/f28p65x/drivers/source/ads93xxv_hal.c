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

#include <ads93xxv_hal.h>

//
// Global variables
//
ads93xxv_data ADS93XXV_HAL_adcCapture1;


//
// Global variables to store ADC output
//
#if(ADS93XXV_BUILD_NUM==ADS93XXV_16B_DMA)

uint16_t ADC_16b_sData[16];                // Send data buffer
uint16_t ADC_16b_rData[16];                // Receive data buffer

const void *destAddr = (const void *)ADC_16b_rData;
const void *srcAddr = (const void *)ADC_16b_sData;

//
// Put the buffers in DMA RX and TX RAM
//
#pragma DATA_SECTION(ADC_16b_sData, "DMA_TX_SECTION");
#pragma DATA_SECTION(ADC_16b_rData, "DMA_RX_SECTION");

#elif(ADS93XXV_BUILD_NUM==ADS93XXV_24B_DMA)

uint16_t ADC_24b_sData[48];                // Send data buffer
uint16_t ADC_24b_rData[48];                // Receive data buffer

const void *destAddr = (const void *)ADC_24b_rData;
const void *srcAddr = (const void *)ADC_24b_sData;

//
// Put the buffers in DMA RX and TX RAM
//
#pragma DATA_SECTION(ADC_24b_sData, "DMA_TX_SECTION");
#pragma DATA_SECTION(ADC_24b_rData, "DMA_RX_SECTION");

#endif



//
//==============================================================================
// ADS93XXV_HAL_DeviceInit - Setup device
//==============================================================================
//
void ADS93XXV_HAL_setupDevice(void)
{
    //
    // Initialize device clock and peripherals
    // Disable pin locks and enable internal pull-ups.
    // Initialize PIE and clear PIE registers. Disables CPU interrupts
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //

    Device_init();
    Device_initGPIO();
    Interrupt_initModule();
    Interrupt_initVectorTable();
}



//
// SPI GPIO Configurations used by the external ADC
//
void ADS93XXV_HAL_setupGpioExtAdcSpi()
{
    //
    // GPIO for SPI
    //
    GPIO_setPinConfig(ADS93XXV_SDIN_PIN_CONFIG);
    GPIO_setPadConfig(ADS93XXV_SDIN_GPIO_NUM, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(ADS93XXV_SDIN_GPIO_NUM, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(ADS93XXV_SDOUT_PIN_CONFIG);
    GPIO_setPadConfig(ADS93XXV_SDOUT_GPIO_NUM, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(ADS93XXV_SDOUT_GPIO_NUM, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(ADS93XXV_SCLK_PIN_CONFIG);
    GPIO_setPadConfig(ADS93XXV_SCLK_GPIO_NUM, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(ADS93XXV_SCLK_GPIO_NUM, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(ADS93XXV_CS_PIN_CONFIG_GPIO);
    GPIO_setPadConfig(ADS93XXV_CS_GPIO_NUM, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(ADS93XXV_CS_GPIO_NUM, GPIO_QUAL_ASYNC);
}


//
// Setup DRDY pin as hardware interrupt
//
void ADS93XXV_HAL_setupGpioExtAdcDrdy()
{
    GPIO_setPinConfig(ADS93XXV_DRDY_PIN_CONFIG_GPIO);
    GPIO_setPadConfig(ADS93XXV_DRDY_GPIO_NUM,
                      GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(ADS93XXV_DRDY_GPIO_NUM, GPIO_QUAL_SYNC);
    GPIO_setDirectionMode(ADS93XXV_DRDY_GPIO_NUM, GPIO_DIR_MODE_IN);
    GPIO_setControllerCore(ADS93XXV_DRDY_GPIO_NUM, GPIO_CORE_CPU1);

    //
    // Configure XINT1 to be a triggered by a falling edge
    //
    GPIO_setInterruptType(GPIO_INT_XINT1, GPIO_INT_TYPE_RISING_EDGE);
    GPIO_setInterruptPin(ADS93XXV_DRDY_GPIO_NUM, GPIO_INT_XINT1);
    GPIO_enableInterrupt(GPIO_INT_XINT1);
}

//
// Setup GPIO for ADC RES
//
void ADS93XXV_HAL_setupGpioExtAdcRst()
{
    GPIO_setPinConfig(ADS93XXV_RST_PIN_CONFIG_GPIO);
    GPIO_setPadConfig(ADS93XXV_RST_GPIO_NUM, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(ADS93XXV_RST_GPIO_NUM, GPIO_QUAL_SYNC);
    GPIO_setDirectionMode(ADS93XXV_RST_GPIO_NUM, GPIO_DIR_MODE_OUT);
    GPIO_setControllerCore(ADS93XXV_RST_GPIO_NUM, GPIO_CORE_CPU1);
}


//
// Setup GPIOs for the external ADC
//
void ADS93XXV_HAL_setupGpioExtAdc()
{
    ADS93XXV_HAL_setupGpioExtAdcDrdy();
    ADS93XXV_HAL_setupGpioExtAdcConvst();
    ADS93XXV_HAL_setupGpioExtAdcRst();
    ADS93XXV_HAL_setupGpioExtAdcSpi();

}

//
//==============================================================================
// ADS93XXV_HAL_setupEpwmExtAdcConvst - Configure EPWM as CONVST signal for
// the external ADC
//==============================================================================
//
void ADS93XXV_HAL_setupEpwmExtAdcConvst(void)
{
    //
    // Maximum supported ePWM clock speed is specified in the datasheet
    //
    EPWM_setClockPrescaler(ADS93XXV_CONVST_EPWM_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    EPWM_setEmulationMode(ADS93XXV_CONVST_EPWM_BASE,
                          EPWM_EMULATION_FREE_RUN);

    //
    // Configure ePWM for count-up operation
    //
    EPWM_setTimeBaseCounter(ADS93XXV_CONVST_EPWM_BASE, 0);
    EPWM_setTimeBasePeriod(ADS93XXV_CONVST_EPWM_BASE,
                           ADS93XXV_CONVST_EPWM_PERIOD);
    EPWM_setPeriodLoadMode(ADS93XXV_CONVST_EPWM_BASE,
                           EPWM_PERIOD_SHADOW_LOAD);
    EPWM_setTimeBaseCounterMode(ADS93XXV_CONVST_EPWM_BASE,
                                EPWM_COUNTER_MODE_UP);
    EPWM_disablePhaseShiftLoad(ADS93XXV_CONVST_EPWM_BASE);

    //
    // Set Compare values for duty cycle
    //
    EPWM_setCounterCompareValue(ADS93XXV_CONVST_EPWM_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                ADS93XXV_CONVST_ONTIME_TICKS);
    EPWM_setCounterCompareValue(ADS93XXV_CONVST_EPWM_BASE,
                            EPWM_COUNTER_COMPARE_B,
                            ADS93XXV_CONVST_EPWM_PERIOD/2);


    //
    // Use shadow mode to update CMPA on TBPRD
    //
    EPWM_setCounterCompareShadowLoadMode(ADS93XXV_CONVST_EPWM_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_PERIOD);
    EPWM_setCounterCompareShadowLoadMode(ADS93XXV_CONVST_EPWM_BASE,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_PERIOD);


    //
    // Configure Action Qualifier SubModule to:
    //

    //
    // Use shadow mode to update AQCTL
    //
    EPWM_setActionQualifierShadowLoadMode(ADS93XXV_CONVST_EPWM_BASE,
                                          EPWM_ACTION_QUALIFIER_A,
                                          EPWM_AQ_LOAD_ON_CNTR_PERIOD);
    EPWM_setActionQualifierAction(ADS93XXV_CONVST_EPWM_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    EPWM_setActionQualifierAction(ADS93XXV_CONVST_EPWM_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);


    //
    //  Enable SOC signal for the internal ADC
    //
    EPWM_enableADCTrigger(ADS93XXV_CONVST_EPWM_BASE, EPWM_SOC_A);
    EPWM_setADCTriggerSource(ADS93XXV_CONVST_EPWM_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_PERIOD);
    EPWM_setADCTriggerEventPrescale(ADS93XXV_CONVST_EPWM_BASE, EPWM_SOC_A,
                                     ADS93XXV_EPWM_ADC_TRIGGER_PRESCALE);
    EPWM_enableADCTriggerEventCountInit(ADS93XXV_CONVST_EPWM_BASE, EPWM_SOC_A);
    EPWM_forceADCTriggerEventCountInit(ADS93XXV_CONVST_EPWM_BASE, EPWM_SOC_A);
}

//
//=============================================================================
// Setup GPIO for ADC conversion clock
//=============================================================================
//
void ADS93XXV_HAL_setupGpioExtAdcConvst(void)
{
    //GPIO_setPadConfig(ADS93XXV_CONVST_EPWM_GPIO_NUM, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(ADS93XXV_CONVST_EPWM_GPIO_NUM, 1);
    GPIO_setDirectionMode(ADS93XXV_CONVST_EPWM_GPIO_NUM, GPIO_DIR_MODE_OUT);
    GPIO_setPinConfig(ADS93XXV_CONVST_PIN_CONFIG_EPWM);
    GPIO_setQualificationMode(ADS93XXV_CONVST_EPWM_GPIO_NUM, GPIO_QUAL_SYNC);
}

//
//==============================================================================
// Enables EPWM peripheral clock
//==============================================================================
//
void ADS93XXV_HAL_enableEpwmCounting(void)
{
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}
//
//==============================================================================
// Disables EPWM peripheral clock
//==============================================================================
//
void ADS93XXV_HAL_disableEpwmCounting(void)
{
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}


//
//==============================================================================
// ADS93XXV_HAL_setupSpiExtAdcDataRead -
// Configure SPI interface for ADC data capture
//==============================================================================
//
void ADS93XXV_HAL_setupSpiExtAdcDataRead(void)
{
    //
    // Must put SPI into reset before configuring it
    //
    SPI_disableModule(ADS93XXV_SPI_BASE);

    //
    // SPI configuration. Use a 1MHz SPICLK and 16-bit word size.
    //
    SPI_enableHighSpeedMode(ADS93XXV_SPI_BASE);



#if(ADS93XXV_BUILD_NUM==ADS93XXV_16B_SPI_FIFO)

    SPI_setConfig(ADS93XXV_SPI_BASE,
                DEVICE_LSPCLK_FREQ,
                SPI_PROT_POL0PHA1,SPI_MODE_CONTROLLER,
                ADS93XXV_SPI_CLOCK_FREQ_HZ, 16U);


    SPI_setEmulationMode(ADS93XXV_SPI_BASE, SPI_EMULATION_STOP_MIDWAY);

    SPI_enableFIFO(ADS93XXV_SPI_BASE);

    SPI_clearInterruptStatus(ADS93XXV_SPI_BASE,
                             SPI_INT_RX_DATA_TX_EMPTY
                             |SPI_INT_RXFF|SPI_INT_TXFF );

    SPI_setFIFOInterruptLevel(ADS93XXV_SPI_BASE,
                              SPI_FIFO_TX0, SPI_FIFO_RX16);
    SPI_enableInterrupt(ADS93XXV_SPI_BASE, SPI_INT_RXFF|SPI_INT_TXFF);

#elif(ADS93XXV_BUILD_NUM==ADS93XXV_16B_DMA)
    SPI_setConfig(ADS93XXV_SPI_BASE,
                DEVICE_LSPCLK_FREQ,
                SPI_PROT_POL0PHA1,SPI_MODE_CONTROLLER,
                ADS93XXV_SPI_CLOCK_FREQ_HZ, 16U);


    SPI_setEmulationMode(ADS93XXV_SPI_BASE, SPI_EMULATION_STOP_MIDWAY);

    SPI_enableFIFO(ADS93XXV_SPI_BASE);

    SPI_clearInterruptStatus(ADS93XXV_SPI_BASE,
                             SPI_INT_RX_DATA_TX_EMPTY
                             |SPI_INT_RXFF|SPI_INT_TXFF );

    SPI_setFIFOInterruptLevel(ADS93XXV_SPI_BASE,
                              SPI_FIFO_TX0, SPI_FIFO_RX16);
    SPI_enableInterrupt(ADS93XXV_SPI_BASE, SPI_INT_RXFF|SPI_INT_TXFF);
    SPI_setFIFOInterruptLevel(ADS93XXV_SPI_BASE,
                              SPI_FIFO_TX8, SPI_FIFO_RX8);
    SPI_disableInterrupt(ADS93XXV_SPI_BASE, SPI_INT_RXFF|SPI_INT_TXFF);
#elif((ADS93XXV_BUILD_NUM==ADS93XXV_24B_DMA))

    SPI_setConfig(ADS93XXV_SPI_BASE,
                DEVICE_LSPCLK_FREQ,
                SPI_PROT_POL0PHA1,SPI_MODE_CONTROLLER,
                ADS93XXV_SPI_CLOCK_FREQ_HZ, 8U);


    SPI_setEmulationMode(ADS93XXV_SPI_BASE, SPI_EMULATION_STOP_MIDWAY);

    SPI_enableFIFO(ADS93XXV_SPI_BASE);

    SPI_clearInterruptStatus(ADS93XXV_SPI_BASE,
                             SPI_INT_RX_DATA_TX_EMPTY
                             |SPI_INT_RXFF|SPI_INT_TXFF );

    SPI_setFIFOInterruptLevel(ADS93XXV_SPI_BASE,
                              SPI_FIFO_TX0, SPI_FIFO_RX16);
    SPI_enableInterrupt(ADS93XXV_SPI_BASE, SPI_INT_RXFF|SPI_INT_TXFF);
    SPI_setFIFOInterruptLevel(ADS93XXV_SPI_BASE,
                              SPI_FIFO_TX8, SPI_FIFO_RX8);
    SPI_disableInterrupt(ADS93XXV_SPI_BASE, SPI_INT_RXFF|SPI_INT_TXFF);

#endif

    //
    // Configuration complete. Enable the module.
    //
    SPI_enableModule(ADS93XXV_SPI_BASE);
}

//
//==============================================================================
// ADS93XXV_HAL_setupSpiAdcRegReadWrite -
// Configure SPI interface for ADC reg read and write
//==============================================================================
//
void ADS93XXV_HAL_setupSpiAdcRegReadWrite()
{
    SPI_disableModule(ADS93XXV_SPI_BASE);
    SPI_setConfig(ADS93XXV_SPI_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA1,
                  SPI_MODE_CONTROLLER,
                  ADS93XXV_SPI_CLOCK_FREQ_HZ, ADS93XXV_SPI_DATAWIDTH_BIT);
    SPI_setPTESignalPolarity(ADS93XXV_SPI_BASE, SPI_PTE_ACTIVE_LOW);
    SPI_enableHighSpeedMode(ADS93XXV_SPI_BASE);
    SPI_enableFIFO(ADS93XXV_SPI_BASE);
    SPI_disableLoopback(ADS93XXV_SPI_BASE);
    SPI_setEmulationMode(ADS93XXV_SPI_BASE, SPI_EMULATION_FREE_RUN);
    SPI_enableModule(ADS93XXV_SPI_BASE);
}


//
//==============================================================================
// ADS93XXV_HAL_delay_ms - Function to introduce delay
//==============================================================================
//
void ADS93XXV_HAL_delay_ms(uint16_t milliseconds)
{
    // System clock looping 2000 cycles = 1ms delay
    DEVICE_DELAY_US(milliseconds*1000);
}

void ADS93XXV_HAL_delay_us(uint16_t microseconds)
{
    // Delay
    DEVICE_DELAY_US(microseconds);
}


//
//==============================================================================
// ADS93XXV_HAL_resetExtAdc
//==============================================================================
//
void ADS93XXV_HAL_resetExtAdc(void)
{
    // (OPTIONAL) Provide additional delay time for power supply settling
    DEVICE_DELAY_US(10000);

    // Reset the ADC
    GPIO_writePin(ADS93XXV_RST_GPIO_NUM, 0U);

    DEVICE_DELAY_US(1);

    // Remove the ADC from reset
    GPIO_writePin(ADS93XXV_RST_GPIO_NUM, 1U);

    // Delay for reference rest
    DEVICE_DELAY_US(10000);

}



void ADS93XXV_HAL_enableExtAdcInterrupt(void)
{

    SPI_resetRxFIFO(ADS93XXV_SPI_BASE);
    SPI_resetTxFIFO(ADS93XXV_SPI_BASE);
    SPI_clearInterruptStatus(ADS93XXV_SPI_BASE, SPI_INT_RXFF);

    Interrupt_enable(ADS93XXV_INT_DRDY);

#if(ADS93XXV_BUILD_NUM==ADS93XXV_16B_SPI_FIFO)
    Interrupt_enable(ADS93XXV_INT_SPI_RX);
#elif(ADS93XXV_BUILD_NUM == ADS93XXV_16B_DMA || ADS93XXV_BUILD_NUM == ADS93XXV_24B_DMA)
    Interrupt_enable(ADS93XXV_INT_HAL_RX_DMA);
#endif
}

void ADS93XXV_HAL_setupExtAdcInterrupt(void)
{
    Interrupt_register(ADS93XXV_INT_DRDY, &ADS93XXV_HAL_INT_DRDY_ISR);

#if(ADS93XXV_BUILD_NUM==ADS93XXV_16B_SPI_FIFO)
    Interrupt_register(ADS93XXV_INT_SPI_RX,
                       &ADS93XXV_ISR_SPI_RXFIFO);

#elif(ADS93XXV_BUILD_NUM == ADS93XXV_16B_DMA || ADS93XXV_BUILD_NUM == ADS93XXV_24B_DMA)

    Interrupt_register(ADS93XXV_INT_HAL_RX_DMA, &ADS93XXV_INT_RX_DMA_ISR);

#endif
}


//
//=============================================================================
// DMA Configurations
//=============================================================================
//
#if(ADS93XXV_BUILD_NUM==ADS93XXV_16B_DMA || ADS93XXV_BUILD_NUM==ADS93XXV_24B_DMA)

void ADS93XXV_HAL_initDmaSpi()
{
    DMA_initController();
    ADS93XXV_HAL_initDmaSpiRx();
    ADS93XXV_HAL_initDmaSpiTx();
}

void ADS93XXV_HAL_initDmaSpiTx()
{
    DMA_setEmulationMode(DMA_EMULATION_STOP);
    DMA_configAddresses(ADS93XXV_HAL_SPI_TX_DMA_BASE,
                        ADS93XXV_HAL_SPI_TX_DMA_ADDRESS, srcAddr);
    DMA_configBurst(ADS93XXV_HAL_SPI_TX_DMA_BASE,
                    ADS93XXV_HAL_SPI_TX_DMA_BURSTSIZE, 1, 0);
    DMA_configTransfer(ADS93XXV_HAL_SPI_TX_DMA_BASE,
                       ADS93XXV_HAL_SPI_TX_DMA_TRANSFERSIZE, 1, 0);
    DMA_configWrap(ADS93XXV_HAL_SPI_TX_DMA_BASE, 65535U, 0, 65535U, 0);
    DMA_configMode(ADS93XXV_HAL_SPI_TX_DMA_BASE,
                   ADS93XXV_HAL_SPI_TX_DMA_TRIGGER, DMA_CFG_ONESHOT_DISABLE
                   | DMA_CFG_CONTINUOUS_DISABLE | DMA_CFG_SIZE_16BIT);

//    DMA_setInterruptMode(ADS93XXV_HAL_SPI_TX_DMA_BASE, DMA_INT_AT_END);
//    DMA_enableInterrupt(ADS93XXV_HAL_SPI_TX_DMA_BASE);
    DMA_disableOverrunInterrupt(ADS93XXV_HAL_SPI_TX_DMA_BASE);
    DMA_enableTrigger(ADS93XXV_HAL_SPI_TX_DMA_BASE);
    DMA_stopChannel(ADS93XXV_HAL_SPI_TX_DMA_BASE);
}

void ADS93XXV_HAL_initDmaSpiRx(){
    DMA_setEmulationMode(DMA_EMULATION_STOP);
    DMA_configAddresses(ADS93XXV_HAL_SPI_RX_DMA_BASE, destAddr,
                        ADS93XXV_HAL_SPI_RX_DMA_ADDRESS);
    DMA_configBurst(ADS93XXV_HAL_SPI_RX_DMA_BASE,
                    ADS93XXV_HAL_SPI_RX_DMA_BURSTSIZE, 0, 1);
    DMA_configTransfer(ADS93XXV_HAL_SPI_RX_DMA_BASE,
                       ADS93XXV_HAL_SPI_RX_DMA_TRANSFERSIZE, 0, 1);
    DMA_configWrap(ADS93XXV_HAL_SPI_RX_DMA_BASE, 65535U, 0, 65535U, 0);
    DMA_configMode(ADS93XXV_HAL_SPI_RX_DMA_BASE,
                   ADS93XXV_HAL_SPI_RX_DMA_TRIGGER,
                   DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_DISABLE
                   | DMA_CFG_SIZE_16BIT);
    DMA_setInterruptMode(ADS93XXV_HAL_SPI_RX_DMA_BASE, DMA_INT_AT_END);
    DMA_enableInterrupt(ADS93XXV_HAL_SPI_RX_DMA_BASE);
    DMA_disableOverrunInterrupt(ADS93XXV_HAL_SPI_RX_DMA_BASE);
    DMA_enableTrigger(ADS93XXV_HAL_SPI_RX_DMA_BASE);
    DMA_stopChannel(ADS93XXV_HAL_SPI_RX_DMA_BASE);
}


//
//==============================================================================
// ADS93XXV_HAL_setupInterruptDmaRx - Setup interrupt for DMA Receive
//==============================================================================
//
void ADS93XXV_HAL_setupInterruptDmaRx(void)
{
    //
    // Interrupt Settings for DMA used for Receive
    // ISR need to be defined for the registered interrupts

    Interrupt_register(ADS93XXV_INT_HAL_RX_DMA, &ADS93XXV_INT_RX_DMA_ISR);
    Interrupt_enable(ADS93XXV_INT_HAL_RX_DMA);
}

#endif


void ADS93XXV_HAL_initUserVariables(void )
{
    //
    // Initialize the data buffers
    //

#if(ADS93XXV_BUILD_NUM==ADS93XXV_16B_DMA)
    uint16_t i_local=16;

    for(i_local = 0; i_local < 16; i_local++)
    {
        ADC_16b_sData[i_local] = 0;
        ADC_16b_rData[i_local]= 0;
    }
#elif(ADS93XXV_BUILD_NUM==ADS93XXV_24B_DMA)

    uint16_t i_local=48;

    for(i_local = 0; i_local < 48; i_local++)
    {
        ADC_24b_sData[i_local] = 0;
        ADC_24b_rData[i_local]= 0;
    }
#endif
}



//
//==============================================================================
// Initialize analog pins
//==============================================================================
//
void ADS93XXV_HAL_initAnalogPinMux()
{

    // PinMux for modules assigned to CPU1
    //

    //
    // ANALOG -> myANALOGPinMux0 Pinmux
    //
    // Analog PinMux for A0/DACA_OUT
    //
    GPIO_setPinConfig(GPIO_227_GPIO227);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(227, GPIO_ANALOG_ENABLED);
    // Analog PinMux for A1
    GPIO_setPinConfig(GPIO_228_GPIO228);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(228, GPIO_ANALOG_ENABLED);
    // Analog PinMux for A10, GPIO213
    GPIO_setPinConfig(GPIO_213_GPIO213);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(213, GPIO_ANALOG_ENABLED);
    // Analog PinMux for A11, GPIO214
    GPIO_setPinConfig(GPIO_214_GPIO214);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(214, GPIO_ANALOG_ENABLED);
    // Analog PinMux for A14/B14/C14
    GPIO_setPinConfig(GPIO_225_GPIO225);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(225, GPIO_ANALOG_ENABLED);
    // Analog PinMux for A15/B15/C15
    GPIO_setPinConfig(GPIO_226_GPIO226);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(226, GPIO_ANALOG_ENABLED);
    // Analog PinMux for A2
    GPIO_setPinConfig(GPIO_229_GPIO229);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(229, GPIO_ANALOG_ENABLED);
    // Analog PinMux for A3
    GPIO_setPinConfig(GPIO_230_GPIO230);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(230, GPIO_ANALOG_ENABLED);
    // Analog PinMux for A4
    GPIO_setPinConfig(GPIO_231_GPIO231);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(231, GPIO_ANALOG_ENABLED);
    // Analog PinMux for A5
    GPIO_setPinConfig(GPIO_232_GPIO232);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(232, GPIO_ANALOG_ENABLED);
    // Analog PinMux for A6, GPIO209
    GPIO_setPinConfig(GPIO_209_GPIO209);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(209, GPIO_ANALOG_ENABLED);
    // Analog PinMux for A7, GPIO210
    GPIO_setPinConfig(GPIO_210_GPIO210);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(210, GPIO_ANALOG_ENABLED);
    // Analog PinMux for A8, GPIO211
    GPIO_setPinConfig(GPIO_211_GPIO211);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(211, GPIO_ANALOG_ENABLED);
    // Analog PinMux for A9, GPIO212
    GPIO_setPinConfig(GPIO_212_GPIO212);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(212, GPIO_ANALOG_ENABLED);
    // Analog PinMux for B0/VDAC
    GPIO_setPinConfig(GPIO_233_GPIO233);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(233, GPIO_ANALOG_ENABLED);
    // Analog PinMux for B1/DACC_OUT
    GPIO_setPinConfig(GPIO_234_GPIO234);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(234, GPIO_ANALOG_ENABLED);
    // Analog PinMux for B10, GPIO219
    GPIO_setPinConfig(GPIO_219_GPIO219);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(219, GPIO_ANALOG_ENABLED);
    // Analog PinMux for B11
    GPIO_setPinConfig(GPIO_240_GPIO240);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(240, GPIO_ANALOG_ENABLED);
    // Analog PinMux for B13
    GPIO_setPinConfig(GPIO_238_GPIO238);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(238, GPIO_ANALOG_ENABLED);
    // Analog PinMux for B2
    GPIO_setPinConfig(GPIO_235_GPIO235);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(235, GPIO_ANALOG_ENABLED);
    // Analog PinMux for B3
    GPIO_setPinConfig(GPIO_236_GPIO236);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(236, GPIO_ANALOG_ENABLED);
    // Analog PinMux for B4, GPIO215
    GPIO_setPinConfig(GPIO_215_GPIO215);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(215, GPIO_ANALOG_ENABLED);
    // Analog PinMux for B5, GPIO216
    GPIO_setPinConfig(GPIO_216_GPIO216);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(216, GPIO_ANALOG_ENABLED);
    // Analog PinMux for B6, GPIO207
    GPIO_setPinConfig(GPIO_207_GPIO207);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(207, GPIO_ANALOG_ENABLED);
    // Analog PinMux for B7, GPIO208
    GPIO_setPinConfig(GPIO_208_GPIO208);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(208, GPIO_ANALOG_ENABLED);
    // Analog PinMux for B8, GPIO217
    GPIO_setPinConfig(GPIO_217_GPIO217);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(217, GPIO_ANALOG_ENABLED);
    // Analog PinMux for B9, GPIO218
    GPIO_setPinConfig(GPIO_218_GPIO218);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(218, GPIO_ANALOG_ENABLED);
    // Analog PinMux for C0, GPIO199
    GPIO_setPinConfig(GPIO_199_GPIO199);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(199, GPIO_ANALOG_ENABLED);
    // Analog PinMux for C10
    GPIO_setPinConfig(GPIO_241_GPIO241);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(241, GPIO_ANALOG_ENABLED);
    // Analog PinMux for C11
    GPIO_setPinConfig(GPIO_242_GPIO242);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(242, GPIO_ANALOG_ENABLED);
    // Analog PinMux for C13
    GPIO_setPinConfig(GPIO_239_GPIO239);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(239, GPIO_ANALOG_ENABLED);
    // Analog PinMux for C2
    GPIO_setPinConfig(GPIO_237_GPIO237);
    // AIO -> Analog mode selected
    GPIO_setAnalogMode(237, GPIO_ANALOG_ENABLED);
    // Analog PinMux for C3, GPIO206
    GPIO_setPinConfig(GPIO_206_GPIO206);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(206, GPIO_ANALOG_ENABLED);
    // Analog PinMux for C4, GPIO205
    GPIO_setPinConfig(GPIO_205_GPIO205);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(205, GPIO_ANALOG_ENABLED);
    // Analog PinMux for C5, GPIO204
    GPIO_setPinConfig(GPIO_204_GPIO204);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(204, GPIO_ANALOG_ENABLED);
    // Analog PinMux for C6, GPIO203
    GPIO_setPinConfig(GPIO_203_GPIO203);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(203, GPIO_ANALOG_ENABLED);
    // Analog PinMux for C1, GPIO200
    GPIO_setPinConfig(GPIO_200_GPIO200);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(200, GPIO_ANALOG_ENABLED);
    // Analog PinMux for C7, GPIO198
    GPIO_setPinConfig(GPIO_198_GPIO198);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(198, GPIO_ANALOG_ENABLED);
    // Analog PinMux for C8, GPIO202
    GPIO_setPinConfig(GPIO_202_GPIO202);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(202, GPIO_ANALOG_ENABLED);
    // Analog PinMux for C9, GPIO201
    GPIO_setPinConfig(GPIO_201_GPIO201);
    // AGPIO -> Analog mode selected
    GPIO_setAnalogMode(201, GPIO_ANALOG_ENABLED);
}

//
//==============================================================================
// ASYSCTL Configurations
//==============================================================================
//
void ADS93XXV_HAL_initAsysctl()
{
    //
    // Asysctl initialization
    // Disables the temperature sensor output to the ADC.
    //
    ASysCtl_disableTemperatureSensor();
    //
    // Set the analog voltage reference selection to internal.
    //
    ASysCtl_setAnalogReferenceInternal( ASYSCTL_VREFHIC );
    //
    // Set the internal analog voltage reference selection to 1.65V.
    //
    ASysCtl_setAnalogReference2P5( ASYSCTL_VREFHIC );

    //
    // Set the analog voltage reference selection to external.
    //
    // ASysCtl_setAnalogReferenceExternal(ASYSCTL_VREFHIA|ASYSCTL_VREFHIB|ASYSCTL_VREFHIC);

}

void ADS93XXV_HAL_enableGlobalInterrupt(void)
{
    EALLOW;
    //
    // Enable Global interrupt INTM
    //
    EINT;
    //
    // Enable Global real-time interrupt DBGM
    //
    ERTM;
    EDIS;
}