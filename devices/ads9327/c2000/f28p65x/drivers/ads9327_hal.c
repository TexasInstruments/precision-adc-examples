/**
 *
 * \copyright Copyright (C) 2025 Texas Instruments Incorporated - http://www.ti.com/
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

#include "ADS9327_settings.h"
#include "epwm.h"
#include "spi.h"
#include "ADS9327_hal.h"


// 1-lane mode data arrays for interrupt functions
uint16_t ADCA_data = 0;
uint16_t ADCB_data = 0;
uint16_t temp_var1 = 0;
uint16_t temp_var2 = 0;

// 4-lane mode data arrays for interrupt functions
uint16_t data_now_A_MSB, data_now_A_LSB, data_now_B_MSB, data_now_B_LSB = 0;
uint16_t fifoADC_A[MAX_ENTRIES];
uint16_t fifoADC_B[MAX_ENTRIES];
uint16_t fifoIndexA, fifoIndexB = 0;

//
//==============================================================================
// ADS9327_HAL_DeviceInit - Setup device
//==============================================================================
//
void ADS9327_HAL_DeviceInit(void)
{

    //
    // Initialize device clock and peripherals
    //
    Device_init(); // CHANGE

    //
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO(); // CHANGE

}

//
//==============================================================================
// ADS9327_HAL_configSpiExtAdcDataRead - Configure SPI interface for different lane modes
//==============================================================================
//
void ADS9327_HAL_configSpiExtAdcDataRead(void)
{
    //
    // Must put SPI into reset before configuring it
    //
    SPI_disableModule(mySPI0_BASE);

    //
    // SPI configuration. Use a 1MHz SPICLK and 16-bit word size.
    //
    SPI_enableHighSpeedMode(mySPI0_BASE);
    
    #if DATA_LANES_SELECT == 1

    SPI_setConfig(mySPI0_BASE,
                DEVICE_LSPCLK_FREQ,
                SPI_PROT_POL0PHA1,SPI_MODE_CONTROLLER,
                mySPI0_BITRATE, 16U);


    SPI_setEmulationMode(mySPI0_BASE, SPI_EMULATION_STOP_MIDWAY);

    SPI_enableFIFO(mySPI0_BASE);

    SPI_clearInterruptStatus(mySPI0_BASE,
                             SPI_INT_RX_DATA_TX_EMPTY
                             |SPI_INT_RXFF|SPI_INT_TXFF );

    SPI_setFIFOInterruptLevel(mySPI0_BASE,
                              SPI_FIFO_TX0, SPI_FIFO_RX3);

    SPI_enableInterrupt(mySPI0_BASE,
                        SPI_INT_RXFF|SPI_INT_TXFF);

    //
    // Configuration complete. Enable the module.
    //
    SPI_enableModule(mySPI0_BASE);

    #endif

    #if DATA_LANES_SELECT == 4

    // enable /CS pin in SPI peripheral
    GPIO_setPinConfig(mySPI0_SPIPTE_PIN_CONFIG_4LANE);
    GPIO_setPadConfig(mySPI0_SPIPTE_GPIO_4LANE, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(mySPI0_SPIPTE_GPIO_4LANE, GPIO_QUAL_ASYNC);

    SPI_setConfig(mySPI0_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA1,
                SPI_MODE_CONTROLLER, mySPI0_BITRATE, mySPI0_DATAWIDTH);
    
    SPI_setEmulationMode(mySPI0_BASE, SPI_EMULATION_STOP_MIDWAY);

    SPI_enableModule(mySPI0_BASE);

    #endif

}

//
//==============================================================================
// ADS9327_HAL_delay - Function to introduce delay
//==============================================================================
//
void ADS9327_HAL_delay(int milliseconds) 
{

    //System clock looping 2000 cycles = 1ms delay
    DEVICE_DELAY_US(milliseconds*1000);

}

//
//==============================================================================
// ADS9327_HAL_writeRegister - Function to write ADC registers using SPI
//==============================================================================
//
void ADS9327_HAL_writeRegister(uint8_t reg_addr, uint16_t reg_value)
{
    // configure 8-bit + 16-bit data into 24-bit SPI frame
    uint32_t reg_addr_value = ((uint32_t)reg_addr << 16) | (reg_value & 0xFFFF); 

    #if DATA_LANES_SELECT == 1

        // Transmit 24-bit frame

        SPI_transmit24Bits(mySPI0_BASE, reg_addr_value,0);
        
    #endif

    #if DATA_LANES_SELECT == 4

        GPIO_writePin(94, 0); 
        //controlling /CS pin using GPIO for better control at higher speeds in 4-lane mode

        // Transmit 24-bit frame
        SPI_transmit24Bits(mySPI0_BASE, reg_addr_value,0);
        
        GPIO_writePin(94, 1);
    
    #endif
}

//
//==============================================================================
// ADS9327_HAL_setupInterrupts - Setup interrupt routines ISR1 and ISR2
//==============================================================================
//
void ADS9327_HAL_setupInterrupts(void){

    Interrupt_register(INT_EPWM1, &ISR1);
    Interrupt_enable(INT_EPWM1);

    #if DATA_LANES_SELECT == 1

    SPI_resetRxFIFO(mySPI0_BASE);
    SPI_resetTxFIFO(mySPI0_BASE);
    SPI_clearInterruptStatus(mySPI0_BASE, SPI_INT_RXFF);

    Interrupt_register(INT_SPID_RX, &ISR2);
    Interrupt_enable(INT_SPID_RX);

    #endif

}

//
//==============================================================================
// ADS9327_HAL_setupEpwmExtAdcStart - Configure EPWM as CONVST signal for the ADC
// the external ADC
//==============================================================================
//
void ADS9327_HAL_setupEpwmExtAdcStart(void){


    //
    // Maximum supported ePWM clock speed is specified in the datasheet
    //
    EPWM_setClockPrescaler(ADS9327_EXT_ADC1_CONVST_EPWM_BASE,
                           EPWM_CLOCK_DIVIDER_1,
                           EPWM_HSCLOCK_DIVIDER_1);

    EPWM_setEmulationMode(ADS9327_EXT_ADC1_CONVST_EPWM_BASE,
                          EPWM_EMULATION_FREE_RUN);

    //
    // Configure ePWM for count-up operation
    //
    EPWM_setTimeBaseCounter(ADS9327_EXT_ADC1_CONVST_EPWM_BASE, 0);
    EPWM_setTimeBasePeriod(ADS9327_EXT_ADC1_CONVST_EPWM_BASE,
                           ADS9327_EXT_ADC1_CONVST_EPWM_PERIOD);
    EPWM_setPeriodLoadMode(ADS9327_EXT_ADC1_CONVST_EPWM_BASE,
                           EPWM_PERIOD_SHADOW_LOAD);
    EPWM_setTimeBaseCounterMode(ADS9327_EXT_ADC1_CONVST_EPWM_BASE,
                                EPWM_COUNTER_MODE_UP);
    EPWM_disablePhaseShiftLoad(ADS9327_EXT_ADC1_CONVST_EPWM_BASE);

    //
    // Set Compare values for duty cycle
    //
    EPWM_setCounterCompareValue(ADS9327_EXT_ADC1_CONVST_EPWM_BASE,
                                EPWM_COUNTER_COMPARE_A,
                                ADS9327_EXT_ADC1_CONVST_ONTIME_TICKS);
    EPWM_setCounterCompareValue(ADS9327_EXT_ADC1_CONVST_EPWM_BASE,
                            EPWM_COUNTER_COMPARE_B,
                            ADS9327_EXT_ADC1_CONVST_EPWM_PERIOD/2);


    //
    // Use shadow mode to update CMPA on TBPRD
    //
    EPWM_setCounterCompareShadowLoadMode(ADS9327_EXT_ADC1_CONVST_EPWM_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_PERIOD);
    EPWM_setCounterCompareShadowLoadMode(ADS9327_EXT_ADC1_CONVST_EPWM_BASE,
                                         EPWM_COUNTER_COMPARE_B,
                                         EPWM_COMP_LOAD_ON_CNTR_PERIOD);


    //
    // Configure Action Qualifier SubModule to:
    //

    //
    // Use shadow mode to update AQCTL
    //
    EPWM_setActionQualifierShadowLoadMode(ADS9327_EXT_ADC1_CONVST_EPWM_BASE,
                                          EPWM_ACTION_QUALIFIER_A,
                                          EPWM_AQ_LOAD_ON_CNTR_PERIOD);
    EPWM_setActionQualifierAction(ADS9327_EXT_ADC1_CONVST_EPWM_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);

    EPWM_setActionQualifierAction(ADS9327_EXT_ADC1_CONVST_EPWM_BASE,
                                      EPWM_AQ_OUTPUT_A,
                                      EPWM_AQ_OUTPUT_LOW,
                                      EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    EPWM_enableInterrupt(ADS9327_EXT_ADC1_CONVST_EPWM_BASE);

    #if DATA_LANES_SELECT == 1

    EPWM_setInterruptSource(ADS9327_EXT_ADC1_CONVST_EPWM_BASE, EPWM_INT_TBCTR_U_CMPA);

    #elif DATA_LANES_SELECT == 4

    EPWM_setInterruptSource(ADS9327_EXT_ADC1_CONVST_EPWM_BASE, EPWM_INT_TBCTR_U_CMPB);
    
    #endif
    
    EPWM_setInterruptEventCount(ADS9327_EXT_ADC1_CONVST_EPWM_BASE, 1);


}

void ADS9327_HAL_setupGpioExtAdcStart(void){
    GPIO_setPadConfig(ADS9327_EXT_ADC1_CONVST_EPWM_GPIO_NUM, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(ADS9327_EXT_ADC1_CONVST_EPWM_GPIO_NUM, 0);
    GPIO_setDirectionMode(ADS9327_EXT_ADC1_CONVST_EPWM_GPIO_NUM, GPIO_DIR_MODE_OUT);
    GPIO_setPinConfig(ADS9327_EXT_ADC1_CONVST_PIN_CONFIG_EPWM);
    GPIO_setQualificationMode(ADS9327_EXT_ADC1_CONVST_EPWM_GPIO_NUM, GPIO_QUAL_SYNC);
}

//
//==============================================================================
// Enables EPWM peripheral clock
//==============================================================================
//
void ADS9327_HAL_enableEpwmCounting(void)
{
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}
//
//==============================================================================
// Disables EPWM peripheral clock
//==============================================================================
//
void ADS9327_HAL_disableEpwmCounting(void){

    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
}

//*****************************************************************************
//
// Board Configurations
// Initializes the rest of the modules. 
// Call this function in your application if you wish to do all module 
// initialization.
// If you wish to not use some of the initializations, instead of the 
// Board_init use the individual Module_inits
//
//*****************************************************************************
void ADS9327_HAL_Board_init()
{
    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();
    
	EALLOW;

	PinMux_init();
	SYSCTL_init();
	MEMCFG_init();
	CLB_init();
	CLB_INPUTXBAR_init();
	GPIO_init();
	SPI_init();

	EDIS;
}

//*****************************************************************************
//
// PINMUX Configurations
//
//*****************************************************************************
void PinMux_init()
{
	//
	// PinMux for modules assigned to CPU1
	//


    #if DATA_LANES_SELECT == 4
        // GPIO94 -> nChip_select Pinmux
        GPIO_setPinConfig(GPIO_94_GPIO94);
    #endif

	// GPIO115 -> ISR_check Pinmux
	GPIO_setPinConfig(GPIO_115_GPIO115);
	// GPIO10 -> CLB_input1 Pinmux
	GPIO_setPinConfig(GPIO_10_GPIO10);
	// GPIO11 -> CLB_input2 Pinmux
	GPIO_setPinConfig(GPIO_11_GPIO11);
	// GPIO8 -> CLB_input3 Pinmux
	GPIO_setPinConfig(GPIO_8_GPIO8);
	// GPIO9 -> CLB_input4 Pinmux
	GPIO_setPinConfig(GPIO_9_GPIO9);
    
	//
	// SPID -> mySPI0 Pinmux
	//
	GPIO_setPinConfig(mySPI0_SPIPICO_PIN_CONFIG);
	GPIO_setPadConfig(mySPI0_SPIPICO_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(mySPI0_SPIPICO_GPIO, GPIO_QUAL_ASYNC);

	GPIO_setPinConfig(mySPI0_SPIPOCI_PIN_CONFIG);
	GPIO_setPadConfig(mySPI0_SPIPOCI_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(mySPI0_SPIPOCI_GPIO, GPIO_QUAL_ASYNC);

	GPIO_setPinConfig(mySPI0_SPICLK_PIN_CONFIG);
	GPIO_setPadConfig(mySPI0_SPICLK_GPIO, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(mySPI0_SPICLK_GPIO, GPIO_QUAL_ASYNC);

    #if DATA_LANES_SELECT == 1
        GPIO_setPinConfig(mySPI0_SPIPTE_PIN_CONFIG);
        GPIO_setPadConfig(mySPI0_SPIPTE_GPIO, GPIO_PIN_TYPE_STD);
        GPIO_setQualificationMode(mySPI0_SPIPTE_GPIO, GPIO_QUAL_ASYNC);
    #endif


}

//*****************************************************************************
//
// CLB Configurations
//
//*****************************************************************************
void CLB_init(){
	D3_D2_init();
	D1_D0_init();
}

void D3_D2_init(){
	CLB_setOutputMask(D3_D2_BASE,
				(0UL << 0UL), true);
	CLB_enableOutputMaskUpdates(D3_D2_BASE);
	//
	// D3_D2 CLB_IN0 initialization
	//
	// The following functions configure the CLB input mux and whether the inputs
	// have synchronization or pipeline enabled; check the device manual for more
	// information on when a signal needs to be synchronized or go through a
	// pipeline filter
	//
	CLB_configLocalInputMux(D3_D2_BASE, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
	CLB_configGlobalInputMux(D3_D2_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_SPI4_SPICLK);
	CLB_configGPInputMux(D3_D2_BASE, CLB_IN0, CLB_GP_IN_MUX_EXTERNAL);
	CLB_enableSynchronization(D3_D2_BASE, CLB_IN0);
	CLB_selectInputFilter(D3_D2_BASE, CLB_IN0, CLB_FILTER_RISING_EDGE);
	CLB_disableInputPipelineMode(D3_D2_BASE, CLB_IN0);
	//
	// D3_D2 CLB_IN1 initialization
	//
	// The following functions configure the CLB input mux and whether the inputs
	// have synchronization or pipeline enabled; check the device manual for more
	// information on when a signal needs to be synchronized or go through a
	// pipeline filter
	//
	CLB_configLocalInputMux(D3_D2_BASE, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
	CLB_configGlobalInputMux(D3_D2_BASE, CLB_IN1, CLB_GLOBAL_IN_MUX_SPI4_SPIPTE);
	CLB_configGPInputMux(D3_D2_BASE, CLB_IN1, CLB_GP_IN_MUX_EXTERNAL);
	CLB_enableSynchronization(D3_D2_BASE, CLB_IN1);
	CLB_selectInputFilter(D3_D2_BASE, CLB_IN1, CLB_FILTER_NONE);
	CLB_disableInputPipelineMode(D3_D2_BASE, CLB_IN1);
	//
	// D3_D2 CLB_IN2 initialization
	//
	// The following functions configure the CLB input mux and whether the inputs
	// have synchronization or pipeline enabled; check the device manual for more
	// information on when a signal needs to be synchronized or go through a
	// pipeline filter
	//
	CLB_configLocalInputMux(D3_D2_BASE, CLB_IN2, CLB_LOCAL_IN_MUX_GLOBAL_IN);
	CLB_configGlobalInputMux(D3_D2_BASE, CLB_IN2, CLB_GLOBAL_IN_MUX_SPI4_SPIPTE);
	CLB_configGPInputMux(D3_D2_BASE, CLB_IN2, CLB_GP_IN_MUX_EXTERNAL);
	CLB_enableSynchronization(D3_D2_BASE, CLB_IN2);
	CLB_selectInputFilter(D3_D2_BASE, CLB_IN2, CLB_FILTER_RISING_EDGE);
	CLB_disableInputPipelineMode(D3_D2_BASE, CLB_IN2);
	//
	// D3_D2 CLB_IN3 initialization
	//
	// The following functions configure the CLB input mux and whether the inputs
	// have synchronization or pipeline enabled; check the device manual for more
	// information on when a signal needs to be synchronized or go through a
	// pipeline filter
	//
	CLB_configLocalInputMux(D3_D2_BASE, CLB_IN3, CLB_LOCAL_IN_MUX_INPUT1);
	CLB_configGlobalInputMux(D3_D2_BASE, CLB_IN3, CLB_GLOBAL_IN_MUX_EPWM1A);
	CLB_configGPInputMux(D3_D2_BASE, CLB_IN3, CLB_GP_IN_MUX_EXTERNAL);
	CLB_enableSynchronization(D3_D2_BASE, CLB_IN3);
	CLB_selectInputFilter(D3_D2_BASE, CLB_IN3, CLB_FILTER_NONE);
	CLB_disableInputPipelineMode(D3_D2_BASE, CLB_IN3);
	//
	// D3_D2 CLB_IN4 initialization
	//
	// The following functions configure the CLB input mux and whether the inputs
	// have synchronization or pipeline enabled; check the device manual for more
	// information on when a signal needs to be synchronized or go through a
	// pipeline filter
	//
	CLB_configLocalInputMux(D3_D2_BASE, CLB_IN4, CLB_LOCAL_IN_MUX_INPUT2);
	CLB_configGlobalInputMux(D3_D2_BASE, CLB_IN4, CLB_GLOBAL_IN_MUX_EPWM1A);
	CLB_configGPInputMux(D3_D2_BASE, CLB_IN4, CLB_GP_IN_MUX_EXTERNAL);
	CLB_enableSynchronization(D3_D2_BASE, CLB_IN4);
	CLB_selectInputFilter(D3_D2_BASE, CLB_IN4, CLB_FILTER_NONE);
	CLB_disableInputPipelineMode(D3_D2_BASE, CLB_IN4);
	//
	// D3_D2 CLB_IN5 initialization
	//
	// The following functions configure the CLB input mux and whether the inputs
	// have synchronization or pipeline enabled; check the device manual for more
	// information on when a signal needs to be synchronized or go through a
	// pipeline filter
	//
	CLB_configLocalInputMux(D3_D2_BASE, CLB_IN5, CLB_LOCAL_IN_MUX_GLOBAL_IN);
	CLB_configGlobalInputMux(D3_D2_BASE, CLB_IN5, CLB_GLOBAL_IN_MUX_SPI4_SPIPTE);
	CLB_configGPInputMux(D3_D2_BASE, CLB_IN5, CLB_GP_IN_MUX_EXTERNAL);
	CLB_enableSynchronization(D3_D2_BASE, CLB_IN5);
	CLB_selectInputFilter(D3_D2_BASE, CLB_IN5, CLB_FILTER_FALLING_EDGE);
	CLB_disableInputPipelineMode(D3_D2_BASE, CLB_IN5);
	CLB_setGPREG(D3_D2_BASE,0);

	initTILE0(D3_D2_BASE);
	CLB_enableCLB(D3_D2_BASE);
}
void D1_D0_init(){
	CLB_setOutputMask(D1_D0_BASE,
				(0UL << 0UL), true);
	CLB_enableOutputMaskUpdates(D1_D0_BASE);
	//
	// D1_D0 CLB_IN0 initialization
	//
	// The following functions configure the CLB input mux and whether the inputs
	// have synchronization or pipeline enabled; check the device manual for more
	// information on when a signal needs to be synchronized or go through a
	// pipeline filter
	//
	CLB_configLocalInputMux(D1_D0_BASE, CLB_IN0, CLB_LOCAL_IN_MUX_GLOBAL_IN);
	CLB_configGlobalInputMux(D1_D0_BASE, CLB_IN0, CLB_GLOBAL_IN_MUX_SPI4_SPICLK);
	CLB_configGPInputMux(D1_D0_BASE, CLB_IN0, CLB_GP_IN_MUX_EXTERNAL);
	CLB_enableSynchronization(D1_D0_BASE, CLB_IN0);
	CLB_selectInputFilter(D1_D0_BASE, CLB_IN0, CLB_FILTER_RISING_EDGE);
	CLB_disableInputPipelineMode(D1_D0_BASE, CLB_IN0);
	//
	// D1_D0 CLB_IN1 initialization
	//
	// The following functions configure the CLB input mux and whether the inputs
	// have synchronization or pipeline enabled; check the device manual for more
	// information on when a signal needs to be synchronized or go through a
	// pipeline filter
	//
	CLB_configLocalInputMux(D1_D0_BASE, CLB_IN1, CLB_LOCAL_IN_MUX_GLOBAL_IN);
	CLB_configGlobalInputMux(D1_D0_BASE, CLB_IN1, CLB_GLOBAL_IN_MUX_SPI4_SPIPTE);
	CLB_configGPInputMux(D1_D0_BASE, CLB_IN1, CLB_GP_IN_MUX_EXTERNAL);
	CLB_enableSynchronization(D1_D0_BASE, CLB_IN1);
	CLB_selectInputFilter(D1_D0_BASE, CLB_IN1, CLB_FILTER_NONE);
	CLB_disableInputPipelineMode(D1_D0_BASE, CLB_IN1);
	//
	// D1_D0 CLB_IN2 initialization
	//
	// The following functions configure the CLB input mux and whether the inputs
	// have synchronization or pipeline enabled; check the device manual for more
	// information on when a signal needs to be synchronized or go through a
	// pipeline filter
	//
	CLB_configLocalInputMux(D1_D0_BASE, CLB_IN2, CLB_LOCAL_IN_MUX_GLOBAL_IN);
	CLB_configGlobalInputMux(D1_D0_BASE, CLB_IN2, CLB_GLOBAL_IN_MUX_SPI4_SPIPTE);
	CLB_configGPInputMux(D1_D0_BASE, CLB_IN2, CLB_GP_IN_MUX_EXTERNAL);
	CLB_enableSynchronization(D1_D0_BASE, CLB_IN2);
	CLB_selectInputFilter(D1_D0_BASE, CLB_IN2, CLB_FILTER_RISING_EDGE);
	CLB_disableInputPipelineMode(D1_D0_BASE, CLB_IN2);
	//
	// D1_D0 CLB_IN3 initialization
	//
	// The following functions configure the CLB input mux and whether the inputs
	// have synchronization or pipeline enabled; check the device manual for more
	// information on when a signal needs to be synchronized or go through a
	// pipeline filter
	//
	CLB_configLocalInputMux(D1_D0_BASE, CLB_IN3, CLB_LOCAL_IN_MUX_INPUT3);
	CLB_configGlobalInputMux(D1_D0_BASE, CLB_IN3, CLB_GLOBAL_IN_MUX_EPWM1A);
	CLB_configGPInputMux(D1_D0_BASE, CLB_IN3, CLB_GP_IN_MUX_EXTERNAL);
	CLB_enableSynchronization(D1_D0_BASE, CLB_IN3);
	CLB_selectInputFilter(D1_D0_BASE, CLB_IN3, CLB_FILTER_NONE);
	CLB_disableInputPipelineMode(D1_D0_BASE, CLB_IN3);
	//
	// D1_D0 CLB_IN4 initialization
	//
	// The following functions configure the CLB input mux and whether the inputs
	// have synchronization or pipeline enabled; check the device manual for more
	// information on when a signal needs to be synchronized or go through a
	// pipeline filter
	//
	CLB_configLocalInputMux(D1_D0_BASE, CLB_IN4, CLB_LOCAL_IN_MUX_INPUT4);
	CLB_configGlobalInputMux(D1_D0_BASE, CLB_IN4, CLB_GLOBAL_IN_MUX_EPWM1A);
	CLB_configGPInputMux(D1_D0_BASE, CLB_IN4, CLB_GP_IN_MUX_EXTERNAL);
	CLB_enableSynchronization(D1_D0_BASE, CLB_IN4);
	CLB_selectInputFilter(D1_D0_BASE, CLB_IN4, CLB_FILTER_NONE);
	CLB_disableInputPipelineMode(D1_D0_BASE, CLB_IN4);
	//
	// D1_D0 CLB_IN5 initialization
	//
	// The following functions configure the CLB input mux and whether the inputs
	// have synchronization or pipeline enabled; check the device manual for more
	// information on when a signal needs to be synchronized or go through a
	// pipeline filter
	//
	CLB_configLocalInputMux(D1_D0_BASE, CLB_IN5, CLB_LOCAL_IN_MUX_GLOBAL_IN);
	CLB_configGlobalInputMux(D1_D0_BASE, CLB_IN5, CLB_GLOBAL_IN_MUX_SPI4_SPIPTE);
	CLB_configGPInputMux(D1_D0_BASE, CLB_IN5, CLB_GP_IN_MUX_EXTERNAL);
	CLB_enableSynchronization(D1_D0_BASE, CLB_IN5);
	CLB_selectInputFilter(D1_D0_BASE, CLB_IN5, CLB_FILTER_FALLING_EDGE);
	CLB_disableInputPipelineMode(D1_D0_BASE, CLB_IN5);
	CLB_setGPREG(D1_D0_BASE,0);

	initTILE0(D1_D0_BASE);
	CLB_enableCLB(D1_D0_BASE);
}

//*****************************************************************************
//
// CLBINPUTXBAR Configurations
//
//*****************************************************************************
void CLB_INPUTXBAR_init(){
	D3_init();
	D2_init();
	D1_init();
	D0_init();
}

void D3_init(){
	XBAR_setInputPin(CLBINPUTXBAR_BASE, D3_INPUT, D3_SOURCE);
}
void D2_init(){
	XBAR_setInputPin(CLBINPUTXBAR_BASE, D2_INPUT, D2_SOURCE);
}
void D1_init(){
	XBAR_setInputPin(CLBINPUTXBAR_BASE, D1_INPUT, D1_SOURCE);
}
void D0_init(){
	XBAR_setInputPin(CLBINPUTXBAR_BASE, D0_INPUT, D0_SOURCE);
}

//*****************************************************************************
//
// GPIO Configurations
//
//*****************************************************************************
void GPIO_init(){
    #if DATA_LANES_SELECT == 4
	nChip_select_init();
    #endif

    ISR_check_init();
    CLB_input1_init();
	CLB_input2_init();
	CLB_input3_init();
	CLB_input4_init();
}

#if DATA_LANES_SELECT == 4

void nChip_select_init(){
	GPIO_setPadConfig(nChip_select, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(nChip_select, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(nChip_select, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(nChip_select, GPIO_CORE_CPU1);

    GPIO_writePin(94, 1); // set /CS high as it is active low
}

#endif

void ISR_check_init(){
	GPIO_setPadConfig(ISR_check, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(ISR_check, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(ISR_check, GPIO_DIR_MODE_OUT);
	GPIO_setControllerCore(ISR_check, GPIO_CORE_CPU1);
}

void CLB_input1_init(){
	GPIO_setPadConfig(CLB_input1, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CLB_input1, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CLB_input1, GPIO_DIR_MODE_IN);
	GPIO_setControllerCore(CLB_input1, GPIO_CORE_CPU1);
}
void CLB_input2_init(){
	GPIO_setPadConfig(CLB_input2, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CLB_input2, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CLB_input2, GPIO_DIR_MODE_IN);
	GPIO_setControllerCore(CLB_input2, GPIO_CORE_CPU1);
}
void CLB_input3_init(){
	GPIO_setPadConfig(CLB_input3, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CLB_input3, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CLB_input3, GPIO_DIR_MODE_IN);
	GPIO_setControllerCore(CLB_input3, GPIO_CORE_CPU1);
}
void CLB_input4_init(){
	GPIO_setPadConfig(CLB_input4, GPIO_PIN_TYPE_STD);
	GPIO_setQualificationMode(CLB_input4, GPIO_QUAL_SYNC);
	GPIO_setDirectionMode(CLB_input4, GPIO_DIR_MODE_IN);
	GPIO_setControllerCore(CLB_input4, GPIO_CORE_CPU1);
}

//*****************************************************************************
//
// MEMCFG Configurations
//
//*****************************************************************************
void MEMCFG_init(){
	//
	// Initialize RAMs
	//
	//
	// Configure LSRAMs
	//
	MemCfg_setLSRAMControllerSel(MEMCFG_SECT_LS0, MEMCFG_LSRAMCONTROLLER_CPU_ONLY);
	MemCfg_setLSRAMControllerSel(MEMCFG_SECT_LS1, MEMCFG_LSRAMCONTROLLER_CPU_ONLY);
	MemCfg_setLSRAMControllerSel(MEMCFG_SECT_LS2, MEMCFG_LSRAMCONTROLLER_CPU_ONLY);
	MemCfg_setLSRAMControllerSel(MEMCFG_SECT_LS3, MEMCFG_LSRAMCONTROLLER_CPU_ONLY);
	MemCfg_setLSRAMControllerSel(MEMCFG_SECT_LS4, MEMCFG_LSRAMCONTROLLER_CPU_ONLY);
	MemCfg_setLSRAMControllerSel(MEMCFG_SECT_LS5, MEMCFG_LSRAMCONTROLLER_CPU_ONLY);
	MemCfg_setLSRAMControllerSel(MEMCFG_SECT_LS6, MEMCFG_LSRAMCONTROLLER_CPU_ONLY);
	MemCfg_setLSRAMControllerSel(MEMCFG_SECT_LS7, MEMCFG_LSRAMCONTROLLER_CPU_ONLY);
	MemCfg_setLSRAMControllerSel(MEMCFG_SECT_LS8, MEMCFG_LSRAMCONTROLLER_CPU_ONLY);
	MemCfg_setLSRAMControllerSel(MEMCFG_SECT_LS9, MEMCFG_LSRAMCONTROLLER_CPU_ONLY);
	//
	// Configure GSRAMs
	//
	MemCfg_setGSRAMControllerSel(MEMCFG_SECT_GS0, MEMCFG_GSRAMCONTROLLER_CPU1);
	MemCfg_setGSRAMControllerSel(MEMCFG_SECT_GS1, MEMCFG_GSRAMCONTROLLER_CPU1);
	MemCfg_setGSRAMControllerSel(MEMCFG_SECT_GS2, MEMCFG_GSRAMCONTROLLER_CPU1);
	MemCfg_setGSRAMControllerSel(MEMCFG_SECT_GS3, MEMCFG_GSRAMCONTROLLER_CPU1);
	MemCfg_setGSRAMControllerSel(MEMCFG_SECT_GS4, MEMCFG_GSRAMCONTROLLER_CPU1);
	//
	// Configure DRAMs
	//
	SysCtl_allocateDxRAM(SYSCTL_D2RAM, SYSCTL_CPUSEL_CPU1);
	SysCtl_allocateDxRAM(SYSCTL_D3RAM, SYSCTL_CPUSEL_CPU1);
	SysCtl_allocateDxRAM(SYSCTL_D4RAM, SYSCTL_CPUSEL_CPU1);
	SysCtl_allocateDxRAM(SYSCTL_D5RAM, SYSCTL_CPUSEL_CPU1);
	//
	// Configure GS FLASH
	//
	SysCtl_allocateFlashBank(SYSCTL_FLASH_BANK0, SYSCTL_CPUSEL_CPU1);
	SysCtl_allocateFlashBank(SYSCTL_FLASH_BANK1, SYSCTL_CPUSEL_CPU1);
	SysCtl_allocateFlashBank(SYSCTL_FLASH_BANK2, SYSCTL_CPUSEL_CPU1);
	SysCtl_allocateFlashBank(SYSCTL_FLASH_BANK3, SYSCTL_CPUSEL_CPU1);
	SysCtl_allocateFlashBank(SYSCTL_FLASH_BANK4, SYSCTL_CPUSEL_CPU1);
	//
	// ROM Access Configuration
	//
	MemCfg_enableROMWaitState();
	//
	// Configure Access Protection for RAMs
	//
	MemCfg_setProtection(MEMCFG_SECT_M0, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_M1, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_D0, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_D1, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_D2, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_D3, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_D4, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_D5, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_LS0, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_LS1, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_LS2, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_LS3, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_LS4, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_LS5, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_LS6, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_LS7, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_LS8, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_LS9, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE);
	MemCfg_setProtection(MEMCFG_SECT_GS0, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE | MEMCFG_PROT_ALLOWDMAWRITE);
	MemCfg_setProtection(MEMCFG_SECT_GS1, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE | MEMCFG_PROT_ALLOWDMAWRITE);
	MemCfg_setProtection(MEMCFG_SECT_GS2, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE | MEMCFG_PROT_ALLOWDMAWRITE);
	MemCfg_setProtection(MEMCFG_SECT_GS3, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE | MEMCFG_PROT_ALLOWDMAWRITE);
	MemCfg_setProtection(MEMCFG_SECT_GS4, MEMCFG_PROT_ALLOWCPUFETCH | MEMCFG_PROT_ALLOWCPUWRITE | MEMCFG_PROT_ALLOWDMAWRITE);
	//
	// Lock/Commit Registers
	//
	//
	// Enable Access Violation Interrupt
	//
	//
	// Correctable error Interrupt
	//
	MemCfg_setCorrErrorThreshold(0);
	MemCfg_disableCorrErrorInterrupt(MEMCFG_CERR_CPUREAD);
}        
//*****************************************************************************
//
// SPI Configurations
//
//*****************************************************************************
void SPI_init(){
	mySPI0_init();
}

void mySPI0_init(){
	SPI_disableModule(mySPI0_BASE);
	SPI_setConfig(mySPI0_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA1,
				  SPI_MODE_CONTROLLER, mySPI0_BITRATE, mySPI0_DATAWIDTH);
	SPI_setPTESignalPolarity(mySPI0_BASE, SPI_PTE_ACTIVE_LOW);
	SPI_enableHighSpeedMode(mySPI0_BASE);
	SPI_enableFIFO(mySPI0_BASE);
	SPI_disableLoopback(mySPI0_BASE);
	SPI_setEmulationMode(mySPI0_BASE, SPI_EMULATION_FREE_RUN);
	SPI_enableModule(mySPI0_BASE);
}

//*****************************************************************************
//
// SYSCTL Configurations
//
//*****************************************************************************
void SYSCTL_init(){
	//
    // sysctl initialization
	//
    SysCtl_setStandbyQualificationPeriod(2);
    SysCtl_configureType(SYSCTL_USBTYPE, 0, 0);
    SysCtl_configureType(SYSCTL_ECAPTYPE, 0, 0);
    SysCtl_configureType(SYSCTL_SDFMTYPE, 0, 0);
    SysCtl_configureType(SYSCTL_MEMMAPTYPE, 0, 0);
    SysCtl_selectErrPinPolarity(0);

    SysCtl_disableMCD();

    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM3, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM4, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM5, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM6, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM7, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM8, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM9, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM10, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM11, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM12, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM13, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM14, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM15, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM16, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM17, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPWM18, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ECAP1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ECAP2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ECAP3, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ECAP4, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ECAP5, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ECAP6, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ECAP7, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EQEP1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EQEP2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EQEP3, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EQEP4, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EQEP5, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EQEP6, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SD1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SD2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SD3, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SD4, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SCIA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SCIB, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SPIA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SPIB, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SPIC, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_SPID, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_I2CA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_I2CB, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CANA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCB, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCC, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCCHECKER1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCCHECKER2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCCHECKER3, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCCHECKER4, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCCHECKER5, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCCHECKER6, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCCHECKER7, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ADCCHECKER8, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS3, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS4, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS5, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS6, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS7, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS8, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS9, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS10, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CMPSS11, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_DCC0, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_DCC1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_DCC2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_DACA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_DACC, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CLB1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CLB2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CLB3, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CLB4, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CLB5, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_CLB6, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_FSITXA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_FSITXB, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_FSIRXA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_FSIRXB, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_FSIRXC, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_FSIRXD, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_LINA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_LINB, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_ECAT, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_HRCAL0, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_HRCAL1, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_HRCAL2, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_AESA, SYSCTL_CPUSEL_CPU1);
    SysCtl_selectCPUForPeripheralInstance(SYSCTL_CPUSEL_EPG1, SYSCTL_CPUSEL_CPU1);

    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCB, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCB, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCC, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCC, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ADCC, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS1, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS2, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS3, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS3, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS3, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS4, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS4, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS4, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS5, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS5, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS5, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS6, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS6, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS6, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS7, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS7, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS7, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS8, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS8, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS8, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS9, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS9, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS9, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS10, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS10, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS10, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS11, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS11, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CMPSS11, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACC, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACC, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_DACC, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM1, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM2, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM3, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM3, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM3, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM4, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM4, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM4, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM5, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM5, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM5, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM6, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM6, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM6, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM7, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM7, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM7, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM8, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM8, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM8, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM9, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM9, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM9, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM10, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM10, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM10, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM11, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM11, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM11, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM12, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM12, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM12, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM13, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM13, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM13, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM14, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM14, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM14, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM15, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM15, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM15, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM16, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM16, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM16, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM17, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM17, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM17, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM18, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM18, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EPWM18, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP1, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP2, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP3, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP3, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP3, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP4, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP4, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP4, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP5, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP5, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP5, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP6, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP6, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_EQEP6, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP1, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP2, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP3, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP3, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP3, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP4, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP4, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP4, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP5, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP5, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP5, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP6, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP6, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP6, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP7, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP7, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAP7, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM1, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM2, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM3, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM3, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM3, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM4, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM4, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SDFM4, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB1, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB2, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB3, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB3, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB3, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB4, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB4, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB4, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB5, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB5, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB5, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB6, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB6, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CLB6, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIB, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SCIB, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIB, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIB, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIC, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIC, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPIC, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPID, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPID, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_SPID, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CB, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_I2CB, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_PMBUSA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_PMBUSA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_PMBUSA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINB, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_LINB, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CANA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CANA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_CANA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_MCANA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_MCANA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_MCANA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_MCANB, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_MCANB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_MCANB, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIATX, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIATX, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIATX, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIARX, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIARX, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIARX, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIBTX, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIBTX, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIBTX, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIBRX, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIBRX, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIBRX, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSICRX, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSICRX, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSICRX, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIDRX, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIDRX, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_FSIDRX, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_USBA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_USBA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_USBA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM0, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM0, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM0, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM1, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM1, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM1, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM2, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM2, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_HRPWM2, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAT, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAT, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_ECAT, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_AESA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_AESA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_AESA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_UARTA, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_UARTA, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_UARTA, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_UARTB, 
        SYSCTL_ACCESS_CPUX, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_UARTB, 
        SYSCTL_ACCESS_CLA1, SYSCTL_ACCESS_FULL);
    SysCtl_setPeripheralAccessControl(SYSCTL_ACCESS_UARTB, 
        SYSCTL_ACCESS_DMA1, SYSCTL_ACCESS_FULL);

    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLA1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DMA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER0);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TIMER2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CPUBGCRC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLA1BGCRC);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_GTBCLKSYNC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ERAD);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EMIF1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM7);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM8);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM9);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM10);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM11);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM12);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM13);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM14);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM15);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM16);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM17);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM18);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAP7);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EQEP6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SD1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SD2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SD3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SD4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SCIB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_UARTA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_UARTB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPIC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_SPID);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_I2CA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_I2CB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_PMBUSA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CANA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_MCANA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_MCANB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_USBA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS7);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS8);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS9);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS10);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CMPSS11);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DACC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_CLB6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_FSITXA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_FSITXB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_FSIRXA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_FSIRXB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_FSIRXC);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_FSIRXD);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_LINA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_LINB);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DCC0);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DCC1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_DCC2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ECAT);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_HRCAL0);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_HRCAL1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_HRCAL2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_AESA);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPG1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCCHECKER1);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCCHECKER2);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCCHECKER3);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCCHECKER4);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCCHECKER5);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCCHECKER6);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCCHECKER7);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCCHECKER8);
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_ADCSEAGGRCPU1);



}

//*****************************************************************************
//
// CLB configuration
//
//*****************************************************************************


const uint32_t TILE0_HLC_initFIFOData[4] = {TILE0_HLC_FIFO0_INIT, TILE0_HLC_FIFO1_INIT, TILE0_HLC_FIFO2_INIT, TILE0_HLC_FIFO3_INIT};

uint16_t TILE0HLCInstr[CLB_NUM_HLC_INSTR + 1] =
{
    TILE0_HLCINSTR_0,
    TILE0_HLCINSTR_1,
    TILE0_HLCINSTR_2,
    TILE0_HLCINSTR_3,
    TILE0_HLCINSTR_4,
    TILE0_HLCINSTR_5,
    TILE0_HLCINSTR_6,
    TILE0_HLCINSTR_7,
    TILE0_HLCINSTR_8,
    TILE0_HLCINSTR_9,
    TILE0_HLCINSTR_10,
    TILE0_HLCINSTR_11,
    TILE0_HLCINSTR_12,
    TILE0_HLCINSTR_13,
    TILE0_HLCINSTR_14,
    TILE0_HLCINSTR_15,
    TILE0_HLCINSTR_16,
    TILE0_HLCINSTR_17,
    TILE0_HLCINSTR_18,
    TILE0_HLCINSTR_19,
    TILE0_HLCINSTR_20,
    TILE0_HLCINSTR_21,
    TILE0_HLCINSTR_22,
    TILE0_HLCINSTR_23,
    TILE0_HLCINSTR_24,
    TILE0_HLCINSTR_25,
    TILE0_HLCINSTR_26,
    TILE0_HLCINSTR_27,
    TILE0_HLCINSTR_28,
    TILE0_HLCINSTR_29,
    TILE0_HLCINSTR_30,
    TILE0_HLCINSTR_31
};



void initTILE0(uint32_t base)
{
    uint16_t i;
    //
    //  Pipeline Mode
    //
    CLB_disablePipelineMode(base);
    //
    //  Output LUT
    //
    CLB_configOutputLUT(base, CLB_OUT0, TILE0_CFG_OUTLUT_0);

    CLB_configOutputLUT(base, CLB_OUT1, TILE0_CFG_OUTLUT_1);

    CLB_configOutputLUT(base, CLB_OUT2, TILE0_CFG_OUTLUT_2);

    CLB_configOutputLUT(base, CLB_OUT3, TILE0_CFG_OUTLUT_3);

    CLB_configOutputLUT(base, CLB_OUT4, TILE0_CFG_OUTLUT_4);

    CLB_configOutputLUT(base, CLB_OUT5, TILE0_CFG_OUTLUT_5);
    
    CLB_configOutputLUT(base, CLB_OUT6, TILE0_CFG_OUTLUT_6);

    CLB_configOutputLUT(base, CLB_OUT7, TILE0_CFG_OUTLUT_7);

    //
    //  AOC
    //
    CLB_configAOC(base, CLB_AOC0, TILE0_OUTPUT_COND_CTR_0);
    CLB_configAOC(base, CLB_AOC1, TILE0_OUTPUT_COND_CTR_1);
    CLB_configAOC(base, CLB_AOC2, TILE0_OUTPUT_COND_CTR_2);
    CLB_configAOC(base, CLB_AOC3, TILE0_OUTPUT_COND_CTR_3);
    CLB_configAOC(base, CLB_AOC4, TILE0_OUTPUT_COND_CTR_4);
    CLB_configAOC(base, CLB_AOC5, TILE0_OUTPUT_COND_CTR_5);
    CLB_configAOC(base, CLB_AOC6, TILE0_OUTPUT_COND_CTR_6);
    CLB_configAOC(base, CLB_AOC7, TILE0_OUTPUT_COND_CTR_7);

    //
    // LUT 0 - 2 are configured as macros in clb_config.h; these macros are used in
    // CLB_selectLUT4Inputs and CLB_configLUT4Function
    //
    //
    //  Equation for Look-Up Table Block 0 for TILE0: i0 & ~i1
    //  User Description for Look-Up Table Block 0 for TILE0
    /*
        SCLK rising edge when CS is low
    */

    //
    //  LUT Configuration
    //
    CLB_selectLUT4Inputs(base, TILE0_CFG_LUT4_IN0, TILE0_CFG_LUT4_IN1, TILE0_CFG_LUT4_IN2, TILE0_CFG_LUT4_IN3);
    CLB_configLUT4Function(base, TILE0_CFG_LUT4_FN10, TILE0_CFG_LUT4_FN2);

    //
    // FSM 0 - 2 are configured in <file>
    //

    //
    //  FSM
    //
    CLB_selectFSMInputs(base, TILE0_CFG_FSM_EXT_IN0, TILE0_CFG_FSM_EXT_IN1, TILE0_CFG_FSM_EXTRA_IN0, TILE0_CFG_FSM_EXTRA_IN1);
    CLB_configFSMNextState(base, TILE0_CFG_FSM_NEXT_STATE_0, TILE0_CFG_FSM_NEXT_STATE_1, TILE0_CFG_FSM_NEXT_STATE_2);
    CLB_configFSMLUTFunction(base, TILE0_CFG_FSM_LUT_FN10, TILE0_CFG_FSM_LUT_FN2);

    //
    // Counter 0 - 2 are configured in <file>
    //

    //
    //  Counters
    //
    CLB_selectCounterInputs(base, TILE0_CFG_COUNTER_RESET, TILE0_CFG_COUNTER_EVENT, TILE0_CFG_COUNTER_MODE_0, TILE0_CFG_COUNTER_MODE_1);
    CLB_configMiscCtrlModes(base, TILE0_CFG_MISC_CONTROL);
    CLB_configCounterLoadMatch(base, CLB_CTR0, TILE0_COUNTER_0_LOAD_VAL, TILE0_COUNTER_0_MATCH1_VAL, TILE0_COUNTER_0_MATCH2_VAL);
    CLB_configCounterLoadMatch(base, CLB_CTR1, TILE0_COUNTER_1_LOAD_VAL, TILE0_COUNTER_1_MATCH1_VAL, TILE0_COUNTER_1_MATCH2_VAL);
    CLB_configCounterLoadMatch(base, CLB_CTR2, TILE0_COUNTER_2_LOAD_VAL, TILE0_COUNTER_2_MATCH1_VAL, TILE0_COUNTER_2_MATCH2_VAL);
    CLB_configCounterTapSelects(base, TILE0_CFG_TAP_SEL);

    //
    // HLC is configured in <file>
    //

    //
    // HLC
    //
    CLB_configHLCEventSelect(base, TILE0_HLC_EVENT_SEL);
    CLB_setHLCRegisters(base, TILE0_HLC_R0_INIT, TILE0_HLC_R1_INIT, TILE0_HLC_R2_INIT, TILE0_HLC_R3_INIT);

    for(i = 0; i <= CLB_NUM_HLC_INSTR; i++)
    {
        CLB_programHLCInstruction(base, i, TILE0HLCInstr[i]);
    }
}


