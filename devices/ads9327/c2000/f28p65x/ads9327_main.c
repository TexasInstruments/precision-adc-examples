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

#include <ads9327.h>

void main(void)
{

    ADS9327_HAL_DeviceInit();

    // Board initialization
    ADS9327_HAL_Board_init();


    ADS9327_HAL_setupGpioExtAdcStart();

    // Clock for External ADC- ePWM setup
    ADS9327_HAL_disableEpwmCounting();
    ADS9327_HAL_setupEpwmExtAdcStart();
    ADS9327_HAL_enableEpwmCounting();

    ADS9327_initalization(); // ADC initialization sequence, needed at power-up
    
    // USER ADC CONFIGURATION CODE STARTS HERE

    ADS9327_unlockRegisterMap(); // unlock register map

    ADS9327_regBankSel(1); // bank 1

    ADS9327_HAL_writeRegister(REGISTER_10H_ADDRESS,0xABCD); // write ABCD to test pattern 1 reg

    ADS9327_HAL_writeRegister(REGISTER_01H_ADDRESS,0x1001); // read back reg 0x10

    ADS9327_HAL_writeRegister(REGISTER_11H_ADDRESS,0xEF12); // write EF12 to test pattern 2 reg

    ADS9327_HAL_writeRegister(REGISTER_01H_ADDRESS,0x1101); // read back reg 0x11

    ADS9327_HAL_writeRegister(REGISTER_0FH_ADDRESS,REGISTER_0FH_TEST_PATT_EN_TRUE); // enable test pattern output 

    ADS9327_HAL_writeRegister(REGISTER_01H_ADDRESS,REGISTER_01H_DATA_SEL_ADCOUTPUT); // enable output

    #if DATA_LANES_SELECT == 1

    ADS9327_HAL_writeRegister(REGISTER_09H_ADDRESS,REGISTER_09H_NUM_DATA_LANES_1_LANES);
    
    #endif

    ADS9327_lockRegisterMap(); // lock register map


    // configure SPI for different modes
    ADS9327_HAL_configSpiExtAdcDataRead();

    // setup interrupt handlers
    ADS9327_HAL_setupInterrupts();


    EALLOW;
    // Enable Global interrupt INTM
    EINT;


    // Enable Global real-time interrupt DBGM
    ERTM;
    EDIS;


    // Loop forever. Suspend or place breakpoints to observe the buffers.
    while(1) {}
}



// ISR1() interrupt function
__interrupt void ISR1(void)
{
    // ISR is triggered by the EPWM
    #if DATA_LANES_SELECT == 1
    
    ADS9327_HAL_1LaneModeISR1();
    
    #elif DATA_LANES_SELECT == 4
    
    ADS9327_HAL_4LaneModeISR1();
    
    #endif

    ADS9327_HAL_clearAckISR1();

}

// ISR2() interrupt function
__interrupt void ISR2(void)
{
    // ISR is triggered by the SPIx_RX

    #if DATA_LANES_SELECT == 1

    ADS9327_HAL_1LaneModeISR2();
    
    #endif
    
    ADS9327_HAL_clearAckISR2();

}