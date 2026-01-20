/*
 * Copyright (c) 2021, Texas Instruments Incorporated
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

#include "main.h"
#include "uart.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include "ti_msp_dl_config.h"
#include "clock.h"
#include <stdbool.h>
#include <core_cm0plus.h> // Include CMSIS core header for __WFI()
#include <stdint.h>

#define UART_CMD_IBRD_4_MHZ_115200_BAUD                                      (2)
#define UART_CMD_FBRD_4_MHZ_115200_BAUD                                     (11)
#define UART_CMD_IBRD_32_MHZ_115200_BAUD                                    (17)
#define UART_CMD_FBRD_32_MHZ_115200_BAUD                                    (23)
#define UART_CMD_IBRD_40_MHZ_115200_BAUD                                    (21)
#define UART_CMD_FBRD_40_MHZ_115200_BAUD                                    (45)


//****************************************************************************
//
// Internal variables and functions
//
//****************************************************************************

// define the gloabl variable
uint32_t g_system_clk_frequency_mhz = 32; // default to 32MHz
// Timer status slg (standby)
static volatile bool standby_timer_expired = false;

/**
 * @brief Initializes system to run at 80 MHz after startup
 *
 * @return STATUS_OK always
 */
enum status_enum cpu_clock_init_80m(void){
    static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
        .inputFreq              = DL_SYSCTL_SYSPLL_INPUT_FREQ_16_32_MHZ,
        .rDivClk2x              = 1,
        .rDivClk1               = 0,
        .rDivClk0               = 0,
        .enableCLK2x            = DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
        .enableCLK1             = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
        .enableCLK0             = DL_SYSCTL_SYSPLL_CLK0_ENABLE,
        .sysPLLMCLK             = DL_SYSCTL_SYSPLL_MCLK_CLK0,
        .sysPLLRef              = DL_SYSCTL_SYSPLL_REF_SYSOSC,
        .qDiv                   = 9,
        .pDiv                   = DL_SYSCTL_SYSPLL_PDIV_2
    };

    //Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfig);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
    DL_UART_Main_setBaudRateDivisor(UART_CMD_INST, UART_CMD_IBRD_40_MHZ_115200_BAUD, UART_CMD_FBRD_40_MHZ_115200_BAUD);
    DL_SYSTICK_config(80000); //80000
    uart_printf("Running at 80 MHz\r\n");
    g_system_clk_frequency_mhz = 80;
    return STATUS_OK;
}

/**
 * @brief Initializes system to run at 32 MHz after startup
 *
 * @return STATUS_OK always
 */
enum status_enum cpu_clock_init_32m(void){
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSTICK_config(320); // 32000 adjust for speed required
    DL_UART_Main_setBaudRateDivisor(UART_IO_INST, UART_CMD_IBRD_32_MHZ_115200_BAUD, UART_CMD_FBRD_32_MHZ_115200_BAUD);
    uart_printf("Running at 32 MHz\r\n");
    g_system_clk_frequency_mhz = 32;
    return STATUS_OK;
}

/**
 * @brief Initializes system to run at 4 MHz after startup
 *
 * @return STATUS_OK always
 */
enum status_enum cpu_clock_init_4m(void){
    //Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_4M);
    /* Set default configuration */
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
    DL_SYSCTL_enableMFCLK();
    DL_SYSTICK_config(32000); 
    DL_UART_Main_setBaudRateDivisor(UART_IO_INST, UART_CMD_IBRD_4_MHZ_115200_BAUD, UART_CMD_FBRD_4_MHZ_115200_BAUD);
    uart_printf("Running at 4 MHz\r\n");
    g_system_clk_frequency_mhz = 4;
    return STATUS_OK;
}

//****************************************************************************
//
// Function Definitions
//
//****************************************************************************
void delay_high_res_init(void) {
    DL_Timer_stopCounter(DELAY_TIMER_HIGH_RES_INST);
    
    DL_Timer_ClockConfig clockConfig = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
        .prescale = 0
    };
    DL_Timer_setClockConfig(DELAY_TIMER_HIGH_RES_INST, &clockConfig);
    
    DL_Timer_setCounterMode(DELAY_TIMER_HIGH_RES_INST, 
        DL_TIMER_COUNT_MODE_DOWN);
}
/*
 * @brief High-resolution microsecond delay function for active mode
 * 
 * Uses the high-resolution timer (DELAY_TIMER_HIGH_RES_INST) running from BUSCLK
 * for precise timing needed in ADC operations during active mode.
 * 
 * @param microseconds Delay time in microseconds
 */
void delay_high_res_us(uint32_t microseconds) {
    if (microseconds == 0) {
        return;
    }
    // Calculate ticks with 1% compensation for overhead
    uint32_t ticks = (microseconds * 32U) + (microseconds * 32U / 100);
    //uint32_t ticks = (microseconds * g_system_clk_frequency_mhz) + (microseconds * g_system_clk_frequency_mhz / 100);
  
    // Ensure timer is stopped
    DL_Timer_stopCounter(DELAY_TIMER_HIGH_RES_INST);
    
    // Clear any pending interrupts/flags
    DL_Timer_clearInterruptStatus(DELAY_TIMER_HIGH_RES_INST, 
        DL_TIMER_INTERRUPT_ZERO_EVENT);
    
    // Set load value
    DL_Timer_setLoadValue(DELAY_TIMER_HIGH_RES_INST, ticks);
    
    // Start the timer
    DL_Timer_startCounter(DELAY_TIMER_HIGH_RES_INST);
    
    // Busy-wait for zero event flag (CPU stays active)
    while ((DL_Timer_getRawInterruptStatus(DELAY_TIMER_HIGH_RES_INST, 
            DL_TIMER_INTERRUPT_ZERO_EVENT) & DL_TIMER_INTERRUPT_ZERO_EVENT) == 0) { 
        __NOP();
    }
    
    // Stop the timer
    DL_Timer_stopCounter(DELAY_TIMER_HIGH_RES_INST);
    
    // Clear the flag
    DL_Timer_clearInterruptStatus(DELAY_TIMER_HIGH_RES_INST, 
        DL_TIMER_INTERRUPT_ZERO_EVENT);
    
    // Disable any interrupts 
    DL_Timer_disableInterrupt(DELAY_TIMER_HIGH_RES_INST, 
        DL_TIMER_INTERRUPT_ZERO_EVENT);
    
    // Disable timer power to save current
    //DL_Timer_disablePower(DELAY_TIMER_HIGH_RES_INST);  // Uncomment if function exists
}

/**
 * @brief Standby microsecond delay function using WFI
 * 
 * Uses the standby timer (DELAY_TIMER_STANDBY_INST) running from LFCLK
 * and the WFI instruction to enter low-power mode while waiting for
 * the timer interrupt. Optimized for power savings.
 * 
 * @param microseconds Delay time in microseconds
 */
void delay_us_standby(uint32_t microseconds) {
    static uint32_t last_microseconds = 0;
    static uint32_t cached_ticks = 0;
    
    // Calculate ticks only if microseconds changed 
    if (microseconds != last_microseconds) {
        uint64_t ticks = ((uint64_t)microseconds * 4096ULL + 999999ULL) / 1000000ULL;
        
        if (ticks == 0) {
            ticks = 1;
        }
        if (ticks > 65535) {
            ticks = 65535;
        }
        
        cached_ticks = (uint32_t)ticks;
        last_microseconds = microseconds;
    }
    
    // Reset flag
    standby_timer_expired = false;
    
    // Stop timer
    DL_TimerG_stopCounter(DELAY_TIMER_STANDBY_INST);
    
    // Clear timer interrupts
    DL_TimerG_clearInterruptStatus(DELAY_TIMER_STANDBY_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_clearInterruptStatus(DELAY_TIMER_STANDBY_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
    NVIC_ClearPendingIRQ(TIMG8_INT_IRQn);
    
    // Set load value
    DL_TimerG_setLoadValue(DELAY_TIMER_STANDBY_INST, cached_ticks - 1);

    // **SAVE INTERRUPT STATE**
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    
    // Save all NVIC interrupt enables
    uint32_t nvic_iser[8];
    for (int i = 0; i < 8; i++) {
        nvic_iser[i] = NVIC->ISER[i];
        NVIC->ICER[i] = 0xFFFFFFFF;  // Disable all
    }
    
    // Save and disable SysTick
    uint32_t systick_ctrl = SysTick->CTRL;
    SysTick->CTRL = 0;
    
    // **ENABLE ONLY TIMG8 AND UART (so you can receive "stop" command)
    NVIC_EnableIRQ(TIMG8_INT_IRQn);
    NVIC_EnableIRQ(UART0_INT_IRQn);  
    NVIC_EnableIRQ(UART2_INT_IRQn);
    
    DL_TimerG_enableInterrupt(DELAY_TIMER_STANDBY_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    
    if ((primask & 0x1) == 0) {
        __enable_irq();
    }
    
    __DSB();
    __ISB();
    
    // Start timer and sleep
    DL_TimerG_startCounter(DELAY_TIMER_STANDBY_INST);
    
    while (!standby_timer_expired) {
        __WFI();
    }
    
    // RESTORE EVERYTHING
    __disable_irq();
    
    // Disable TIMG8
    DL_TimerG_disableInterrupt(DELAY_TIMER_STANDBY_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_DisableIRQ(TIMG8_INT_IRQn);
    DL_TimerG_stopCounter(DELAY_TIMER_STANDBY_INST);
    NVIC_ClearPendingIRQ(TIMG8_INT_IRQn);
    
    // Restore SysTick
    SysTick->CTRL = systick_ctrl;
    
    // Restore NVIC interrupts
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;  // Clear all first
        NVIC->ISER[i] = nvic_iser[i];  // Then restore saved state
    }
    
    __DSB();
    __ISB();
    
    if ((primask & 0x1) == 0) {
        __enable_irq();
    }
}

void TIMG8_IRQHandler(void)
{
    if(DL_TimerG_getPendingInterrupt(DELAY_TIMER_STANDBY_INST) & DL_TIMERG_INTERRUPT_ZERO_EVENT)
    {
        DL_TimerG_clearInterruptStatus(DELAY_TIMER_STANDBY_INST, DL_TIMERG_INTERRUPT_ZERO_EVENT);
        
        standby_timer_expired = true;
    }
}
