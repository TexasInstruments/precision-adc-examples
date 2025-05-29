/**
 * \brief Device setup for examples.
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

#ifndef __DEVICE_H__
#define __DEVICE_H__

#include "driverlib.h"
#include <cpy_tbl.h>

#if (!defined(CPU1) && !defined(CPU2))
#error "You must define CPU1 or CPU2 in your project properties.  Otherwise, \
the offsets in your header files will be inaccurate."
#endif

#if (defined(CPU1) && defined(CPU2))
#error "You have defined both CPU1 and CPU2 in your project properties.  Only \
a single CPU should be defined."
#endif


#ifdef generic_ram_lnk
void generic_ram_lnk_init();
#endif
#ifdef generic_flash_lnk
extern COPY_TABLE copyTable_ramfunc;
void generic_flash_lnk_init();
#endif

void CMD_init();
//*****************************************************************************
//
// Macro to call SysCtl_delay() to achieve a delay in microseconds. The macro
// will convert the desired delay in microseconds to the count value expected
// by the function. \b x is the number of microseconds to delay.
//
//*****************************************************************************
#define DEVICE_DELAY_US(x) SysCtl_delay(((((long double)(x)) / (1000000.0L /  \
                                (long double)DEVICE_SYSCLK_FREQ)) - 9.0L) / 5.0L)

//*****************************************************************************
//
// Macros related to booting CPU2. These can be used while invoking the
// function Device_bootCPU2()
//
//*****************************************************************************
#ifdef CPU1
#define BOOT_KEY                                0x5A000000UL

#define BOOTMODE_BOOT_TO_FLASH_BANK0_SECTOR0         0x03U
#define BOOTMODE_BOOT_TO_FLASH_BANK0_SECTOR127_END   0x23U
#define BOOTMODE_BOOT_TO_FLASH_BANK1_SECTOR0         0x43U
#define BOOTMODE_BOOT_TO_FLASH_BANK2_SECTOR0         0x63U
#define BOOTMODE_BOOT_TO_FLASH_BANK3_SECTOR0         0x83U
#define BOOTMODE_BOOT_TO_FLASH_BANK4_SECTOR0         0xA3U
#define BOOTMODE_BOOT_TO_FLASH_BANK4_SECTOR127_END   0xC3U
#define BOOTMODE_BOOT_TO_SECURE_FLASH_BANK0_SECTOR0  0x0AU
#define BOOTMODE_BOOT_TO_SECURE_FLASH_BANK1_SECTOR0  0x4AU
#define BOOTMODE_BOOT_TO_SECURE_FLASH_BANK2_SECTOR0  0x6AU
#define BOOTMODE_BOOT_TO_SECURE_FLASH_BANK3_SECTOR0  0x8AU
#define BOOTMODE_BOOT_TO_SECURE_FLASH_BANK4_SECTOR0  0xAAU
#define BOOTMODE_IPC_MSGRAM_COPY_BOOT_TO_M1RAM       0x01U
#define BOOTMODE_BOOT_TO_M0RAM                       0x05U
#define BOOTMODE_BOOT_TO_FWU_FLASH                   0x0BU
#define BOOTMODE_BOOT_TO_FWU_FLASH_ALT1              0x2BU
#define BOOTMODE_BOOT_TO_FWU_FLASH_ALT2              0x4BU
#define BOOTMODE_BOOT_TO_FWU_FLASH_ALT3              0x6BU

#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_100W    0x10000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_200W    0x20000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_300W    0x30000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_400W    0x40000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_500W    0x50000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_600W    0x60000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_700W    0x70000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_800W    0x80000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_900W    0x90000U
#define BOOTMODE_IPC_MSGRAM_COPY_LENGTH_1000W   0xA0000U
#endif

//*****************************************************************************
//
// Defines, Globals, and Header Includes related to Flash Support
//
//*****************************************************************************
#ifdef _FLASH
#include <stddef.h>

#ifndef CMDTOOL
extern uint16_t RamfuncsLoadStart;
extern uint16_t RamfuncsLoadEnd;
extern uint16_t RamfuncsLoadSize;
extern uint16_t RamfuncsRunStart;
extern uint16_t RamfuncsRunEnd;
extern uint16_t RamfuncsRunSize;
#endif

#define DEVICE_FLASH_WAITSTATES 4

#endif

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************
//*****************************************************************************
//
//! \addtogroup device_api
//! @{
//
//*****************************************************************************
//*****************************************************************************
//
//! Function to initialize the device. Primarily initializes system 
//! control to a known state by disabling the watchdog, setting up the
//! SYSCLKOUT frequency, and enabling the clocks to the peripherals.
//!
//! \param None.
//! \return None.
//
//*****************************************************************************
extern void Device_init(void);


//*****************************************************************************
//!
//!
//! @brief Function to turn on all peripherals, enabling reads and writes to the
//! peripherals' registers.
//!
//! Note that to reduce power, unused peripherals should be disabled.
//!
//! @param None
//! @return None
//
//*****************************************************************************
extern void Device_enableAllPeripherals(void);
//*****************************************************************************
//!
//!
//! @brief Function to disable pin locks on GPIOs.
//!
//! @param None
//! @return None
//
//*****************************************************************************
extern void Device_initGPIO(void);



//*****************************************************************************
//
// Function to boot CPU2
// Available bootmodes :
//      - BOOTMODE_BOOT_TO_FLASH_BANK0_SECTOR0
//      - BOOTMODE_BOOT_TO_FLASH_BANK0_SECTOR127_END
//      - BOOTMODE_BOOT_TO_FLASH_BANK1_SECTOR0
//      - BOOTMODE_BOOT_TO_FLASH_BANK2_SECTOR0
//      - BOOTMODE_BOOT_TO_FLASH_BANK3_SECTOR0
//      - BOOTMODE_BOOT_TO_FLASH_BANK4_SECTOR0
//      - BOOTMODE_BOOT_TO_FLASH_BANK4_SECTOR127_END
//      - BOOTMODE_BOOT_TO_SECURE_FLASH_BANK0_SECTOR0
//      - BOOTMODE_BOOT_TO_SECURE_FLASH_BANK1_SECTOR0
//      - BOOTMODE_BOOT_TO_SECURE_FLASH_BANK2_SECTOR0
//      - BOOTMODE_BOOT_TO_SECURE_FLASH_BANK3_SECTOR0
//      - BOOTMODE_BOOT_TO_SECURE_FLASH_BANK4_SECTOR0
//      - BOOTMODE_IPC_MSGRAM_COPY_BOOT_TO_M1RAM
//      - BOOTMODE_BOOT_TO_M0RAM
//      - BOOTMODE_BOOT_TO_FWU_FLASH
//      - BOOTMODE_BOOT_TO_FWU_FLASH_ALT1
//      - BOOTMODE_BOOT_TO_FWU_FLASH_ALT2
//      - BOOTMODE_BOOT_TO_FWU_FLASH_ALT3
//
// Note that while using BOOTMODE_IPC_MSGRAM_COPY_BOOT_TO_M1RAM,
// BOOTMODE_IPC_MSGRAM_COPY_LENGTH_xxxW must be ORed with the bootmode parameter
//
// This function must be called after Device_init function
//
//*****************************************************************************
extern void Device_bootCPU2(uint32_t bootmode);

//*****************************************************************************
//
//!
//! @brief Function to verify the XTAL frequency
//! \param freq is the XTAL frequency in MHz
//! \return The function return true if the the actual XTAL frequency matches with the
//! input value
//
//*****************************************************************************
extern bool Device_verifyXTAL(float freq);


//*****************************************************************************
//!
//! @brief Error handling function to be called when an ASSERT is violated
//!
//! @param *filename File name in which the error has occurred
//! @param line Line number within the file
//! @return None
//
//*****************************************************************************
extern void __error__(const char *filename, uint32_t line);

//*****************************************************************************
//
// Macro definitions used in device.c (SYSPLL / LSPCLK)
//
//*****************************************************************************
//
//	Input Clock to SYSPLL (OSCCLK) = X1 = 25 MHz
//
#define DEVICE_OSCSRC_FREQ          25000000U
//
// Define to pass to SysCtl_setClock(). Will configure the clock as follows:
// SYSPLL ENABLED
// SYSCLK = 200 MHz = 25 MHz (OSCCLK) * 32 (IMULT) / (2 (REFDIV) * 2 (ODIV) * 1 (SYSCLKDIVSEL))
//
#define DEVICE_SETCLOCK_CFG         (SYSCTL_OSCSRC_XTAL_SE  | SYSCTL_IMULT(32) | \
									 SYSCTL_REFDIV(2) | SYSCTL_ODIV(2) | \
									 SYSCTL_SYSDIV(1) | SYSCTL_PLL_ENABLE | \
									 SYSCTL_DCC_BASE_0)
									 
									 
#define DEVICE_SYSCLK_FREQ          ((DEVICE_OSCSRC_FREQ * 32) / (2 * 2 * 1))

//
// Define to pass to SysCtl_setLowSpeedClock().
// Low Speed Clock (LSPCLK) = 200 MHz / 1 = 200 MHz
//
#define DEVICE_LSPCLK_CFG  			SYSCTL_LSPCLK_PRESCALE_1

#define DEVICE_LSPCLK_FREQ          (DEVICE_SYSCLK_FREQ / 1)

//
//*****************************************************************************
//
// Macro definitions used in device.c (AUXPLL)
//
//*****************************************************************************
//
//	Input Clock to AUXPLL (AUXOSCCLK) = X1 = 25 MHz
//
#define DEVICE_AUXOSCSRC_FREQ 		25000000U
//
// Define to pass to SysCtl_setAuxClock(). Will configure the clock as follows:
// AUXPLL ENABLED
// AUXPLLCLK = 125 MHz = 25 MHz (AUXOSCCLK) * 40 (IMULT) / (2 (REFDIV) * 4 (ODIV) * 1 (AUXCLKDIVSEL))
#define DEVICE_AUXCLK_FREQ          (DEVICE_AUXOSCSRC_FREQ * 40) / (2 * 4 * 1)
//
#define DEVICE_AUXSETCLOCK_CFG      (SYSCTL_AUXPLL_OSCSRC_XTAL_SE  | SYSCTL_AUXPLL_IMULT(40) | \
									 SYSCTL_REFDIV(2) | SYSCTL_ODIV(4)| \
									 SYSCTL_AUXPLL_DIV_1 | SYSCTL_AUXPLL_ENABLE | \
									 SYSCTL_DCC_BASE_0)
									 
//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
#endif // __DEVICE_H__
