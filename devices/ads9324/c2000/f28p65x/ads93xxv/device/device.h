//#############################################################################
//
// FILE:   device.h
//
// TITLE:  Device setup for examples.
//
//#############################################################################
//
//
// $Copyright:
// Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

#ifndef __DEVICE_H__
#define __DEVICE_H__

#ifdef __cplusplus
extern "C"
{
#endif

//
// Included Files
//
#include "driverlib.h"

#if (!defined(CPU1) && !defined(CPU2))
#error "You must define CPU1 or CPU2 in your project properties.  Otherwise, \
the offsets in your header files will be inaccurate."
#endif

#if (defined(CPU1) && defined(CPU2))
#error "You have defined both CPU1 and CPU2 in your project properties.  Only \
a single CPU should be defined."
#endif

//
// Check for invalid compile flags
//
#if defined(__TMS320C28XX_TMU1__)
#error "Invalid TMU Configuration"
#endif

//*****************************************************************************
//
// Defines for pin numbers
//
//*****************************************************************************


//
// ControlCARD
//

//
// LEDs
//
#define DEVICE_GPIO_PIN_LED1        31U             // GPIO number for LED1
#define DEVICE_GPIO_PIN_LED2        34U             // GPIO number for LED2
#define DEVICE_GPIO_CFG_LED1        GPIO_31_GPIO31  // "pinConfig" for LED1
#define DEVICE_GPIO_CFG_LED2        GPIO_34_GPIO34  // "pinConfig" for LED2

//
// CANA
//
#define DEVICE_GPIO_PIN_CANTXA      37U              // GPIO number for CANTXA
#define DEVICE_GPIO_PIN_CANRXA      36U              // GPIO number for CANRXA
#define DEVICE_GPIO_CFG_CANRXA      GPIO_36_CANA_RX  // "pinConfig" for CANA RX
#define DEVICE_GPIO_CFG_CANTXA      GPIO_37_CANA_TX  // "pinConfig" for CANA TX

//
// SCI for USB-to-UART adapter on FTDI chip
//
#define DEVICE_GPIO_PIN_SCIRXDA     28U             // GPIO number for SCI RX
#define DEVICE_GPIO_PIN_SCITXDA     29U             // GPIO number for SCI TX
#define DEVICE_GPIO_CFG_SCIRXDA     GPIO_28_SCIA_RX // "pinConfig" for SCI RX
#define DEVICE_GPIO_CFG_SCITXDA     GPIO_29_SCIA_TX // "pinConfig" for SCI TX

//
// MCANA
//
#define DEVICE_GPIO_PIN_MCANTXA    37U              // GPIO number for MCANTXA
#define DEVICE_GPIO_PIN_MCANRXA    36U              // GPIO number for MCANRXA
#define DEVICE_GPIO_CFG_MCANRXA    GPIO_36_MCANA_RX // "pinConfig" for MCANA RX
#define DEVICE_GPIO_CFG_MCANTXA    GPIO_37_MCANA_TX // "pinConfig" for MCANA TX

//
// MCANB
//
#define DEVICE_GPIO_PIN_MCANTXB    41U              // GPIO number for MCANTXB
#define DEVICE_GPIO_PIN_MCANRXB    40U              // GPIO number for MCANRXB
#define DEVICE_GPIO_CFG_MCANRXB    GPIO_40_MCANB_RX // "pinConfig" for MCANB RX
#define DEVICE_GPIO_CFG_MCANTXB    GPIO_41_MCANB_TX // "pinConfig" for MCANB TX

//
// LINA
//
#define DEVICE_GPIO_PIN_LINTXA      6U              // GPIO number for LINA TX
#define DEVICE_GPIO_PIN_LINRXA      7U              // GPIO number for LINA RX
#define DEVICE_GPIO_CFG_LINTXA      GPIO_6_LINA_TX  // "pinConfig" for LINA TX
#define DEVICE_GPIO_CFG_LINRXA      GPIO_7_LINA_RX  // "pinConfig" for LINA RX

//
// LINB
//
#define DEVICE_GPIO_PIN_LINTXB      24U             // GPIO number for LINA TX
#define DEVICE_GPIO_PIN_LINRXB      25U             // GPIO number for LINA RX
#define DEVICE_GPIO_CFG_LINTXB      GPIO_24_LINB_TX // "pinConfig" for LINB TX
#define DEVICE_GPIO_CFG_LINRXB      GPIO_25_LINB_RX // "pinConfig" for LINB RX

//
// I2C
//
#define DEVICE_GPIO_PIN_SDAA        31U  // GPIO number for I2C SDAA
#define DEVICE_GPIO_PIN_SCLA        32U  // GPIO number for I2C SCLA
#define DEVICE_GPIO_CFG_SDAA        GPIO_31_I2CA_SDA  // "pinConfig" for I2C SDAA
#define DEVICE_GPIO_CFG_SCLA        GPIO_32_I2CA_SCL  // "pinConfig" for I2C SCLA

//I2CB GPIO pins
#define DEVICE_GPIO_PIN_SDAB    2
#define DEVICE_GPIO_PIN_SCLB    3

#define DEVICE_GPIO_CFG_SDAB GPIO_2_I2CB_SDA
#define DEVICE_GPIO_CFG_SCLB GPIO_3_I2CB_SCL

//
// FSI
//
#define DEVICE_GPIO_PIN_FSI_RXCLK   13U  // GPIO number for FSI RXCLK
#define DEVICE_GPIO_PIN_FSI_RX0     12U  // GPIO number for FSI RX0
#define DEVICE_GPIO_PIN_FSI_RX1     11U  // GPIO number for FSI RX1
#define DEVICE_GPIO_CFG_FSI_TXCLK   GPIO_27_FSITXA_CLK  // "pinConfig" for FSI TXCLK
#define DEVICE_GPIO_CFG_FSI_TX0     GPIO_26_FSITXA_D0   // "pinConfig" for FSI TX0
#define DEVICE_GPIO_CFG_FSI_TX1     GPIO_25_FSITXA_D1   // "pinConfig" for FSI TX1
#define DEVICE_GPIO_CFG_FSI_RXCLK   GPIO_13_FSIRXA_CLK  // "pinConfig" for FSI RXCLK
#define DEVICE_GPIO_CFG_FSI_RX0     GPIO_12_FSIRXA_D0   // "pinConfig" for FSI RX0
#define DEVICE_GPIO_CFG_FSI_RX1     GPIO_11_FSIRXA_D1   // "pinConfig" for FSI RX1


//*****************************************************************************
//
// Defines related to clock configuration
//
//*****************************************************************************

//
// To use INTOSC as the clock source, comment the #define USE_PLL_SRC_XTAL,
// and uncomment the #define USE_PLL_SRC_INTOSC
//
#define USE_PLL_SRC_XTAL
//#define USE_PLL_SRC_INTOSC


#if defined(USE_PLL_SRC_XTAL)
//
// 20MHz XTAL on controlCARD is used as the PLL source.
// For use with SysCtl_getClock().
//
#define DEVICE_OSCSRC_FREQ          25000000U

//
// Define to pass to SysCtl_setClock(). Will configure the clock as follows:
// PLLSYSCLK = 25MHz (XTAL_OSC) * 40 (IMULT) / (2 (REFDIV) * 2 (ODIV) * 1(SYSDIV))
//
#define DEVICE_SETCLOCK_CFG          (SYSCTL_OSCSRC_XTAL | SYSCTL_IMULT(32) | \
                                      SYSCTL_REFDIV(2) | SYSCTL_ODIV(2) | \
                                      SYSCTL_SYSDIV(1) | SYSCTL_PLL_ENABLE | \
                                      SYSCTL_DCC_BASE_0)

//
// 200MHz SYSCLK frequency based on the above DEVICE_SETCLOCK_CFG. Update the
// code below if a different clock configuration is used!
//
#define DEVICE_SYSCLK_FREQ          ((DEVICE_OSCSRC_FREQ * 32) / (2 * 2 * 1))

#elif defined(USE_PLL_SRC_INTOSC)
//
// 10MHz INTOSC on the device is used as the PLL source.
// For use with SysCtl_getClock().
//
#define DEVICE_OSCSRC_FREQ          10000000U

//
// Define to pass to SysCtl_setClock(). Will configure the clock as follows:
// PLLSYSCLK = 10MHz (INT_OSC2) * 40 (IMULT) / (1 (REFDIV) * 2 (ODIV) * 1(SYSDIV))
//
#define DEVICE_SETCLOCK_CFG          (SYSCTL_OSCSRC_OSC2 | SYSCTL_IMULT(40) | \
                                      SYSCTL_REFDIV(1) | SYSCTL_ODIV(2) | \
                                      SYSCTL_SYSDIV(1) | SYSCTL_PLL_ENABLE | \
                                      SYSCTL_DCC_BASE_0)

//
// 200MHz SYSCLK frequency based on the above DEVICE_SETCLOCK_CFG. Update the
// code below if a different clock configuration is used!
//
#define DEVICE_SYSCLK_FREQ          ((DEVICE_OSCSRC_FREQ * 40) / (1 * 2 * 1))
#endif

//
// 50MHz LSPCLK frequency based on the above DEVICE_SYSCLK_FREQ and a default
// low speed peripheral clock divider of 4. Update the code below if a
// different LSPCLK divider is used!
//
#define DEVICE_LSPCLK_FREQ          (DEVICE_SYSCLK_FREQ / 4)

//*****************************************************************************
//
// Macro to call SysCtl_delay() to achieve a delay in microseconds. The macro
// will convert the desired delay in microseconds to the count value expected
// by the function. \b x is the number of microseconds to delay.
//
//*****************************************************************************
#define DEVICE_DELAY_US(x) SysCtl_delay(((((long double)(x)) / (1000000.0L /  \
                              (long double)DEVICE_SYSCLK_FREQ)) - 9.0L) / 5.0L)

//
//  Defines for setting FSI clock speeds
//
#define FSI_PRESCALE_50MHZ          2U
#define FSI_PRESCALE_25MHZ          4U
#define FSI_PRESCALE_10MHZ          10U
#define FSI_PRESCALE_5MHZ           20U

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

//#ifndef CMDTOOL
extern uint16_t RamfuncsLoadStart;
extern uint16_t RamfuncsLoadEnd;
extern uint16_t RamfuncsLoadSize;
extern uint16_t RamfuncsRunStart;
extern uint16_t RamfuncsRunEnd;
extern uint16_t RamfuncsRunSize;
//#endif

extern uint16_t isrfuncLoadStart;
extern uint16_t isrfuncLoadEnd;
extern uint16_t isrfuncLoadSize;
extern uint16_t isrfuncRunStart;
extern uint16_t isrfuncRunEnd;
extern uint16_t isrfuncRunSize;

#endif


#define DEVICE_FLASH_WAITSTATES 5

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
//! @brief Function to initialize the device. Primarily initializes system
//!  control to aknown state by disabling the watchdog, setting up the
//!  SYSCLKOUT frequency, and enabling the clocks to the peripherals.
//!
//! \param None.
//! \return None.
//
//*****************************************************************************
extern void Device_init(void);

//*****************************************************************************
//
//! @brief Function to boot CPU2.
//!
//! \param bootmode is the mode in which CPU2 should boot.
//!
//! Available bootmodes :
//!   -  BOOTMODE_BOOT_TO_FLASH_BANK0_SECTOR0
//!   -  BOOTMODE_BOOT_TO_FLASH_BANK0_SECTOR127_END
//!   -  BOOTMODE_BOOT_TO_FLASH_BANK1_SECTOR0
//!   -  BOOTMODE_BOOT_TO_FLASH_BANK2_SECTOR0
//!   -  BOOTMODE_BOOT_TO_FLASH_BANK3_SECTOR0
//!   -  BOOTMODE_BOOT_TO_FLASH_BANK4_SECTOR0
//!   -  BOOTMODE_BOOT_TO_FLASH_BANK4_SECTOR127_END
//!   -  BOOTMODE_BOOT_TO_SECURE_FLASH_BANK0_SECTOR0
//!   -  BOOTMODE_BOOT_TO_SECURE_FLASH_BANK1_SECTOR0
//!   -  BOOTMODE_BOOT_TO_SECURE_FLASH_BANK2_SECTOR0
//!   -  BOOTMODE_BOOT_TO_SECURE_FLASH_BANK3_SECTOR0
//!   -  BOOTMODE_BOOT_TO_SECURE_FLASH_BANK4_SECTOR0
//!   -  BOOTMODE_IPC_MSGRAM_COPY_BOOT_TO_M1RAM
//!   -  BOOTMODE_BOOT_TO_M0RAM
//!   -  BOOTMODE_BOOT_TO_FWU_FLASH
//!   -  BOOTMODE_BOOT_TO_FWU_FLASH_ALT1
//!   -  BOOTMODE_BOOT_TO_FWU_FLASH_ALT2
//!   -  BOOTMODE_BOOT_TO_FWU_FLASH_ALT3
//!
//! Note that while using BOOTMODE_IPC_MSGRAM_COPY_BOOT_TO_M1RAM,
//! BOOTMODE_IPC_MSGRAM_COPY_LENGTH_xxxW must be ORed with the bootmode parameter
//!
//! This function must be called after Device_init function
//! \return None.
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
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

#ifdef __cplusplus
}
#endif

#endif // __DEVICE_H__
