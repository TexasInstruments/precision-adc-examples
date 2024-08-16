/* ******************************************************************************
 *
 *  Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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
 *****************************************************************************/

#include "settings.h"

/*
 * Internal function prototypes
 *
 * static void PAMB_InitClock(void);
 * static void PAMB_InitGPIO(void);
 * static void PAMB_InitSysTick(void);
 * static void PAMB_InitUSB(void);
 */

//****************************************************************************
//
// Clock
//
//****************************************************************************

/*****************************************************************************
 *
 * @brief Variable to remember the system clock frequency.
 *
 ****************************************************************************/
// TODO: the variable is declared static but given a 'g' global representation.
static uint32_t g_ui32SysClock = 0;



// Function to configure clock
/*****************************************************************************
 *
 * @brief Function to configure MSP432E master clock.
 *
 * @return none
 *
 ****************************************************************************/
static void PAMB_InitClock(void)
{
    //
    // Run from the PLL at 120 MHz.
    //
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_PLL |
                                       SYSCTL_CFG_VCO_480), 120000000);
}

/****************************************************************************
 *
 * @brief Getter function to return configured clock frequency.
 *
 * @return \a g_ui32SysClock which is the stored value of the master clock frequency in Hz.
 *
 ***************************************************************************/
uint32_t getSysClockHz(void)
{
    return g_ui32SysClock;
}

//****************************************************************************
//
// Systick
//
//****************************************************************************

/****************************************************************************
 *
 * @brief Macro to set the number of system ticks in one second.
 *
 ***************************************************************************/
#define SYSTICKS_PER_SECOND     (1000)

/****************************************************************************
 *
 * @brief Global system tick counter variable.
 *
 ***************************************************************************/
volatile uint32_t g_ui32SysTickCount = 0;

/****************************************************************************
 *
 * @brief Getter to make the system tick counter accessible to other modules.

 * @return uint32_t system tick counter
 *
 ***************************************************************************/
uint32_t getSysTickCount(void)
{
    return g_ui32SysTickCount;
}

void resetSysTickCount(void)
{
    g_ui32SysTickCount = 0;
}

/****************************************************************************
 *
 * @brief Interrupt handler for the system tick counter.
 *
 * @details The handler increments the global variable \a g_ui32SysTickCount
 * after each system tick timer period.
 *
 * @return none
 *
 ***************************************************************************/
void SysTick_Handler(void)
{
    ++g_ui32SysTickCount;
}

/****************************************************************************
 *
 * @brief Function to set the system tick period for use with the USB interface.
 *
  * @return none
 *
 ***************************************************************************/
static void PAMB_InitSysTick(void)
{
    // Enable the system tick
    SysTickPeriodSet(g_ui32SysClock / SYSTICKS_PER_SECOND);
    SysTickIntRegister(SysTick_Handler);
    SysTickIntEnable();
    SysTickEnable();

    g_ui32SysTickCount = 0;
}


//****************************************************************************
//
// GPIOs
//
//****************************************************************************

/****************************************************************************
 *
 * @brief Initialization routine for PAMB GPIO.
 *
 * @details Configuration includes the enabling of port peripherals available and
 *      the minimum pin configuration required for the base GPIO operation of PAMB. This
 *      includes disabling of the BSL watchdog timer, boost power circuit
 *      enable, and LEDs used on the board.
 *
 * @note This configuration does not configure GPIO used in conjunction with USB,
 *      I2C, SPI, etc, peripherals.  Configuration specific to the BoosterPack EVM
 *      will also be configured elsewhere.
 *
 * @return none
 *
 ***************************************************************************/
static void PAMB_InitGPIO(void)
{
    /* Call driver initialization functions */
     GPIO_init();

    //
    // Enable all the required GPIO peripherals.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);    // BP Header
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);    // USB
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);    // JTAG + BSL Watchdog + BOOST EN
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);    // BP Header
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);  // not used
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);    // LEDs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);    // BP Header + Boost EN
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);    // BP Header
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);  // not used
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);    // BP Header
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);    // USB + ULPI
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);    // BP Header
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);    // BSL+ LEDs + BP Header
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);    // UPLI + BP Header + EEPROM WE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);    // BP Header
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOQ))
    {
    }

}

//****************************************************************************
//
// Timers
//
//****************************************************************************

// TODO: Create timers for profiling function execution time




//****************************************************************************
//
// USB
//
//****************************************************************************

#ifdef USE_ULPI
void ConfigureULPI(const bool enableULPI)
{
    // TODO: Utilize SimpleLink TI Driver functions in SYSCONFIG...

    if (enableULPI)
    {
        // Configures the ULPI interface for the USB MAC core
        GPIOPinConfigure(GPIO_PL0_USB0D0);
        GPIOPinConfigure(GPIO_PL1_USB0D1);
        GPIOPinConfigure(GPIO_PL2_USB0D2);
        GPIOPinConfigure(GPIO_PL3_USB0D3);
        GPIOPinConfigure(GPIO_PL4_USB0D4);
        GPIOPinConfigure(GPIO_PL5_USB0D5);
        GPIOPinConfigure(GPIO_PP5_USB0D6);
        GPIOPinConfigure(GPIO_PP4_USB0D7);
        GPIOPinConfigure(GPIO_PB3_USB0CLK);
        GPIOPinConfigure(GPIO_PB2_USB0STP);
        GPIOPinConfigure(GPIO_PP3_USB0DIR);
        GPIOPinConfigure(GPIO_PP2_USB0NXT);

        GPIOPinTypeUSBDigital(GPIO_PORTL_BASE, GPIO_PIN_0);
        GPIOPinTypeUSBDigital(GPIO_PORTL_BASE, GPIO_PIN_1);
        GPIOPinTypeUSBDigital(GPIO_PORTL_BASE, GPIO_PIN_2);
        GPIOPinTypeUSBDigital(GPIO_PORTL_BASE, GPIO_PIN_3);
        GPIOPinTypeUSBDigital(GPIO_PORTL_BASE, GPIO_PIN_4);
        GPIOPinTypeUSBDigital(GPIO_PORTL_BASE, GPIO_PIN_5);
        GPIOPinTypeUSBDigital(GPIO_PORTP_BASE, GPIO_PIN_5);
        GPIOPinTypeUSBDigital(GPIO_PORTP_BASE, GPIO_PIN_4);
        GPIOPinTypeUSBDigital(GPIO_PORTB_BASE, GPIO_PIN_3);
        GPIOPinTypeUSBDigital(GPIO_PORTB_BASE, GPIO_PIN_2);
        GPIOPinTypeUSBDigital(GPIO_PORTP_BASE, GPIO_PIN_3);
        GPIOPinTypeUSBDigital(GPIO_PORTP_BASE, GPIO_PIN_2);
    }
    else
    {
        // TODO: (OPTIONAL) Undo above GPIO pin type assignments
    }

    // Set relevant GPIO pins
    GPIO_write(MUXSEL_CONST, (enableULPI ? 0 : 1));
    GPIO_write(ULPI_nRESET_CONST, 0);

    // Enable USB peripheral
    if (enableULPI)
    {
        SysCtlDelay(1200);  // hopefully the following GPIO configurations take as much as 1 microsecond or more to execute before deassertion of reset.
        GPIO_write(ULPI_nRESET_CONST, 1);
    }
    else
    {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);
    }

    // Tell the USB library if we are using the ULPI interface
    uint32_t ui32ULPI = (enableULPI ? USBLIB_FEATURE_ULPI_HS : USBLIB_FEATURE_ULPI_NONE);
    USBDCDFeatureSet(0, USBLIB_FEATURE_USBULPI, &ui32ULPI);
}
#endif


/****************************************************************************
 *
 * @brief USB initialization.
 *
 * @details Initialize the USB stack for device mode.  The PAMB connector is OTG
 *      and can be configured as either Host or Device, but firmware works as Device
 *      only which requires the ULPI peripheral to be recognized as Device only,
 *      so the mode is 'forced' instead of auto recognized (LP vs PAMB for PB0).
 *
   @note The caller is responsible for cleaning up the interface and removing itself
   from the bus prior to making this call and reconfiguring afterwards.
   @see http://processors.wiki.ti.com/index.php/Tiva_C_USB_Mode_Force_Device
 *
 * @return none
 *
 ***************************************************************************/
static void PAMB_InitUSB(void)
{
    /* Register a custom interrupt handler using the
     * low-level device APIs */
    IntRegister(INT_USB0, &USB0_IRQDeviceHandler);

    // Set the USB configuration to Device mode
    USBStackModeSet(0, eUSBModeForceDevice, 0);

    #ifdef USE_ULPI
        ConfigureULPI(true);    // NOTE: ULPI is disabled by default in hardware, so we do not need to call this function when the ULPI is not being used.
    #else

        // Configure USB pins
        GPIOPinTypeUSBAnalog(GPIO_PORTL_BASE, GPIO_PIN_6 | GPIO_PIN_7);

        // Enable the USB peripheral (only required when not using the ULPI)
        SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);

    #endif

    // Tell the USB library the CPU clock and the PLL frequency
    uint32_t ui32PLLRate;
    SysCtlVCOGet(SYSCTL_XTAL_25MHZ, &ui32PLLRate);
    USBDCDFeatureSet(0, USBLIB_FEATURE_CPUCLK, &g_ui32SysClock);
    USBDCDFeatureSet(0, USBLIB_FEATURE_USBPLL, &ui32PLLRate);

#ifdef USE_COMPOSITE

    // Initialize the transmit and receive buffers for BULK device.
    USBBufferInit(BULK_TX_BUFFER);
    USBBufferInit(BULK_RX_BUFFER);

    // Initialize the first device
    g_sCompDevice.psDevices[0].pvInstance =
            USBDBulkCompositeInit(0, &g_psBULKDevice, &g_psCompEntries[0]);

    // Initialize the transmit and receive buffers for CDC (serial) device.
    USBBufferInit(CDC_TX_BUFFER);
    USBBufferInit(CDC_RX_BUFFER);

    // Initialize the second device
    g_sCompDevice.psDevices[1].pvInstance =
            USBDCDCCompositeInit(0, &g_psCDCDevice, &g_psCompEntries[1]);

    //
    // Pass the device information to the USB library and place the device
    // on the bus.
    //
    USBDCompositeInit(0, &g_sCompDevice, DESCRIPTOR_DATA_SIZE, g_pui8DescriptorData);

#endif

    // TODO: Do we need an ifdef here?  Won't we always auto enumerate as composite and MSOS 2.0 descriptors?
#ifdef USE_MS_OS_20
//    g_sCompDevice.sPrivateData.sDeviceDescriptor.bcdUSB = 0x0201;

    // TODO: Allow for MS OS 2.0 descriptor for non-composite device

#endif

}


/****************************************************************************
 *
 * @brief PAMB initialization.
 *
 * @details Initializes the standard peripherals used on PAMB.
 *
 * @return none
 *
 ***************************************************************************/
void INIT_PAMB(void)
{
    PAMB_InitClock();
    PAMB_InitSysTick();
    PAMB_InitGPIO();
    PAMB_InitUSB();
#ifdef USE_I2C_PULLUP
    // Enabling Weak Pull-up in I2C lines
    GPION->PUR |= (GPIO_PIN_5 | GPIO_PIN_4);
#endif
}


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error. // TODO: Determine if this function is useful for debugging...
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    USBprintf("Error at line %d of %s\n", ui32Line, pcFilename);
}
#endif
