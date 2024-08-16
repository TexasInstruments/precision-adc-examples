/*
 * settings.c
 *
 *  Created on: Oct 22, 2018
 *      Author: a0282860
 */


#include "settings.h"

// Internal function prototypes
static void PAMB_InitClock(void);
static void PAMB_InitGPIO(void);
static void PAMB_InitSysTick(void);
static void PAMB_InitUSB(void);



//****************************************************************************
//
// Initializes minimum required PAMB peripherals.
// BoosetPack peripherals are initialized in hal.c
//
//****************************************************************************
void INIT_PAMB(void)
{
    PAMB_InitClock();
    PAMB_InitSysTick();
    PAMB_InitGPIO();
    PAMB_InitUSB();
}



//****************************************************************************
//
// Clock
//
//****************************************************************************

// Variable to remember our system clock frequency
static uint32_t g_ui32SysClock = 0;

// Function to configure clock
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

// Getter to return configured clock frequency
uint32_t getSysClockHz(void)
{
    return g_ui32SysClock;
}



//****************************************************************************
//
// Systick
//
//****************************************************************************

// The system tick rate
#define SYSTICKS_PER_SECOND     (100)
#define SYSTICK_PERIOD_MS       (1000 / SYSTICKS_PER_SECOND)


static void PAMB_InitSysTick(void)
{
    //
    // Enable the system tick.
    //
    SysTickPeriodSet(g_ui32SysClock / SYSTICKS_PER_SECOND);
    SysTickIntEnable();
    SysTickEnable();

    //TODO: What is the difference between these functions?
    //    ROM_SysTickPeriodSet(g_ui32SysClock / SYSTICKS_PER_SECOND);
    //    ROM_SysTickIntEnable();
    //    ROM_SysTickEnable();

}

// The global system tick counter.
volatile uint32_t g_ui32SysTickCount = 0;

// Interrupt handler for the system tick counter.
void SysTick_Handler(void)
{
    //
    // Update our system tick counter.
    //
    g_ui32SysTickCount++;
}



//****************************************************************************
//
// GPIOs
//
//****************************************************************************

static void PAMB_InitGPIO(void)
{
    //
    // Enable all the required GPIO peripherals.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);    // BP Header
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);    // USB
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);    // JTAG
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);    // BP Header
    //ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);    // LEDs
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);    // BP Header + Boost EN
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);    // BP Header
    //ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);    // BP Header
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);    // USB + ULPI
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);    // BP Header
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);    // BSL+ LEDs + BP Header
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);    // UPLI + BP Header
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);    // BP Header
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOQ))
    {
    }

    //
    // PAM Boost enable pin for powering 5V supply to BoosterPack.
    // Defaults to off, until USB enumeration completes.
    //
    ROM_GPIOPinTypeGPIOOutput(BOOST_EN_PORT, BOOST_EN_PIN);
    ROM_GPIOPinWrite(BOOST_EN_PORT, BOOST_EN_PIN, 0);

    //
    // LEDs - default to OFF
    //
    GPIOPinTypeGPIOOutput(LED1_PORT, LED1_PIN);
    GPIOPinTypeGPIOOutput(LED2_PORT, LED2_PIN);
    GPIOPinTypeGPIOOutput(LED3_PORT, LED3_PIN);
    GPIOPinTypeGPIOOutput(LED4_PORT, LED4_PIN);
    ROM_GPIOPinWrite(LED1_PORT, LED1_PIN, 0);
    ROM_GPIOPinWrite(LED2_PORT, LED2_PIN, 0);
    ROM_GPIOPinWrite(LED3_PORT, LED3_PIN, 0);
    ROM_GPIOPinWrite(LED4_PORT, LED4_PIN, 0);
    MAP_GPIOPadConfigSet(LED1_PORT, LED1_PIN, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    MAP_GPIOPadConfigSet(LED2_PORT, LED2_PIN, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    MAP_GPIOPadConfigSet(LED3_PORT, LED3_PIN, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    MAP_GPIOPadConfigSet(LED4_PORT, LED4_PIN, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);


#ifdef  BSL_INTERRUPT
    //
    // BSL button
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION))
    {
    }
    ROM_GPIOPinTypeGPIOInput(BSL_PORT, BSL_PIN);
    MAP_GPIOPadConfigSet(BSL_PORT, BSL_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    MAP_GPIOIntTypeSet(BSL_PORT, BSL_PIN, GPIO_FALLING_EDGE);
    MAP_GPIOIntEnable(BSL_PORT, BSL_PIN);
    MAP_IntEnable(BSL_INT);
#endif

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

    // TODO: Utilize SimpleLink functions...
    //! // Enable ULPI PHY with full VBUS control.
    //! USBULPIConfig(USB0_BASE, USB_ULPI_EXTVBUS | USB_ULPI_EXTVBUS_IND);

    //! // Enable ULPI function.
    //! USBULPIEnable(USB0_BASE);

    //! // Disable ULPI function.
    //! USBULPIDisable(USB0_BASE);


    if (enableULPI)
    {
        //
        // Configures the ULPI interface for the USB MAC core
        //
        MAP_GPIOPinConfigure(GPIO_PL0_USB0D0);
        MAP_GPIOPinConfigure(GPIO_PL1_USB0D1);
        MAP_GPIOPinConfigure(GPIO_PL2_USB0D2);
        MAP_GPIOPinConfigure(GPIO_PL3_USB0D3);
        MAP_GPIOPinConfigure(GPIO_PL4_USB0D4);
        MAP_GPIOPinConfigure(GPIO_PL5_USB0D5);
        MAP_GPIOPinConfigure(GPIO_PP5_USB0D6);
        MAP_GPIOPinConfigure(GPIO_PP4_USB0D7);
        MAP_GPIOPinConfigure(GPIO_PB3_USB0CLK);
        MAP_GPIOPinConfigure(GPIO_PB2_USB0STP);
        MAP_GPIOPinConfigure(GPIO_PP3_USB0DIR);
        MAP_GPIOPinConfigure(GPIO_PP2_USB0NXT);

        MAP_GPIOPinTypeUSBDigital(GPIO_PORTL_BASE, GPIO_PIN_0);
        MAP_GPIOPinTypeUSBDigital(GPIO_PORTL_BASE, GPIO_PIN_1);
        MAP_GPIOPinTypeUSBDigital(GPIO_PORTL_BASE, GPIO_PIN_2);
        MAP_GPIOPinTypeUSBDigital(GPIO_PORTL_BASE, GPIO_PIN_3);
        MAP_GPIOPinTypeUSBDigital(GPIO_PORTL_BASE, GPIO_PIN_4);
        MAP_GPIOPinTypeUSBDigital(GPIO_PORTL_BASE, GPIO_PIN_5);
        MAP_GPIOPinTypeUSBDigital(GPIO_PORTP_BASE, GPIO_PIN_5);
        MAP_GPIOPinTypeUSBDigital(GPIO_PORTP_BASE, GPIO_PIN_4);
        MAP_GPIOPinTypeUSBDigital(GPIO_PORTB_BASE, GPIO_PIN_3);
        MAP_GPIOPinTypeUSBDigital(GPIO_PORTB_BASE, GPIO_PIN_2);
        MAP_GPIOPinTypeUSBDigital(GPIO_PORTP_BASE, GPIO_PIN_3);
        MAP_GPIOPinTypeUSBDigital(GPIO_PORTP_BASE, GPIO_PIN_2);
    }
    else
    {
        // TODO: (OPTIONAL) Undo above GPIO pin type assignments
    }

    // Configure ULPI related GPIOs
    ROM_GPIOPinTypeGPIOOutput(MUXSEL_PORT, MUXSEL_PIN);
    ROM_GPIOPinTypeGPIOOutput(ULPI_nRESET_PORT, ULPI_nRESET_PIN);

    // Switch USB MUX: LOW=ULPI, HIGH=USB peripheral
    ROM_GPIOPinWrite(MUXSEL_PORT, MUXSEL_PIN, (enableULPI ? 0 : MUXSEL_PIN));

    // Reset the ULPI
    ROM_GPIOPinWrite(ULPI_nRESET_PORT, ULPI_nRESET_PIN, 0);
    if (enableULPI)
    {
        SysCtlDelay(1200);  // hopefully the following GPIO configurations take as much as 1 microsecond or more to execute before deassertion of reset.
        ROM_GPIOPinWrite(ULPI_nRESET_PORT, ULPI_nRESET_PIN, ULPI_nRESET_PIN);   // deassert reset
    }
    else
    {
        // Enable the USB peripheral
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);
    }

    // Tell the USB library to use ULPI interface?
    uint32_t ui32ULPI = (enableULPI ? USBLIB_FEATURE_ULPI_HS : USBLIB_FEATURE_ULPI_NONE);
    USBDCDFeatureSet(0, USBLIB_FEATURE_USBULPI, &ui32ULPI);
}
#endif



static void PAMB_InitUSB(void)
{
    //
    // Initialize the USB stack for device mode.
    // NOTE: The caller is responsible for cleaning up the interface and removing itself from the bus prior to making this call and reconfiguring afterwards
    // Reference: http://processors.wiki.ti.com/index.php/Tiva_C_USB_Mode_Force_Device
    //
    USBStackModeSet(0, eUSBModeForceDevice, 0);

    //
    // ULPI configuration
    //
    #ifdef USE_ULPI
        ConfigureULPI(true);    // NOTE: ULPI is disabled by default in hardware, so we do not need to call this function when the ULPI is not being used.
    #else

        // Configure USB pins
        ROM_GPIOPinTypeUSBAnalog(GPIO_PORTL_BASE, GPIO_PIN_6 | GPIO_PIN_7);

        // Enable the USB peripheral (only required when not using the ULPI)
        ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);

    #endif

    //
    // Tell the USB library the CPU clock and the PLL frequency.
    //
    uint32_t ui32PLLRate;
    SysCtlVCOGet(SYSCTL_XTAL_25MHZ, &ui32PLLRate);
    USBDCDFeatureSet(0, USBLIB_FEATURE_CPUCLK, &g_ui32SysClock);
    USBDCDFeatureSet(0, USBLIB_FEATURE_USBPLL, &ui32PLLRate);

    // TODO: Update the following function calls to allow selection between composite OR CDC device initialization...
#ifdef USE_COMPOSITE

    //
    // Initialize the transmit and receive buffers for BULK device.
    //
    USBBufferInit(BULK_TX_BUFFER);
    USBBufferInit(BULK_RX_BUFFER);

    //
    // Initialize the first serial port instances that is part of this
    // composite device.
    //
    g_sCompDevice.psDevices[0].pvInstance =
            USBDBulkCompositeInit(0, &g_psBULKDevice, &g_psCompEntries[0]);

    //
    // Initialize the transmit and receive buffers for CDC (serial) device.
    //
    USBBufferInit(CDC_TX_BUFFER);
    USBBufferInit(CDC_RX_BUFFER);

    //
    // Initialize the second serial port instances that is part of this
    // composite device.
    //
    g_sCompDevice.psDevices[1].pvInstance =
            USBDCDCCompositeInit(0, &g_psCDCDevice, &g_psCompEntries[1]);

    //
    // Pass the device information to the USB library and place the device
    // on the bus.
    //
    USBDCompositeInit(0, &g_sCompDevice, DESCRIPTOR_DATA_SIZE, g_pui8DescriptorData);

#else   // TODO: Clean up

    //
    //    //
    //    // Initialize the transmit and receive buffers.
    //    //
    //    USBBufferInit(&g_psTxBuffer2);
    //    USBBufferInit(&g_psRxBuffer2);
    //
    //    //
    //    // Initialize the serial port
    //    //
    //    USBDCDCCompositeInit(0, &g_psCDCDevice, 0);

    //#ifdef USB_BULK_CLASS
    //    //
    //    // Pass our device information to the USB library and place the device
    //    // on the bus.
    //    //
    //    USBDBulkInit(0, &g_sBulkDevice);
    //#endif
    //
    //    //
    //    // Initial serial device and pass the device information to the USB library and place the device
    //    // on the bus.
    //    //
    //    USBDCDCInit(0, &g_psCDCDevice);
#endif

#ifdef USE_MS_OS_20
    g_sCompDevice.sPrivateData.sDeviceDescriptor.bcdUSB = 0x0201;

    // TODO: Allow for MS OS 2.0 descriptor for non-composite device

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
