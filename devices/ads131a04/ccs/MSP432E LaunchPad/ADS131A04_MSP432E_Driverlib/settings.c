/*
 * settings.c
 *
 *  Created on: Oct 22, 2018
 *      Author: a0282860
 */


#include "settings.h"

// Internal function prototypes
static void LaunchPad_InitClock(void);
static void LaunchPad_InitGPIO(void);
static void LaunchPad_InitSysTick(void);



//****************************************************************************
//
// Initializes minimum required PAMB peripherals.
// BoosetPack peripherals are initialized in hal.c
//
//****************************************************************************
void INIT_LAUNCHPAD(void)
{
    LaunchPad_InitClock();
    LaunchPad_InitSysTick();
    LaunchPad_InitGPIO();
}



//****************************************************************************
//
// Clock
//
//****************************************************************************

// Variable to remember our system clock frequency
static uint32_t g_ui32SysClock = 0;

// Function to configure clock
static void LaunchPad_InitClock(void)
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


static void LaunchPad_InitSysTick(void)
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

static void LaunchPad_InitGPIO(void)
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

}



//****************************************************************************
//
// Timers
//
//****************************************************************************

// TODO: Create timers for profiling function execution time





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
