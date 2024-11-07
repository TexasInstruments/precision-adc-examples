/**
 * \copyright Copyright (C) 2019-2020 Texas Instruments Incorporated - http://www.ti.com/
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

#include "hal.h"


//****************************************************************************
//
// Internal variables
//
//****************************************************************************
static uint32_t conversionTime_us = 3;


//****************************************************************************
//
// Internal function prototypes
//
//****************************************************************************
static void initGPIO(void);
static void initSPI(void);
static void initTIMER(void);


//****************************************************************************
//
// External Functions (prototypes declared in hal.h)
//
//****************************************************************************

//*****************************************************************************
//
//! Initializes MCU peripherals for interfacing with the ADC.
//!
//! \fn void initAdcPeripherals(void)
//!
//! \return None.
//
//*****************************************************************************
void initAdcPeripherals(void)
{
    // IMPORTANT: Make sure device is powered before setting GPIOs pins to HIGH state.

    // Initialize GPIOs pins used by ADS7066
    initGPIO();

    // Initialize SPI peripheral used to interface to the ADS7066
    initSPI();

    // Enable timer peripheral used to control ADS7066 conversion period
    initTIMER();
}


//*****************************************************************************
//
//! Configures the MCU's GPIO pins that interface with the ADC.
//!
//! \fn static void initGPIO(void)
//!
//! \return None.
//
//*****************************************************************************
static void initGPIO(void)
{
    /* --- INSERT YOUR CODE HERE --- */
    // NOTE: Not all hardware implementations may control each of these pins...

    //
    // Enable the clock to the GPIO Port M and wait for it to be ready
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    while (!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)));

    //
    // Configure the GPIO for 'nCS' as output and set high
    //
    MAP_GPIOPinTypeGPIOOutput(nCS_PORT, nCS_PIN);
    MAP_GPIOPinWrite(nCS_PORT, nCS_PIN, nCS_PIN);
}


//*****************************************************************************
//
//! Configures the MCU's SPI peripheral for interfacing with the ADC.
//!
//! \fn static void initSPI(void)
//!
//! \return None.
//
//*****************************************************************************
static void initSPI(void)
{
    /* --- INSERT YOUR CODE HERE ---
     * NOTE: The ADS7066 operates in SPI mode 0 (CPOL = 0, CPHA = 0) by default.
     * The SPI mode of the device can be configured in the CPOL_CPHA[1:0] field,
     * but the first write operation must use mode 0.
     *
     * This example only uses SPI mode 0 and does not showcase daisy-chain mode.
     *
     */

    //
    // Enable the clock to SSI-3 module and configure the SSI Master
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
    while (!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI3)))
    {
    }

    //
    // Enable clocks to GPIO Port Q and configure pins as SSI
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    while (!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOQ)))
    {
    }

    //
    // Configure the GPIO pins PQ0, PQ2 & PQ3 as SPI. For nCS, the default FSS pin PQ1
    // is not used, instead PM7 is used.
    //
    MAP_GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
    MAP_GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
    MAP_GPIOPinConfigure(GPIO_PQ3_SSI3XDAT1);
    MAP_GPIOPinTypeSSI(GPIO_PORTQ_BASE, (GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3));

    MAP_GPIOPadConfigSet(GPIO_PORTQ_BASE, (GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3), GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

    //
    // Configure: SPI MODE 1, 5 MHz SCLK, 8-bits per frame
    //
    MAP_SSIConfigSetExpClk(SSI3_BASE, getSysClockHz(), SSI_FRF_MOTO_MODE_0,   \
                           SSI_MODE_MASTER, 5000000, 8);

    //
    // Enable the SSI-3 module.
    //
    MAP_SSIEnable(SSI_BASE_ADDR);

    //
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
    uint32_t junk;
    while (MAP_SSIDataGetNonBlocking(SSI_BASE_ADDR, &junk));
}


//*****************************************************************************
//
//! Enables the MCU's TIMER to control ADC conversions.
//!
//! \fn static void initTIMER(void)
//!
//! \return None.
//
//*****************************************************************************
static void initTIMER(void)
{
    /* --- INSERT YOUR CODE HERE --- */

    // Enable timer peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
}

//****************************************************************************
//
// GPIO functions
//
//****************************************************************************

//*****************************************************************************
//
//! Controls the state of the /CS GPIO pin.
//!
//! \fn void setCS(const bool state)
//! \param state boolean indicating which state to set the /CS pin (0=low, 1=high)
//!
//! NOTE: The 'HIGH' and 'LOW' macros defined in hal.h can be passed to this
//! function for the 'state' parameter value.
//!
//! \return None.
//
//*****************************************************************************
void setCS(const bool state)
{
    /* --- INSERT YOUR CODE HERE --- */

    //
    // td(CSSC) delay
    //
    if (state) { SysCtlDelay(2); }

    uint8_t value = (uint8_t) (state ? nCS_PIN : 0);
    MAP_GPIOPinWrite(nCS_PORT, nCS_PIN, value);

    //
    // td(SCCS) delay
    //
    if (!state) { SysCtlDelay(2); }
}


//****************************************************************************
//
// SPI functions
//
//****************************************************************************

//*****************************************************************************
//
//! Sends SPI byte on MOSI pin and captures MISO return byte value.
//!
//! \fn uint8_t spiSendReceiveByte(const uint8_t dataTx)
//!
//! \param dataTx data byte to send on MOSI pin.
//!
//! NOTE: This function is called by spiSendReceiveArrays(). If it is called
//! directly, then the /CS pin must also be directly controlled.
//!
//! \return Captured MISO response byte.
//
//*****************************************************************************
uint8_t spiSendReceiveByte(const uint8_t dataTx)
{
    /*  --- INSERT YOUR CODE HERE ---
     *  This function should send and receive single bytes over the SPI.
     *  NOTE: This function does not control the /CS pin to allow for
     *  more programming flexibility.
     */

    //
    // Remove any residual or old data from the receive FIFO
    //
    uint32_t junk;
    while (SSIDataGetNonBlocking(SSI_BASE_ADDR, &junk));

    //
    // SSI TX & RX
    //
    uint8_t dataRx;
    MAP_SSIDataPut(SSI_BASE_ADDR, (uint32_t) dataTx);
    MAP_SSIDataGet(SSI_BASE_ADDR, (uint32_t *) &dataRx);

    return dataRx;
}


//*****************************************************************************
//
//! Sends SPI byte array on MOSI pin and captures MISO data to a byte array.
//!
//! \fn void spiSendReceiveArrays(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t byteLength)
//!
//! \param dataTx[] byte array of SPI data to send on MOSI.
//!
//! \param dataRx[] byte array of SPI data captured on MISO.
//!
//! \param byteLength number of bytes to send & receive.
//!
//! NOTE: Make sure 'dataTx[]' and 'dataRx[]' contain at least as many bytes of data,
//! as indicated by 'byteLength'.
//!
//! \return None.
//
//*****************************************************************************
void spiSendReceiveArray(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t byteLength)
{
    /*  --- INSERT YOUR CODE HERE ---
     *
     *  This function should send and receive multiple bytes over the SPI.
     *
     *  A typical SPI send/receive sequence may look like the following:
     *  1) Make sure SPI receive buffer is empty
     *  2) Set the /CS pin low (if controlled by GPIO)
     *  3) Send command bytes to SPI transmit buffer
     *  4) Wait for SPI receive interrupt
     *  5) Retrieve data from SPI receive buffer
     *  6) Set the /CS pin high (if controlled by GPIO)
     */

    //
    // Require that dataTx and dataRx are not NULL pointers
    //
    assert(dataTx && dataRx);

    //
    // Set the nCS pin LOW
    //
    setCS(LOW);

    //
    // Send all dataTx[] bytes on MOSI, and capture all MISO bytes in dataRx[]
    //
    int i;
    for (i = 0; i < byteLength; i++)
    {
        dataRx[i] = spiSendReceiveByte(dataTx[i]);
    }

    //
    // Set the nCS pin HIGH
    //
    setCS(HIGH);
}


//****************************************************************************
//
// Timing functions
//
//****************************************************************************

//*****************************************************************************
//
//! Provides a timing delay with 'ms' resolution.
//!
//! \fn void delay_ms(const uint32_t delay_time_ms)
//!
//! \param delay_time_ms is the number of milliseconds to delay.
//!
//! \return None.
//
//*****************************************************************************
void delay_ms(const uint32_t delay_time_ms)
{
    /* --- INSERT YOUR CODE HERE --- */

    const uint32_t cycles_per_loop = 3;
    MAP_SysCtlDelay( delay_time_ms * getSysClockHz() / (cycles_per_loop * 1000u) );
}


//*****************************************************************************
//
//! Provides a timing delay with 'us' resolution.
//!
//! \fn void delay_us(const uint32_t delay_time_us)
//!
//! \param delay_time_us is the number of microseconds to delay.
//!
//! \return None.
//
//*****************************************************************************
void delay_us(const uint32_t delay_time_us)
{
    /* --- INSERT YOUR CODE HERE --- */

    const uint32_t cycles_per_loop = 3;
    MAP_SysCtlDelay( delay_time_us * getSysClockHz() / (cycles_per_loop * 1000000u) );
}


//*****************************************************************************
//
//! Initializes timer to interrupt at specified frequency
//!
//! \fn void startTimer(uint32_t timerFreq)
//! \param timerFreq interrupt frequency in units of Hz (or SPS)
//!
//! \return None.
//
//*****************************************************************************
void startTimer(uint32_t timerFreq)
{
    const uint32_t systemClock = 120000000;

    // Disable and clear timer interrupts
    MAP_TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Enable processor interrupts
    MAP_IntMasterEnable();

    // Configure the two 32-bit periodic timers
    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, systemClock / timerFreq);

    // Register the Timer 0A interrupt ISR
    IntRegister(INT_TIMER0A, TIMER0IntHandler);

    // Setup the interrupts for the timer timeouts
    MAP_IntEnable(INT_TIMER0A);
    MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timer
    MAP_TimerEnable(TIMER0_BASE, TIMER_A);
}


//*****************************************************************************
//
//! Disables the timer
//!
//! \fn void stopTimer(void)
//!
//! \return None.
//
//*****************************************************************************
void stopTimer(void)
{
    // Disable and clear timer interrupts
    MAP_TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Disable the timer
    MAP_TimerDisable(TIMER0_BASE, TIMER_A);
}



//*****************************************************************************
//
// The interrupt handler for the timer interrupt.
//
//*****************************************************************************
void TIMER0IntHandler(void)
{
    // Clear the timer interrupt
    MAP_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Array to store ADC conversion results
    uint8_t data[4] = {0};

    // Start conversion
    setCS(HIGH);

    // Wait for conversion to complete
    // IMPORTANT: This delay will need to be modified if averaging is enabled!
    delay_us(3);

    // Read data
    readData(data);
}
