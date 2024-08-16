/**
 * \copyright Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
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

// Flag to indicate if a nDRDY gpio interrupt has occurred
static volatile bool flag_nDRDY_INTERRUPT = false;



//****************************************************************************
//
// Internal function prototypes
//
//****************************************************************************
void InitGPIO(void);
void InitSPI(void);
void GPIO_DRDY_IRQHandler(void);



//****************************************************************************
//
// External Functions (prototypes declared in hal.h)
//
//****************************************************************************

#ifdef EXAMPLE_CODE
#else
bool getDRDYinterruptStatus(void)
{
   return flag_nDRDY_INTERRUPT;
}

void setDRDYinterruptStatus(const bool value)
{
    flag_nDRDY_INTERRUPT = value;
}

void enableDRDYinterrupt(const bool intEnable)
{
    if (intEnable)
    {
        MAP_IntEnable(nDRDY_INT);
    }
    else
    {
        MAP_IntDisable(nDRDY_INT);
    }
}
#endif

//*****************************************************************************
//
//! Initializes MCU peripherals for interfacing with the ADC.
//!
//! \fn void InitADC(void)
//!
//! \return None.
//
//*****************************************************************************
void InitADC(void)
{
    // IMPORTANT: Make sure device is powered before setting GPIOs pins to HIGH state.

    // Initialize GPIOs pins used by ADC
    InitGPIO();

    // Initialize SPI peripheral used by ADC
    InitSPI();

    // Run ADC startup function
    adcStartup();
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
//! \param delay_time_ms number of milliseconds to delay.
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
//! \param delay_time_us number of microseconds to delay.
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



//****************************************************************************
//
// GPIO initialization
//
//****************************************************************************


//*****************************************************************************
//
//! Configures the MCU's GPIO pins that interface with the ADC.
//!
//! \fn void InitGPIO(void)
//!
//! \return None.
//
//*****************************************************************************
void InitGPIO(void)
{
    /* --- INSERT YOUR CODE HERE --- */
    // NOTE: Not all hardware implementations may control each of these pins...

    /* Enable the clock to the GPIO Port K and wait for it to be ready */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)))
    {
    }

    /* Configure the GPIO for 'nRESET' as output and set low */
    MAP_GPIOPinTypeGPIOOutput(nRESET_PORT, nRESET_PIN);
    MAP_GPIOPinWrite(nRESET_PORT, nRESET_PIN, 0);

    /* Configure the GPIO for 'nCS' as output and set high */
    MAP_GPIOPinTypeGPIOOutput(nCS_PORT, nCS_PIN);
    MAP_GPIOPinWrite(nCS_PORT, nCS_PIN, nCS_PIN);

    /* Enable the clock to the GPIO Port M and wait for it to be ready */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    while(!(SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)))
    {
    }

    /* Configure the GPIO for 'nDRDY' as input with falling edge interrupt */
    GPIOIntRegister(nDRDY_PORT, GPIO_DRDY_IRQHandler);
    MAP_GPIOPinTypeGPIOInput(nDRDY_PORT, nDRDY_PIN);
    MAP_GPIOIntTypeSet(nDRDY_PORT, nDRDY_PIN, GPIO_FALLING_EDGE);
    MAP_GPIOIntEnable(nDRDY_PORT, nDRDY_PIN);
    MAP_IntEnable(nDRDY_INT);
}



//*****************************************************************************
//
// Interrupt handler for nDRDY GPIO
//
//*****************************************************************************


//*****************************************************************************
//
//! Interrupt handler for nDRDY falling edge interrupt.
//!
//! \fn void GPIO_DRDY_IRQHandler(void)
//!
//! \return None.
//
//*****************************************************************************
void GPIO_DRDY_IRQHandler(void)
{
    /* --- INSERT YOUR CODE HERE --- */
    //NOTE: You many need to rename or register this interrupt function for your processor

    // In this example we set a flag and exit the interrupt routine. In the main program loop,
    // the application can examine the state flags and decide which state (operation) to perform next.

    // Alternatively, if you decide to read data here, you may want to disable other interrupts
    // to avoid partial data read operations.

    /* Get the interrupt status from the GPIO and clear the status */
    uint32_t getIntStatus = MAP_GPIOIntStatus(nDRDY_PORT, true);

    /* Check if the nDRDY pin triggered the interrupt */
    if(getIntStatus & nDRDY_PIN)
    {
        /* Interrupt action: Set a flag */
        flag_nDRDY_INTERRUPT = true;
    }

    /* Clear interrupt */
    MAP_GPIOIntClear(nDRDY_PORT, getIntStatus);

    // NOTE: We add a short delay at the end to prevent re-entrance. Refer to E2E thread:
    // https://e2e.ti.com/support/microcontrollers/tiva_arm/f/908/p/332605/1786938#1786938
    SysCtlDelay(3);
}



//****************************************************************************
//
// GPIO helper functions
//
//****************************************************************************

#ifdef EXAMPLE_CODE
#else
//*****************************************************************************
//
//! Reads that current state of the nCS GPIO pin.
//!
//! \fn bool getCS(void)
//!
//! \return boolean ('true' if high, 'false' if low).
//
//*****************************************************************************
bool getCS(void)
{
    /* --- INSERT YOUR CODE HERE --- */
    return (bool) GPIOPinRead(nCS_PORT, nCS_PIN);
}



//*****************************************************************************
//
//! Reads that current state of the nRESET GPIO pin.
//!
//! \fn bool getRESET(void)
//!
//! \return boolean ('true' if high, 'false' if low).
//
//*****************************************************************************
bool getRESET(void)
{
    /* --- INSERT YOUR CODE HERE --- */
    return (bool) GPIOPinRead(nRESET_PORT, nRESET_PIN);
}
#endif


//*****************************************************************************
//
//! Controls the state of the nCS GPIO pin.
//!
//! \fn void setCS(const bool state)
//!
//! \param state boolean indicating which state to set the nCS pin (0=low, 1=high)
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

    // td(CSSC) delay
    if(state) { SysCtlDelay(2); }

    uint8_t value = (uint8_t) (state ? nCS_PIN : 0);
    MAP_GPIOPinWrite(nCS_PORT, nCS_PIN, value);

    // td(SCCS) delay
    if(!state) { SysCtlDelay(2); }
}



//*****************************************************************************
//
//! Controls the state of the nRESET GPIO pin.
//!
//! \fn void setRESET(const bool state)
//!
//! \param state boolean indicating which state to set the nRESET pin (0=low, 1=high)
//!
//! NOTE: The 'HIGH' and 'LOW' macros defined in hal.h can be passed to this
//! function for the 'state' parameter value.
//!
//! \return None.
//
//*****************************************************************************
void setRESET(const bool state)
{
    /* --- INSERT YOUR CODE HERE --- */
    uint8_t value = (uint8_t) (state ? nRESET_PIN : 0);
    MAP_GPIOPinWrite(nRESET_PORT, nRESET_PIN, value);
}



//*****************************************************************************
//
//! Toggles the nRESET pin to trigger a reset (LOW, delay 2 ms, then HIGH).
//!
//! \fn void toggleRESET(void)
//!
//!  NOTE: The adcStartup() function should be called after resetting the device.
//!
//! \return None.
//
//*****************************************************************************
void toggleRESET(void)
{
    /* --- INSERT YOUR CODE HERE --- */
    MAP_GPIOPinWrite(nRESET_PORT, nRESET_PIN, 0);

    // Minimum nRESET pulse width (tSRLRST) equals 2,048 CLKIN periods (1 ms @ 2.048 MHz)
    delay_ms(2);

    MAP_GPIOPinWrite(nRESET_PORT, nRESET_PIN, nRESET_PIN);

    // tREGACQ delay before communicating with the device again
    delay_us(5);

    // Update register array
    restoreRegisterDefaults();
}



//*****************************************************************************
//
//! Waits for the nDRDY interrupt or until the specified timeout occurs.
//!
//! \fn bool waitForDRDYinterrupt(const uint32_t timeout_ms)
//!
//! \param timeout_ms number of milliseconds to wait before timeout event.
//!
//! \return Returns 'true' if nDRDY interrupt occurred before the timeout.
//
//*****************************************************************************
bool waitForDRDYinterrupt(const uint32_t timeout_ms)
{
    /* --- INSERT YOUR CODE HERE ---
     * Poll the nDRDY GPIO pin until it goes low. To avoid potential infinite
     * loops, you may also want to implement a timer interrupt to occur after
     * the specified timeout period, in case the nDRDY pin is not active.
     * Return a boolean to indicate if nDRDY went low or if a timeout occurred.
     */

#ifdef EXAMPLE_CODE
    // Convert ms to a # of loop iterations, OR use a timer...
#else
    // TODO: In a future revision, utilize an internal timer to implement the timeout feature
#endif
    uint32_t timeout = timeout_ms * 6000;   // convert to # of loop iterations

    // Reset interrupt flag
    flag_nDRDY_INTERRUPT = false;

    // Enable interrupts
    IntMasterEnable();

    // Wait for nDRDY interrupt or timeout - each iteration is about 20 ticks
    do {
        timeout--;
    } while (!flag_nDRDY_INTERRUPT && (timeout > 0));

    // Reset interrupt flag
    flag_nDRDY_INTERRUPT = false;

    // Timeout counter greater than zero indicates that an interrupt occurred
    return (timeout > 0);
}



//****************************************************************************
//
// SPI Communication
//
//****************************************************************************

#define SSI_BASE_ADDR       (SSI3_BASE)


//*****************************************************************************
//
//! Configures the MCU's SPI peripheral, for interfacing with the ADC.
//!
//! \fn void InitSPI(void)
//!
//! \return None.
//
//*****************************************************************************
void InitSPI(void)
{
    /* --- INSERT YOUR CODE HERE ---
     * NOTE: The ADS131A04 operates in SPI mode 1 (CPOL = 0, CPHA = 1).
     */

    //
    // Enable the clock to SSI-3 module and configure the SSI Master
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI3)))
    {
    }

    //
    // Enable clocks to GPIO Port Q and configure pins as SSI
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOQ)))
    {
    }

    MAP_GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
    //MAP_GPIOPinConfigure(GPIO_PA3_SSI0FSS); // Using GPIO for nCS instead of the FSS pin.
    MAP_GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
    MAP_GPIOPinConfigure(GPIO_PQ3_SSI3XDAT1);
    MAP_GPIOPinTypeSSI(GPIO_PORTQ_BASE, (GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3));

    // Configure: SPI MODE 1, 24 MHz SCLK, 8-bits per frame
    MAP_SSIConfigSetExpClk(SSI_BASE_ADDR, getSysClockHz(), SSI_FRF_MOTO_MODE_1,   \
                           SSI_MODE_MASTER, (getSysClockHz()/4), 8);

    //
    // Enable the SSI module.
    //
    SSIEnable(SSI_BASE_ADDR);
    SSIAdvModeSet(SSI_BASE_ADDR, SSI_ADV_MODE_READ_WRITE);
    SSIAdvFrameHoldDisable(SSI_BASE_ADDR);

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
    while(MAP_SSIDataGetNonBlocking(SSI_BASE_ADDR, &junk));
}



//*****************************************************************************
//
//! Sends SPI byte array on MOSI pin and captures MISO data to a byte array.
//!
//! \fn void spiSendReceiveArrays(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t byteLength)
//!
//! \param dataTx byte array of SPI data to send on MOSI.
//!
//! \param dataRx byte array of SPI data captured on MISO.
//!
//! \param byteLength number of bytes to send & receive.
//!
//! NOTE: Make sure 'dataTx[]' and 'dataRx[]' contain at least as many bytes of data,
//! as indicated by 'byteLength'.
//!
//! \return None.
//
//*****************************************************************************
void spiSendReceiveArrays(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t byteLength)
{
    /*  --- INSERT YOUR CODE HERE ---
     *
     *  This function should send and receive multiple bytes over the SPI.
     *
     *  A typical SPI send/receive sequence may look like the following:
     *  1) Make sure SPI receive buffer is empty
     *  2) Set the nCS pin low (if controlled by GPIO)
     *  3) Send command bytes to SPI transmit buffer
     *  4) Wait for SPI receive interrupt
     *  5) Retrieve data from SPI receive buffer
     *  6) Set the nCS pin high (if controlled by GPIO)
     */

    // Require that dataTx and dataRx are not NULL pointers
    assert(dataTx && dataRx);

    // Set the nCS pin LOW
    setCS(LOW);

    // Send all dataTx[] bytes on MOSI, and capture all MISO bytes in dataRx[]
    int i;
    for (i = 0; i < byteLength; i++)
    {
        dataRx[i] = spiSendReceiveByte(dataTx[i]);
    }

    // Set the nCS pin HIGH
    setCS(HIGH);
}



//*****************************************************************************
//
//! Sends SPI byte on MOSI pin and captures MISO return byte value.
//!
//! \fn uint8_t spiSendReceiveByte(const uint8_t dataTx)
//!
//! \param dataTx data byte to send on MOSI pin.
//!
//! NOTE: This function is called by spiSendReceiveArrays(). If it is called
//! directly, then the nCS pin must also be directly controlled.
//!
//! \return Captured MISO response byte.
//
//*****************************************************************************
uint8_t spiSendReceiveByte(const uint8_t dataTx)
{
    /*  --- INSERT YOUR CODE HERE ---
     *  This function should send and receive single bytes over the SPI.
     *  NOTE: This function does not control the nCS pin to allow for
     *  more programming flexibility.
     */

    // Remove any residual or old data from the receive FIFO
    uint32_t junk;
    while (SSIDataGetNonBlocking(SSI_BASE_ADDR, &junk));

    // SSI TX & RX
    uint8_t dataRx;
    MAP_SSIDataPut(SSI_BASE_ADDR, (uint32_t) dataTx);
    MAP_SSIDataGet(SSI_BASE_ADDR, (uint32_t *) &dataRx);

#ifdef EXAMPLE_CODE
#else
    // TODO: can we remove the SSIDataGetNonBlocking() call here and move it to spiSendReceiveArrays()?
    // TODO: Add error checking and handling here in case of TX or RX problems...
#endif
    return dataRx;
}