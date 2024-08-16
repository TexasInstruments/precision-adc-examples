/**
 * @file hal.c
 *
 * @brief Example of a hardware abstraction layer
 * @warning This software utilizes TI Drivers
 *
 * @copyright Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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
// Flag to indicate if a /DRDY interrupt has occurred
static volatile bool flag_nDRDY_INTERRUPT = false;
//****************************************************************************
//
// Internal function prototypes
//
//****************************************************************************
void InitGPIO(void);
void InitSPI(void);
void GPIO_DRDY_IRQHandler(uint_least8_t index);
//****************************************************************************
//
// External Functions (prototypes declared in hal.h)
//
//****************************************************************************

/**
 * @brief getDRDYinterruptStatus()
 * Gets the current status of nDRDY interrupt flag.
 *
 * @ return boolean status of flag_nDRDY_INTERRUPT.
 */
bool getDRDYinterruptStatus(void)
{
   return flag_nDRDY_INTERRUPT;
}
/**
 * @brief setDRDYinterruptStatus(void)
 * Sets the value of the nDRDY interrupt flag.
 *
 * @param[in] value where status is set with true; false clears the status.
 *
 * @return none
 */
void setDRDYinterruptStatus(const bool value)
{
    flag_nDRDY_INTERRUPT = value;
}
/**
 *
 * @brief enableDRDYinterrupt()
 * Enables or disables the nDRDY interrupt.
 *
 * @param[in] intEnable Where interrupt is enabled with true; false disables the interrupt.
 *
 * @return none
 */
void enableDRDYinterrupt(const bool intEnable)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */
    if(intEnable)
    {
        flag_nDRDY_INTERRUPT = false;
        GPIO_clearInt(DRDY_CONST);
        SysCtlDelay(10);
        GPIO_enableInt(DRDY_CONST);
    }
    else GPIO_disableInt(DRDY_CONST);

}

/**
 *
 * @brief InitADC()
 * Initializes MCU peripherals for interfacing with the ADC.
 *
 * @return none
 */
void InitADC(void)
{
    // IMPORTANT: Make sure device is powered before setting GPIOs pins to HIGH state.

    InitGPIO();

    // Initialize SPI peripheral
    InitSPI();

    // Run ADC startup function
    adcStartup();
}

//****************************************************************************
//
// Timing functions
//
//****************************************************************************

/**
 *
 * @brief delay_ms()
 * Provides a timing delay with 'ms' resolution.
 *
 * @param[in] delay_time_ms Is the number of milliseconds to delay.
 *
 * @return none
 */
void delay_ms(const uint32_t delay_time_ms)
{
    /* --- INSERT YOUR CODE HERE --- */

    const uint32_t cycles_per_loop = 3;
    uint32_t delayTime = getSysClockHz() / (cycles_per_loop * 1000u);
    delayTime = delayTime * delay_time_ms;
    MAP_SysCtlDelay( delayTime );
}

/**
 *
 * @brief delay_us()
 * Provides a timing delay with 'us' resolution.
 *
 * @param[in] delay_time_us Is the number of microseconds to delay.
 *
 * @return none
 */
void delay_us(const uint32_t delay_time_us)
{
    /* --- INSERT YOUR CODE HERE --- */

    const uint32_t cycles_per_loop = 3;
    uint32_t delayTime = getSysClockHz() / (cycles_per_loop * 1000000u);
    delayTime = delayTime * delay_time_us;
    MAP_SysCtlDelay( delayTime );
}

//****************************************************************************
//
// GPIO initialization
//
//****************************************************************************

/**
 *
 * @brief InitGPIO()
 * Configures the MCU's GPIO pins that interface with the ADC.
 *
 * @return none
 *
 */
void InitGPIO(void)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */

    /* Call driver init functions */
    GPIO_init();

    /* Set the interrupt callback function */
    GPIO_setCallback(DRDY_CONST,GPIO_DRDY_IRQHandler );
}
//*****************************************************************************
//
// Interrupt handler for nDRDY GPIO
//
//*****************************************************************************

/**
 *
 * @brief GPIO_DRDY_IRQHandler()
 * Interrupt handler for nDRDY falling edge interrupt.
 *
 * @param[in] index Position of the interrupt for callback.
 *
 * @return none
 */
void GPIO_DRDY_IRQHandler(uint_least8_t index)
{
    /* --- INSERT YOUR CODE HERE --- */

    //NOTE: You many need to rename or register this interrupt function for your processor

    /* Interrupt action: Set a flag */
    flag_nDRDY_INTERRUPT = true;

}

//****************************************************************************
//
// GPIO helper functions
//
//****************************************************************************

/**
 *
 * @brief waitForDRDYinterrupt()
 * Waits for the nDRDY interrupt or until the specified timeout occurs.
 *
 * @param[in] timeout_ms Number of milliseconds to wait before timeout event.
 *
 * @return Returns 'true' if nDRDY interrupt occurred before the timeout.
 */
bool waitForDRDYinterrupt(const uint32_t timeout_ms)
{
    /* --- INSERT YOUR CODE HERE --- */


    /* The following code is based on a TI Drivers implementation */
    /*
     * Poll the nDRDY GPIO pin until it goes low. To avoid potential infinite
     * loops, you may also want to implement a timer interrupt to occur after
     * the specified timeout period, in case the nDRDY pin is not active.
     * Return a boolean to indicate if nDRDY went low or if a timeout occurred.
     */

    uint32_t timeout = timeout_ms * 6000;   // convert to # of loop iterations

    // Reset interrupt flag
    flag_nDRDY_INTERRUPT = false;

    // Enable interrupts
    GPIO_clearInt(DRDY_CONST);
    delay_us(1);
    GPIO_enableInt(DRDY_CONST);

    // Wait for nDRDY interrupt or timeout - each iteration is about 20 ticks
    do {
        timeout--;
    } while (!flag_nDRDY_INTERRUPT && (timeout > 0));

    GPIO_disableInt(DRDY_CONST);

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

/**
 *
 * @brief setCS()
 * Controls the state of the /CS GPIO pin.
 *
 * @param[in] state boolean indicating which state to set the /CS pin (0=low, 1=high)
 *
 * NOTE: The 'HIGH' and 'LOW' macros defined in hal.h can be passed to this
 *       function for the 'state' parameter value.
 *
 * @return none.
 */
void setCS(const bool state)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */
    if(state) delay_us(1);
    uint8_t value = (uint8_t) (state ? 1 : 0);
    GPIO_write(CS_CONST, value);
    if(!state) delay_us(1);
}

/**
 *
 * @brief InitSPI()
 * Configures the MCU's SPI peripheral, for interfacing with the ADC.
 *
 * @return none.
 */
void InitSPI(void)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation for GPIO functions */

    /*
     * NOTE: The ADS1118 operates in SPI mode 1 (CPOL = 0, CPHA = 1).
     */

    //
    // Enable the clock to SSI-3 module and configure the SSI Master
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI3)))
    {
    }

    //
    // Enable clocks to GPIO Port Q and configure pins as SSI
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOQ)))
    {
    }

    GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
    GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
    GPIOPinConfigure(GPIO_PQ3_SSI3XDAT1);
    GPIOPadConfigSet(GPIO_PORTQ_BASE, (GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3), GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPD);
    GPIOPinTypeSSI(GPIO_PORTQ_BASE, (GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3));

    // Configure: SPI MODE 1, 1 MHz SCLK, 8-bits per frame
    SSIConfigSetExpClk(SSI_BASE_ADDR, getSysClockHz(), SSI_FRF_MOTO_MODE_1,   \
                           SSI_MODE_MASTER, 1000000, 8);
    //
    // Enable the SSI module.
    //
    SSIEnable(SSI_BASE_ADDR);

    GPIO_setCallback(DRDY_CONST,GPIO_DRDY_IRQHandler );

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
    while(SSIDataGetNonBlocking(SSI_BASE_ADDR, &junk));
}

/**
 *
 * @brief spiSendReceiveArrays()
 * Sends SPI byte array on DIN pin and captures DOUT data to a byte array
 *
 * @param[in] dataTx Const unsigned byte array of SPI data to send on DIN.
 *
 * @param[out] dataRx Unsigned byte array of SPI data captured on DOUT.
 *
 * @param[in] byteLength Total number of bytes to send and receive.
 *
 * Note: Make sure 'dataTx[]' and 'dataRx[]' contain at least as many bytes of data,
 *       as indicated by 'byteLength'.
 *
 * @return none.
 */
void spiSendReceiveArrays(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t byteLength)
{
    /*  --- INSERT YOUR CODE HERE --- */

    /*
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

    // For the ADS1118 reading the first 2 bytes is the data and the second 2 bytes is configuration

    // Require that dataTx and dataRx are not NULL pointers
    assert(dataTx && dataRx);

    // Set the nCS pin LOW
    setCS(LOW);

    // Send all dataTx[] bytes on DIN, and capture all DOUT bytes in dataRx[]
    // For ADS1118 first 16 bits are conversion data second 16 bits is configuration.
    // 32-bit sequence forces DOUT high if pin polling is desired instead of interrupt.
    int i;
    for (i = 0; i < byteLength; i++)
    {   // 16-bit configuration is sent twice...first 16 bits are conversion data, second 16 bits is configuration
        if (i<2) dataRx[i] = spiSendReceiveByte(dataTx[i]);
        else dataRx[i-2] = spiSendReceiveByte(dataTx[i-2]);
    }

    // Set the nCS pin HIGH
    setCS(HIGH);
}

/**
 *
 * @brief spiSendReceiveByte()
 * Sends SPI byte on DIN pin and captures DOUT return byte value.
 *
 * @param[in] dataTx Const unsigned data byte to send on DIN.
 *
 * NOTE: This function is called by spiSendReceiveArrays(). If it is called
 *       directly, then the /CS pin must also be directly controlled.
 *
 * @return dataRx Captured DOUT response byte.
 */
uint8_t spiSendReceiveByte(const uint8_t dataTx)
{
    /*  --- INSERT YOUR CODE HERE --- */

    /*
     *  This function should send and receive single bytes over the SPI.
     *  NOTE: This function does not control the /CS pin to allow for
     *  more programming flexibility.
     */

    // Remove any residual or old data from the receive FIFO
    uint32_t junk;
    while (SSIDataGetNonBlocking(SSI_BASE_ADDR, &junk));

    // SSI TX & RX
    uint8_t dataRx;
    SSIDataPut(SSI_BASE_ADDR, (uint32_t) dataTx);
    SSIDataGet(SSI_BASE_ADDR, (uint32_t *) &dataRx);

    return dataRx;
}

