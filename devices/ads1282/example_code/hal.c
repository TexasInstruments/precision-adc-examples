/**
 * @file hal.c
 *
 * @brief Hardware abstraction layer (HAL) implementation
 *
 * @warning This HAL implementation is provided as an example of how to interface
 * to TI SysConfig supported processors (https://www.ti.com/tool/SYSCONFIG).
 * This code will need to be re-implemented to target other processors!
 *
 * @copyright Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
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

/// Flag to indicate if a /DRDY interrupt has occurred
static volatile bool flag_nDRDY_INTERRUPT = false;

/// SPI Peripheral handle for TI Driver
SPI_Handle ads1282_spi_handle;


//****************************************************************************
//
// Internal function prototypes
//
//****************************************************************************

void initializeGPIO(void);
void initializeSPI(void);
void nDRDYinterruptHandler(void);


//*****************************************************************************
//
//! Initializes MCU peripherals for interfacing with the ADC then calls the
//! adcStartupRoutine() function.
//!
//! \fn void initADCperhiperhals(void)
//!
//! \return None.
//
//*****************************************************************************
void initADCperhiperhals(void)
{
    /* --- INSERT YOUR CODE HERE --- */

    // IMPORTANT: Make sure device is powered before setting GPIOs pins to HIGH state.
    // If you have additional code to power the ADC you may add it here.

    initializeGPIO();       // Initialize GPIO pins
    initializeSPI();        // Initialize SPI peripheral

    adcStartupRoutine();    // Run ADC startup function
}


//****************************************************************************
//
// GPIO functions
//
//****************************************************************************

//*****************************************************************************
//
//! Configures the MCU's GPIO pins that interface with the ADC.
//!
//! \fn void initializeGPIO(void)
//!
//! \return None.
//
//*****************************************************************************
void initializeGPIO(void)
{
    /* --- INSERT YOUR CODE HERE --- */
    /* The following code is based on a TI Drivers implementation */

    // Initialize the GPIO driver
    GPIO_init();

    // Configure callback function for /DRDY falling edge interrupt
    //GPIO_setCallback(DRDY_CONST, nDRDYinterruptHandler);                  // TODO: Include this or not? Do we need to test it?
}


//*****************************************************************************
//
//! Controls the state of the /PWDN GPIO pin.
//!
//! \fn setPWDN(const bool state)
//!
//! \param state boolean indicating which state to set the /PWDN pin (0=low, 1=high).
//!
//! The 'HIGH' and 'LOW' macros defined in hal.h can be use for the 'state' parameter.
//!
//! \return None.
//
//*****************************************************************************
void setPWDN(const bool state)
{
    /* --- INSERT YOUR CODE HERE --- */

    // The following code is based on a SimpleLink MCU SDK "TI Drivers" implementation
    GPIO_write(ADC_nPWDN, (state ? 1 : 0));

    // Update internal state variables when powering down the device
    if (!state) { _restoreRegisterDefaults(); }
}


//*****************************************************************************
//
//! Controls the state of the /RESET GPIO pin.
//!
//! \fn void setRESET(const bool state)
//!
//! \param state boolean indicating which state to set the /RESET pin (0=low, 1=high).
//!
//! The 'HIGH' and 'LOW' macros defined in hal.h can be use for the 'state' parameter.
//!
//! \return None.
//
//*****************************************************************************
void setRESET(const bool state)
{
    /* --- INSERT YOUR CODE HERE --- */

    // The following code is based on a SimpleLink MCU SDK "TI Drivers" implementation
    GPIO_write(ADC_nRESET, (state ? 1 : 0));

    // Update internal state variables when resetting the device
    if (!state) { _restoreRegisterDefaults(); }
}


//*****************************************************************************
//
//! Toggles the /RESET pin to trigger a reset
//! (LOW, delay 2 ms, then HIGH).
//!
//! \fn void toggleRESET(void)
//!
//! \return None.
//
//*****************************************************************************
void toggleRESET(void)
{
    setRESET(LOW);
    delay_us(2);      // Minimum nRESET pulse low width = 2 tCLKs (2 us @ 1 MHz)
    setRESET(HIGH);

    // Update internal state variables when resetting the device
    // NOTE: This is only needed if setRESET() is not used above!
    //_restoreRegisterDefaults();
}


//*****************************************************************************
//
//! Interrupt handler for /DRDY falling edge interrupt.
//!
//! \warning You many need to rename or register this interrupt function with your processor
//!
//! \fn void nDRDYinterruptHandler(void)
//!
//! \return None.
//
//*****************************************************************************
void nDRDYinterruptHandler(void)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* Interrupt action: Set a flag */
    flag_nDRDY_INTERRUPT = true;
}


//*****************************************************************************
//
//! Waits for the /DRDY interrupt or until the specified timeout occurs.
//!
//! \fn bool waitForDRDYinterrupt(const uint32_t timeout_ms)
//!
//! \param timeout_ms Number of milliseconds to wait before timeout event.
//!
//! \return Returns 'true' if /DRDY interrupt occurred before the timeout.
//
//*****************************************************************************
bool waitForDRDYinterrupt(const uint32_t timeout_ms)
{
    /* --- INSERT YOUR CODE HERE --- */
    /* The following code is based on a TI Drivers implementation */

    /*
     * Poll the /DRDY GPIO pin until it goes low. To avoid potential infinite
     * loops, you may also want to implement a timer interrupt to occur after
     * the specified timeout period, in case the /DRDY pin is not active.
     * Return a boolean to indicate if /DRDY went low or if a timeout occurred.
     */

    uint32_t timeout = timeout_ms * 6000;   // convert to # of loop iterations

    // Reset interrupt flag
    flag_nDRDY_INTERRUPT = false;

    // Enable interrupt
    GPIO_clearInt(ADC_nDRDY);
    GPIO_enableInt(ADC_nDRDY);

    // Wait for /DRDY interrupt or timeout - each iteration is about 20 ticks
    do {
        timeout--;
    } while (!flag_nDRDY_INTERRUPT && (timeout > 0));

    // Disable interrupt
    GPIO_disableInt(ADC_nDRDY);

    return flag_nDRDY_INTERRUPT;
}


//*****************************************************************************
//
// SPI Communication
//
//*****************************************************************************

//*****************************************************************************
//
//! Configures the MCU's SPI peripheral for interfacing with the ADC.
//!
//! \fn void initializeSPI(void)
//!
//! \return None.
//
//*****************************************************************************
void initializeSPI(void)
{
    /* --- INSERT YOUR CODE HERE --- */
    /* The following code is based on a TI Drivers implementation */

    // Initialize the SPI driver
    SPI_init();

    // Open an SPI driver instance
    SPI_Params      spiParams;
    SPI_Params_init(&spiParams);
    spiParams.dataSize = 8;
    spiParams.bitRate = SCLK_FREQ_HZ;
    spiParams.frameFormat = SPI_POL0_PHA0;
    ads1282_spi_handle = SPI_open(CONFIG_SPI_0, &spiParams);    // WARNING: This may toggle SCLK.
    if (ads1282_spi_handle == NULL) {
        // SPI_open() failed
    }

    // Enable pull-down on SCLK (using Driverlib) to hold it low when when TI Driver is inactive.
    GPIOQ->PDR |= (GPIO_PIN_0);
}

//*****************************************************************************
//
//! Sends SPI byte array on MOSI pin and captures MISO data to a byte array.
//!
//! \fn bool spiSendReceive(const uint8_t transmitBuffer[], uint8_t receiveBuffer[], const uint8_t byteLength)
//!
//! \param[in] transmitBuffer[] byte array of SPI data to send on MOSI.
//!
//! \param[out] receiveBuffer[] byte array of SPI data captured on MISO.
//!
//! \param[in] byteLength number of bytes to send & receive.
//!
//! NOTE: Make sure 'dataTx[]' and 'dataRx[]' contain at least as many bytes of data,
//! as indicated by 'byteLength'.
//!
//! \return None.
//
//*****************************************************************************
bool spiSendReceive(const uint8_t transmitBuffer[], uint8_t receiveBuffer[], const uint8_t byteLength)
{
    /*  --- INSERT YOUR CODE HERE --- */
    /* The following code is based on a TI Drivers implementation */

    SPI_Transaction spiTransaction;
    spiTransaction.count = byteLength;
    spiTransaction.txBuf = (void *) transmitBuffer;
    spiTransaction.rxBuf = (void *) receiveBuffer;

    bool transferOK = SPI_transfer(ads1282_spi_handle, &spiTransaction);
    return transferOK;
}


//****************************************************************************
//
// Timing functions
//
//****************************************************************************

//*****************************************************************************
//
//! Blocking delay function with approximate 'us' resolution.
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
    MAP_SysCtlDelay( (delay_time_us / cycles_per_loop) * (getSysClockHz() / 1000000u) );
}

//*****************************************************************************
//
//! Blocking delay function with approximate 'ms' resolution.
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
    // This implementations uses the ARM SysTick timer configured for a 1 ms period

    resetSysTickCount();
    while (getSysTickCount() <= delay_time_ms)
    {
        // do nothing
    }
}
