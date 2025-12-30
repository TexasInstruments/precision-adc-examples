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
#include <ti/drivers/GPIO.h>
#include "lib/json.h"
/* Driver configuration */
#include "ti_drivers_config.h"
#include "ti/devices/msp432e4/driverlib/driverlib.h"
#ifdef SPI_COMMS
#include <ti/drivers/SPI.h>
#define SPI_SPEED 10000000
#define SPI_WORD_SIZE 8
SPI_Handle g_SPIhandle;
#endif
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
#ifdef EXAMPLE_CODE
void InitGPIO(void);
#ifdef USE_I2C
void InitI2C(void);
#endif
#else
void InitGPIO(void);
#endif
//TODO: Why are these prototypes declared here and not in the hal.h?
//void InitSPI(void);
#ifdef EXAMPLE_CODE
#else
void GPIO_DRDYn_IRQHandler(uint_least8_t index);
#endif
//****************************************************************************
//
// External Functions (prototypes declared in hal.h)
//
//****************************************************************************
#ifdef EXAMPLE_CODE
#else
bool getDRDYninterruptStatus(void)
{
   return flag_nDRDY_INTERRUPT;
}
void setDRDYninterruptStatus(const bool value)
{
    flag_nDRDY_INTERRUPT = value;
}
void enableDRDYninterrupt(const bool intEnable)
{
    if(intEnable)
    {
        flag_nDRDY_INTERRUPT = false;
        GPIO_clearInt(DRDYn_CONST);
        SysCtlDelay(10);
        GPIO_enableInt(DRDYn_CONST);
    }
    else GPIO_disableInt(DRDYn_CONST);
}
#endif
/**
 *
 * @brief InitADC()
 * Initializes MCU peripherals for interfacing with the ADC.
 *
 * @return none
 */
//****************************************************************************
//
// Timing functions
//
/*****************************************************************************
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
#ifdef EXAMPLE_CODE
    /* --- INSERT YOUR CODE HERE --- */

#else
    const uint32_t cycles_per_loop = 3;
    uint32_t delayTime = getSysClockHz() / (cycles_per_loop * 1000u);
    delayTime = delayTime * delay_time_ms;
    MAP_SysCtlDelay( delayTime );
#endif
}
/**
 *@brief delay_us()
 * Provides a timing delay with 'us' resolution.
 *
 * @param[in] delay_time_us Is the number of microseconds to delay.
 *
 * @return none
 */
//
void delay_us(const uint32_t delay_time_us)
{
#ifdef EXAMPLE_CODE
    /* --- INSERT YOUR CODE HERE --- */
#else
    const uint32_t cycles_per_loop = 3;
    uint32_t delayTime = getSysClockHz() / (cycles_per_loop * 1000000u);
    delayTime = delayTime * delay_time_us;
    MAP_SysCtlDelay( delayTime );
#endif
}
//****************************************************************************
//
// GPIO initialization
//
//****************************************************************************
/*
 * @brief InitGPIO()
 * Configures the MCU's GPIO pins that interface with the ADC.
 *
 * @return none
 *
 */
void InitGPIO(void)
{
    /* The following code is based on a TI Drivers implementation */
    /* Call driver init functions */
    GPIO_init();    // this is the TI-Driver
// change the GPIO config to add a weak pull up to CSn
//   GPIOPadConfigSet(portBase, pinMask, strength, gpioType);
    /* Set the interrupt callback function */
    GPIO_setCallback(DRDYn_CONST,GPIO_DRDYn_IRQHandler );
}
//*****************************************************************************
//
// Interrupt handler for nDRDY GPIO
//
//*****************************************************************************
//! Interrupt handler for /DRDY falling edge interrupt.
//!
//! \fn void GPIO_DRDYn_IRQHandler(void)
//!
//! \return None.
//
//*****************************************************************************
void GPIO_DRDYn_IRQHandler(uint_least8_t index)
{
    /* --- INSERT YOUR CODE HERE --- */
    //NOTE: You many need to rename or register this interrupt function for your processor
    // TODO: This is sort of generic, but this should be reviewed and adjusted appropriately
    // Possible ways to handle this interrupt:
    // If you decide to read data here, you may want to disable other interrupts to avoid partial data reads.
    // In this example we set a flag and exit the interrupt routine. In the main program loop, your application can examine
    // all state flags and decide which state (operation) to perform next.
    /* Interrupt action: Set a flag */
    flag_nDRDY_INTERRUPT = true;
}
//****************************************************************************
//
// GPIO helper functions
//
//****************************************************************************
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
    // This will be device specific and will require changes from device to device
    // Convert ms to a # of loop iterations, OR even better use a timer here...
    uint32_t timeout = timeout_ms * 6000;   // convert to # of loop iterations
    // Reset interrupt flag
    flag_nDRDY_INTERRUPT = false;
    // Enable interrupts
    GPIO_clearInt(DRDYn_CONST);
   // SysCtlDelay(10);
    GPIO_enableInt(DRDYn_CONST);
    // Wait for nDRDY interrupt or timeout - each iteration is about 20 ticks
    do {
        timeout--;
    } while (!flag_nDRDY_INTERRUPT && (timeout > 0));
    GPIO_disableInt(DRDYn_CONST);
    // Reset interrupt flag
    flag_nDRDY_INTERRUPT = false;
    // Timeout counter greater than zero indicates that an interrupt occurred
    return (timeout > 0);
}
bool getDRDYstatus(void)
{
    return flag_nDRDY_INTERRUPT;
}
void setDRDYstatus(bool DRDYflag)
{
    if (DRDYflag) flag_nDRDY_INTERRUPT = false;
}
#ifdef SPI_COMMS
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
void InitSPI(void)
{
    SPI_Params      spiParams;
    SPI_Params_init( &spiParams );
    spiParams.dataSize    = SPI_WORD_SIZE;
    spiParams.frameFormat = SPI_POL0_PHA1;
    spiParams.bitRate     = SPI_SPEED;

    g_SPIhandle = SPI_open( CONFIG_SPI_0, &spiParams );
    return;//( status );
}
//*****************************************************************************
void setCS(const bool state)
{
    uint8_t value = (uint8_t) (state ? 1 : 0);
 //  GPIO_write(CSn_CONST, value);
}
void enableCSpulldown(void)
{
    // adjust CSn pin pad type to pull down and observe DRDY signal.
    GPIOPadConfigSet(GPIO_PORTQ_BASE, GPIO_PIN_1, GPIO_CFG_OUT_STR_MED, GPIO_PIN_TYPE_STD_WPD);
}
void enableCSpullup(void)
{
    // return bus configuration to allow CS to be pulled up high
    GPIOPadConfigSet(GPIO_PORTQ_BASE, GPIO_PIN_1, GPIO_CFG_OUT_STR_MED, GPIO_PIN_TYPE_STD_WPU);
}
//*****************************************************************************
//*****************************************************************************
//
//! Sends SPI byte array on MOSI pin and captures MISO data to a byte array.
//!
//! \fn void spiSendReceiveArrays(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t byteLength)
//!
//! \param const uint8_t dataTx[] byte array of SPI data to send on MOSI.
//!
//! \param uint8_t dataRx[] byte array of SPI data captured on MISO.
//!
//! \param uint8_t byteLength number of bytes to send & receive.
//!
//! NOTE: Make sure 'dataTx[]' and 'dataRx[]' contain at least as many bytes of data,
//! as indicated by 'byteLength'.
//!
//! \return None.
//
//*****************************************************************************
bool spiSendReceiveArrays(const uint8_t dataTx[], uint8_t dataRx[], const uint8_t bufferLength)
{
    SPI_Transaction spiTransaction;
    spiTransaction.count = bufferLength;
    spiTransaction.txBuf = (void *) dataTx;
    spiTransaction.rxBuf = (void *) dataRx;

    bool            transferOK;
    transferOK = SPI_transfer( g_SPIhandle, &spiTransaction);
    if (transferOK == NULL) {
        // send error message
    }
    return transferOK;
}

#endif

//*****************************************************************************
//
//                   I2C Communications
//
//*****************************************************************************

int8_t transferI2CData( uint8_t *writeBuff, uint8_t wLength, uint8_t *readBuff, uint8_t rLength)
{
    int8_t retStatus = 0;
    uint8_t wData[256];
    uint8_t rData[256];
    uint16_t i;
    for(i = 0; i < (wLength); i++)
    {
        wData[i] = writeBuff[i];
    }
    //
    // Initialize the optional I2C bus parameters
    //
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_1000kHz;
    //
    // Open the I2C bus for usage
    //
    I2C_Handle i2cHandle = I2C_open(I2Cbus, &params);
    //
    // Initialize the slave address for transactions
    //
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = ADS122C14_ADDRESS;
    transaction.readBuf = rData;
    transaction.readCount = rLength;
    transaction.writeBuf = wData;
    transaction.writeCount = wLength;
    retStatus = I2C_transfer(i2cHandle, &transaction);
    //
    // Return error, if read failed
    //
    if (retStatus == false)
    {
        I2C_close(i2cHandle);
        json_error(ERROR_I2C_READ_WRITE_FAILED,"I2C read failed");
        return -1;
    }
    I2C_close(i2cHandle);
    for(i = 0; i < rLength; i++)
    {
        readBuff[i] = rData[i];
    }
    return 0;
}
