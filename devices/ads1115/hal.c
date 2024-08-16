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
/* Following statements relate to use of TI Driver implementation */
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CMSP432E4.h>
extern I2C_Handle g_I2Chandle;

//****************************************************************************
//
// Internal variables
//
//****************************************************************************
// Flag to indicate if an ALERT/RDY interrupt has occurred
static volatile bool flag_nALERT_INTERRUPT = false;
//****************************************************************************
//
// Internal function prototypes
//
//****************************************************************************
void InitGPIO(void);
void InitI2C(void);

//****************************************************************************
//
// External Functions (prototypes declared in hal.h)
//
//****************************************************************************
/**
 * @brief getALERTinterruptStatus()
 * Gets the current status of ALERT/RDY interrupt flag.
 *
 * @ return boolean status of flag_nALERT_INTERRUPT.
 */
bool getALERTinterruptStatus(void)
{
   return flag_nALERT_INTERRUPT;
}
/**
 * @brief setALERTinterruptStatus(void)
 * Sets the value of the ALERT/RDY interrupt flag.
 *
 * @param[in] value where status is set with true; false clears the status.
 *
 * @return none
 */
void setALERTinterruptStatus(const bool value)
{
    flag_nALERT_INTERRUPT = value;
}
/**
 *
 * @brief enableALERTinterrupt()
 * Enables or disables the ALERT/RDY interrupt.
 *
 * @param[in] intEnable Where interrupt is enabled with true; false disables the interrupt.
 *
 * @return none
 */
void enableALERTinterrupt(const bool intEnable)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */
    if(intEnable)
    {
        flag_nALERT_INTERRUPT = false;
        GPIO_clearInt(ALERT_CONST);
        SysCtlDelay(10);
        GPIO_enableInt(ALERT_CONST);
    }
    else GPIO_disableInt(ALERT_CONST);
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

    // Initialize GPIOs pins used by ADS1x15
    InitGPIO();
    /* Call driver init functions */
    InitI2C();
    // Run ADC startup function (in ADS1115.c)
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
//
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
    // NOTE: Not all hardware implementations may control each of these pins...

    /* The following code is based on a TI Drivers implementation */

    /* Call driver init functions */
    GPIO_init();

    /* Set the interrupt callback function */
    GPIO_setCallback(ALERT_CONST,GPIO_ALERT_IRQHandler );

}
//*****************************************************************************
//
// Interrupt handler for ALERT/RDY GPIO
//
//*****************************************************************************

/**
 *
 * @brief GPIO_ALERT_IRQHandler()
 * Interrupt handler for ALERT/RDY falling edge interrupt.
 *
 * @param[in] index Position of the interrupt for callback.
 *
 * @return none
 */
void GPIO_ALERT_IRQHandler(uint_least8_t index)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* NOTE: You many need to rename or
     * register this interrupt function for your processor.
     *  - Possible ways to handle this interrupt:
     *  1. If you decide to read data here, you may want to
     *      disable other interrupts to avoid partial data reads.
     *  2. Set a flag to read during a polling loop
     *
     *  In this example we set a flag and exit the interrupt routine.
     *  In the main program loop, your application can examine
     *  all state flags and decide which state (operation) to perform next.
     */

    /* Interrupt action: Set a flag */
    flag_nALERT_INTERRUPT = true;
}

//****************************************************************************
//
// GPIO helper functions
//
//****************************************************************************
/**
 *
 * @brief waitForALERTinterrupt()
 * Waits for the ALERT/RDY interrupt or until the specified timeout occurs.
 *
 * @param[in] timeout_ms Number of milliseconds to wait before timeout event.
 *
 * @return Returns 'true' if ALERT/RDY interrupt occurred before the timeout.
 */
bool waitForALERTinterrupt(const uint32_t timeout_ms)
{
    /* --- INSERT YOUR CODE HERE ---
     *
     * Poll the ALERT/RDY GPIO pin until it goes low. To avoid potential infinite
     * loops, you may also want to implement a timer interrupt to occur after
     * the specified timeout period, in case the ALERT/RDY pin is not active.
     * Return a boolean to indicate if ALERT/RDY went low or if a timeout occurred.
     */

    /* The following code is based on a TI Drivers implementation */

    // Convert ms to a # of loop iterations, OR even better use a timer here...
    uint32_t timeout = timeout_ms * 6000;   // convert to # of loop iterations

    // Reset interrupt flag
    flag_nALERT_INTERRUPT = false;

    // Enable interrupts
    GPIO_clearInt(ALERT_CONST);
    delay_us(1);
    GPIO_enableInt(ALERT_CONST);

    // Wait for nDRDY interrupt or timeout - each iteration is about 20 ticks
    do {
        timeout--;
    } while (!flag_nALERT_INTERRUPT && (timeout > 0));

    GPIO_disableInt(ALERT_CONST);

    // Reset interrupt flag
    flag_nALERT_INTERRUPT = false;

    // Timeout counter greater than zero indicates that an interrupt occurred
    return (timeout > 0);
}
/**
 *
 * @brief InitI2C()
 * Configures the MCU's I2C peripheral, for interfacing with target devices.
 *
 * @return none
 */
void InitI2C(void)
{
    /* --- INSERT YOUR CODE HERE --- */

    //
    // Enabling I2C2 peripheral.
    //

    //
    // Configuring the pin muxing for I2C2 functions.
    //

    //
    // Enabling and initializing the I2C2 master module.
    //

    /* The following code is based on a TI Drivers implementation */

    /* Call driver init functions */
    I2C_init();
}

/**
 *
 * @brief sendI2CData()
 * Configures the MCU's I2C peripheral, for interfacing with target devices.
 *
 * @param[in] address Of target device.
 * @param[in] arrayIndex Is pointer to an array of data to be transmitted.
 * @param[in] length Of the data array.
 *
 * @return 0 is communication success; < 0 is communication failure.
 */
int8_t sendI2CData(uint8_t address, uint8_t *arrayIndex, uint8_t length)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */
    int8_t retStatus = 0;
    uint8_t wData[256];
    uint8_t rData[1];
    wData[0] = address;
    uint16_t i;
    // Copy the data array
    for(i = 1; i < (length + 1); i++)
    {
        wData[i] = arrayIndex[i-1];
    }
    //
    // Initialize the optional I2C bus parameters
    //
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;
    //
    // Open the I2C bus for usage
    //
    I2C_Handle i2cHandle = I2C_open(I2Cbus, &params);
    //
    // Initialize the slave address for transactions
    //
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = ADS1115_ADDRESS;
    transaction.readBuf = rData;
    transaction.readCount = 0;
    transaction.writeBuf = wData;
    transaction.writeCount = length + 1;
    retStatus = I2C_transfer(i2cHandle, &transaction);

    //
    // Return error, if read failed
    //
    if (retStatus == false)
    {
        /* Send error response here then close the port */
        I2C_close(i2cHandle);
        return -1;
    }

    I2C_close(i2cHandle);

    return 0;
}
/**
 *
 * @brief sendI2CRegPointer()
 * Write to set the desired pointer position for reading/writing register data.
 *
 * @param[in] address Of target device.
 * @param[in] arrayIndex Is pointer to an array of data to be transmitted.
 * @param[in] length Of the data array.
 *
 * @return 0 is communication success; < 0 is communication failure.
 */
int8_t sendI2CRegPointer(uint8_t address, uint8_t *arrayIndex, uint8_t length)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */

    int8_t retStatus = 0;
    uint8_t wData[256];
    uint8_t rData[1];
    wData[0] = address;
    uint16_t i;

    for(i = 1; i < (length + 1); i++)
    {
        wData[i] = arrayIndex[i-1];
    }

    //
    // Initialize the slave address for transactions
    //
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = ADS1115_ADDRESS;

    transaction.readBuf = rData;
    transaction.readCount = 0;
    transaction.writeBuf = wData;
    transaction.writeCount = length + 1;
    retStatus = I2C_transfer(g_I2Chandle, &transaction);

    //
    // Return error, if read failed
    //
    if (retStatus == false)
    {
        // Add error handling routine
        return -1;
    }

    return 0;
}
/**
 *
 * @brief receiveI2CData()
 * Read data from the desired pointer position for register data.
 *
 * @param[in] address Of target device.
 * @param[out] arrayIndex Is pointer to an array of data to be transmitted.
 * @param[in] length Of the data array.
 *
 * @return 0 is communication success; < 0 is communication failure.
 */
int8_t receiveI2CData(uint8_t address, uint8_t *arrayIndex, uint8_t length)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */

    int8_t retStatus = 0;
    uint8_t wData[1];
    uint8_t rData[256];
    wData[0] = address;
    uint16_t i;
    //
    // Initialize the optional I2C bus parameters
    //
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_400kHz;
    //
    // Open the I2C bus for usage
    //
    I2C_Handle i2cHandle = I2C_open(I2Cbus, &params);
    //
    // Initialize the slave address for transactions
    //
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = ADS1115_ADDRESS;

    transaction.readBuf = rData ;
    transaction.readCount = length;
    transaction.writeBuf = wData;
    transaction.writeCount = 1;
    retStatus = I2C_transfer(i2cHandle, &transaction);

    //
    // Return error, if read failed
    //
    if (retStatus == false)
    {
        I2C_close(i2cHandle);
        // Insert error handling routine
        return -1;
    }

    I2C_close(i2cHandle);

    for(i = 0; i < length; i++)
    {
        arrayIndex[i] = rData[i];
    }

    return 0;
}
/**
 *
 * @brief receiveI2CDataNoWrite()
 * Read data from the desired pointer position for register data.
 *
 * @param[in] address Of target device.
 * @param[out] arrayIndex Is pointer to an array of data to be transmitted.
 * @param[in] length Of the data array.
 * @param[in] noWrite Boolean of condition if write command is required.
 * false = no write of device address, true = write device address
 *
 * @return 0 is communication success; < 0 is communication failure.
 */
int8_t receiveI2CDataNoWrite(uint8_t address, uint8_t *arrayIndex, uint8_t length, bool noWrite)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */

    int8_t retStatus = 0;
    uint8_t wData[1];
    uint8_t rData[256];
    wData[0] = address;
    uint16_t i;
    //
    // Initialize the slave address for transactions
    //
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = ADS1115_ADDRESS;

    transaction.readBuf = rData ;
    transaction.readCount = length;
    transaction.writeBuf = wData;
    if(noWrite) transaction.writeCount = 0;
    else transaction.writeCount = 1;
    retStatus = I2C_transfer(g_I2Chandle, &transaction);

    //
    // Return error, if read failed
    //
    if (retStatus == false)
    {
        // Insert error handling routine
        return -1;
    }

    for(i = 0; i < length; i++)
    {
        arrayIndex[i] = rData[i];
    }

    return 0;
}
