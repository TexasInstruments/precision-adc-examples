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
#include <ti/drivers/GPIO.h>
#include "ti_drivers_config.h"
extern I2C_Handle g_I2Chandle;
extern I2C_Params g_I2Cparams;

/****************************************************************************
 *
 * Internal variables
 *
 ****************************************************************************/
// Flag to indicate if a /DRDY interrupt has occurred
static volatile bool flag_nDRDY_INTERRUPT = false;

/****************************************************************************
 *
 * Internal function prototypes
 *
 ****************************************************************************/

void GPIO_DRDY_IRQHandler(uint_least8_t index);

/****************************************************************************
 *
 * Internal function prototypes
 *
 ****************************************************************************/

/**
 *
 * @brief getDRDYinterruptStatus()
 * Returns the value of the DRDY interrupt flag.
 *
 * @return bool Of the interrupt status flag
 */
bool getDRDYinterruptStatus(void)
{
   return flag_nDRDY_INTERRUPT;
}

/**
 *
 * @brief setDRDYinterruptStatus()
 * Sets the value of the DRDY interrupt flag.
 *
 * @param[in] value Is the boolean value to set/clear for the interrupt flag.
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
 * Sets the state for the DRDY interrupt.
 *
 * @param[in] intEnable Is the boolean value to enable/disable the DRDY interrupt.
 *
 * @return none
 */
void enableDRDYinterrupt(const bool intEnable)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* Following statements relate to use of a TI Driver implementation */

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
 * @brief InitADCPeripherals()
 * Initializes MCU peripherals for interfacing with the ADC.
 *
 * @return none
 */
void InitADCPeripherals(void)
{
    // IMPORTANT: Make sure device is powered before setting GPIOs pins to HIGH state.
    // Initialize GPIOs pins used by ADS1x2C04
    InitGPIO();

    // Initialize the I2C bus for communication
    InitI2C();

    // Run ADC startup function (in ADS122C04.c)
    adcStartup();
}

/****************************************************************************
 *
 * Timing functions
 *
 ****************************************************************************/

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
    uint32_t delayTime = MASTER_CLOCK / (cycles_per_loop * 1000u);
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
    uint32_t delayTime = MASTER_CLOCK / (cycles_per_loop * 1000000u);
    delayTime = delayTime * delay_time_us;
    MAP_SysCtlDelay( delayTime );
}

/****************************************************************************
 *
 * GPIO initialization
 *
 ****************************************************************************/

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

    /* Call driver initialize functions */
    GPIO_init();

    /* Enable the the ADC by setting RESET high */
    setRESET(HIGH);

    /* Set the interrupt callback function */
    GPIO_setCallback(DRDY_CONST,GPIO_DRDY_IRQHandler );
}

/****************************************************************************
 *
 * Interrupt handler for nDRDY GPIO
 *
 ****************************************************************************/

/**
 *
 * @brief GPIO_DRDY_IRQHandler()
 * Interrupt handler for /DRDY falling edge interrupt.
 *
 * @param[in] index Is the parameter value passed from the interrupt.
 *
 * @return none
 *
 */
void GPIO_DRDY_IRQHandler(uint_least8_t index)
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

/****************************************************************************
 *
 * GPIO helper functions
 *
 ****************************************************************************/

/**
 *
 * @brief set_RESET()
 * Controls the state of the nRESET GPIO pin.
 *
 * @param[in] state Is boolean indicating which state to set the nRESET pin (0=low, 1=high).
 *
 * NOTE: The 'HIGH' and 'LOW' macros defined in hal.h can be passed to this
 * function for the 'state' parameter value.
 *
 * @return none
 */
void setRESET(const bool state)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */
    GPIO_write(RESET_CONST, state);

}

/**
 *
 * @brief toggleRESET()
 * Toggles the "nRESET" pin to trigger a reset (LOW, delay 100 us, then HIGH).
 *
 * @return none
 */
void toggleRESET(void)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */
    GPIO_write(RESET_CONST, 0);
    delay_us(100);
    GPIO_write(RESET_CONST, 1);
}

/**
 *
 * @brief waitForDRDYinterrupt()
 * Waits for the nDRDY interrupt or until the specified timeout occurs.
 *
 * @param[in] timeout_ms Is the number of milliseconds to wait before timeout event.
 *
 * @return Returns 'true' if nDRDY interrupt occurred before the timeout.
 */
bool waitForDRDYinterrupt(const uint32_t timeout_ms)
{
    /* --- INSERT YOUR CODE HERE ---
     * Poll the nDRDY GPIO pin until it goes low. To avoid potential infinite
     * loops, you may also want to implement a timer interrupt to occur after
     * the specified timeout period, in case the nDRDY pin is not active.
     * Return a boolean to indicate if nDRDY went low or if a timeout occurred.
     */

    /* The following code is based on a TI Drivers implementation */

    // Convert ms to a # of loop iterations, OR even better use a timer here...

    uint32_t timeout = timeout_ms * 6000;   // convert to # of loop iterations

    // Reset interrupt flag
    flag_nDRDY_INTERRUPT = false;

    // Enable interrupts

    GPIO_clearInt(DRDY_CONST);
    SysCtlDelay(10);
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

/**
 *
 * @brief getDRDYstatus()
 * Provides the current status of the DRDY interrupt flag.
 *
 * @return status of the flag as a boolean value
 *
 */
bool getDRDYstatus(void)
{
    return flag_nDRDY_INTERRUPT;
}

/**
 *
 * @brief setDRDYstatus()
 * Sets/clears the DRDY interrupt flag.
 *
 * @param[in] DRDYflag Is the boolean value to set for the interrupt flag.
 *
 * @return none
 */
void setDRDYstatus(bool DRDYflag)
{
    if (DRDYflag) flag_nDRDY_INTERRUPT = false;
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
    //
    // Enabling I2C2 peripheral.
    //

    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */

    //
    // Initialize the optional I2C bus parameters
    //
    I2C_Params_init(&g_I2Cparams);
    g_I2Cparams.bitRate = I2C_1000kHz;

    //
    // Open the I2C bus for usage
    //
    g_I2Chandle = I2C_open(I2Cbus, &g_I2Cparams);

}

/**
 *
 * @brief I2C_SendReceive()
 * Transmits and receives data through the UART interface.
 *
 * @param[in] DataTX Is the pointer to an array of data to transmit.
 * @param[in] byteLengthTX Is the number of bytes of data to transmit.
 * @param[in] DataRX Is the pointer to an array of data to receive.
 * @param[in] byteLengthRX Is the number of bytes of data to receive.
 *
 * @return int8_t Any value not zero as a failure
 */
int8_t I2C_SendReceive(uint8_t *DataTX, uint8_t byteLengthTX, uint8_t *DataRX, uint8_t byteLengthRX)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */


    int8_t retStatus = 0;
    uint8_t i;
    uint8_t rData[8], wData[8];

    // Copy data to local array
    for(i = 0; i < byteLengthTX; i++)
    {
        wData[i] = DataTX[i];
    }

    //
    // Initialize the slave address for transactions
    //
    I2C_Transaction transaction = {0};
    transaction.slaveAddress = ADS122C04_ADDRESS;

    transaction.readBuf = rData ;
    transaction.readCount = byteLengthRX;
    transaction.writeBuf = wData;
    transaction.writeCount = byteLengthTX;
    retStatus = I2C_transfer(g_I2Chandle, &transaction);

    //
    // Return error, if read failed
    //
    if (retStatus == false)
    {
        /* --- INSERT YOUR ERROR PROCESSING CODE HERE --- */

        return -1;
    }

    for(i = 0; i < byteLengthRX; i++)
    {
        DataRX[i] = rData[i];
    }

        return 0;

}

