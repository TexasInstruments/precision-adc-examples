/**
 * @file hal.c
 *
 * @brief Example of a hardware abstraction layer
 * @warning This software utilizes TI Drivers
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
#include "ads124s08.h"

// SPI configuration
#define SPI_SPEED 2000000
#define SPI_WORD_SIZE 8
SPI_Handle g_SPIhandle;



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

/************************************************************************************//**
 * @brief getDRDYinterruptStatus()
 *          Gets the current status of nDRDY interrupt flag.
 *
 * @ return boolean status of flag_nDRDY_INTERRUPT
 */
bool getDRDYinterruptStatus(void)
{
   return flag_nDRDY_INTERRUPT;
}

/************************************************************************************//**
 * @brief setDRDYinterruptStatus()
 *          Sets the value of the nDRDY interrupt flag.
 *
 * @param[in] value where status is set with true; false clears the status.
 *
 * @return None
 */
void setDRDYinterruptStatus(const bool value)
{
    flag_nDRDY_INTERRUPT = value;
}
/************************************************************************************//**
 *
 * @brief enableDRDYinterrupt()
 *          Enables or disables the nDRDY interrupt.
 *
 * @param[in] intEnable Where interrupt is enabled with true; false disables the interrupt.
 *
 * @return None
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

//****************************************************************************
//
// Timing functions
//
//****************************************************************************

/************************************************************************************//**
 *
 * @brief delay_ms()
 *          Provides a timing delay with 'ms' resolution.
 *
 * @param[in] delay_time_ms Is the number of milliseconds to delay.
 *
 * @return none
 */
void delay_ms(const uint32_t delay_time_ms)
{
    /* --- INSERT YOUR CODE HERE --- */

    // Be careful of overflow
    uint32_t cycles_per_loop = 3;
    cycles_per_loop = getSysClockHz() / (cycles_per_loop * 1000u);
    MAP_SysCtlDelay( delay_time_ms * cycles_per_loop);
}

/************************************************************************************//**
 *
 * @brief delay_us()
 *          Provides a timing delay with 'us' resolution.
 *
 * @param[in] delay_time_us Is the number of microseconds to delay.
 *
 * @return none
 */
void delay_us(const uint32_t delay_time_us)
{
    /* --- INSERT YOUR CODE HERE --- */

    uint32_t cycles_per_loop = 3;

    // Be careful of overflow
    cycles_per_loop = getSysClockHz() / (cycles_per_loop * 1000000u);
    MAP_SysCtlDelay( delay_time_us * cycles_per_loop);
}

//****************************************************************************
//
// GPIO initialization
//
//****************************************************************************

/************************************************************************************//**
 *
 * @brief InitGPIO()
 *          Configures the MCU's GPIO pins that interface with the ADC.
 *
 * @return none
 *
 */
void InitGPIO(void)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */

    /* Call driver init functions */

    /* Set the interrupt callback function */
    GPIO_setCallback(DRDY_CONST,GPIO_DRDY_IRQHandler );
}

//*****************************************************************************
//
// Interrupt handler for nDRDY GPIO
//
//*****************************************************************************

/************************************************************************************//**
 *
 * @brief GPIO_DRDY_IRQHandler()
 *          Interrupt handler for nDRDY falling edge interrupt.
 *
 * @param[in] index Position of the interrupt for callback.
 *
 * @return None
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
// SPI Communication
//
//****************************************************************************

/************************************************************************************//**
 *
 * @brief InitSPI()
 *          Configures the MCU's SPI peripheral, for interfacing with the ADC.
 *
 * @return None
 */
void InitSPI(void)
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation for GPIO functions */

    /*
     * NOTE: The ADS124S08 operates in SPI mode 1 (CPOL = 0, CPHA = 1).
     */

    SPI_Params      spiParams;
    SPI_Params_init( &spiParams );
    spiParams.dataSize    = SPI_WORD_SIZE;
    spiParams.frameFormat = SPI_POL0_PHA1;
    spiParams.bitRate     = SPI_SPEED;
    g_SPIhandle = SPI_open( CONFIG_SPI_0, &spiParams );

    return;
}

/************************************************************************************//**
 *
 * @brief InitADCPeripherals()
 *          Initialize MCU peripherals and pins to interface with ADC
 *
 * @param[in]   *adcChars  ADC characteristics
 * @param[in]   *spiHdl    SPI_Handle pointer for TI Drivers
 *
 * @return      true for successful initialization
 *              false for unsuccessful initialization
 *
 * @code
 *     if ( !InitADCPeripherals( &spiHdl ) ) {
 *        // Error initializing MCU SPI Interface
 *        Display_printf( displayHdl, 0, 0, "Error initializing master SPI\n" );
 *        return( false );
 *     }
 *     // MCU initialized ADC successfully
 *     return( true );
 * @endcode
 */
bool InitADCPeripherals( SPI_Handle *spiHdl )
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */


    SPI_Params      spiParams;
    bool            status;

    SPI_Params_init( &spiParams );
    spiParams.dataSize    = SPI_WORD_SIZE;
    spiParams.frameFormat = SPI_POL0_PHA1;
    spiParams.bitRate     = SPI_SPEED;


    *spiHdl = SPI_open( CONFIG_SPI_0, &spiParams );
    if (*spiHdl == NULL) {
        // Display error initializing host SPI
        while (1);
    }
    else {
        // Display host SPI initialized successfully
    }

    // Start up the ADC
    status = adcStartupRoutine( *spiHdl );

    return( status );
}


/************************************************************************************//**
 *
 * @brief getRESET()
 *          Returns the state of the MCU's ADC_RESET GPIO pin
 *
 * @return boolean level of /RESET pin (false = low, true = high)
 */
bool getRESET( void )
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */
    return (bool) GPIO_read( RESET );
}

/************************************************************************************//**
 *
 * @brief setRESET()
 *            Sets the state of the MCU ADC_RESET GPIO pin
 *
 * @param[in]   state   level of /RESET pin (false = low, true = high)
 *
 * @return      None
 */

void setRESET( bool state )
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */
    GPIO_write( RESET, (unsigned int) state );
    return;
}

/************************************************************************************//**
 *
 * @brief toggleRESET()
 *            Pulses the /RESET GPIO pin low
 *
 * @return      None
 */
void toggleRESET( void )
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */
    GPIO_write( RESET, (unsigned int) LOW );

    // Minimum nRESET width: 4 tCLKs = 4 * 1/4.096MHz =
    delay_us( DELAY_4TCLK );

    GPIO_write( RESET, (unsigned int) HIGH );
    return;

}

/************************************************************************************//**
 *
 * @brief getSTART()
 *          Returns the state of the MCU's ADC_START GPIO pin
 *
 * @return boolean level of START pin (false = low, true = high)
 */
bool getSTART( void )
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */
    return (bool) GPIO_read( START );
}

/************************************************************************************//**
 *
 * @brief setSTART()
 *            Sets the state of the MCU START GPIO pin
 *
 * @param[in]   state   level of START pin (false = low, true = high)
 *
 * @return      None
 */
void setSTART( bool state )
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */
    GPIO_write( START, (unsigned int) state );

    // Minimum START width: 4 tCLKs
    delay_us( DELAY_4TCLK );

    return;
}

/************************************************************************************//**
 *
 * @brief toggleSTART()
 *            Pulses the START GPIO pin low
 * param[in]    direction sets the toggle direction base on initial START pin configuration
 *
 * @return      None
 */
void toggleSTART( bool direction )
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */
    if ( direction )
    {
        GPIO_write( START, (unsigned int) LOW );

        // Minimum START width: 4 tCLKs
        delay_us( DELAY_4TCLK );

        GPIO_write( START, (unsigned int) HIGH );
    }
    else
    {
        GPIO_write( START, (unsigned int) HIGH );

        // Minimum START width: 4 tCLKs
        delay_us( DELAY_4TCLK );

        GPIO_write( START, (unsigned int) LOW );
    }
    return;
}

/************************************************************************************//**
 *
 * @brief sendSTART()
 *            Sends START Command through SPI
 *
 * @param[in]   spiHdl    SPI_Handle pointer for TI Drivers
 *
 * @return      None
 */
void sendSTART( SPI_Handle spiHdl )
{
    uint8_t dataTx = OPCODE_START;

    // Send START Command
    sendCommand( spiHdl, dataTx );
    return;
}

/************************************************************************************//**
 *
 * @brief sendSTOP()
 *            Sends STOP Command through SPI
 *
 * @param[in]   spiHdl    SPI_Handle pointer for TI Drivers
 *
 * @return      None
 */
void sendSTOP( SPI_Handle spiHdl )
{
    uint8_t dataTx = OPCODE_STOP;

    // Send STOP Command
    sendCommand( spiHdl, dataTx );
    return;
}

/************************************************************************************//**
 *
 * @brief sendRESET()
 *            Sends RESET Command through SPI, then waits 4096 tCLKs
 *
 * @param[in]   spiHdl    SPI_Handle pointer for TI Drivers
 *
 * @return      None
 */
void sendRESET( SPI_Handle spiHdl )
{
    uint8_t dataTx = OPCODE_RESET;

    // Send RESET command
    sendCommand( spiHdl, dataTx );
    return;
}

/************************************************************************************//**
 *
 * @brief sendWakeup()
 *            Sends WAKEUP command through SPI
 *
 * @param[in]   spiHdl    SPI_Handle pointer for TI Drivers
 *
 * @return      None
 */
void sendWakeup( SPI_Handle spiHdl )
{
    uint8_t dataTx = OPCODE_WAKEUP;

    // Wakeup device
    sendCommand( spiHdl, dataTx );
    return;
}

/************************************************************************************//**
 *
 * @brief sendPowerdown()
 *            Sends POWERDOWN command through SPI
 *
 * @param[in]   spiHdl    SPI_Handle pointer for TI Drivers
 *
 * @return      None
 */
void sendPowerdown( SPI_Handle spiHdl )
{
    uint8_t dataTx = OPCODE_POWERDOWN;

    // Power down device
    sendCommand( spiHdl, dataTx );
    return;
}

/************************************************************************************//**
 *
 * @brief setCS()
 *            Sets the state of the "/CS" GPIO pin
 *
 * @param[in]   level   Sets the state of the "/CS" pin
 *
 * @return      None
 */
void setCS( bool state )
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */
    GPIO_write( CS, (unsigned int) state );
    return;
}

/************************************************************************************//**
 *
 * @brief getCS()
 *          Returns the state of the MCU's ADC_CS GPIO pin
 *
 * @return boolean level of CS pin (false = low, true = high)
 */
bool getCS( void )
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */
    return (bool) GPIO_read( CS );
}

/************************************************************************************//**
 *
 * @brief waitForDRDYHtoL()
 *            Waits for a nDRDY GPIO to go from High to Low or until a timeout condition occurs
 *            The DRDY output line is used as a status signal to indicate
 *            when a conversion has been completed. DRDY goes low
 *            when new data is available.
 *
 * @param[in]   timeout_ms number of milliseconds to allow until a timeout
 *
 * @return      Returns true if nDRDY interrupt occurred before the timeout
 *
 * @code
 *      // Read next conversion result
 *      if ( waitForDRDYHtoL( TIMEOUT_COUNTER ) ) {
 *          adcValue = readConvertedData( spiHdl, &status, COMMAND );
 *      } else {
 *          // Error reading conversion result
 *          Display_printf( displayHdl, 0, 0, "Timeout on conversion\n" );
 *          return( false );
 *      }
 * @endcode
 */
bool waitForDRDYHtoL( uint32_t timeout_ms )
{
    uint32_t timeoutCounter = timeout_ms * 8000;   // convert to # of loop iterations;

    do {
    } while ( !(flag_nDRDY_INTERRUPT) && (--timeoutCounter) );

    if ( !timeoutCounter ) {
        return false;
    } else {
        flag_nDRDY_INTERRUPT = false; // Reset flag
        return true;
    }
}

/************************************************************************************//**
 *
 * @brief spiSendReceiveArrays()
 *             Sends SPI commands to ADC and returns a response in array format
 *
 * @param[in]   spiHdl      SPI_Handle from TI Drivers
 * @param[in]   *DataTx     array of SPI data to send on MOSI pin
 * @param[in]   *DataRx     array of SPI data that will be received from MISO pin
 * @param[in]   byteLength  number of bytes to send/receive on the SPI
 *
 * @return     None
 */
void spiSendReceiveArrays( SPI_Handle spiHdl, uint8_t DataTx[], uint8_t DataRx[], uint8_t byteLength )
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */

    SPI_Transaction spiTransaction;
    bool            transferOK;

    /*
     *  This function sends and receives multiple bytes over the SPI.
     *
     *  A typical SPI send/receive sequence may look like the following:
     *  1) Make sure SPI receive buffer is empty
     *  2) Set the /CS pin low (if controlled by GPIO)
     *  3) Send command bytes to SPI transmit buffer
     *  4) Wait for SPI receive interrupt
     *  5) Retrieve data from SPI receive buffer
     *  6) Set the /CS pin high (if controlled by GPIO)
     *
     */

    spiTransaction.txBuf = DataTx;
    spiTransaction.rxBuf = DataRx;


    setCS( LOW );

    /* Send or Receive Data */
    if ( byteLength > 0 ) {
        spiTransaction.count = byteLength;
        transferOK = SPI_transfer( spiHdl, &spiTransaction);
        if (transferOK == NULL) {
            // Display and handle error with SPI data transfer
            while (1);
        }
    }

   setCS( HIGH );
   return;
}


/************************************************************************************//**
 *
 * @brief spiSendReceiveByte()
 *             Sends a single byte to ADC and returns a response
 *
 * @param[in]   spiHdl      SPI_Handle from TI Drivers
 * @param[in]   dataTx      byte to send on DIN pin
 *
 * @return     SPI response byte
 */
uint8_t spiSendReceiveByte( SPI_Handle spiHdl, uint8_t dataTx )
{
    /* --- INSERT YOUR CODE HERE --- */

    /* The following code is based on a TI Drivers implementation */

    uint8_t         dataRx = 0;
    SPI_Transaction spiTransaction;
    bool            transferOK;

    spiTransaction.count = 1;
    spiTransaction.txBuf = (void *) &dataTx;
    spiTransaction.rxBuf = (void *) &dataRx;


    setCS( LOW );

    transferOK = SPI_transfer( spiHdl, &spiTransaction);

    setCS( HIGH );

    if (transferOK == NULL) {
        // Display and handle error with SPI data transfer
        while (1);
    }
    return( dataRx );
}
