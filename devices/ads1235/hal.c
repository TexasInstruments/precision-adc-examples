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

/**
 * \fn bool pollForDRDY(uint32_t timeout_ms)
 * \brief Polls the nDRDY pin until it goes low or or a timeout condition occurs
 * \param timeout_ms number of milliseconds to allow until a timeout
 * \return returns true if nDRDY pin went low before the timeout occurred
 */
bool pollForDRDY(uint32_t timeout_ms)
{
    /* --- INSERT YOUR CODE HERE ---
     * Poll the nDRDY GPIO pin until it goes low. To avoid potential infinite
     * loops, you may also want to implement a timer interrupt to occur after
     * the specified timeout period, in case the nDRDY pin is not active.
     * Return a boolean to indicate if nDRDY went low or if a timeout occurred.
     */

}


/**
 * \fn void delay_ms(uint32_t time_ms)
 * \brief Delays for a number of milliseconds, specified by argument
 * \param time_ms number of milliseconds to delay
 */
void delay_ms(uint32_t time_ms)
{
    /* --- INSERT YOUR CODE HERE ---
     * Delay for a number of milliseconds, as specified by "time_ms".
     *
     * The following code shows an example using TivaWare™...
     * NOTE: In this example 40,000 system ticks is 1 ms.
     */

    uint32_t ticks = 40000 * time_ms;
	SysCtlDelay(ticks);
}


/**
 * \fn void setCS(bool state)
 * \brief Sets nCS pin state and delays for ~50 ns
 * \param state boolean to control pin output logic
 */
void setCS(bool state)
{
    /* --- INSERT YOUR CODE HERE ---
     * This function sets the "nCS" GPIO pin state,
     * according to the input argument, and delays for 50 ns.
     *
     * The following code shows an example using TivaWare™...
     */

    if (state)
	{
        /* Sets the nCS pin HIGH */
        GPIOPinWrite(nCS_PORT, nCS_PIN, nCS_PIN);
	}
	else
	{
	    /* Sets the nCS pin LOW */
		GPIOPinWrite(nCS_PORT, nCS_PIN, 0);
	}

    /* td(SCCS)/td(CSSC) delay */
    SysCtlDelay(2);

}


/**
 * \fn void setPWDN(bool state)
 * \brief Sets nPWDN pin state and delays for ~1 ms
 * \param state boolean to control pin output logic
 */
void setPWDN(bool state)
{
    /* --- INSERT YOUR CODE HERE ---
     * This function sets the "nPWDN" GPIO pin state
     * according to the input argument, and delays for 1 ms.
     *
     * The following code shows an example using TivaWare™...
     */

    if (state)
	{
		/* Set nPWDN pin HIGH */
		GPIOPinWrite(PWDN_PORT, PWDN_PIN, PWDN_PIN);

	   /* If using an external clock, enable the clock here
		* and allow for at 2 clock periods before communicating with the device
		*/

        /* Delay to allow internal oscillator to settle */
        delay_ms(1);
	}
	else
	{
	    /* Set nPWDN pin LOW */
		GPIOPinWrite(PWDN_PORT, PWDN_PIN, 0);

		/* NOTE: SPI communication will be ignored in HW power-down mode! */
	}
}


/**
 * \fn void setRESET(bool state)
 * \brief Sets nRESET pin high during startup
 * \param state boolean argument must be TRUE or assertion will fail.
 */
void setRESET(void)
{
    /* --- INSERT YOUR CODE HERE ---
     * This function sets the "nRESET" GPIO pin high,
     * and delays for 4 fclk periods.
     *
     * The following code shows an example using TivaWare™...
     */

    GPIOPinWrite(RESET_PORT, RESET_PIN, RESET_PIN);
    SysCtlDelay(25);
}


/**
 * \fn void setSTART(bool state)
 * \brief Set the START pin logic level according to the input argument
 * \param state boolean to control pin output logic
 */
void setSTART(bool state)
{
   /* --- INSERT YOUR CODE HERE ---
    *
    * This function sets the "START" GPIO pin logic level according
    * to the input argument and delays for 4 fclk periods.
    *
    * The following code shows an example using TivaWare™...
    */

    if (state)
	{
		/* Set START pin HIGH */
		GPIOPinWrite(START_PORT, START_PIN, START_PIN);
	}
	else
	{
		/* Set START pin LOW */
		GPIOPinWrite(START_PORT, START_PIN, 0);
	}

	/* tw(STx) delay: 4 fclk periods */
	SysCtlDelay(25);
}


/**
 * \fn void toggleRESET()
 * \brief Toggles the RESET pin ( LOW -> HIGH ) to reset the device
 */
void toggleRESET(void)
{
   /* --- INSERT YOUR CODE HERE ---
    *
    * This function should toggle the /RESET GPIO pin using the sequence:
    * 1) Set the RESET pin low.
    * 2) Allow for tw(RSTL) delay.
    * 3) Set the RESET pin high.
    * 4) Delay (before communicating with the device)
    * 5) Restore the global register array to default values
    *
    * The following code shows an example using TivaWare™...
    */

    GPIOPinWrite(RESET_PORT, RESET_PIN, 0);
	SysCtlDelay(25);
	GPIOPinWrite(RESET_PORT, RESET_PIN, RESET_PIN);
	SysCtlDelay(25);

	/* To keep SW in sync with device, restore default settings.
	 * The reset occurs even when the device is in hardware power-down */
	restoreRegisterDefaults();
}


/**
 * \fn void toggleSTART()
 * \brief Toggles the START pin (LOW -> HIGH) to restart ADC conversions
 */
void toggleSTART(void)
{
   /* --- INSERT YOUR CODE HERE ---
    *
    *  This function should toggle the START GPIO pin using the sequence:
    *  1) Set the START pin low.
    *  2) Allow for tw(STL) delay.
    *  3) Set the START pin high.
    *  4) Allow for tw(STH) delay.
    *
    * The following code shows an example using TivaWare™...
    */

	GPIOPinWrite(START_PORT, START_PIN, 0);
	SysCtlDelay(25);
	GPIOPinWrite(START_PORT, START_PIN, START_PIN);
	SysCtlDelay(25);
}


/**
 * \fn void SPI_SendReceive(uint8_t *DataTx, uint8_t *DataRx, uint8_t byteLength)
 * \brief Handles SPI errors
 * \param *DataTx array of SPI data to send to DIN pin
 * \param *DataRx array of SPI data that will be received on DOUT pin
 * \param byteLength number of bytes to send/receive on the SPI
 */
void SPI_SendReceive(uint8_t *DataTx, uint8_t *DataRx, uint8_t byteLength)
{
   /*	--- INSERT YOUR CODE HERE ---
	*
	* 	This function should send and receive multiple bytes over the SPI.
	*
	* 	A typical SPI send/receive sequence may look like the following:
	* 	1) Make sure SPI receive buffer is empty
	* 	2) Set the /CS pin low (if controlled by GPIO)
	* 	3) Send command bytes to SPI transmit buffer
	* 	4) Wait for SPI receive interrupt
	* 	5) Retrieve data from SPI receive buffer
	* 	6) Set the /CS pin high (if controlled by GPIO)
	*
	*/

	/* Set the nCS pin LOW */
	setCS(LOW);

	/* Remove any residual or old data from the receive FIFO */
	uint32_t junk;
	while (SSIDataGetNonBlocking(SSI0_BASE, &junk));

	/* SSI TX & RX */
	int i;
	for (i = 0; i < byteLength; i++)
	{
		/* Set up data for the next transmit */
		HWREG(SSI0_BASE + SSI_O_DR) = DataTx[i];

		/* Wait for data to appear */
		while (!(HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_RNE));

		/* Grab the return data */
		DataRx[i] = HWREG(SSI0_BASE + SSI_O_DR);
	}

	/* Set the nCS pin HIGH */
	setCS(HIGH);
}
/**
 * \fn void handleSPIerror(uint8_t *DataTx, uint8_t *DataRx, uint8_t byteLength, char *command)
 * \brief Handles SPI errors
 * \param *DataTx array of the SPI data transmitted to DIN pin
 * \param *DataRx array of the SPI data received on DOUT pin
 * \param byteLength number of bytes sent/received on the SPI
 * \param *command name of SPI command being sent when the error occurred
 */
void handleSPIerror(uint8_t *DataTx, uint8_t *DataRx, uint8_t byteLength, char *command)
{
   /* --- INSERT YOUR CODE HERE ---
	*
	* This function gets called after a SPI communication error is detected,
	* and should be programmed to gracefully handle the error.
	*
	* How a communication error should get handled will largely depend on
	* personal preference and other system requirements; however, here are
	* some suggestions on how you may want to handle it...
	*
	* 1) Try to re-send the command. In case of a glitch on the SPI,
	* re-sending the command may be all that is needed to correct the issue.
	* To correct for the possibility of SPI communication being out of sync
	* with SCLK, toggle the /CS pin or wait for the SPI auto-reset timeout
	* of the device (if enabled) before trying to re-sending the last command.
	*
	* 2) If the device was previously using CRC mode, try re-sending the
	* command without the required CRC bytes. In case of a brown-out or
	* device reset, the CRC mode many no longer be enabled and the error
	* may be a result of the software trying to communicate in CRC mode.
	*
	* 3) Perform any relevant system self-diagnostic tests or operations.
	* For example, try to read back the ADS1235 register settings, or
	* try resetting/reinitializing the system to restore proper operation.
	*
	* 4) Report or log the error and defer to the end-user for any necessary
	* corrective actions.
	*/

}
