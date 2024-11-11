ADS131M08 Example C Code
=====================

The ADS131M0x family of simultaneously-sampling, 24-bit, delta-sigma (ΔΣ), analog-to-digital converters (ADC) communicate using a serial peripheral interface (SPI) to allow for device configuration, control, and data retrieval. To interface the ADS131M0x devices with a microcontroller (MCU), the firmware or software engineer needs to know how to correctly configure their MCU's serial peripheral, sequence the serial commands, and control the SPI timing of command bytes to the ADC. To assist in expediting this process, this example code is intended to show how to initialize and communicate with the ADS131M0x at a high-level.

Included modules
----------------

There are three modules included in this example code:

1.  **`ADS131M0x`**

	*Description:* High-level functions and register map definitions for communicating with the ADS131M0x.
	
	*Files: ads131m0x.h, ads131m0x.c*

2.  **`Hardware Abstraction Layer (HAL)`**

	*Description:* The HAL provides processor specific functions called by the `ADS131M0x` module.
	
	*Files: hal.h, hal.c*
	
	**IMPORTANT**: This module will need to be modified to work with your hardware!


How to use this code
--------------------

Reference the *ads131m0x.c* file while writing your own code for examples of how how to perform basic ADC operations, such as register read/writes and reading data...

OR

Copy and paste the example code into your project, and update the files as needed to get access to the provided APIs...

 1. Copy the `ADS131M0x` and `HAL` module files into your firmware project.
 2. Add library references in *hal.h* to your processor-specific library file(s).
	```c
	//****************************************************************************
	//
	// Insert processor specific header file(s) here
	//
	//****************************************************************************"
	
	/*  --- TODO: INSERT YOUR CODE HERE --- */
	#include "ti/devices/msp432e4/driverlib/driverlib.h"
	
	```

 3. Edit all of the function implementations inside of *hal.c* to provide the specified functionality with your processor and processor-specific library APIs. 
	```c
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
		* NOTE: The ADS131M0x operates in SPI mode 1 (CPOL = 0, CPHA = 1).
		*/
	```
	NOTE 1: The *hal.c* functions are called from within *ads131m0x.c* to interface with the hardware. The hardware functionality is kept in a separate module to allow the `ADS131M0x` module to remain portable.
	
	NOTE 2: Provided code examples in *hal.c* functions utilize the [TI SimpleLink SDK](http://www.ti.com/wireless-connectivity/simplelink-solutions/overview/software.html) and allow for quick integration with TI SimpleLink MCUs.
	
 4. Include a reference to *hal.h* somewhere in your program and call the **InitADC()** function to initialize the MCU peripherals connected to the ADC.

 5. Include a reference to *ads131m0x.h* in your application (from *main.c* or wherever ADC communication is handled). You should now be able to begin calling the `ADS131M0x` module functions in your code.

> **DISCLAIMER**: This code was tested on an MSP432E401Y 32-bit ARM® Cortex®-M4F based MCU using TI Code Composer Studio's 18.1.3.LTS ARM compiler. This code is provided as example to aid in the creation of your own software implementation and should not be considered to be fully verified and production ready. This example code was written for readability and has not been optimized for performance.

Support
-------

For questions or issues, visit the [TI E2E Forums](https://e2e.ti.com/).

![E2E Logo](http://e2e.ti.com/resized-image/__size/75x0/__key/CommunityServer-Wikis-Components-Files/00-00-00-01-27/2234.ti_2D00_e2e_2D00_Pos_2D00_no_2D00_text_2D00_150.jpg)

Release History
---------------
| Version     | Date        | Description            |
|:-----------:| ----------- | ---------------------- |
| 1.0.0       | 6/07/2019   | Initial release        |
| 1.0.1       | 6/19/2019   | Fixed bug in WLENGTH macro |
| 1.0.2       | 7/13/2020   | Corrected CRC polynomials |



![TI Logo](http://www.ti.com/assets/images/ic-logo.png)  
*Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/*