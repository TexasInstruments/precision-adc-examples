ADS7066 Example C Code
=====================

The ADS7066 is a small, 16-bit, 8-channel, highprecision successive-approximation register (SAR) analog-to-digital converter (ADC) that communicates using a serial peripheral interface (SPI) to allow for device configuration, control, and data retrieval. To interface the ADS7066 with a microcontroller (MCU), the firmware or software engineer needs to know how to correctly configure their MCU's serial peripheral, sequence the serial commands, and control the SPI timing of command bytes to the ADC. To assist in expediting this process, this example code is intended to show how to initialize and communicate with the ADC at a high-level.

Included modules
----------------

There are three modules included in this example code:

1.  **`ADS7066`**

	*Description:* High-level functions and register map definitions for communicating with the ADS7066.
	
	*Files: ads7066.h, ads7066.c*

2.  **`Hardware Abstraction Layer (HAL)`**

	*Description:* The HAL provides processor specific functions called by the `ADS7066` module.
	
	*Files: hal.h, hal.c*
	
	**IMPORTANT**: This module will need to be modified to work with your hardware!


How to use this code
--------------------

Reference the *ads7066.c* file while writing your own code for examples of how how to perform basic ADC operations, such as register read/writes and reading data...

OR

Copy and paste the example code into your project, and update the files as needed to get access to the provided APIs...

 1. Copy the `ADS7066` and `HAL` module files into your firmware project.
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
	//! Configures the MCU's SPI peripheral for interfacing with the ADC.
	//!
	//! \fn void InitSPI(void)
	//!
	//! \return None.
	//
	//*****************************************************************************
	void initSPI(void)
	{
		/* --- INSERT YOUR CODE HERE --- */
	```
	NOTE 1: The *hal.c* functions are called from within *ads7066.c* to interface with the hardware. The hardware functionality is kept in a separate module to allow the `ADS7066` module to remain portable.
	
	NOTE 2: Provided code examples in *hal.c* functions utilize the [TI SimpleLink SDK](http://www.ti.com/wireless-connectivity/simplelink-solutions/overview/software.html) and allow for quick integration with TI SimpleLink MCUs.
	
 4. Include a reference to *hal.h* somewhere in your program and call the **initAdcPeripherals()** function to initialize the MCU peripherals connected to the ADC.

 5. Include a reference to *ads7066.h* in your application (from *main.c* or wherever ADC communication is handled). You should now be able to begin calling the `ADS7066` module functions in your code.

> **DISCLAIMER**: This code was tested on an MSP432E401Y 32-bit ARM® Cortex®-M4F based MCU using TI Code Composer Studio's 20.2.2.LTS ARM compiler. This code is provided as example to aid in the creation of your own software implementation and should not be considered to be fully verified and production ready. This example code was written for readability and has not been optimized for performance.

Support
-------

For questions or issues, visit the [TI E2E Forums](https://e2e.ti.com/).

![E2E Logo](http://e2e.ti.com/resized-image/__size/75x0/__key/CommunityServer-Wikis-Components-Files/00-00-00-01-27/2234.ti_2D00_e2e_2D00_Pos_2D00_no_2D00_text_2D00_150.jpg)

Release History
---------------
| Version     | Date        | Description            |
|:-----------:| ----------- | ---------------------- |
| 1.0.0       | 10/23/2020  | Initial release        |