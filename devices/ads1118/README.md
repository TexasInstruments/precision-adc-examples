![IC Image](http://www.ti.com/graphics/folders/partimages/ADS1118.jpg)ADS1118 Example C Code
=====================

The ADS1118 code can be adapted for use with the ADS1018 (12-bit version). This example code is intended to show how to initialize and communicate with the ADS1118 at a high-level.

Included modules
----------------

There are two modules included in this example code:

1.  **`ADS1118`**

	*Description:* High-level functions and register map definitions for communicating with the ADS1118.
	
	*Files: ads1118.h, ads1118.c*

2.  **`Hardware Abstraction Layer (HAL)`**

	*Description:* The HAL provides processor specific functions called by the `ADS1118` module.
	
	*Files: hal.h, hal.c*
	
	**IMPORTANT**: This module will need to be modified to work with your hardware!


How to use this code
--------------------

Reference the *ads1118.c* file while writing your own code for examples of how how to perform basic ADC operations, such as register read/writes and reading data...

OR

Copy and paste the example code into your project, and update the files as needed to get access to the provided APIs...

 1. Copy the `ADS1118` and `HAL` module files into your firmware project.

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
		* NOTE: The ADSxxxx operates in SPI mode 1 (CPOL = 0, CPHA = 1).
		*/
	```
	NOTE 1: The *hal.c* functions are called from within *ads1118.c* to interface with the hardware. The hardware functionality is kept in a separate module to allow the `ADS1118` module to remain portable.
	
	NOTE 2: Provided code examples in *hal.c* functions utilize the [TI SimpleLink SDK](http://www.ti.com/wireless-connectivity/simplelink-solutions/overview/software.html) and allow for quick integration with TI SimpleLink MCUs.
	
 4. Include a reference to *hal.h* somewhere in your program and call the **InitADC()** function to initialize the MCU peripherals connected to the ADC.

 5. Include a reference to *ads1118.h* in your application (from *main.c* or wherever ADC communication is handled). You should now be able to begin calling the `ADS1118` module functions in your code.

> **DISCLAIMER**: This code was tested on an MSP432E401Y 32-bit ARM® Cortex®-M4F based MCU using TI Code Composer Studio's <use current 18.1.3.LTS version> ARM compiler. This code is provided as example to aid in the creation of your own software implementation and should not be considered to be fully verified and production ready. This example code was written for readability and has not been optimized for performance.

Support
-------

For questions or issues, visit the [TI E2E Forums](https://e2e.ti.com/).


Release History
---------------
| Version     | Date        | Description            |
|:-----------:| ----------- | ---------------------- |
| 1.0.0       | 04/28/2021  | Initial release        |
