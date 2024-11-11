ADS1258 Example C Code
=====================

The [ADS1258](http://www.ti.com/product/ADS1258) precision delta-sigma (ΔΣ) analog-to-digital converter (ADC) communicates using a serial peripheral interface (SPI) to allow for device configuration, control, and data retrieval. To interface with the ADS1258 with a microcontroller (MCU), the firmware or software engineer needs to know how to correctly configure their MCU's serial peripheral, sequence serial commands, and control the SPI timing of command bytes to the ADC. To assist in this process, this example code is intended to show how to initialize communication with the ADS1258 at a high-level.

Included modules
----------------

There are three modules included in this example code:

1.  **`ADS1258`**

	*Description:* Low-level functions and register map definitions for communicating with the ADS1258.
	
	*Files: ads1258.h, ads1258.c*

2.  **`Hardware Abstraction Layer (HAL)`**

	*Description:* The HAL provides processor specific functions called by the `ADS1258` module.
	
	*Files: hal.h, hal.c*
	
	***IMPORTANT**: This module will need to be modified to work with your hardware!* 

3. **`Unit Tests`**

	*Description:* This is an optional module that you may include in your project during development to test example code functionality.

	*Files: unit_tests.h, unit_tests.c*

How to use this code
--------------------

Reference the *ads1258.c* file while writing your own code for examples of how how to perform basic ADC operations, such as register read/writes and reading data...

OR 

Copy and paste the example code into your project, and update the files as needed to get access to the provided APIs...

 1. Copy the `ADS1258` and `HAL` module files into your firmware project. Optionally, copy the `Unit Tests` module into your project as well.

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
	/**
	 * \fn void InitSPI(void)
	 * \brief Configures the MCU's SPI peripheral for interfacing with the ADS1258
	 */
	void InitSPI(void)
	{
	    /* TODO: INSERT YOUR CODE HERE */
	}
	```
	NOTE 1: The *hal.c* functions are called from within *ads1258.c* to interface with the hardware. The hardware functionality is kept in a separate module to allow the `ADS1258` module to remain portable.
	
	NOTE 2: Provided code examples in *hal.c* functions utilize the [TI SimpleLink SDK](http://www.ti.com/wireless-connectivity/simplelink-solutions/overview/software.html).
	
 4. Include a reference to *hal.h* somewhere in your program and call the **InitADCPeripherals()** function to initialize the MCU peripherals connected to the ADS1258.

 5. Include a reference to *ads1261.h* in your application (from *main.c* or wherever ADC communication is handled). You should now be able to begin calling the `ADS1258` module functions in your code.

> **IMPORTANT**: This code was tested on an MSP432E401Y 32-bit ARM® Cortex®-M4F based MCU using TI Code Composer Studio's 18.1.3.LTS ARM compiler (using the provided `Unit Tests` functions). This code is provided as example to aid in the creation of your own software implementation and should not be considered to be fully verified and end-equipment ready. This example code was written for readability and has not been optimized for performance.

Support
-------

For questions or issues, visit the [TI E2E Forums](https://e2e.ti.com/).

![E2E Logo](http://e2e.ti.com/resized-image/__size/75x0/__key/CommunityServer-Wikis-Components-Files/00-00-00-01-27/2234.ti_2D00_e2e_2D00_Pos_2D00_no_2D00_text_2D00_150.jpg)


Release History
---------------
`v1.0.0` [11-7-2018] - Initial release
