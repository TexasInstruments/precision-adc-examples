![IC Image](http://www.ti.com/graphics/folders/partimages/ADS125H02.jpg)ADS125H02 Example C Code
=====================

The [ADS125H02](http://www.ti.com/product/ADS125H02) precision delta-sigma (ΔΣ) analog-to-digital converter (ADC) communicates using a serial peripheral interface (SPI) to allow for device configuration, control, and data retrieval. To interface with the ADS125H02 with a microcontroller (MCU), the firmware or software engineer needs to know how to correctly configure their MCU's serial peripheral, sequence serial commands, and control the SPI timing of command bytes to the ADC. To assist in this process, this example code is intended to show how to initialize communication with the ADS125H02 at a high-level.

Included modules
----------------

There are two modules included in this example code:

1.  **`ADS125H02`**

	*Description:* Low-level functions and register map definitions for communicating with the ADS125H02.
	
	*Files: ads125h02.h, ads125h02.c*

2.  **`Hardware Abstraction Layer (HAL)`**

	*Description:* The HAL provides processor specific functions called by the `ADS125H02` module.
	
	*Files: hal.h, hal.c*
	
	***IMPORTANT**: This module will need to be modified to work with your hardware!* 


How to use this code
--------------------

Reference the *ads125h02.c* file while writing your own code for examples of how how to perform basic ADC operations, such as register read/writes and reading data...

OR 

Copy and paste the example code into your project, and update the files as needed to get access to the provided APIs...

 1. Copy the `ADS125H02` and `HAL` module files into your firmware project.

 2. Add library references in *hal.h* to your processor-specific library file(s).
	```c
	//****************************************************************************
	//
	// Insert processor specific header file(s) here
	//
	//****************************************************************************"
	
	/*  --- TODO: INSERT YOUR CODE HERE --- */
	#include "driverlib/sysctl.h"
	#include "driverlib/gpio.h"
	#include "driverlib/ssi.h"
	```

 3. Edit all of the function implementations inside of *hal.c* to provide the specified functionality with your processor and processor-specific library APIs. 
	```c
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
	```
	NOTE 1: The *hal.c* functions are called from within *ads125h02.c* to interface with the hardware. The hardware functionality is kept in a separate module to allow the `ADS125H02` module to remain portable.
	
	NOTE 2: Provided code examples in *hal.c* functions utilize the [TI TivaWare™ for C Series](http://www.ti.com/tool/SW-TM4C) code library.
	
 4. Configure an initialize the MCU peripherals connected to the ADS125H02.
 
 5. Include a reference to *ads125h02.h* in your application (from *main.c* or wherever ADC communication is handled). You should now be able to begin calling the `ADS125H02` module functions in your code.

> **IMPORTANT**: This code was tested on an TM4C1294NCPDT 32-bit ARM® Cortex®-M4F based MCU using TI Code Composer Studio's 18.9.0.LTS ARM compiler. This code is provided as example to aid in the creation of your own software implementation and should not be considered to be fully verified and end-equipment ready. This example code was written for readability and has not been optimized for performance.

Support
-------

For questions or issues, visit the [TI E2E Forums](https://e2e.ti.com/).

![E2E Logo](http://e2e.ti.com/resized-image/__size/75x0/__key/CommunityServer-Wikis-Components-Files/00-00-00-01-27/2234.ti_2D00_e2e_2D00_Pos_2D00_no_2D00_text_2D00_150.jpg)

Release History
---------------

`v1.0.0` [1-11-2019] - Initial release
`v1.1.0` [1-11-2019] - Bug fix in calculateCRC() function.
