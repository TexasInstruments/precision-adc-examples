![IC Image](https://www.ti.com/graphics/folders/partimages/ADS127L11.jpg)ADS127L11 Example C Code
=====================

The ADS127L11 is a 24-bit delta-sigma (ΔΣ), analog-to-digital converter (ADC) which communicates using a serial peripheral interface (SPI) to allow for device configuration, control, and data retrieval. To interface the ADS127L11 with a microcontroller (MCU), the firmware or software engineer needs to know how to correctly configure their MCU's serial peripheral, sequence the serial commands, and control the SPI timing of command bytes to the ADC. To assist in expediting this process, this example code is intended to show how to initialize and communicate with the ADS127L11 at a high-level.

Included modules
----------------

There are three modules included in this example code:

1.  **`ADS127L11`**

	*Description:* High-level functions and register map definitions for communicating with the ADS127L11.
	
	*Files: ads127l11.h, ads127l11.c*

2.  **`Hardware Abstraction Layer (HAL)`**

	*Description:* The HAL provides processor specific functions called by the `ADS127L11` module.
	
	*Files: hal.h, hal.c*
	
	**IMPORTANT**: This module will need to be modified to work with your hardware!
	
2.  **`CRC Calculation`**

	*Description:* The CRC calculation module provides CRC related functions called by the `ADS127L11` module.
	
	*Files: crc.h, crc.c*


How to use this code
--------------------

Reference the *ads127l11.c* file while writing your own code for examples of how how to perform basic ADC operations, such as register read/writes and reading data...

OR

Copy and paste the example code into your project, and update the files as needed to get access to the provided APIs...

 1. Copy the `ADS127L11` and `HAL` module files into your firmware project.
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
		* NOTE: The ADS127L11 operates in SPI mode 1 (CPOL = 0, CPHA = 1).
		*/
	```
	NOTE 1: The *hal.c* functions are called from within *ads127l11.c* to interface with the hardware. The hardware functionality is kept in a separate module to allow the `ADS127L11` module to remain portable.
	
	NOTE 2: Provided code examples in *hal.c* functions utilize the [TI SimpleLink SDK](http://www.ti.com/wireless-connectivity/simplelink-solutions/overview/software.html) and allow for quick integration with TI SimpleLink MCUs.
	
 4. Include a reference to *hal.h* somewhere in your program and call the **InitADC()** function to initialize the MCU peripherals connected to the ADC.

 5. Include a reference to *ads127l11.h* in your application (from *main.c* or wherever ADC communication is handled). You should now be able to begin calling the `ADS127L11` module functions in your code.
 
 5. Configure definitions in *ads127l11.h* to match your desired `ADS127L11` operating modes.
	```c
	//****************************************************************************
	//
	// Select the SPI mode and desired CONFIG4 register settings...
	// NOTE: These settings will be enforced and not modifiable during runtime!
	//
	//****************************************************************************

	/* Enable this to use 3 wire SPI mode*/
	//#define SPI_3_WIRE

	/* Enable this to use 24-bit resolution mode*/
	#define RES_24_BIT_MODE

	/* Enable this define statement to enable SPI_CRC */
	//#define SPI_CRC_MODE

	/* Enable this define statement to enable SPI STATUS bytes */
	//#define STATUS_BYTE_MODE
	```
	NOTE 1: 3 WIRE SPI mode is not currently supported in this release of example code.

> **DISCLAIMER**: This code was tested on an MSP432E401Y 32-bit ARM® Cortex®-M4F based MCU using TI Code Composer Studio's 20.2.5.LTS ARM compiler. This code is provided as example to aid in the creation of your own software implementation and should not be considered to be fully verified and production ready. This example code was written for readability and has not been optimized for performance.

Support
-------

For questions or issues, visit the [TI E2E Forums](https://e2e.ti.com/).

![E2E Logo](http://e2e.ti.com/resized-image/__size/75x0/__key/CommunityServer-Wikis-Components-Files/00-00-00-01-27/2234.ti_2D00_e2e_2D00_Pos_2D00_no_2D00_text_2D00_150.jpg)

Release History
---------------
| Version     | Date        | Description            |
|:-----------:| ----------- | ---------------------- |
| 1.0.0       | 11/18/2021  | Initial release        |
