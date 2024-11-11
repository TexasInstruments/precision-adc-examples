ADS122U04 Example C Code
=====================

The [ADS122U04](http://www.ti.com/product/ADS122U04) precision delta-sigma (ΔΣ) analog-to-digital converter (ADC) communicates using an UART peripheral interface to allow for device configuration, control, and data retrieval. To interface the ADS122U04 with a microcontroller (MCU), the firmware or software engineer needs to know how to correctly configure their MCU's serial peripheral, sequence commands, and control the UART timing of command bytes to the ADC. To assist in this process, this example code is intended to show how to initialize communication with the ADS122U04 at a high-level.

Included modules
----------------

There are three modules included in this example code:

1.  **`ADS122U04`**

	*Description:* Low-level functions and register map definitions for communicating with the ADS122U04. The functions also apply to the ADS112U04 if the define for ADS122U04 is removed in `ads122U04.h`.
	
	*Files: ads122u04.h, ads122u04.c*

2.  **`Hardware Abstraction Layer (HAL)`**

	*Description:* The HAL provides processor specific functions called by the `ADS122U04` module.
	
	*Files: hal.h, hal.c*
	
	***IMPORTANT**: This module will need to be modified to work with your hardware!*
	
3.  **`Data Integrity Functions`** 

    *Description:* The Data Integrity Functions allow calculation and verification of any CRC data integrity modes enabled for register and conversion data reads. This module is only required if the CRC data integrity mode is enabled on the `ADS122U04`.
    
    *Files: crc.h, crc.c*


How to use this code
--------------------

Reference the *ads122u04.c* file while writing your own code for examples of how how to perform basic ADC operations, such as register read/writes and reading data...

OR 

Copy and paste the example code into your project, and update the files as needed to get access to the provided APIs...

 1. Copy the `ADS122U04`, `HAL` and 'Data Integrity' module files into your firmware project. 

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
	 * \fn void InitUART(void)
	 * \brief Configures the MCU's UART peripheral for interfacing with the ADS122U04
	 */
	void InitUART(void)
	{
	    /* TODO: INSERT YOUR CODE HERE */
	}
	```
	NOTE 1: The *hal.c* and *crc.c* functions are called from within *ads122u04.c* to interface with the hardware. The hardware functionality is kept in a separate module to allow the `ADS122U04` module to remain portable.
	
	NOTE 2: Provided code examples in *hal.c* functions utilize the [TI SimpleLink SDK](http://www.ti.com/wireless-connectivity/simplelink-solutions/overview/software.html).
	
 4. Include a reference to *hal.h* somewhere in your program and call the **InitADCPeripherals()** function to initialize the MCU peripherals connected to the `ADS122U04`.

 5. Include a reference to *ads122u04.h* in your application (from *main.c* or wherever ADC communication is handled). You should now be able to begin calling the `ADS122U04` module functions in your code.

> **IMPORTANT**: This code was tested on an MSP432E401Y 32-bit ARM® Cortex®-M4F based MCU using TI Code Composer Studio's 20.2.1.LTS ARM compiler. This code is provided as example to aid in the creation of your own software implementation and should not be considered to be fully verified and end-equipment ready. This example code was written for readability and has not been optimized for performance.

Support
-------

For questions or issues, visit the [TI E2E Forums](https://e2e.ti.com/).



Release History
---------------
`v1.0.0` [11-15-2021] - Initial release

