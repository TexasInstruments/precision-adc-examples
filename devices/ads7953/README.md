ADS79xx Example C Code
=====================

The ADS79xx example code can be used for all devices in the family including 12/10/8-bit, and 4/8/12/16-channel variants.  To assist in expediting the code development process, this example code provides functions that abstract device functionality, and an example script showcasing use of the functions.

Included modules
----------------

There are three modules included in this example code:

1.  **`ADS79xx_main`**

	*Description:* Top-level example script file calling APIs provided by `ADS79xx` module. Showcases initialization, GPIO configuration, alarm configuration, and operation in Manual, Auto-1, and Auto-2 modes.
	
	*Files: ads79xx_main.c*

2.  **`ADS79xx`**

	*Description:* Provides device operation/configuration APIs, data types, and device selection/configuration definitions.
	
	*Files: ads79xx.h, ads79xx.c*

2.  **`Hardware Abstraction Layer (HAL)`**

	*Description:* The HAL provides processor specific functions called by the `ADS79xx` module.
	
	*Files: hal.h, hal.c*
	
	**IMPORTANT**: This module will need to be modified to work with your hardware!

LP-MSPM0G3507 to ADS7953EVM connections
---------------------------------------
| LP-MSPM0G3507 Pin | Signal | ADS7953EVM Pin |
|-------------------|--------|----------------|
| J1[4]             | ~CS    |          J4[7] |
| J2[33]            | SCLK   |          J5[1] |
| J2[21]            | MOSI   |          J4[5] |
| J2[34]            | MISO   |          J5[3] |

How to use this code
--------------------

Reference the *ads79xx-main.c* file while writing your own code for examples of how how to perform basic ADC operations, such as register read/writes and reading data...

OR

Copy and paste the example code into your project, and update the files as needed to get access to the provided APIs...

 1. Copy the `ADS79xx` and `HAL` module files into your firmware project.
 2. Add library references in *hal.h* to your processor-specific library file(s).
	```c
	//****************************************************************************
	//
	// Insert processor specific header file(s) here
	//
	//****************************************************************************"
	
	/*  --- TODO: INSERT YOUR CODE HERE --- */
	#include "#include "ti_msp_dl_config.h"
	
	```

 3. Edit all of the function implementations inside of *hal.c* to provide the specified functionality with your processor and processor-specific library APIs. 
	```c
    /**
     *
     * @brief Send and receive data arrays via SPI
     * 
     * @param dataTx Array of bytes to transmit
     * @param dataRx Array to store received bytes
     * @param bufferLength Number of bytes to transfer
     * @return none
     */
    spiSendReceiveArrays(const uint16_t dataTx[], uint16_t dataRx[], const uint8_t bufferLength);
    {
    /* --- INSERT YOUR CODE HERE --- */
        //
        // Loop through Tx/Rx arrays, send/receive data
        //
    }


	```
	NOTE 1: The *hal.c* functions are called from within *ads79xx.c* to interface with the hardware. The hardware functionality is kept in a separate module to allow the `ADS79xx` module to remain portable.
	
	NOTE 2: Provided code examples in *hal.c* functions utilize the [TI MSPM0 SDK](https://www.ti.com/tool/MSPM0-SDK) and allow for quick integration with TI MSPM0 MCUs.

 4. Include a reference to *ads79xx.h* in your application (from *main.c* or wherever ADC communication is handled). You should now be able to begin calling the `ADS79xx` module functions in your code.

> **DISCLAIMER**: This code was tested on the LP-MSPM0G3507, a LaunchPad development kit for the MSPM0G3507 32-bit ARM® Cortex®-M0+ based MCU, using TI CCS Theia <use current 1.5.1 version> ARM compiler, along with the ADS7953EVM evaluation module. This code is provided as example to aid in the creation of your own software implementation and should not be considered to be fully verified and production ready. This example code was written for readability and has not been optimized for performance.

Order Hardware
--------------

[ADS7953](https://www.ti.com/product/ADS7953#order-quality)

[ADS7953EVM-PDK](https://www.ti.com/tool/ADS7953EVM-PDK#order-start-development)

[LP_MSPM0G3507 LaunchPad](https://www.ti.com/tool/LP-MSPM0G3507)

Support
-------

For questions or issues, visit the [TI E2E Forums](https://e2e.ti.com/).



Release History
---------------
| Version     | Date        | Description            |
|:-----------:| ----------- | ---------------------- |
| 1.0.0       | 05/21/2025  | Initial release        |