# ADS124S08 Example C Code {#mainpage}

[TOC]

![](chip.jpg) <br>
The [ADS124S08](https://www.ti.com/product/ADS124S08) precision delta-sigma (ΔΣ) analog-to-digital converter (ADC) communicates using a serial peripheral interface (SPI) to allow for device configuration, control, and data retrieval. To interface the ADC with a microcontroller (MCU), the firmware or software engineer needs to know how to correctly configure their MCU's serial peripheral, sequence serial commands, and control the SPI timing of command bytes to the ADC. To assist in this process, this example code is intended to show how to initialize communication with the `ADS124S08` at a high-level.

# Modules {#modules}

There are three modules included in this example code:

1.  **ADS124S08**
	*Files:* ads124S08.h, ads124S08.c <br>
	*Description:* Contains header and source files with register map definitions and high-level functions for interfacing with this ADC.
<br>

2.  **Hardware Abstraction Layer (HAL)**
	*Files:* hal.h, hal.c <br>
	*Description:* The HAL provides processor specific functions called by the `ADS124S08` module. 
	
	**IMPORTANT**: The HAL module MUST be modified to work with your hardware!
<br>

3.  **Data Integrity Functions**
	*Files: crc.h, crc.c* <br>
    *Description:* The Data Integrity Functions allow calculation and verification of any CRC data integrity modes enabled for register and conversion data reads. This module is only required if the CRC data integrity mode is enabled on the `ADS124S08`.
<br>    
    
4.  **Unit Test Functions**
	*Files: unit_tests.h, unit_tests.c* <br>
    *Description:* The Unit Test Functions allow verification of hardware code as it relates to control functions of the ADC. This module is not required but is available to check hardware configuration for the processor being used.
<br>    
	
## How to use this code

Reference the *ads124s08.c* file while writing your own code for examples of how to perform typical ADC operations, such as register read/writes and reading data...

OR 

Copy and paste the example code into your project, and update the files as needed to get access to the provided APIs...

 1. Copy the `ADS124S08` and `HAL` module files into your firmware project.

 2. Add library references in *hal.h* to your processor-specific library file(s).

 3. Edit *ALL* of the function implementations inside of *hal.c* to provide the specified functionality with your processor and processor-specific library APIs. 
	
 4. Include a reference to *hal.h* somewhere in your program and call the **initADCperhiperhals()** function to initialize the MCU peripherals connected to the `ADS124S08`.

 5. Include a reference to *ads124s08.h* in your application (from *main.c* or wherever ADC communication is handled) to call the `ADS124S08` module functions.
   
<br>

> **IMPORTANT**: This code was tested on an MSP432E401Y 32-bit ARM® Cortex®-M4F based MCU using TI Code Composer Studio's 20.2.1.LTS ARM compiler. Example code is provided to aid in the creation of your own software implementation and should not be considered as fully verified and end-equipment ready. Example code is written for readability and therefore is typically not optimized for performance.

<br>

# Support {#support}
For questions or issues, visit the [TI E2E Forums](https://e2e.ti.com/).

![](e2e.jpg)

<br>

# Release History {#history}
`v1.0.0` - Initial release (4-04-2022)
