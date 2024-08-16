![IC Image](http://www.ti.com/graphics/folders/partimages/ADS1261.jpg) ADS1261 Example Code
=====================

<span style="color:red">TODO: Update the application note references and link...</span>

The [ADS1261](http://www.ti.com/product/ADS1261) precision delta-sigma (ΔΣ) analog-to-digital converter (ADC) communicates using a serial peripheral interface (SPI) to allow for device configuration, control, and data retrieval. To insterface with the ADS1261 with a microcontroller (MCU), the firmware or software engineer needs to know how to correctly configure their MCU's serial interface, sequence, and time the command bytes to send to the ADC. To assist in this process, this example code is intended to show how to quickly initialize communication with the ADS1261 at a high-level, while also being generic enough to be used with any MCU and in a wide range of applications.

Included modules
----------------

There are two modules included in this example code:

1. **`ADS1261`**\
*Description:* Low-level functions and register map definitions for communicating with the ADS1261.\
*Files: ads1261.h, ads1261.c*

2. **`Interface`**\
*Description:* Processor specific functions called by the `ADS1261` module.\
*Files: interface.h, interface.c*

How to use this code
--------------------

1. Copy the `ADS1261` and `Interface` module files into your firmware project.

2. Change all library references in *interface.h* to your processor-specific library file(s).

```c
/* SPI header file - Insert your own processor specific header file(s) here */
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
```

3. Edit all of the function implementations inside of *interface.c* to provide the specified funcationality with your processor and processor-specific library APIs.

```c
/**
 * \fn void init_spi_peripheral(void)
 * \brief Configures the MCU's SPI peripheral for interfacing with the ADS1261
 */
void init_spi_peripheral(void)
{
    /* NOTE: INSERT YOUR CODE HERE */
}
```

4. Finally, include a reference to *ads1261.h* in your application (from *main.c* or wherever ADC communication is handled). You should now be able to begin calling the `ADS1261` module functions in your code. Refer to app note section x.x.x for additional details and examples of how to sequence function calls to the `ADS1261` module.

> **IMPORTANT**: This code was tested on a TM4C1294NCPDT 32-bit ARM® Cortex®-M4F based MCU using TI Code Composer Studio's 16.9.1.LTS ARM compiler. This code is provided only as example to aid in the creation of your own software implementation and should not be considered to be fully verified and ready for use in your end application. Also, this example code was written for readability and has not been optimized for performance.

Support
-------

Reference the replated application note: [SBACxxx](https://e2e.ti.com/).\
For questions or issues, visit the [TI E2E Forums](https://e2e.ti.com/).

![E2E Logo](http://e2e.ti.com/resized-image/__size/75x0/__key/CommunityServer-Wikis-Components-Files/00-00-00-01-27/2234.ti_2D00_e2e_2D00_Pos_2D00_no_2D00_text_2D00_150.jpg)

Release History
---------------

`v1.0` [04-17-2018] - Initial release\
`v1.1` [07-19-2018] - Updated documentation and added SPI initialization code example

License
-------

**BSD-3-Clause**\
Refer to *manifest.html* for license text...

![TI Logo](http://www.ti.com/assets/images/ic-logo.png)