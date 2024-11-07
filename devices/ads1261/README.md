# ADS1261 Example C Code
Reference this code as an example to help to you get started writing your own ADS1261 code.

## Included modules

**`ADS1261`** - Low-level functions and register map definitions for communicating with the ADS1261 and related devices.
\
**`HAL`** - Processor specific functions required by the `ADS1261` module.

**How to use this code...**
\
The `ADS1261` and `HAL` modules can be copied into your project; however, you will have to edit all of the function contents inside the *hal.c* file and add all required library references to *hal.h* to ensure compatibility with your processor. Once that is complete, include *ads1261.h* in your application and you can begin calling `ADS1261` functions in your code.

---

**IMPORTANT**: This code was tested on a TM4C1294NCPDT 32-bit ARM® Cortex®-M4F based MCU using TI Code Composer Studio's 16.9.1.LTS ARM compiler. This code is provided only as example to aid in the creation of your own software implementation and should not be considered to be fully verified and ready for use in your application.