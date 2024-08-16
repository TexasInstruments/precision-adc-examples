### SysConfig Notice

All examples will soon be supported by SysConfig, a tool that will help you graphically configure your software components. A preview is available today in the examples/syscfg_preview directory. Starting in 3Q 2019, with SDK version 3.30, only SysConfig-enabled versions of examples will be provided. For more information, click [here](http://www.ti.com/sysconfignotice).

---
# gpiointerrupt

---

## Example Summary

Application that toggles an LED(s) using a GPIO pin interrupt.

## Peripherals Exercised

* `Board_GPIO_LED0` - Indicates that the board was initialized within
`mainThread()` also toggled by `Board_GPIO_BUTTON0`
* `Board_GPIO_LED1` - Toggled by `Board_GPIO_BUTTON1`
* `Board_GPIO_BUTTON0` - Toggles `Board_GPIO_LED0`
* `Board_GPIO_BUTTON1` - Toggles `Board_GPIO_LED1`

## Resources & Jumper Settings

> If you're using an IDE (such as CCS or IAR), please refer to Board.html in
your project directory for resources used and board-specific jumper settings.
Otherwise, you can find Board.html in the directory
&lt;SDK_INSTALL_DIR&gt;/source/ti/boards/&lt;BOARD&gt;.


## Example Usage

* Run the example. `Board_GPIO_LED0` turns ON to indicate driver
initialization is complete.

* `Board_GPIO_LED0` is toggled by pushing `Board_GPIO_BUTTON0`.
* `Board_GPIO_LED1` is toggled by pushing `Board_GPIO_BUTTON1`.

## Application Design Details

* The `gpioButtonFxn0/1` functions are configured in the *Board.c* file. These
functions are called in the context of the GPIO interrupt.

* Not all boards have more than one button, so `Board_GPIO_LED1` may not be
toggled.

* There is no button de-bounce logic in the example.

TI-RTOS:

* When building in Code Composer Studio, the configuration project will be
imported along with the example. These projects can be found under
\<SDK_INSTALL_DIR>\/kernel/tirtos/builds/\<BOARD\>/(release|debug)/(ccs|gcc).
The configuration project is referenced by the example, so it
will be built first. The "release" configuration has many debug features
disabled. These features include assert checking, logging and runtime stack
checks. For a detailed difference between the "release" and "debug"
configurations, please refer to the TI-RTOS Kernel User's Guide.

FreeRTOS:

* Please view the `FreeRTOSConfig.h` header file for example configuration
information.
