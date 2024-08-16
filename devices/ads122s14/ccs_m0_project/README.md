## Example Summary

Note: The use of "Master" and "Slave", along with "MOSI/MISO" terminology is being considered obsolete. These terms will be replaced with "Controller" and "Peripheral", and "PICO/POCI" respectively.

The following example configures the SPI as a Controller.

This example uses the CD/CS3 line as a signal to distinguish between Command and Data information.
- C/D level low: command information
- C/D level high: data information

When a value is written into the CTL1.CDMODE bits the C/D line will go low for the given numbers of byte which
are sent by the SPI, starting with the next value to be transmitted. After the number of bytes are transmitted the
C/D will be set to high automatically.

After all transmits have been performed, the LED will toggle to indicate completion.

## Peripherals & Pin Assignments

| Peripheral | Pin | Function |
| --- | --- | --- |
| GPIOA | PA0 | Open-Drain Output |
| SYSCTL |  |  |
| SPI0 | PA6 | SPI SCLK (Clock) |
| SPI0 | PA5 | SPI PICO (Peripheral In, Controller Out) |
| SPI0 | PA4 | SPI POCI (Peripheral Out, Controller In) |
| SPI0 | PA3 | SPI CS1 (Chip Select 1) |
| SPI0 | PA27 | SPI CS3/CD (Chip Select 3/Command Data) |
| EVENT |  |  |
| DEBUGSS | PA20 | Debug Clock |
| DEBUGSS | PA19 | Debug Data In Out |

## BoosterPacks, Board Resources & Jumper Settings

Visit [LP_MSPM0L1306](https://www.ti.com/tool/LP-MSPM0L1306) for LaunchPad information, including user guide and hardware files.

| Pin | Peripheral | Function | LaunchPad Pin | LaunchPad Settings |
| --- | --- | --- | --- | --- |
| PA0 | GPIOA | PA0 | J1_10 | <ul><li>PA0 is 5V tolerant open-drain so it requires pull-up<br><ul><li>`J10 2:3` Use 3.3V pull-up<br><li>`J10 1:2` Use 5V pull-up</ul><br><li>PA0 can be connected to LED1<br><ul><li>`J2 ON` Connect to LED1<br><li>`J2 OFF` Disconnect from LED1</ul></ul> |
| PA6 | SPI0 | SCLK | J1_7 | N/A |
| PA5 | SPI0 | MOSI | J2_15 | N/A |
| PA4 | SPI0 | MISO | J2_14 | N/A |
| PA3 | SPI0 | CS1_MISO1 | J2_19 | N/A |
| PA27 | SPI0 | CS3_CD_MISO3 | J4_37 | <ul><li>PA27 can be connected to LED2 Blue<br><ul><li>`J13 OFF` Disconnect from LED2 Blue</ul></ul> |
| PA20 | DEBUGSS | SWCLK | J2_13 | <ul><li>PA20 is used by SWD during debugging<br><ul><li>`J101 15:16 ON` Connect to XDS-110 SWCLK while debugging<br><li>`J101 15:16 OFF` Disconnect from XDS-110 SWCLK if using pin in application</ul></ul> |
| PA19 | DEBUGSS | SWDIO | J2_17 | <ul><li>PA19 is used by SWD during debugging<br><ul><li>`J101 13:14 ON` Connect to XDS-110 SWDIO while debugging<br><li>`J101 13:14 OFF` Disconnect from XDS-110 SWDIO if using pin in application</ul></ul> |

### Device Migration Recommendations
This project was developed for a superset device included in the LP_MSPM0L1306 LaunchPad. Please
visit the [CCS User's Guide](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/tools/ccs_ide_guide/doc_guide/doc_guide-srcs/ccs_ide_guide.html#sysconfig-project-migration)
for information about migrating to other MSPM0 devices.

### Low-Power Recommendations
TI recommends to terminate unused pins by setting the corresponding functions to
GPIO and configure the pins to output low or input with internal
pullup/pulldown resistor.

SysConfig allows developers to easily configure unused pins by selecting **Board**→**Configure Unused Pins**.

For more information about jumper configuration to achieve low-power using the
MSPM0 LaunchPad, please visit the [LP-MSPM0L1306 User's Guide](https://www.ti.com/lit/slau869).

## Example Usage
Make the following connections between the SPI Controller and SPI Peripheral:
- Controller SCLK -> Peripheral SCLK
- Controller PICO -> Peripheral PICO
- Controller POCI <- Peripheral POCI
- Controller CS   -> Peripheral CS
- Controller CD   -> Peripheral CD

Compile, load and run the example. The SPI will automatically start
to transmit and receive data.

## Application Design Details
The SPI is initialized with the following configuration:
- SPI Controller
- Motorola 4 Wire with Polarity 0, Phase 0
- No parity
- 8 bits per transfer
- MSB first
