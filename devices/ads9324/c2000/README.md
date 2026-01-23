# ADS9324-C2000-EXAMPLE-CODE
This is an SPI communication example for the [ADS9324](https://www.ti.com/product/ADS9324) and [C2000&trade; MCU (F28P65x)](https://www.ti.com/product/TMS320F28P650DK).

## Hardware Setup
Hardware used:
- [ADS9324EVM](https://www.ti.com/tool/ADS9324EVM)
- [TMS320F28P65X controlCARD](https://www.ti.com/tool/TMDSCNCD28P65X)
- [controlCARD docking station](https://www.ti.com/tool/TMDSHSECDOCK)

To use the ADS9324EVM with the C2000 controlCard, complete the following steps:
- Remove R78 to R96 to disconnect level translators from ADC pins
- Remove R75 to disconnect IOVDD = 1.8V
- Populate R76 to connect IOVDD = 3.3V
- 3.3V LDO is not populated so you can connect external 3.3V to test point TP5
- Move jumper of JP1 to EXT_PWR position
- Connect 5.5V and GND to terminal block J19

## Software Setup
- To use this code, make sure you have installed [Code Composer Studio&trade; (CCS)](https://www.ti.com/tool/CCSTUDIO) (v20.4.0 or later) and [C2000Ware](https://www.ti.com/tool/C2000WARE) (v5.04.00.00)
- Copy the project folder "ads93xxv_f28p65x" to `C2000Ware_5_04_00_00\examples\demos`
- Open CCS and click on Projects -> Import CCS Projects
- Navigate to ads93xxv example

## Files in Project:
- ads93xxv.c : ADC level functions, like software reset, register map bank select, and initialization sequence
- ads93xxv.h : ADC level functions header file
- ads93xxv_hal.c : Hardware abstraction layer, peripheral initializations 
- ads93xxv_hal.h : Hardware abstraction layer, register read/write, interrupts, GPIO number definitions
- ads93xxv_main.c : main function, test pattern test
- ads93xxv_regbank0.h : Register map, bank 0 macros
- ads93xxv_regbank1.h : Register map, bank 1 macros
- ads93xxv_regbank2.h : Register map, bank 2 macros
- ads93xxv_settings.h : Settings to set SPI bitrate, number of ADC cycles/iterations to capture, CONVST frequency macros.


## EVM Digital Signal Pins
Header J20:
- CONVST = CONVST (on TMDSCNCD28P65X)
- BAS = DRDY (on TMDSCNCD28P65X)

Header J21:
- SDOUT = SPIA_POCI (on TMDSCNCD28P65X)
- SCLK = SPIA_CLK (on TMDSCNCD28P65X)
- SDI = SPIA_PICO (on TMDSCNCD28P65X)
- CSn = SPIA_PTE (on TMDSCNCD28P65X)
- RESETn = connect to 3.3V (on TMDSCNCD28P65X)
- REFSEL = connect to 3.3V for internal reference mode (on TMDSCNCD28P65X)

## C2000 Pin Connections
Pin numbers are for TMDSCNCD28P65X. [Pin Map](https://www.ti.com/lit/ml/spruja5/spruja5.pdf)
- SPIA_CLK = ADC SCLK = GPIO18
- SPIA_PTE = ADC CS = GPIO35
- SPIA_PICO = ADC SDI = GPIO16
- SPIA_POCI = ADC SDO = GPIO17
- CONVST = GPIO-6 (EPWM4_A)
- DRDY/ALARM = GPIO32

## Variables to Control
Variables can be found in ads93xxv_settings.h file
- ADS93XXV_SPI_CLOCK_FREQ_HZ controls the SPI interface speed. In Hz units. Can be set to max 50MHz.
- ADS93XXV_SAMPLING_FREQUENCY_KHZ controls CONVST frequency. In kHz units. ADC data rate will be CONVST/OSR. 
- ads93xxv is programmed to OSR of 8 at default

## Code Testing
- 1-lane mode has been tested upto ADS93XXV_SPI_CLOCK_FREQ_HZ = 25MHz and ADS93XXV_SAMPLING_FREQUENCY_KHZ = 200KHz
- This code was tested using CCS version 20.4.0 and C2000Ware version 5.04