ADS9327-C2000-EXAMPLE-CODE
============================

This is an SPI communication example for the [ADS9327](https://www.ti.com/product/ADS9327) and [C2000&trade; MCU (F28P65x)](https://www.ti.com/product/TMS320F28P650DK). The code loads the test pattern register for ADC-A with 0xABCD and the test pattern register for ADC-B with 0xEF12. The test pattern data is then output by the ADC in either 1-lane mode or 4-lane mode, depending on user configured macro.

## Setup Instructions
- To use this code, make sure you have installed [Code Composer Studio&trade; (CCS)](https://www.ti.com/tool/CCSTUDIO) v20.0.0 or later and [C2000Ware](https://www.ti.com/tool/C2000WARE) (5.04)
- Create a new directory in `C2000Ware_5_04_00_00\examples\demos\`, we'll name it `ADS9327_F28P65X` for referene.
- Copy the `f28p65x` and `source` folders to the new directory. e.g.
  - `C2000Ware_5_04_00_00\examples\demos\ADS9327_F28P65X\f28p65x`
  - `C2000Ware_5_04_00_00\examples\demos\ADS9327_F28P65X\source`
- Open CCS and click on Projects -> Import CCS Projects
- Navigate to ADS9327 example

## C2000 Pins Used
Pin numbers are for TMDSHSEDOCK board when used with [TMDSCNCD28P65X](https://www.ti.com/tool/TMDSCNCD28P65X).

- SPID_CLK = GPIO93 = Pin 162
- SPID_PTE = GPIO94 = Pin 163
- SPID_PICO = GPIO91 = Pin 160
- SPID_POCI = GPIO92 = Pin 161
- CONVST = GPIO0 (EPWM1_A) = Pin 49
- CLB_Input1 = GPIO10 = Pin 61
- CLB_Input2 = GPIO11 = Pin 63
- CLB_Input3 = GPIO8 = Pin 57
- CLB_Input4 = GPIO9 = Pin 59

## Signal Connections
Connections between ADS9327 ADC and C2000

### 1-Lane Mode Connections:
- ADC SCLK connected to C2000 SPID_CLK
- ADC /CS connected to C2000 SPID_PTE
- ADC SDI connected to C2000 SPID_PICO
- ADC D3 connected to C2000 SPID_POCI
- ADC CONVST connected to C2000 CONVST  

### 4-Lane Mode Connections:
- ADC SCLK connected to C2000 SPID_CLK
- ADC /CS connected to C2000 SPID_PTE
- ADC SDI connected to C2000 SPID_PICO
- ADC D3 connected to C2000 CLB_Input1
- ADC D2 connected to C2000 CLB_Input2
- ADC D1 connected to C2000 CLB_Input3
- ADC D0 connected to C2000 CLB_Input4
- ADC CONVST connected to C2000 CONVST

## Variables to Control
Variables can be found in ADS9327_settings.h file
- DATA_LANE_SELECT MACRO controls the output data lanes. It can be set to 1 or 4.
- mySPI0_BITRATE controls the SPI interface speed. In Hz units. Can be set to max 50MHz.
- ADS9327_EXT_ADC1_SAMPLING_FREQUENCY_KHZ controls CONVST frequency. In kHz units.
- MAX_ENTRIES controls ADC output data array length.

## Code Testing
- 1-lane mode has been tested upto mySPI0_BITRATE = 50MSPS and CONVST = 900KHz
- 4-lane mode has been tested upto mySPI0_BITRATE = 33MSPS and CONVST = 2MHz
- This code was tested using CCS Theia Version: 1.4.1.1 and C2000Ware Version 5.04

## License
This project is licensed under the BSD-3-Clause License.

## Support
- [Data converters forum](https://e2e.ti.com/support/data-converters-group/data-converters/f/data-converters-forum)
- [C2000 microcontrollers forum](https://e2e.ti.com/support/microcontrollers/c2000-microcontrollers-group/c2000/f/c2000-microcontrollers-forum)