## Example Summary

The following example configures the MSPM0 SPI as a Controller to interface with ADS7066 and ADS7067 ADCs.
This example is intended to be used with the with [ADS7066](https://www.ti.com/product/ADS7066) and [ADS7067](https://www.ti.com/product/ADS7067) ADCs.

This example sets up the SPI to read and write registers using 24-bit SPI frames with manual CS control. ADC data is transmitted using 16-bit SPI frames.

This example was built with the following tools:
- Code Composer Studio Version 20.0.1.4__1.6.1 
- TI Clang compiler version 4.0.0 LTS 
- MSPM0 SDK version 2.2.0.05
- SysConfig 1.21.1

## BoosterPacks, Board Resources & Jumper Settings

Visit [LP_MSPM0G3507](https://www.ti.com/tool/LP-MSPM0G3507) for LaunchPad information, including user guide and hardware files.
Visit [ADS7066EVM-PDK](https://www.ti.com/tool/ADS7066EVM-PDK) for ADS7066 and ADS7067 booster pack information, including user guide.

## Hardware Connections

This example uses the booster-pack from the ADS7066EVM-PDK mounted on the LP-MSPM02507. 

![Booster Pack Image](https://www.ti.com/content/dam/ticom/images/products/ic/microcontrollers/msp/evm-board/lp-mspm0g3507-top.png:small) ![EVM image](https://www.ti.com/content/dam/ticom/images/products/ic/dataconverters/evm-board/ads7066evm-pdk-top.png:small)

1. On the ADS7066 booster pack, connect J2-13 to J2-19 for CS control.
2. On the ADS7066 booster pack, short AIN2 to AIN3 by installing a jumper between pins J5-5 and J5-6. 



## Example Usage
The code example resets the ADC and configures the following channels of the ADC:
AIN2: digital output, push-pull
AIN3: analog input
AIN7: digital output to control LED onboard ADS7066EVM-PDK

The LED on the booster pack is turned on by configuring AIN7 as a digital output and setting it to logic 0.

Analog input channel 3 is configured as an analog input. For the purpose of this demo, the AIN3/GPIO3 drives logic 0 (close to 0V) and logic 1 (close to AVDD) that is measured by AIN3.

After initialization, read the AIN3 signal in a loop forever.