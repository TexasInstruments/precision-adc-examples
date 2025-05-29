# ADS93227 Example C Code for MSPM0
The following example is a [Code Composer Studio (CCS)](https://www.ti.com/tool/CCSTUDIO) project that configures the MSPM0 SPI as a Controller to interface with the [ADS9327](https://www.ti.com/product/ADS9327). 

--------------------------------------------------------------------
## BoosterPacks, Board Resources & Jumper Settings
Visit [LP_MSPM0G3507](https://www.ti.com/tool/LP-MSPM0G3507) for LaunchPad information, including user guide and hardware files.
Visit [ADS9327EVM](https://www.ti.com/tool/ADS9327EVM) for ADS9327 evm information, including user guide.

--------------------------------------------------------------------
## Hardware Connections
This example uses the ADS9327 SPI port wired to the LP-MSPM02507.
| Peripheral  | Pin  | Function |
| ------------ | ------------ | ------------ |
| SPI1  | PB9 | SPI SCLK (Clock)   |
| SPI1 | PB8 | SPI PICO (Peripheral In, Controller Out) |
| SPI1 | PB7 | SPI POCI (Peripheral Out, Controller In) |
| GPIO| PB1 | ADC CONVST signal |

--------------------------------------------------------------------
## Example Usage
The code example provides the following functionality:
1. ADC register read and write on power-up using 24-bit SPI
2. ADC register read and write after enabling 1-SDO mode using 48-bit SPI
3. ADC data read in 1-SDO mode

On power-up, the ADS9327 uses 24-bit SPI frames for register read and write and 4-SDO output. MSPM0 supports 1-SDO interface. This code configures the ADS9327 to use 1-SDO data interface using 24-bit register writes in the initialize_ads9327() function. Enabling 1-SDO mode switches all ADS9327 communication (data and register access) to 48-bit mode. Hence the MSPM0 SPI controller is reconfigured for 48-bit SPI.

--------------------------------------------------------------------
## Additional Resources
- [Code Composer Studio (CCS)](https://www.ti.com/tool/CCSTUDIO)
- [MSPM0-SDK](https://www.ti.com/tool/MSPM0-SDK)