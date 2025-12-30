## ADS122S14 Example Summary

In this application, the MSPM0 Launchpad acts as an SPI Controller, sending commands and receiving ADC readings from the ADS122S14 SPI target. 

The following example configures the SPI as a Controller and communicates with an ADS122S14.  The example program `main_nortos.c` performs the following:
  1) Initalize M0 & ADC
  2) Set ADC configuration for a single conversion (one shot)
  3) Poll status word for a new conversion
  4) Store conversion result 
  5) Change to continuous conversion mode and start converting
  6) Collect adcNumSamples # of samples
  7) Stop conversions and powerdown. 

note: additional product family variants are supported to enable useage with I2C and 16-bit product variants. eg. ADS122C14, ADS112S14, ADS112C14, etc. 

## Project structure
```
evm/
├─ ads122y1x.c
├─ ads122y1x.h
├─ crc.c
├─ crc.h
├─ hal.c
├─ hal.h
targetConfig/
├─ MSPM0L1306.ccxml
main_nortos.c
M0-Launchpad.syscfg
```




## Application Design Details
The SPI is initialized with the following configuration:
- SPI Controller
- Motorola 4 Wire with clock polarity low and data latched on falling edge of SCLK
- No parity
- 8 bits per transfer
- MSB first



The delay between SPI transmissions is controlled by the macro `SPI_DELAY`.  Adjusting this constant will introduce more/less delay between SPI frames. 



Look to the `ads122y1x.c` file for additional functions to add complexity to the example application:

```
static void       restoreRegisterDefaults(void);
uint8_t           getRegisterValue(uint8_t address);
void              adcStartup(void);
void              clearSTATUSflags(void);
uint8_t           readSingleRegister(uint8_t address);
bool              writeSingleRegister(uint8_t address, uint8_t data);
adc_channel_t     readData(void);
int32_t           signExtend(const uint8_t dataBytes[]);
bool              resetDevice(void);
void              enableRegisterMapCrc(bool enable);
bool              isValidCrcOut(void); 
```
## Launchpads and EMVs used
This example uses the [MSPM0L1306](https://www.ti.com/tool/LP-MSPM0L1306) Launchpad along with the [ADS122S14 EMV](https://www.ti.com/tool/ADS122S14EVM).  The EVM has been designed to be booster pack compatable, and connects to the MSPM0 Launchpad as a standard boosterpack. 

## MSPM0 Peripherals & Pin Assignments

| Peripheral | Pin | Function |
| --- | --- | --- |
| SPI0 | PA6 | SPI SCLK (Clock) |
| SPI0 | PA5 | SPI PICO (Peripheral In, Controller Out) |
| SPI0 | PA4 | SPI POCI (Peripheral Out, Controller In) |
| SPI0 | PA23 | SPI CS3/CD (Chip Select 3/Command Data) |
| I2C0 | PA0 | SDA |
| I2C0 | PA1 | SCL |
| DEBUGSS | PA20 | Debug Clock |
| DEBUGSS | PA19 | Debug Data In Out |

## BoosterPacks, Board Resources & Jumper Settings

Visit [LP_MSPM0L1306](https://www.ti.com/tool/LP-MSPM0L1306) for LaunchPad information, including user guide and hardware files.
For more information about jumper configuration to achieve low-power using the
MSPM0 LaunchPad, please visit the [LP-MSPM0L1306 User's Guide](https://www.ti.com/lit/slau869).