## ADS122S14 Example Summary

In this application, the MSPM0 Launchpad acts as an I2C Controller, sending commands and receiving ADC readings from the ADS122C14 target. 

The following example configures the SPI as a Controller and communicates with an ADS122S14.  The example program `main_nortos.c` performs the following:
  1) Initalize M0 & ADC
  2) Set ADC configuration for a single conversion (one shot)
  3) Poll status word for a new conversion
  4) Store conversion result 
  5) Change to continuous conversion mode and start converting
  6) Collect adcNumSamples # of samples
  7) Stop conversions and powerdown. 


## Project structure
```
.
├── Apps/
│   ├── Example.c
│   └── Example.h
├── Debug
├── Driver/
│   ├── ads122y1x.c
│   ├── ads122y1x.h
│   ├── crc.c
│   ├── crc.h
│   ├── hal.c
│   └── hal.h
├── targetConfigs/
│   └── MSPM0L1306.ccxml
├── M0-Launchpad.syscfg
├── main_nortos.c
└── README.md
```



## Application Design Details
The I2C is initialized with the following configuration:
- I2C Controller
- Fast mode +


Look to the `ads122y1x.c` file for additional functions to add complexity to the example application:

```
static void   restoreRegisterDefaults(void);
uint8_t       getRegisterValue(uint8_t address);
void          adcStartup(void);
void          clearSTATUSflags(void);
uint8_t       readSingleRegister(uint8_t address);
bool          writeSingleRegister(uint8_t address, uint8_t data);
adc_channel_t     readData(void);
int32_t       signExtend(const uint8_t dataBytes[]);
bool          resetDevice(void);
void          enableRegisterMapCrc(bool enable);
bool          isValidCrcOut(void);
```
## Launchpads and EMVs used
This example uses the [MSPM0L1306](https://www.ti.com/tool/LP-MSPM0L1306) Launchpad along with the [ADS122C14 EMV](https://www.ti.com/tool/ADS122C14EVM-PDK).  The EVM has been designed to be booster pack compatable, and connects to the MSPM0 Launchpad as a standard boosterpack. 

## MSPM0 Peripherals & Pin Assignments

| Peripheral | Pin | Function |
| --- | --- | --- |
| I2C0 | PA0 | SDA |
| I2C0 | PA1 | SCL |
| DEBUGSS | PA20 | Debug Clock |
| DEBUGSS | PA19 | Debug Data In Out |

## BoosterPacks, Board Resources & Jumper Settings

Visit [LP_MSPM0L1306](https://www.ti.com/tool/LP-MSPM0L1306) for LaunchPad information, including user guide and hardware files.
For more information about jumper configuration to achieve low-power using the
MSPM0 LaunchPad, please visit the [LP-MSPM0L1306 User's Guide](https://www.ti.com/lit/slau869).