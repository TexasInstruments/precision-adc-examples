## ADS1x1y0x Example Summary

In this application, the MSPM0 Launchpad acts as an I2C or SPI Controller, sending commands and receiving ADC readings from the ADC target. 

This code example supports the following products: 


| | SPI | I2C |
|---|---|---|
|**16-bit**|ADS111S02, ADS111S04 |ADS111C01, ADS111C02, ADS111C04 |
|**12-bit**|ADS101S02,ADS101S04 |ADS101C01, ADS101C02, ADS101C04 | 
 
**Product Family Decoder:** 

- **1x1y01** - 2 wire input
- **1x1y02** - 4 wire input
- **1x1y04** - 8 wire input
- **111y0x** - 16-bit & 6ksps
- **101y0x** - 12-bit & 4ksps

**Only the C01 and C02 has a Comparator/Alarm feature**




## Project structure
```
.
├── Apps/
│   ├── Sensor/
│   │   ├── inc/
│   │   │   ├── thermocouple.h
│   │   │   ├── thermocouple_tables.h
│   │   │   ├── thermocouple_typeB.h
│   │   │   ├── thermocouple_typeC.h
│   │   │   ├── thermocouple_typeD.h
│   │   │   ├── thermocouple_typeE.h
│   │   │   ├── thermocouple_typeG.h
│   │   │   ├── thermocouple_typeJ.h
│   │   │   ├── thermocouple_typeK.h
│   │   │   ├── thermocouple_typeM.h
│   │   │   ├── thermocouple_typeN.h
│   │   │   ├── thermocouple_typeP.h
│   │   │   ├── thermocouple_typeR.h
│   │   │   ├── thermocouple_typeS.h
│   │   │   └── thermocouple_typeT.h
│   │   └── thermocouple.c
│   ├── example.c
│   └── example.h
├── Debug/
├── Driver/
│   ├── Inc/
│   │   ├── ads1x1c02.h
│   │   ├── ads1x1c04.h
│   │   ├── ads1x1s02.h
│   │   └── ads1x1s04.h
│   ├── ads1x1c02.c
│   ├── ads1x1c04.c
│   ├── ads1x1s04.c
│   ├── ads1x1s0x.c
│   ├── crc.c
│   ├── crc.h
│   ├── hal.c
│   └── hal.h
├── targetConfigs/
│   └── MSPM0G3507.ccxml
├── M0-Launchpad.syscfg
├── main_nortos.c
└── README.md
```



## Application Design Details
The I2C is initialized with the following configuration:
- I2C Controller
- Fast mode +


Look to the `ads1x1c02.c` and `ads1x1c04.c` files (headers in `Driver/Inc`) for additional functions to add complexity to the example application:

```
void     adcStartup(void);
uint16_t readSingleRegister(uint8_t reg);
int32_t  readConversionRegister(void);
bool     writeSingleRegister(uint8_t reg, uint16_t data);
bool     resetDevice(void);
float    measureInternalTemperature(void);
int16_t  readData(void);
uint16_t startAdcConversion(void);
uint16_t stopAdcConversion(void);
uint16_t getRegisterValue(uint8_t reg);
int32_t  signExtend(const uint8_t dataBytes[]);
```
## applicationExample()

`applicationExample()` (in `Apps/example.c`, declared in `Apps/example.h`) is the example application. `main_nortos.c` calls it after `SYSCFG_DL_init()` and a short delay. It measures the temperature of a K-type thermocouple connected to AIN0 and never returns.

**Setup (runs once)**
1. `adcStartup()` waits 50 ms for the ADC to power up, loads the default register values into the local register map, and reads the CONFIG register.
2. CONFIG is written for single-shot conversions with AIN0 measured against GND, a PGA gain of 1, and the TS field set to external measurement (`CONFIG_TS_MUX_SELECTED_FOR_MEASUREMENT`). The write differs slightly by device:
   - **ADS111S02 / S04:** also sets `CONFIG_WRITE_EN_VALID_DATA`.
   - **ADS111C02:** also writes `0x8000` to `HI_THRESH` to enable the DRDY pin.
   - **ADS111C04:** no extra write.
3. The `ADCsettings` structure (`ADCchar_Set`) is filled in with the resolution (16 or 12 bits, from `RESOLUTION_IS_16_BIT`), a PGA gain of 1 and a reference of 1.024 V.

**Measurement loop (repeats every 100 ms)**
1. `startAdcConversion()` starts a single-shot conversion.
2. `readData()` waits for DRDY (up to 1000 ms) and returns the signed conversion result in `ADCsettings.adcValue`.
3. The code is converted to millivolts: `ADCmV = 1000 * adcValue * Vref / 2^(resolution - 1)`.
4. `measureInternalTemperature()` reads the ADC's internal temperature sensor. It is used as the cold-junction temperature (`internalCJCtemp`).
5. `Thermocouple_Linearization_and_CJC(TCtype, ADCmV, internalCJCtemp)` adds the cold-junction voltage to the measured voltage and converts the sum to a temperature in °C, stored in `TCtemp`.
6. `delay_ms(100)` waits before the next measurement.

**Configuration**

| Setting | Where | Default |
|---|---|---|
| Thermocouple type | `TCtype` in `Apps/example.c` | `K` |
| ADC device | `ADS_DEVICE_xxx` define in `Driver/Inc/ads_device.h` | `ADS_DEVICE_C02` |
| I2C address | `g_i2c_addr` in `Apps/example.c` | `ADDR_1` (0x49) |
| Conversion method | `POLYNOMIAL` / `LUT` define in `Apps/Sensor/inc/thermocouple.h` | `POLYNOMIAL` |

**Results**

The latest values are in the globals `ADCmV`, `internalCJCtemp` and `TCtemp`, so they can be viewed in the debugger's watch window. The example does not print or transmit them.

**Test circuit**

The thermocouple is biased through a 1 kΩ resistor from the 3.3 V supply, as drawn in the comment at the top of `Apps/example.c`:

```
[3.3V] --- [1 kΩ] ---+--- [K-type thermocouple] --- [GND]
                     |
                    AIN0
```

## Launchpads and EVMs used
This example uses the [LP-MSPM0G3507](https://www.ti.com/tool/LP-MSPM0G3507) 

## MSPM0G3507 Overview
The [MSPM0G3507](https://www.ti.com/product/MSPM0G3507) is an Arm Cortex-M0+ microcontroller from the MSPM0 family. In this project it is configured as follows:

- **Target configuration:** `targetConfigs/MSPM0G3507.ccxml` (SysConfig board `LP_MSPM0G3507`)
- **CPU clock:** 32 MHz (`CPUCLK_FREQ`)
- **I2C:** I2C1 as a controller in Fast-mode Plus, using the 32 MHz bus clock
- **SPI:** SPI0 as a controller, 8-bit frames, MSB first, Motorola 3-wire mode 1 (CPOL = 0, CPHA = 1), 800 kHz
- **GPIO:** one output for the ADC chip select (CS_n) and one falling-edge interrupt input for the ADC data-ready signal (DRDYn)

Debugging and flashing use the XDS110 on the LaunchPad over the DEBUGSS (SWD) interface.

## MSPM0 Peripherals & Pin Assignments

| Peripheral | Pin | Function |
| --- | --- | --- |
| I2C1 | PB3 | SDA |
| I2C1 | PB2 | SCL |
| SPI0 | PB18 | SCLK |
| SPI0 | PB17 | PICO (MOSI) |
| SPI0 | PB19 | POCI (MISO) |
| GPIO | PA27 | CS_n (SPI chip select) |
| GPIO | PA18 | DRDYn (data ready, falling-edge interrupt) |
| DEBUGSS | PA20 | Debug Clock |
| DEBUGSS | PA19 | Debug Data In Out |

The I2C pins are used with the C-series (I2C) devices and the SPI, CS_n and DRDYn pins with the S-series (SPI) devices.

## BoosterPacks, Board Resources & Jumper Settings

Visit [LP_MSPM0G3507](https://www.ti.com/tool/LP-MSPM0G3507) for LaunchPad information, including user guide and hardware files.
For more information about jumper configuration to achieve low-power using the
