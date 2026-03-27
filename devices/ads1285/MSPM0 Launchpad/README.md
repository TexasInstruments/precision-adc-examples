## [ADS1285](https://www.ti.com/product/ADS1285) Example Code

The following example code project is intended to run on a [TI MSPM0L1306 Lauchpad](https://www.ti.com/tool/LP-MSPM0L1306).  This project utilizes a Hardware Abstraction Layer (HAL) and can be ported to other processors by making use of the HAL.c / HAL.h files. To run this code, import into your own development environment, or use TI's provided embedded development studio CCS.

[CCS](https://www.ti.com/tool/CCSTUDIO) - Embedded IDE

[MSPM0-SDK](https://www.ti.com/tool/MSPM0-SDK) - SDK for TI's MSPM0

[ADS1285 EVM](https://www.ti.com/tool/ADS1285EVM-PDK) Additional hardware used for this demo. 

---

## Peripherals & Pin Assignments

Visit [LP_MSPM0L1306](https://www.ti.com/tool/LP-MSPM0L1306) for LaunchPad information, including user guide and hardware files.

| Peripheral | Pin | Function |
| --- | --- | --- |
| SPI0 | PA6 | SCLK  |
| SPI0 | PA5 | PICO  |
| SPI0 | PA4 | POCI  |
| GPIO | PA23 | CS   |
| GPIO | PA22 | DRDYn |
| GPIO | PA27 | RESETn |
| GPIO | PA7  | PWDNn |
| GPIO | PA3  | SYNCn |
---
## Example Application
The included example application executes a simple data collection routine.  The routine configures the ADC with unique settings, performs an offset calibration, and collects a number of samples before entering standby.

### Helper functions
This example application utilizes several helper functions to increase code readability and development complexity.  The helper functions in conjunction with the Hardware Abstraction Layer (HAL) form an ADS1285 specific SPI layer, providing access to register and ADC data.

## Example Code Functions


```C
void initADC(void);
void sendCommand(uint8_t CMD);
void resetDevice(void);
void writeSingleRegister(uint8_t address, uint8_t data);
uint8_t readSingleRegister(uint8_t address);
void readMultipleRegisters(uint8_t address, uint8_t numRegs);
uint32_t readDataDirect(void);
uint32_t readDataCMD(void);
void restoreRegisterDefaults(void);
uint8_t getRegisterValue(uint8_t address);
double_t calcFdataPeriod(int CONFIG0_DR);
```

## Register Macros
Included in this example code are register definition macros.  These macros are intended to provide easy to read short cuts to defined register values. 

```C

/* Register 0x00 (ID_SYNC) definition
 * |-----------------------------------------------------------------------------------------------|
 * |   Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 * |-----------------------------------------------------------------------------------------------|
 * |                   REVID[3:0]                  |             DEVID[2:0]            |    SYNC   |
 * |-----------------------------------------------------------------------------------------------|
 */

    /* ID_SYNC register */
    #define ID_SYNC_ADDRESS										((uint8_t) 0x00)
    #define ID_SYNC_DEFAULT										((uint8_t) 0x00)
    #define ID_SYNC_DEFAULT_MASK								((uint8_t) 0x0F)

    /* SYNC field */
    #define ID_SYNC_SYNC_MASK									((uint8_t) 0x01)
    #define ID_SYNC_SYNC_BITOFFSET								(0)
    #define ID_SYNC_SYNC_PULSESYNCMODE							((uint8_t) 0x00)    // DEFAULT
    #define ID_SYNC_SYNC_CONTINUOUSSYNCMODE						((uint8_t) 0x01)

    /* DEVID field */
    #define ID_SYNC_DEVID_MASK									((uint8_t) 0x0E)
    #define ID_SYNC_DEVID_BITOFFSET								(1)
    #define ID_SYNC_DEVID_ADS1285								((uint8_t) 0x00)    // DEFAULT

    /* REVID field */
    #define ID_SYNC_REVID_MASK									((uint8_t) 0xF0)
    #define ID_SYNC_REVID_BITOFFSET								(4)

```

## Shadow Memory
This example code provides and utilizes a copy of the ADC's register data as 'shadow memory'.  This memory is instantiated in the uC and helper functions such as ```readSingleRegister();``` and ```writeSingleRegister();``` keep this memory sincronized with the actual contents of the ADCs register values.  This shadow memory is made useful when your application needs to know a register value in the ADC, such as the current data rate.  The data rate of the ADC could be accessed via the CONFIG1 register, which would require an SPI bus transaction, but with the use of shadow memory, CONFIG1 register data is already saved in the uC local memory banks. 
