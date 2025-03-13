<div align="center">

# Precision Analog to Digital Converters Example Code

[Introduction](#introduction) | [Devices](#devices) | [Precision ADC Homepage](https://www.ti.com/precisionadc) | [Repo Organization](#repo-organization) | [Usage](#usage) | [Contribute](#contributing-to-the-project) 


<img src="https://github.com/TexasInstruments/precision-adc-examples/blob/main/docs/media/ADC.jpg"><br/>

</div>

## Introduction

Precision Analog to Digital Converters from Texas Instruments are specialized mixed signal products mainly used in signal acquisition and sensor applications.  Generally, these products communicate with a micro-controller or FPGA as an SPI or I2C peripheral to configure internal settings and transmit digitized analog signals.  

This repository is a collection of firmware examples for controlling, configuring, reading, and interacting with these products.  You will find examples of micro-controller firmware applications which configure and operate these ADCs.

Find more information about precision ADC device selection [here](https://content.ti.com/ls/9f37f889-9ac5-4ed2-8f85-d28a7d7c0631/N7KG5rNwX6Uaws6h)

## Devices

Device examples include the following products:

### General Purpose ADCs:
- [ADS1115](https://www.ti.com/product/ADS1115) 16-bit, 860-SPS, 4-channel, delta-sigma ADC with PGA, oscillator, VREF, comparator and I2C
- [ADS1118](https://www.ti.com/product/ADS1118) 16-bit, 860-SPS, 4-channel, delta-sigma ADC with PGA, oscillator, VREF, temp sensor and SPI
- [ADS1256](https://www.ti.com/product/ADS1256) 24-Bit, 30kSPS, 8-Ch Delta-Sigma ADC With PGA for Factory Automation and Process Control
- [ADS7128](https://www.ti.com/product/ADS7128) Small 8-ch 12-bit analog-to-digital converter (ADC) with I2C interface, GPIOs, CRC and RMS module
### Multi-Channel Sensor Measurement:
- [ADS1220](https://www.ti.com/product/ADS1220) 24-bit, 2-kSPS, four-channel, low-power, delta-sigma ADC with PGA, VREF, SPI and two IDACs
- [ADS122C04](https://www.ti.com/product/ADS122C04) **upcoming release** Engineering Samples availble today contact - email: dcc-padc-marketing@list.ti.com
- [ADS122S14](https://www.ti.com/product/ADS122S14)	**upcoming release** Engineering Samples availble today contact - email: dcc-padc-marketing@list.ti.com
- [ADS122U04](https://www.ti.com/product/ADS122U04) 24-bit, 2-kSPS, 4-ch, low-power, small-size delta-sigma ADC w/ PGA, VREF, 2x IDACs & UART interface
- [ADS1235](https://www.ti.com/product/ADS1235) 24-bit, 7.2-kSPS, 3-ch differential, delta-sigma ADC with PGA and AC excitation for bridge sensors
- [ADS124S08](https://www.ti.com/product/ADS124S08) 24-bit, 4-kSPS, 12-ch delta-sigma ADC with PGA and voltage reference for sensor measurement
- [ADS1258](https://www.ti.com/product/ADS1258) 24-bit, 125kSPS, 16-ch delta-sigma ADC with fast channel scan and automatic sequencer
- [ADS1262](https://www.ti.com/product/ADS1262) 32-bit 38-kSPS 10-ch delta-sigma ADC with PGA and voltage reference for factory automation
### Multi-Channel Simultaneous ADCs:
- [ADS127L18](https://www.ti.com/product/ADS127L18) 24-bit, 512/1365-ksps, 8-channel, simultaneous-sampling, wideband delta-sigma ADC
- [ADS131A04](https://www.ti.com/product/ADS131A04) 24-bit 128-kSPS 4-channel simultaneous-sampling delta-sigma ADC
- [ADS131B04-Q1](https://www.ti.com/product/ADS131B04-Q1) Automotive 24-bit, 32-kSPS, four-channel, simultaneous-sampling, delta-sigma ADC
- [ADS131M08](https://www.ti.com/product/ADS131M08) 24-bit, 32-kSPS, 8-channel, simultaneous-sampling, delta-sigma ADC

### Special Purpose ADCs:
- [ADS1292](https://www.ti.com/product/ADS1292) 24-bit, 2-ch, Low-Power Analog Front END (AFE) for ECG Applications
- [ADS1299](https://www.ti.com/product/ADS1299)  Low-Noise, mulit-Channel, 24-Bit, Analog-to-Digital Converter for EEG and
### Biopotential Measurements:
- [ADS1282](https://www.ti.com/product/ADS1282) Ultra-high-resolution 4-kSPS 2-channel delta-sigma ADC with PGA for seismic and energy exploration	

## Repo Organization
This repository is organized by individual device name.  In the **/devices** directory, our entire catalog of example code can be found.  Inside each product directory will be a set of C example code files.  The`'device'.c`and`'device'.h` files will be specific to the ADC and can be used 'as is' in an embedded development environment.  The `hal.c` and `hal.h` files function as a **Hardware Abstraction Layer (HAL)** and should be edited to match your specific processor.  In some device directories there may be multiple `hal.c` and `hal.h` files which may provide the **HAL** functions for a variety of processors.  Additionally, some devices may include complete *Code Composer Studio* projects or example applications.  These self contained projects will have additional support functions besides the **device** and **HAL** files.  
## Usage
  `'device'.c` and `'device'.h` include processor agnostic functions which perform basic actions to the data converter.  The `'device'.c` file will include functions such as:
- `initalize();`
- `reset();`
- `loadConfig();`
- `writeRegister(address,data);`
- `readRegister(address);`
- `readADCdata(numSamples);`

### Function Example:
```c
void writeSingleRegister( SPI_Handle spiHdl, uint8_t address, uint8_t data )
{
    /* Initialize arrays */
    uint8_t DataTx[COMMAND_LENGTH + 1] = { OPCODE_WREG | (address & OPCODE_RWREG_MASK), 0, data};
    uint8_t DataRx[COMMAND_LENGTH + 1] = { 0 };

    /* Check that the register address is in range */
    assert( address < NUM_REGISTERS );

    /* Build TX array and send it */
    spiSendReceiveArrays( spiHdl, DataTx, DataRx, COMMAND_LENGTH + 1 );

    /* Update register array */
    registerMap[address] = DataTx[COMMAND_LENGTH];
}
```
### Internal Memory
An array of `registerMap[]` variables are generated in order to create and store a copy of the ADCs current register values.  These variables are used in multiple ways in order to speed development and communication with the ADC.  

**Usage case #1:** When reading (or writing) register values, update the internal registerMap variable to contain the present value.  Because registers can hold the values of multiple fields, updating an entire register will overwrite all fields at `[address]` with a new value.  In order to preserve the values already present in the register, use the registerMap[] object to *or* the the current register value with the changing field value. 
```c
// Update register value  
    registerMap[address] = DataTx[COMMAND_LENGTH];
    writeSingleRegister( address, registerMap[address] | newFieldValueMask);
    registerMap[address] = registerMap[address] | newFieldValueMask
```
**Usage case #2:**
When utilizing complex SPI or I2C interfaces, some device operations require the frame size to be adjusted. This happens, for the example, when adding a STATUS or CRC byte to the transaction.  Using the internal device registerMap memory can be a fast and safe way to determine which communication method should be used. 
```c
if(registerMap[deviceConfigAddress] == CRC_Enabled)
{
spiSendRecieve(Tx,Rx,3); // send 3 word SPI frame ( includes CRC)
}
else
{
spiSendRecieve(Tx,Rx,2); // send 2 word SPI frame
}
```
The `'device'.h` file will include device specific `#define`c statements to enumerate and document fields in the device register map.  these definitions will provide a direct correlation to registers defined in the product data sheet. 
```c
/* ADS124S08 Register 0x1 (STATUS) Definition
 *|  Bit 7   |   Bit 6   |   Bit 5   |   Bit 4   |   Bit 3   |   Bit 2   |   Bit 1   |   Bit 0   |
 *------------------------------------------------------------------------------------------------
 *|  FL_POR  |    nRDY   | FL_P_RAILP| FL_P_RAILN| FL_N_RAILP| FL_N_RAILN| FL_REF_L1 | FL_REF_L0 |
 *------------------------------------------------------------------------------------------------
 */
    /** STATUS register address */
        #define REG_ADDR_STATUS         ((uint8_t) 0x01)

    /** STATUS default (reset) value */
	#define STATUS_DEFAULT          ((uint8_t) 0x80)
	#define ADS_FL_POR_MASK			0x80
	#define ADS_NRDY_MASK			0x40
	#define ADS_FL_P_RAILP_MASK		0x20
	#define ADS_FL_P_RAILN_MASK		0x10
	#define ADS_FL_N_RAILP_MASK		0x08
	#define ADS_FL_N_RAILN_MASK		0x04
	#define ADS_FL_REF_L1_MASK		0x02
	#define ADS_FL_REF_L0_MASK		0x10

```

These functions should be included in an embedded project by including the `'device'.c` and `'device'.h` files.  Once imported, these functions can be used inline in your main application to assist in the development of higher level functions or complete applications.
 
### Contributing to the project
- Currently we are not accepting contributions but if you have feedback for us based upon the examples provided feel free to email [us](mailto:precision-adc-examples-github@list.ti.com)
