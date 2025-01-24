## Example Summary

The following example configures the MSPM0 SPI as a Controller to interface with ADS704x and ADS705x ADCs.
This example is intended to be used with the with ADS704x and ADS705x ADCs.

This example sets up the SPI to trigger the DMA Channel 0 to do a transmit data transfer and the DMA Channel 1 to do a receive data transfer repeatedly every second as triggered by a timer.

The DMA Channel 0 is configured in single transfer mode, using the static SPI TX DMA event as the DMA trigger.
When the SPI TXINT is set (which indicates that the SPI is ready to transmit data), this triggers the DMA to transfer the data from a user buffer to the SPI TXFIFO. This transfer will start as soon as the channel is enabled, because the SPI TXINT should be set when the SPI is enabled.
The DMA Channel 1 is configured in single transfer mode, using the static SPI RX DMA event as the DMA trigger.
When ADC data is received and the SPI RXINT is set, this triggers the DMA to transfer the data from the SPI RXFIFO to a user buffer.

After SPI_PACKET_SIZE bytes have been transferred by the DMA, this will trigger the DL_SPI_IIDX_DMA_DONE_TX interrupt indicating that the DMA is done transferring all expected bytes from the buffer to the TXFIFO.
The SPI Controller will then initiate the transaction. The SPI will transmit the data that is in the TXFIFO until it is empty. When the TXFIFO is empty, this will trigger the DL_SPI_IIDX_TX_EMPTY interrupt indicating that the transfer is complete.

When the SPI Controller receives SPI_PACKET_SIZE bytes from the SPI Peripheral, this will trigger the DL_SPI_IIDX_DMA_DONE_RX interrupt indicating that the DMA is done transferring all expected bytes from the RXFIFO to the buffer.

This example was built with the following tools:
- Code Composer Studio Version 20.0.1.4__1.6.1 
- TI Clang compiler version 4.0.0 LTS 
- MSPM0 SDK version 2.2.0.05
- SysConfig 1.21.1

## BoosterPacks, Board Resources & Jumper Settings

Visit [LP_MSPM0G3507](https://www.ti.com/tool/LP-MSPM0G3507) for LaunchPad information, including user guide and hardware files.

| Pin | Peripheral | Function | LaunchPad Pin | LaunchPad Settings |
| --- | --- | --- | --- | --- |
| PA0 | GPIOA | PA0 | J27_9 | <ul><li>PA0 is 5V tolerant open-drain so it requires pull-up<br><ul><li>`J19 1:2` Use 3.3V pull-up<br><li>`J19 2:3` Use 5V pull-up</ul><br><li>PA0 can be connected to LED1<br><ul><li>`J4 ON` Connect to LED1<br><li>`J4 OFF` Disconnect from LED1</ul></ul> |
| PA15 | GPIOA | PA15 | J3_30 | N/A |
| PB9 | SPI1 | SCLK | J1_7 | N/A |
| PB8 | SPI1 | MOSI | J2_15 | N/A |
| PB7 | SPI1 | MISO | J2_14 | N/A |
| PB17 | SPI1 | CS1_MISO1 | J2_18 | N/A |
| PA20 | DEBUGSS | SWCLK | N/A | <ul><li>PA20 is used by SWD during debugging<br><ul><li>`J101 15:16 ON` Connect to XDS-110 SWCLK while debugging<br><li>`J101 15:16 OFF` Disconnect from XDS-110 SWCLK if using pin in application</ul></ul> |
| PA19 | DEBUGSS | SWDIO | N/A | <ul><li>PA19 is used by SWD during debugging<br><ul><li>`J101 13:14 ON` Connect to XDS-110 SWDIO while debugging<br><li>`J101 13:14 OFF` Disconnect from XDS-110 SWDIO if using pin in application</ul></ul> |


![Booster Pack Image](https://www.ti.com/content/dam/ticom/images/products/ic/microcontrollers/msp/evm-board/lp-mspm0g3507-top.png:small) 

### Device Migration Recommendations
This project was developed for a superset device included in the LP_MSPM0G3507 LaunchPad. Please
visit the [CCS User's Guide](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/tools/ccs_ide_guide/doc_guide/doc_guide-srcs/ccs_ide_guide.html#sysconfig-project-migration)
for information about migrating to other MSPM0 devices.

### Low-Power Recommendations
TI recommends to terminate unused pins by setting the corresponding functions to
GPIO and configure the pins to output low or input with internal
pullup/pulldown resistor.

SysConfig allows developers to easily configure unused pins by selecting **Board**→**Configure Unused Pins**.

For more information about jumper configuration to achieve low-power using the
MSPM0 LaunchPad, please visit the [LP-MSPM0G3507 User's Guide](https://www.ti.com/lit/slau873).

## Example Usage
Make the following connections between the SPI Controller and SPI Peripheral:
- Controller SCLK -> ADC SCLK
- Controller PICO -> not used
- Controller POCI <- ADC SDO
- Controller CS   -> ADC CS

Compile, load and run the example.

Ensure that the SPI Peripheral example is running **before** starting the SPI Controller example.
Once the example is started, the SPI Controller will automatically start to transmit data every 1s.
The transmitted data packet is {'M', 'S', 'P', 'x'}, where 'x' starts at '0' and will increment with each new transfer. LED0 and USER_TEST pin will toggle every time a new transmission starts.

An optional SW breakpoint can be uncommented in the application to check the results of the received data.
If this example is used with the spi_peripheral_repeated_multibyte_fifo_dma_interruptsexample, the expected data that will be received in gRxPacket is {'x', 0x2, 0x3, 0x4}, where 'x' starts at 0x0 and should increment every time the Peripheral example sends a new data packet.

The SPI is initialized with the configuration selected by #define ADC_PART_NUMBER