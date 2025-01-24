
# Introduction
This document details the communication setup and data collection between the MSPM0 and ADS127L18 devices.

All code is written using CCS Theia

# 1 MSPM0
## 1.1 SPI Introduction
This section discusses useful videos, labs, and tips to help learn about the MSPM0 serial peripheral interface (SPI) module. Additionally, review the TI Precision Labs training content discussing SPI communication basics: https://www.ti.com/video/6245520669001 
- Configuring the MSPM0 SPI communication using SYS CONFIG:
  - SYSCONFIG is a GUI that helps the user select the desired settings for the device, then creates a function to automatically write to the selected registers when the program begins
  - Configure the SPI by selecting “SPI” under the “Communications” dropdown on the left side

    ![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image.png?raw=true)

  - Basic Configuration:
    - Mode Select:
      - Defines the SPI port to be either a controller or a peripheral device. The user has the ability to control the SPI communication bit rate when the device is configured as a controller.
      - Review the TI Precision Labs video to understand the terminology of “controller” and “peripheral”.
    - Clock Configuration:
      - This defines the clocking configuration used by the SPI protocol
      - The user can review the ‘sysctl_mclk_syspll’ example to learn how to set up the clock tree (syspll) for the MSPM0 device for higher frequency operation
    - Frame Format:
      - This project uses Motorola 4-wire and Motorola 3-wire:
        - Motorola 4-wire format is explained in the TI Lab Precision video, and is used when the SPI peripheral communicates with the ADC i.e. the bit-banging method is not used
        - Motorola 3-wire is used to receive data from the ADS127L18. This format does not use the chip select signal. Instead, this protocol counts the number of SPI clocks based on the defined “Frame Size (bits)”. 
    - Clock Polarity and Phase
      - These terms are explained in the TI Lab Precision video
      - Use a logic analyzer or other digital communication tool to ensure that the controller outputs the correct clock phase and polarity for the ADS127L18
  - Interrupt Configuration
    - This section allows the user to enable interrupts based on certain actions that the MSPM0 SPI protocol completes.
  
    ![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-1.png?raw=true)

    ![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-2.png?raw=true)

    - The figure above is a code snippet from the ‘spi_controller_echo_interrupts’ example project 
      - An interrupt is generated every time the MSPM0 SPI port receives data
      - In the code, the IRQHandler function stores the received data into the variable gRxData, increments gTxData, and then toggles the LED.
      - All of the listed actions occur each time the MSPM0 SPI port receives data
  - Direct Memory Access (DMA) Configuration
    - DMA is a dedicated hardware feature that transfers data without CPU intervention
    - Review the ‘spi_peripheral_fifo_dma_interrupts’ MSPM0 example project for help understanding how to set up the DMA
  - PinMux (Peripheral and Pin Configuration)
    - Used to define the pin configuration for each SPI signal
## 1.2 MSPM0G3507
The MSPM0G3507 was used for this project, which has the following features:
- Frequency: 80MHz
- RAM: 32kB
- Number of SPI ports: 2
### 1.2.1 SPI Clock Frequency Limitation
The MSPM0G3507 operates at 3.3 V while the ADS127L18 IOVDD must operate at 1.8 V. To resolve this issue, a voltage translator board was built specifically for the MSPM0G3507 LaunchPad that reduces the M0 operating voltage from 3.3 V to 1.8 V. As a result, the MSPM0 has a maximum SPI clock frequency of 24 MHz. Using a different controller could potentially allow the user to increase the SPI clock frequency and therefore increase the data throughput

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-3.png?raw=true)

*Figure 3: MSPM0G350x Datasheet: Section 7.20.1*

# 2 ADS127L18
This section introduces important ADS127L18 features and behaviors that are necessary to understand this example code. All of this information can be found in the datasheet: https://www.ti.com/lit/ds/symlink/ads127l18.pdf
## 2.1 Frame-Sync Data Port
The ADS127L18 includes a frame-sync data port that outputs the channel conversion data. This port is a synchronous, read-only interface with synchronized output clock signals (FSYNC and DCLK) as well as up to 8 channel data (DOUTn) lanes. Enabling multiple lanes allows the user to clock out data in parallel, which increases throughput compared to strictly serial communication.

In this project, both MSPM0G3507 dedicated SPI peripherals are used to read in the data port signals using an SPI port in peripheral mode.

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-4.png?raw=true)

### 2.1.1 FSYNC signal
The previous figure shows how the FSYNC signal indicates the beginning and end of the data frame. The FSYNC frame period ($t_{DATA}$) is the modulator clock period ($t_{MOD}$) multiplied by the oversampling ratio (OSR). Since DCLK and $t_{DATA}$ are not necessarily correlated, it is possible that the FSYNC period can be longer than the actual time required to output all valid data. In this case, DOUT outputs additional zeroes** until the end of $t_{DATA}$ once all valid channel has been received. These additional zeroes must be ignored by the data processing algorithm

****NOTE**: this assumes the user enables TDM mode = 1, 2, or 4, and the DINx pins are grounded. If using TDM mode = 8, or the DP_DAISY bit is set to 1 in the DP_CFG1 register, then the additional data output on DOUT is repeated data. The rest of this document assumes the first set of conditions are met such that all additional data are zeroes

For example, assume the user configures the ADS127L18 with the following settings:
- DCLK = 24 MHz (41.7ns)
- $t_{MOD}$ = 62.5ns ($f_{MOD}$ = 16MHz, $f_{CLK}$ = 32MHz)
- OSR = 1024
- $t_{DATA}$ = 64µs
- TDM mode = 2 data output lanes
- Channel data length = 24 bits

The number of DCLK periods ($π_{DCLK}$) within a single FSYNC frame is given by this equation:

$$π_{DCLK} = t_{DATA} / t_{DCLK}$$

Therefore, $π_{DCLK}$ = 1536 using these settings. However, there are only 96 valid bits (4 channels * 24 bits) such that 1440 additional zeroes are output on DOUT during each FSYNC frame. This data must be ignored, and is removed by the parsing routine that is described later in this document

### 2.1.2 DCLK Pin
The DCLK pin is the bit-clock output signal that shifts data out from the frame-sync port. DOUT channel data are **updated on the falling edge** of DCLK and **latched by the host on the rising edge**.

After synchronization, DCLK idles low, then transitions high, then transitions low again to indicate that the first data bit will be output on DOUT on the next DCLK rising edge. However, since the DOUT channel data latches on the rising edge, the initial DCLK transition, from low-to-high, is read as the first rising edge by the SPI peripheral. This behavior causes the all of the data to be right shifted by one-bit when read by the SPI peripheral. This first bit must be ignored, and is removed by the parsing routine that is described later in this document.

The DCLK frequency is derived from the ADC clock through a programmable DCLK divider. Ensure that the DCLK signal frequency is fast enough to transmit the channel data within one conversion period (FSYNC clock period), otherwise data are lost. The following equation shows how to derive the minimum DCLK frequency:

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-5.png?raw=true)

Data rate calculation:

$$f_{DATA}=\frac{f_{CLK}}{2(CLK\_DIV)(OSR)}$$

Where:
- $f_{CLK}$ = ADC clock frequency
- CLK_DIV = ADC clock divider
- OSR = Oversampling Ratio

The DCLK frequency is limited to 24 MHz in this project because the MSPM0G3507 maximum SPI clock frequency is 24 MHz when the device is operating at 1.8V. The following equation is a representation of DCLK limitations when the ADS127L18 is used with the MSPM0G3507 device.

$$24\ MHz ≥ f_{DCLK} ≥ f_{DATA} * TDM\ factor * Data\ packet\ size$$

Using a different processor with a higher maximum SPI clock frequency at 1.8V allows the user to increase the ADS127L18 data rate, assuming the same TDM factor and data packet size.

## 2.2 SPI Programming
The ADS127L18 configuration registers are accessed through SPI communication. This project employs bit-banging instead of a dedicated SPI peripheral because both MSPM0 SPI peripherals are used to capture ADC data in this example.

More info on bit-banging: https://en.wikipedia.org/wiki/Bit_banging 

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-6.png?raw=true)

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-7.png?raw=true)

Additionally, the ADS127L18 has a MODE pin that allows the user to program the ADC using SPI or pin control. Pull the MODE pin up to IOVDD to enable SPI programming mode for the ADS127L18.

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-8.png?raw=true)

## 2.3 Register Map
This section highlights the registers used in this project, and the specific bits that were manipulated for each register.

- GEN_CFG2 Register (Address = 09h) [Reset = 04h]
  - The SPEED_MODE bits adjust the internal bias current to lower the power consumption. Use the correct CLK_DIV bits to scale the 25.6 MHz clock frequency to the desired speed mode when the internal oscillator is used. Apply the correct clock frequency stated in each SPEED_MODE bit setting when an external oscillator is used. Choosing a faster clock speed results in faster output data on the DOUT pins, but also increases the ADC current consumption

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-9.png?raw=true)
  
- DP_CFG1 Register (Address = 0Bh) [Reset = 20h]
  - The DP_TDM bits select the number of DOUT pins that provide channel data. This project uses two data output pins that are routed to the two SPI peripherals on the MSPM0. Using a controller with additional SPI peripherals allows the user to enable additional data output pins. More output pins allow more data to be clocked out in parallel, resulting in higher possible data throughput

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-10.png?raw=true)

- DP_CFG2 Register (Address = 0Ch) [Reset = 00h]
  - The DCLK_DIV bits sets the rate at which each data bit is output on the bus. Therefore, a larger DCLK divider results in slower DCLK speeds and therefore reduced throughput

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-11.png?raw=true)

- CLK_CFG Register (Address = 0Dh) [Reset = 00h]
  - The CLK_SEL bit chooses to use either the internal oscillator (default) or an external oscillator. The external oscillator must be a high-accuracy, low-jitter clock provided by the user
  - Use the correct CLK_DIV bits to scale the 25.6 MHz clock frequency to the desired speed mode when the internal oscillator is used. The CLK_DIV bits also further reduce the clock frequency when an external oscillator is used. Choosing a faster clock speed results in faster output data on the DOUT pins, but also increases the ADC current 

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-12.png?raw=true)
  
# 3 MSPM0 x ADS127L18
This section discusses the code that enables the communication and data collection between the MSPM0 and ADS127L18. 

**NOTE**: using a different controller can require a different set of configuration steps and code. Refer to the controller datasheet for more information about the integrated peripherals and how to configure them

## 3.1 High Level Overview
The diagram below shows how the MSPM0 and ADS127L18 devices interact with each other. The “Communication Subsystem” is captured in blue, while the “Data Collection Subsystem” is captured in orange.

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-13.png?raw=true)

As discussed previously, the ADS127L18 includes an SPI port for configuring the ADC and a frame-sync data port that provides data to the controller. This specific implementation uses the MSPM0 GPIO pins to bit-bang the ADC SPI and configure the device. Both MSPM0 SPI peripherals are then used to receive data in parallel once the ADC is configured. Finally, the captured data is transmitted to RAM using the MSPM0 DMA where it is processed

## 3.2 Code Walkthrough
This section provides a detailed code walkthrough.

Files:
- ads127l18.h
- hal.c
- hal.h
- MSPM0_x_ADS127L18.c
- MSPM0_x_ADS127L18.syscfg

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-14.png?raw=true)

### 3.2.1 SYSCFG_DL_init()
The function, SYSCFG_DL_init(), initializes the code defined by the MSPM0_x_ADS127L18.syscfg file.

GPIO:
- GPIO_L18: START GPIO (PA25):
  - Connects to the ADS127L18 START pin
  - Set low on initial code start up so to be able to write the ADC register map while keeping the frame-sync data port inactive
- GPIO_L18: RESET (PA8)
  - Connects ADS127L18 RESET pin
  - Initially low, this pin is set high after the M0 is initialized

SPI:

Both SPI ports are used for data collection:

- Initialize the two SPI ports on the MSPM0G3507 device (the configurations are nearly identical)
  - Peripheral Mode
  - Motorola 3-wire
  - Clock Polarity = Low
  - Clock Phase = Capture data on the rising edge 
  - DMA Configuration (DMA_CH0 & DMA_CH1)
  - Pin configuration
    - SPI_PERIPHERAL
      - SCLK = PA12
      - PICO = PB17
      - POCI = PA13
    - SPI_PERIPHERAL2
      - SCLK = PA17
      - PICO = PB15
      - POCI = PA16

Clock Tree:

This sets the CPUCLK to operate at 48MHz, which allows the SPI clock frequency to operate up to 24MHz.

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-15.png?raw=true)

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-16.png?raw=true)

### 3.2.2 WriteSingleRegister()
This function toggles the GPIO pins to mimic the SPI protocol and replicate the ADS127L18 write command. Additionally, the WriteSingleRegister() function calls the ReadSingleRegister() function to read back the register and confirm the register write was successful. This is shown in the figure below.

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-17.png?raw=true)

the first CS frame is a write register command, while the second and third CS frames are a read register command and the command response, respectively.

### 3.2.3 SPI_receieve()
The ADS127L18 frame-sync data port begins outputting data once the START pin is set high. The ADC clock frequency, data rate, DCLK frequency, and number of DOUT pins are defined by the user in the “user inputs” section.

The SPI_receive() function also configures each SPI port’s DMA to automatically transfer and store the received data to the MSPM0 RAM. 

Finally, this function pulls the START pin low and sets ‘data_received_flag’ true to start the parsing routine once the data transfer is complete

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-18.png?raw=true)

As mentioned previously, the MSPM0G3507 only has two dedicated SPI ports such that only two ADS127L18 DOUT pins can be used. Using a different processor with additional SPI ports (e.g. 8 SPI ports) could enable the user to dedicate each DOUT pin to an SPI port.

Using additional SPI peripherals enables higher data throughput because more data can be clocked out in parallel compared to using only two SPI peripherals.

### 3.2.4 Parsing Routine
The code parsing routine includes multiple functions and follows the flow shown here:

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-19.png?raw=true)

The next sections explore these functions in more detail

#### 3.2.4.1 “Left_shift_one_bit()” function:
As mentioned in the section discussing the DCLK pin, the ADS127L18 data port creates an extra DCLK rising edge. The SPI peripheral interprets this as valid data, causing all data to be right-shifted by one bit.

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-20.png?raw=true)

In the figure above, the L18 is outputting data from a negative full-scale test, therefore each data packet should be 0x800000. But, the extra DCLK rising edge causes all of the data to be right shifted one bit, resulting in data packets of 0x400000h (See table 7-14 in the ADS127L18 datasheet for the Data Format). The “left_shift_one_bit()” function properly aligns all of the data

#### 3.2.4.2 “parse_null_data()” function:
As mentioned in the section introducing the FSYNC signal, it is possible that the FSYNC period can be longer than the actual time required to output all valid data. In this case, DOUT outputs additional zeroes until the end of $t_{DATA}$ once all valid channel has been received. These additional zeroes must be ignored by the data processing algorithm.

The “parse_null_data()” function calculates the number of valid bits as well as the total number of DCLK pulses so that the total number of extraneous zeroes is known. These bits are then excluded from the rest of the data processing algorithm

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-21.png?raw=true)

#### 3.2.4.3 “insert_ch_id()” function:
As its name implies, this function converts the 24-bit data words into 32-bit values to fit into the MSMP0 memory space. However, the channel number is written into the first 8 bits instead of simply padding the data with all 1’s or all 0’s. This extra step helps execute the sorting routine later in the code because all of the channel data can be easily grouped together. This data is correctly sign-extended later in the code once the sorting algorithm has completed

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-22.png?raw=true)

Note the format of gRxPacket which now contains both the 24-bit ADC data plus the channel ID is now:

**[Channel ID] [Most significant byte] [Middle significant byte] [Least significant byte]**

For Example, a ADC reading of +FS from channel 2 will read 0x027FFFFF, while a -FS reading from channel 7 would read 0x07800000.

#### 3.2.4.4 “memory_cleaning()” function:
Some collected data cannot be processed due to the conversion from 24- to 32-bit data, which now occupies 33% more RAM. This function zeros out all of the leftover space after the 24-bit to 32-bit conversion

#### 3.2.4.5 “insertionSort()” function:
This function rearranges the data in the MSPM0 memory so that all ADC channel 0 data is in a contiguous memory space, all ADC channel 1 data is in a contiguous memory space, and so on. As mentioned previously, this function is simplified by including the channel number in the previous processing step (“insert_ch_id()”)

#### 3.2.4.6 “signExtend()” function
This function properly sign extends the 24-bit data received from ADS127L18, overwriting the channel data that was included in the “insert_ch_id()” function. The first 8 bits of the overall 32-bit data words are either all 1’s or all 0’s because the ADS127L18 data is output in a binary two’s complement format

# 4 Hardware Setup
## 4.1 MSPM0 Voltage Translator
The MSPM0G3507 operates at 3.3V while the ADS127L18 operates at 1.8V. Both devices need to operate at the same voltage level in order for the communication to work.

To solve this issue, a voltage translator board was built that connects to the debug header (J101) on the MSPM0G3507 LaunchPad. This translator board employs a simple LDO to reduce the MSPM0 operating voltage from 3.3V to 1.8V. The figure below shows the voltage translator board (green PCB) connected to the MSMP0 Launchpad (red PCB)

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-23.png?raw=true)
![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-26.JPG)


  
## 4.2 Connecting the ADS127L18EVM to the MSPM0 Launchpad
The following image shows how the MSMP0 Launchpad is connected to the ADS127L18 EVM using the connections described in the “SYSCFG_DL_init” section

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-25.png?raw=true)

|EVM Signal| M0 Launchpad Pin|
|----------|-----------------|
|/Reset    |PA8              |
|START     |PA25             |
|MODE      |+VDD ( 1.8V)     |
|DCLK      |PA12 AND PA17    |
|DOUT1     |PB15             |
|DOUT0     |PB17             |
|SDO       |PB7              |
|SDI       |PB8              |
|SCLK      |PB9              |
|/CS       |PB6              |
|GROUND    |GROUND           |


## 4.3 ADS127L18 $\overline{RESET}$ Pin
The EVM includes a pulldown resistor connected to the $\overline{RESET}$ (Not RESET) pin. This holds the ADC in reset until the EVM GUI takes control and brings this pin high. The schematic connections are shown in the image below

![Alt text](https://github.com/TexasInstruments/precision-adc-examples/blob/main/devices/ads127l18/MSPM0_example/docs/image-24.png?raw=true)

