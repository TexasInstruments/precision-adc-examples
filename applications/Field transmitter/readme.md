
# ADS122S14 Dual ADC Firmware - Developer File Guide

**User's Guides:**  
For hardware configuration and electrical specifications for the SNSR-DUAL-ADC-EVM refer to the full [User's Guide](https://www.ti.com/lit/ug/slvudh7/slvudh7.pdf)

**Other Useful documentation**

For more information about the ADS122S14 see the documentation visit: https://www.ti.com/product/ADS122S14

For more information about the MSPM0G1507 or to access the software development kit (SDK) visit: https://www.ti.com/product/MSPM0G1507

For more information about the 4mA to 20mA output board or the system see the documentation of [TIDA-010982](https://www.ti.com/tool/TIDA-010982)

## Overview

This document provides a comprehensive overview of the firmware file structure, focusing on what developers can modify and customize for their specific applications. 
This firmware is a **universal field transmitter platform** for industrial process control applications. It reads sensor data (pressure, temperature, flow, level, etc.), applies calibration and compensation, and outputs industry-standard signals (4-20mA, 0-10V, HART, IO-Link). The architecture is designed for flexibility, allowing easy substitution of components without affecting the overall system. This firmware is designed to support the input and output boards in the table below which is subject to change as addtional boards are designed into the ecosystem.  


**Supported Input and Output Boards** 

| Supported Input Boards  | Description              |
|:-----------------------:|:------------------------:|
| SNSR-DUAL-ADC-EVM       | 2xADS122S14 + MSPM0G1507 |
|                         |                          |

| Supported Output Boards  |           Description              |
|:------------------------:|:----------------------------------:|
|      TIDA-010982         | 4mA to 20mA loop-powered interface |
|                          |                                    |

---

## Architecture and Data Flow

Sensor applications typically require ADC communication (SPI, timing, DMA), sensor-specific calibration math, output range scaling, and hardware output control (DAC, protocols). Combining all these functions into one monolithic file makes it difficult to reuse code for different sensors, test individual components in isolation, modify one part without breaking others, and understand the overall code flow. This firmware addresses these challenges through a modular pipeline architecture that separates hardware communication, signal processing, and output control into independent, interchangeable stages.

The firmware implements a modular signal processing pipeline that transforms raw sensor data into calibrated output values. 
```
┌──────────┐    ┌──────────────┐    ┌──────────────┐    ┌──────────┐
│   ADC    │───>│    Input     │───>│    Output    │───>│  Output  │
│  Driver  │    │ Conditioning │    │ Conditioning │    │  Driver  │
└──────────┘    └──────────────┘    └──────────────┘    └──────────┘
```
- **ADC Driver**: Communicates with ADS122S14 hardware, returns raw 24-bit counts
- **Input Conditioning**: Applies sensor calibration and temperature compensation, outputs engineering units (PSI, °C, etc.)
- **Output Conditioning**: Scales engineering values to DAC range using slope/offset
- **Output Driver**: Writes to AFE881/882 DAC or IO-Link, generates physical output 
---
## File Structure Overview
```
├── main.c # Main application - rarely modified
├── identification.c/h # Driver selection - modify to add drivers
│
├── system/ # Core system files - rarely modified
│   ├── clock.c/h # Clock and timing functions
│   ├── flash.c/h # Configuration storage
│   ├── uart.c/h # Command interface
│   └── system.c/h # System utilities
│
├── adc/ # ADC driver modules - hardware selected 
│   ├── ads122s14_ptx_daisychain.c/h # Production driver (daisy-chain)
│   ├── adc_example.c/h # Simulation/testing driver (default)
│   └── ads122s14.c/h # Template/stub driver
│
├── condition/ # Signal conditioning modules
│   ├── adc_condition_example.c/h # Pass-through (no processing)
│   ├── adc_condition_pressure_temp.c/h # Pressure/temp compensation
│   ├── out_condition_example.c/h # Pass-through output
│   └── out_condition_offset_slope_uint16.c/h # Linear scaling
│
└── output/ # Output driver modules - hardware selected
    ├── output_afe881.c/h # AFE881/882 DAC driver
    ├── output_io-link.c/h # IO-Link output
    └── output_example.c/h # Simulation/testing drive 
```

---

## Core Files (Rarely Modified)

### `main.c`
**Purpose:** Main application loop and initialization sequence

**What it does:**
- Initializes all system components
- Runs the main processing loop
- Calls cyclic functions for each module
- Handles the data pipeline: ADC → Input Conditioning → Output Conditioning → Output

**When to modify:**
- Changing the main loop timing (SysTick configuration)
- Adding additional processing steps
- Modifying the data flow pipeline

### `identification.c/h`
**Purpose:** Maps hardware IDs to driver implementations

**What it does:**
- Defines which ADC driver is active
- Defines which output driver is active
- Provides driver name strings for the `version` command

**When to modify:**
- Adding a new ADC driver 
- Adding a new output driver 
- Changing which driver is compiled into the firmware 

**To add a new driver:**
1. Add entry to the appropriate array
2. Update the count variable (AdcDriverCnt or OutputDriverCnt)
3. Include the new driver's header file
---
## System Files (Rarely Modified)

The "system" folder contains the system control files such as the system clock, flash, UART communication and the terminal system control functions.

### `system/clock.c/h`
**Purpose:** Clock configuration and timing functions

**What it contains:**
- cpu_clock_init_80m() - Set system to 80 MHz
- cpu_clock_init_32m() - Set system to 32 MHz (default)
- cpu_clock_init_4m() - Set system to 4 MHz (low power)
- delay_high_res_us() - Active mode microsecond delay
- delay_us_standby() - Low-power mode microsecond delay with WFI
- g_system_clk_frequency_mhz - Global clock frequency variable

**When to modify:**
- Adding custom clock speeds
- Modifying delay function implementations
- Adjusting UART baud rates for different clock speeds

**DO NOT modify unless:**
- You need custom clock configurations
- You're optimizing power consumption
- You're adding custom timing peripherals

### `system/flash.c/h` 

**Purpose:** Non-volatile configuration storage

**What it contains:**
- config_init() - Load configuration from flash to RAM
- config_save() - Save configuration from RAM to flash
- config_reset() - Erase configuration (reset to defaults)
- config_dump() - Display configuration via UART
- crc8() - CRC validation for configuration integrity

**Configuration memory layout:**

char gAdcConfigRam[256];        // ADC driver configuration\
char aAdcCondConfigRam[256];    // Input conditioning configuration\
char gOutCondConfigRam[256];    // Output conditioning configuration\
char gOutputConfigRam[256];     // Output driver configuration

**When to modify:**
- Changing configuration sector size
- Adding new configuration sections
- Modifying CRC algorithm

**DO NOT modify unless:**
- You need more/less configuration space
- You're adding new configurable modules

### `system/uart.c/h`

**Purpose:** UART command interface and routing

**What it contains:**
- uart_init() - Initialize UART peripheral
- uart_printf() - Printf-style output to UART
- uart_handle_cmd() - Parse and route commands
- uart_cmd_init() - Initialize command table
- Command routing table

**Command Routing**
```
struct command commands[] = {
    {"sys",  system_cmd},           // System commands
    {"adc",  adc_cmd},              // ADC driver commands
    {"out",  output_cmd},           // Output driver commands
    {"cin",  condition_adc_cmd},    // Input conditioning commands
    {"cout", condition_out_cmd},    // Output conditioning commands
    {"help", help_cmd},             // Help command
    {"version", version_cmd}        // Version info
};
```
**When to modify:**
- Adding new top-level command categories
- Changing UART settings (baud rate, pins)
- Modifying command buffer size

**DO NOT modify unless:**
- You're adding new command subsystems
- You need different UART configuration

### `system/system.c/h`
**Purpose:** System-level utilities and commands

**What it contains:**
- system_cyclic() - Called every main loop iteration
- system_cmd() - Handle sys commands
- SysTick_Handler() - 1ms system tick interrupt
- gSysTick - Global millisecond counter
- gMainLoop - Flag to trigger main loop execution

**System commands:**
- sys help - Show system commands
- sys saveenv - Save configuration to flash
- sys dumpenv - Display configuration
- sys resetenv - Reset configuration
- sys load - Show processing time
- sys reset - Restart MCU

**When to modify:**
- Adding new system-level commands
- Changing main loop timing (modify SysTick configuration)
- Adding system monitoring features
---
## ADC Drivers 

The "adc" folder contains the files relevent to the function and configuration of the on board ADC(s).\
The identification.c file will read the sensor data aquisition board to determine the software configuraiton for the hardware board.  

### `adc/ads122s14_ptx_daisychain.c/h` (SNSR-DUAL-ADC-EVM Primary Driver)
**Purpose:** Production driver for dual ADS122S14 in daisy-chain

**What it does:**
- Communicates with two ADS122S14 ADCs via SPI
- Implements DMA-based non-blocking transfers
- Provides adaptive timing based on ADC configuration
- Manages bridge excitation control
- Implements low-power duty cycle mode

**ADC commands:**
- adc help - Show adc commands
- adc get - return one raw reading
- adc stream - starts/stops streaming raw readings
- adc rreg - Read register from both ADCs
- adc wreg [1|2] - Write register to specific ADC
- adc wreg - Write same register to both ADCs
- adc start - Start conversions
- adc stop - Stop conversions
- adc reset - Reload default firmware configuration
- adc bridgectrl [on|off] - Control bridge excitation
- adc lowpwr [ms|stop] - Low-power duty cycle mode
- adc cm_test - Run manufacturing test

**When to modify:**
- Changing default ADC register settings
- Modifying bridge excitation timing
- Adjusting low-power mode behavior
- Customizing manufacturing test limits

**Common Modifications**
1. Change default register values 
2. Add custom ADC commands 

### `adc/adc_example.c/h` 
**Purpose:** Simulation driver for testing without hardware

**What it does:**
- Simulates ADC readings without real hardware
- Provides static values or counting sequences
- Useful for software development and testing

**Commands:**
- adc sim static <value> - Set static value
- adc sim up <start> <step> <interval> - Counting mode
- adc get - Read current simulated value

**When to modify:**
- Adding new simulation modes (ramp, sine wave, noise, etc.)
- Creating test sequences
- Simulating sensor behavior

### `adc/ads122s14.c/h`
**Purpose:** Empty template for creating custom ADC drivers

**What it contains:**
- Function stubs that return STATUS_OK
- Template structure for implementing new drivers

**When to use:**
- Starting point for a new ADC driver
- Interfacing with different ADC hardware
- Creating custom acquisition logic
---
## Input Conditioning Modules 

The "condition" folder contains the ADC condition files, the output conditioning files, and the pressure and temperature condtioning files. 

### `condition/adc_condition-example.c/h`
**Purpose:** Pass-through (no processing)

**What it does:**
- Converts int32_t ADC reading to float
- No actual conditioning applied
- Useful for testing or when no conditioning is needed

**When to use:**
- Testing the signal chain
- When ADC output doesn't need processing
- As a template for custom conditioning

### `condition/adc_condition_pressure_temp.c/h` (Commonly Modified)
**Purpose:** Pressure sensor temperature compensation

**What it does:**
- Implements 3rd-order polynomial compensation
- Corrects pressure readings for temperature effects
- Uses normalized coefficients for fixed-point math

**Compensation algorithm:**
```
Output = h(T) + g(T)·P + n(T)·P² + m(T)·P³

where:
  h(T) = h₀ + h₁·T + h₂·T² + h₃·T³
  g(T) = g₀ + g₁·T + g₂·T² + g₃·T³
  n(T) = n₀ + n₁·T + n₂·T² + n₃·T³
  m(T) = m₀ + m₁·T + m₂·T² + m₃·T³
```
**Coefficient storage:**
```
// All coefficients normalized to 2^30 (1073741824)
static int32_t h0 = 27340;
static int32_t h1 = 0;
static int32_t h2 = 0;
static int32_t h3 = 0;

static int32_t g0 = 18135484;
static int32_t g1 = 0;
// ... continue for g2, g3, n0-n3, m0-m3

// Offsets applied before normalization
static int32_t P_offset = -1769446;
static int32_t T_offset = 0;
```
**When to modify:**
- ALWAYS - These coefficients are sensor-specific
- After sensor calibration
- When changing sensor types
- To improve accuracy
---
## Output Conditioning Modules 

The "condition" folder contains the ADC condition files, the output conditioning files, and the pressure and temperature condtioning files. 

### `condition/out_condition_example.c/h`
**Purpose:** Pass-through (no processing)

**What it does:**
- Converts float to int32_t
- No scaling or offset applied

**When to use:**
- When input conditioning provides final value
- Testing the output chain
- No additional scaling needed

### `condition/out_condition_offset_slope_uint16.c/h` (Commonly Modified)
**Purpose:** Linear scaling with offset and slope

**What it does:**
- Applies linear transformation: output = input × slope + offset
- Clamps output to 16-bit range (0-65535)
- Stores configuration in flash

**Commands:**
- cout offset [value] - Get/set offset
- cout slope [value] - Get/set slope

**When to modify:**
- Changing output range (e.g., 12-bit instead of 16-bit)
- Adding non-linear scaling
- Implementing unit conversions
---
## Output Drivers

The "output" folder contains the output board configuration based on the output board hardware connected. 

### `output/output_afe881.c/h`

**Purpose:** AFE881/882 DAC output driver

**What it does:**
- Controls AFE881/882 DAC via SPI
- Supports HART communication (if enabled)
- Manages DAC registers and configuration

**When to modify:**
- Changing DAC configuration
- Adding HART protocol implementation
- Modifying SPI timing

### `output/output_io-link.c/h`

**Purpose:** IO-Link output driver

**What it does:**
- Implements IO-Link device stack
- Manages TIOL112 transceiver
- Handles COM1/COM2/COM3 modes
- Controls Pin 2 configuration

**When to modify:**
- Customizing IO-Link behavior
- Adding custom process data handling
- Modifying Pin 2 functionality

**Only modify if:**
- You're using IO-Link output
- You need custom IO-Link features

### `output/output_example.c/h`
**Purpose:** Simulation driver for testing

**What it does:**
- Stores output value in memory
- No actual hardware interaction
- Useful for software testing

**When to use:**
- Testing without output hardware
- Software development
- Creating custom output drivers
---


## Additional Resources

**Application Notes:**
* [A Basic Guide to Bridge Measurements](https://www.ti.com/lit/an/sbaa532a/sbaa532a.pdf) 
* [A Basic Guide to RTD Measurements](https://www.ti.com/lit/an/sbaa275a/sbaa275a.pdf) 
* [A Basic Guide to Thermocouple Measurements](https://www.ti.com/lit/an/sbaa274a/sbaa274a.pdf) 


## Revision History 
**1.0.1 Initial Release** 
- Build Date: See version command
