PGA3xx/9xx Calibration Coefficient Calculator
=============================================

This module provides a comprehensive algorithm for computing calibration coefficients for Texas Instruments PGA3xx and PGA9xx programmable gain amplifiers. The tool generates polynomial regression coefficients that define the transfer functions mapping temperature (TADC) and pressure (PADC)measurements to specific DAC output codes.


### Key Features

- **Z-score normalization**: Prevents numerical instability in polynomial regression
- **Multiple configurations**: Supports 1P1T to 4P4T calibration configurations
- **ADC resolution options**: Handles both 16-bit and 24-bit ADC resolutions
- **EEPROM Scaling**: Provides integer-scaled coefficients for device storage
- **Error analysis**: Determines prediction error


## Prerequisites

Requires Python >=3.11 and numpy >= 2.3.1. This script uses inline script metadata (PEP 723) to declare its dependencies, allowing compatible package managers to automatically install and run it.


## Running the Script

### Direct execution (recommended):
Package managers that support PEP 723 inline scripts can run the script directly:

```bash
# Using UV (automatically installs numpy if needed)
uv run pga_coefficient_calculator.py

# Using pipx (if available)
pipx run pga_coefficient_calculator.py
```

### Manual installation and execution:
To install the dependencies into your enviromnment:

**Using pip:**
It is recommended to first create a virtual environment. Refer to [Install packages in a virtual environment using pip and venv](https://packaging.python.org/en/latest/guides/installing-using-pip-and-virtual-environments/) for instructions.
```
pip install "numpy>=2.3.1"
python pga_coefficient_calculator.py
```

**Using UV:**
UV handles virtual environment setup automatically.
```
uv init --bare
uv add numpy>=2.3.1
uv run python pga_coefficient_calculator.py
```


### How to use this script

Either modify the code below the `if __name__ == "__main__":` section directly in `pga_coefficient_calculator.py` OR create a new script in the same folder and import the **PGACoeffCalculator** class (e.g. `from pga_coefficient_calculator import PGACoeffCalculator`), then call the functions in the order shown below...
- Update the `cal_point=(T, P)` arguments as needed, where `T` is number of temperature points and `P` is number of pressure points, corresponding to the number of rows and columns of the data matrices (respectively).
- Update the matrices with the corresponding number of data points.


#### Input example

```python
# Matrix structure (4T4P)
# Omit the unused rows/columns for fewer calibration points.
#
#        P0:   P1:   P2:   P3:
#   T0: [T0P0, T0P1, T0P2, T0P3],
#   T1: [T1P0, T1P1, T1P2, T1P3],
#   T2: [T2P0, T2P1, T2P2, T2P3],
#   T3: [T3P0, T3P1, T3P2, T3P3],

# Example 4T4P data
tadc = [
    [0x3243B3, 0x324991, 0x324B34, 0x3247F2],
    [0x38C14B, 0x38CD8B, 0x38D8ED, 0x38D326],
    [0x53A5DC, 0x53C289, 0x53E7A3, 0x5408B2],
    [0x619158, 0x619E32, 0x61A6D2, 0x61AD6D],
]

padc = [
    [0xF585B6, 0x1146C8, 0x397173, 0x574F0C],
    [0xF8434C, 0x125217, 0x38020D, 0x5411B3],
    [0xFE9E3E, 0x1328D1, 0x30FDB3, 0x474B08],
    [0xFFF43F, 0x125D8A, 0x2D2411, 0x4134DA],
]

dac = [
    [0x666, 0x1FFF, 0x3998, 0x3FFF],
    [0x666, 0x1FFF, 0x3998, 0x3FFF],
    [0x666, 0x1FFF, 0x3998, 0x3FFF],
    [0x666, 0x1FFF, 0x3998, 0x3FFF],
]

cc = PGACoeffCalculator(
    cal_point=(4, 4),  # 4T4P
    adc_resolution=24,  # Use 24 for PGA305, 16 for PGA300
    tad_matrix=tadc,
    pad_matrix=padc,
    dac_matrix=dac,
)
cc.recommend_calibration(offset_enabled=False)

# (OPTIONAL) Override calibration settings here...
# cc.tadc_gain = 1
# cc.padc_gain = 1

cc.normalize_data()
cc.calculate_regression()
cc.summarize_results()

# To test the DAC output for different TADC and PADC values:
dac_output = cc.compute_dac_value(tadc_value=0x3243B3, padc_value=0xF585B6)
print(f"DAC output: {dac_output} (Hex: 0x{cc.signed_int_to_hex(dac_output)})")
```

#### Output example

The algorithm solves the polynomial equation to find coefficients that result in output DAC values that closely match with the desired output values for the specified input temperature and pressure combinations and shows the equivalent hex values to write to the device's EEPROM.

```
================================================================================
CALIBRATION SUMMARY - 4T4P Configuration
================================================================================

Calibration Settings:
Setting              Value          EEPROM (Hex)
------------------------------------------------
OFF_EN               0                      0x00
TADC_GAIN            1                  0x000001
TADC_OFFSET          -4847760           0xB60770
PADC_GAIN            1                  0x000001
PADC_OFFSET          -2517601           0xD9959F

Coefficients:
Name        Float Value   EEPROM (Hex)
--------------------------------------
h0         2.997735e-03     0x00311D
h1         4.607823e-04     0x00078D
h2         4.627157e-04     0x000795
h3        -2.018772e-04     0xFFFCB1
g0         2.943026e-03     0x003038
g1         9.340898e-04     0x000F4E
g2        -9.722807e-04     0xFFF012
g3        -2.447421e-04     0xFFFBFD
n0        -5.561416e-04     0xFFF6E3
n1        -8.377596e-04     0xFFF246
n2        -5.881312e-04     0xFFF65D
n3        -5.174212e-04     0xFFF786
m0        -2.047170e-03     0xFFDE76
m1        -2.499748e-03     0xFFD70B
m2        -1.423864e-03     0xFFE8AC
m3        -5.351629e-04     0xFFF73B

Calibration Point Comparison:
Point   TADC (Hex)   PADC (Hex)   Expected   Computed   Error
--------------------------------------------------------------
T0P0    0x3243B3     0xF585B6     0x000666   0x000666   0
T0P1    0x324991     0x1146C8     0x001FFF   0x001FFF   0
T0P2    0x324B34     0x397173     0x003998   0x003998   0
T0P3    0x3247F2     0x574F0C     0x003FFF   0x003FFF   0
T1P0    0x38C14B     0xF8434C     0x000666   0x000666   0
T1P1    0x38CD8B     0x125217     0x001FFF   0x001FFF   0
T1P2    0x38D8ED     0x38020D     0x003998   0x003998   0
T1P3    0x38D326     0x5411B3     0x003FFF   0x003FFF   0
T2P0    0x53A5DC     0xFE9E3E     0x000666   0x000666   0
T2P1    0x53C289     0x1328D1     0x001FFF   0x001FFF   0
T2P2    0x53E7A3     0x30FDB3     0x003998   0x003998   0
T2P3    0x5408B2     0x474B08     0x003FFF   0x003FFF   0
T3P0    0x619158     0xFFF43F     0x000666   0x000666   0
T3P1    0x619E32     0x125D8A     0x001FFF   0x001FFF   0
T3P2    0x61A6D2     0x2D2411     0x003998   0x003998   0
T3P3    0x61AD6D     0x4134DA     0x003FFF   0x003FFF   0

Error Statistics:
  Max Error:        0 codes  (   0.0 ppm FSR)
  Mean Error:    0.00 codes  (   0.0 ppm FSR)
```


## Troubleshooting

- **Import errors**: Ensure python dependencies are installed and the virtual environment has been activated (if applicable)
- **Data formatting**: Ensure that input matrices match the specified configuration (e.g. cal_point=(4, 4)) and that data does not exceed specifed ADC resolution.
