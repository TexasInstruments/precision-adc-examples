PGA3xx/9xx Calibration Coefficient Calculator
=============================================

This module provides a comprehensive algorithm for computing calibration coefficients for Texas Instruments PGA3xx and PGA9xx programmable gain amplifiers. The tool generates polynomial regression coefficients that define the transfer functions mapping temperature (TADC) and pressure (PADC)measurements to specific DAC output codes.


### Key Features

- **Z-score normalization**: Prevents numerical instability in polynomial regression
- **Multiple configurations**: Supports upto 4 temperature and 4 pressure (4T4P) calibration points
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
#        P1:   P2:   P3:   P4:
#   T1: [T1P1, T1P2, T1P3, T1P4],
#   T2: [T2P1, T2P2, T2P3, T2P4],
#   T3: [T3P1, T3P2, T3P3, T3P4],
#   T4: [T4P1, T4P2, T4P3, T4P4],

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
    device="PGA305",
    tad_matrix=tadc,
    pad_matrix=padc,
    dac_matrix=dac,
)
cc.recommend_calibration(offset_enabled=False)
cc.normalize_data()
cc.calculate_regression()
cc.summarize_results()
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
h0         7.674201e-01     0x311D69
h1         1.179603e-01     0x078CA9
h2         1.184552e-01     0x0794C5
h3        -5.168057e-02     0xFCB144
g0         7.534146e-01     0x3037F2
g1         2.391270e-01     0x0F4DDB
g2        -2.489039e-01     0xF011F6
g3        -6.265397e-02     0xFBFD7A
n0        -5.240755e-01     0xDE758C
n1        -6.399355e-01     0xD70B4C
n2        -3.645092e-01     0xE8ABE1
n3        -1.370017e-01     0xF73B5D
m0        -1.423722e-01     0xF6E360
m1        -2.144665e-01     0xF2462E
m2        -1.505616e-01     0xF65D33
m3        -1.324598e-01     0xF785C7

Calibration Point Comparison:
Point TADC (Hex)   PADC (Hex)   Expected   Computed   Error (codes)
-------------------------------------------------------------------
t1p1  0x3243B3     0xF585B6     0x0666     0x0666        -0.0031
t1p2  0x324991     0x1146C8     0x1FFF     0x1FFF        -0.0012
t1p3  0x324B34     0x397173     0x3998     0x3998         0.0001
t1p4  0x3247F2     0x574F0C     0x3FFF     0x3FFF         0.0021
t2p1  0x38C14B     0xF8434C     0x0666     0x0666        -0.0025
t2p2  0x38CD8B     0x125217     0x1FFF     0x1FFF        -0.0010
t2p3  0x38D8ED     0x38020D     0x3998     0x3998        -0.0000
t2p4  0x38D326     0x5411B3     0x3FFF     0x3FFF         0.0015
t3p1  0x53A5DC     0xFE9E3E     0x0666     0x0666        -0.0016
t3p2  0x53C289     0x1328D1     0x1FFF     0x1FFF        -0.0012
t3p3  0x53E7A3     0x30FDB3     0x3998     0x3998        -0.0007
t3p4  0x5408B2     0x474B08     0x3FFF     0x3FFF        -0.0001
t4p1  0x619158     0xFFF43F     0x0666     0x0666        -0.0018
t4p2  0x619E32     0x125D8A     0x1FFF     0x1FFF        -0.0014
t4p3  0x61A6D2     0x2D2411     0x3998     0x3998        -0.0011
t4p4  0x61AD6D     0x4134DA     0x3FFF     0x3FFF        -0.0007

Error Statistics:
  Max (Abs.) Error:       0.0031 codes  (    0.21 ppm)
  Mean (Abs.) Error:      0.0013 codes  (    0.09 ppm)
```


## Troubleshooting

- **Import errors**: Ensure python dependencies are installed and the virtual environment has been activated (if applicable)
- **Data formatting**: Ensure that input matrices match the specified configuration (e.g. cal_point=(4, 4)) and that data does not exceed specifed ADC resolution.
