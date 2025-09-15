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
To install the depedencies into your enviromnment:

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

# Compute coefficients  
cc = PGACoeffCalculator(
    cal_point=(4, 4),   # (T, P)
    adc_resolution=24,
    tad_matrix=tadc,
    pad_matrix=padc,
    dac_matrix=dac,
)
cc.normalize_data()
cc.calculate_regression()
cc.validate_regression()
cc.calculate_eeprom_coefficients()
cc.summarize_results()

# Testing other
print("\n === Example single value approximation ===")
T, P = 0x3243B3, 0xF585B6
print(f"For T = 0x{T} and P = 0x{P}, DAC = 0x{cc.approximate_dac_value(T, P)}\n")
```

#### Output example

The algorithm solves the polynomial equation to find coefficients that result in output DAC values that closely match with the desired output values for the specified input temperature and pressure combinations. Additionally, the script shows the equivalent 24-bit hex values to write to the device's EEPROM and the estimated error after rounding.

```
Results:

Coeff    Rounded Float     EEPROM (Hex)
----------------------------------------
h0           2.851e-03         0x2EB585
h1           1.164e-04         0x01E85C
h2           4.757e-05         0x00C787
h3          -5.314e-06         0xFFE9B6
g0           1.535e-03         0x1924F0
g1           1.804e-04         0x02F4BD
g2          -3.781e-05         0xFF616C
g3          -2.751e-06         0xFFF476
m0          -4.704e-04         0xF84B0B
m1          -1.726e-04         0xFD2BE7
m2          -2.948e-05         0xFF8455
m3          -3.183e-06         0xFFF2A6
n0          -6.533e-05         0xFEEDFC
n1          -2.988e-05         0xFF82A9
n2          -6.102e-06         0xFFE668
n3          -1.756e-06         0xFFF8A2

Regression Equation: DAC =
 ( 2.851e-03 + 1.164e-04·T + 4.757e-05·T² - 5.314e-06·T³ ) +
 ( 1.535e-03 + 1.804e-04·T - 3.781e-05·T² - 2.751e-06·T³ ) · P +
 (-4.704e-04 - 1.726e-04·T - 2.948e-05·T² - 3.183e-06·T³ ) · P² +
 (-6.533e-05 - 2.988e-05·T - 6.102e-06·T² - 1.756e-06·T³ ) · P³

Requested DAC Values (0x):
   000666       001FFF       003998       003FFF
   000666       001FFF       003998       003FFF
   000666       001FFF       003998       003FFF
   000666       001FFF       003998       003FFF

Predicted DAC Values (0x):
   000665       001FFE       003997       003FFE
   000665       001FFE       003997       003FFE
   000665       001FFE       003997       003FFF
   000665       001FFE       003998       003FFF

Prediction Error: (ppmFSR)
   0.238        0.238        0.238        0.238
   0.238        0.238        0.238        0.238
   0.238        0.238        0.238        0.000
   0.238        0.238        0.000        0.000

 === Example single value approximation ===
For T = 0x3243B3 and P = 0xF585B6, the resulting DAC output = 0x000665
```


## Troubleshooting

- **Import errors**: Ensure python dependencies are installed and the virtual environment has been activated (if applicable)
