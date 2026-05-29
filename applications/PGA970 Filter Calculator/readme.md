# PGA970 Filter Coefficient Generation Tools

This repository contains MATLAB scripts for generating Low Pass Filter (LPF) and Bandpass Filter (BPF) coefficients for the **PGA970 programmable gain amplifier with integrated demodulator**. The scripts automate the calculation of Butterworth filter coefficients and convert them to the fixed-point integer format required by the PGA970 hardware registers.

## Table of Contents

- [Quick Start](#quick-start)
- [Overview](#overview)
- [Low Pass Filter (LPF)](#low-pass-filter-lpf)
- [Bandpass Filter (BPF)](#bandpass-filter-bpf)
- [Workflow Guide](#workflow-guide)
- [Configuration Reference](#configuration-reference)
- [Understanding the Output](#understanding-the-output)
- [Troubleshooting](#troubleshooting)

## Quick Start

### Generate LPF Coefficients

1. Open `PGA970LPFcoeff.m` in MATLAB
2. Adjust configuration parameters (lines 11-17):
   - `cfStart` — Starting cutoff frequency (Hz)
   - `cfEnd` — Maximum cutoff frequency (Hz)
   - `cfStep` — Frequency step size (Hz)
3. Run the script: Press **Ctrl+Enter** or click **Run**
4. View the coefficient table in the **Command Window** and `lpf.txt`
5. Select a row number and set `selectedRow` (line 53) to visualize that filter
6. Read the hexadecimal values printed at the bottom — these go into your PGA970 registers

### Generate BPF Coefficients

1. Open `PGA970BPFcoeff.m` in MATLAB
2. Adjust configuration parameters (lines 11-17):
   - `centerFrequencies` — Target frequency (Hz)
   - `bandwidths` — Bandwidth(s) to evaluate (Hz)
3. Run the script
4. Select a row and read the hex values for `b1`, `a2`, and `a3`

---

## Overview

The PGA970 demodulator uses three filter coefficients to implement a 1st-order Low Pass Filter or a 2nd-order Bandpass Filter. These filters are critical for signal conditioning and noise rejection.

| Filter Type | Order | Registers | Data Type |
|-------------|-------|-----------|-----------|
| LPF | 1st | `DEMODx_LPF_B1`, `DEMODx_LPF_A2` | 16-bit signed |
| BPF | 2nd | `DEMODx_BPF_B1`, `DEMODx_BPF_A2`, `DEMODx_BPF_A3` | 24-bit / 23-bit signed |

The scripts use MATLAB's `butter()` function to design Butterworth filters, then scale the coefficients to fixed-point integers compatible with the PGA970 hardware.

---

## Low Pass Filter (LPF)

### Overview

The 1st-order Butterworth LPF is used to attenuate high-frequency noise after demodulation. The filter cutoff frequency and the data rate (set via downsample ratio) are independently configurable.

### Transfer Function

$$H(z) = \frac{b_1 z + b_1}{z + a_2}$$

Where:
- `b1` — Numerator coefficient (stored in register `DEMODx_LPF_B1`)
- `a2` — Denominator coefficient (stored in register `DEMODx_LPF_A2`)

### Hardware Registers

- **DEMODx_LPF_B1** — Numerator coefficient (16-bit signed, scaled by 2^15)
- **DEMODx_LPF_A2** — Denominator coefficient (16-bit signed, scaled by 2^15)

### Using the LPF Script

#### Step 1: Configure Parameters

Edit lines 11-17 in `PGA970LPFcoeff.m`:

```matlab
fSample = 1/(256e-6);           % Sample rate: 3906.25 Hz (fixed by ADC)
minDS = 0.5;                    % Minimum downsample rate
maxDS = 1;                      % Maximum downsample rate
cfStart = 20;                   % Starting cutoff frequency (Hz)
cfEnd = 1000;                   % Maximum cutoff frequency (Hz)
cfStep = 10;                    % Cutoff frequency increment (Hz)
```

**Parameter Notes:**
- **fSample**: Fixed by your ADC configuration. Current value assumes 256 µs ADC conversion time.
- **minDS / maxDS**: Downsample ratio determines the effective output data rate. Higher downsample = lower data rate, more filtering.
- **cfStart / cfEnd / cfStep**: Controls the range of frequencies generated. Larger step = faster computation, coarser table.

#### Step 2: Run the Script

The script generates `lpf.txt` containing a table where each row represents one filter configuration:

```
FilterNum  CutoffFreq  DS  OutputRate   b(1)      b(2)      a(1)  a(2)      b1      a2    cf/1000
    1         20      0.5     65.5    0.01559   0.01559   1.0  -0.96881   512     -31801   0.02
    2         20      1.0    131.1    0.00775   0.00775   1.0  -0.98437   254     -16160   0.02
    ...
```

#### Step 3: Select and Visualize

Set `selectedRow` (line 53) to the row you want to examine:

```matlab
selectedRow = 197;  % Change this number
```

The script plots:
1. **Frequency Response** — Shows the -3dB cutoff point and roll-off
2. **Step Response** — Shows settling time and overshoot

#### Step 4: Extract Hex Values

The script prints register values in both decimal and hexadecimal:

```
Selected Filter (Row 197):
  Cutoff Frequency: 160.0 Hz
  b1 (decimal): 6553
  a2 (decimal): -31801
  b1 (hex): 0x1999
  a2 (hex): 0xFFFF8427
```

Program these hex values into your PGA970 registers:
- `DEMODx_LPF_B1 = 0x1999`
- `DEMODx_LPF_A2 = 0xFFFF8427`

---

## Bandpass Filter (BPF)

### Overview

The 2nd-order Butterworth BPF is used to isolate a narrow frequency band from the demodulated signal. This is useful for recovering AM-modulated signals or filtering to a specific carrier frequency. The center frequency and bandwidth are independently configurable.

### Transfer Function

$$H(z) = \frac{b_1 z^2 - b_1}{z^2 - 2a_2 z + a_3}$$

Where:
- `b1` — Numerator coefficient (stored in register `DEMODx_BPF_B1`)
- `a2` — First denominator coefficient (stored in register `DEMODx_BPF_A2`)
- `a3` — Second denominator coefficient (stored in register `DEMODx_BPF_A3`)

### Hardware Registers

- **DEMODx_BPF_B1** — Numerator coefficient (24-bit signed, scaled by 2^24)
- **DEMODx_BPF_A2** — Denominator coefficient (23-bit signed, scaled by 2^23)
- **DEMODx_BPF_A3** — Denominator coefficient (24-bit signed, scaled by 2^24)

**Note:** Different scaling factors are used for each register to optimize precision and dynamic range for the PGA970 hardware.

### Using the BPF Script

#### Step 1: Configure Parameters

Edit lines 11-17 in `PGA970BPFcoeff.m`:

```matlab
fSample = 1000e3;               % Sample rate: 1 MHz (fixed by ADC)

% Single center frequency:
centerFrequencies = 12500;

% Or multiple frequencies (uncomment to use):
% centerFrequencies = [1500, 2500, 3500, 4500, 5500, 6500, 7500, 8500, ...
%                      9500, 10500, 11500, 12500, 13500, 14500, 15500, ...
%                      16500, 17500, 18500, 19500];

bandwidths = [20, 100, 250];    % Bandwidth(s) to evaluate (Hz)
```

**Parameter Notes:**
- **fSample**: 1 MHz for the BPF evaluation. This may differ from your actual ADC rate — adjust as needed.
- **centerFrequencies**: The frequency you want to isolate. Can be a single value or an array.
- **bandwidths**: The passband width (-3dB points). Narrower = more selective but slower settling time.

#### Step 2: Run the Script

The script generates `bpf.txt` with a table of all combinations:

```
FilterNum  CenterFreq  BW   b(1)      b(2)      b(3)      a(1)    a(2)      a(3)      b1          a2         a3        cf/1000  bw/1000
    1        12500     20  -0.00391  0.00000  -0.00391   1.0   -1.99997  0.99979  -103604288  -33619456  16775360      12.5      0.02
    2        12500     100 -0.00391  0.00000  -0.00391   1.0   -1.99996  0.99897  -103604288  -33619456  16775360      12.5      0.10
    ...
```

#### Step 3: Extract Hex Values

Set `selectedRow` (line 60) to choose a filter configuration:

```matlab
selectedRow = 3;  % Change this number
```

The script prints register values:

```
Selected Filter (Row 3):
  Center Frequency: 12500.0 Hz
  Bandwidth: 250.0 Hz
  b1 (decimal): -103604288
  a2 (decimal): -33619456
  a3 (decimal): 16775360
  b1 (hex): 0xF93F0000
  a2 (hex): 0xFE00A280
  a3 (hex): 0x010010C0
```

Program these into your PGA970 registers:
- `DEMODx_BPF_B1 = 0xF93F0000`
- `DEMODx_BPF_A2 = 0xFE00A280`
- `DEMODx_BPF_A3 = 0x010010C0`

---

## Workflow Guide

### Typical Design Process

1. **Determine your input signal characteristics:**
   - What is the ADC sample rate?
   - What bandwidth does your signal occupy?
   - What is the interfering noise frequency?

2. **Run the appropriate script** with parameters matching your requirements

3. **Examine the plots** to verify the frequency response meets your needs

4. **Extract the hex values** and program them into your PGA970

5. **Test on hardware** and adjust if needed (go back to step 3 with different parameters)

---

## Configuration Reference

### LPF Parameters

| Parameter | Range | Effect | Notes |
|-----------|-------|--------|-------|
| `cfStart` | 1–10000 Hz | Lowest frequency in table | Lower = more rows, slower script |
| `cfEnd` | cfStart–10000 Hz | Highest frequency in table | — |
| `cfStep` | 1–1000 Hz | Frequency resolution | Smaller = finer table, slower script |
| `minDS` | 0.1–10 | Lowest downsample ratio | Affects output data rate |
| `maxDS` | minDS–10 | Highest downsample ratio | — |

### BPF Parameters

| Parameter | Range | Effect | Notes |
|-----------|-------|--------|-------|
| `centerFrequencies` | 1–500 kHz | Frequencies to evaluate | Can be a scalar or array |
| `bandwidths` | 1–100 kHz | Bandwidth(s) to test | Narrower = more selective |

---

## Understanding the Output

### LPF Table Columns

```
[FilterNum, CutoffFreq, DS, OutputRate, b(1), b(2), a(1), a(2), b1, a2, cf/1000]
```

- **FilterNum**: Row index in the table
- **CutoffFreq**: Cutoff frequency in Hz
- **DS**: Downsample ratio
- **OutputRate**: Effective output data rate in microseconds
- **b(1), b(2)**: Floating-point numerator coefficients
- **a(1), a(2)**: Floating-point denominator coefficients
- **b1, a2**: Fixed-point register values (what you program into hardware)
- **cf/1000**: Cutoff frequency in kHz (for sorting/searching)

### BPF Table Columns

```
[FilterNum, CenterFreq, BW, b(1), b(2), b(3), a(1), a(2), a(3), b1, a2, a3, cf/1000, bw/1000]
```

- **FilterNum**: Row index in the table
- **CenterFreq**: Center frequency in Hz
- **BW**: Bandwidth in Hz
- **b(1), b(2), b(3)**: Floating-point numerator coefficients
- **a(1), a(2), a(3)**: Floating-point denominator coefficients
- **b1, a2, a3**: Fixed-point register values (what you program into hardware)
- **cf/1000, bw/1000**: Center frequency and bandwidth in kHz (for sorting/searching)

### Fixed-Point Scaling

Different bit-widths and scaling factors optimize each coefficient for hardware efficiency:

| Coefficient | Scaling | Bit-Width | Reason |
|-------------|---------|-----------|--------|
| LPF b1 | 2^15 | 16-bit | Numerator typically ≤ 1.0 |
| LPF a2 | 2^15 | 16-bit | Denominator coefficient near ±1.0 |
| BPF b1 | 2^24 | 24-bit | Needs higher precision |
| BPF a2 | 2^23 | 23-bit | Denominator needs careful scaling |
| BPF a3 | 2^24 | 24-bit | Denominator coefficient scaling |

---

## Troubleshooting

### MATLAB Errors

**Error: "Undefined function 'butter'"**
- **Cause**: Signal Processing Toolbox not installed
- **Solution**: Install the Signal Processing Toolbox. Run `tbx = matlab.addons.toolbox.installedToolboxes;` to check what you have.

**Error: "butter" produces warning about deprecated syntax**
- **Cause**: Older MATLAB version (<R2013a)
- **Solution**: Update MATLAB, or replace `butter(1, [lEdge, uEdge])` with `butter(1, [lEdge uEdge])`

### File Issues

**Cannot find `lpf.txt` or `bpf.txt` after running the script**
- **Cause**: Files saved to MATLAB's working directory, not the script directory
- **Solution**: Check `pwd` in the Command Window, or set working directory: `cd('C:\path\to\filter\scripts')`

**Cannot open files in Excel or text editor**
- **Cause**: Tab-delimited format may not display correctly
- **Solution**: Use MATLAB's `readtable()` or a text editor that supports tabs. In Excel, use **Data > Text to Columns** with Tab delimiter.

### Coefficient Issues

**Frequency response plot doesn't look right**
- **Cause**: `selectedRow` is pointing to the wrong configuration
- **Solution**: Try different row values. Look for your desired frequency in the table first.

**Hex values are negative or unexpectedly large**
- **Cause**: This is normal! Fixed-point scaling can produce large integers or negative (two's complement) values.
- **Solution**: Copy the hex values as shown — the PGA970 will interpret them correctly.

### Design Issues

**Filter response is too broad/narrow**
- **Cause**: Bandwidth parameter needs adjustment
- **Solution**: For BPF, reduce bandwidth for narrower passband. For LPF, reduce cutoff frequency.

**Filter settling time is too long**
- **Cause**: Too-narrow bandwidth or low cutoff frequency
- **Solution**: Increase cutoff frequency (LPF) or bandwidth (BPF) to trade off selectivity for speed.

---

## Requirements

- **MATLAB** (R2013a or later)
- **Signal Processing Toolbox** (for `butter`, `freqz`, `stepz` functions)

## References

- PGA970 datasheet — See register descriptions for `DEMODx_LPF_B1`, `DEMODx_LPF_A2`, `DEMODx_BPF_B1`, `DEMODx_BPF_A2`, `DEMODx_BPF_A3`
- MATLAB Butterworth Filter Design: `help butter`
- Butterworth Filter Theory: https://en.wikipedia.org/wiki/Butterworth_filter
