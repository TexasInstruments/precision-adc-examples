# Thermocouple Conversion Library

Converts a measured thermocouple voltage (e.g. from an ADS111x ADC) into a temperature, with cold junction compensation (CJC).

| File | Purpose |
|------|---------|
| `thermocouple.c` | Conversion routines (polynomial and look-up-table implementations) |
| `inc/thermocouple.h` | Public API, `TC_Type` enum, coefficient data structures, method selection |
| `inc/thermocouple_type[B,E,J,K,N,R,S,T].h` | NIST ITS-90 polynomial coefficients (T→mV and mV→T) |
| `inc/thermocouple_type[C,D,G,M,P].h` | Polynomial coefficients, T→mV only |
| `inc/thermocouple_tables.h` | 1 °C-step look-up tables (B, E, J, K, N, R, S, T) for the LUT method |

## Supported types

| Method | Types | Notes |
|--------|-------|-------|
| `POLYNOMIAL` | B, E, J, K, N, R, S, T | Full support, including `Thermocouple_Linearization_and_CJC()` |
| `POLYNOMIAL` | C, D, G, M, P | Enum values and T→mV coefficients exist, but there is no mV→T set, so they are **not** accepted by `Thermocouple_Linearization_and_CJC()` (returns `NAN`) |
| `LUT` | B, E, J, K, N, R, S, T | C, D, G, M, P are not available |

Sources: NIST ITS-90 (`srdata.nist.gov/its90/download/allcoeff.tab`) for B/E/J/K/N/R/S/T; ASTM E1751 for G/M/P; Omega z202 for C/D.

## Selecting the conversion method

In `inc/thermocouple.h`, enable exactly one:

```c
#define POLYNOMIAL          // less memory, more compute
//#define LUT               // more memory, less compute
```

- **POLYNOMIAL** evaluates the NIST polynomial for the matching range. Ranges are looked up by value (°C for T→mV, mV for mV→T).
- **LUT** linearly interpolates a table with one entry per °C, using a binary search for mV→T.

## API

```c
float Thermocouple_Linearization_and_CJC(TC_Type TCtype, float TCmV, float CJtemp);
float Thermocouple_Temp2mV(<coeff set> *tcSet, float temp);   // °C -> mV
float Thermocouple_mV2Temp(<coeff set> *tcSet, float mV);     // mV -> °C
```

`<coeff set>` is `thermoPolyCoefficientSet` (POLYNOMIAL) or `thermoTableCoefficientSet` (LUT).

### `Thermocouple_Linearization_and_CJC`

| Parameter | Description |
|-----------|-------------|
| `TCtype` | `B, E, J, K, N, R, S, T` |
| `TCmV` | Measured thermoelectric voltage (mV) |
| `CJtemp` | Cold junction temperature (°C), e.g. from an on-board sensor |

Returns the thermocouple temperature in °C, or `NAN` if the type is unsupported or a value is outside the valid range.

Steps: (1) convert `CJtemp` to the cold junction's equivalent mV, (2) add it to `TCmV`, (3) convert the total mV to temperature.

### Example

```c
#include "inc/thermocouple.h"

float tcMilliVolts = 1.234f;   // from ADC
float boardTemp    = 25.0f;    // cold junction temperature

float tempC = Thermocouple_Linearization_and_CJC(K, tcMilliVolts, boardTemp);
if (isnan(tempC)) {
    // out of range or unsupported type
}
```

## Coefficient header layout

Each `thermocouple_typeX.h` defines, per direction:

- `const float` coefficient arrays, `c(0)` first, one array per range
- a `TC_TempRanges[]` giving `min`, `max`, `numCoeffs`, `coeffs` for each range
- a `thermoPolyCoefficientSet` (`typeX_T2mV`, `typeX_mV2T`) tying them together

Type K, for example: T→mV has two ranges (-270–0 °C, 0–1372 °C); mV→T has three (-5.891–0, 0–20.644, 20.644–54.886 mV).

To add a type: create the header with the same layout, add an entry to `TC_Type`, include it in `thermocouple.c`, and add it to the `switch` statements in `Thermocouple_Linearization_and_CJC()`.


## License

BSD 3-clause, Copyright (c) 2026 Texas Instruments Incorporated.
