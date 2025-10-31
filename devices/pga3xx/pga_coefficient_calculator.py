"""
This module is used to compute the calibration coefficients for the PGA3xx temperature/pressure ADCs.
"""

# SPDX-License-Identifier: BSD-3-Clause
# Copyright (C) 2025 Texas Instruments Incorporated
#
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "numpy>=2.3.1",
# ]
# ///

import numpy as np

__version__ = "1.1.0"


class PGACoeffCalculator:
    """
    This class implements least squares regression to calculate calibration coefficients for PGA3xx pressure sensors.

    Inputs: Temperature ADC, Pressure ADC, DAC Output matrices
    Outputs: Polynomial coefficients, EEPROM hex values, calibration settings, model fit statistics

    Mathematical Foundation:
    Uses polynomial features where DAC = Σᵢⱼ cᵢⱼ * T^i * P^j
    - T, P: Normalized temperature and pressure data
    - cᵢⱼ: Polynomial coefficients (computed via least squares)
    - Normalization factor: 2^(adc_resolution-2)

    Features:
    - Input validation (matrix dimensions, ADC value ranges)
    - Support for both 16-bit and 24-bit ADC resolutions
    - Two's complement conversion for hex literals and strings
    - Automatic gain/offset calculation with configurable offset enable
    - Detailed calibration summary with settings table and error statistics
    """

    def __init__(
        self,
        cal_point: tuple[int, int],
        adc_resolution: int,
        tad_matrix: list[list[int | str]],
        pad_matrix: list[list[int | str]],
        dac_matrix: list[list[int | str]],
    ):
        """
        Initialize the PGACoeffCalculator class.

        Args:
            cal_point: Calibration configuration as (number temperature points, number pressure points).
            adc_resolution: ADC resolution in bits (e.g., 16, 24).
            tad_matrix: Temperature ADC readings as hex strings or integers.
            pad_matrix: Pressure ADC readings as hex strings or integers.
            dac_matrix: DAC output values as hex strings or integers.

        Raises:
            ValueError: If cal_point exceeds 4T4P configuration limits, matrix dimensions
                       don't match cal_point, or ADC values exceed resolution range.

        Note:
            - Input matrices are validated for proper dimensions matching cal_point
            - ADC values are validated to fit within the specified adc_resolution range
            - Automatic two's complement conversion applied to both hex strings and integer literals
            - Matrix elements can be hex strings ("0xFF00") or integer literals (0xFF00)
        """
        if cal_point[0] > 4 or cal_point[1] > 4:
            raise ValueError(
                f"Device doesn't support more than 4T4P calibration. Received {cal_point[0]}T{cal_point[1]}P."
            )

        # Internal constants
        self.cal_point = cal_point
        """Calibration point configuration as (number temperature points, number pressure points)"""
        self.adc_resolution = adc_resolution
        """ADC resolution in bits (e.g., 16, 24)"""
        self.min_code = -1 * (2 ** (adc_resolution - 1))
        """Minimum ADC code value for given ADC resolution"""
        self.max_code = (2 ** (adc_resolution - 1)) - 1
        """Maximum ADC code value for given ADC resolution"""
        self.normalization_factor = 2 ** (adc_resolution - 2)
        """Normalization factor for ADC and Coefficient values"""

        # Calibration settings
        self.tadc_offset: int = 0
        """Integer offset added to temperature ADC readings"""
        self.tadc_gain: int = 1
        """Integer gain applied to temperature ADC readings"""
        self.padc_offset: int = 0
        """Integer offset added to pressure ADC readings"""
        self.padc_gain: int = 1
        """Integer gain applied to pressure ADC readings"""

        # Import data with validation
        self.tadc_matrix = self.import_integer_matrix(tad_matrix, "tadc_matrix")
        """Signed integer matrix of temperature ADC readings"""
        self.padc_matrix = self.import_integer_matrix(pad_matrix, "padc_matrix")
        """Signed integer matrix of pressure ADC readings"""
        self.dac_matrix = self.import_integer_matrix(dac_matrix, "dac_matrix")
        """Signed integer matrix of DAC output values"""

        # Initialize attributes for regression
        self.tadc_norm: np.ndarray | None = None
        """Normalized temperature ADC matrix (after applying offset and gain scaling)"""
        self.padc_norm: np.ndarray | None = None
        """Normalized pressure ADC matrix (after applying offset and gain scaling)"""
        self.dac_norm: np.ndarray | None = None
        """Normalized DAC matrix"""

    def recommend_calibration(self, offset_enabled: bool = True) -> None:
        """
        Calculate optimal gain and offset values for ADC calibration based on input data range.

        Args:
            offset_enabled: Whether to enable offset correction (OFF_EN bit).
                           If True: apply offset first, then gain (offset before gain)
                           If False: apply gain first, then offset (gain before offset)

        Sets:
            self.offset_enabled: Stores the offset enable setting
            self.tadc_gain, self.padc_gain: Optimal gain values to maximize ADC range usage
            self.tadc_offset, self.padc_offset: Optimal offset values to center data

        Algorithm:
            - offset_enabled=True: Center data around zero, then apply max gain within ADC limits
            - offset_enabled=False: Apply max gain to raw data, then center the gained data

        Note:
            All calculated values are converted to integers for hardware compatibility.
            The optimal settings maximize the use of the available ADC range while preventing overflow.
        """
        self.offset_enabled = offset_enabled

        tadc_max = np.max(self.tadc_matrix)
        tadc_min = np.min(self.tadc_matrix)
        padc_max = np.max(self.padc_matrix)
        padc_min = np.min(self.padc_matrix)

        if offset_enabled:
            # Compute the offset value that centers the min and max values around 0
            self.tadc_offset = -np.floor((tadc_max + tadc_min) / 2).astype(int)
            self.padc_offset = -np.floor((padc_max + padc_min) / 2).astype(int)

            # Compute the max gain that keeps the data within hardware limits
            tadc_max_adj = np.max([np.abs(tadc_max + self.tadc_offset), np.abs(tadc_min + self.tadc_offset)])
            self.tadc_gain = np.floor((self.max_code / tadc_max_adj)).astype(int)

            padc_max_adj = np.max([np.abs(padc_max + self.padc_offset), np.abs(padc_min + self.padc_offset)])
            self.padc_gain = np.floor((self.max_code / padc_max_adj)).astype(int)

        else:
            # Compute the max gain that keeps the data within hardware limits
            tadc_abs_max = np.max([np.abs(tadc_max), np.abs(tadc_min)])
            self.tadc_gain = np.floor((self.max_code / tadc_abs_max)).astype(int)

            padc_abs_max = np.max([np.abs(padc_max), np.abs(padc_min)])
            self.padc_gain = np.floor((self.max_code / padc_abs_max)).astype(int)

            # Compute the offset value that centers the (gained) min and max values around 0
            self.tadc_offset = -np.floor(self.tadc_gain * (tadc_max + tadc_min) / 2).astype(int)
            self.padc_offset = -np.floor(self.padc_gain * (padc_max + padc_min) / 2).astype(int)

        # print("-" * 80)
        # print(f"Recommended calibration settings for OFF_EN={int(self.offset_enabled)}:")
        # print(f"PADC Offset: {self.padc_offset}")
        # print(f"PADC Gain: {self.padc_gain}")
        # print(f"TADC Offset: {self.tadc_offset}")
        # print(f"TADC Gain: {self.tadc_gain}")

    def normalize_data(self) -> None:
        """
        Apply gain and offset scaling to input data and normalize for polynomial regression.

        Applies the calibration settings (gain/offset) determined by recommend_calibration()
        and normalizes the data by the ADC resolution-based normalization factor.

        Processing Steps:
        1. Apply gain and offset scaling based on offset_enabled setting:
           - offset_enabled=True: (data + offset) * gain
           - offset_enabled=False: (data * gain) + offset
        2. Validate scaled data is within ADC hardware limits
        3. Normalize by dividing by normalization_factor (2^(adc_resolution-2))

        Sets:
            self.tadc_norm: Normalized temperature ADC data
            self.padc_norm: Normalized pressure ADC data
            self.dac_norm: Normalized DAC output data

        Raises:
            ValueError: If scaled data exceeds ADC hardware limits

        Note:
            Must be called after recommend_calibration() and before calculate_regression().
            The normalization prevents numerical issues in polynomial regression.
        """
        if self.offset_enabled:
            tadc_matrix_scaled = (self.tadc_matrix + self.tadc_offset) * self.tadc_gain  # type: ignore
            padc_matrix_scaled = (self.padc_matrix + self.padc_offset) * self.padc_gain  # type: ignore
        else:
            tadc_matrix_scaled = (self.tadc_matrix * self.tadc_gain) + self.tadc_offset  # type: ignore
            padc_matrix_scaled = (self.padc_matrix * self.padc_gain) + self.padc_offset  # type: ignore

        # Check if any values are out of bounds
        if np.any(tadc_matrix_scaled > self.max_code) or np.any(tadc_matrix_scaled < -self.max_code):
            raise ValueError("TADC data exceeds hardware limits")
        if np.any(padc_matrix_scaled > self.max_code) or np.any(padc_matrix_scaled < -self.max_code):
            raise ValueError("PADC data exceeds hardware limits")
        if np.any(self.dac_matrix > self.max_code) or np.any(self.dac_matrix < -self.max_code):
            raise ValueError("DAC data exceeds hardware limits")

        self.tadc_norm = tadc_matrix_scaled / self.normalization_factor
        self.padc_norm = padc_matrix_scaled / self.normalization_factor
        self.dac_norm = self.dac_matrix / self.normalization_factor

    def calculate_regression(self) -> None:
        """
        Performs multivariate polynomial regression to calculate calibration coefficients.

        Implements a three-step process:
        1. Generates polynomial features T^i * P^j based on calibration configuration
        2. Applies least squares regression to solve for coefficients
        3. Formats coefficients into structured output matrix

        The regression model: DAC = Σᵢⱼ cᵢⱼ * T^i * P^j where:
        - T, P are normalized temperature/pressure data
        - cᵢⱼ are polynomial coefficients (computed via least squares)
        - i ∈ [0, t_deg-1], j ∈ [0, p_deg-1] based on cal_point configuration

        Attributes Set:
            features (list[ndarray]): Polynomial feature arrays for regression
            coefficients (ndarray): Raw regression coefficients (floating-point)

        Raises:
            ValueError: If normalized data is not available (normalize_data() not called)
            numpy.linalg.LinAlgError: If regression matrix is singular or ill-conditioned

        Mathematical Foundation:
            Uses numpy.linalg.lstsq which solves: coefficients = (A^T A)^(-1) A^T b
            where A is the feature matrix and b is the target DAC values.
            Normalization prevents numerical instability from disparate
            feature magnitudes in higher-order polynomial terms.

        References:
            Polynomial Feature Engineering: https://medium.com/@adnan.mazraeh1993/polynomial-features-a-comprehensive-guide-from-basics-to-advanced-5f18c430a137
            NumPy Least Squares: https://www.pythontutorials.net/blog/numpylinalglstsq/
        """

        if self.tadc_norm is None or self.padc_norm is None or self.dac_norm is None:
            raise ValueError("Normalized data not available for regression.")

        # Flatten the matrices for regression
        tadc_flat = self.tadc_norm.flatten()
        padc_flat = self.padc_norm.flatten()
        dac_flat = self.dac_norm.flatten()

        # Generate polynomial features inline
        t_deg, p_deg = self.cal_point
        features = []
        for i in range(t_deg):
            for j in range(p_deg):
                feature = (tadc_flat**i) * (padc_flat**j)
                features.append(feature)

        # Create feature matrix and perform least squares regression
        feature_stack = np.column_stack(features)
        coefficients, residuals, rank, s = np.linalg.lstsq(a=feature_stack, b=dac_flat, rcond=None)

        self.coeff_norm = coefficients

        # Create EEPROM dictionary with integer values scaled by normalization factor as hex strings
        eeprom_values = []
        for coeff in coefficients:
            scaled_coeff = int(round(coeff * self.normalization_factor))
            hex_val = self.signed_int_to_hex(scaled_coeff)
            eeprom_values.append(hex_val)

        # Create coefficient dictionary with proper names (h0, h1, g0, g1, m0, m1, n0, n1, etc.)
        coeff_names = []
        coeff_vars = ["h", "g", "m", "n"]  # Standard PGA coefficient naming for up to 4P4T

        for i in range(t_deg):
            for j in range(p_deg):
                coeff_names.append(f"{coeff_vars[j]}{i}")

        self.coeff_dict_norm = dict(zip(coeff_names, coefficients))
        self.coeff_dict_eeprom = dict(zip(coeff_names, eeprom_values))

    def compute_dac_value(self, tadc_value: str | int, padc_value: str | int) -> int:
        """
        Compute the expected DAC value for given TADC and PADC inputs using the calibrated coefficients.

        Args:
            tadc_value: Temperature ADC value (hex string or integer)
            padc_value: Pressure ADC value (hex string or integer)

        Returns:
            int: Computed DAC value as signed integer

        Raises:
            ValueError: If coefficients are not available (calculate_regression() not called)
        """
        if self.coeff_dict_norm is None:
            raise ValueError("Regression coefficients not available. Call calculate_regression() first.")

        # Convert hex inputs to signed integers (or pass through if already integers)
        tadc_int = self.hex_to_signed_int(tadc_value)
        padc_int = self.hex_to_signed_int(padc_value)

        # Apply gain and offset scaling based on offset_enabled
        if self.offset_enabled:
            # OFF_EN=1: apply offset first, then gain
            tadc_scaled = (tadc_int + self.tadc_offset) * self.tadc_gain
            padc_scaled = (padc_int + self.padc_offset) * self.padc_gain
        else:
            # OFF_EN=0: apply gain first, then offset
            tadc_scaled = tadc_int * self.tadc_gain + self.tadc_offset
            padc_scaled = padc_int * self.padc_gain + self.padc_offset

        # Normalize the scaled values
        tadc_norm = tadc_scaled / self.normalization_factor
        padc_norm = padc_scaled / self.normalization_factor

        # Compute polynomial using coefficients
        t_deg, p_deg = self.cal_point
        dac_result = 0.0

        for i in range(t_deg):
            for j in range(p_deg):
                coeff_name = f"{'hgmn'[j]}{i}"
                if coeff_name in self.coeff_dict_norm:
                    term_value = (tadc_norm**i) * (padc_norm**j) * self.coeff_dict_norm[coeff_name]
                    dac_result += term_value

        # Scale back to DAC units and return as integer
        return int(round(dac_result * self.normalization_factor))

    def summarize_results(self) -> None:
        """
        Print a comprehensive summary of calibration results including calibration settings,
        polynomial coefficients, calibration point comparison, and error statistics.

        Output includes:
        - Calibration Settings table: OFF_EN, TADC/PADC gain and offset values with hex representation
        - Coefficients table: Polynomial coefficients (h, g, n, m series) with float and EEPROM hex values
        - Calibration Point Comparison: Expected vs computed DAC values for each calibration point
        - Error Statistics: Maximum and mean error with ppm FSR calculations

        Raises:
            ValueError: If regression coefficients are not available (call calculate_regression() first)

        Note:
            Hex values that exceed the ADC resolution range are displayed as "OVERFLOW".
            Coefficients are printed in ordered groups: h0-h3, g0-g3, n0-n3, m0-m3.
        """
        if self.coeff_dict_norm is None:
            raise ValueError("Regression must be calculated first.")

        print("\n" + "=" * 80)
        print(f"CALIBRATION SUMMARY - {self.cal_point[0]}T{self.cal_point[1]}P Configuration")
        print("=" * 80)

        # Print calibration settings table
        print("\nCalibration Settings:")
        print(f"{'Setting':<20} {'Value':<12} {'EEPROM (Hex)':>14}")
        print("-" * 48)

        # OFF_EN setting
        off_en_value = 1 if hasattr(self, "offset_enabled") and self.offset_enabled else 0
        off_en_hex = f"0x{off_en_value:02X}"
        print(f"{'OFF_EN':<20} {off_en_value:<12} {off_en_hex:>14}")

        # TADC settings
        try:
            tadc_gain_hex = f"0x{self.signed_int_to_hex(self.tadc_gain)}"
        except ValueError:
            tadc_gain_hex = "OVERFLOW"

        try:
            tadc_offset_hex = f"0x{self.signed_int_to_hex(self.tadc_offset)}"
        except ValueError:
            tadc_offset_hex = "OVERFLOW"

        print(f"{'TADC_GAIN':<20} {self.tadc_gain:<12} {tadc_gain_hex:>14}")
        print(f"{'TADC_OFFSET':<20} {self.tadc_offset:<12} {tadc_offset_hex:>14}")

        # PADC settings
        try:
            padc_gain_hex = f"0x{self.signed_int_to_hex(self.padc_gain)}"
        except ValueError:
            padc_gain_hex = "OVERFLOW"

        try:
            padc_offset_hex = f"0x{self.signed_int_to_hex(self.padc_offset)}"
        except ValueError:
            padc_offset_hex = "OVERFLOW"

        print(f"{'PADC_GAIN':<20} {self.padc_gain:<12} {padc_gain_hex:>14}")
        print(f"{'PADC_OFFSET':<20} {self.padc_offset:<12} {padc_offset_hex:>14}")

        # Print coefficients table
        print("\nCoefficients:")
        print(f"{'Name':<6} {'Float Value':>16} {'EEPROM (Hex)':>14}")
        print("-" * 38)

        # Print coefficients in specific order: h0,h1,h2,h3, g0,g1,g2,g3, n0,n1,n2,n3, m0,m1,m2,m3
        coeff_order = ["h0", "h1", "h2", "h3", "g0", "g1", "g2", "g3", "n0", "n1", "n2", "n3", "m0", "m1", "m2", "m3"]

        for name in coeff_order:
            if name in self.coeff_dict_norm:
                float_val = self.coeff_dict_norm[name]
                hex_val = self.coeff_dict_eeprom[name]
                print(f"{name:<6} {float_val:>16.6e}     0x{hex_val}")

        # Create comprehensive comparison table
        print("\nCalibration Point Comparison:")
        header = f"{'Point':<7} {'TADC (Hex)':<12} {'PADC (Hex)':<12} {'Expected':<10} {'Computed':<10} {'Error':<6}"
        print(header)
        print("-" * len(header))

        # Flatten arrays and create point labels
        tadc_flat = self.tadc_matrix.flatten()
        padc_flat = self.padc_matrix.flatten()
        dac_expected_flat = self.dac_matrix.flatten()

        errors = []
        point_index = 0
        for t in range(self.cal_point[0]):
            for p in range(self.cal_point[1]):
                if point_index < len(tadc_flat):
                    point_label = f"T{t}P{p}"
                    tadc_val = tadc_flat[point_index]
                    padc_val = padc_flat[point_index]
                    expected_val = dac_expected_flat[point_index]

                    # Use the dedicated compute_dac_value function (uses rounded coefficients)
                    computed_val = self.compute_dac_value(int(tadc_val), int(padc_val))

                    tadc_hex = f"0x{self.signed_int_to_hex(tadc_val)}"
                    padc_hex = f"0x{self.signed_int_to_hex(padc_val)}"
                    expected_hex = f"0x{self.signed_int_to_hex(expected_val)}"
                    computed_hex = f"0x{self.signed_int_to_hex(computed_val)}"
                    error = abs(expected_val - computed_val)
                    errors.append(error)

                    print(
                        f"{point_label:<7} {tadc_hex:<12} {padc_hex:<12} {expected_hex:<10} {computed_hex:<10} {error:<6}"
                    )
                    point_index += 1

        # Calculate and print error statistics
        max_error = max(errors) if errors else 0
        mean_error = sum(errors) / len(errors) if errors else 0

        print("\nError Statistics:")
        print(f"  Max Error:   {max_error:>6} codes  ({max_error * 1e6 / self.normalization_factor:>6.1f} ppm FSR)")
        print(
            f"  Mean Error:  {mean_error:>6.2f} codes  ({mean_error * 1e6 / self.normalization_factor:>6.1f} ppm FSR)"
        )
        print("")

    #### DATA CONVERSION HELPERS ####

    def import_integer_matrix(self, matrix: list[list[int | str]], matrix_name: str) -> np.ndarray:
        """
        Convert list[list[int]] to np.ndarray with signed integers using two's complement conversion.

        Args:
            matrix: Input matrix as list of lists
            matrix_name: Name of the matrix for error reporting

        Returns:
            np.ndarray: Validated signed integer matrix

        Raises:
            ValueError: If matrix dimensions don't match cal_point or values exceed ADC range
        """
        # Validate matrix dimensions match cal_point configuration
        expected_rows = self.cal_point[0]  # Temperature points
        expected_cols = self.cal_point[1]  # Pressure points

        if len(matrix) != expected_rows:
            raise ValueError(
                f"{matrix_name} has {len(matrix)} rows but cal_point=({self.cal_point[0]}, {self.cal_point[1]}) requires {expected_rows} rows"
            )

        for i, row in enumerate(matrix):
            if len(row) != expected_cols:
                raise ValueError(
                    f"{matrix_name} row {i} has {len(row)} columns but cal_point=({self.cal_point[0]}, {self.cal_point[1]}) requires {expected_cols} columns"
                )

        # Convert and validate ADC values
        signed_matrix = []
        for i, row in enumerate(matrix):
            signed_row = []
            for j, val in enumerate(row):
                signed_val = self.hex_to_signed_int(val)

                # Only validate ADC range for TADC and PADC matrices (not DAC output)
                if matrix_name in ["tadc_matrix", "padc_matrix"]:
                    if signed_val < self.min_code or signed_val > self.max_code:
                        raise ValueError(
                            f"{matrix_name}[{i}][{j}] = {signed_val} exceeds {self.adc_resolution}-bit ADC range [{self.min_code}, {self.max_code}]"
                        )

                signed_row.append(signed_val)
            signed_matrix.append(signed_row)

        return np.asarray(signed_matrix, dtype=np.int32)

    def hex_to_signed_int(self, hex_string_or_int: str | int) -> int:
        """
        Convert hex string or integer to signed integer using two's complement conversion.

        Args:
            hex_string_or_int: Hex string (e.g., "FF00") or integer value (e.g., 0xFF00)

        Returns:
            int: Signed integer value using the instance's adc_resolution for bit width

        Note:
            Uses self.adc_resolution to determine the bit width for two's complement conversion.
            Both hex strings and integer literals are converted using the same bit width.
        """
        if isinstance(hex_string_or_int, str):
            # Convert hex string and apply two's complement conversion
            unsigned_int = int(hex_string_or_int, 16)
            sign_bit = 1 << (self.adc_resolution - 1)
            return unsigned_int - (1 << self.adc_resolution) if unsigned_int >= sign_bit else unsigned_int
        elif isinstance(hex_string_or_int, int):
            # Apply two's complement conversion for integers that might be unsigned
            unsigned_int = hex_string_or_int
            sign_bit = 1 << (self.adc_resolution - 1)
            return unsigned_int - (1 << self.adc_resolution) if unsigned_int >= sign_bit else unsigned_int
        else:
            raise TypeError("hex_string_or_int must be a hex string or an integer.")

    def signed_int_to_hex(self, value) -> str:
        """
        Convert a signed integer to hex string using the instance's ADC resolution.

        Args:
            value: Signed integer value to convert

        Returns:
            str: Hex string (without 0x prefix) with proper padding for adc_resolution

        Raises:
            ValueError: If value doesn't fit in the adc_resolution bit range

        Note:
            Uses self.adc_resolution to determine bit width and hex string padding.
            Negative values are converted using two's complement representation.
        """
        # Check if value fits in the specified bit width
        min_val = -(1 << (self.adc_resolution - 1))
        max_val = (1 << (self.adc_resolution - 1)) - 1

        if value < min_val or value > max_val:
            raise ValueError(
                f"Value {value} does not fit in {self.adc_resolution}-bit signed integer range [{min_val}, {max_val}]"
            )

        padding = int(self.adc_resolution / 4)
        if value < 0:
            value = (1 << self.adc_resolution) + value  # convert to 2's complement
        return hex(value)[2:].upper().zfill(padding)


if __name__ == "__main__":
    # 4T4P data format:
    #      P0     P1     P2     P3
    # T0  [T0P0   T0P1   T0P2   T0P3],
    # T1  [T1P0   T1P1   T1P2   T1P3],
    # T2  [T2P0   T2P1   T2P2   T2P3],
    # T3  [T3P0   T3P1   T3P2   T3P3],

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
    # dac_output = cc.compute_dac_value(tadc_value=0x3243B3, padc_value=0xF585B6)
    # print(f"DAC output: {dac_output} (Hex: 0x{cc.signed_int_to_hex(dac_output)})")
