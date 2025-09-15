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


class PGACoeffCalculator:
    """
    This class implements polynomial regression using Z-score normalization and least squares regression
    to calculate calibration coefficients for PGA3xx.

    Inputs: Temperature, Pressure, DAC Output
    Outputs: Coefficients, Model Fit Error, Predicted DAC Outputs

    Mathematical Foundation:
    Uses polynomial feature engineering where DAC = Σᵢⱼ cᵢⱼ * T^i * P^j
    - T, P: Z-score normalized temperature and pressure data
    - cᵢⱼ: Polynomial coefficients (computed via least squares)
    - Normalization prevents matrix conditioning issues with higher-order terms
    """

    def __init__(
        self,
        cal_point: tuple[int, int],
        adc_resolution: int,
        tad_matrix: list[list],
        pad_matrix: list[list],
        dac_matrix: list[list],
    ):
        """
        Initialize the PGACoeffCalculator class.

        Args:
            cal_point: Calibration configuration as (number temperature points, number pressure points).
            adc_resolution: ADC resolution in bits (e.g., 16, 24).
            tad_matrix: Temperature ADC readings as hex strings.
            pad_matrix: Pressure ADC readings as hex strings.
            dac_matrix: DAC output values as hex strings.

        Raises:
            ValueError: If cal_point exceeds 4T4P configuration limits.
            ZeroDivisionError: If temperature or pressure data has zero range.

        Note:
            Input matrices are automatically converted from hex to signed integers
            using 24-bit word length for proper two's complement interpretation.
        """
        if cal_point[0] > 4 or cal_point[1] > 4:
            raise ValueError(
                f"Device doesn't support more than 4T4P calibration. Received {cal_point[0]}T{cal_point[1]}P."
            )

        # Calculate internal constants
        self.cal_point = cal_point
        self.adc_resolution = adc_resolution
        self.min_code = -1 * (2 ** (adc_resolution - 1))
        self.max_code = (2 ** (adc_resolution - 1)) - 1
        self.normalization_factor = 2 ** (adc_resolution - 2)
        self.eeprom_normalization_factor = self.normalization_factor * (
            2**8 if adc_resolution == 24 else 1
        )

        # Format input matrices
        self.tad_matrix = self.convert_matrix_to_signed(tad_matrix, self.adc_resolution)
        self.pad_matrix = self.convert_matrix_to_signed(pad_matrix, self.adc_resolution)
        self.dac_matrix = self.convert_matrix_to_signed(dac_matrix, self.adc_resolution)

        # Setup output matrix formatting
        # NOTE: PGA devices require minimum of 16 coefficients (4x4 matrix) for cal_point <= 4T4P.
        #       Output matrices >4T4P (future improvement) will still be square matrices
        #       i.e. (1...5)T(1...5)P,...,(1...26)T(1...26)P has output matrix dimensions 5x5...26x26
        self.output_matrix_width = np.max((4, np.max(self.cal_point)))
        self.output_matrix_size = self.output_matrix_width**2
        self.coefficient_map = self.generate_coefficient_map(self.output_matrix_width)

        # Setup error validators
        self.T = self.P = None
        self.coefficients = None
        self.rounded_eeprom_coefficients = None

    def normalize_data(self) -> None:
        """
        Performs Z-score normalization on temperature and pressure data matrices.

        Flattens the 4x4 input matrices and applies Z-score normalization to temperature
        and pressure data to prevent numerical instability during polynomial regression.
        DAC values are normalized by the ADC full-scale range for consistent scaling.

        The Z-score transformation (z = (x - mean) / std) ensures all variables have
        mean=0 and std=1, preventing higher-order polynomial terms from dominating
        the regression matrix and causing numerical conditioning issues.

        Attributes Set:
            T (ndarray): Z-score normalized temperature data (mean=0, std=1)
            P (ndarray): Z-score normalized pressure data (mean=0, std=1)
            dac_normalized (ndarray): DAC values scaled by ADC normalization factor
            t_mean (float): Temperature mean for denormalization
            t_std (float): Temperature standard deviation for denormalization
            p_mean (float): Pressure mean for denormalization
            p_std (float): Pressure standard deviation for denormalization
            tad (ndarray): Flattened temperature ADC readings
            pad (ndarray): Flattened pressure ADC readings
            dac_actual (ndarray): Flattened DAC output values

        Raises:
            ZeroDivisionError: If temperature or pressure data has zero standard
                             deviation (indicating constant values across matrix)

        Notes:
            Z-score normalization prevents matrix ill-conditioning in polynomial
            regression by ensuring feature magnitudes are comparable across terms.

        References:
            Z-score normalization: https://spotintelligence.com/2025/02/14/z-score-normalization/
        """

        # Flatten matrices
        self.tad = np.array(
            [val for row in self.tad_matrix for val in row], dtype=float
        )
        self.pad = np.array(
            [val for row in self.pad_matrix for val in row], dtype=float
        )
        self.dac_actual = np.array(
            [val for row in self.dac_matrix for val in row], dtype=float
        )

        # calculate z-score constants
        self.t_mean = np.mean(self.tad)
        self.t_std = np.std(self.tad)
        self.p_mean = np.mean(self.pad)
        self.p_std = np.std(self.pad)

        if self.t_std == 0 or self.p_std == 0:
            raise ZeroDivisionError(
                "Standard Deviation of TADC or PADC input matrices is 0."
            )

        self.T = (self.tad - self.t_mean) / self.t_std
        self.P = (self.pad - self.p_mean) / self.p_std

        self.dac_normalized = self.dac_actual / self.normalization_factor

    def calculate_regression(self) -> None:
        """
        Performs multivariate polynomial regression to calculate calibration coefficients.

        Implements a three-step process:
        1. Generates polynomial features T^i * P^j based on calibration configuration
        2. Applies least squares regression to solve for coefficients
        3. Formats coefficients into structured output matrix

        The regression model: DAC = Σᵢⱼ cᵢⱼ * T^i * P^j where:
        - T, P are Z-score normalized temperature/pressure data
        - cᵢⱼ are polynomial coefficients (computed via least squares)
        - i ∈ [0, t_deg-1], j ∈ [0, p_deg-1] based on cal_point configuration

        Attributes Set:
            features (list[ndarray]): Polynomial feature arrays for regression
            feature_names (list[str]): Feature identifiers (e.g., 'T0P0', 'T1P2')
            A_list_inputs_feature_stack (ndarray): Feature matrix for regression
            coefficients (ndarray): Raw regression coefficients (floating-point)
            coefficients_formatted (ndarray): Coefficients arranged in output matrix

        Raises:
            ValueError: If normalized data is not available (normalize_data() not called)
            numpy.linalg.LinAlgError: If regression matrix is singular or ill-conditioned

        Mathematical Foundation:
            Uses numpy.linalg.lstsq which solves: coefficients = (A^T A)^(-1) A^T b
            where A is the feature matrix and b is the target DAC values.
            Z-score normalization prevents numerical instability from disparate
            feature magnitudes in higher-order polynomial terms.

        References:
            Polynomial Feature Engineering: https://medium.com/@adnan.mazraeh1993/polynomial-features-a-comprehensive-guide-from-basics-to-advanced-5f18c430a137
            NumPy Least Squares: https://www.pythontutorials.net/blog/numpylinalglstsq/
        """

        if self.T is None or self.P is None:
            raise ValueError("Normalized data not available for regression.")

        self.features, self.feature_names = self.create_polynomial_features(
            self.T, self.P
        )

        self.A_list_inputs_feature_stack = np.column_stack(self.features)

        coefficients, residuals, rank, s = np.linalg.lstsq(
            a=self.A_list_inputs_feature_stack, b=self.dac_normalized, rcond=None
        )

        self.coefficients = coefficients
        self.coefficients_formatted = self.format_into_matrix(
            coefficients, self.feature_names
        )

    def calculate_eeprom_coefficients(self) -> None:
        """
        Converts floating-point regression coefficients to fixed-point integers for EEPROM storage.

        Transforms regression coefficients from floating-point to fixed-point format
        suitable for embedded device storage and computation. The scaling factor is
        determined by ADC resolution to maintain precision while fitting within
        the device's integer arithmetic capabilities.

        The conversion process:
        1. Scales coefficients by eeprom_normalization_factor
        2. Converts to 32-bit signed integers
        3. Formats into structured output matrix
        4. Generates hex representations for device programming
        5. Creates internal scaled coefficients for validation

        Attributes Set:
            eeprom_coefficients_formatted (ndarray): Integer coefficients in matrix format
            eeprom_coefficients_hex (list[str]): Hex strings for device programming
            _eeprom_coefficients_adc_scaled (ndarray): Internal scaled coefficients for validation

        Raises:
            ValueError: If regression coefficients are not available (calculate_regression() not called)
            OverflowError: If scaled coefficients exceed 32-bit signed integer range

        Notes:
            The EEPROM coefficients preserve the polynomial relationship while enabling
            efficient integer arithmetic in the embedded device firmware. The hex format
            is directly programmable into the PGA3xx device EEPROM registers.

            Internal validation coefficients undergo additional hex/signed conversion
            to simulate the exact device behavior including potential precision loss.
        """
        if self.coefficients is None:
            raise ValueError("Regression data not available for conversion.")

        raw_eeprom_coefficients = (
            self.coefficients * self.eeprom_normalization_factor
        ).astype(np.int32)

        eeprom_coefficients_hex = [
            self.signed_int_to_hex(val, self.adc_resolution)
            for val in raw_eeprom_coefficients
        ]

        self.eeprom_hex_coefficients = self.format_into_matrix(
            eeprom_coefficients_hex, self.feature_names
        )

        # Recalculate floating-point coefficients from truncated
        # (24-bit) values to use for prediction + error analysis.
        self.rounded_eeprom_coefficients = (
            np.array(
                [
                    self.hex_to_signed_int(val, self.adc_resolution)
                    for val in eeprom_coefficients_hex
                ]
            ).astype(np.float32)
            / self.eeprom_normalization_factor
        )

    def validate_regression(self) -> None:
        """
        Validates regression accuracy using EEPROM coefficients and computes prediction error metrics.

        Performs comprehensive validation of the calibration by:
        1. Converting EEPROM coefficients back to normalized floating-point scale
        2. Computing predicted DAC values using the polynomial model
        3. Validating predictions are within ADC full-scale range
        4. Calculating prediction errors in parts-per-million of full-scale range
        5. Generating hex representations for comparison with target values

        This validation simulates the exact device behavior by using integer EEPROM
        coefficients (including any quantization effects) rather than the original
        floating-point regression coefficients.

        Error Calculation:
            prediction_error_ppm_fsr = |DAC_actual - DAC_predicted| * 10^6 / FSR
            where FSR = normalization_factor (half of ADC full-scale range)

        Attributes Set:
            eeprom_coefficients_adc_scaled_normalized (ndarray): EEPROM coefficients scaled back to regression units
            dac_theoretical (ndarray): Predicted DAC values using EEPROM coefficients
            prediction_error_ppm_fsr (ndarray): Prediction error per calibration point (ppm FSR)
            dac_theoretical_hex (list[str]): Predicted DAC values as hex strings
            dac_actual_hex (list[str]): Target DAC values as hex strings

        Raises:
            ValueError: If EEPROM coefficients are not available (calculate_eeprom_coefficients() not called)
                       or if predicted values exceed ADC range (indicating poor calibration or invalid data)

        Notes:
            Range validation ensures calibration coefficients produce physically realizable
            DAC outputs. Values outside the ADC range indicate either poor model fit,
            invalid input data, or numerical issues in the regression calculation.

            The ppm FSR metric provides a normalized error measurement independent of
            ADC resolution, enabling comparison across different device configurations.
        """
        if self.rounded_eeprom_coefficients is None:
            raise ValueError("EEPROM coefficients not available for validation.")

        # calculate predicted DAC values using normalized EEPROM coefficients
        self.dac_theoretical = (
            self.A_list_inputs_feature_stack @ self.rounded_eeprom_coefficients
        ) * self.normalization_factor

        # check if predicted values are within ADC full-scale range
        if np.any(self.dac_theoretical > self.max_code) or np.any(
            self.dac_theoretical < self.min_code
        ):
            raise ValueError("Predicted values out of range.")

        # calculate prediction error
        prediction_error = self.dac_actual.astype(
            np.int32
        ) - self.dac_theoretical.astype(int)
        self.prediction_error_ppm_fsr = (
            np.abs(prediction_error) * 1e6 / self.normalization_factor
        )

        # create hex lists for dac values
        self.dac_theoretical_hex = [
            self.signed_int_to_hex(val, self.adc_resolution)
            for val in self.dac_theoretical.astype(np.int32)
        ]

        self.dac_actual_hex = [
            self.signed_int_to_hex(val, self.adc_resolution)
            for val in self.dac_actual.astype(np.int32)
        ]

    def approximate_dac_value(self, temperature: str | int, pressure: str | int) -> int:
        """
        Predicts DAC output for a single temperature-pressure measurement pair.

        Applies the calibrated polynomial model to predict the required DAC output
        for given temperature and pressure ADC readings. Uses the same Z-score
        normalization parameters computed during calibration to ensure consistent
        feature scaling.

        The prediction process:
        1. Converts hex inputs to signed integers
        2. Applies Z-score normalization using calibration statistics
        3. Generates polynomial features for the single measurement
        4. Computes prediction using EEPROM coefficients
        5. Scales result back to DAC units and returns as integer

        Args:
            temperature: Single temperature value (integer or hex string)
            pressure: Single pressure value (integer or hex string)

        Returns:
            int: Predicted DAC value as signed integer

        Raises:
            ValueError: If regression coefficients are not available (regression not calculated)
            TypeError: If temperature or pressure inputs are not valid hex strings

        Examples:
            >>> cc.approximate_dac_value("3243B3", "F585B6")
            1638

        Notes:
            This method simulates the embedded device's calibration calculation,
            using the same EEPROM coefficients that would be programmed into the device.
            The prediction accuracy depends on how well the polynomial model fits
            the calibration data and the measurement's proximity to calibration points.
        """
        if self.rounded_eeprom_coefficients is None:
            raise ValueError("Regression coefficients not available for approximation.")

        temperature = self.hex_to_signed_int(temperature, self.adc_resolution)
        pressure = self.hex_to_signed_int(pressure, self.adc_resolution)

        # Apply same Z-score normalization used during coefficient calculation
        T = (temperature - self.t_mean) / self.t_std
        P = (pressure - self.p_mean) / self.p_std

        features, _ = self.create_polynomial_features(T, P)

        A_single_TP_feature_stack = np.array(features)

        dac_single_approximation = (
            A_single_TP_feature_stack @ self.rounded_eeprom_coefficients
        ) * self.normalization_factor

        return int(dac_single_approximation.astype(np.int32))

    def summarize_results(self) -> None:
        """
        Prints a comprehensive, formatted summary of calibration results to console.
        """

        # ----------------------------------------------------------------------------
        ## utility methods for summarize_results()
        def print_in_chunks(
            data, format_specifier: str = "12.3f", chunk_size: int = 4
        ) -> None:
            for i in range(0, len(data), chunk_size):
                chunk = data[i : i + chunk_size]
                formatted_chunk = [f"{x:^{format_specifier}}" for x in chunk]
                print(" ".join(formatted_chunk))

        def format_polynomial_equation(coeffs):
            """Format polynomial coefficients with Unicode superscripts"""
            terms = []
            powers = ["", "T", f"T{superscript_two}", f"T{superscript_three} "]

            for i, coeff in enumerate(coeffs):
                if coeff != 0:
                    if i == 0:
                        # First term: add space for positive, keep - for negative
                        if coeff >= 0:
                            terms.append(f" {coeff:.3e}")
                        else:
                            terms.append(f"{coeff:.3e}")
                    else:
                        # Subsequent terms: always show sign
                        sign = " + " if coeff >= 0 else " - "
                        terms.append(f"{sign}{abs(coeff):.3e}{middle_dot}{powers[i]}")

            return "".join(terms) if terms else "0"

        # ----------------------------------------------------------------------------

        print("Results:\n")
        formatted_eeprom_coefficients = self.format_into_matrix(
            self.rounded_eeprom_coefficients, self.feature_names
        ).tolist()

        # Print table header
        print(f"{'Coeff':<5} {'Rounded Float':>16} {'EEPROM (Hex)':>16}")
        print("-" * 40)

        # Print coefficient table in grouped order (h0-h3, g0-g3, m0-m3, n0-n3)
        coeff_groups = ["h", "g", "m", "n"]
        for group in coeff_groups:
            for i in range(4):
                coeff_name = f"{group}{i}"
                feature_name = f"T{i}P{coeff_groups.index(group)}"
                if feature_name in self.coefficient_map:
                    idx = self.coefficient_map[feature_name]["index"]
                    coeff_val = formatted_eeprom_coefficients[idx]
                    hex_val = self.eeprom_hex_coefficients[idx]
                    print(f"{coeff_name:<5} {coeff_val:>16.3e} {'0x' + hex_val:>16}")

        print("\nRegression Equation: DAC =")
        superscript_two = "\u00b2"
        superscript_three = "\u00b3"
        middle_dot = "\u00b7"
        h_str = format_polynomial_equation(formatted_eeprom_coefficients[0::4])
        g_str = format_polynomial_equation(formatted_eeprom_coefficients[1::4])
        m_str = format_polynomial_equation(formatted_eeprom_coefficients[2::4])
        n_str = format_polynomial_equation(formatted_eeprom_coefficients[3::4])
        max_width = max(len(h_str), len(g_str), len(m_str), len(n_str))
        print(
            f" ({h_str:<{max_width}}) +\n ({g_str:<{max_width}}) {middle_dot} P +\n ({m_str:<{max_width}}) {middle_dot} P{superscript_two} +\n ({n_str:<{max_width}}) {middle_dot} P{superscript_three}"
        )

        print("\nRequested DAC Values (0x):")
        print_in_chunks(
            self.dac_actual_hex, format_specifier="12", chunk_size=self.cal_point[1]
        )

        print("\nPredicted DAC Values (0x):")
        print_in_chunks(
            self.dac_theoretical_hex,
            format_specifier="12",
            chunk_size=self.cal_point[1],
        )

        print("\nPrediction Error: (ppmFSR)")
        print_in_chunks(
            self.prediction_error_ppm_fsr.tolist(),
            format_specifier="12.3f",
            chunk_size=self.cal_point[1],
        )

    #### COEFF CALC HELPERS ####
    def generate_coefficient_map(self, max_size: int) -> dict:
        """
        Generates a mapping between polynomial feature names and coefficient variables.

        This method creates a structured mapping that associates polynomial terms
        (e.g., "T0P0", "T1P2") with their corresponding coefficient variable names
        (e.g., "h0", "n2") and matrix indices. This mapping is essential for
        formatting coefficients into the expected output matrix structure.

        The coefficient naming follows the convention:
        - First 4 columns: h, g, n, m (traditional PGA naming)
        - Additional columns: remaining alphabet letters (excluding h, g, n, m)
        - Rows indicated by subscript numbers (e.g., h0, h1, h2, h3)

        Args:
            max_size (int): Maximum matrix dimension (supports up to 26x26 due to alphabet limit)

        Returns:
            dict: Mapping of feature names to coefficient info containing:
                - 'index': Linear index in flattened output matrix
                - 'var': Coefficient variable name (e.g., "h0", "g2", "n1")

        Note:
            Maximum supported matrix size is 26x26 i.e. 26T,26P due to alphabet constraints.
        """
        excluded = {"h", "g", "m", "n"}
        available_vars = [
            chr(i) for i in range(ord("a"), ord("z") + 1) if chr(i) not in excluded
        ]
        # Use traditional h,g,n,m for first 4 columns, then remaining letters
        coeff_vars = ["h", "g", "m", "n"] + available_vars[
            : self.output_matrix_width - 4
        ]

        formatted_matrix_map: dict = {}
        t_max, p_max = max_size, max_size

        for i in range(t_max):
            for j in range(p_max):
                feature_name = f"T{i}P{j}"
                index = i * t_max + j
                coeff_var = f"{coeff_vars[j]}{i}"
                formatted_matrix_map[feature_name] = {"index": index, "var": coeff_var}
        return formatted_matrix_map

    def format_into_matrix(self, values_to_format, feature_names) -> np.ndarray:
        """
        Formats coefficient values into a structured output matrix.

        This method takes coefficient values and their corresponding feature names
        and arranges them into the required output matrix format using the
        coefficient mapping. Unused positions in the matrix are filled with zeros.

        Args:
            values_to_format (array-like): Coefficient values to be formatted
            feature_names (list[str]): List of polynomial feature names corresponding to values

        Returns:
            ndarray: Formatted coefficient matrix with values placed according to
                    the coefficient mapping and unused positions filled with zeros
        """
        # Preserve input dtype instead of forcing float
        input_array = np.asarray(values_to_format)
        formatted = np.zeros(self.output_matrix_size, dtype=input_array.dtype)
        for i, key in enumerate(feature_names):
            index = self.coefficient_map[key]["index"]
            formatted[index] = values_to_format[i]
        return formatted

    def create_polynomial_features(self, T, P) -> tuple[list, list]:
        """
        Creates polynomial features for multivariate regression based on calibration configuration.

        This method generates polynomial terms of the form T^i * P^j where i ranges from
        0 to t_deg-1 and j ranges from 0 to p_deg-1, based on the calibration point
        configuration (t_deg, p_deg). Each combination creates a feature for the
        regression model.

        Args:
            T (array-like): Normalized temperature data
            P (array-like): Normalized pressure data

        Returns:
            tuple: Contains:
                - features (list[ndarray]): List of polynomial feature arrays
                - feature_names (list[str]): Corresponding feature names (e.g., "T0P0", "T1P2")

        Example:
            For cal_point=(2, 3), generates features:
            T0P0 (1), T0P1 (P), T0P2 (P²), T1P0 (T), T1P1 (T*P), T1P2 (T*P²)

        Note:
            Features are generated in row-major order, matching the coefficient matrix organization.
        """
        t_deg, p_deg = self.cal_point
        features = []
        feature_names = []
        for i in range(t_deg):
            for j in range(p_deg):
                feature_names.append(f"T{i}P{j}")
                feature = (T**i) * (P**j)
                features.append(feature)

        return (features, feature_names)

    #### DATA CONVERSION HELPERS ####
    def hex_to_signed_int(self, hex_value: str | int, word_length: int = 16) -> int:
        # Convert hex value to integer
        if isinstance(hex_value, str):
            unsigned_int = int(hex_value, 16)
        elif isinstance(hex_value, int):
            unsigned_int = hex_value
        else:
            raise TypeError("hex_value must be a hex string or an integer.")

        # Calculate the maximum value for the given word length
        max_value = (1 << (word_length - 1)) - 1

        # Check if the most significant bit (MSB) is set
        if unsigned_int > max_value:
            # Subtract 2^word_length to get signed value
            signed_int = unsigned_int - (1 << word_length)
        else:
            signed_int = unsigned_int

        return signed_int

    def convert_matrix_to_signed(self, matrix, word_length=24) -> list:
        signed_matrix = []
        for row in matrix:
            signed_row = [
                self.hex_to_signed_int(hex_val, word_length) for hex_val in row
            ]
            signed_matrix.append(signed_row)
        return signed_matrix

    def signed_int_to_hex(self, value, bits: int = 32) -> str:
        padding = int(bits / 4)
        if value < 0:
            value = (1 << bits) + value  # convert to 2's complement
        return hex(value)[2:].upper().zfill(padding)


if __name__ == "__main__":
    # example 4T4P data
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
        cal_point=(4, 4),
        adc_resolution=24,
        tad_matrix=tadc,
        pad_matrix=padc,
        dac_matrix=dac,
    )
    cc.normalize_data()
    cc.calculate_regression()
    cc.calculate_eeprom_coefficients()
    cc.validate_regression()
    cc.summarize_results()

    print("\n === Example single value approximation ===")
    T, P = 0x3243B3, 0xF585B6
    dac_output = cc.approximate_dac_value(T, P)
    print(
        f"For T = 0x{T:06X} and P = 0x{P:06X}, the resulting DAC output = 0x{dac_output:06X}\n"
    )
