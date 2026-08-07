"""EEPROM CSV utilities for the PGA3xx paged register-dump format.

Provides:
- Round-trip conversion between a tab-delimited 16-page x 8-byte CSV
  (as shipped by TI's factory tooling) and a flat ``dict[int, int]``
  mapping register address to byte value.
- Per-device EEPROM register layouts (:class:`DeviceLayout`,
  :data:`DEVICE_LAYOUTS`) for PGA300, PGA302, and PGA305.
- Functions to overlay calibration output from a
  :class:`~pga_coefficient_calculator.PGACoeffCalculator` onto a
  flat register map, compute the CRC-8-ATM checksum, and write a
  calibrated CSV end-to-end (:func:`write_calibrated_csv`).
"""

# SPDX-License-Identifier: BSD-3-Clause
# Copyright (C) 2025-2026 Texas Instruments Incorporated
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
#     "crc>=7.0.0",
#     "numpy>=2.3.1",
# ]
# ///

from __future__ import annotations

import csv
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import TYPE_CHECKING

from crc import Calculator, Configuration

if TYPE_CHECKING:
    try:
        from .pga_coefficient_calculator import PGACoeffCalculator
    except ImportError:
        from pga_coefficient_calculator import PGACoeffCalculator

__version__ = "1.0.0"


# ── CSV / EEPROM format constants ───────────────────────────────────────────
# TI's paged CSV file format (16 rows x 8 byte columns)

NUM_PAGES = 16
BYTES_PER_PAGE = 8
TOTAL_BYTES = NUM_PAGES * BYTES_PER_PAGE
DELIMITER = "\t"
LINE_TERMINATOR = "\r\n"
HEADER: tuple[str, ...] = (
    "Page",
    "Byte 0",
    "Byte 1",
    "Byte 2",
    "Byte 3",
    "Byte 4",
    "Byte 5",
    "Byte 6",
    "Byte 7",
)

DEFAULT_CRC_ADDR = 0x7F
"""Public default CRC byte address (0x7F) shared by all currently supported PGA3xx devices."""

_CRC_CONFIG = Configuration(width=8, polynomial=0x07, init_value=0xFF)
_CRC_CALCULATOR = Calculator(_CRC_CONFIG, optimized=True)


# ── Per-device EEPROM register layouts ──────────────────────────────────────


@dataclass(frozen=True)
class DeviceLayout:
    """EEPROM byte-address map for a single PGA3xx device variant.

    All address tuples are MSB-first. Tuple length equals the register byte width.

    Attributes:
        coeff_addrs: Mapping of coefficient name (``"h0"``..``"m3"``) to
            an MSB-first tuple of byte addresses. Length is 2 for 16-bit
            devices, 3 for 24-bit.
        padc_gain, padc_offset, tadc_gain, tadc_offset: MSB-first byte
            address tuples. Length may be 1, 2, or 3 depending on register
            width.
        offset_enable_addr: Byte address of the offset-enable flag, or
            ``None`` if the device has no such register.
        crc_addr: Byte address of the EEPROM CRC (always 0x7F on
            currently-supported devices).
    """

    coeff_addrs: Mapping[str, tuple[int, ...]]
    padc_gain: tuple[int, ...]
    padc_offset: tuple[int, ...]
    tadc_gain: tuple[int, ...]
    tadc_offset: tuple[int, ...]
    offset_enable_addr: int | None
    crc_addr: int


def _pga305_coeff_addrs() -> dict[str, tuple[int, int, int]]:
    """24-bit sequential coefficient layout: base = 12*group + 3*index."""
    addrs: dict[str, tuple[int, int, int]] = {}
    for grp_idx, grp in enumerate("hgnm"):
        for i in range(4):
            base = 12 * grp_idx + 3 * i
            addrs[f"{grp}{i}"] = (base + 2, base + 1, base)
    return addrs


def _pga302_coeff_addrs() -> dict[str, tuple[int, int]]:
    """16-bit sequential coefficient layout: base = 8*group + 2*index."""
    addrs: dict[str, tuple[int, int]] = {}
    for grp_idx, grp in enumerate("hgnm"):
        for i in range(4):
            base = 8 * grp_idx + 2 * i
            addrs[f"{grp}{i}"] = (base + 1, base)
    return addrs


_PGA300_COEFF_ADDRS: dict[str, tuple[int, int]] = {
    "h0": (0x01, 0x00),
    "g0": (0x03, 0x02),
    "n0": (0x05, 0x04),
    "h1": (0x07, 0x06),
    "g1": (0x09, 0x08),
    "n1": (0x0B, 0x0A),
    "h2": (0x0D, 0x0C),
    "g2": (0x0F, 0x0E),
    "n2": (0x11, 0x10),
    "h3": (0x37, 0x36),
    "g3": (0x39, 0x38),
    "n3": (0x3B, 0x3A),
    "m0": (0x3D, 0x3C),
    "m1": (0x3F, 0x3E),
    "m2": (0x41, 0x40),
    "m3": (0x43, 0x42),
}


DEVICE_LAYOUTS: Mapping[str, DeviceLayout] = MappingProxyType(
    {
        "PGA305": DeviceLayout(
            coeff_addrs=_pga305_coeff_addrs(),
            padc_gain=(0x46, 0x45, 0x44),
            padc_offset=(0x49, 0x48, 0x47),
            tadc_gain=(0x60, 0x5F, 0x5E),
            tadc_offset=(0x63, 0x62, 0x61),
            offset_enable_addr=0x69,
            crc_addr=0x7F,
        ),
        "PGA302": DeviceLayout(
            coeff_addrs=_pga302_coeff_addrs(),
            padc_gain=(0x20,),
            padc_offset=(0x23, 0x22),
            tadc_gain=(0x21,),
            tadc_offset=(0x25, 0x24),
            offset_enable_addr=0x29,
            crc_addr=0x7F,
        ),
        "PGA300": DeviceLayout(
            coeff_addrs=_PGA300_COEFF_ADDRS,
            padc_gain=(0x33, 0x32),
            padc_offset=(0x35, 0x34),
            tadc_gain=(0x4D, 0x4C),
            tadc_offset=(0x4F, 0x4E),
            offset_enable_addr=None,
            crc_addr=0x7F,
        ),
    }
)


# ── Helpers ─────────────────────────────────────────────────────────────────


def _validate_flat(flat: dict[int, int]) -> None:
    """Raise ``ValueError`` unless ``flat`` is a complete 128-byte address map."""
    if len(flat) != TOTAL_BYTES or set(flat) != set(range(TOTAL_BYTES)):
        raise ValueError(f"flat must have exactly {TOTAL_BYTES} entries covering addresses 0..{TOTAL_BYTES - 1}")
    for addr, value in flat.items():
        if not 0 <= value <= 0xFF:
            raise ValueError(f"flat[0x{addr:02X}] = {value} out of byte range [0, 255]")


def _split_signed(value: int, num_bytes: int) -> tuple[int, ...]:
    """Split a signed integer into ``num_bytes`` bytes in MSB-first order.

    Matches the address-tuple convention used by :data:`DEVICE_LAYOUTS`,
    where the first element is the most-significant byte's address. On
    little-endian storage the caller writes the tuple's last byte to the
    lowest address.

    Args:
        value: Signed integer to split. Must fit in ``num_bytes`` signed
            range (``-2**(8*num_bytes-1) <= value < 2**(8*num_bytes-1)``).
        num_bytes: Register width in bytes (1, 2, or 3).

    Returns:
        Tuple of length ``num_bytes`` with bytes ordered MSB to LSB.

    Raises:
        ValueError: If ``value`` overflows the signed ``num_bytes`` range,
            or ``num_bytes`` is not positive.
    """
    if num_bytes < 1:
        raise ValueError(f"num_bytes must be >= 1, got {num_bytes}")
    max_val = (1 << (8 * num_bytes - 1)) - 1
    min_val = -(1 << (8 * num_bytes - 1))
    if not min_val <= value <= max_val:
        raise ValueError(f"{value} does not fit in {num_bytes * 8}-bit signed range [{min_val}, {max_val}]")
    if value < 0:
        value += 1 << (8 * num_bytes)
    return tuple((value >> (8 * i)) & 0xFF for i in range(num_bytes - 1, -1, -1))


def _write_register(
    result: dict[int, int],
    addrs: tuple[int, ...],
    value: int,
    field_name: str | None = None,
) -> None:
    """Overwrite ``result`` at the MSB-first ``addrs`` with a split of ``value``.

    ``field_name`` is only used to make ``ValueError`` messages more
    identifiable. Defaults to the base address if not provided.
    """
    try:
        parts = _split_signed(value, len(addrs))
    except ValueError as exc:
        label = field_name if field_name is not None else f"register at 0x{addrs[-1]:02X}"
        raise ValueError(f"{label}: {exc}") from exc
    for addr, byte in zip(addrs, parts):
        result[addr] = byte


# ── Paged CSV I/O ───────────────────────────────────────────────────────────


def paged_csv_to_flat(csv_path: str | Path) -> dict[int, int]:
    """Read a tab-delimited paged EEPROM CSV into a flat ``{addr: byte}`` dict.

    Args:
        csv_path: Path to a tab-delimited CSV in the paged format
            (16 rows x 8 byte columns, plus a Page column). Byte values
            are hex without ``0x`` prefix.

    Returns:
        dict mapping each address 0x00-0x7F to its byte value (0-255).

    Raises:
        FileNotFoundError: If ``csv_path`` does not exist.
        ValueError: If the header, row count, page column, or any byte
            value does not match the expected format.
    """
    path = Path(csv_path)
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.reader(f, delimiter=DELIMITER)
        rows = list(reader)

    if not rows:
        raise ValueError(f"{path}: file is empty")

    header = tuple(cell.strip() for cell in rows[0])
    if header != HEADER:
        raise ValueError(f"{path}: header mismatch. Expected {HEADER}, got {header}")

    data_rows = rows[1:]
    if len(data_rows) != NUM_PAGES:
        raise ValueError(f"{path}: expected {NUM_PAGES} data rows, got {len(data_rows)}")

    flat: dict[int, int] = {}
    for row_idx, row in enumerate(data_rows):
        cells = [cell.strip() for cell in row]
        if len(cells) != 1 + BYTES_PER_PAGE:
            raise ValueError(f"{path}: row {row_idx} has {len(cells)} cells, expected {1 + BYTES_PER_PAGE}")

        try:
            page = int(cells[0], 16)
        except ValueError as exc:
            raise ValueError(f"{path}: row {row_idx} page column '{cells[0]}' is not hex") from exc

        if page != row_idx:
            raise ValueError(f"{path}: row {row_idx} has page {page}, expected {row_idx}")

        for byte_idx in range(BYTES_PER_PAGE):
            cell = cells[1 + byte_idx]
            try:
                value = int(cell, 16)
            except ValueError as exc:
                raise ValueError(f"{path}: row {row_idx} byte {byte_idx} value '{cell}' is not hex") from exc
            if not 0 <= value <= 0xFF:
                raise ValueError(f"{path}: row {row_idx} byte {byte_idx} value 0x{value:X} out of [0, 0xFF]")
            flat[page * BYTES_PER_PAGE + byte_idx] = value

    return flat


def flat_to_paged_csv(flat: dict[int, int], csv_path: str | Path) -> None:
    """Write a flat ``{addr: byte}`` dict to a tab-delimited paged EEPROM CSV.

    Args:
        flat: Dict mapping addresses 0x00-0x7F to byte values 0-255.
            Must have exactly 128 entries covering the full address range.
        csv_path: Destination file path. Overwritten if it exists.

    Raises:
        ValueError: If ``flat`` is missing addresses, has extra keys, or
            has values outside 0-255.
    """
    _validate_flat(flat)

    path = Path(csv_path)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f, delimiter=DELIMITER, lineterminator=LINE_TERMINATOR)
        writer.writerow(HEADER)
        for page in range(NUM_PAGES):
            row = [format(page, "X")]
            for byte_idx in range(BYTES_PER_PAGE):
                row.append(format(flat[page * BYTES_PER_PAGE + byte_idx], "02X"))
            writer.writerow(row)


# ── CRC ─────────────────────────────────────────────────────────────────────


def compute_crc(flat: dict[int, int], crc_addr: int = DEFAULT_CRC_ADDR) -> int:
    """Compute the CRC-8-ATM checksum over EEPROM addresses ``[0, crc_addr)``.

    Uses polynomial 0x07, initial value 0xFF, no reflection, no final XOR.
    The byte at ``crc_addr`` (the CRC byte itself) is excluded from the
    computation.

    Args:
        flat: Dict containing at least addresses ``[0, crc_addr)`` with byte
            values.
        crc_addr: Byte address where the CRC is stored on the target device.
            Defaults to :data:`DEFAULT_CRC_ADDR` (0x7F) - the value shared by
            all currently supported devices. Pass ``layout.crc_addr`` from
            :data:`DEVICE_LAYOUTS` when computing for a specific device.

    Returns:
        int: Computed CRC in [0, 255].
    """
    payload = bytes(flat[addr] for addr in range(crc_addr))
    return _CRC_CALCULATOR.checksum(payload)


def validate_crc(csv_path: str | Path, crc_addr: int = DEFAULT_CRC_ADDR) -> tuple[bool, int]:
    """Check whether the stored CRC in a paged EEPROM CSV matches the computed one.

    Recomputes the CRC-8-ATM checksum over the bytes preceding ``crc_addr``
    and prints a message reporting the outcome.

    Args:
        csv_path: Path to a tab-delimited paged EEPROM CSV (see
            :func:`paged_csv_to_flat` for the expected format).
        crc_addr: Byte address of the stored CRC. Defaults to
            :data:`DEFAULT_CRC_ADDR` (0x7F).

    Returns:
        tuple[bool, int]: (True, computed_crc) if the stored CRC matches
            the computed CRC, otherwise (False, computed_crc).

    Raises:
        FileNotFoundError: If ``csv_path`` does not exist.
        ValueError: If the CSV is malformed (see :func:`paged_csv_to_flat`).
    """
    flat = paged_csv_to_flat(csv_path)
    stored = flat[crc_addr]
    computed = compute_crc(flat, crc_addr)
    valid = stored == computed

    if valid:
        print(f"{csv_path}: CRC valid (0x{stored:02X})")
    else:
        print(f"{csv_path}: CRC mismatch - stored 0x{stored:02X}, computed 0x{computed:02X}")

    return valid, computed


# ── Calibration overlay ─────────────────────────────────────────────────────


def apply_calibration_to_flat(flat: dict[int, int], calc: PGACoeffCalculator) -> dict[int, int]:
    """Overlay a completed PGACoeffCalculator's output onto a flat EEPROM byte map.

    Uses the device-specific register layout from :data:`DEVICE_LAYOUTS`.
    Overwrites the H0-M3 coefficient block, PADC_GAIN/OFFSET,
    TADC_GAIN/OFFSET, the optional offset-enable flag, and recomputes the
    CRC at ``layout.crc_addr``. All other bytes are copied through
    unchanged.

    Args:
        flat: Full 128-byte EEPROM map (addresses 0x00-0x7F).
        calc: A :class:`PGACoeffCalculator` on which
            ``calculate_regression()`` has already been called.

    Returns:
        A new dict with the calibration bytes overwritten. Input is not
        mutated.

    Raises:
        ValueError: If ``flat`` is not a full 128-byte map, ``calc`` has
            no regression coefficients, the device is not in
            :data:`DEVICE_LAYOUTS`, or a calibration value overflows its
            target register width.
    """
    _validate_flat(flat)

    if not calc.coeff_dict_eeprom:
        raise ValueError("calc has no coefficients; call calculate_regression() first")

    if calc.device not in DEVICE_LAYOUTS:
        raise ValueError(
            f"No EEPROM layout defined for device '{calc.device}'. Supported devices: {sorted(DEVICE_LAYOUTS)}"
        )
    layout = DEVICE_LAYOUTS[calc.device]

    result = dict(flat)

    for name, addrs in layout.coeff_addrs.items():
        signed_val = calc.hex_to_signed_int(calc.coeff_dict_eeprom[name])
        _write_register(result, addrs, signed_val, f"coefficient {name}")

    _write_register(result, layout.tadc_gain, int(calc.tadc_gain), "tadc_gain")
    _write_register(result, layout.tadc_offset, int(calc.tadc_offset), "tadc_offset")
    _write_register(result, layout.padc_gain, int(calc.padc_gain), "padc_gain")
    _write_register(result, layout.padc_offset, int(calc.padc_offset), "padc_offset")

    if layout.offset_enable_addr is not None:
        result[layout.offset_enable_addr] = 1 if calc.offset_enabled else 0

    result[layout.crc_addr] = compute_crc(result, layout.crc_addr)

    return result


def write_calibrated_csv(
    input_csv_path: str | Path,
    output_csv_path: str | Path,
    calc: PGACoeffCalculator,
    *,
    verbose: bool = True,
) -> None:
    """Overlay a completed PGACoeffCalculator's results onto an EEPROM CSV.

    Loads a factory-reset EEPROM snapshot, overlays the calculator's
    coefficients and calibration settings onto it via
    :func:`apply_calibration_to_flat` (which also recomputes the CRC),
    and writes the result to a new CSV file. Bytes outside the
    calibration register block are preserved from the input.

    Args:
        input_csv_path: Path to the source tab-delimited paged CSV
            (16 pages x 8 bytes). Not modified.
        output_csv_path: Destination path. Overwritten if it exists.
        calc: A :class:`PGACoeffCalculator` on which
            ``recommend_calibration()``, ``normalize_data()``, and
            ``calculate_regression()`` have already been called.
        verbose: When True, prints ``calc.summarize_results()`` after
            writing.

    Raises:
        FileNotFoundError: ``input_csv_path`` does not exist.
        ValueError: CSV is malformed, or ``calc`` has no regression
            coefficients.
    """
    flat = paged_csv_to_flat(input_csv_path)
    flat = apply_calibration_to_flat(flat, calc)
    flat_to_paged_csv(flat, output_csv_path)

    if verbose:
        calc.summarize_results()

        print("\nOutput EEPROM layout:")
        flat = paged_csv_to_flat(output_csv_path)
        print(DELIMITER.join(HEADER))
        for page in range(NUM_PAGES):
            row = [format(page, "X")]
            row.extend(format(flat[page * BYTES_PER_PAGE + byte_idx], "02X") for byte_idx in range(BYTES_PER_PAGE))
            print(DELIMITER.join(row))


if __name__ == "__main__":
    from pga_coefficient_calculator import PGACoeffCalculator

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

    calc = PGACoeffCalculator(
        cal_point=(4, 4),
        device="PGA305",
        tad_matrix=tadc,
        pad_matrix=padc,
        dac_matrix=dac,
    )
    calc.recommend_calibration(offset_enabled=False)
    calc.normalize_data()
    calc.calculate_regression()

    output_csv_path = "PGA305_Calibrated.csv"
    write_calibrated_csv(
        input_csv_path="example_eeprom.csv",
        output_csv_path=output_csv_path,
        calc=calc,
        verbose=True,  # Print calibration summary
    )

    print(f"\nWrote calibrated EEPROM CSV to: {Path(output_csv_path).resolve()}")
