"""Icon regenerator - stdlib only (struct + zlib + base64), no Pillow.
Run: py -3 _gen_icon.py
Prints a base64 PNG string to paste into adc_monitor.py's
ICON_PNG_BASE64 constant. adc_monitor.py never imports this script or
PIL - the icon lives entirely as an embedded string, no asset file.
"""

import base64
import math
import struct
import zlib

BG = (0x1E, 0x1E, 0x2E, 255)
BORDER = (0x45, 0x47, 0x5A, 255)
CH1 = (0x89, 0xB4, 0xFA, 255)
CH2 = (0xF3, 0x8B, 0xA8, 255)


def make_pixels(size: int) -> list[tuple[int, int, int, int]]:
    pixels: list[tuple[int, int, int, int]] = [BG] * (size * size)
    for x in range(size):
        pixels[x] = BORDER
        pixels[(size - 1) * size + x] = BORDER
    for y in range(size):
        pixels[y * size] = BORDER
        pixels[y * size + size - 1] = BORDER
    for x in range(1, size - 1):
        t = x / (size - 1) * 2 * math.pi * 1.5
        y1 = max(1, min(size - 2, int(size * 0.35 + math.sin(t) * size * 0.18)))
        y2 = max(1, min(size - 2, int(size * 0.65 + math.sin(t + 1.0) * size * 0.12)))
        pixels[y1 * size + x] = CH1
        pixels[y2 * size + x] = CH2
    return pixels


def make_png(width: int, height: int, pixels: list) -> bytes:
    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(
            ">I", zlib.crc32(tag + data) & 0xFFFFFFFF
        )

    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)
    raw = bytearray()
    for y in range(height):
        raw.append(0)
        for x in range(width):
            raw.extend(pixels[y * width + x])
    idat = zlib.compress(bytes(raw), 9)
    return sig + chunk(b"IHDR", ihdr) + chunk(b"IDAT", idat) + chunk(b"IEND", b"")


def make_icon_base64(size: int = 32) -> str:
    png = make_png(size, size, make_pixels(size))
    return base64.b64encode(png).decode("ascii")


if __name__ == "__main__":
    b64 = make_icon_base64(32)
    width = 76
    lines = [b64[i : i + width] for i in range(0, len(b64), width)]
    print('ICON_PNG_BASE64 = (\n    "' + '"\n    "'.join(lines) + '"\n)')
