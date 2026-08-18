"""
ADC Stream Monitor
==================
Connects to hardware over serial, resets it, streams ADC data, and plots
both channels in real-time with live statistics.

Run with: uv run adc_monitor.py [--port COM5] [--baud 115200]
If --port is omitted, the serial port is auto-detected by USB VID:PID.
"""

# Copyright (c) 2026, Texas Instruments Incorporated
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# *  Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# *  Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# *  Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# /// script
# requires-python = ">=3.9"
# dependencies = [
#     "pyserial",
#     "matplotlib",
#     "numpy",
# ]
# ///

import argparse
import base64
import re
import sys
import threading
import time
from collections import deque
from contextlib import suppress

with suppress(AttributeError):
    sys.stdout.reconfigure(errors="replace")  # type: ignore

import matplotlib.pyplot as plt  # type: ignore
import numpy as np  # type: ignore
import serial  # type: ignore
from matplotlib import animation, gridspec  # type: ignore
from matplotlib.ticker import FuncFormatter  # type: ignore
from matplotlib.widgets import RadioButtons  # type: ignore
from serial.tools import list_ports  # type: ignore


# ---------------------------------------------------------------------------
# CLI arguments
# ---------------------------------------------------------------------------
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ADC Stream Monitor")
    parser.add_argument(
        "--port",
        default=None,
        help="Serial port (default: auto-detect by USB VID:PID)",
    )
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Generate synthetic data instead of connecting to real hardware",
    )
    return parser.parse_args()


def find_port(vid: "int | None", pid: "int | None") -> str:
    """Auto-detect the serial port by USB VID:PID, or prompt the user to
    pick one if VID/PID aren't set or multiple ports match."""
    all_ports = list_ports.comports()
    filtering = vid is not None and pid is not None
    matches = [p for p in all_ports if p.vid == vid and p.pid == pid] if filtering else all_ports

    if not matches:
        if filtering:
            print(f"[error] No serial port found with VID:PID {vid:04X}:{pid:04X}.")
        else:
            print("[error] No serial ports detected.")
        raise SystemExit(1)

    if filtering and len(matches) == 1:
        p = matches[0]
        print(f"[detect] Found device on {p.device} ({p.description})")
        return p.device

    print("Serial ports found:" if not filtering else "Multiple serial ports found:")
    for i, p in enumerate(matches, 1):
        port_id = f"{p.vid:04X}:{p.pid:04X}" if p.vid is not None else "?"
        print(f"  [{i}] {p.device}  ({port_id})  {p.description}")
    while True:
        choice = input(f"Select port [1-{len(matches)}] (q to cancel): ").strip()
        if choice.lower() in ("q", "quit"):
            print("Cancelled.")
            raise SystemExit(1)
        if choice.isdigit() and 1 <= int(choice) <= len(matches):
            return matches[int(choice) - 1].device
        print("Invalid selection, try again.")


args = parse_args()

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
# USB VID:PID for auto-detecting the serial port. Unknown for now, so
# find_port() falls back to listing every detected port and prompting.
DEVICE_VID = None
DEVICE_PID = None

if args.port:
    PORT = args.port
elif args.demo:
    PORT = None  # unused in demo mode
else:
    PORT = find_port(DEVICE_VID, DEVICE_PID)

BAUD_RATE = args.baud
WINDOW_SIZE = 100  # number of samples visible in plot
UPDATE_MS = 200  # plot refresh interval in milliseconds
BOOT_TIMEOUT = 8.0  # max seconds to wait for 'initialized...'
STREAM_TIMEOUT = 3.0  # seconds without data before re-sending 'adc stream'
MAX_RETRIES = 5  # max times to retry a command on 'command not found'
ADC_RESOLUTION_BITS = 24  # ADC resolution; sets hex digit width and two's-complement mask
HEX_DIGITS = ADC_RESOLUTION_BITS // 4
HEX_MASK = (1 << ADC_RESOLUTION_BITS) - 1

# Matches any line containing two 0x... tokens (hex of any length)
HEX_PATTERN = re.compile(r"(0x[0-9A-Fa-f]+)\s+(0x[0-9A-Fa-f]+)")

WINDOW_TITLE = "ADC Stream Monitor"
ICON_PNG_BASE64 = (
    "iVBORw0KGgoAAAANSUhEUgAAACAAAAAgCAYAAABzenr0AAAA00lEQVR42u2XzQ2AIAxGmcQR3MK4"
    "gxsZF3EAN/PmRYMJpGqB/hAJCQeMiW2/10KrmmGczpLL2EvX9UVWA6gfYN6OexUBgMJSCDEAJiiB"
    "EAHEhLgQ9R1CSoacKrAAOIGpth5A00qaFiZXQAJI8XkAhBw01UnF/FTgj62AGugWuPEaGrP7sorG"
    "NRaP1QVW2InD+1/mQEhMA2F9SQApEQmE80kCUINzIKBtFICbGcX+bZP9ZRSDwJ6hANoTjvmG4nkA"
    "J5qjtWASqZjtq7gBNAAPUPL3/AJj90p5B0cv5QAAAABJRU5ErkJggg=="
)

# Catppuccin Mocha palette
THEME = {
    "bg": "#1e1e2e",
    "plot_bg": "#181825",
    "grid": "#313244",
    "border": "#45475a",
    "text": "#cdd6f4",
    "muted": "#6c7086",
    "ch1": "#89b4fa",
    "ch1_mean": "#74c7ec",
    "ch2": "#f38ba8",
    "ch2_mean": "#fab387",
    "info": "#a6e3a1",
    "rate": "#f9e2af",
}

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
ch1 = deque(maxlen=WINDOW_SIZE)
ch2 = deque(maxlen=WINDOW_SIZE)
lock = threading.Lock()

total_samples = 0
t_start = None  # monotonic time when first sample arrived
t_last_sample = None  # monotonic time of most recent sample
status_msg = "Starting demo..." if args.demo else "Connecting..."
running = True
hex_mode = False  # toggled by the Decimal/Hex selector widget


# ---------------------------------------------------------------------------
# Serial helpers
# ---------------------------------------------------------------------------
def send_cmd(ser: serial.Serial, cmd: str) -> None:
    r"""Send a command with LF terminator (device requires \n)."""
    print(f"[tx] {cmd}")
    ser.write((cmd + "\n").encode())


def wait_for_boot(ser: serial.Serial) -> bool:
    """
    Send 'sys reset', then read until 'initialized...' appears or timeout.
    Returns True on success, False on timeout.
    """
    global status_msg
    status_msg = "Resetting device..."
    ser.reset_input_buffer()
    send_cmd(ser, "sys reset")
    deadline = time.monotonic() + BOOT_TIMEOUT
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if line:
            print(f"[boot] {line}")
            status_msg = f"Booting: {line}"
        if "initialized" in line.lower():
            status_msg = "Initialized - starting stream..."
            print("[boot] Device ready.")
            return True
    print("[boot] Timeout waiting for 'initialized...'")
    status_msg = "Boot timeout - retrying..."
    return False


def start_stream(ser: serial.Serial) -> bool:
    """
    Send 'adc stream' and wait for 'ret: 0' confirmation.
    Retries up to MAX_RETRIES times if 'command not found' is received.
    Returns True when confirmed, False if all retries exhausted.
    """
    global status_msg
    for attempt in range(1, MAX_RETRIES + 1):
        status_msg = f"Starting stream (attempt {attempt}/{MAX_RETRIES})..."
        ser.reset_input_buffer()
        send_cmd(ser, "adc stream")
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            print(f"[stream-init] {line}")
            if "ret: 0" in line:
                status_msg = "Streaming"
                print("[stream] Stream confirmed (ret: 0).")
                return True
            if "command not found" in line:
                print(f"[stream] Command not found on attempt {attempt}, retrying...")
                time.sleep(0.3)
                break  # inner loop -> retry outer loop
    print("[stream] Failed to start stream after retries.")
    status_msg = "Stream start failed - will retry..."
    return False


def restart_stream(ser: serial.Serial) -> None:
    """Retry 'adc stream' until confirmed. Assumes the device is already booted."""
    while running:
        if start_stream(ser):
            return
        time.sleep(0.5)


def reboot_and_stream(ser: serial.Serial) -> None:
    """Full boot + stream handshake, retrying indefinitely until confirmed."""
    while running and not wait_for_boot(ser):
        pass
    if not running:
        return
    time.sleep(0.1)
    while running:
        if start_stream(ser):
            return
        time.sleep(0.5)
        while running and not wait_for_boot(ser):
            pass


# ---------------------------------------------------------------------------
# Serial reader thread
# ---------------------------------------------------------------------------
def serial_reader(ser: serial.Serial) -> None:
    """Boot the device, start the ADC stream, then read samples until the
    plot window closes, restarting the stream on stalls or errors."""
    global total_samples, t_start, t_last_sample, status_msg

    reboot_and_stream(ser)
    if not running:
        return

    # --- Main data-read loop ---
    while running:
        try:
            raw = ser.readline()
            if not raw:
                # Timeout on readline -- check watchdog
                if t_last_sample is not None:
                    gap = time.monotonic() - t_last_sample
                    if gap > STREAM_TIMEOUT:
                        print(f"[watchdog] No data for {gap:.1f}s - restarting stream...")
                        status_msg = "Stream stalled - restarting..."
                        restart_stream(ser)
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            m = HEX_PATTERN.search(line)
            if m:
                v1 = int(m.group(1), 16)
                v2 = int(m.group(2), 16)
                now = time.monotonic()
                with lock:
                    ch1.append(v1)
                    ch2.append(v2)
                    total_samples += 1
                    t_last_sample = now
                    if t_start is None:
                        t_start = now
                    status_msg = "Streaming"
            elif "command not found" in line:
                print(f"[warn] Stream interrupted: {line}")
                status_msg = "Command error - restarting stream..."
                restart_stream(ser)
            elif line:
                print(f"[hw] {line}")

        except serial.SerialException as exc:
            print(f"[serial] Error: {exc}")
            status_msg = f"Serial error: {exc}"
            break
        except Exception as exc:  # noqa: BLE001 - keep reader thread alive on any unexpected error
            print(f"[reader] Unexpected error: {exc}")


# ---------------------------------------------------------------------------
# Demo data generator (no hardware required)
# ---------------------------------------------------------------------------
def demo_reader() -> None:
    """Generate synthetic two-channel ADC data in place of a real serial link."""
    global total_samples, t_start, t_last_sample, status_msg

    status_msg = "Streaming (demo)"
    rng = np.random.default_rng()
    phase = 0.0

    while running:
        phase += 0.15
        v1 = 0.0 + 20000.0 * np.sin(phase) + rng.normal(0, 150)  # bipolar, zero-centered
        v2 = 30000.0 + 2500.0 * np.sin(phase * 0.6 + 1.0) + rng.normal(0, 300)  # offset (nonzero mean)
        now = time.monotonic()
        with lock:
            ch1.append(v1)
            ch2.append(v2)
            total_samples += 1
            t_last_sample = now
            if t_start is None:
                t_start = now
        time.sleep(0.01)  # ~100 sps


# ---------------------------------------------------------------------------
# Statistics helper
# ---------------------------------------------------------------------------
def compute_stats(data: list[float]) -> dict[str, float]:
    """Compute mean, std, min/max, peak-to-peak, and RMS for one channel's samples."""
    if len(data) < 2:
        return {}
    a = np.asarray(data, dtype=float)
    return {
        "n": len(a),
        "mean": a.mean(),
        "std": a.std(),
        "p2p": a.max() - a.min(),
        "min": a.min(),
        "max": a.max(),
        "rms": np.sqrt(np.mean(a**2)),  # true RMS, DC included - differs from std when mean != 0
    }


def stats_table(s1: dict[str, float], s2: dict[str, float], hex_mode: bool = False) -> str:
    """Format CH1/CH2 stats as an aligned monospace table for the on-plot info box."""
    if not s1 or not s2:
        return "Waiting for data..."

    def fmt_count(v: float, decimals: int = 0) -> str:
        if hex_mode:
            return f"0x{round(v) & HEX_MASK:0{HEX_DIGITS}X}"
        return f"{v:.{decimals}f}"

    columns = [
        ("Mean", lambda s: fmt_count(s["mean"], 1)),
        ("P2P", lambda s: f"{s['p2p']:.0f}"),
        ("StdDev", lambda s: f"{s['std']:.2f}"),
        ("RMS", lambda s: f"{s['rms']:.2f}"),
        ("Min", lambda s: fmt_count(s["min"])),
        ("Max", lambda s: fmt_count(s["max"])),
        ("N", lambda s: f"{s['n']}"),
    ]
    label_w, col_w = 6, 9
    header = " " * label_w + "".join(f"{name:>{col_w}}" for name, _ in columns)
    row1 = f"{'CH1':<{label_w}}" + "".join(f"{fn(s1):>{col_w}}" for _, fn in columns)
    row2 = f"{'CH2':<{label_w}}" + "".join(f"{fn(s2):>{col_w}}" for _, fn in columns)
    return f"{header}\n{row1}\n{row2}"


# ---------------------------------------------------------------------------
# Build figure
# ---------------------------------------------------------------------------
fig = plt.figure(figsize=(14, 9))
fig.set_facecolor(THEME["bg"])
plt.rcParams.update(
    {
        "text.color": THEME["text"],
        "axes.labelcolor": THEME["text"],
        "xtick.color": THEME["muted"],
        "ytick.color": THEME["muted"],
    }
)

gs = gridspec.GridSpec(
    3,
    1,
    figure=fig,
    height_ratios=[5, 5, 3],
    hspace=0.5,
    left=0.08,
    right=0.97,
    top=0.93,
    bottom=0.06,
)

ax1 = fig.add_subplot(gs[0])
ax2 = fig.add_subplot(gs[1])
ax_info = fig.add_subplot(gs[2])
ax_info.axis("off")
ax_info.set_facecolor(THEME["bg"])


def format_count(x, _):
    """Y-axis tick formatter; switches to hex when the display-mode widget is set to Hex."""
    if hex_mode:
        return f"0x{round(x) & HEX_MASK:0{HEX_DIGITS}X}"
    return f"{int(x):,}"


for ax in (ax1, ax2):
    ax.set_facecolor(THEME["plot_bg"])
    ax.grid(True, color=THEME["grid"], linewidth=0.5, linestyle="--")
    ax.spines[:].set_color(THEME["border"])
    ax.tick_params(labelsize=8)
    ax.yaxis.set_major_formatter(FuncFormatter(format_count))

_title_suffix = "DEMO MODE" if args.demo else f"{PORT} @ {BAUD_RATE}"
fig.suptitle(
    f"ADC Stream Monitor  -  {_title_suffix}",
    fontsize=13,
    color=THEME["text"],
    fontweight="bold",
)

(line1,) = ax1.plot([], [], color=THEME["ch1"], linewidth=0.9, label="CH1")
(mean1,) = ax1.plot([], [], color=THEME["ch1_mean"], linewidth=1.2, linestyle="--", alpha=0.75, label="mean")
(line2,) = ax2.plot([], [], color=THEME["ch2"], linewidth=0.9, label="CH2")
(mean2,) = ax2.plot([], [], color=THEME["ch2_mean"], linewidth=1.2, linestyle="--", alpha=0.75, label="mean")

for ax, title in ((ax1, "Channel 1"), (ax2, "Channel 2")):
    ax.set_title(title, color=THEME["text"], fontsize=10, pad=4)
    ax.set_xlabel("Sample index", fontsize=8)
    ax.set_ylabel("ADC count", fontsize=8)
    ax.legend(
        loc="upper right",
        fontsize=8,
        facecolor=THEME["grid"],
        edgecolor=THEME["border"],
        labelcolor=THEME["text"],
    )

info_text = ax_info.text(
    0.03,
    0.95,
    "Waiting for data...",
    transform=ax_info.transAxes,
    fontsize=9,
    va="top",
    ha="left",
    fontfamily="monospace",
    color=THEME["info"],
    linespacing=1.5,
)
rate_text = ax_info.text(
    0.995,
    0.95,
    "",
    transform=ax_info.transAxes,
    fontsize=8.5,
    va="top",
    ha="right",
    fontfamily="monospace",
    color=THEME["rate"],
)

# Decimal/Hex display-mode selector (matplotlib has no native dropdown widget,
# so RadioButtons is the closest equivalent).
ax_radio = fig.add_axes([0.855, 0.945, 0.115, 0.045])
ax_radio.set_facecolor(THEME["bg"])
for spine in ax_radio.spines.values():
    spine.set_color(THEME["border"])
radio_display = RadioButtons(
    ax_radio,
    ("Decimal", "Hex"),  # type: ignore[arg-type]
    active=0,
    activecolor=THEME["info"],
)
for label in radio_display.labels:
    label.set_color(THEME["text"])
    label.set_fontsize(8)


def on_display_mode(label: str) -> None:
    global hex_mode
    hex_mode = label == "Hex"


radio_display.on_clicked(on_display_mode)


# ---------------------------------------------------------------------------
# Animation update callback
# ---------------------------------------------------------------------------
def update(_frame):
    """FuncAnimation callback: redraw both channel plots and the stats/rate text."""
    with lock:
        d1 = list(ch1)
        d2 = list(ch2)
        n_total = total_samples

    n = min(len(d1), len(d2))

    if n == 0:
        info_text.set_text(status_msg)
        return line1, mean1, line2, mean2, info_text, rate_text

    x = np.arange(n)

    # Channel 1
    y1 = np.asarray(d1[:n], dtype=float)
    line1.set_data(x, y1)
    m1 = y1.mean()
    mean1.set_data([0, n - 1], [m1, m1])
    span1 = max(1.0, y1.max() - y1.min())
    ax1.set_xlim(0, max(n - 1, 1))
    ax1.set_ylim(y1.min() - span1 * 0.15, y1.max() + span1 * 0.15)

    # Channel 2
    y2 = np.asarray(d2[:n], dtype=float)
    line2.set_data(x, y2)
    m2 = y2.mean()
    mean2.set_data([0, n - 1], [m2, m2])
    span2 = max(1.0, y2.max() - y2.min())
    ax2.set_xlim(0, max(n - 1, 1))
    ax2.set_ylim(y2.min() - span2 * 0.15, y2.max() + span2 * 0.15)

    # Stats table
    s1 = compute_stats(d1[:n])
    s2 = compute_stats(d2[:n])
    info_text.set_text(stats_table(s1, s2, hex_mode))

    # Rate / status (top-right)
    if t_start is not None and n_total > 0:
        elapsed = time.monotonic() - t_start
        rate = n_total / elapsed if elapsed > 0 else 0.0
        rate_text.set_text(f"status: {status_msg}  |  {rate:.1f} sps  |  total: {n_total:,}")
    else:
        rate_text.set_text(f"status: {status_msg}")

    return line1, mean1, line2, mean2, info_text, rate_text


_icon_image_ref = None  # keeps the Tk PhotoImage alive (Tk drops it otherwise)


def set_window_title_and_icon(fig, title: str, icon_png_base64: str) -> None:
    """Set the OS title bar text and icon from an in-memory PNG (no asset
    file on disk). Backend-agnostic (Tk/Qt); no-ops silently if the backend
    doesn't support it."""
    global _icon_image_ref

    manager = fig.canvas.manager
    with suppress(Exception):
        manager.set_window_title(title)

    window = getattr(manager, "window", None)
    if window is None:
        return

    with suppress(Exception):
        import tkinter as tk  # TkAgg

        _icon_image_ref = tk.PhotoImage(data=icon_png_base64)
        window.iconphoto(False, _icon_image_ref)
        return

    with suppress(Exception):
        from matplotlib.backends.qt_compat import QtGui  # Qt5Agg / QtAgg  # type: ignore

        pixmap = QtGui.QPixmap()
        pixmap.loadFromData(base64.b64decode(icon_png_base64))
        window.setWindowIcon(QtGui.QIcon(pixmap))


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    """Open the serial port (or start demo mode), launch the reader thread, and run the plot."""
    global running

    ser = None
    if args.demo:
        print("[demo] Running in demo mode - no serial connection.")
        reader = threading.Thread(target=demo_reader, daemon=True)
    else:
        try:
            ser = serial.Serial(PORT, BAUD_RATE, timeout=0.5)
            print(f"[serial] Opened {PORT} at {BAUD_RATE} baud")
        except serial.SerialException as exc:
            print(f"[error] Cannot open {PORT}: {exc}")
            return
        reader = threading.Thread(target=serial_reader, args=(ser,), daemon=True)

    reader.start()

    set_window_title_and_icon(fig, WINDOW_TITLE, ICON_PNG_BASE64)

    ani = animation.FuncAnimation(  # noqa: F841  keep reference alive
        fig,
        update,
        interval=UPDATE_MS,
        blit=False,
        cache_frame_data=False,
    )

    try:
        plt.show()
    finally:
        running = False
        time.sleep(0.4)
        if ser is not None:
            ser.close()
            print("[serial] Port closed.")


if __name__ == "__main__":
    main()
