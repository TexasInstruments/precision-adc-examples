# ADC Monitor

A Python script, `adc_monitor.py`, that connects to a serial device (port auto-detected, or set via `--port`, @ 115200 baud by default), issues boot/stream commands, and renders a live dual-channel ADC plot with running statistics. It also has a `--demo` mode that generates synthetic data so the plot can be exercised without hardware.

## Install

```bash
pip install pyserial matplotlib numpy
```

Dependencies are also declared inline via PEP 723 script metadata at the top of `adc_monitor.py`, so `uv run` resolves them automatically without a separate install step.

## Usage

Run against real hardware:
```bash
python adc_monitor.py --port COM7   # Select COM port and run in the current environment 
uv run adc_monitor.py --port COM7   # Installs inline script dependencies and run in temporary environment
```

Run without hardware (synthetic sine + noise data on both channels):
```bash
python adc_monitor.py --demo
```

`--port`/`--baud` are ignored in `--demo` mode since no serial connection is opened.

There is no build step, test suite, or linter configured for this project.

## Tunables

Key settings live at the top of `adc_monitor.py`: `PORT`, `DEVICE_VID`/`DEVICE_PID` (USB VID:PID for auto-detection), `BAUD_RATE`, `WINDOW_SIZE` (visible sample count), `UPDATE_MS` (plot refresh rate), `BOOT_TIMEOUT`, `STREAM_TIMEOUT`, `MAX_RETRIES`.

Raise `UPDATE_MS` to redraw less often, or lower `WINDOW_SIZE` to shrink CPU/redraw load if the plot feels sluggish.

To regenerate the window icon artwork, run `_gen_icon.py` (stdlib-only PNG encoder, no Pillow) and paste its printed output over `ICON_PNG_BASE64` in `adc_monitor.py`. 