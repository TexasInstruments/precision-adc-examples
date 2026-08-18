# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

See [README.md](README.md) for the project overview, install steps, and run commands.

## Architecture

The script is organized as a single-threaded producer, animation-driven consumer:

- **Serial reader thread** (`serial_reader`, spawned as a daemon in `main`): owns the serial port and runs a state machine:
  1. `wait_for_boot` sends `sys reset` and blocks until a line containing `initialized` appears (or `BOOT_TIMEOUT` elapses), retrying the reset indefinitely.
  2. `start_stream` sends `adc stream` and waits for `ret: 0` confirmation, retrying up to `MAX_RETRIES` times on `command not found` responses.
  3. The main read loop parses each line with `HEX_PATTERN` (two `0x...` tokens per line = one CH1/CH2 sample pair) and appends values into the shared `ch1`/`ch2` deques under `lock`.
  4. A watchdog inside the read loop detects stalls (`STREAM_TIMEOUT` seconds without a sample) and re-issues `start_stream`; a `command not found` line at any point also triggers a stream restart.
- **Shared state**: `ch1`/`ch2` (bounded `deque(maxlen=WINDOW_SIZE)`), `total_samples`, `t_start`/`t_last_sample`, and `status_msg` are all mutated by the reader thread under `lock` and read by the plot callback without holding the lock for the full duration (snapshot-copy pattern).
- **Plot/animation** (`update`, driven by `matplotlib.animation.FuncAnimation` on the main thread): snapshots the deques, recomputes axis limits and per-channel statistics (`compute_stats`) every `UPDATE_MS`, and updates two time-series subplots plus a status/stats text bar. `status_msg` from the reader thread is surfaced directly in the plot so hardware-side state (booting, streaming, stalled, retrying) is visible without console access.
- **Demo mode** (`demo_reader`, used instead of `serial_reader` when `--demo` is passed): a daemon thread with no serial dependency that writes synthetic sine + Gaussian-noise samples into the same `ch1`/`ch2` deques under the same `lock`, at roughly the same cadence (~100 sps) as real hardware. Because it mutates the identical shared state, `update()` and the rest of the plotting code are unaware of which producer is running.

- **Window title/icon** (`set_window_title_and_icon`, called once in `main` before `plt.show()`): sets the OS title bar text and icon. The icon is a 32x32 PNG embedded as a base64 string (`ICON_PNG_BASE64`) rather than a shipped asset file - decoded in-memory via `tk.PhotoImage(data=...)` for TkAgg, or `QPixmap.loadFromData` for Qt backends. Regenerate the icon artwork with `_gen_icon.py` (stdlib-only PNG encoder, no Pillow) and paste its printed output over `ICON_PNG_BASE64`.
  - **Gotcha**: `window.iconphoto(default, image)` on TkAgg/Windows silently no-ops when `default=True` - the call raises no exception, but the title bar keeps showing matplotlib's own default icon. `default=False` is required for the icon to actually apply to the current window (confirmed via screenshot A/B test; matplotlib's own internal icon-setting call also uses `False`). The `-default` flag is for registering a fallback icon for *future* Toplevels, not a "just apply it" switch - don't assume `True` is the safe default.

- **Port auto-detection** (`find_port`, called at module load when `--port` is omitted and `--demo` is not set): filters `serial.tools.list_ports.comports()` by `DEVICE_VID`/`DEVICE_PID` if set. It only auto-selects silently when those are set and exactly one port matches that USB VID:PID - a confident identification. In every other case (VID/PID unset, or still multiple matches) it lists the candidate ports and prompts the user to pick one, since an unverified single port (e.g. an unrelated on-board COM device) must not be assumed correct. `DEVICE_VID`/`DEVICE_PID` are currently `None` because the real hardware IDs aren't known yet.

Because the reader thread retries forever on boot/stream failures, the script has no explicit "give up" path other than closing the plot window (which sets `running = False` and joins on port close in `main`'s `finally` block).

`ch1`/`ch2` are `deque(maxlen=WINDOW_SIZE)` rather than lists specifically so the reader thread's `append()` can evict the oldest sample in O(1) once the window is full; the equivalent with a list (`pop(0)` + `append`) would be O(n) per sample. The plot callback converts back to a list (`list(ch1)`) since numpy/matplotlib work needs random access, which deques don't provide efficiently.

## Performance notes

At the current scale (`WINDOW_SIZE=100`, 5 redraws/sec) the script has no real bottleneck; serial I/O blocks on hardware and per-frame numpy work is trivial. If optimizing further:

- `update()` calls `y1.min()`/`y1.max()` twice each (once for `span`, once for `set_ylim`) - cheap to memoize into local variables, but negligible at this size.
- `FuncAnimation` runs with `blit=False`, causing a full-canvas redraw every frame - the single largest per-frame cost. Switching to `blit=True` is not a clean win here because `set_xlim`/`set_ylim` are recomputed every frame (autoscaling), which invalidates the cached background blitting relies on. Only worth revisiting if the y-axis range were fixed instead of autoscaled.
- The real levers for CPU/redraw load are the existing tunables: raise `UPDATE_MS` to redraw less often, or lower `WINDOW_SIZE` to shrink the per-frame data set.
