# radio_examples

Standalone Pico 2 W radio bench-test tools, split out from the full ground-station
firmware so the RF link can be exercised in isolation (range tests, antenna
comparisons, sanity checks). Each is a self-contained CMake project.

| Example | Radio | Band / mode | Role |
|---------|-------|-------------|------|
| `lora915_rx_test`   | SX1276 (LoRa0, SPI0) | 915 MHz LoRa | continuous receive |
| `lora915_tx_test`   | SX1276 (LoRa0, SPI0) | 915 MHz LoRa | 1 Hz beacon transmit |
| `rfm69_433_rx_test`  | RFM69HCW (LoRa1, SPI1) | 433 MHz GFSK | continuous receive |
| `rfm69_433_tx_test`  | RFM69HCW (LoRa1, SPI1) | 433 MHz GFSK | 1 Hz beacon transmit |

Pins and air config are copied from the ground-station firmware
(`projects/ground_station/pico/src/shared.hpp`) so these talk to the real GS.

## Architecture (FreeRTOS SMP)

Each tool runs FreeRTOS SMP across the RP2350's two cores, mirroring the
ground-station firmware, with three tasks:

- **console task** (core 0) — owns **all** USB serial I/O. No other task touches
  `printf`/`getchar`. Other tasks enqueue output with `log_print()` /
  `log_puts()` (see [`common/rf_console.cpp`](common/rf_console.cpp)); the console
  task drains that queue and prints it, and reads typed commands (`list`,
  `export`, `format`, `live`, `help`). Pinned to core 0 because the TinyUSB IRQ
  lives there.
- **gps task** (core 1) — drains the UART0 RX IRQ ring and parses NMEA/UBX into a
  mutex-guarded shared fix. Because the UART is serviced by an interrupt and the
  task is scheduled preemptively, GPS bytes are no longer lost when the radio or
  flash logger is busy (the root cause of earlier dropped frames).
- **radio task** (core 1) — the per-tool RX/TX loop. Reads the latest GPS fix via
  `gps_task_fix()` and emits CSV rows.

Flash logging is made multicore-safe with `flash_safe_execute()` (supported under
FreeRTOS SMP) so a flash erase/program on one core can't fault the other while it
executes from XIP; all littlefs access is additionally serialized behind a mutex.
FreeRTOS config is [`common/FreeRTOSConfig.h`](common/FreeRTOSConfig.h); the
shared sources/libraries are wired up by
[`common/radio_common.cmake`](common/radio_common.cmake).

> Note: a flash **erase** briefly halts both cores' interrupts, so a GPS byte or
> two can still be lost the moment littlefs crosses a block boundary — the parser
> resyncs on the next sentence, so at most one fix is skipped.

## Shared output schema

All four emit the same CSV (one row per event) defined in
[`common/rf_csv.h`](common/rf_csv.h): frequency, RSSI/SNR, packet-loss counters,
`gps_lat`/`gps_lon`/`gps_alt_m` position, and a `utc` timestamp (last two from
the on-board GPS, below). The analysis tooling in `scripts/` (repo root) parses
these logs and plots antenna comparisons.

## On-board GPS (UART0, UBX NAV-PVT)

Each tool drives a u-blox GPS on **UART0** and stamps every CSV row with the
latest fix — `utc` (`YYYY-MM-DDTHH:MM:SSZ`) plus `gps_lat`/`gps_lon`/`gps_alt_m`.
On boot the module is auto-configured to emit **UBX NAV-PVT only** (NMEA
silenced) at 1 Hz; see [`common/gps_task.cpp`](common/gps_task.cpp).

Wiring (UART0 is free in every example — the radios use SPI0/SPI1):

| Pico pin | Signal | GPS |
|----------|--------|-----|
| GPIO 0 (phys 1) | UART0 TX | → GPS RX |
| GPIO 1 (phys 2) | UART0 RX | ← GPS TX |
| GND / 3V3 | power | — |

The GPS is serviced by a dedicated FreeRTOS task (core 1) fed by the UART0 RX
interrupt (see Architecture above). The columns stay blank with no GPS attached
or before the first fix.

> The GPS UART pins/baud differ per board — e.g. the 915 MHz tools use GPIO 13/12
> at 230400 in listen-only mode; check the `GPS_*` / `RADIO_GPS_*` constants at
> the top of each `main.cpp`.

## On-board file logging (littlefs)

Every CSV row is also written to a file on the Pico's flash, so a run survives
an unplugged laptop or a field test with no host attached. The 4 MB flash is
split into **1 MB program / 3 MB files**; the file region is a
[littlefs](../../libs/Third_Party/littlefs) filesystem driven by a small
on-chip-flash block device in [`common/lfs_pico_flash.c`](common/lfs_pico_flash.c).

On boot each tool mounts the FS (formatting it on first use), then opens a fresh
file `/<role>_NNNN.csv` (`rx`/`tx`), where `NNNN` is one past the highest
existing index — so older runs are never clobbered. Buffered rows are flushed to
flash about once a second. A `# [log] writing to /rx_0003.csv (N bytes free)`
comment is printed over USB at startup.

The wiring lives in [`common/rf_log.cpp`](common/rf_log.cpp) and is shared by all
four tools via [`common/radio_common.cmake`](common/radio_common.cmake)
(`LITTLEFS_PATH` comes from [`cmake/deps.cmake`](../../../cmake/deps.cmake)).
Under FreeRTOS SMP, flash erase/program goes through `flash_safe_execute()` so the
other core is parked during the operation, and all littlefs access is serialized
behind a mutex.

### Retrieving logs over USB (console)

The easiest way to get logs off the board is the built-in USB-serial console
([`common/rf_console.cpp`](common/rf_console.cpp)). Open the serial port and type:

| Command | Action |
|---------|--------|
| `list` | list stored logs with an index and byte size (the open file is marked `(active)`) |
| `export <n>` | dump the whole CSV of log `n` to the terminal |
| `format yes` | erase **all** logs and start a fresh file (destructive; `format` alone just warns) |
| `help` | show the commands |

Example session:

```
list
# logs (2, 3133440 bytes free):
#   0  rx_0000.csv     20480 bytes
#   1  rx_0001.csv      4096 bytes  (active)
# use 'export <n>' to dump one
export 0
# ---- begin export rx_0000.csv (20480 bytes) ----
timestamp_ms,role,freq_mhz,...,utc
1000,rx,915.0,...
...
# ---- end export rx_0000.csv ----
```

The dump is bracketed by `# ---- begin/end export ----` comment lines, so a
serial capture piped into the `scripts/` loader parses cleanly (it skips `#`
lines). An `export` runs to completion inside the console task while holding the
filesystem mutex, so the radio task's flash writes pause and no live rows
interleave with the dump. The active log can be exported too (it is flushed and
rewound through its own handle).

To pull the raw filesystem image instead, read the 3 MB file region (offset
`0x100000`) with `picotool save -r 0x10100000 0x10400000 logs.bin` and mount it
with the host `littlefs-fuse` / `lfs` tools.

## Build

From any example directory:

```sh
mkdir build && cd build
cmake .. && make
```

Drag the resulting `build/<name>.uf2` onto the Pico in BOOTSEL mode, then open
the USB serial port. Each project's own README has the details.
