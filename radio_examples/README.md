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

## Shared output schema

All four emit the same CSV (one row per event) defined in
[`common/rf_csv.h`](common/rf_csv.h): frequency, RSSI/SNR, packet-loss counters,
and blank `gps_*` columns reserved for future GPS logging. The analysis tooling
in `scripts/` (repo root) parses these logs and plots antenna comparisons.

## Build

From any example directory:

```sh
mkdir build && cd build
cmake .. && make
```

Drag the resulting `build/<name>.uf2` onto the Pico in BOOTSEL mode, then open
the USB serial port. Each project's own README has the details.
