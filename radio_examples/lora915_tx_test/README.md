# lora915_tx_test

Minimal standalone Raspberry Pi Pico 2 W program that **transmits** a short
numbered test packet once a second on the **915 MHz SX1276 (LoRa0, SPI0)** and
prints the result + on-air time over USB serial.

Sibling of `lora915_rx_test` — same pins and air config. Run TX on one board
(COTS antenna) and RX on another to verify the link end-to-end.

## Config (matches ground-station LoRa0)

- 915.0 MHz, BW 125 kHz, SF7, CR 4/5, sync word 0x12, preamble 8, **+20 dBm**
- SPI0: SCK=18, MOSI=19, MISO=20, NSS=21; DIO0=17, RST=22, power-enable=16

Pins and air config are copied from
`projects/ground_station/pico/src/shared.hpp` and `lora_task.cpp`.

## Build

```sh
mkdir build && cd build
cmake ..
make
```

Drag `build/lora915_tx_test.uf2` onto the Pico while it's in BOOTSEL mode, then
open the USB serial port to watch the output.

## Note

Transmits at +20 dBm into the antenna — always have a proper (COTS) antenna or
matched load connected before powering on, never an open/badly matched port.
