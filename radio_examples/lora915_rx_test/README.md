# lora915_rx_test

Minimal standalone Raspberry Pi Pico 2 W program that puts the **915 MHz SX1276
(LoRa0, SPI0)** into continuous LoRa receive and prints packets + RSSI/SNR over
USB serial. It also samples the channel **noise floor** once a second.

## Why

We're running the radio through a DIY antenna with a terrible (~4 kOhm)
impedance. We expect it to perform badly — this is a quick second opinion to
cross-check a VNA reading we suspect might be wrong. If the noise floor moves
when a transmitter keys nearby, or any packet / CRC-error appears, the RF front
end is alive regardless of what the VNA says.

## Config (matches ground-station LoRa0)

- 915.0 MHz, BW 125 kHz, SF7, CR 4/5, sync word 0x12, preamble 8
- SPI0: SCK=18, MOSI=19, MISO=20, NSS=21; DIO0=17, RST=22, power-enable=16

Pins and air config are copied from
`projects/ground_station/pico/src/shared.hpp` and `lora_task.cpp`.

## Build

```sh
mkdir build && cd build
cmake ..
make
```

Drag `build/lora915_rx_test.uf2` onto the Pico while it's in BOOTSEL mode, then
open the USB serial port to watch the output.

## Reading the output

- `noise floor X dBm` — instantaneous channel RSSI with no packet. A working
  receiver typically floats around -100 to -120 dBm; it jumps up when it hears
  energy.
- `PACKET … RSSI … SNR …` — a valid frame decoded.
- `CRC ERROR …` — energy received but corrupt. Still proof the front end hears
  something — exactly what you want to see with a marginal antenna.
