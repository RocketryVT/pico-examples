# rfm69_433_rx_test

Minimal standalone Raspberry Pi Pico 2 W program that puts the **433 MHz
RFM69HCW (RF69, LoRa1, SPI1)** into continuous **GFSK** receive and prints
packets + RSSI + packet-loss (PER) over USB serial. It also samples the channel
**noise floor** once a second.

Sibling of `rfm69_433_tx_test`. 433 MHz GFSK counterpart of the 915 MHz LoRa
`lora915_rx_test`.

## Config (matches ground-station LoRa1 / RF69)

- 433.0 MHz, GFSK (Gaussian shaping BT=0.5), 4.8 kbps, 5 kHz deviation,
  125 kHz RX bandwidth, 16-bit preamble
- SPI1: SCK=10, MOSI=11, MISO=8, NSS=9; DIO0=27, RST=26, power-enable=28

Pins and air config are copied from
`projects/ground_station/pico/src/shared.hpp` and `lora1_task.cpp`.

## Build

```sh
mkdir build && cd build
cmake ..
make
```

Drag `build/rfm69_433_rx_test.uf2` onto the Pico while it's in BOOTSEL mode,
then open the USB serial port.

## Reading the output

FSK has **no SNR** (that's a LoRa/spread-spectrum concept), so link quality is
judged by RSSI margin above the noise floor and by PER:

- `noise floor X dBm` — current channel RSSI with no packet.
- `PACKET … RSSI …` then `link: good/lost/crc PER% RSSI avg/min/max` — a valid
  frame plus the running link summary. PER and the rolling RSSI come from the
  `#N` sequence number the TX embeds.
- `CRC ERROR …` — energy received but corrupt.
