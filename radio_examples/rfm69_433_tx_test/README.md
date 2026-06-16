# rfm69_433_tx_test

Minimal standalone Raspberry Pi Pico 2 W program that **transmits** a short
numbered test packet once a second on the **433 MHz RFM69HCW (RF69, LoRa1,
SPI1)** in **GFSK**, and prints the result + on-air time over USB serial.

Sibling of `rfm69_433_rx_test`. 433 MHz GFSK counterpart of the 915 MHz LoRa
`lora915_tx_test`. Run TX on one board and RX on another to verify the link
end-to-end.

## Config (matches ground-station LoRa1 / RF69)

- 424.500 MHz, GFSK (Gaussian shaping BT=0.5), 4.8 kbps, 5 kHz deviation,
  125 kHz RX bandwidth, 16-bit preamble, **+20 dBm** (HCW PA boost)
- SPI1: SCK=10, MOSI=11, MISO=8, NSS=9; DIO0=27, RST=26, power-enable=28

Pins, devices, and air config come from this target's `board_profile.hpp`.

## Build

```sh
mkdir build && cd build
cmake ..
make
```

Drag `build/rfm69_433_tx_test.uf2` onto the Pico while it's in BOOTSEL mode,
then open the USB serial port.

## Note

Transmits at +20 dBm (RFM69HCW high-power PA boost) into the antenna — always
have a proper antenna or matched load connected before powering on.
