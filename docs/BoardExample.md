# ESP32-S3 DevKit Board Example

This example demonstrates how to wire the QCA7000 modem to an
ESP32-S3 board using PlatformIO. The board uses custom SPI pins and
operates the modem without connecting the optional interrupt line.

## PlatformIO configuration

```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
monitor_speed = 115200
monitor_filters = esp32_exception_decoder
board_build.partitions = default_8MB.csv
board_upload.flash_size = 8MB
board_build.arduino.memory_type = dio_qspi
build_flags = \
    -DPLC_SPI_CS_PIN=41 \
    -DPLC_SPI_RST_PIN=40
```

The SPI bus is initialised by the driver using the pin macros from
`qca7000.hpp`. Chip select is driven manually, so `SPI.begin()` is
called with `-1` for the CS pin.

```cpp
qca7000_config cfg{&SPI, 41, 40, my_mac};
slac::port::Qca7000Link link(cfg);
while (true) {
    qca7000Process();
    // application tasks
}
```

No interrupt pin is used. The loop above shows how the modem can be
polled without relying on an external IRQ line.
