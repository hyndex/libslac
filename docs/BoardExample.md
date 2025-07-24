# Microcontroller DevKit Board Example

This example demonstrates how to wire the QCA7000 modem to a microcontroller board using PlatformIO. The board uses custom SPI pins and operates the modem without connecting the optional interrupt line.

## PlatformIO configuration

```ini
[env:microcontroller]
platform = espressif32
board = <your_board>
framework = arduino
monitor_speed = 115200
# monitor_filters = esp32_exception_decoder
board_build.partitions = default_8MB.csv
board_upload.flash_size = 8MB
board_build.arduino.memory_type = dio_qspi
build_flags = \
    -DPLC_SPI_CS_PIN=41 \
    -DPLC_SPI_RST_PIN=40
```

The SPI bus is started with the custom pins in `setup()`:

```cpp
SPI.begin(48 /*SCK*/, 21 /*MISO*/, 47 /*MOSI*/, 41 /*CS*/);
qca7000_config cfg{&SPI, 41, 40, my_mac};
slac::port::Qca7000Link link(cfg);
while (true) {
    qca7000Process();
    // application tasks
}
```

No interrupt pin is used. The loop above shows how the modem can be
polled without relying on an external IRQ line.
