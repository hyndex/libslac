# QCA7000 Bring-up Guide

This short guide covers basic wiring and verification of a QCA7000-based power line modem. It summarises default pin assignments, shows what a successful start-up log looks like and points to logic-analyser traces from the original application note. Common pitfalls from the SPI guide are repeated for convenience.

## Pin Configuration

The ESP32-S3 port defines default SPI pins in `port/esp32s3/qca7000.hpp`:

| Signal        | Macro                | Default Pin |
|---------------|---------------------|-------------|
| SPI MISO      | `PLC_SPI_MISO_PIN`  | 21          |
| SPI MOSI      | `PLC_SPI_MOSI_PIN`  | 47          |
| SPI SCK       | `PLC_SPI_SCK_PIN`   | 48          |
| Chip Select   | `PLC_SPI_CS_PIN`    | 17          |
| Reset         | `PLC_SPI_RST_PIN`   | 5           |

Override these macros or pass explicit values to `qca7000_config` if your wiring differs. The optional interrupt line is unused by the provided driver and may remain unconnected.

## Typical Serial Log

Running `examples/platformio_complete` on an ESP32-S3 yields output similar to:

```text
Starting SLAC modem...
Starting SPI
Starting QCA7000 Link 
```

If `channel.open()` fails the example prints `Failed to open SLAC channel, aborting` and stays in an error loop.

## Logic-Analyser Traces

`docs/QCA700X.md` includes several SPI timing diagrams useful for debugging:

- **Figure 1 & 2** – reading `SPI_REG_SIGNATURE`
- **Figure 3** – interrupt handling sequence
- **Figure 4** – external read transaction
- **Figure 5** – external write transaction

These were captured with a logic analyser that provides at least five channels at 24 MHz sampling, as recommended in the note.

## Common Pitfalls

The SPI guide lists multiple mistakes that frequently cause issues:

- Using a SPI mode other than mode 3.
- Neglecting the QCA framing or forgetting to pad short frames to 60 bytes.
- Toggling chip select in the middle of a transfer.
- Failing to clear interrupt causes, leading to lost or repeated interrupts.
- Reading or writing beyond the indicated buffer length or not resetting after buffer errors.

Refer to `docs/QCA_SPI.md` for the full discussion of these and other pitfalls.
