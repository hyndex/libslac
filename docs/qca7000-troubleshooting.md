# QCA7000 Troubleshooting Checklist

This document summarises a step-by-step procedure to diagnose issues when
bringing up a QCA7000 modem over SPI. It expands on the `qca7000-bring-up`
guide and highlights common pitfalls that lead to log messages such as
`Reset probe failed`, `modem missing`, or signature `0x5555` instead of
`0xAA55`.

## 1. Verify Boot-Strap Pins

Ensure the boot GPIOs on the QCA7000 are strapped correctly at power-up:

| Pin   | Purpose                     | Required Level for SPI Burst Mode |
|-------|-----------------------------|-----------------------------------|
| GPIO0 | Boot source                 | leave **unstrapped** (internal pull-up) |
| GPIO1 | Interface select            | **pull-down** to select SPI       |
| GPIO2 | Legacy vs burst CS          | **pull-up** for burst mode        |
| GPIO3 | —                           | don't care                        |

A wrong level on **GPIO2** forces legacy multi-CS mode and typically
results in the modem returning `0x5555` on the first read. Measure these
lines right after power-on to confirm the levels.

### Floating GPIO2

Without an external pull-up, the level on GPIO2 is undefined at power-up.
Whichever weak pull (internal or leakage) wins decides whether the modem
starts in burst or legacy mode. If the pin reads low and legacy mode is
selected, every 32‑bit SPI transfer is split internally into two 16‑bit
chunks. Drivers that keep CS low for 32 bits then read `0x5555` instead of
the expected `0xAA55` and `hardReset()` fails. Even slight moisture or
noise can flip the level, so behaviour may change between boards. Always
strap GPIO2 high with a resistor or implement the legacy‑mode handshake
as described in Qualcomm AN4 if a floating strap must be supported.

## 2. Confirm SPI Mode and Timing

* Use **SPI mode 3** (CPOL=1, CPHA=1). A wrong edge yields `0x5555`.
* Keep **CS low for the entire 32‑bit transaction** (command + data).
* Start with a clock around 4‑8 MHz (≤12 MHz required by the chip).
* Drive MOSI with valid data even for read operations.

A logic analyser trace of the very first transaction should show:

```
CS   _____--------------------------_____
MOSI 0x41 00 00 00  (read SIGNATURE)
MISO          xx xx AA 55
```

If you still read `0x5555`, double‑check clock polarity, phase and CS
behaviour.

## 3. Reset Sequence

A reliable start requires both a hardware reset pulse and a software
reset:

1. Toggle `RESET_L` low for at least 10 ms, then release.
2. Read `SIGNATURE` once and ignore the value.
3. Read `SIGNATURE` again – it must be `0xAA55`.
4. Enable interrupts `PKT_AVLBL`, `RDBUF_ERR`, `WRBUF_ERR`, `CPU_ON`.
5. Set the `SLAVE_RESET` bit in `SPI_CONFIG` and wait for `CPU_ON`.

If step 2 or 3 still returns `0x5555` the SPI link is not correct.

## 4. Power Integrity

The modem can draw up to 200 mA in bursts. Brown‑outs during the reset
sequence make the chip appear dead while SPI still returns `0x5555`.
Provide ample decoupling (100 µF bulk + 100 nF close to VDD) and keep the
3.3 V rail within the limits from the datasheet.

## 5. Driver Checks

Double-check the following in your software:

* SPI pin assignments in `SPI.begin()` match your wiring.
* Call `qca7000startSlac()` **only after** `hardReset()` succeeded.
* In burst mode the interrupt must be edge triggered; after servicing
  the ISR the IRQ line should drop.

## 6. Quick Hardware Sanity Tests

| Test                                  | Expected result                   |
|--------------------------------------|-----------------------------------|
| CS and CLK idle levels                | CS high, CLK high                |
| MISO state during reset               | 0 V (tri-stated)                 |
| Repeated `SIGNATURE` reads with scope | 0xAA55 every 32 µs @ 8 MHz        |

If the signature is valid only with the PLC transformer disconnected,
the analogue front-end may be shorted or wired incorrectly.

## 7. When `0xAA55` Appears …

* `hardReset()` should succeed and `CPU_ON` will be seen in
  `INTR_CAUSE`.
* Watch for `WRBUF_ERR` or `RDBUF_ERR` afterwards – these often mean the
  buffer size in `BFR_SIZE` didn't match the length you transferred.

Following this checklist usually resolves bring-up problems and lets you
open the SLAC channel successfully.

## 8. RX len mismatch Errors

Persistent "RX len mismatch" errors in the log indicate that the modem
and host no longer agree on the SPI framing. Noise or missing bytes can
break the length field so subsequent reads return garbage. The driver
resets the QCA7000 via `qca7000ResetAndCheck()` and then checks the
`MULTI_CS` bit in `SPI_REG_SPI_CONFIG` to confirm burst mode.

```cpp
qca7000ResetAndCheck();
uint16_t cfg = qca7000ReadInternalReg(SPI_REG_SPI_CONFIG);
if (cfg & QCASPI_MULTI_CS_BIT) {
    // device started in legacy mode – verify GPIO2 strap
}
```
