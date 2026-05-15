# Battery Monitor

A 12 V off-grid battery monitor for Arduino Mega 2560. Watches one battery
bank fed by three charge sources (solar, AC charger, car alternator) plus one
load line, computes state-of-charge by coulomb counting, drives a 16x2 RGB
LCD with a 7-screen menu, and logs CSV to an SD card.

Designed for camper / overland / off-grid-shed installs where you have
multiple charge sources, no grid fallback, and want to know — at a glance —
whether the bank is net-charging, how much energy you've burned today, and
roughly how much usable capacity is left.

## Highlights

- Five analog channels: battery voltage + four current sensors (ACS712-style).
- **Relay-gated voltage sense** — the divider is only powered for 200 ms
  per sample so it isn't bleeding milliamps continuously.
- **Coulomb-counter SOC** with periodic re-anchor at float voltage.
- **CSV logging** to SD with automatic file rotation at 512 KiB.
- **7 LCD screens**: summary, currents, power, energy, min/max V, log
  status, firmware info. Backlight colour reflects battery state
  (green = charging, yellow = low, red = critical).
- **Idle deep-sleep** drops the MCU into `SLEEP_MODE_PWR_DOWN` after 5 min
  with no button activity; wakes on INT0.
- **Web simulator** in `sim/` runs the firmware logic in the browser so you
  can drive the buttons and watch the LCD without hardware.
- **Unit tests** for the entire math layer run on the host (no AVR needed).

## Quick start

### Build / flash the firmware

```bash
pio run                 # build
pio run -t upload       # flash to Mega
pio device monitor      # 9600 baud — see boot + log output
```

`platformio.ini` declares all library dependencies, so a fresh checkout
builds without manual library installation.

### Run the host-side math tests

Either of these works:

```bash
# Quickcheck via vanilla g++ (no PlatformIO needed):
g++ -std=gnu++17 -O2 -Wall -Wextra -Iinclude test/host_quickcheck.cpp -o quickcheck && ./quickcheck

# Or via PlatformIO's Unity runner:
pio test -e native
```

Both exercise the same pure-math layer (`CurrentMath`, `SocLookup`,
`StateOfCharge`, `EnergyAccumulator`, `RingBuffer`).

### Try the web simulator

```bash
cd sim && python3 server.py
# then open http://localhost:8765
```

You'll see a virtual LCD, the five buttons, sliders for sun / charger /
alternator / load, and live gauges. Everything is driven by a JavaScript
port of the firmware logic — same calibration constants, same SOC table,
same screen layouts.

## Architecture

```
include/
  Config.h               Pins, calibration, intervals — single source of truth
  CurrentMath.h          ADC → amps / volts conversion (header-only, host-testable)
  SocLookup.h            Voltage → SOC% lookup table
  StateOfCharge.h        Coulomb-counter SOC estimator with re-anchor
  EnergyAccumulator.h    Watt-hour integrator
  RingBuffer.h           Fixed-capacity float buffer for min/max/mean
  Measurements.h         Sample struct + field enum + labels
  Sampler.h              Hardware-coupled ADC reader (Arduino dep)
  Logger.h               SD-card CSV logger with rotation
  Display.h              LCD UI + button debouncing + menu navigation
  PowerManager.h         Idle-timeout deep sleep
src/
  main.cpp               Wiring: two scheduler threads + setup/loop
  Sampler.cpp            Drives the voltage relay and reads ADC channels
  Logger.cpp             File rotation + CSV emission
  Display.cpp            7 LCD screens, backlight state machine
  PowerManager.cpp       SLEEP_MODE_PWR_DOWN + INT0 wake
  Measurements.cpp       Field labels
sim/
  index.html / sim.js    Browser-only simulator (same calibration constants)
  server.py              Tiny static file server for local serving
test/
  host_quickcheck.cpp    Vanilla g++ test runner
  test_native/           PlatformIO Unity tests (CI)
```

The pure-math headers don't include `<Arduino.h>` — they're compiled by both
the AVR build and host tooling. Hardware-coupled code is gated on
`TARGET_AVR` so the same .cpp files can technically also be compiled for the
host (currently the AVR path is the only one wired up; the simulator
re-implements the same logic in JS).

## Hardware

| Function              | Pin       | Notes                                     |
|-----------------------|-----------|-------------------------------------------|
| Battery voltage sense | A0        | Through divider, gated by relay on D8     |
| Load current          | A1        | ACS712-style, 66 mV/A                     |
| Car alternator        | A2        | ACS712-style                              |
| AC charger            | A3        | ACS712-style                              |
| Solar array           | A9        | ACS712-style                              |
| Wake interrupt        | D2 (INT0) | Active-low, pulled up                     |
| Voltage relay         | D8        | Drives divider closed only during sample  |
| SD card               | D10–D13   | CS / MOSI / MISO / SCK                    |
| LCD                   | I²C       | Adafruit RGB LCD Shield (MCP23017)        |

See `docs/hardware.md` for wiring notes, BOM, and calibration procedure.

## Calibration

The defaults in `include/Config.h` match a specific build. Recalibrate per
install:

1. **Zero-current offsets** (`kSolarOffsetA`, `kDrawOffsetA`, etc.) —
   disconnect each sense wire so no current flows through the sensor; read
   the firmware's reported current; the value (with sign flipped) is the
   offset.
2. **Voltage divider** (`kBatteryDividerVmax`) — feed a known DC source,
   note the firmware-reported voltage versus a multimeter, scale.
3. **Battery capacity** (`kCapacityAh`) — nameplate of the bank.
4. **SOC voltage table** — defaults are flooded lead-acid at 25 °C. AGM,
   gel, and LiFePO4 all need different curves; edit `kSocCurve` in
   `include/SocLookup.h`.

After editing, re-run `pio test -e native` to confirm the math still
passes its sanity checks, then reflash.

## CI

GitHub Actions (`.github/workflows/ci.yml`) runs on every push:

- **build-firmware** — `pio run -e mega`, fails if AVR build breaks.
- **unit-tests** — `pio test -e native`.
- **host-math-quickcheck** — vanilla g++ on `test/host_quickcheck.cpp`.
- **lint** — cppcheck across `src/` and `include/`.

## License

MIT — see [LICENSE](LICENSE).
