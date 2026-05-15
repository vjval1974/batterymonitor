# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

Battery monitor firmware for an Arduino Mega 2560 + accompanying browser
simulator. The device watches a 12 V off-grid battery bank fed by three
charge sources (solar, AC charger, car alternator) and one load line,
estimates state-of-charge by coulomb counting, drives a 16x2 RGB LCD with a
7-screen menu, and logs CSV to an SD card. A `sim/` directory contains a
1:1 JavaScript port of the firmware's math and screen layouts so the device
can be driven from a browser without hardware.

## Common commands

| Goal                              | Command                                               |
|-----------------------------------|-------------------------------------------------------|
| Build firmware (Mega)             | `pio run`                                             |
| Flash firmware                    | `pio run -t upload`                                   |
| Serial monitor                    | `pio device monitor` (9600 baud)                      |
| Native unit tests (Unity)         | `pio test -e native`                                  |
| Host quickcheck (no pio needed)   | `g++ -std=gnu++17 -O2 -Wall -Iinclude test/host_quickcheck.cpp -o /tmp/qc && /tmp/qc` |
| Simulator JS cross-check          | `node sim/test_firmware.mjs`                          |
| Run the simulator                 | `python3 sim/server.py` then open http://localhost:8765 |
| Static analysis                   | `pio check` (cppcheck)                                |

CI (`.github/workflows/ci.yml`) runs all four: AVR build, native tests, host
quickcheck, cppcheck lint.

## Architecture

### Pure-math layer (host-testable, no Arduino dep)

These headers compile under both AVR and host g++. The same headers are
used by the simulator's JS port — keep the calibration constants and SOC
table in lockstep across `include/Config.h`, `include/SocLookup.h`, and
`sim/firmware.js`.

- `include/Config.h` — single source of truth for pins, calibration
  constants, intervals, feature flags. Organised into `bm::config::{pin,
  cal, battery, timing, logger, ui}` namespaces.
- `include/CurrentMath.h` — `adcToCurrent`, `adcToBatteryVolts`, with the
  0.18 A noise-floor clamp.
- `include/SocLookup.h` — voltage → SOC% table (flooded lead-acid by
  default; replace for other chemistries).
- `include/StateOfCharge.h` — coulomb-counter with `maybeReanchor()` that
  snaps to 100 % after 5 min at float voltage + quiescent current.
- `include/EnergyAccumulator.h` — Wh integrator.
- `include/RingBuffer.h` — fixed-capacity float buffer with min/max/mean.
- `include/Measurements.h` — `Sample` struct + `Field` enum + label table.
  `kFieldCount` and the label array in `src/Measurements.cpp` must stay
  in lockstep; the native test `test_field_count_stays_in_lockstep`
  enforces it.

### Hardware-coupled modules (AVR-only)

- `include/Sampler.h` / `src/Sampler.cpp` — drives the voltage relay,
  reads the five ADC channels, applies calibration, updates SOC + energy
  trackers. Relay timing is in `Config::cal::kRelaySettleMs` /
  `kRelayReleaseMs`; if you change these, also update
  `Config::timing::kMeasurementWindowSec` (it's the energy integration
  step).
- `include/Logger.h` / `src/Logger.cpp` — SD-card CSV logger. Filename
  `YYMMDDHH.CSV`; rotates when the active file exceeds
  `Config::logger::kMaxLogBytes` (512 KiB). Header row is written on
  first open. Failures are non-fatal — the firmware degrades gracefully
  and the LCD's Logger screen surfaces the problem.
- `include/Display.h` / `src/Display.cpp` — 7 LCD screens, debounced
  button handling, backlight state machine. Pages cycle on ◀/▶. New
  screens go in the `switch` in `Display::tick()` and increment
  `kPageCount`.
- `include/PowerManager.h` / `src/PowerManager.cpp` — drops the MCU into
  `SLEEP_MODE_PWR_DOWN` after `Config::timing::kIdleSleepTimeoutMs` of no
  button activity. Opens the voltage-sense relay before sleep so the
  divider doesn't drain during downtime.

### Entry point

`src/main.cpp` wires two cooperative `Thread`s into a `ThreadController`:
sampling at 5 s, display at 53 ms. The `sampling` flag is the only
synchronisation between them; the display thread skips a frame mid-sample
so the ADC isn't disturbed. `loop()` is just
`scheduler.run(); power.tick(millis());`.

The whole file is wrapped in `#ifdef TARGET_AVR` so the same source
participates cleanly in native-test builds (which exclude `src/main.cpp`
via `build_src_filter` in the `[env:native]` section of `platformio.ini`).

### Web simulator (`sim/`)

- `sim/firmware.js` — ES-module port of the pure-math layer. Same
  calibration constants and SOC curve as `include/Config.h` /
  `include/SocLookup.h`. Same screen layouts as `src/Display.cpp`.
- `sim/sim.js` — battery + sources model with sliders, button handlers,
  gauges, CSV log stream. Drives `takeSample()` on the firmware's
  sampling cadence (scaled by a time-acceleration slider).
- `sim/index.html` / `sim/style.css` — single-page UI.
- `sim/server.py` — minimal static-file server for local dev.
- `sim/test_firmware.mjs` — Node script that asserts the JS port matches
  the C++ math. 26 cross-checks. Run with `node sim/test_firmware.mjs`.

If you change a calibration constant in `include/Config.h`, mirror it in
`sim/firmware.js`. If you change a screen format in `src/Display.cpp`,
mirror it in `SCREENS[]` in `sim/firmware.js`. `test_firmware.mjs` will
catch screen-width regressions but not value mismatches — those are
covered by the matching `pio test -e native` cases.

## Repository state

- Default branch: `master` — contains the original single-file sketch.
- Active development branch: `claude/init-repo-review-cOZX8` — modular
  refactor + simulator + tests + CI. Push all work here, not to `master`.

## Things worth knowing before editing

- **Charge currents are stored as negative-out / positive-in** at the
  raw sensor level, then sign-flipped in `Sampler::take` so callers see
  charging as a positive contribution. `netA = drawA - chargeA` is the
  net battery drain (positive = discharging).
- **Energy integration constant** `Config::timing::kMeasurementWindowSec`
  bakes in the 5-second sample period plus the 200 ms + 100 ms relay
  delays. Change those and Wh totals will drift.
- **AVR memory** is tight (8 KB SRAM). All log/render code uses fixed
  `char` buffers and `snprintf` — no `String` objects, no heap.
- **Filenames** on the SD card must be 8.3-compliant; the formatter in
  `Logger::formatName` produces `YYMMDDHH.CSV` (or `NORTC.CSV` when the
  RTC is missing).
