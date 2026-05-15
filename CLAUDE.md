# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Battery monitor firmware for an Arduino Mega 2560. The device measures battery voltage and the current flowing into/out of a 12V battery bank from three charge sources (solar, AC charger, car alternator) and a draw line, derives power/energy, displays values on a 16x2 RGB LCD shield, and logs to an SD card with timestamps from a DS1307 RTC.

## Build / Flash

Uses PlatformIO (no Make/CMake). The single environment `[env:mega]` is defined in `platformio.ini`.

- Build: `pio run`
- Upload to board: `pio run -t upload`
- Serial monitor: `pio device monitor -b 9600`
- Clean: `pio run -t clean`

`platformio.ini` references a hard-coded host library path (`lib_extra_dirs = /home/brad/Arduino/libraries`). On any other machine that line must be edited or the required libraries (`Thread`/`ThreadController`, `RTClib`, `SD`, `Adafruit_RGBLCDShield`, `Adafruit_MCP23017`) installed via `pio pkg install` instead. There are no tests or lint config.

## Architecture

Single translation unit: `src/BatteryMonitor.ino`. Despite the `.ino` extension, structure it as standard Arduino + PlatformIO.

Runtime is cooperative, not interrupt-driven for the main work:

- `setup()` initializes Serial (9600), the wake interrupt on pin 2, the LCD, SD card, and RTC, then opens a log file named `MMDDHHMM.txt` (or `NoRTC.txt` if the RTC isn't running). If `SD.open` fails it spins forever — this is intentional.
- `loop()` only calls `threadController.run()`. Two `Thread` objects are registered:
  - `measurementThread` → `MeasurementRunner` every 5000 ms: takes measurements then logs them.
  - `lcdDisplayThread` → `displayHandler` every 53 ms: reads buttons and refreshes the LCD.
- `measurementsRunning` is a flag the display thread checks to avoid drawing mid-sample. Treat the two threads as mutually exclusive on shared state via this flag, not as preemptive.

Measurements layout: `measurements[]` is indexed by the `Calculation` enum (`BattVoltage`, `DrawCurrent`, `SolarCurrent`, `AcCurrent`, `CarCurrent`, `TotalChargingCurrent`, `CurrentBalance`, `Power`, `Energy`, `SolarPower`). Charge currents are negated so that `TotalChargingCurrent + DrawCurrent` yields a signed `CurrentBalance`. If you add a measurement, update `NUM_MEASUREMENTS`, the enum, and the parallel `measurementText[]` array together — they're indexed in lockstep.

Sensing details worth knowing before touching the math:
- ACS712-style current sensors: `mvPerAmp = 0.066`, ADC mapped to ±1650 mV around midpoint. Per-channel offsets (`SolarChargeCurrentOffset`, `DrawCurrentOffset`, `CarChargeCurrentOffset`, `AcChargeCurrentOffset`) are calibration constants. `CalculateCurrent` zeroes out |I| ≤ 0.18 A as noise.
- Battery voltage is read through a divider gated by a relay on `BatteryVoltageSenseRelayPin` (pin 8) to avoid continuous current through the sense resistor. The sequence in `TakeMeasurements` — relay HIGH, delay 200 ms, read, delay 100 ms, relay LOW — must stay intact; full-scale at 5 V corresponds to `vMaxAt5v = 15.88` V.
- `Energy` uses a hard-coded `MeasurementPeriodInSeconds = 5.3` (thread interval + sampling delays). If you change `measurementThread->setInterval(...)` or the relay delays, update this constant or energy totals will drift.

Sleep path (`sleepNow`/`wakeUpNow`, INT0 on pin 2) is wired up but not invoked from `loop()` — `SleepModeLoop` exists but is unused. The wake ISR is intentionally empty.

Logging: `Log()` writes one line per call to `dataFile` and `flush()`es every time. The TODO about rotating files when size exceeds ~1 MB is unimplemented.

## Repository State

- Default branch: `master` — contains the only substantive commit, `e0523d4 Initial checkin of project in platformIO`.
- Active working branch: `claude/init-repo-review-cOZX8` (this is where Claude Code on the web develops; push here, not to `master`).
- No README, no tests, no CI. The repo is a single-file Arduino sketch wrapped in PlatformIO scaffolding.
