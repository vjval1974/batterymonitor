# Hardware reference

## Bill of materials

| Item                              | Qty | Notes                                       |
|-----------------------------------|----:|---------------------------------------------|
| Arduino Mega 2560                 |   1 | ATmega2560, 8 KB SRAM, 256 KB flash         |
| Adafruit RGB LCD Shield + Keypad  |   1 | I²C, MCP23017 backpack, 5 buttons           |
| DS1307 RTC breakout               |   1 | Battery-backed, on the I²C bus              |
| Micro-SD breakout                 |   1 | SPI, CS=D10                                 |
| ACS712 current sensor, 30 A       |   4 | One each: solar, AC charger, car, load      |
| Voltage divider resistors         |   2 | Size so V_batt_max → ≤ 5 V at ADC pin        |
| Single-pole relay (5 V coil)      |   1 | Gates the voltage divider, driven by D8     |
| Diodes, capacitors, terminal blk  | … | Per schematic                                |

## Wiring

```
                       ┌──────────────── Battery + ──────────────┐
                       │                                          │
                       R1 ──┬── relay COM   relay NO ── R2 ── GND │
                            │                                     │
                            └─────── A0 (battery voltage ADC) ────┘
                       ▲
                       │ (Battery + sense; relay gates this branch)

   Solar array  ─── (Hall sensor) ── A9
   AC charger   ─── (Hall sensor) ── A3
   Car/DC-DC    ─── (Hall sensor) ── A2
   Load bus     ─── (Hall sensor) ── A1

   Mega D8  ────── Relay coil + (with flyback diode)
   Mega D2  ────── Wake button (active low to GND, internal pull-up)

   I²C SDA/SCL ── LCD shield + DS1307
   SPI D10..D13 ── SD card (CS, MOSI, MISO, SCK)
```

## Power budget

The Mega + LCD + SD pull ~80 mA active, dominated by the LCD backlight.
With `PowerManager::sleepNow()` engaged, current drops to the AVR's
~0.5 mA `PWR_DOWN` floor plus the LCD backlight (which is forced off
before sleep) and whatever the regulator quiescent draw is on your board.

The voltage-sense relay is only energised during the 300 ms sampling
window every 5 s — duty cycle is ≈ 6 %, so the divider's continuous
contribution to drain is negligible.

## Calibration procedure

### 1. Zero-current offsets

For each ACS712 channel, with the battery system live but that branch
guaranteed at zero amps (e.g. fuse pulled, charger off):

1. Boot the firmware and watch the relevant current on the LCD's
   "Currents" page.
2. The displayed value is the channel's drift. Negate it and set the
   matching constant in `include/Config.h`:
   - `cal::kSolarOffsetA`
   - `cal::kAcOffsetA`
   - `cal::kCarOffsetA`
   - `cal::kDrawOffsetA`

Rebuild and reflash.

### 2. Voltage divider

1. Feed a known DC voltage (e.g. bench supply set to 12.50 V) into the
   battery-sense input.
2. Read the "Summary" page. Compute `kBatteryDividerVmax_new =
   kBatteryDividerVmax_old × (true_voltage / reported_voltage)`.
3. Update `cal::kBatteryDividerVmax` in `include/Config.h`.

### 3. Battery capacity

Set `battery::kCapacityAh` to the nameplate Ah of your bank (or the
20-hour-rate capacity if you have it — the coulomb counter will be more
honest about real-world available energy that way).

### 4. SOC curve

The default `kSocCurve` table in `include/SocLookup.h` is for flooded
lead-acid at 25 °C. Other chemistries:

- **AGM**: similar shape, slightly higher resting voltages. Add ~0.05 V
  to each row.
- **Gel**: closer to AGM than to flooded.
- **LiFePO4**: completely different curve — voltage is almost flat across
  20–80 % SOC. Replace the table wholesale; consider using mostly the
  coulomb counter and only re-anchoring at the knees (charging cutoff
  voltage and ~10 % SOC).

After editing, run `pio test -e native` to confirm the new curve is
monotonic and the interpolation still passes the sanity checks.
