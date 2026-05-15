#pragma once
//
// Centralised configuration: pin map, calibration constants, intervals,
// feature flags. This is the single source of truth for anything that might
// need to be re-tuned for a different hardware build.
//
// Pure header — safe to include from native (host) unit tests.

#include <stdint.h>

namespace bm {
namespace config {

// ---------------------------------------------------------------------------
// Firmware identity
// ---------------------------------------------------------------------------
#ifndef FW_VERSION
#    define FW_VERSION "0.0.0-dev"
#endif

// ---------------------------------------------------------------------------
// Pin map (Arduino Mega 2560)
// ---------------------------------------------------------------------------
namespace pin {
constexpr uint8_t kBatteryVoltageAdc      = 0;  // A0  via divider, relay-gated
constexpr uint8_t kDrawCurrentAdc         = 1;  // A1  load shunt sensor (ACS712-like)
constexpr uint8_t kChargeCurrentCarAdc    = 2;  // A2  alternator / DC-DC
constexpr uint8_t kChargeCurrentAcAdc     = 3;  // A3  240 V mains charger
constexpr uint8_t kChargeCurrentSolarAdc  = 9;  // A9  solar array

constexpr uint8_t kWake                   = 2;  // INT0 — wake-from-sleep
constexpr uint8_t kBatteryVoltageRelay    = 8;  // gates the voltage divider

constexpr uint8_t kSdChipSelect           = 10;
constexpr uint8_t kSdMosi                 = 11;
constexpr uint8_t kSdMiso                 = 12;
constexpr uint8_t kSdSck                  = 13;
} // namespace pin

// ---------------------------------------------------------------------------
// Sensing / calibration
// ---------------------------------------------------------------------------
namespace cal {
// ACS712-style hall-effect current sensors, 30 A variant: 66 mV per amp.
constexpr float kAcs712VoltsPerAmp = 0.066f;

// ADC reference is 5 V, sensor outputs are centred at VCC/2. Full-scale ADC
// reading (1023) corresponds to +1650 mV deviation from midpoint.
constexpr float kAdcMidpointMv     = 1650.0f;
constexpr int   kAdcMaxCount       = 1023;

// Voltage at battery terminals corresponding to a 5 V reading at the divider.
// Recalibrate by feeding a known DC source and reading raw ADC.
constexpr float kBatteryDividerVmax = 15.88f;

// Per-channel zero-current offsets. Determined empirically with no current flowing.
constexpr float kSolarOffsetA = 0.05f;
constexpr float kDrawOffsetA  = 0.03f;
constexpr float kCarOffsetA   = 0.24f;
constexpr float kAcOffsetA    = 0.00f;

// Currents below this magnitude (after offset correction) are clamped to zero
// to suppress sensor noise.
constexpr float kCurrentNoiseFloorA = 0.18f;

// Voltage-sense relay timing. The relay is held closed only long enough to
// take a stable reading; this minimises continuous drain through the divider.
constexpr uint16_t kRelaySettleMs   = 200;
constexpr uint16_t kRelayReleaseMs  = 100;
} // namespace cal

// ---------------------------------------------------------------------------
// Battery model
// ---------------------------------------------------------------------------
namespace battery {
// Nominal capacity. Used to scale coulomb-counter SOC. Override per install.
constexpr float kCapacityAh = 100.0f;

// Voltage thresholds (12 V lead-acid / AGM, at rest).
constexpr float kVoltageFull      = 13.5f;   // resting voltage at ~100% SOC
constexpr float kVoltageNominal   = 12.6f;   // ~50%
constexpr float kVoltageLowAlarm  = 11.8f;   // ~20% — visible warning
constexpr float kVoltageCritical  = 11.0f;   // disconnect-load territory
} // namespace battery

// ---------------------------------------------------------------------------
// Scheduling
// ---------------------------------------------------------------------------
namespace timing {
constexpr uint16_t kMeasurementPeriodMs = 5000;
constexpr uint16_t kDisplayPeriodMs     = 53;
constexpr uint16_t kButtonDebounceMs    = 40;

// Total wall-time spent per measurement cycle, including relay settle + release
// delays. Used by the energy integrator. Update if relay timings change.
constexpr float kMeasurementWindowSec =
    (kMeasurementPeriodMs + cal::kRelaySettleMs + cal::kRelayReleaseMs) / 1000.0f;

// Idle time before the device drops into deep sleep.
constexpr uint32_t kIdleSleepTimeoutMs  = 5UL * 60UL * 1000UL; // 5 min
} // namespace timing

// ---------------------------------------------------------------------------
// Logger
// ---------------------------------------------------------------------------
namespace logger {
// Rotate to a new file once the current one passes this size.
constexpr uint32_t kMaxLogBytes = 512UL * 1024UL; // 512 KiB

// CSV column order is fixed; the header is emitted on file open.
} // namespace logger

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------
namespace ui {
// Backlight colours encode battery state. Matches the Adafruit RGB shield enum.
constexpr uint8_t kBacklightCharging    = 0x2; // GREEN
constexpr uint8_t kBacklightIdle        = 0x4; // BLUE
constexpr uint8_t kBacklightDischarging = 0x7; // WHITE
constexpr uint8_t kBacklightWarning     = 0x3; // YELLOW
constexpr uint8_t kBacklightCritical    = 0x1; // RED
constexpr uint8_t kBacklightOff         = 0x0;
} // namespace ui

} // namespace config
} // namespace bm
