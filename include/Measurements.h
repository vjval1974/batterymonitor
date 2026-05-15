#pragma once
//
// Snapshot of one sampling cycle — what the firmware just read from the analog
// front-end and the values derived from it.
//
// `kCount` and the enum stay in lockstep with the human-readable labels in
// Measurements.cpp. If you add a field, update both.

#include <stdint.h>

namespace bm {

enum class Field : uint8_t
{
    BatteryVolts = 0,
    DrawCurrentA,
    SolarCurrentA,
    AcCurrentA,
    CarCurrentA,
    TotalChargeCurrentA,
    NetCurrentA,
    DrawPowerW,
    SolarPowerW,
    EnergyDrawnWh,
    SocPct,
    kCount
};

constexpr uint8_t kFieldCount = static_cast<uint8_t>(Field::kCount);

struct Sample
{
    float values[kFieldCount];

    float& operator[](Field f) { return values[static_cast<uint8_t>(f)]; }
    float  operator[](Field f) const { return values[static_cast<uint8_t>(f)]; }
};

// Short, fixed-length labels suited to a 16-wide LCD. Defined in
// Measurements.cpp so they live in PROGMEM on AVR.
const char* labelFor(Field f);
const char* unitFor(Field f);

} // namespace bm
