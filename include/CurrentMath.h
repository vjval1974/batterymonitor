#pragma once
//
// Pure math for the current/voltage conversion path. No Arduino dependency —
// included from both firmware code and host-side unit tests so the conversion
// formulas only live in one place.

#include "Config.h"

#include <math.h>
#include <stdint.h>

namespace bm {

// Linear interpolation helper. Equivalent to Arduino's `map()` but written
// against `float` so we don't truncate sub-amp readings.
constexpr float mapLinear(float x, float inLo, float inHi, float outLo, float outHi)
{
    return (x - inLo) * (outHi - outLo) / (inHi - inLo) + outLo;
}

// Convert a raw 10-bit ADC reading from an ACS712-style sensor into amperes.
// The sensor outputs VCC/2 at zero current; full-scale ADC corresponds to
// ±kAdcMidpointMv around midpoint. `offset` is the per-channel calibration
// (added before noise gating).
//
// Returns 0.0f if the absolute corrected reading is below the noise floor.
inline float adcToCurrent(int adcValue, float offset)
{
    using namespace config::cal;
    const float mv =
        mapLinear(static_cast<float>(adcValue), 0.0f, static_cast<float>(kAdcMaxCount),
                  -kAdcMidpointMv, kAdcMidpointMv);
    const float amps = (mv / kAcs712VoltsPerAmp) / 1000.0f;
    const float corrected = amps + offset;
    if (fabsf(corrected) <= kCurrentNoiseFloorA)
    {
        return 0.0f;
    }
    return corrected;
}

// Convert a raw ADC reading on the battery-voltage divider into volts at the
// battery terminals. The divider is sized so that 5 V at the ADC pin maps to
// `kBatteryDividerVmax` at the battery.
inline float adcToBatteryVolts(int adcValue)
{
    using namespace config::cal;
    return mapLinear(static_cast<float>(adcValue), 0.0f, static_cast<float>(kAdcMaxCount), 0.0f,
                     kBatteryDividerVmax);
}

// Instantaneous DC power. Sign follows current sign — positive = discharging.
inline float power(float volts, float amps) { return volts * amps; }

} // namespace bm
