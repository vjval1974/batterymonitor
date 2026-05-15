#pragma once
//
// Voltage → state-of-charge lookup for a resting 12 V lead-acid / AGM bank.
// Coulomb counting carries SOC between samples; this lookup is used to:
//   1. Initialise the coulomb counter at boot from a resting reading.
//   2. Re-anchor the counter when the battery sits at a known-full voltage.
//
// Curve is approximate; recalibrate for your chemistry by editing the table.
// Header-only and Arduino-free so host tests can exercise it.

#include <stdint.h>

namespace bm {

struct SocPoint
{
    float volts;
    float socPct;
};

// Coarse OCV curve for a flooded 12 V lead-acid bank (resting, 25 °C).
// Linear interpolation between points is good enough for indicative SOC.
inline constexpr SocPoint kSocCurve[] = {
    {10.50f,   0.0f},
    {11.31f,  10.0f},
    {11.58f,  20.0f},
    {11.75f,  30.0f},
    {11.90f,  40.0f},
    {12.06f,  50.0f},
    {12.20f,  60.0f},
    {12.32f,  70.0f},
    {12.42f,  80.0f},
    {12.50f,  90.0f},
    {12.70f, 100.0f},
};
inline constexpr uint8_t kSocCurveLen = sizeof(kSocCurve) / sizeof(kSocCurve[0]);

// Returns SOC in percent, clamped to [0, 100].
inline float voltageToSoc(float volts)
{
    if (volts <= kSocCurve[0].volts)
    {
        return 0.0f;
    }
    if (volts >= kSocCurve[kSocCurveLen - 1].volts)
    {
        return 100.0f;
    }
    for (uint8_t i = 1; i < kSocCurveLen; ++i)
    {
        if (volts <= kSocCurve[i].volts)
        {
            const SocPoint& lo = kSocCurve[i - 1];
            const SocPoint& hi = kSocCurve[i];
            const float t = (volts - lo.volts) / (hi.volts - lo.volts);
            return lo.socPct + t * (hi.socPct - lo.socPct);
        }
    }
    return 100.0f;
}

} // namespace bm
