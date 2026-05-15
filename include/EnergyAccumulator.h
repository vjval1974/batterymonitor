#pragma once
//
// Integrates instantaneous power into accumulated energy (Wh) since boot.
// Separate trackers for draw, solar, AC, and car so the user can see "how much
// did each source contribute today" without post-processing the SD log.
//
// Header-only, Arduino-free.

#include <stdint.h>

namespace bm {

class EnergyAccumulator
{
public:
    EnergyAccumulator() : wattHours_(0.0f) {}

    // Add a sample observed over `dtSec` seconds at `watts`.
    void accumulate(float watts, float dtSec) { wattHours_ += (watts * dtSec) / 3600.0f; }

    float wattHours() const { return wattHours_; }
    void  reset() { wattHours_ = 0.0f; }

private:
    float wattHours_;
};

} // namespace bm
