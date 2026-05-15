#pragma once
//
// Coulomb-counter state-of-charge estimator with voltage-table re-anchoring.
//
//   * `seedFromVoltage()` initialises the counter from a resting OCV reading
//     using the lookup table in SocLookup.h.
//   * `update()` integrates the *net* current (positive = discharging) over the
//     elapsed window and decrements / increments the counter accordingly.
//   * `maybeReanchor()` snaps the counter back to 100% when the battery has
//     spent enough time at a known-full voltage with negligible current — this
//     suppresses long-term drift from sensor offset error.
//
// Arduino-free, header-only.

#include "Config.h"
#include "SocLookup.h"

#include <math.h>
#include <stdint.h>

namespace bm {

class StateOfCharge
{
public:
    explicit StateOfCharge(float capacityAh = config::battery::kCapacityAh)
        : capacityAh_(capacityAh), socPct_(50.0f), reanchorAccumSec_(0.0f)
    {
    }

    // Initialise SOC% from a resting voltage reading.
    void seedFromVoltage(float restingVolts) { socPct_ = voltageToSoc(restingVolts); }

    // Integrate over `dtSec` seconds.  `netCurrentA` is positive when the
    // battery is discharging, negative when it's being charged.
    void update(float netCurrentA, float dtSec)
    {
        const float deltaAh = (netCurrentA * dtSec) / 3600.0f;
        const float deltaPct = (deltaAh / capacityAh_) * 100.0f;
        socPct_ -= deltaPct;
        clamp();
    }

    // Snap to 100% if the battery has held a full-charge voltage with
    // negligible draw for long enough (default: 5 min of integration window).
    void maybeReanchor(float volts, float netCurrentA, float dtSec)
    {
        const bool atFloat = volts >= config::battery::kVoltageFull;
        const bool quiescent = fabsf(netCurrentA) < 0.5f;
        if (atFloat && quiescent)
        {
            reanchorAccumSec_ += dtSec;
            if (reanchorAccumSec_ >= 300.0f)
            {
                socPct_ = 100.0f;
                reanchorAccumSec_ = 0.0f;
            }
        }
        else
        {
            reanchorAccumSec_ = 0.0f;
        }
    }

    float percent() const { return socPct_; }
    float capacityAh() const { return capacityAh_; }

    // For tests / EEPROM persistence.
    void setPercent(float p)
    {
        socPct_ = p;
        clamp();
    }

private:
    void clamp()
    {
        if (socPct_ < 0.0f)
        {
            socPct_ = 0.0f;
        }
        if (socPct_ > 100.0f)
        {
            socPct_ = 100.0f;
        }
    }

    float capacityAh_;
    float socPct_;
    float reanchorAccumSec_;
};

} // namespace bm
