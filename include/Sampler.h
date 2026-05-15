#pragma once
//
// Hardware-coupled sensor sampler. Drives the voltage-sense relay, reads the
// ADC channels, applies calibration, and writes the result into a Sample.
//
// Only the production build needs the Arduino-specific implementation; for the
// web simulator we provide a parallel JavaScript port. Host tests exercise the
// pure-math layer (CurrentMath, SocLookup, StateOfCharge) directly.

#include "Measurements.h"

namespace bm {

class StateOfCharge;
class EnergyAccumulator;

class Sampler
{
public:
    // `soc` and `energy` are updated alongside the raw measurements so each
    // sampling cycle leaves the device with a complete, self-consistent view.
    void begin();
    void take(Sample& out, StateOfCharge& soc, EnergyAccumulator& drawEnergy,
              EnergyAccumulator& solarEnergy);
};

} // namespace bm
