#pragma once
//
// Idle-timeout deep-sleep manager.
//
// The MCU drops into SLEEP_MODE_PWR_DOWN after `kIdleSleepTimeoutMs` of no
// button presses. The wake interrupt is INT0 on pin 2 (kWake). The voltage
// sense relay is forced open before sleep so the divider isn't draining the
// bank while we're down.

#include <stdint.h>

namespace bm {

class PowerManager
{
public:
    void begin();
    void noteActivity();             // call when a button is pressed / something changes
    void tick(uint32_t nowMs);       // call from loop, decides whether to sleep
    void sleepNow();                 // force a sleep cycle

private:
    uint32_t lastActivityMs_ = 0;
};

} // namespace bm
