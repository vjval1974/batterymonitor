#include "PowerManager.h"

#include "Config.h"

#ifdef TARGET_AVR
#    include <Arduino.h>
#    include <avr/sleep.h>
#endif

namespace bm {

namespace {
#ifdef TARGET_AVR
void wakeIsr() {}
#endif
} // namespace

void PowerManager::begin()
{
#ifdef TARGET_AVR
    pinMode(config::pin::kWake, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(config::pin::kWake), wakeIsr, LOW);
#endif
    lastActivityMs_ = 0;
}

void PowerManager::noteActivity() { lastActivityMs_ = 0; /* reset, see tick() */ }

void PowerManager::tick(uint32_t nowMs)
{
    if (lastActivityMs_ == 0)
    {
        lastActivityMs_ = nowMs;
    }
    if ((nowMs - lastActivityMs_) >= config::timing::kIdleSleepTimeoutMs)
    {
        sleepNow();
        lastActivityMs_ = nowMs; // restart timer after wake
    }
}

void PowerManager::sleepNow()
{
#ifdef TARGET_AVR
    digitalWrite(config::pin::kBatteryVoltageRelay, HIGH); // open the divider
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    attachInterrupt(digitalPinToInterrupt(config::pin::kWake), wakeIsr, LOW);
    sleep_mode();
    // -- wake here --
    sleep_disable();
    detachInterrupt(digitalPinToInterrupt(config::pin::kWake));
#endif
}

} // namespace bm
