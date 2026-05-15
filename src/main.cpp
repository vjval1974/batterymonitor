//
// Battery Monitor — Arduino Mega 2560 firmware entry point.
//
// Wires up the cooperative scheduler (one thread for sampling, one for the
// LCD) and delegates everything else to the dedicated modules. Keep this file
// small — feature logic belongs in the modules.

#ifdef TARGET_AVR

#    include "Config.h"
#    include "Display.h"
#    include "EnergyAccumulator.h"
#    include "Logger.h"
#    include "Measurements.h"
#    include "PowerManager.h"
#    include "Sampler.h"
#    include "StateOfCharge.h"

#    include <Arduino.h>
#    include <RTClib.h>
#    include <Thread.h>
#    include <ThreadController.h>

namespace {

RTC_DS1307 rtc;

bm::Sampler           sampler;
bm::StateOfCharge     soc;
bm::EnergyAccumulator drawEnergy;
bm::EnergyAccumulator solarEnergy;
bm::Logger            logger;
bm::Display           display;
bm::PowerManager      power;

bm::Sample lastSample;

ThreadController scheduler;
Thread           sampleThread;
Thread           displayThread;

volatile bool sampling = false;

bm::DateTimeFields readClock()
{
    if (!rtc.isrunning())
    {
        return bm::DateTimeFields{0, 0, 0, 0, 0, 0};
    }
    const DateTime now = rtc.now();
    return bm::DateTimeFields{static_cast<uint16_t>(now.year()),
                              static_cast<uint8_t>(now.month()),
                              static_cast<uint8_t>(now.day()),
                              static_cast<uint8_t>(now.hour()),
                              static_cast<uint8_t>(now.minute()),
                              static_cast<uint8_t>(now.second())};
}

void runSampleCycle()
{
    sampling = true;
    sampler.take(lastSample, soc, drawEnergy, solarEnergy);
    logger.record(readClock(), lastSample);
    sampling = false;
}

void runDisplayCycle()
{
    if (sampling)
    {
        return; // avoid drawing while ADC is busy
    }
    const uint32_t uptimeSec = millis() / 1000UL;
    display.tick(lastSample, logger, uptimeSec);
}

} // namespace

void setup()
{
    Serial.begin(9600);
    Serial.println(F("Battery Monitor " FW_VERSION));

    sampler.begin();
    power.begin();
    display.begin();

    if (!rtc.begin())
    {
        Serial.println(F("RTC missing"));
    }

    if (!logger.begin(readClock()))
    {
        Serial.println(F("Logger degraded"));
    }

    // Seed SOC from the resting voltage at boot. This is approximate — the
    // bank may not actually be at rest, but it gives the coulomb counter a
    // sane starting point until the first re-anchor at float voltage.
    bm::Sample seed{};
    sampler.take(seed, soc, drawEnergy, solarEnergy);
    soc.seedFromVoltage(seed[bm::Field::BatteryVolts]);
    lastSample = seed;

    sampleThread.onRun(runSampleCycle);
    sampleThread.setInterval(bm::config::timing::kMeasurementPeriodMs);

    displayThread.onRun(runDisplayCycle);
    displayThread.setInterval(bm::config::timing::kDisplayPeriodMs);

    scheduler.add(&sampleThread);
    scheduler.add(&displayThread);
}

void loop()
{
    scheduler.run();
    power.tick(millis());
}

#endif // TARGET_AVR
