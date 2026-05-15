#include "Sampler.h"

#include "Config.h"
#include "CurrentMath.h"
#include "EnergyAccumulator.h"
#include "StateOfCharge.h"

#ifdef TARGET_AVR
#    include <Arduino.h>
#endif

namespace bm {

void Sampler::begin()
{
#ifdef TARGET_AVR
    pinMode(config::pin::kBatteryVoltageRelay, OUTPUT);
    digitalWrite(config::pin::kBatteryVoltageRelay, LOW);
#endif
}

void Sampler::take(Sample& out, StateOfCharge& soc, EnergyAccumulator& drawEnergy,
                   EnergyAccumulator& solarEnergy)
{
    using namespace config;

    int batteryAdc = 0;
    int solarAdc   = 0;
    int acAdc      = 0;
    int carAdc     = 0;
    int drawAdc    = 0;

#ifdef TARGET_AVR
    // Gate the divider only long enough to take a stable reading.
    digitalWrite(pin::kBatteryVoltageRelay, HIGH);
    delay(cal::kRelaySettleMs);
    batteryAdc = analogRead(pin::kBatteryVoltageAdc);
    delay(cal::kRelayReleaseMs);
    digitalWrite(pin::kBatteryVoltageRelay, LOW);

    solarAdc = analogRead(pin::kChargeCurrentSolarAdc);
    acAdc    = analogRead(pin::kChargeCurrentAcAdc);
    carAdc   = analogRead(pin::kChargeCurrentCarAdc);
    drawAdc  = analogRead(pin::kDrawCurrentAdc);
#endif

    const float volts = adcToBatteryVolts(batteryAdc);
    // Charge sources are wired so that "current into battery" reads negative
    // out of the sensor; flip the sign so callers see charging as a positive
    // contribution.
    const float solarA = -adcToCurrent(solarAdc, cal::kSolarOffsetA);
    const float acA    = -adcToCurrent(acAdc, cal::kAcOffsetA);
    const float carA   = -adcToCurrent(carAdc, cal::kCarOffsetA);
    const float drawA  = adcToCurrent(drawAdc, cal::kDrawOffsetA);
    const float chargeA = solarA + acA + carA;
    // Net current at the battery terminal: + = leaving the battery.
    const float netA = drawA - chargeA;

    const float drawW  = power(volts, drawA);
    const float solarW = power(volts, solarA);

    soc.update(netA, timing::kMeasurementWindowSec);
    soc.maybeReanchor(volts, netA, timing::kMeasurementWindowSec);

    drawEnergy.accumulate(drawW, timing::kMeasurementWindowSec);
    solarEnergy.accumulate(solarW, timing::kMeasurementWindowSec);

    out[Field::BatteryVolts]        = volts;
    out[Field::DrawCurrentA]        = drawA;
    out[Field::SolarCurrentA]       = solarA;
    out[Field::AcCurrentA]          = acA;
    out[Field::CarCurrentA]         = carA;
    out[Field::TotalChargeCurrentA] = chargeA;
    out[Field::NetCurrentA]         = netA;
    out[Field::DrawPowerW]          = drawW;
    out[Field::SolarPowerW]         = solarW;
    out[Field::EnergyDrawnWh]       = drawEnergy.wattHours();
    out[Field::SocPct]              = soc.percent();
}

} // namespace bm
