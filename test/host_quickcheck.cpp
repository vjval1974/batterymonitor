// Minimal host-side quickcheck. Runs the pure-math layer through enough cases
// to catch regressions without depending on the PlatformIO test framework.
// Build:  g++ -std=gnu++17 -O2 -Wall -Wextra -Iinclude test/host_quickcheck.cpp -o quickcheck
// Run:    ./quickcheck   (exits non-zero on failure)

#include "CurrentMath.h"
#include "EnergyAccumulator.h"
#include "Measurements.h"
#include "RingBuffer.h"
#include "SocLookup.h"
#include "StateOfCharge.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {
int failures = 0;

bool nearly(float a, float b, float eps = 0.01f)
{
    return std::fabs(a - b) <= eps;
}

#define CHECK(cond)                                                                                  \
    do                                                                                               \
    {                                                                                                \
        if (!(cond))                                                                                 \
        {                                                                                            \
            std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);                     \
            ++failures;                                                                              \
        }                                                                                            \
    } while (0)

void test_adc_to_current_zero_at_midpoint()
{
    // Midpoint ADC reading (512) → ~0 A before offset. With zero offset that
    // falls inside the noise floor so we expect exactly 0.
    CHECK(bm::adcToCurrent(512, 0.0f) == 0.0f);
}

void test_adc_to_current_positive()
{
    // ADC 1023 → +1650 mV → +25 A (above noise floor).
    const float i = bm::adcToCurrent(1023, 0.0f);
    CHECK(nearly(i, 25.0f, 0.3f));
}

void test_adc_to_current_negative()
{
    const float i = bm::adcToCurrent(0, 0.0f);
    CHECK(nearly(i, -25.0f, 0.3f));
}

void test_adc_to_current_noise_floor()
{
    // ADC 525 ≈ +42 mV → ~0.64 A — above the 0.18 A noise floor.
    const float aboveFloor = bm::adcToCurrent(525, 0.0f);
    CHECK(aboveFloor != 0.0f);
    // Just a few counts off midpoint — below the noise floor — clamped to 0.
    CHECK(bm::adcToCurrent(514, 0.0f) == 0.0f);
}

void test_voltage_conversion()
{
    // 1023 → kBatteryDividerVmax (15.88).
    CHECK(nearly(bm::adcToBatteryVolts(1023), 15.88f, 0.02f));
    CHECK(nearly(bm::adcToBatteryVolts(0), 0.0f, 0.001f));
    CHECK(nearly(bm::adcToBatteryVolts(512), 7.94f, 0.05f));
}

void test_soc_lookup_extremes()
{
    CHECK(bm::voltageToSoc(10.0f) == 0.0f);
    CHECK(bm::voltageToSoc(15.0f) == 100.0f);
}

void test_soc_lookup_interpolation()
{
    // Halfway between 12.06 (50%) and 12.20 (60%) ≈ 12.13 → ~55%.
    CHECK(nearly(bm::voltageToSoc(12.13f), 55.0f, 1.0f));
}

void test_soc_lookup_monotonic()
{
    float prev = -1.0f;
    for (float v = 10.0f; v <= 13.0f; v += 0.05f)
    {
        const float s = bm::voltageToSoc(v);
        CHECK(s >= prev - 0.001f);
        prev = s;
    }
}

void test_state_of_charge_coulomb_counter()
{
    bm::StateOfCharge soc(100.0f); // 100 Ah bank
    soc.setPercent(80.0f);
    // Discharge 10 A for 1 hour → 10 Ah → 10% drop on a 100 Ah bank.
    soc.update(10.0f, 3600.0f);
    CHECK(nearly(soc.percent(), 70.0f, 0.05f));
    // Charge 5 A for 1 hour → 5 Ah → 5% rise.
    soc.update(-5.0f, 3600.0f);
    CHECK(nearly(soc.percent(), 75.0f, 0.05f));
}

void test_state_of_charge_clamps()
{
    bm::StateOfCharge soc(50.0f);
    soc.setPercent(10.0f);
    soc.update(100.0f, 3600.0f); // would underflow
    CHECK(soc.percent() == 0.0f);
    soc.setPercent(90.0f);
    soc.update(-100.0f, 3600.0f); // would overflow
    CHECK(soc.percent() == 100.0f);
}

void test_state_of_charge_reanchor()
{
    bm::StateOfCharge soc(100.0f);
    soc.setPercent(85.0f);
    // 6 minutes of float voltage + near-zero current → snap to 100%.
    for (int i = 0; i < 6; ++i)
    {
        soc.maybeReanchor(13.6f, 0.0f, 60.0f);
    }
    CHECK(soc.percent() == 100.0f);
}

void test_energy_accumulator()
{
    bm::EnergyAccumulator e;
    e.accumulate(120.0f, 3600.0f); // 120 W for 1 h → 120 Wh
    CHECK(nearly(e.wattHours(), 120.0f, 0.001f));
    e.accumulate(60.0f, 1800.0f); // 60 W for 30 min → 30 Wh
    CHECK(nearly(e.wattHours(), 150.0f, 0.001f));
}

void test_ring_buffer_min_max()
{
    bm::RingBuffer<4> rb;
    rb.push(1.0f);
    rb.push(2.0f);
    rb.push(3.0f);
    CHECK(rb.size() == 3);
    CHECK(rb.min() == 1.0f);
    CHECK(rb.max() == 3.0f);
    CHECK(nearly(rb.mean(), 2.0f, 0.001f));

    // Push beyond capacity — oldest sample evicted.
    rb.push(4.0f);
    rb.push(5.0f);
    CHECK(rb.full());
    CHECK(rb.min() == 2.0f);
    CHECK(rb.max() == 5.0f);
}

void test_ring_buffer_empty()
{
    bm::RingBuffer<8> rb;
    CHECK(rb.size() == 0);
    CHECK(rb.mean() == 0.0f);
    CHECK(rb.min() == 0.0f);
    CHECK(rb.max() == 0.0f);
}

void test_field_count_matches_enum()
{
    // If somebody adds a Field without bumping the labels array this would
    // walk off the end at runtime. The Measurements.cpp array is sized
    // explicitly to kFieldCount; mirror that here.
    CHECK(static_cast<int>(bm::Field::kCount) == 11);
}

} // namespace

int main()
{
    test_adc_to_current_zero_at_midpoint();
    test_adc_to_current_positive();
    test_adc_to_current_negative();
    test_adc_to_current_noise_floor();
    test_voltage_conversion();
    test_soc_lookup_extremes();
    test_soc_lookup_interpolation();
    test_soc_lookup_monotonic();
    test_state_of_charge_coulomb_counter();
    test_state_of_charge_clamps();
    test_state_of_charge_reanchor();
    test_energy_accumulator();
    test_ring_buffer_min_max();
    test_ring_buffer_empty();
    test_field_count_matches_enum();

    if (failures == 0)
    {
        std::printf("OK — all checks passed\n");
        return 0;
    }
    std::fprintf(stderr, "%d failure(s)\n", failures);
    return 1;
}
