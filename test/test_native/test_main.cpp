// PlatformIO Unity test entry point for the host-runnable math layer.
// Mirrors test/host_quickcheck.cpp but uses TEST_ASSERT_* so failures surface
// in PlatformIO's test runner output.

#include "CurrentMath.h"
#include "EnergyAccumulator.h"
#include "Measurements.h"
#include "RingBuffer.h"
#include "SocLookup.h"
#include "StateOfCharge.h"

#include <unity.h>

void setUp() {}
void tearDown() {}

void test_adc_to_current_full_scale_positive()
{
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 25.0f, bm::adcToCurrent(1023, 0.0f));
}

void test_adc_to_current_full_scale_negative()
{
    TEST_ASSERT_FLOAT_WITHIN(0.3f, -25.0f, bm::adcToCurrent(0, 0.0f));
}

void test_adc_to_current_noise_floor_clamps()
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, bm::adcToCurrent(512, 0.0f));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, bm::adcToCurrent(514, 0.0f));
}

void test_voltage_conversion_full_scale()
{
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 15.88f, bm::adcToBatteryVolts(1023));
}

void test_soc_lookup_endpoints()
{
    TEST_ASSERT_EQUAL_FLOAT(0.0f, bm::voltageToSoc(9.0f));
    TEST_ASSERT_EQUAL_FLOAT(100.0f, bm::voltageToSoc(13.5f));
}

void test_soc_lookup_interpolation()
{
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 55.0f, bm::voltageToSoc(12.13f));
}

void test_coulomb_counter_discharge()
{
    bm::StateOfCharge soc(100.0f);
    soc.setPercent(80.0f);
    soc.update(10.0f, 3600.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 70.0f, soc.percent());
}

void test_coulomb_counter_clamps()
{
    bm::StateOfCharge soc(50.0f);
    soc.setPercent(10.0f);
    soc.update(1000.0f, 3600.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, soc.percent());
}

void test_soc_reanchor_at_float()
{
    bm::StateOfCharge soc(100.0f);
    soc.setPercent(75.0f);
    for (int i = 0; i < 6; ++i)
    {
        soc.maybeReanchor(13.6f, 0.0f, 60.0f);
    }
    TEST_ASSERT_EQUAL_FLOAT(100.0f, soc.percent());
}

void test_energy_accumulator_integrates()
{
    bm::EnergyAccumulator e;
    e.accumulate(120.0f, 3600.0f);
    e.accumulate(60.0f, 1800.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 150.0f, e.wattHours());
}

void test_ring_buffer_wraps_and_evicts()
{
    bm::RingBuffer<4> rb;
    for (float v : {1.0f, 2.0f, 3.0f, 4.0f, 5.0f})
    {
        rb.push(v);
    }
    TEST_ASSERT_TRUE(rb.full());
    TEST_ASSERT_EQUAL_FLOAT(2.0f, rb.min());
    TEST_ASSERT_EQUAL_FLOAT(5.0f, rb.max());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.5f, rb.mean());
}

void test_field_count_stays_in_lockstep()
{
    TEST_ASSERT_EQUAL_INT(11, static_cast<int>(bm::Field::kCount));
}

int main(int, char**)
{
    UNITY_BEGIN();
    RUN_TEST(test_adc_to_current_full_scale_positive);
    RUN_TEST(test_adc_to_current_full_scale_negative);
    RUN_TEST(test_adc_to_current_noise_floor_clamps);
    RUN_TEST(test_voltage_conversion_full_scale);
    RUN_TEST(test_soc_lookup_endpoints);
    RUN_TEST(test_soc_lookup_interpolation);
    RUN_TEST(test_coulomb_counter_discharge);
    RUN_TEST(test_coulomb_counter_clamps);
    RUN_TEST(test_soc_reanchor_at_float);
    RUN_TEST(test_energy_accumulator_integrates);
    RUN_TEST(test_ring_buffer_wraps_and_evicts);
    RUN_TEST(test_field_count_stays_in_lockstep);
    return UNITY_END();
}
