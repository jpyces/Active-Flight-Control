// Unity tests for gnc::SensorHealth (pio test -e native).
// Covers the debounce state machine, the freshness flag, the staleness timeout,
// and two regressions from the per-driver debounce code this class replaced.

#include <unity.h>
#include <cstdint>
#include "SensorHealth.h"

using namespace gnc;

namespace
{
    constexpr std::uint32_t kPeriodUs = 10000; // 100 Hz reads
    constexpr std::uint32_t kStaleDegradeUs = 50000;
    constexpr std::uint32_t kStaleFailUs = 200000;

    SensorHealthConfig makeConfig(std::uint32_t staleDegradeUs = kStaleDegradeUs,
                                  std::uint32_t staleFailUs = kStaleFailUs)
    {
        SensorHealthConfig c{};
        c.degradeAfterFailures = 2;
        c.failAfterFailures = 4;
        c.recoverAfterSuccesses = 3;
        c.fullRecoverAfterSuccesses = 5;
        c.staleDegradeUs = staleDegradeUs;
        c.staleFailUs = staleFailUs;
        return c;
    }

    // Feeds n reads of the same kind, advancing time by one period each.
    void feed(SensorHealth &h, std::uint32_t &nowUs, int n, bool readOk, bool newData)
    {
        for (int i = 0; i < n; ++i)
        {
            nowUs += kPeriodUs;
            h.record(readOk, newData, nowUs);
        }
    }

    // Forces NOMINAL -> FAILED with read failures.
    void failSensor(SensorHealth &h, std::uint32_t &nowUs)
    {
        feed(h, nowUs, 4, false, false);
        TEST_ASSERT_EQUAL(SensorStatus::FAILED, h.status());
    }
}

void setUp() {}
void tearDown() {}

// --- Lifecycle ---

void test_starts_uninitialized_and_ignores_reads_before_begin()
{
    SensorHealth h(makeConfig());
    TEST_ASSERT_EQUAL(SensorStatus::UNINITIALIZED, h.status());

    std::uint32_t now = 0;
    feed(h, now, 10, true, true);
    TEST_ASSERT_EQUAL(SensorStatus::UNINITIALIZED, h.status());
    TEST_ASSERT_FALSE(h.isFresh());
}

void test_begin_sets_nominal_or_failed()
{
    SensorHealth ok(makeConfig());
    ok.begin(true, 0);
    TEST_ASSERT_EQUAL(SensorStatus::NOMINAL, ok.status());
    TEST_ASSERT_FALSE(ok.isFresh()); // nothing has been read yet

    SensorHealth bad(makeConfig());
    bad.begin(false, 0);
    TEST_ASSERT_EQUAL(SensorStatus::FAILED, bad.status());
}

// --- Freshness ---

void test_fresh_only_when_read_ok_and_new()
{
    SensorHealth h(makeConfig());
    h.begin(true, 0);
    std::uint32_t now = 0;

    feed(h, now, 1, true, true);
    TEST_ASSERT_TRUE(h.isFresh());
    TEST_ASSERT_EQUAL_UINT32(now, h.lastSampleUs());

    const std::uint32_t sampleTime = now;
    feed(h, now, 1, true, false); // healthy, nothing new
    TEST_ASSERT_FALSE(h.isFresh());
    TEST_ASSERT_EQUAL_UINT32(sampleTime, h.lastSampleUs());

    feed(h, now, 1, false, true); // failed read never counts as new data
    TEST_ASSERT_FALSE(h.isFresh());
    TEST_ASSERT_EQUAL_UINT32(sampleTime, h.lastSampleUs());
}

void test_slow_healthy_sensor_stays_nominal()
{
    // A sensor delivering a new sample every 4th read (e.g. 100 Hz baro in a
    // 400 Hz loop) is healthy; not-fresh reads must not count against it.
    SensorHealth h(makeConfig());
    h.begin(true, 0);
    std::uint32_t now = 0;

    for (int i = 0; i < 400; ++i)
    {
        feed(h, now, 1, true, (i % 4) == 0);
        TEST_ASSERT_EQUAL(SensorStatus::NOMINAL, h.status());
    }
}

// --- Debounce ---

void test_failures_degrade_then_fail()
{
    SensorHealth h(makeConfig());
    h.begin(true, 0);
    std::uint32_t now = 0;

    feed(h, now, 1, false, false);
    TEST_ASSERT_EQUAL(SensorStatus::NOMINAL, h.status());
    feed(h, now, 1, false, false);
    TEST_ASSERT_EQUAL(SensorStatus::DEGRADED, h.status());
    feed(h, now, 1, false, false);
    TEST_ASSERT_EQUAL(SensorStatus::DEGRADED, h.status());
    feed(h, now, 1, false, false);
    TEST_ASSERT_EQUAL(SensorStatus::FAILED, h.status());
}

void test_success_resets_failure_run()
{
    SensorHealth h(makeConfig());
    h.begin(true, 0);
    std::uint32_t now = 0;

    for (int i = 0; i < 20; ++i)
    {
        feed(h, now, 1, false, false); // isolated glitch
        feed(h, now, 1, true, true);
    }
    TEST_ASSERT_EQUAL(SensorStatus::NOMINAL, h.status());
}

void test_empty_read_breaks_failure_run()
{
    SensorHealth h(makeConfig(0, 0)); // staleness off: isolate the debounce
    h.begin(true, 0);
    std::uint32_t now = 0;

    // The bus answering (even with nothing new) means failures are not consecutive.
    for (int i = 0; i < 20; ++i)
    {
        feed(h, now, 1, false, false);
        feed(h, now, 1, true, false);
    }
    TEST_ASSERT_EQUAL(SensorStatus::NOMINAL, h.status());
}

void test_recovery_goes_failed_degraded_nominal()
{
    SensorHealth h(makeConfig());
    h.begin(true, 0);
    std::uint32_t now = 0;
    failSensor(h, now);

    feed(h, now, 2, true, true);
    TEST_ASSERT_EQUAL(SensorStatus::FAILED, h.status());
    feed(h, now, 1, true, true); // 3rd success
    TEST_ASSERT_EQUAL(SensorStatus::DEGRADED, h.status());

    // Full recovery counts from the FAILED -> DEGRADED step, not from FAILED.
    feed(h, now, 4, true, true);
    TEST_ASSERT_EQUAL(SensorStatus::DEGRADED, h.status());
    feed(h, now, 1, true, true); // 5th success since DEGRADED
    TEST_ASSERT_EQUAL(SensorStatus::NOMINAL, h.status());
}

void test_healthy_but_stale_reads_do_not_advance_recovery()
{
    SensorHealth h(makeConfig(0, 0)); // staleness off: isolate the debounce
    h.begin(true, 0);
    std::uint32_t now = 0;
    failSensor(h, now);

    feed(h, now, 50, true, false); // bus answers, but no new data
    TEST_ASSERT_EQUAL(SensorStatus::FAILED, h.status());
}

// Regression: the old Imu/Altimeter checkHealth() set DEGRADED whenever the
// success count sat between RECOVERY and FULL_RECOVERY, even from NOMINAL. One
// glitch followed by clean reads briefly demoted a healthy sensor.
void test_single_glitch_never_demotes_nominal_during_recovery_run()
{
    SensorHealth h(makeConfig());
    h.begin(true, 0);
    std::uint32_t now = 0;

    feed(h, now, 1, false, false); // below the degrade threshold
    for (int i = 0; i < 20; ++i)
    {
        feed(h, now, 1, true, true);
        TEST_ASSERT_EQUAL(SensorStatus::NOMINAL, h.status());
    }
}

// Regression: the old counters were uint8_t with an unbounded ++, so after 255
// clean reads the success count wrapped to 0 and re-entered the demotion window.
void test_long_nominal_run_does_not_wrap()
{
    SensorHealth h(makeConfig());
    h.begin(true, 0);
    std::uint32_t now = 0;

    for (int i = 0; i < 1000; ++i)
    {
        feed(h, now, 1, true, true);
        TEST_ASSERT_EQUAL(SensorStatus::NOMINAL, h.status());
    }
}

void test_degrade_only_demotes_nominal()
{
    SensorHealth h(makeConfig());
    h.begin(true, 0);
    h.degrade();
    TEST_ASSERT_EQUAL(SensorStatus::DEGRADED, h.status());

    std::uint32_t now = 0;
    feed(h, now, 5, true, true);
    TEST_ASSERT_EQUAL(SensorStatus::NOMINAL, h.status());

    SensorHealth failed(makeConfig());
    failed.begin(false, 0);
    failed.degrade(); // must not promote FAILED
    TEST_ASSERT_EQUAL(SensorStatus::FAILED, failed.status());
}

// --- idle() ---

void test_idle_clears_fresh_without_counting()
{
    SensorHealth h(makeConfig(0, 0));
    h.begin(true, 0);
    std::uint32_t now = 0;

    feed(h, now, 1, true, true);
    TEST_ASSERT_TRUE(h.isFresh());
    now += kPeriodUs;
    h.idle(now);
    TEST_ASSERT_FALSE(h.isFresh());
    TEST_ASSERT_EQUAL(SensorStatus::NOMINAL, h.status());
}

// Unlike a healthy-but-empty read, an idle tick must not break a failure run:
// a sensor whose every read fails, with idle ticks between reads, still fails.
void test_idle_does_not_break_failure_run()
{
    SensorHealth h(makeConfig(0, 0)); // staleness off: isolate the debounce
    h.begin(true, 0);
    std::uint32_t now = 0;

    for (int i = 0; i < 4; ++i)
    {
        feed(h, now, 1, false, false);
        now += kPeriodUs;
        h.idle(now);
        now += kPeriodUs;
        h.idle(now);
    }
    TEST_ASSERT_EQUAL(SensorStatus::FAILED, h.status());
}

void test_idle_does_not_advance_recovery()
{
    SensorHealth h(makeConfig(0, 0));
    h.begin(true, 0);
    std::uint32_t now = 0;
    failSensor(h, now);

    // Two successes, idle ticks, one more success: 3 total -> DEGRADED, and the
    // idle ticks neither reset nor added to the count.
    feed(h, now, 2, true, true);
    for (int i = 0; i < 10; ++i)
    {
        now += kPeriodUs;
        h.idle(now);
    }
    TEST_ASSERT_EQUAL(SensorStatus::FAILED, h.status());
    feed(h, now, 1, true, true);
    TEST_ASSERT_EQUAL(SensorStatus::DEGRADED, h.status());
}

void test_idle_ticks_still_age_the_sample()
{
    SensorHealth h(makeConfig());
    h.begin(true, 0);
    std::uint32_t now = 0;
    feed(h, now, 1, true, true);

    for (int i = 0; i < 5; ++i) // 50 ms with no new sample
    {
        now += kPeriodUs;
        h.idle(now);
    }
    TEST_ASSERT_EQUAL(SensorStatus::DEGRADED, h.status());
}

void test_idle_before_begin_is_ignored()
{
    SensorHealth h(makeConfig());
    h.idle(1000000);
    TEST_ASSERT_EQUAL(SensorStatus::UNINITIALIZED, h.status());
}

// --- Staleness ---

void test_stale_sensor_degrades_then_fails()
{
    SensorHealth h(makeConfig());
    h.begin(true, 0);
    std::uint32_t now = 0;
    feed(h, now, 1, true, true); // last new sample at 10 ms

    feed(h, now, 4, true, false); // 50 ms: age 40 ms
    TEST_ASSERT_EQUAL(SensorStatus::NOMINAL, h.status());
    feed(h, now, 1, true, false); // 60 ms: age 50 ms
    TEST_ASSERT_EQUAL(SensorStatus::DEGRADED, h.status());

    feed(h, now, 14, true, false); // 200 ms: age 190 ms
    TEST_ASSERT_EQUAL(SensorStatus::DEGRADED, h.status());
    feed(h, now, 1, true, false); // 210 ms: age 200 ms
    TEST_ASSERT_EQUAL(SensorStatus::FAILED, h.status());
}

void test_stale_recovery_needs_full_debounce()
{
    SensorHealth h(makeConfig());
    h.begin(true, 0);
    std::uint32_t now = 0;
    feed(h, now, 100, true, true); // long clean run before going stale
    feed(h, now, 5, true, false);  // age 50 ms -> DEGRADED
    TEST_ASSERT_EQUAL(SensorStatus::DEGRADED, h.status());

    // The pre-stale success run must not carry over.
    feed(h, now, 1, true, true);
    TEST_ASSERT_EQUAL(SensorStatus::DEGRADED, h.status());
    feed(h, now, 4, true, true);
    TEST_ASSERT_EQUAL(SensorStatus::NOMINAL, h.status());
}

void test_failed_reads_also_age_the_sample()
{
    SensorHealth h(makeConfig(kStaleDegradeUs, 0)); // fail-by-staleness off
    h.begin(true, 0);
    std::uint32_t now = 0;

    // One failure, one healthy-but-empty read, alternating: never enough
    // consecutive failures to degrade, but no new data for 50 ms either.
    for (int i = 0; i < 3; ++i)
    {
        feed(h, now, 1, false, false);
        feed(h, now, 1, true, false);
    }
    TEST_ASSERT_EQUAL(SensorStatus::DEGRADED, h.status());
}

void test_staleness_disabled_when_zero()
{
    SensorHealth h(makeConfig(0, 0));
    h.begin(true, 0);
    std::uint32_t now = 0;

    feed(h, now, 1000, true, false); // 10 s with nothing new
    TEST_ASSERT_EQUAL(SensorStatus::NOMINAL, h.status());
}

void test_stale_clock_starts_at_begin()
{
    SensorHealth h(makeConfig());
    h.begin(true, 1000000); // begin() late, e.g. after a slow boot
    h.record(true, false, 1000000 + kStaleDegradeUs - 1);
    TEST_ASSERT_EQUAL(SensorStatus::NOMINAL, h.status());
    h.record(true, false, 1000000 + kStaleDegradeUs);
    TEST_ASSERT_EQUAL(SensorStatus::DEGRADED, h.status());
}

void test_age_survives_micros_wraparound()
{
    SensorHealth h(makeConfig());
    const std::uint32_t nearWrap = UINT32_MAX - 5000; // 5 ms before micros() wraps
    h.begin(true, nearWrap);
    h.record(true, true, nearWrap);

    h.record(true, false, 20000); // ~25 ms later, after the wrap
    TEST_ASSERT_EQUAL(SensorStatus::NOMINAL, h.status());
    h.record(true, false, 50000); // ~55 ms later
    TEST_ASSERT_EQUAL(SensorStatus::DEGRADED, h.status());
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_starts_uninitialized_and_ignores_reads_before_begin);
    RUN_TEST(test_begin_sets_nominal_or_failed);
    RUN_TEST(test_fresh_only_when_read_ok_and_new);
    RUN_TEST(test_slow_healthy_sensor_stays_nominal);
    RUN_TEST(test_failures_degrade_then_fail);
    RUN_TEST(test_success_resets_failure_run);
    RUN_TEST(test_empty_read_breaks_failure_run);
    RUN_TEST(test_recovery_goes_failed_degraded_nominal);
    RUN_TEST(test_healthy_but_stale_reads_do_not_advance_recovery);
    RUN_TEST(test_single_glitch_never_demotes_nominal_during_recovery_run);
    RUN_TEST(test_long_nominal_run_does_not_wrap);
    RUN_TEST(test_degrade_only_demotes_nominal);
    RUN_TEST(test_idle_clears_fresh_without_counting);
    RUN_TEST(test_idle_does_not_break_failure_run);
    RUN_TEST(test_idle_does_not_advance_recovery);
    RUN_TEST(test_idle_ticks_still_age_the_sample);
    RUN_TEST(test_idle_before_begin_is_ignored);
    RUN_TEST(test_stale_sensor_degrades_then_fails);
    RUN_TEST(test_stale_recovery_needs_full_debounce);
    RUN_TEST(test_failed_reads_also_age_the_sample);
    RUN_TEST(test_staleness_disabled_when_zero);
    RUN_TEST(test_stale_clock_starts_at_begin);
    RUN_TEST(test_age_survives_micros_wraparound);
    return UNITY_END();
}
