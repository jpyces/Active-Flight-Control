#include <unity.h>
#include <cmath>
#include "TouchdownDetector.h"

using namespace gnc;

namespace
{
    constexpr float kDt = 0.01f;

    TouchdownConfig makeConfig()
    {
        TouchdownConfig c{};
        c.altitudeBandM = 0.3f;
        c.maxVerticalSpeedMps = 0.2f;
        c.maxThrottleFraction = 0.25f; // hover assumed ~0.4
        c.holdTimeS = 0.5f;            // ~50 ticks
        return c;
    }

    // Feeds the same sample n times; returns the last landed().
    bool feed(TouchdownDetector &td, float alt, float vz, float thr, int n)
    {
        bool landed = false;
        for (int k = 0; k < n; ++k)
        {
            landed = td.update(alt, vz, thr, kDt);
        }
        return landed;
    }

    // Takes the detector from its boot "landed" state to airborne.
    void liftOff(TouchdownDetector &td)
    {
        feed(td, 1.0f, 0.0f, 0.4f, 1);
        TEST_ASSERT_FALSE(td.landed());
    }
}

void setUp() {}
void tearDown() {}

void test_starts_landed()
{
    TouchdownDetector td(makeConfig());
    TEST_ASSERT_TRUE(td.landed());
}

void test_clears_immediately_on_takeoff_throttle()
{
    // Still on the ground and still, but spooling up: throttle alone clears it.
    TouchdownDetector td(makeConfig());
    TEST_ASSERT_FALSE(td.update(0.0f, 0.0f, 0.5f, kDt));
}

void test_sets_only_after_hold_time()
{
    TouchdownDetector td(makeConfig());
    liftOff(td);
    TEST_ASSERT_FALSE(feed(td, 0.05f, 0.0f, 0.1f, 45));
    TEST_ASSERT_TRUE(feed(td, 0.05f, 0.0f, 0.1f, 10));
}

void test_low_hover_is_not_a_touchdown()
{
    // The classic false positive: hovering 20 cm up, perfectly still, but at
    // hover throttle. Must never read as landed.
    TouchdownDetector td(makeConfig());
    liftOff(td);
    TEST_ASSERT_FALSE(feed(td, 0.2f, 0.0f, 0.4f, 1000));
}

void test_still_descending_is_not_a_touchdown()
{
    // Low throttle, near the ground, but still moving down (failsafe descent).
    TouchdownDetector td(makeConfig());
    liftOff(td);
    TEST_ASSERT_FALSE(feed(td, 0.1f, -0.5f, 0.1f, 1000));
}

void test_above_band_is_not_a_touchdown()
{
    // Low throttle and zero vertical speed at 2 m (e.g. apex of a bounce).
    TouchdownDetector td(makeConfig());
    liftOff(td);
    TEST_ASSERT_FALSE(feed(td, 2.0f, 0.0f, 0.1f, 1000));
}

void test_interruption_restarts_the_hold()
{
    TouchdownDetector td(makeConfig());
    liftOff(td);
    feed(td, 0.05f, 0.0f, 0.1f, 40);
    feed(td, 0.05f, -0.5f, 0.1f, 1); // bounce
    TEST_ASSERT_FALSE(feed(td, 0.05f, 0.0f, 0.1f, 40));
    TEST_ASSERT_TRUE(feed(td, 0.05f, 0.0f, 0.1f, 15));
}

void test_band_is_relative_to_ground_reference()
{
    // Baro drift since boot: ground now reads 5 m. Captured at arming.
    TouchdownDetector td(makeConfig());
    td.setGroundReference(5.0f);
    liftOff(td);
    TEST_ASSERT_TRUE(feed(td, 5.1f, 0.0f, 0.1f, 60));

    TouchdownDetector td2(makeConfig());
    liftOff(td2);
    TEST_ASSERT_FALSE(feed(td2, 5.1f, 0.0f, 0.1f, 60)); // without the reference
}

void test_nan_input_reads_as_not_landed()
{
    TouchdownDetector td(makeConfig());
    TEST_ASSERT_FALSE(feed(td, NAN, 0.0f, 0.1f, 100));
    TEST_ASSERT_FALSE(feed(td, 0.0f, NAN, 0.1f, 100));
    TEST_ASSERT_FALSE(feed(td, 0.0f, 0.0f, NAN, 100));
}

void test_stays_landed_while_parked()
{
    TouchdownDetector td(makeConfig());
    liftOff(td);
    feed(td, 0.0f, 0.0f, 0.0f, 60);
    TEST_ASSERT_TRUE(feed(td, 0.0f, 0.0f, 0.0f, 100000)); // long idle, no drift
}

// Exactly one rising edge per landing: what the FSM's touchdown logic needs.
void test_one_rising_edge_per_landing()
{
    TouchdownDetector td(makeConfig());
    int edges = 0;
    bool prev = td.landed();
    auto step = [&](float alt, float vz, float thr, int n) {
        for (int k = 0; k < n; ++k)
        {
            const bool now = td.update(alt, vz, thr, kDt);
            edges += (now && !prev) ? 1 : 0;
            prev = now;
        }
    };
    step(0.0f, 0.0f, 0.5f, 20);  // spool-up
    step(0.5f, 0.5f, 0.45f, 50); // climb
    step(1.0f, 0.0f, 0.4f, 300); // hover
    step(0.5f, -0.3f, 0.3f, 50); // descend
    step(0.0f, 0.0f, 0.1f, 200); // on the ground, throttle wound down
    TEST_ASSERT_EQUAL_INT(1, edges);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_starts_landed);
    RUN_TEST(test_clears_immediately_on_takeoff_throttle);
    RUN_TEST(test_sets_only_after_hold_time);
    RUN_TEST(test_low_hover_is_not_a_touchdown);
    RUN_TEST(test_still_descending_is_not_a_touchdown);
    RUN_TEST(test_above_band_is_not_a_touchdown);
    RUN_TEST(test_interruption_restarts_the_hold);
    RUN_TEST(test_band_is_relative_to_ground_reference);
    RUN_TEST(test_nan_input_reads_as_not_landed);
    RUN_TEST(test_stays_landed_while_parked);
    RUN_TEST(test_one_rising_edge_per_landing);
    return UNITY_END();
}
