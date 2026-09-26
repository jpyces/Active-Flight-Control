// Unity tests for gnc::StateEstimator (pio test -e native).
// Ported from StateEstimatorTest.m, plus C++-port-specific cases (g -> m/s^2
// accel scaling, sensor gating, yawObservable).

#include <unity.h>
#include <cmath>
#include <limits>
#include "eigen.h"
#include "ComplimentaryFilter.h"
#include "StateEstimator.h"

namespace gnc
{
    // Test-only seam declared as a friend in StateEstimator.h. Lets tests force
    // FallbackComplementary without engineering a real Madgwick divergence.
    struct StateEstimatorTestAccess
    {
        static void forceFallback(StateEstimator &s) { s.m_source = AhrsSource::FallbackComplementary; }
    };
}

using namespace gnc;

namespace
{
    constexpr float kDt = 0.01f;
    constexpr float kSeedingDuration = 0.5f;
    constexpr float kBeta = 0.1f;
    constexpr float kAlpha = 0.98f;
    constexpr float kG = 9.81f;
    constexpr int kSeedSteps = 50; // round(kSeedingDuration / kDt)

    StateEstimatorConfig makeConfig(const Eigen::Vector3f &x0 = Eigen::Vector3f::Zero())
    {
        StateEstimatorConfig c{};
        c.madgwickBeta = kBeta;
        c.compAlpha = kAlpha;
        c.gravity = kG;
        c.seedingDuration = kSeedingDuration;
        c.kfQ = Eigen::Matrix3f::Identity() * 1e-4f;
        c.kfR = 0.0121f;
        c.kfX0 = x0;
        c.kfP0 = Eigen::Matrix3f::Identity();
        return c;
    }

    // At rest, level. Accel is in g's per SensorReadings.h. Every sensor
    // delivers a new sample each step unless a test says otherwise.
    SensorReadings stillReadings()
    {
        SensorReadings r{};
        r.accel = {0.0f, 0.0f, 1.0f};
        r.accelStatus = SensorStatus::NOMINAL;
        r.accelFresh = true;
        r.gyro = {0.0f, 0.0f, 0.0f};
        r.gyroStatus = SensorStatus::NOMINAL;
        r.gyroFresh = true;
        r.mag = {1.0f, 0.0f, 0.0f};
        r.magStatus = SensorStatus::NOMINAL;
        r.magFresh = true;
        r.baroAltitude = 0.0f;
        r.baroStatus = SensorStatus::NOMINAL;
        r.baroFresh = true;
        return r;
    }

    void assertKfMatches(const VerticalKF &ref, const VerticalKF &actual)
    {
        for (int i = 0; i < 3; ++i)
        {
            TEST_ASSERT_FLOAT_WITHIN(1e-6f, ref.state()(i), actual.state()(i));
            for (int j = 0; j < 3; ++j)
            {
                TEST_ASSERT_FLOAT_WITHIN(1e-6f, ref.covariance()(i, j), actual.covariance()(i, j));
            }
        }
    }

    void runSteps(StateEstimator &est, const SensorReadings &r, int n)
    {
        for (int k = 0; k < n; ++k)
        {
            est.update(r, kDt);
        }
    }

    void assertQuatEqual(const Quaternion &expected, const Quaternion &actual)
    {
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, expected.W(), actual.W());
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, expected.X(), actual.X());
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, expected.Y(), actual.Y());
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, expected.Z(), actual.Z());
    }
}

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------- AHRS source

void test_starts_in_seeding()
{
    StateEstimator est(makeConfig());
    TEST_ASSERT_TRUE(est.source() == AhrsSource::Seeding);
    TEST_ASSERT_FALSE(est.isConverged());
    TEST_ASSERT_FALSE(est.isDegraded());
}

void test_seeding_output_tracks_complementary_filter()
{
    StateEstimator est(makeConfig());
    const SensorReadings r = stillReadings();
    Quaternion ref = Quaternion::identity();
    for (int k = 0; k < 10; ++k)
    {
        est.update(r, kDt);
        ref = complementaryFilter(ref, r.gyro, r.accel, kDt, kAlpha);
        assertQuatEqual(ref, est.attitude());
    }
    TEST_ASSERT_TRUE(est.source() == AhrsSource::Seeding);
}

// Also guards the float-accumulation fix: 0.01f summed 50 times must flip on
// step 50, not 51.
void test_transitions_to_nominal_at_seeding_duration()
{
    StateEstimator est(makeConfig());
    const SensorReadings r = stillReadings();
    for (int k = 1; k < kSeedSteps; ++k)
    {
        est.update(r, kDt);
        TEST_ASSERT_TRUE_MESSAGE(est.source() == AhrsSource::Seeding, "Flipped to Nominal early");
    }
    est.update(r, kDt);
    TEST_ASSERT_TRUE(est.source() == AhrsSource::Nominal);
    TEST_ASSERT_TRUE(est.isConverged());
}

void test_seed_handoff_is_continuous()
{
    StateEstimator est(makeConfig());
    const SensorReadings r = stillReadings();
    Quaternion ref = Quaternion::identity();
    for (int k = 0; k < kSeedSteps; ++k)
    {
        ref = complementaryFilter(ref, r.gyro, r.accel, kDt, kAlpha);
        est.update(r, kDt);
    }
    assertQuatEqual(ref, est.attitude());
}

void test_no_silent_recovery_from_fallback()
{
    StateEstimator est(makeConfig());
    StateEstimatorTestAccess::forceFallback(est);
    const SensorReadings r = stillReadings();
    for (int k = 0; k < 20; ++k)
    {
        est.update(r, kDt);
        TEST_ASSERT_TRUE_MESSAGE(est.source() == AhrsSource::FallbackComplementary,
                                 "Left FallbackComplementary without reset()");
    }
    TEST_ASSERT_TRUE(est.isDegraded());
}

// ---------------------------------------------------------------- reset()

void test_reset_from_nominal_stays_nominal()
{
    StateEstimator est(makeConfig());
    runSteps(est, stillReadings(), kSeedSteps);
    TEST_ASSERT_TRUE(est.source() == AhrsSource::Nominal);
    est.reset();
    TEST_ASSERT_TRUE(est.source() == AhrsSource::Nominal);
}

void test_reset_from_fallback_returns_to_nominal()
{
    StateEstimator est(makeConfig());
    StateEstimatorTestAccess::forceFallback(est);
    est.update(stillReadings(), kDt);
    TEST_ASSERT_TRUE(est.source() == AhrsSource::FallbackComplementary);

    est.reset();
    TEST_ASSERT_TRUE(est.source() == AhrsSource::Nominal);
    TEST_ASSERT_TRUE(est.isConverged());
}

void test_reset_from_seeding_skips_remaining_seed()
{
    StateEstimator est(makeConfig());
    runSteps(est, stillReadings(), 3);
    est.reset();
    TEST_ASSERT_TRUE(est.source() == AhrsSource::Nominal);
}

void test_reset_does_not_touch_vertical_kf()
{
    StateEstimator est(makeConfig(Eigen::Vector3f(0.0f, 0.5f, 0.0f)));
    SensorReadings r = stillReadings();
    r.baroStatus = SensorStatus::FAILED;
    runSteps(est, r, 20);
    const Eigen::Vector3f before = est.verticalKF().state();
    est.reset();
    const Eigen::Vector3f after = est.verticalKF().state();
    for (int i = 0; i < 3; ++i)
    {
        TEST_ASSERT_EQUAL_FLOAT(before(i), after(i));
    }
}

// ---------------------------------------------------------------- detectInstability

void test_detect_instability_flags_nan()
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    TEST_ASSERT_TRUE(StateEstimator::detectInstability(Quaternion(nan, 0, 0, 0), 1.0f));
    TEST_ASSERT_TRUE(StateEstimator::detectInstability(Quaternion::identity(), nan));
}

void test_detect_instability_flags_inf()
{
    const float inf = std::numeric_limits<float>::infinity();
    TEST_ASSERT_TRUE(StateEstimator::detectInstability(Quaternion(1, inf, 0, 0), 1.0f));
    TEST_ASSERT_TRUE(StateEstimator::detectInstability(Quaternion::identity(), inf));
}

// The case normalize() used to hide: the raw step collapsed, and the returned
// quaternion is a perfectly healthy-looking identity.
void test_detect_instability_flags_collapsed_step()
{
    TEST_ASSERT_TRUE(StateEstimator::detectInstability(Quaternion::identity(), 0.0f));
    TEST_ASSERT_TRUE(StateEstimator::detectInstability(Quaternion::identity(), 1e-8f));
}

void test_detect_instability_accepts_normal_step()
{
    TEST_ASSERT_FALSE(StateEstimator::detectInstability(Quaternion::identity(), 1.0f));
    // A big-but-finite step (fast rotation) is not instability.
    TEST_ASSERT_FALSE(StateEstimator::detectInstability(Quaternion::identity(), 1.02f));
}

// ---------------------------------------------------------------- mag / yaw

void test_yaw_observable_tracks_mag_status_each_step()
{
    StateEstimator est(makeConfig());
    runSteps(est, stillReadings(), kSeedSteps);
    TEST_ASSERT_FALSE(est.yawObservable()); // flip step: Madgwick hasn't fused mag yet
    est.update(stillReadings(), kDt);
    TEST_ASSERT_TRUE(est.yawObservable());

    SensorReadings noMag = stillReadings();
    noMag.magStatus = SensorStatus::DEGRADED; // DEGRADED mag is NOT used
    est.update(noMag, kDt);
    TEST_ASSERT_FALSE(est.yawObservable());
    TEST_ASSERT_TRUE(est.isConverged()); // mag loss is not a convergence loss

    est.update(stillReadings(), kDt);
    TEST_ASSERT_TRUE(est.yawObservable());
}

// NOMINAL status but an all-zero mag reading: Madgwick skips it, so yaw is not
// observable even though magStatus says the sensor is fine.
void test_nominal_but_zero_mag_is_not_yaw_observable()
{
    StateEstimator est(makeConfig());
    runSteps(est, stillReadings(), kSeedSteps + 1);
    SensorReadings r = stillReadings();
    r.mag = {0.0f, 0.0f, 0.0f};
    est.update(r, kDt);
    TEST_ASSERT_FALSE(est.yawObservable());
}

void test_yaw_not_observable_outside_nominal()
{
    StateEstimator est(makeConfig());
    est.update(stillReadings(), kDt);
    TEST_ASSERT_FALSE(est.yawObservable()); // Seeding

    StateEstimatorTestAccess::forceFallback(est);
    est.update(stillReadings(), kDt);
    TEST_ASSERT_FALSE(est.yawObservable()); // FallbackComplementary
}

// ---------------------------------------------------------------- vertical KF

void test_vertical_kf_predicts_every_step_regardless_of_baro()
{
    StateEstimator est(makeConfig(Eigen::Vector3f(0.0f, 0.5f, 0.0f)));
    SensorReadings r = stillReadings();
    r.baroStatus = SensorStatus::FAILED;

    est.update(r, kDt);
    const float first = est.verticalKF().altitude();
    runSteps(est, r, 19);
    const float last = est.verticalKF().altitude();
    TEST_ASSERT_TRUE(last > first); // positive initial velocity accumulates
}

void test_vertical_kf_corrects_only_when_baro_nominal()
{
    const StateEstimatorConfig c = makeConfig();
    StateEstimator est(c);
    VerticalKF ref(c.kfQ, c.kfR, c.kfX0, c.kfP0);

    SensorReadings noCorrect = stillReadings();
    noCorrect.baroStatus = SensorStatus::DEGRADED;
    noCorrect.baroAltitude = 3.0f; // would visibly move the estimate if fused
    for (int k = 0; k < 5; ++k)
    {
        est.update(noCorrect, kDt);
        ref.predict(0.0f, kDt); // at rest, true vertical accel is 0
    }
    for (int i = 0; i < 3; ++i)
    {
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, ref.state()(i), est.verticalKF().state()(i));
    }

    const SensorReadings correct = stillReadings();
    est.update(correct, kDt);
    ref.predict(0.0f, kDt);
    ref.correct(correct.baroAltitude);
    for (int i = 0; i < 3; ++i)
    {
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, ref.state()(i), est.verticalKF().state()(i));
    }
}

// SensorReadings.accel is in g. If update() forgot to scale by gravity, a level
// 1 g reading would look like -8.81 m/s^2 and velocity would run away.
void test_accel_in_g_is_scaled_before_vertical_channel()
{
    StateEstimator est(makeConfig());
    SensorReadings r = stillReadings();
    r.baroStatus = SensorStatus::FAILED;
    runSteps(est, r, 200);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, est.verticalKF().velocity());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, est.verticalKF().altitude());
}

// ---------------------------------------------------------------- sensor gating

// Lost accel must not read as free fall: vertical channel coasts at constant velocity.
void test_unusable_accel_coasts_vertical_channel()
{
    StateEstimator est(makeConfig(Eigen::Vector3f(0.0f, 0.5f, 0.0f)));
    runSteps(est, stillReadings(), kSeedSteps);

    SensorReadings r = stillReadings();
    r.accelStatus = SensorStatus::FAILED;
    r.accel = {0.0f, 0.0f, 0.0f};
    r.baroStatus = SensorStatus::FAILED;
    const float v0 = est.verticalKF().velocity();
    runSteps(est, r, 100);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, v0, est.verticalKF().velocity());
}

// Lost gyro: a garbage rate reading must not be integrated into attitude.
void test_unusable_gyro_is_not_integrated()
{
    StateEstimator est(makeConfig());
    runSteps(est, stillReadings(), kSeedSteps);

    SensorReadings r = stillReadings();
    r.gyroStatus = SensorStatus::FAILED;
    r.gyro = {5.0f, -3.0f, 2.0f};
    runSteps(est, r, 50);

    const StateEstimate out = est.update(r, kDt);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, out.angles.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, out.angles.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, out.angles.z);
}

// A NaN reading with a NOMINAL status must not poison either filter -- if it
// reached the complementary filter, fallback and reset() would both inherit NaN.
void test_non_finite_reading_is_gated_out()
{
    StateEstimator est(makeConfig());
    runSteps(est, stillReadings(), kSeedSteps);

    SensorReadings r = stillReadings();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    r.gyro = {nan, 0.0f, 0.0f};
    r.accel = {0.0f, nan, 1.0f};
    r.mag = {nan, nan, nan};
    runSteps(est, r, 10);

    TEST_ASSERT_TRUE(est.source() == AhrsSource::Nominal);
    const Quaternion &q = est.attitude();
    TEST_ASSERT_TRUE(std::isfinite(q.W()) && std::isfinite(q.X()) && std::isfinite(q.Y()) && std::isfinite(q.Z()));
    TEST_ASSERT_TRUE(std::isfinite(est.verticalKF().altitude()));

    est.reset(); // resync source must also be clean
    const Quaternion &q2 = est.attitude();
    TEST_ASSERT_TRUE(std::isfinite(q2.W()));
}

// DEGRADED gyro is still the only rate source -- it IS integrated.
void test_degraded_gyro_is_still_used()
{
    StateEstimator est(makeConfig());
    runSteps(est, stillReadings(), kSeedSteps);

    SensorReadings r = stillReadings();
    r.gyroStatus = SensorStatus::DEGRADED;
    r.gyro = {0.0f, 0.0f, 1.0f}; // yaw rate: unobservable to accel, so gyro must drive it
    r.magStatus = SensorStatus::FAILED;
    runSteps(est, r, 10);
    TEST_ASSERT_TRUE(std::fabs(est.update(r, kDt).angles.z) > 0.05f);
}

// ---------------------------------------------------------------- output contract

void test_output_rates_are_raw_gyro_passthrough()
{
    StateEstimator est(makeConfig());
    SensorReadings r = stillReadings();
    r.gyro = {0.1f, -0.2f, 0.3f};
    const StateEstimate out = est.update(r, kDt);
    TEST_ASSERT_EQUAL_FLOAT(0.1f, out.rates.x);
    TEST_ASSERT_EQUAL_FLOAT(-0.2f, out.rates.y);
    TEST_ASSERT_EQUAL_FLOAT(0.3f, out.rates.z);
    TEST_ASSERT_FALSE(out.position.has_value()); // Milestone 2 fields stay empty
}

void test_output_altitude_matches_kf()
{
    StateEstimator est(makeConfig(Eigen::Vector3f(1.0f, 0.5f, 0.0f)));
    SensorReadings r = stillReadings();
    r.baroStatus = SensorStatus::FAILED;
    const StateEstimate out = est.update(r, kDt);
    TEST_ASSERT_EQUAL_FLOAT(est.verticalKF().altitude(), out.altitude);
    TEST_ASSERT_EQUAL_FLOAT(est.verticalKF().velocity(), out.verticalVelocity);
}

void test_vertical_kf_skips_stale_baro()
{
    const StateEstimatorConfig c = makeConfig();
    StateEstimator est(c);
    VerticalKF ref(c.kfQ, c.kfR, c.kfX0, c.kfP0);

    // Healthy baro, but no new sample: must not be fused.
    SensorReadings stale = stillReadings();
    stale.baroFresh = false;
    stale.baroAltitude = 3.0f; // would visibly move the estimate if fused
    for (int k = 0; k < 5; ++k)
    {
        est.update(stale, kDt);
        ref.predict(0.0f, kDt);
    }
    assertKfMatches(ref, est.verticalKF());
}

// A baro slower than the loop: the same reading is held for several steps and
// only marked fresh when it changes. The KF must correct once per new sample,
// not once per step (which would shrink P as if every repeat were independent).
void test_held_baro_is_corrected_once_per_new_sample()
{
    const StateEstimatorConfig c = makeConfig();
    StateEstimator est(c);
    VerticalKF ref(c.kfQ, c.kfR, c.kfX0, c.kfP0);

    constexpr int kLoopStepsPerBaroSample = 5; // e.g. 100 Hz baro in a 500 Hz loop
    SensorReadings r = stillReadings();
    for (int k = 0; k < 4 * kLoopStepsPerBaroSample; ++k)
    {
        r.baroFresh = (k % kLoopStepsPerBaroSample) == 0;
        if (r.baroFresh)
        {
            r.baroAltitude = 0.5f * static_cast<float>(k / kLoopStepsPerBaroSample + 1);
        }

        est.update(r, kDt);
        ref.predict(0.0f, kDt);
        if (r.baroFresh)
        {
            ref.correct(r.baroAltitude);
        }
    }
    assertKfMatches(ref, est.verticalKF());
}

void test_readings_default_to_not_fresh()
{
    const SensorReadings r{};
    TEST_ASSERT_FALSE(r.accelFresh);
    TEST_ASSERT_FALSE(r.gyroFresh);
    TEST_ASSERT_FALSE(r.magFresh);
    TEST_ASSERT_FALSE(r.baroFresh);
    TEST_ASSERT_FALSE(r.gnssFresh);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_starts_in_seeding);
    RUN_TEST(test_seeding_output_tracks_complementary_filter);
    RUN_TEST(test_transitions_to_nominal_at_seeding_duration);
    RUN_TEST(test_seed_handoff_is_continuous);
    RUN_TEST(test_no_silent_recovery_from_fallback);
    RUN_TEST(test_reset_from_nominal_stays_nominal);
    RUN_TEST(test_reset_from_fallback_returns_to_nominal);
    RUN_TEST(test_reset_from_seeding_skips_remaining_seed);
    RUN_TEST(test_reset_does_not_touch_vertical_kf);
    RUN_TEST(test_detect_instability_flags_nan);
    RUN_TEST(test_detect_instability_flags_inf);
    RUN_TEST(test_detect_instability_flags_collapsed_step);
    RUN_TEST(test_detect_instability_accepts_normal_step);
    RUN_TEST(test_nominal_but_zero_mag_is_not_yaw_observable);
    RUN_TEST(test_yaw_observable_tracks_mag_status_each_step);
    RUN_TEST(test_yaw_not_observable_outside_nominal);
    RUN_TEST(test_vertical_kf_predicts_every_step_regardless_of_baro);
    RUN_TEST(test_vertical_kf_corrects_only_when_baro_nominal);
    RUN_TEST(test_vertical_kf_skips_stale_baro);
    RUN_TEST(test_held_baro_is_corrected_once_per_new_sample);
    RUN_TEST(test_readings_default_to_not_fresh);
    RUN_TEST(test_accel_in_g_is_scaled_before_vertical_channel);
    RUN_TEST(test_unusable_accel_coasts_vertical_channel);
    RUN_TEST(test_unusable_gyro_is_not_integrated);
    RUN_TEST(test_non_finite_reading_is_gated_out);
    RUN_TEST(test_degraded_gyro_is_still_used);
    RUN_TEST(test_output_rates_are_raw_gyro_passthrough);
    RUN_TEST(test_output_altitude_matches_kf);
    return UNITY_END();
}
