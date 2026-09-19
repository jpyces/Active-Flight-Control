// test/test_vertical_kf/test_vertical_kf.cpp
//
// Native Unity test for gnc::VerticalKF, ported (deterministically -- no RNG,
// so results are exactly reproducible) from the cases VerticalKFTest.m
// covers: run with `pio test -e native`.

#include <unity.h>
#include <cmath>

#include "VerticalKF.h"

using namespace gnc;

namespace
{
    constexpr float kTol = 1e-3f;

    void zeroMatrix(float M[3][3])
    {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                M[i][j] = 0.0f;
    }
}

void setUp() {}
void tearDown() {}

// predict() alone, with no correct() calls, should drift -- not hold steady.
// A nonzero initial velocity with zero accel input should carry altitude
// forward exactly (no noise involved, so this is exact, not approximate):
// altitude_N = altitude_0 + velocity_0 * dt * N.
void test_predict_only_drifts_without_correction()
{
    float Q[3][3];
    zeroMatrix(Q);
    float P0[3][3];
    zeroMatrix(P0);
    float x0[3] = {0.0f, 1.0f, 0.0f}; // altitude=0, velocity=1 m/s, bias=0

    VerticalKF kf(Q, /*R=*/0.01f, x0, P0);

    float dt = 0.1f;
    int steps = 20;
    for (int i = 0; i < steps; ++i)
    {
        kf.predict(/*aMeas=*/0.0f, dt);
    }

    TEST_ASSERT_FLOAT_WITHIN(kTol, 1.0f * dt * steps, kf.altitude());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 1.0f, kf.velocity());
}

// Repeated predict+correct against a constant, noiseless true altitude
// should converge the altitude estimate to that true value, starting from a
// wrong initial guess.
void test_converges_to_known_altitude()
{
    float Q[3][3] = {
        {1e-4f, 0.0f, 0.0f},
        {0.0f, 1e-4f, 0.0f},
        {0.0f, 0.0f, 1e-6f},
    };
    float P0[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    };
    float x0[3] = {0.0f, 0.0f, 0.0f};
    float trueAltitude = 5.0f;

    VerticalKF kf(Q, /*R=*/0.05f, x0, P0);

    float dt = 0.01f;
    for (int i = 0; i < 500; ++i)
    {
        kf.predict(/*aMeas=*/0.0f, dt); // true accel is 0 -- stationary at trueAltitude
        kf.correct(trueAltitude);
    }

    TEST_ASSERT_FLOAT_WITHIN(0.05f, trueAltitude, kf.altitude());
}

// A constant, unmodeled accelerometer bias (sensor reads a fixed nonzero
// value even though the true vertical acceleration is 0) should be absorbed
// into the bias state over time, rather than permanently corrupting the
// altitude/velocity estimate -- that's the entire reason accelBias is a
// state here rather than assumed away.
void test_bias_state_converges_to_known_bias()
{
    float trueBias = 0.05f; // m/s^2 -- deliberately larger than the real
                            // literature-informed Qbias placeholder so this
                            // unit test converges in a tractable number of
                            // iterations; not meant to reflect real hardware
                            // tuning (see gnc-findings.md's Q/R section).
    float Q[3][3] = {
        {1e-5f, 0.0f, 0.0f},
        {0.0f, 1e-5f, 0.0f},
        {0.0f, 0.0f, 1e-4f}, // large enough Qbias to let the bias state move
    };
    float P0[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    };
    float x0[3] = {0.0f, 0.0f, 0.0f};
    float trueAltitude = 0.0f; // true vehicle is stationary throughout

    VerticalKF kf(Q, /*R=*/0.01f, x0, P0);

    float dt = 0.01f;
    for (int i = 0; i < 2000; ++i)
    {
        kf.predict(/*aMeas=*/trueBias, dt); // sensor always reads the bias
        kf.correct(trueAltitude);           // but truth never moves
    }

    TEST_ASSERT_FLOAT_WITHIN(0.01f, trueBias, kf.accelBias());
    // And the altitude estimate should have converged back near truth too,
    // not stayed corrupted by the unmodeled bias.
    TEST_ASSERT_FLOAT_WITHIN(0.05f, trueAltitude, kf.altitude());
}

// Covariance must stay well-formed (symmetric, non-negative diagonal) over a
// long mixed run of predicts and corrects -- the Joseph-form update exists
// specifically to guarantee this under floating-point roundoff.
void test_covariance_stays_symmetric_and_psd()
{
    float Q[3][3] = {
        {1e-4f, 0.0f, 0.0f},
        {0.0f, 1e-4f, 0.0f},
        {0.0f, 0.0f, 1e-6f},
    };
    float P0[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    };
    float x0[3] = {0.0f, 0.0f, 0.0f};

    VerticalKF kf(Q, /*R=*/0.05f, x0, P0);

    float dt = 0.01f;
    for (int i = 0; i < 1000; ++i)
    {
        // Deterministic "noisy-ish" accel/altitude readings via a simple
        // oscillation rather than RNG, so the test is exactly reproducible.
        float aMeas = 0.1f * std::sin(0.1f * i);
        kf.predict(aMeas, dt);
        if (i % 5 == 0)
        {
            float zMeas = 1.0f + 0.02f * std::cos(0.3f * i);
            kf.correct(zMeas);
        }
    }

    float P[3][3];
    kf.covariance(P);

    for (int i = 0; i < 3; ++i)
    {
        TEST_ASSERT_TRUE(P[i][i] >= 0.0f);
        for (int j = 0; j < 3; ++j)
        {
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, P[i][j], P[j][i]);
        }
    }
}

// The filter shouldn't chase individual noisy barometer samples: alternating
// the measurement above/below the true altitude by a fixed amount (a
// deterministic stand-in for measurement noise) should leave the converged
// altitude estimate's swing much smaller than the injected measurement swing.
void test_rejects_individual_measurement_noise()
{
    float Q[3][3] = {
        {1e-5f, 0.0f, 0.0f},
        {0.0f, 1e-5f, 0.0f},
        {0.0f, 0.0f, 1e-6f},
    };
    float P0[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    };
    float x0[3] = {2.0f, 0.0f, 0.0f};
    float trueAltitude = 2.0f;
    float noiseAmplitude = 0.3f;

    VerticalKF kf(Q, /*R=*/0.05f, x0, P0);

    float dt = 0.01f;
    float minAltitude = 1e9f;
    float maxAltitude = -1e9f;
    for (int i = 0; i < 300; ++i)
    {
        kf.predict(/*aMeas=*/0.0f, dt);
        float noisySign = (i % 2 == 0) ? 1.0f : -1.0f;
        kf.correct(trueAltitude + noisySign * noiseAmplitude);

        if (i > 50) // let initial transient settle before measuring swing
        {
            minAltitude = std::fmin(minAltitude, kf.altitude());
            maxAltitude = std::fmax(maxAltitude, kf.altitude());
        }
    }

    float estimateSwing = maxAltitude - minAltitude;
    // The raw measurement swings by 2*noiseAmplitude every other sample; the
    // filtered estimate should swing far less than that.
    TEST_ASSERT_TRUE(estimateSwing < noiseAmplitude);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_predict_only_drifts_without_correction);
    RUN_TEST(test_converges_to_known_altitude);
    RUN_TEST(test_bias_state_converges_to_known_bias);
    RUN_TEST(test_covariance_stays_symmetric_and_psd);
    RUN_TEST(test_rejects_individual_measurement_noise);
    return UNITY_END();
}

