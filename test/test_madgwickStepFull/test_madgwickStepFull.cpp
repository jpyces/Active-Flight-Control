#include <unity.h>
#include <cmath>
#include <algorithm>
#include <limits>

#include "Madgwick.h"
#include "Quaternion.h"
#include "Vector3.h"

using namespace gnc;

namespace
{
    constexpr float kTol = 1e-4f;
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kDegToRad = kPi / 180.0f;

    float quatNorm(const Quaternion &q)
    {
        return std::sqrt(q.W() * q.W() + q.X() * q.X() + q.Y() * q.Y() + q.Z() * q.Z());
    }

    void assertQuaternionsClose(const Quaternion &expected, const Quaternion &actual, float tol)
    {
        TEST_ASSERT_FLOAT_WITHIN(tol, expected.W(), actual.W());
        TEST_ASSERT_FLOAT_WITHIN(tol, expected.X(), actual.X());
        TEST_ASSERT_FLOAT_WITHIN(tol, expected.Y(), actual.Y());
        TEST_ASSERT_FLOAT_WITHIN(tol, expected.Z(), actual.Z());
    }

    // Angle (rad) between two quaternions' represented rotations, via
    // |dot product| -- robust to the +q/-q sign ambiguity (both
    // represent the same rotation), same technique the MATLAB reference
    // tests use.
    float quaternionAngleDifference(const Quaternion &a, const Quaternion &b)
    {
        float dot = a.W() * b.W() + a.X() * b.X() + a.Y() * b.Y() + a.Z() * b.Z();
        dot = std::min(1.0f, std::fabs(dot));
        return 2.0f * std::acos(dot);
    }

    // Replicates ONLY the gravity/accel block's gradient-descent step --
    // i.e. what madgwickStepFull computes when mag is invalid, and
    // exactly what a separate madgwickNoMag would have computed. Used to
    // adapt madgwickFullEdgeCasesTest.m's Case 2 (zero-norm mag), which
    // originally cross-checked against that now-removed function.
    Quaternion gravityOnlyStep(const Quaternion &q, const Vector3 &gyro, Vector3 accel, float dt, float beta)
    {
        Quaternion qdotGyro = 0.5f * q * Quaternion(0.0f, gyro.x, gyro.y, gyro.z);

        float accelNorm = norm(accel);
        if (accelNorm <= std::numeric_limits<float>::epsilon())
        {
            return (q + qdotGyro * dt).normalize();
        }
        accel = accel / accelNorm;

        float fg[3] = {
            2.0f * (q.X() * q.Z() - q.W() * q.Y()) - accel.x,
            2.0f * (q.W() * q.X() + q.Y() * q.Z()) - accel.y,
            2.0f * (0.5f - q.X() * q.X() - q.Y() * q.Y()) - accel.z};

        // clang-format off
        float Jg[3][4] = {
            {-2.0f * q.Y(),  2.0f * q.Z(), -2.0f * q.W(),  2.0f * q.X()},
            { 2.0f * q.X(),  2.0f * q.W(),  2.0f * q.Z(),  2.0f * q.Y()},
            { 0.0f,         -4.0f * q.X(), -4.0f * q.Y(),  0.0f}};
        // clang-format on

        float gradient[4];
        for (int k = 0; k < 4; ++k)
        {
            gradient[k] = Jg[0][k] * fg[0] + Jg[1][k] * fg[1] + Jg[2][k] * fg[2];
        }

        float gradientNorm = std::sqrt(gradient[0] * gradient[0] + gradient[1] * gradient[1] +
                                       gradient[2] * gradient[2] + gradient[3] * gradient[3]);

        Quaternion gradientHat(0.0f, 0.0f, 0.0f, 0.0f);
        if (gradientNorm > std::numeric_limits<float>::epsilon())
        {
            gradientHat = Quaternion(gradient[0] / gradientNorm, gradient[1] / gradientNorm,
                                     gradient[2] / gradientNorm, gradient[3] / gradientNorm);
        }

        Quaternion qdotCorrected = qdotGyro - beta * gradientHat;
        return (q + qdotCorrected * dt).normalize();
    }
}

void setUp() {}
void tearDown() {}

// --- Ported from madgwickFullTest.m ---
// Static convergence: nonzero roll/pitch/yaw (none trivial), synthetic
// accel/mag built by solving the filter's OWN residual formulas in
// reverse at the ground-truth quaternion -- guarantees the test's truth
// uses the same convention as the filter, so a convention mismatch can't
// masquerade as (or hide) a real bug.
void test_static_converges_roll_pitch_and_yaw()
{
    float rollTrue = 15.0f * kDegToRad;
    float pitchTrue = -20.0f * kDegToRad;
    float yawTrue = 35.0f * kDegToRad;
    Quaternion qTrue = Quaternion::fromEulerZYX(rollTrue, pitchTrue, yawTrue);

    float dipAngle = 60.0f * kDegToRad;
    float bxRef = std::cos(dipAngle);
    float bzRef = std::sin(dipAngle);

    float w = qTrue.W(), x = qTrue.X(), y = qTrue.Y(), z = qTrue.Z();
    Vector3 accelTrue = {
        2.0f * (x * z - w * y),
        2.0f * (w * x + y * z),
        2.0f * (0.5f - x * x - y * y)};
    Vector3 magTrue = {
        2.0f * bxRef * (0.5f - y * y - z * z) + 2.0f * bzRef * (x * z - w * y),
        2.0f * bxRef * (x * y - w * z) + 2.0f * bzRef * (w * x + y * z),
        2.0f * bxRef * (w * y + x * z) + 2.0f * bzRef * (0.5f - x * x - y * y)};

    Vector3 gyro = {0.0f, 0.0f, 0.0f};
    float dt = 0.01f;
    int nIterations = 550; // 500 wasn't enough to converge, per the MATLAB reference
    float beta = 0.1f;

    Quaternion q(1.0f, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < nIterations; ++i)
    {
        q = madgwickStepFull(q, gyro, accelTrue, magTrue, dt, beta);
    }

    Quaternion::EulerAngles finalAngles = q.toEulerZYX();

    float angleTol = 1.0f * kDegToRad;
    TEST_ASSERT_FLOAT_WITHIN(angleTol, rollTrue, finalAngles.roll);
    TEST_ASSERT_FLOAT_WITHIN(angleTol, pitchTrue, finalAngles.pitch);
    TEST_ASSERT_FLOAT_WITHIN(angleTol, yawTrue, finalAngles.yaw);
}

// --- Ported from madgwickFullEdgeCasesTest.m, Case 1 ---
// Zero-norm accel (sensor glitch): should mean "unusable this step," not
// "gravity points nowhere" -- the correction from this branch must be
// suppressed, not fed a phantom target.
void test_zero_norm_accel_does_not_move_estimate_from_identity()
{
    Quaternion qIdentity(1.0f, 0.0f, 0.0f, 0.0f);
    Vector3 gyro = {0.0f, 0.0f, 0.0f};
    Vector3 magLevelRef = {0.6f, 0.0f, 0.8f};
    float dt = 0.01f;
    float beta = 0.1f;

    Quaternion qNext = madgwickStepFull(qIdentity, gyro, Vector3{0.0f, 0.0f, 0.0f}, magLevelRef, dt, beta);

    float angleFromIdentity = 2.0f * std::acos(std::min(1.0f, std::fabs(qNext.W())));

    TEST_ASSERT_TRUE(angleFromIdentity < 1.0f * kDegToRad);
}

// --- Ported from madgwickFullEdgeCasesTest.m, Case 2 (adapted) ---
// Zero-norm mag: mag=[0,0,0] should behave identically to mag not being
// used at all -- adapted to compare against a hand-computed gravity-only
// step (see gravityOnlyStep() above) since there's no separate
// madgwickNoMag to cross-check against in this single-function port.
void test_zero_norm_mag_matches_gravity_only_step()
{
    Quaternion qIdentity(1.0f, 0.0f, 0.0f, 0.0f);
    Vector3 gyro = {0.0f, 0.0f, 0.0f};
    Vector3 accelTilt = {0.3f, -0.2f, 0.9f};
    accelTilt = accelTilt / norm(accelTilt);
    float dt = 0.01f;
    float beta = 0.1f;

    Quaternion qFullZeroMag = madgwickStepFull(qIdentity, gyro, accelTilt, Vector3{0.0f, 0.0f, 0.0f}, dt, beta);
    Quaternion qGravityOnly = gravityOnlyStep(qIdentity, gyro, accelTilt, dt, beta);

    float diffAngle = quaternionAngleDifference(qFullZeroMag, qGravityOnly);
    TEST_ASSERT_TRUE(diffAngle < 1e-4f);
}

// --- Ported from madgwickFullEdgeCasesTest.m, Case 3 ---
// Near-pole magnetic field (bx=0, purely vertical) -- physically what
// you'd read near a magnetic pole. Yaw genuinely can't be corrected from
// this (a real physical limit, not a bug), so only check: no NaN, and
// roll/pitch still converge even though yaw won't.
void test_near_pole_field_converges_roll_pitch_without_nan()
{
    float rollT = 10.0f * kDegToRad;
    float pitchT = -10.0f * kDegToRad;
    float yawT = 50.0f * kDegToRad;
    Quaternion qT = Quaternion::fromEulerZYX(rollT, pitchT, yawT);

    float w = qT.W(), x = qT.X(), y = qT.Y(), z = qT.Z();
    Vector3 accelPole = {
        2.0f * (x * z - w * y),
        2.0f * (w * x + y * z),
        2.0f * (0.5f - x * x - y * y)};

    float bxPole = 0.0f, bzPole = 1.0f;
    Vector3 magPole = {
        2.0f * bxPole * (0.5f - y * y - z * z) + 2.0f * bzPole * (x * z - w * y),
        2.0f * bxPole * (x * y - w * z) + 2.0f * bzPole * (w * x + y * z),
        2.0f * bxPole * (w * y + x * z) + 2.0f * bzPole * (0.5f - x * x - y * y)};

    Vector3 gyro = {0.0f, 0.0f, 0.0f};
    float dt = 0.01f;
    float beta = 0.1f;

    Quaternion q(1.0f, 0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 500; ++i)
    {
        q = madgwickStepFull(q, gyro, accelPole, magPole, dt, beta);
    }

    TEST_ASSERT_TRUE(std::isfinite(q.W()));
    TEST_ASSERT_TRUE(std::isfinite(q.X()));
    TEST_ASSERT_TRUE(std::isfinite(q.Y()));
    TEST_ASSERT_TRUE(std::isfinite(q.Z()));

    Quaternion::EulerAngles finalAngles = q.toEulerZYX();
    float angleTol = 1.0f * kDegToRad;
    TEST_ASSERT_FLOAT_WITHIN(angleTol, rollT, finalAngles.roll);
    TEST_ASSERT_FLOAT_WITHIN(angleTol, pitchT, finalAngles.pitch);
    // Yaw deliberately not checked -- unobservable from a purely
    // vertical field, not expected to converge.
}

// --- Ported from madgwickFullEdgeCasesTest.m, Case 4 ---
// Magnitude invariance over an actual iterated trajectory (not just one
// step): scaling accel/mag's raw magnitude by different, unrelated
// factors must not change the resulting trajectory at all, since both
// get normalized internally -- only direction should ever matter.
void test_magnitude_invariance_over_iterated_trajectory()
{
    Quaternion qIdentity(1.0f, 0.0f, 0.0f, 0.0f);
    Vector3 gyro = {0.0f, 0.0f, 0.0f};
    Vector3 accelTilt = {0.3f, -0.2f, 0.9f};
    accelTilt = accelTilt / norm(accelTilt);
    Vector3 magLevelRef = {0.6f, 0.0f, 0.8f};
    float dt = 0.01f;
    float beta = 0.1f;
    float scale = 137.0f;

    Quaternion qUnscaled = qIdentity;
    Quaternion qScaled = qIdentity;
    for (int i = 0; i < 50; ++i)
    {
        qUnscaled = madgwickStepFull(qUnscaled, gyro, accelTilt, magLevelRef, dt, beta);
        Vector3 accelScaled = {accelTilt.x * scale, accelTilt.y * scale, accelTilt.z * scale};
        Vector3 magScaled = {magLevelRef.x / 7.3f, magLevelRef.y / 7.3f, magLevelRef.z / 7.3f};
        qScaled = madgwickStepFull(qScaled, gyro, accelScaled, magScaled, dt, beta);
    }

    float scaleDiff = quaternionAngleDifference(qUnscaled, qScaled);
    TEST_ASSERT_TRUE(scaleDiff < 1e-4f);
}

// --- Convention-independent regression cases added during the C++ port ---

// With both accel and mag zeroed (invalid), the gradient is exactly zero,
// so qdot_corrected collapses to qdot_gyro with no blended term at all --
// result must exactly match plain gyro-only integration.
void test_degrades_to_pure_gyro_integration_when_accel_and_mag_invalid()
{
    Quaternion q(0.9825f, 0.1f, 0.1f, 0.1f);
    Vector3 gyro = {0.2f, -0.1f, 0.05f};
    float dt = 0.01f;
    float beta = 0.1f;

    Quaternion actual = madgwickStepFull(q, gyro, Vector3{0.0f, 0.0f, 0.0f},
                                         Vector3{0.0f, 0.0f, 0.0f}, dt, beta);

    Quaternion qdotGyro = 0.5f * q * Quaternion(0.0f, gyro.x, gyro.y, gyro.z);
    Quaternion expected = (q + qdotGyro * dt).normalize();

    assertQuaternionsClose(expected, actual, kTol);
}

// beta=0 zeroes the correction term regardless of accel/mag validity --
// same expected result as above, reached through a different guard, with
// accel/mag both valid/nonzero this time to confirm beta is doing the work.
void test_beta_zero_ignores_valid_accel_and_mag()
{
    Quaternion q(0.9825f, 0.1f, 0.1f, 0.1f);
    Vector3 gyro = {0.2f, -0.1f, 0.05f};
    Vector3 accel = {0.0f, 0.0f, 1.0f};
    Vector3 magUT = {20.0f, 0.0f, 40.0f};
    float dt = 0.01f;
    float beta = 0.0f;

    Quaternion actual = madgwickStepFull(q, gyro, accel, magUT, dt, beta);

    Quaternion qdotGyro = 0.5f * q * Quaternion(0.0f, gyro.x, gyro.y, gyro.z);
    Quaternion expected = (q + qdotGyro * dt).normalize();

    assertQuaternionsClose(expected, actual, kTol);
}

// General regression: any single valid step should leave q unit-norm,
// regardless of specific inputs -- Madgwick's final normalize() should
// guarantee this.
void test_result_stays_unit_norm()
{
    Quaternion q = Quaternion::identity();
    Vector3 gyro = {0.05f, 0.02f, -0.03f};
    Vector3 accel = {0.1f, -0.2f, 0.95f};
    Vector3 magUT = {22.0f, 3.0f, 41.0f};
    float dt = 0.005f;
    float beta = 0.15f;

    Quaternion actual = madgwickStepFull(q, gyro, accel, magUT, dt, beta);

    TEST_ASSERT_FLOAT_WITHIN(kTol, 1.0f, quatNorm(actual));
}

// --- MadgwickDiagnostics ---

// Passing a diagnostics pointer must not change the result.
void test_diagnostics_do_not_change_result()
{
    Quaternion q(0.9825f, 0.1f, 0.1f, 0.1f);
    Vector3 gyro = {0.2f, -0.1f, 0.05f};
    Vector3 accel = {0.1f, -0.2f, 0.95f};
    Vector3 magUT = {22.0f, 3.0f, 41.0f};

    MadgwickDiagnostics diag;
    Quaternion withDiag = madgwickStepFull(q.normalize(), gyro, accel, magUT, 0.01f, 0.1f, &diag);
    Quaternion without = madgwickStepFull(q.normalize(), gyro, accel, magUT, 0.01f, 0.1f);

    TEST_ASSERT_TRUE(withDiag == without);
}

// A normal small step: raw norm is ~1, and the flags report which sensors were fused.
void test_diagnostics_report_normal_step()
{
    MadgwickDiagnostics diag;
    madgwickStepFull(Quaternion::identity(), Vector3{0.1f, 0.0f, 0.0f}, Vector3{0.0f, 0.0f, 1.0f},
                     Vector3{20.0f, 0.0f, 40.0f}, 0.01f, 0.1f, &diag);

    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, diag.preNormalizeNorm);
    TEST_ASSERT_TRUE(diag.accelUsed);
    TEST_ASSERT_TRUE(diag.magUsed);
}

void test_diagnostics_report_skipped_sensors()
{
    MadgwickDiagnostics diag;
    madgwickStepFull(Quaternion::identity(), Vector3{0.0f, 0.0f, 0.0f}, Vector3{0.0f, 0.0f, 0.0f},
                     Vector3{0.0f, 0.0f, 0.0f}, 0.01f, 0.1f, &diag);

    TEST_ASSERT_FALSE(diag.accelUsed);
    TEST_ASSERT_FALSE(diag.magUsed);
}

// The case the final normalize() hides: a NaN gyro makes the raw step NaN. The
// diagnostics must show it.
void test_diagnostics_expose_non_finite_step()
{
    MadgwickDiagnostics diag;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    madgwickStepFull(Quaternion::identity(), Vector3{nan, 0.0f, 0.0f}, Vector3{0.0f, 0.0f, 1.0f},
                     Vector3{20.0f, 0.0f, 40.0f}, 0.01f, 0.1f, &diag);

    TEST_ASSERT_FALSE(std::isfinite(diag.preNormalizeNorm));
}

// A quaternion that is already degenerate going in collapses the raw step, and
// normalize() turns it into a healthy-looking identity. Only preNormalizeNorm
// tells you.
void test_diagnostics_expose_collapsed_step()
{
    MadgwickDiagnostics diag;
    Quaternion out = madgwickStepFull(Quaternion(0.0f, 0.0f, 0.0f, 0.0f), Vector3{0.0f, 0.0f, 0.0f},
                                      Vector3{0.0f, 0.0f, 0.0f}, Vector3{0.0f, 0.0f, 0.0f}, 0.01f, 0.1f, &diag);

    TEST_ASSERT_TRUE(out == Quaternion::identity()); // looks fine...
    TEST_ASSERT_TRUE(diag.preNormalizeNorm < 1e-6f); // ...but wasn't
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_static_converges_roll_pitch_and_yaw);
    RUN_TEST(test_zero_norm_accel_does_not_move_estimate_from_identity);
    RUN_TEST(test_zero_norm_mag_matches_gravity_only_step);
    RUN_TEST(test_near_pole_field_converges_roll_pitch_without_nan);
    RUN_TEST(test_magnitude_invariance_over_iterated_trajectory);
    RUN_TEST(test_degrades_to_pure_gyro_integration_when_accel_and_mag_invalid);
    RUN_TEST(test_beta_zero_ignores_valid_accel_and_mag);
    RUN_TEST(test_result_stays_unit_norm);
    RUN_TEST(test_diagnostics_do_not_change_result);
    RUN_TEST(test_diagnostics_report_normal_step);
    RUN_TEST(test_diagnostics_report_skipped_sensors);
    RUN_TEST(test_diagnostics_expose_non_finite_step);
    RUN_TEST(test_diagnostics_expose_collapsed_step);
    return UNITY_END();
}