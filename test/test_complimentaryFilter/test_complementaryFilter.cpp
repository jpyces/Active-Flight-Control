// test/test_complementary_filter/test_complementary_filter.cpp
//
// Native Unity test for gnc::complementaryFilter, mirroring the shape of
// test/test_quaternion/test_quaternion.cpp: run with `pio test -e native`.
//
// NOTE: this assumes Quaternion exposes const accessors w()/x()/y()/z() and
// that Quaternion::EulerAngles (nested inside Quaternion) has .roll/.pitch/.yaw
// fields. If the real accessor names differ, swap them in below -- nothing
// else about the test logic depends on the exact accessor spelling.

#include <unity.h>
#include <cmath>

#include "ComplimentaryFilter.h"
#include "Quaternion.h"
#include "Vector3.h"

using namespace gnc;

namespace
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTol = 1e-4f;

    // Componentwise tolerance check, accounting for the fact that q and -q
    // represent the same rotation (double cover) and that anything routed
    // through atan2/sqrt (rather than compared bit-for-bit) will pick up a
    // few ULPs of floating-point error even when mathematically identical.
    bool quaternionsApproxEqual(const Quaternion &a, const Quaternion &b, float tol)
    {
        bool sameSign = std::fabs(a.W() - b.W()) < tol &&
                        std::fabs(a.X() - b.X()) < tol &&
                        std::fabs(a.Y() - b.Y()) < tol &&
                        std::fabs(a.Z() - b.Z()) < tol;
        bool oppositeSign = std::fabs(a.W() + b.W()) < tol &&
                            std::fabs(a.X() + b.X()) < tol &&
                            std::fabs(a.Y() + b.Y()) < tol &&
                            std::fabs(a.Z() + b.Z()) < tol;
        return sameSign || oppositeSign;
    }

    // Gravity-only, unit-magnitude accel reading for a given roll/pitch, at
    // yaw=0 -- i.e. exactly the reading that should make the accel branch
    // reconstruct that roll/pitch. Matches the filter's own atan2 convention
    // (roll = atan2(ay, az), pitch = atan2(-ax, sqrt(ay^2+az^2))) so the test
    // can't disagree with the filter about what "tilted by rollAngle" means.
    Vector3 gravityAccelForTilt(float rollAngle, float pitchAngle)
    {
        return {
            -std::sin(pitchAngle),
            std::cos(pitchAngle) * std::sin(rollAngle),
            std::cos(pitchAngle) * std::cos(rollAngle),
        };
    }
}

void setUp() {}
void tearDown() {}

// At rest (zero gyro, level accel) starting from identity, both branches
// agree exactly on identity, so the blend at any alpha must be identity too.
void test_at_rest_identity_is_noop()
{
    Quaternion q = Quaternion::identity();
    Vector3 gyro = {0.0f, 0.0f, 0.0f};
    Vector3 accel = {0.0f, 0.0f, 1.0f};

    Quaternion result = complementaryFilter(q, gyro, accel, 0.01f, 0.98f);

    TEST_ASSERT_TRUE(result == Quaternion::identity());
}

// alpha = 1 should reduce exactly to the gyro-only (predict) branch,
// regardless of what the accel branch says -- (1 - alpha) weights it to zero.
void test_alpha_one_matches_gyro_branch_only()
{
    Quaternion q = Quaternion::identity();
    Vector3 gyro = {0.1f, -0.2f, 0.05f};
    // Deliberately inconsistent accel reading -- should be fully ignored.
    Vector3 accel = {5.0f, -3.0f, 1.0f};
    float dt = 0.01f;

    Quaternion expected = q.integrateGyro(gyro, dt).normalize();
    Quaternion result = complementaryFilter(q, gyro, accel, dt, 1.0f);

    TEST_ASSERT_TRUE(result == expected);
}

// alpha = 0 should reduce exactly to the accel-only (correct) branch,
// regardless of gyro input -- alpha weights the gyro branch to zero.
void test_alpha_zero_matches_accel_branch_only()
{
    Quaternion q = Quaternion::identity();
    Vector3 gyro = {1.0f, -1.0f, 0.5f}; // should be fully ignored
    float rollAngle = 0.3f;
    float pitchAngle = -0.2f;
    Vector3 accel = gravityAccelForTilt(rollAngle, pitchAngle);

    Quaternion expected = Quaternion::fromEulerZYX(rollAngle, pitchAngle, 0.0f).normalize();
    Quaternion result = complementaryFilter(q, gyro, accel, 0.01f, 0.0f);

    // Not bit-exact: the filter recovers roll/pitch via atan2/sqrt on the
    // normalized accel reading before calling fromEulerZYX, rather than
    // reusing rollAngle/pitchAngle directly, so a few ULPs of floating-point
    // error are expected even though the two are mathematically the same
    // rotation.
    TEST_ASSERT_TRUE(quaternionsApproxEqual(result, expected, kTol));
}

// Roll-only tilt: the accel branch should recover the true roll angle (yaw
// forced to 0, pitch stays 0), confirmed by round-tripping through
// toEulerZYX rather than comparing raw quaternion components.
void test_roll_only_tilt_recovered()
{
    Quaternion q = Quaternion::identity();
    Vector3 gyro = {0.0f, 0.0f, 0.0f};
    float rollAngle = 25.0f * kPi / 180.0f;
    Vector3 accel = gravityAccelForTilt(rollAngle, 0.0f);

    Quaternion result = complementaryFilter(q, gyro, accel, 0.01f, 0.0f);
    Quaternion::EulerAngles angles = result.toEulerZYX();

    TEST_ASSERT_FLOAT_WITHIN(kTol, rollAngle, angles.roll);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, angles.pitch);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, angles.yaw);
}

// Pitch-only tilt: same idea, other axis.
void test_pitch_only_tilt_recovered()
{
    Quaternion q = Quaternion::identity();
    Vector3 gyro = {0.0f, 0.0f, 0.0f};
    float pitchAngle = -15.0f * kPi / 180.0f;
    Vector3 accel = gravityAccelForTilt(0.0f, pitchAngle);

    Quaternion result = complementaryFilter(q, gyro, accel, 0.01f, 0.0f);
    Quaternion::EulerAngles angles = result.toEulerZYX();

    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, angles.roll);
    TEST_ASSERT_FLOAT_WITHIN(kTol, pitchAngle, angles.pitch);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, angles.yaw);
}

// Result must always be unit-norm -- the whole reason normalize() is called
// after the weighted-addition blend (addition of two unit quaternions is not
// itself unit-norm in general).
void test_result_is_unit_norm()
{
    Quaternion q = Quaternion::identity();
    Vector3 gyro = {0.3f, 0.1f, -0.2f};
    Vector3 accel = gravityAccelForTilt(0.2f, 0.1f);

    Quaternion result = complementaryFilter(q, gyro, accel, 0.01f, 0.7f);

    TEST_ASSERT_FLOAT_WITHIN(kTol, 1.0f, result.norm());
}

// Fixed: a zero-norm accel reading (sensor glitch / free-fall) now hits the
// accelValid-style guard and falls back to the gyro-only prediction, instead
// of dividing by ~0 and producing NaN. Same principle as the fix already
// applied to madgwickStepFull/madgwickNoMag: zero the correction, don't feed
// a fake reading into the normal formula.
void test_zero_norm_accel_does_not_produce_nan()
{
    Quaternion q = Quaternion::identity();
    Vector3 gyro = {0.0f, 0.0f, 0.0f};
    Vector3 accel = {0.0f, 0.0f, 0.0f};

    Quaternion result = complementaryFilter(q, gyro, accel, 0.01f, 0.98f);

    TEST_ASSERT_FALSE(std::isnan(result.W()));
    TEST_ASSERT_FALSE(std::isnan(result.X()));
    TEST_ASSERT_FALSE(std::isnan(result.Y()));
    TEST_ASSERT_FALSE(std::isnan(result.Z()));
}

// The fallback isn't just "not NaN" -- it should be exactly the gyro-only
// prediction (alpha's weighting is moot once the accel branch is fully
// zeroed out), same as the alpha=1 case above.
void test_zero_norm_accel_falls_back_to_gyro_only()
{
    Quaternion q = Quaternion::identity();
    Vector3 gyro = {0.2f, -0.1f, 0.05f};
    Vector3 accel = {0.0f, 0.0f, 0.0f};
    float dt = 0.01f;

    Quaternion expected = q.integrateGyro(gyro, dt).normalize();
    Quaternion result = complementaryFilter(q, gyro, accel, dt, 0.5f);

    TEST_ASSERT_TRUE(result == expected);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_at_rest_identity_is_noop);
    RUN_TEST(test_alpha_one_matches_gyro_branch_only);
    RUN_TEST(test_alpha_zero_matches_accel_branch_only);
    RUN_TEST(test_roll_only_tilt_recovered);
    RUN_TEST(test_pitch_only_tilt_recovered);
    RUN_TEST(test_result_is_unit_norm);
    RUN_TEST(test_zero_norm_accel_does_not_produce_nan);
    RUN_TEST(test_zero_norm_accel_falls_back_to_gyro_only);
    return UNITY_END();
}