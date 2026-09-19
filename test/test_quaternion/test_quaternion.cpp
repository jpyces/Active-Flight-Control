#include <unity.h>
#include <cmath>
#include "Quaternion.h"

using namespace gnc;

constexpr float kTol = 1e-4f;

void setUp(void) {}
void tearDown(void) {}

// Identity multiply is a no-op
void test_identity_multiply_is_noop(void)
{
    Quaternion q(0.5f, 0.1f, 0.2f, 0.3f);
    Quaternion result = q * Quaternion::identity();

    TEST_ASSERT_FLOAT_WITHIN(kTol, q.W(), result.W());
    TEST_ASSERT_FLOAT_WITHIN(kTol, q.X(), result.X());
    TEST_ASSERT_FLOAT_WITHIN(kTol, q.Y(), result.Y());
    TEST_ASSERT_FLOAT_WITHIN(kTol, q.Z(), result.Z());
}

// Norm stays 1 after normalize
void test_normalize_gives_unit_norm(void)
{
    Quaternion q(2.0f, 1.0f, 1.0f, 1.0f);
    Quaternion unit = q.normalize();
    TEST_ASSERT_FLOAT_WITHIN(kTol, 1.0f, unit.norm());
}

// conjugate then inverse-relationship: q * conjugate(q) should be (norm^2, 0, 0, 0)
void test_conjugate_multiply_gives_norm_squared(void)
{
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion result = q * q.conjugate();
    float expectedW = q.norm() * q.norm();

    TEST_ASSERT_FLOAT_WITHIN(kTol, expectedW, result.W());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, result.X());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, result.Y());
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, result.Z());
}

// toEulerZYX/fromEulerZYX round-trip, away from gimbal lock
// -- this is the case that would have caught the sinp bug:
// a garbage/uninitialized sinp would make pitch fail this
// assertion nearly every run, since it wouldn't match the
// roll/pitch/yaw actually fed in.
void test_euler_round_trip(void)
{
    float roll = 0.3f, pitch = 0.2f, yaw = 0.5f; // away from +-90 deg pitch
    Quaternion q = Quaternion::fromEulerZYX(roll, pitch, yaw);
    Quaternion::EulerAngles result = q.toEulerZYX();

    TEST_ASSERT_FLOAT_WITHIN(kTol, roll, result.roll);
    TEST_ASSERT_FLOAT_WITHIN(kTol, pitch, result.pitch);
    TEST_ASSERT_FLOAT_WITHIN(kTol, yaw, result.yaw);
}

// Degenerate quaternion normalizes to identity rather than NaN/crash
void test_normalize_degenerate_returns_identity(void)
{
    Quaternion zero(0.0f, 0.0f, 0.0f, 0.0f);
    Quaternion result = zero.normalize();

    TEST_ASSERT_TRUE(result == Quaternion::identity());
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_identity_multiply_is_noop);
    RUN_TEST(test_normalize_gives_unit_norm);
    RUN_TEST(test_conjugate_multiply_gives_norm_squared);
    RUN_TEST(test_euler_round_trip);
    RUN_TEST(test_normalize_degenerate_returns_identity);
    return UNITY_END();
}