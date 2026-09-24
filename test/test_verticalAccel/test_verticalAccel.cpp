// test/test_vertical_accel/test_vertical_accel.cpp
//
// Unity tests for gnc::verticalAccelFromBody (pio test -e native).
//
// Contract under test (1:1 port of verticalAccelFromBody.m):
//   float verticalAccelFromBody(const Vector3& accelBody,  // specific force, m/s^2, body frame
//                               const Quaternion& q,        // Madgwick's estimate
//                               float gravity);             // m/s^2, positive
//   returns world-frame vertical acceleration, z-up, m/s^2 (0 at rest, -g in free fall)
//
// Expected values are hand-computed from Madgwick's own gravity-direction formula
// (the same convention the MATLAB version is built on). Quaternion::rotateVector is
// deliberately NOT used, matching the MATLAB reference's choice.
//
// ASSUMPTIONS to check against your headers:
//   - header name "VerticalAccel.h" (rename to match yours)
//   - Quaternion(w, x, y, z) constructor, scalar-first; W()/X()/Y()/Z() accessors
//   - Vector3 is a {x, y, z} aggregate

#include <unity.h>
#include <cmath>
#include "Quaternion.h"
#include "VerticalAccel.h"

using namespace gnc;

namespace
{
    constexpr float kG = 9.81f;
    constexpr float kTol = 1e-4f;
    constexpr float kPi = 3.14159265358979f;

    Quaternion axisAngle(float ax, float ay, float az, float angleRad)
    {
        const float n = std::sqrt(ax * ax + ay * ay + az * az);
        const float s = std::sin(angleRad / 2.0f) / n;
        return Quaternion(std::cos(angleRad / 2.0f), ax * s, ay * s, az * s);
    }

    // What a perfect accelerometer reads at rest for attitude q, per Madgwick's f_g:
    // world "up" expressed in body axes, scaled by g.
    Vector3 atRestReading(const Quaternion &q)
    {
        const float w = q.W(), x = q.X(), y = q.Y(), z = q.Z();
        return Vector3{kG * 2.0f * (x * z - w * y),
                       kG * 2.0f * (w * x + y * z),
                       kG * (1.0f - 2.0f * (x * x + y * y))};
    }
} // namespace

void setUp() {}
void tearDown() {}

// Level, stationary: accel reads +g on body z, vertical accel is zero.
void test_level_at_rest_is_zero()
{
    const float a = verticalAccelFromBody(Vector3{0.0f, 0.0f, kG}, Quaternion::identity(), kG);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, a);
}

// Level, accelerating upward at 1 g: accel reads 2g, vertical accel is +g.
void test_level_climbing_one_g()
{
    const float a = verticalAccelFromBody(Vector3{0.0f, 0.0f, 2.0f * kG}, Quaternion::identity(), kG);
    TEST_ASSERT_FLOAT_WITHIN(kTol, kG, a);
}

// Free fall: accelerometer reads zero, vertical accel is -g. Sign check on gravity removal.
void test_free_fall_is_minus_g()
{
    const float a = verticalAccelFromBody(Vector3{0.0f, 0.0f, 0.0f}, Quaternion::identity(), kG);
    TEST_ASSERT_FLOAT_WITHIN(kTol, -kG, a);
}

// Rolled +90 deg, stationary: body y points up, accel reads (0, +g, 0).
// Skipping the rotation (the MATLAB synthetic-sensor bug class) would return -g.
void test_rolled_90_at_rest_is_zero()
{
    const Quaternion q = axisAngle(1.0f, 0.0f, 0.0f, kPi / 2.0f);
    const float a = verticalAccelFromBody(Vector3{0.0f, kG, 0.0f}, q, kG);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, a);
}

// Pitched +90 deg, stationary: accel reads (-g, 0, 0).
// A sign error in the 2(xz - wy) term returns -2g here.
void test_pitched_90_at_rest_is_zero()
{
    const Quaternion q = axisAngle(0.0f, 1.0f, 0.0f, kPi / 2.0f);
    const float a = verticalAccelFromBody(Vector3{-kG, 0.0f, 0.0f}, q, kG);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, a);
}

// Arbitrary attitude at rest: exercises every term at once.
void test_arbitrary_attitude_at_rest_is_zero()
{
    const Quaternion q = axisAngle(0.3f, -0.7f, 0.5f, 1.1f);
    const float a = verticalAccelFromBody(atRestReading(q), q, kG);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, a);
}

// Tilted hover: 30 deg roll with body-z thrust of g/cos(30) holds altitude exactly.
void test_tilted_hover_holding_altitude_is_zero()
{
    const float tilt = kPi / 6.0f;
    const Quaternion q = axisAngle(1.0f, 0.0f, 0.0f, tilt);
    const float a = verticalAccelFromBody(Vector3{0.0f, 0.0f, kG / std::cos(tilt)}, q, kG);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, a);
}

// Tilted with level-hover thrust: sinks at g*(cos(tilt) - 1).
void test_tilted_with_level_thrust_sinks()
{
    const float tilt = kPi / 6.0f;
    const Quaternion q = axisAngle(0.0f, 1.0f, 0.0f, tilt);
    const float a = verticalAccelFromBody(Vector3{0.0f, 0.0f, kG}, q, kG);
    TEST_ASSERT_FLOAT_WITHIN(kTol, kG * std::cos(tilt) - kG, a);
}

// Yaw alone must not change the vertical channel.
void test_yaw_does_not_affect_vertical()
{
    const Quaternion q = axisAngle(0.0f, 0.0f, 1.0f, 2.0f);
    const float a = verticalAccelFromBody(Vector3{0.0f, 0.0f, 1.5f * kG}, q, kG);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.5f * kG, a);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_level_at_rest_is_zero);
    RUN_TEST(test_level_climbing_one_g);
    RUN_TEST(test_free_fall_is_minus_g);
    RUN_TEST(test_rolled_90_at_rest_is_zero);
    RUN_TEST(test_pitched_90_at_rest_is_zero);
    RUN_TEST(test_arbitrary_attitude_at_rest_is_zero);
    RUN_TEST(test_tilted_hover_holding_altitude_is_zero);
    RUN_TEST(test_tilted_with_level_thrust_sinks);
    RUN_TEST(test_yaw_does_not_affect_vertical);
    return UNITY_END();
}