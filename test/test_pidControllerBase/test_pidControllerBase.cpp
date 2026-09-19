#include <unity.h>
#include <cmath>

#include "PIDControllerBase.h"

using namespace gnc;

namespace
{
    constexpr float kTol = 1e-4f;
}

void setUp() {}
void tearDown() {}

// Ki=Kd=0: output should be exactly Kp * error, nothing else contributing.
void test_pure_p_output()
{
    PIDControllerBase pid(2.0f, 0.0f, 0.0f, -100.0f, 100.0f);

    float output = pid.update(/*setpoint=*/10.0f, /*measurement=*/4.0f, /*dt=*/0.1f);

    TEST_ASSERT_FLOAT_WITHIN(kTol, 2.0f * (10.0f - 4.0f), output);
}

// Kp=Kd=0: integral should accumulate as a plain Riemann sum (error * dt each
// step), so after N identical steps the I-term is Ki * error * dt * N.
void test_integral_accumulates_as_riemann_sum()
{
    PIDControllerBase pid(0.0f, 1.0f, 0.0f, -100.0f, 100.0f);
    float setpoint = 5.0f;
    float measurement = 0.0f; // constant error = 5 each step
    float dt = 0.1f;

    float out1 = pid.update(setpoint, measurement, dt);
    float out2 = pid.update(setpoint, measurement, dt);
    float out3 = pid.update(setpoint, measurement, dt);

    TEST_ASSERT_FLOAT_WITHIN(kTol, 5.0f * dt * 1.0f, out1);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 5.0f * dt * 2.0f, out2);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 5.0f * dt * 3.0f, out3);
}

// D is suppressed on the very first update() call -- no prevMeasurement to
// difference against yet, so the D contribution must be exactly zero, not
// just small.
void test_derivative_suppressed_on_first_call()
{
    PIDControllerBase pid(0.0f, 0.0f, 10.0f, -100.0f, 100.0f);

    float output = pid.update(/*setpoint=*/0.0f, /*measurement=*/7.0f, /*dt=*/0.1f);

    TEST_ASSERT_FLOAT_WITHIN(kTol, 0.0f, output);
}

// From the second call on, D should match the D-on-measurement finite
// difference: -Kd * (measurement - prevMeasurement) / dt.
void test_derivative_matches_finite_difference_after_first_call()
{
    PIDControllerBase pid(0.0f, 0.0f, 10.0f, -1000.0f, 1000.0f);
    float dt = 0.1f;

    pid.update(0.0f, 2.0f, dt);                // first call: seeds prevMeasurement=2, D suppressed
    float output = pid.update(0.0f, 5.0f, dt); // measurement jumps 2 -> 5

    float expectedD = -10.0f * (5.0f - 2.0f) / dt;
    TEST_ASSERT_FLOAT_WITHIN(kTol, expectedD, output);
}

// Output must clamp to outMax when the raw P+I+D sum exceeds it.
void test_output_clamps_at_upper_bound()
{
    PIDControllerBase pid(10.0f, 0.0f, 0.0f, -1.0f, 1.0f);

    float output = pid.update(/*setpoint=*/100.0f, /*measurement=*/0.0f, /*dt=*/0.1f);

    TEST_ASSERT_FLOAT_WITHIN(kTol, 1.0f, output);
}

// Output must clamp to outMin when the raw P+I+D sum is below it.
void test_output_clamps_at_lower_bound()
{
    PIDControllerBase pid(10.0f, 0.0f, 0.0f, -1.0f, 1.0f);

    float output = pid.update(/*setpoint=*/-100.0f, /*measurement=*/0.0f, /*dt=*/0.1f);

    TEST_ASSERT_FLOAT_WITHIN(kTol, -1.0f, output);
}

// Anti-windup: with an ASYMMETRIC bound (a [0,1] throttle-trim-style range,
// not zero-centered) and a large positive, saturating error, the integral
// must freeze rather than keep accumulating -- checked against the actual
// bound, not the sign of the output, which is the specific bug class this
// case exists to catch.
void test_integral_freezes_while_pushing_into_asymmetric_saturation()
{
    PIDControllerBase pid(0.0f, 1.0f, 0.0f, /*outMin=*/0.0f, /*outMax=*/1.0f);
    float dt = 0.1f;

    // error = 100 - 0 = 100 each call; Ki=1 means the raw integral term alone
    // is already far past outMax=1 on the very first step.
    float out1 = pid.update(100.0f, 0.0f, dt);
    float out2 = pid.update(100.0f, 0.0f, dt);
    float out3 = pid.update(100.0f, 0.0f, dt);

    // All three calls should saturate at outMax, and stay there -- if the
    // integral were wrongly still accumulating (e.g. a sign-based windup
    // check misreading this asymmetric bound), the output itself can't show
    // it directly since it's clamped either way, but a subsequent reversal
    // (tested next) would expose it.
    TEST_ASSERT_FLOAT_WITHIN(kTol, 1.0f, out1);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 1.0f, out2);
    TEST_ASSERT_FLOAT_WITHIN(kTol, 1.0f, out3);
}

// Continuation of the above: once the error reverses (measurement now
// exceeds setpoint), the frozen integral should resume accumulating from
// where it was left, not from some inflated windup value. Verified by
// comparing against a fresh controller driven directly to the same integral
// state without ever having saturated.
void test_integral_resumes_once_error_stops_pushing_into_saturation()
{
    float dt = 0.1f;

    PIDControllerBase pidUnderTest(0.0f, 1.0f, 0.0f, 0.0f, 1.0f);
    pidUnderTest.update(100.0f, 0.0f, dt); // saturates upper, integral frozen at 0
    pidUnderTest.update(100.0f, 0.0f, dt); // still frozen at 0
    // Error reverses hard: now well past setpoint, pushing toward outMin instead.
    float outAfterReversal = pidUnderTest.update(-100.0f, 0.0f, dt);

    // Integral was frozen at 0 (never accumulated while saturated), so this
    // is equivalent to a fresh controller's very first call with the same
    // reversed error.
    PIDControllerBase freshPid(0.0f, 1.0f, 0.0f, 0.0f, 1.0f);
    float expected = freshPid.update(-100.0f, 0.0f, dt);

    TEST_ASSERT_FLOAT_WITHIN(kTol, expected, outAfterReversal);
}

// reset() should restore fresh-object behavior: integral back to 0 and D
// suppressed again on the next call, exactly like a newly constructed
// controller.
void test_reset_restores_fresh_object_behavior()
{
    float dt = 0.1f;
    PIDControllerBase pid(1.0f, 1.0f, 5.0f, -1000.0f, 1000.0f);

    pid.update(10.0f, 0.0f, dt);
    pid.update(10.0f, 3.0f, dt);
    pid.reset();

    float output = pid.update(10.0f, 4.0f, dt);

    PIDControllerBase freshPid(1.0f, 1.0f, 5.0f, -1000.0f, 1000.0f);
    float expected = freshPid.update(10.0f, 4.0f, dt);

    TEST_ASSERT_FLOAT_WITHIN(kTol, expected, output);
}

// Two independently constructed controllers must not share state -- this is
// the C++ analogue of the MATLAB handle-semantics test (two variables
// referencing one handle object share state; here, two separate objects
// simply are separate objects, which is the point of NOT making this a
// singleton/shared-state design).
void test_separate_instances_do_not_share_state()
{
    float dt = 0.1f;
    PIDControllerBase pidA(0.0f, 1.0f, 0.0f, -1000.0f, 1000.0f);
    PIDControllerBase pidB(0.0f, 1.0f, 0.0f, -1000.0f, 1000.0f);

    pidA.update(10.0f, 0.0f, dt);
    pidA.update(10.0f, 0.0f, dt);

    // pidB has never been touched -- its first call should behave exactly
    // like any other fresh controller, unaffected by pidA's accumulated integral.
    float outputB = pidB.update(10.0f, 0.0f, dt);

    TEST_ASSERT_FLOAT_WITHIN(kTol, 10.0f * dt, outputB);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_pure_p_output);
    RUN_TEST(test_integral_accumulates_as_riemann_sum);
    RUN_TEST(test_derivative_suppressed_on_first_call);
    RUN_TEST(test_derivative_matches_finite_difference_after_first_call);
    RUN_TEST(test_output_clamps_at_upper_bound);
    RUN_TEST(test_output_clamps_at_lower_bound);
    RUN_TEST(test_integral_freezes_while_pushing_into_asymmetric_saturation);
    RUN_TEST(test_integral_resumes_once_error_stops_pushing_into_saturation);
    RUN_TEST(test_reset_restores_fresh_object_behavior);
    RUN_TEST(test_separate_instances_do_not_share_state);
    return UNITY_END();
}
