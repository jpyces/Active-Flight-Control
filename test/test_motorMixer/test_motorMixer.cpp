#include <unity.h>
#include "eigen.h"
#include "MotorMixer.h"

using namespace gnc;

void test_no_saturation_needed_passes_through()
{
    Eigen::Vector4f command(0.5f, 0.6f, 0.3f, 0.4f);
    Eigen::Vector4f motorCmds = mixMotors(command, Eigen::Matrix4f::Identity());

    for (int i = 0; i < 4; ++i)
    {
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, command(i), motorCmds(i));
    }
}

void test_zero_command_gives_zero_motors()
{
    Eigen::Vector4f command = Eigen::Vector4f::Zero();
    Eigen::Vector4f motorCmds = mixMotors(command, Eigen::Matrix4f::Identity());

    for (int i = 0; i < 4; ++i)
    {
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, motorCmds(i));
    }
}

void test_negative_motor_gets_shifted_preserving_differences()
{
    Eigen::Vector4f command(0.3f, -0.2f, 0.5f, 0.1f);
    Eigen::Vector4f before = command;

    Eigen::Vector4f motorCmds = mixMotors(command, Eigen::Matrix4f::Identity());

    // lowest motor should now be exactly 0
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, motorCmds.minCoeff());

    // pairwise differences must be unchanged by a uniform shift
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            float diffBefore = before(i) - before(j);
            float diffAfter = motorCmds(i) - motorCmds(j);
            TEST_ASSERT_FLOAT_WITHIN(1e-6f, diffBefore, diffAfter);
        }
    }
}

void test_over_max_motor_gets_scaled_preserving_ratios()
{
    Eigen::Vector4f command(0.6f, 0.9f, 1.5f, 0.3f);
    Eigen::Vector4f before = command;

    Eigen::Vector4f motorCmds = mixMotors(command, Eigen::Matrix4f::Identity());

    // highest motor should now be exactly 1.0
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, motorCmds.maxCoeff());

    // pairwise ratios must be unchanged by a uniform scale
    // (compare via cross-multiplication to avoid divide-by-zero)
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            float crossBefore = before(i) * motorCmds(j);
            float crossAfter = motorCmds(i) * before(j);
            TEST_ASSERT_FLOAT_WITHIN(1e-4f, crossBefore, crossAfter);
        }
    }
}

void test_shift_then_scale_applied_together()
{
    // constructed so shifting alone still leaves a motor above 1.0,
    // forcing both correction steps to run, in shift-then-scale order
    Eigen::Vector4f command(-0.5f, 1.2f, 0.0f, 0.3f);

    Eigen::Vector4f motorCmds = mixMotors(command, Eigen::Matrix4f::Identity());

    for (int i = 0; i < 4; ++i)
    {
        TEST_ASSERT_TRUE(motorCmds(i) >= -1e-6f);
        TEST_ASSERT_TRUE(motorCmds(i) <= 1.0f + 1e-6f);
    }
    // at least one bound should be tight, confirming saturation logic actually ran
    TEST_ASSERT_TRUE(motorCmds.minCoeff() < 1e-3f || motorCmds.maxCoeff() > 1.0f - 1e-3f);
}

void test_all_motors_already_in_range_unchanged()
{
    Eigen::Vector4f command(0.2f, 0.4f, 0.6f, 0.8f);
    Eigen::Vector4f motorCmds = mixMotors(command, Eigen::Matrix4f::Identity());

    for (int i = 0; i < 4; ++i)
    {
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, command(i), motorCmds(i));
    }
}

void test_pure_throttle_command_produces_equal_motors()
{
    // illustrative quad-X mix matrix: columns are [throttle, roll, pitch, yaw]
    // swap for the project's real mixMatrix if motor order/signs differ
    Eigen::Matrix4f mixMatrix;
    mixMatrix << 1.0f, 1.0f, 1.0f, 1.0f,
        1.0f, -1.0f, 1.0f, -1.0f,
        1.0f, -1.0f, -1.0f, 1.0f,
        1.0f, 1.0f, -1.0f, -1.0f;

    Eigen::Vector4f command(0.5f, 0.0f, 0.0f, 0.0f);
    Eigen::Vector4f motorCmds = mixMotors(command, mixMatrix);

    for (int i = 1; i < 4; ++i)
    {
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, motorCmds(0), motorCmds(i));
    }
}

void test_mix_matrix_columns_are_orthogonal()
{
    Eigen::Matrix4f mixMatrix;
    mixMatrix << 1.0f, 1.0f, 1.0f, 1.0f,
        1.0f, -1.0f, 1.0f, -1.0f,
        1.0f, -1.0f, -1.0f, 1.0f,
        1.0f, 1.0f, -1.0f, -1.0f;

    // roll, pitch, yaw columns should be mutually orthogonal (decoupled axes) —
    // throttle is intentionally excluded, since it's a uniform column, not an
    // independent control axis in the same sense
    for (int i = 1; i < 4; ++i)
    {
        for (int j = i + 1; j < 4; ++j)
        {
            float dot = mixMatrix.col(i).dot(mixMatrix.col(j));
            TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, dot);
        }
    }
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_no_saturation_needed_passes_through);
    RUN_TEST(test_zero_command_gives_zero_motors);
    RUN_TEST(test_negative_motor_gets_shifted_preserving_differences);
    RUN_TEST(test_over_max_motor_gets_scaled_preserving_ratios);
    RUN_TEST(test_shift_then_scale_applied_together);
    RUN_TEST(test_all_motors_already_in_range_unchanged);
    RUN_TEST(test_pure_throttle_command_produces_equal_motors);
    RUN_TEST(test_mix_matrix_columns_are_orthogonal);
    return UNITY_END();
}