#include <unity.h>
#include "eigen.h"
#include "VerticalKF.h"

using namespace gnc;

namespace
{
    VerticalKF makeDefaultKF()
    {
        Eigen::Matrix3f Q = Eigen::Matrix3f::Identity() * 1e-4f;
        float R = 1.0f;
        Eigen::Vector3f x0 = Eigen::Vector3f::Zero();
        Eigen::Matrix3f P0 = Eigen::Matrix3f::Identity();
        return VerticalKF(Q, R, x0, P0);
    }
}

void test_predict_with_zero_accel_holds_state()
{
    VerticalKF kf = makeDefaultKF();
    kf.predict(0.0f, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, kf.altitude());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, kf.velocity());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, kf.accelBias());
}

void test_predict_with_constant_accel_matches_kinematics()
{
    VerticalKF kf = makeDefaultKF();
    float a = 2.0f;
    float dt = 0.01f;

    for (int i = 0; i < 100; ++i)
    {
        kf.predict(a, dt);
    }

    float t = 100 * dt;
    float expectedVel = a * t;
    float expectedPos = 0.5f * a * t * t;

    TEST_ASSERT_FLOAT_WITHIN(1e-3f, expectedVel, kf.velocity());
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, expectedPos, kf.altitude());
}

void test_correct_moves_estimate_toward_measurement_not_onto_it()
{
    VerticalKF kf = makeDefaultKF();
    float zMeas = 5.0f;

    kf.correct(zMeas);

    TEST_ASSERT_TRUE(kf.altitude() > 0.0f);
    TEST_ASSERT_TRUE(kf.altitude() < zMeas);
}

void test_repeated_correct_converges_to_measurement()
{
    VerticalKF kf = makeDefaultKF();
    float zMeas = 5.0f;

    for (int i = 0; i < 2000; ++i)
    {
        kf.correct(zMeas);
    }

    TEST_ASSERT_FLOAT_WITHIN(1e-2f, zMeas, kf.altitude());
}

void test_correct_shrinks_covariance_trace()
{
    VerticalKF kf = makeDefaultKF();
    Eigen::Matrix3f P0 = kf.covariance();
    float trace0 = P0.trace();

    kf.correct(5.0f);

    float trace1 = kf.covariance().trace();
    TEST_ASSERT_TRUE(trace1 < trace0);
}

void test_covariance_stays_symmetric_after_several_cycles()
{
    VerticalKF kf = makeDefaultKF();

    for (int i = 0; i < 50; ++i)
    {
        kf.predict(1.0f, 0.01f);
        kf.correct(1.0f);
    }

    Eigen::Matrix3f P = kf.covariance();
    for (int r = 0; r < 3; ++r)
    {
        for (int c = 0; c < 3; ++c)
        {
            TEST_ASSERT_FLOAT_WITHIN(1e-4f, P(r, c), P(c, r));
        }
    }
}

void test_reset_restores_given_state_and_covariance()
{
    VerticalKF kf = makeDefaultKF();
    kf.predict(3.0f, 0.02f);
    kf.correct(1.5f);

    Eigen::Vector3f x0(1.0f, 2.0f, 0.1f);
    Eigen::Matrix3f P0 = Eigen::Matrix3f::Identity() * 0.5f;
    kf.reset(x0, P0);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, kf.altitude());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.0f, kf.velocity());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.1f, kf.accelBias());

    Eigen::Matrix3f P = kf.covariance();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, P(0, 0));
}

void test_state_and_covariance_reflect_latest_update()
{
    VerticalKF kf = makeDefaultKF();
    kf.predict(1.0f, 0.01f);
    kf.correct(0.5f);

    Eigen::Vector3f x = kf.state();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, kf.altitude(), x(0));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, kf.velocity(), x(1));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, kf.accelBias(), x(2));
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_predict_with_zero_accel_holds_state);
    RUN_TEST(test_predict_with_constant_accel_matches_kinematics);
    RUN_TEST(test_correct_moves_estimate_toward_measurement_not_onto_it);
    RUN_TEST(test_repeated_correct_converges_to_measurement);
    RUN_TEST(test_correct_shrinks_covariance_trace);
    RUN_TEST(test_covariance_stays_symmetric_after_several_cycles);
    RUN_TEST(test_reset_restores_given_state_and_covariance);
    RUN_TEST(test_state_and_covariance_reflect_latest_update);
    return UNITY_END();
}