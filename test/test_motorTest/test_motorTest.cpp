#include <unity.h>
#include "MotorTest.h"

using namespace gnc;

namespace
{
    constexpr float kDt = 0.01f;

    MotorTestConfig makeConfig()
    {
        MotorTestConfig c{};
        c.spinCommand = 0.08f;
        c.spinTimeS = 0.1f; // ~10 ticks per motor
        return c;
    }

    int activeMotor(const Eigen::Vector4f &cmd)
    {
        int idx = -1;
        for (int i = 0; i < 4; ++i)
        {
            if (cmd(i) != 0.0f)
            {
                if (idx != -1)
                {
                    return -2; // more than one motor on
                }
                idx = i;
            }
        }
        return idx;
    }
}

void setUp() {}
void tearDown() {}

void test_idle_before_start()
{
    MotorTest mt(makeConfig());
    for (int k = 0; k < 100; ++k)
    {
        TEST_ASSERT_TRUE(mt.update(kDt).isZero());
    }
    TEST_ASSERT_TRUE(mt.result() == CheckResult::Pending);
    TEST_ASSERT_FALSE(mt.running());
}

void test_spins_each_motor_once_in_order_then_passes()
{
    MotorTest mt(makeConfig());
    mt.start();
    int ticksPerMotor[4] = {0, 0, 0, 0};
    int lastMotor = 0;
    for (int k = 0; k < 200 && mt.running(); ++k)
    {
        TEST_ASSERT_TRUE(mt.result() == CheckResult::Pending);
        const Eigen::Vector4f cmd = mt.update(kDt);
        const int m = activeMotor(cmd);
        TEST_ASSERT_TRUE(m >= 0);      // exactly one motor on
        TEST_ASSERT_TRUE(m >= lastMotor); // never goes back
        TEST_ASSERT_EQUAL_FLOAT(0.08f, cmd(m));
        ++ticksPerMotor[m];
        lastMotor = m;
    }
    for (int i = 0; i < 4; ++i)
    {
        TEST_ASSERT_INT_WITHIN(1, 10, ticksPerMotor[i]);
    }
    TEST_ASSERT_TRUE(mt.result() == CheckResult::Passed);
    TEST_ASSERT_TRUE(mt.update(kDt).isZero()); // all off afterwards
}

void test_restart_resets_to_pending_and_motor_zero()
{
    MotorTest mt(makeConfig());
    mt.start();
    for (int k = 0; k < 200; ++k)
    {
        mt.update(kDt);
    }
    TEST_ASSERT_TRUE(mt.result() == CheckResult::Passed);

    mt.start();
    TEST_ASSERT_TRUE(mt.result() == CheckResult::Pending);
    TEST_ASSERT_EQUAL_INT(0, activeMotor(mt.update(kDt)));
}

void test_zero_spin_time_still_touches_every_motor()
{
    MotorTestConfig c = makeConfig();
    c.spinTimeS = 0.0f;
    MotorTest mt(c);
    mt.start();
    for (int i = 0; i < 4; ++i)
    {
        TEST_ASSERT_EQUAL_INT(i, activeMotor(mt.update(kDt)));
    }
    TEST_ASSERT_TRUE(mt.result() == CheckResult::Passed);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_idle_before_start);
    RUN_TEST(test_spins_each_motor_once_in_order_then_passes);
    RUN_TEST(test_restart_resets_to_pending_and_motor_zero);
    RUN_TEST(test_zero_spin_time_still_touches_every_motor);
    return UNITY_END();
}
