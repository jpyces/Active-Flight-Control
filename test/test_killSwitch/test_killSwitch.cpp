#include <unity.h>
#include "KillSwitch.h"

using namespace gnc;

namespace
{
    KillSwitchConfig installed(uint8_t debounce = 3)
    {
        KillSwitchConfig c{};
        c.installed = true;
        c.debounceTicks = debounce;
        return c;
    }

    bool feed(KillSwitch &ks, bool loopOpen, int n)
    {
        bool k = false;
        for (int i = 0; i < n; ++i)
        {
            k = ks.update(loopOpen);
        }
        return k;
    }
}

void setUp() {}
void tearDown() {}

void test_closed_loop_never_kills()
{
    KillSwitch ks(installed());
    TEST_ASSERT_FALSE(feed(ks, false, 1000));
}

void test_trips_after_debounce_ticks()
{
    KillSwitch ks(installed(3));
    TEST_ASSERT_FALSE(feed(ks, true, 2));
    TEST_ASSERT_TRUE(ks.update(true));
}

void test_short_dropouts_are_filtered()
{
    KillSwitch ks(installed(3));
    for (int i = 0; i < 200; ++i)
    {
        feed(ks, true, 2); // two open ticks, then contact comes back
        feed(ks, false, 1);
    }
    TEST_ASSERT_FALSE(ks.killed());
}

void test_latches_after_loop_closes_again()
{
    KillSwitch ks(installed(3));
    feed(ks, true, 3);
    TEST_ASSERT_TRUE(feed(ks, false, 1000));
}

void test_open_at_boot_kills()
{
    // Plug not inserted when powered up -> can never arm.
    KillSwitch ks(installed(3));
    TEST_ASSERT_TRUE(feed(ks, true, 3));
}

void test_not_installed_ignores_input()
{
    KillSwitchConfig c = installed(3);
    c.installed = false;
    KillSwitch ks(c);
    TEST_ASSERT_FALSE(feed(ks, true, 1000));
}

void test_zero_debounce_trips_on_first_open_reading()
{
    KillSwitch ks(installed(0));
    TEST_ASSERT_FALSE(ks.update(false));
    TEST_ASSERT_TRUE(ks.update(true));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_closed_loop_never_kills);
    RUN_TEST(test_trips_after_debounce_ticks);
    RUN_TEST(test_short_dropouts_are_filtered);
    RUN_TEST(test_latches_after_loop_closes_again);
    RUN_TEST(test_open_at_boot_kills);
    RUN_TEST(test_not_installed_ignores_input);
    RUN_TEST(test_zero_debounce_trips_on_first_open_reading);
    return UNITY_END();
}
