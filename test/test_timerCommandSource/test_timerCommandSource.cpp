#include <unity.h>
#include "TimerCommandSource.h"
#include "FlightStateMachine.h"

using namespace gnc;

namespace
{
    constexpr float kDt = 0.01f;

    TimerCommandConfig makeConfig()
    {
        TimerCommandConfig c{};
        c.armDelayS = 0.5f;       // ~50 ticks
        c.takeoffDelayS = 0.3f;   // ~30 ticks
        c.flightDurationS = 1.0f; // ~100 ticks
        c.hoverAltitudeM = 1.0f;
        c.climbRateMps = 2.0f;    // 0.02 m per tick
        c.descentRateMps = 1.0f;  // 0.01 m per tick
        c.landAltitudeM = -0.2f;
        c.yawSetpointRad = 0.25f;
        return c;
    }

    Commands stepN(TimerCommandSource &src, FlightState s, int n, bool landed = true)
    {
        Commands cmd{};
        for (int k = 0; k < n; ++k)
        {
            cmd = src.update(s, landed, kDt);
        }
        return cmd;
    }

    // Feeds state s until the source first raises the chosen command, the way
    // the real FSM would leave s on the next tick. Returns ticks taken (1-based).
    int stepUntil(TimerCommandSource &src, FlightState s, bool Commands::*field)
    {
        for (int k = 1; k <= 1000; ++k)
        {
            if (src.update(s, true, kDt).*field)
            {
                return k;
            }
        }
        return -1;
    }

    // Drives the source to WaitTakeoff (armed, idling in Armed).
    void reachWaitTakeoff(TimerCommandSource &src)
    {
        stepUntil(src, FlightState::Disarmed, &Commands::armed);
        src.update(FlightState::MotorCheck, true, kDt);
        src.update(FlightState::Armed, true, kDt);
    }

    // Drives the source to Flying with takeoff just commanded.
    void reachFlying(TimerCommandSource &src)
    {
        reachWaitTakeoff(src);
        stepUntil(src, FlightState::Armed, &Commands::flightTrigger);
    }
}

void setUp() {}
void tearDown() {}

void test_idle_during_boot()
{
    TimerCommandSource src(makeConfig());
    const FlightState boot[] = {FlightState::On, FlightState::MemoryCheck, FlightState::SensorInit,
                                FlightState::CalibrationCheck, FlightState::StateEstimationInit};
    for (FlightState s : boot)
    {
        const Commands cmd = stepN(src, s, 200); // far longer than armDelay
        TEST_ASSERT_FALSE(cmd.armed);
        TEST_ASSERT_FALSE(cmd.flightTrigger);
        TEST_ASSERT_TRUE(cmd.rcValid);
    }
    TEST_ASSERT_EQUAL(TimerPhase::WaitArm, src.phase());
}

void test_arms_after_delay_in_disarmed()
{
    TimerCommandSource src(makeConfig());
    const int ticks = stepUntil(src, FlightState::Disarmed, &Commands::armed);
    TEST_ASSERT_INT_WITHIN(1, 50, ticks); // 0.5 s at 100 Hz, +-1 for float accumulation
    TEST_ASSERT_EQUAL(TimerPhase::Arming, src.phase());
}

void test_motor_check_to_armed_holds_armed_without_takeoff()
{
    TimerCommandSource src(makeConfig());
    stepUntil(src, FlightState::Disarmed, &Commands::armed);
    Commands cmd = stepN(src, FlightState::MotorCheck, 20);
    TEST_ASSERT_TRUE(cmd.armed);
    TEST_ASSERT_EQUAL(TimerPhase::Arming, src.phase());

    cmd = stepN(src, FlightState::Armed, 25); // shorter than takeoffDelay
    TEST_ASSERT_TRUE(cmd.armed);
    TEST_ASSERT_FALSE(cmd.flightTrigger);
    TEST_ASSERT_EQUAL(TimerPhase::WaitTakeoff, src.phase());
}

void test_failed_motor_check_ends_mission_without_rearm()
{
    TimerCommandSource src(makeConfig());
    stepUntil(src, FlightState::Disarmed, &Commands::armed);
    src.update(FlightState::MotorCheck, true, kDt);
    const Commands cmd = stepN(src, FlightState::Disarmed, 500); // FSM fell back
    TEST_ASSERT_EQUAL(TimerPhase::Done, src.phase());
    TEST_ASSERT_FALSE(cmd.armed);
}

void test_failsafe_disarm_while_waiting_for_takeoff_ends_mission()
{
    TimerCommandSource src(makeConfig());
    reachWaitTakeoff(src);
    const Commands cmd = src.update(FlightState::Disarmed, true, kDt);
    TEST_ASSERT_EQUAL(TimerPhase::Done, src.phase());
    TEST_ASSERT_FALSE(cmd.armed);
}

void test_takeoff_after_delay_and_altitude_ramps_to_hover()
{
    TimerCommandSource src(makeConfig());
    reachWaitTakeoff(src);
    TEST_ASSERT_INT_WITHIN(1, 30, stepUntil(src, FlightState::Armed, &Commands::flightTrigger));
    Commands cmd = src.update(FlightState::Flight, false, kDt);
    TEST_ASSERT_TRUE(cmd.flightTrigger);
    TEST_ASSERT_TRUE(cmd.armed);
    TEST_ASSERT_EQUAL(TimerPhase::Flying, src.phase());
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.02f, cmd.setpoints.altitude); // first ramp step from ground

    cmd = stepN(src, FlightState::Flight, 9, false);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.2f, cmd.setpoints.altitude); // 10 x 0.02 m

    float peak = cmd.setpoints.altitude;
    for (int k = 0; k < 60; ++k) // past the 50-tick climb
    {
        cmd = src.update(FlightState::Flight, false, kDt);
        peak = cmd.setpoints.altitude > peak ? cmd.setpoints.altitude : peak;
    }
    TEST_ASSERT_TRUE(peak <= 1.0f);                        // never overshoots
    TEST_ASSERT_EQUAL_FLOAT(1.0f, cmd.setpoints.altitude); // settles on hover
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, cmd.setpoints.angle(0));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, cmd.setpoints.angle(1));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.25f, cmd.setpoints.angle(2));
}

void test_blocked_takeoff_ends_mission()
{
    TimerCommandSource src(makeConfig());
    reachFlying(src);
    // FSM never left Armed (e.g. a failsafe consumed the takeoff edge).
    const Commands cmd = src.update(FlightState::Armed, true, kDt);
    TEST_ASSERT_EQUAL(TimerPhase::Done, src.phase());
    TEST_ASSERT_FALSE(cmd.armed);
    TEST_ASSERT_FALSE(cmd.flightTrigger);
}

void test_descends_after_flight_duration_then_disarms_on_touchdown()
{
    TimerCommandSource src(makeConfig());
    reachFlying(src);
    stepN(src, FlightState::Flight, 95, false);
    TEST_ASSERT_EQUAL(TimerPhase::Flying, src.phase());
    stepN(src, FlightState::Flight, 10, false);
    TEST_ASSERT_EQUAL(TimerPhase::Landing, src.phase());

    Commands cmd = stepN(src, FlightState::Flight, 20, false);
    TEST_ASSERT_TRUE(cmd.armed);
    TEST_ASSERT_TRUE(cmd.setpoints.altitude < 1.0f); // descending

    cmd = stepN(src, FlightState::Flight, 300, false); // long enough to finish the ramp
    TEST_ASSERT_EQUAL_FLOAT(-0.2f, cmd.setpoints.altitude);

    cmd = src.update(FlightState::Armed, true, kDt); // FSM saw touchdown
    TEST_ASSERT_EQUAL(TimerPhase::Done, src.phase());
    TEST_ASSERT_FALSE(cmd.armed);
}

void test_done_is_permanent()
{
    TimerCommandSource src(makeConfig());
    reachWaitTakeoff(src);
    src.update(FlightState::Disarmed, true, kDt);
    const FlightState states[] = {FlightState::Disarmed, FlightState::Armed, FlightState::Flight};
    for (FlightState s : states)
    {
        const Commands cmd = stepN(src, s, 300);
        TEST_ASSERT_FALSE(cmd.armed);
        TEST_ASSERT_FALSE(cmd.flightTrigger);
        TEST_ASSERT_EQUAL(TimerPhase::Done, src.phase());
    }
}

// Closed-loop harness: the timer source drives the real FSM with FlightCore's
// one-tick state lag. degradedAt(k, state) injects estimatorDegraded (the
// AhrsDegraded failsafe) on the tick it returns true.
namespace
{
    struct MissionResult
    {
        bool reachedFlight{false};
        bool sawFailsafeInFlight{false};
        bool sawRearm{false};
        FlightState finalState{FlightState::On};
        Failsafe finalFailsafe{Failsafe::None};
        TimerPhase finalPhase{TimerPhase::WaitArm};
    };

    template <typename DegradedFn>
    MissionResult runMission(DegradedFn degradedAt)
    {
        FlightStateMachineConfig fc{};
        fc.altitudeErrorTol = 100.0f; // no altitude plant here; keep that failsafe out
        fc.altitudeErrorTimeout = 1.0f;
        fc.rcLossTimeout = 1.0f;
        fc.rcRequired = true;
        FlightStateMachine fsm(fc);
        TimerCommandSource src(makeConfig());

        MissionResult r{};
        FlightState lastState = FlightState::On;
        bool lastLanded = true;
        CheckResult motorCheck = CheckResult::Pending;
        bool doneSeen = false;

        for (int k = 0; k < 3000; ++k)
        {
            const Commands cmd = src.update(lastState, lastLanded, kDt);

            FsmInputs in{};
            in.dt = kDt;
            in.memoryCheck = CheckResult::Passed;
            in.sensorInit = CheckResult::Passed;
            in.calibrationCheck = CheckResult::Passed;
            in.estimatorConverged = true;
            in.estimatorDegraded = degradedAt(k, lastState);
            in.motorCheck = motorCheck;
            in.armed = cmd.armed;
            in.flightTrigger = cmd.flightTrigger;
            in.rcValid = cmd.rcValid;
            in.altitudeSetpoint = cmd.setpoints.altitude;
            // Crude touchdown: airborne while the altitude setpoint is above 0.1 m.
            // During a failsafe, stand in for the level descent by landing at once.
            const bool descending = lastState == FlightState::Flight && fsm.failsafe() != Failsafe::None;
            in.landed = descending || !(lastState == FlightState::Flight && cmd.setpoints.altitude > 0.1f);
            in.altitude = in.landed ? 0.0f : cmd.setpoints.altitude;

            const FsmOutputs out = fsm.step(in);

            // Stand-in motor test: passes one tick after it starts.
            if (out.startMotorCheck)
            {
                motorCheck = CheckResult::Pending;
            }
            else if (out.state == FlightState::MotorCheck)
            {
                motorCheck = CheckResult::Passed;
            }

            r.reachedFlight = r.reachedFlight || out.state == FlightState::Flight;
            r.sawFailsafeInFlight = r.sawFailsafeInFlight ||
                                    (out.state == FlightState::Flight && out.failsafe != Failsafe::None);
            r.sawRearm = r.sawRearm || (doneSeen && cmd.armed);
            doneSeen = doneSeen || src.phase() == TimerPhase::Done;

            lastState = out.state;
            lastLanded = in.landed;
            r.finalFailsafe = out.failsafe;
        }
        r.finalState = lastState;
        r.finalPhase = src.phase();
        return r;
    }
}

// Nominal mission runs once and ends Disarmed, never re-arming.
void test_full_mission_with_fsm()
{
    const MissionResult r = runMission([](int, FlightState) { return false; });
    TEST_ASSERT_TRUE(r.reachedFlight);
    TEST_ASSERT_FALSE(r.sawFailsafeInFlight);
    TEST_ASSERT_EQUAL(TimerPhase::Done, r.finalPhase);
    TEST_ASSERT_EQUAL(FlightState::Disarmed, r.finalState);
    TEST_ASSERT_EQUAL(Failsafe::None, r.finalFailsafe);
    TEST_ASSERT_FALSE(r.sawRearm);
}

// AhrsDegraded in the air: state stays Flight (failsafe is a property, not a
// state), touchdown under failsafe goes straight to Disarmed, mission ends.
void test_failsafe_in_flight_ends_mission_disarmed()
{
    int flightTicks = 0;
    const MissionResult r = runMission([&](int, FlightState s) {
        if (s == FlightState::Flight)
        {
            ++flightTicks;
        }
        return flightTicks > 20; // latched once tripped, like the real estimator
    });
    TEST_ASSERT_TRUE(r.reachedFlight);
    TEST_ASSERT_TRUE(r.sawFailsafeInFlight);
    TEST_ASSERT_EQUAL(TimerPhase::Done, r.finalPhase);
    TEST_ASSERT_EQUAL(FlightState::Disarmed, r.finalState);
    TEST_ASSERT_FALSE(r.sawRearm);
}

// Failsafe active on the ground when the arm command goes out: the FSM
// consumes the edge and stays Disarmed; the source gives up instead of retrying.
void test_failsafe_on_ground_at_arm_ends_mission()
{
    const MissionResult r = runMission([](int k, FlightState) { return k >= 30 && k < 200; });
    TEST_ASSERT_FALSE(r.reachedFlight);
    TEST_ASSERT_EQUAL(TimerPhase::Done, r.finalPhase);
    TEST_ASSERT_EQUAL(FlightState::Disarmed, r.finalState);
    TEST_ASSERT_FALSE(r.sawRearm);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_idle_during_boot);
    RUN_TEST(test_arms_after_delay_in_disarmed);
    RUN_TEST(test_motor_check_to_armed_holds_armed_without_takeoff);
    RUN_TEST(test_failed_motor_check_ends_mission_without_rearm);
    RUN_TEST(test_failsafe_disarm_while_waiting_for_takeoff_ends_mission);
    RUN_TEST(test_takeoff_after_delay_and_altitude_ramps_to_hover);
    RUN_TEST(test_blocked_takeoff_ends_mission);
    RUN_TEST(test_descends_after_flight_duration_then_disarms_on_touchdown);
    RUN_TEST(test_done_is_permanent);
    RUN_TEST(test_full_mission_with_fsm);
    RUN_TEST(test_failsafe_in_flight_ends_mission_disarmed);
    RUN_TEST(test_failsafe_on_ground_at_arm_ends_mission);
    return UNITY_END();
}
