#include <unity.h>
#include "FlightStateMachine.h"

using namespace gnc;

namespace
{
    constexpr float kDt = 0.01f;

    FlightStateMachineConfig makeConfig(bool rcRequired = false)
    {
        FlightStateMachineConfig c{};
        c.altitudeErrorTol = 1.0f;
        c.altitudeErrorTimeout = 0.5f; // ~50 ticks
        c.rcLossTimeout = 1.0f;        // ~100 ticks
        c.rcRequired = rcRequired;
        return c;
    }

    // Healthy inputs with every boot check passed and the estimator converged.
    FsmInputs healthy()
    {
        FsmInputs in{};
        in.dt = kDt;
        in.memoryCheck = CheckResult::Passed;
        in.sensorInit = CheckResult::Passed;
        in.calibrationCheck = CheckResult::Passed;
        in.estimatorConverged = true;
        in.landed = true;
        return in;
    }

    FsmOutputs stepN(FlightStateMachine &fsm, const FsmInputs &in, int n)
    {
        FsmOutputs out{};
        for (int k = 0; k < n; ++k)
        {
            out = fsm.step(in);
        }
        return out;
    }

    // On -> Mem -> Sensor -> Cal -> SEI -> Disarmed: 5 transitions.
    void bootToDisarmed(FlightStateMachine &fsm, FsmInputs &in)
    {
        stepN(fsm, in, 5);
        TEST_ASSERT_TRUE(fsm.state() == FlightState::Disarmed);
    }

    void disarmedToArmed(FlightStateMachine &fsm, FsmInputs &in)
    {
        in.armed = true;
        fsm.step(in); // rising edge -> MotorCheck
        in.motorCheck = CheckResult::Passed;
        fsm.step(in); // -> Armed
        in.motorCheck = CheckResult::Pending;
        TEST_ASSERT_TRUE(fsm.state() == FlightState::Armed);
    }

    void armedToFlight(FlightStateMachine &fsm, FsmInputs &in)
    {
        in.flightTrigger = true;
        fsm.step(in); // rising edge -> Flight (landed still true: must not end flight)
        in.landed = false;
        fsm.step(in); // airborne
        TEST_ASSERT_TRUE(fsm.state() == FlightState::Flight);
    }

    void bootToFlight(FlightStateMachine &fsm, FsmInputs &in)
    {
        bootToDisarmed(fsm, in);
        disarmedToArmed(fsm, in);
        armedToFlight(fsm, in);
    }
}

void setUp() {}
void tearDown() {}

// ============================================================ boot sequence

void test_starts_in_on_with_motors_off()
{
    FlightStateMachine fsm(makeConfig());
    TEST_ASSERT_TRUE(fsm.state() == FlightState::On);
    TEST_ASSERT_TRUE(fsm.failsafe() == Failsafe::None);
}

void test_boot_advances_one_state_per_tick()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    const FlightState expected[] = {FlightState::MemoryCheck, FlightState::SensorInit,
                                    FlightState::CalibrationCheck, FlightState::StateEstimationInit,
                                    FlightState::Disarmed};
    for (FlightState s : expected)
    {
        const FsmOutputs out = fsm.step(in);
        TEST_ASSERT_TRUE(out.state == s);
        TEST_ASSERT_FALSE(out.motorsEnabled);
    }
}

void test_boot_checks_hold_while_pending_or_failed()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in{};
    in.dt = kDt;
    fsm.step(in); // On -> MemoryCheck

    in.memoryCheck = CheckResult::Failed;
    stepN(fsm, in, 20);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::MemoryCheck);
    in.memoryCheck = CheckResult::Passed;
    fsm.step(in);

    in.sensorInit = CheckResult::Pending;
    stepN(fsm, in, 20);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::SensorInit);
    in.sensorInit = CheckResult::Failed;
    stepN(fsm, in, 20);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::SensorInit);
    in.sensorInit = CheckResult::Passed; // non-latching: late success proceeds
    fsm.step(in);

    in.calibrationCheck = CheckResult::Failed;
    stepN(fsm, in, 20);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::CalibrationCheck);
    in.calibrationCheck = CheckResult::Passed;
    fsm.step(in);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::StateEstimationInit);
}

void test_state_estimation_init_waits_for_convergence()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    in.estimatorConverged = false;
    stepN(fsm, in, 30);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::StateEstimationInit);
    in.estimatorConverged = true;
    fsm.step(in);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::Disarmed);
}

// ============================================================ arming

void test_arm_switch_held_from_boot_does_not_arm()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    in.armed = true; // on since power-up: never had a rising edge in Disarmed
    stepN(fsm, in, 30);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::Disarmed);
}

void test_arm_edge_requests_controller_reset_and_motor_check_not_estimator_reset()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToDisarmed(fsm, in);

    in.armed = true;
    const FsmOutputs out = fsm.step(in);
    TEST_ASSERT_TRUE(out.state == FlightState::MotorCheck);
    TEST_ASSERT_TRUE(out.resetController);
    TEST_ASSERT_TRUE(out.startMotorCheck);
    TEST_ASSERT_FALSE(out.resetEstimator);
    TEST_ASSERT_TRUE(out.motorsEnabled);
}

void test_request_flags_last_exactly_one_tick()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToDisarmed(fsm, in);
    in.armed = true;
    fsm.step(in);
    const FsmOutputs next = fsm.step(in);
    TEST_ASSERT_FALSE(next.resetController);
    TEST_ASSERT_FALSE(next.startMotorCheck);
}

void test_motor_check_pending_holds_failed_disarms_passed_arms()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToDisarmed(fsm, in);

    in.armed = true;
    fsm.step(in);
    stepN(fsm, in, 20); // Pending
    TEST_ASSERT_TRUE(fsm.state() == FlightState::MotorCheck);

    in.motorCheck = CheckResult::Failed;
    fsm.step(in);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::Disarmed);

    // retry needs a fresh arm edge
    in.motorCheck = CheckResult::Pending;
    stepN(fsm, in, 5);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::Disarmed);
    in.armed = false;
    fsm.step(in);
    in.armed = true;
    fsm.step(in);
    in.motorCheck = CheckResult::Passed;
    fsm.step(in);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::Armed);
}

void test_disarm_from_armed()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToDisarmed(fsm, in);
    disarmedToArmed(fsm, in);

    in.armed = false;
    const FsmOutputs out = fsm.step(in);
    TEST_ASSERT_TRUE(out.state == FlightState::Disarmed);
    TEST_ASSERT_FALSE(out.motorsEnabled);
}

void test_disarm_wins_over_takeoff_on_same_tick()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToDisarmed(fsm, in);
    disarmedToArmed(fsm, in);

    in.armed = false;
    in.flightTrigger = true;
    fsm.step(in);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::Disarmed);
}

// ============================================================ flight / landing

void test_takeoff_on_trigger_edge_and_landed_at_takeoff_does_not_end_flight()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy(); // landed = true throughout this test
    bootToDisarmed(fsm, in);
    disarmedToArmed(fsm, in);

    in.flightTrigger = true;
    fsm.step(in);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::Flight);
    stepN(fsm, in, 20); // still on the ground, landed level held
    TEST_ASSERT_TRUE(fsm.state() == FlightState::Flight);
}

void test_touchdown_returns_to_armed_and_resets_controller()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToFlight(fsm, in);

    in.landed = true;
    const FsmOutputs out = fsm.step(in);
    TEST_ASSERT_TRUE(out.state == FlightState::Armed);
    TEST_ASSERT_TRUE(out.resetController);
    TEST_ASSERT_FALSE(out.startMotorCheck); // MotorCheck does not re-run after landing
    TEST_ASSERT_TRUE(out.motorsEnabled);
}

void test_held_trigger_does_not_relaunch_after_landing()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToFlight(fsm, in); // flightTrigger left true

    in.landed = true;
    fsm.step(in);
    stepN(fsm, in, 20);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::Armed);

    in.flightTrigger = false;
    fsm.step(in);
    in.flightTrigger = true;
    fsm.step(in);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::Flight);
}

void test_disarm_in_flight_ignored_in_air_honoured_on_ground()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToFlight(fsm, in); // airborne: landed = false

    in.armed = false;
    stepN(fsm, in, 10);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::Flight);

    // failed-takeoff abort: on the ground in Flight, operator disarms
    FlightStateMachine fsm2(makeConfig());
    FsmInputs in2 = healthy();
    bootToDisarmed(fsm2, in2);
    disarmedToArmed(fsm2, in2);
    in2.flightTrigger = true;
    fsm2.step(in2); // Flight, never lifted off (landed stays true)
    in2.armed = false;
    fsm2.step(in2);
    TEST_ASSERT_TRUE(fsm2.state() == FlightState::Disarmed);
}

// ============================================================ failsafes: gating

void test_sensor_failure_ignored_during_boot()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    in.gyroStatus = SensorStatus::FAILED;
    in.estimatorConverged = false;
    stepN(fsm, in, 10); // parks in StateEstimationInit
    TEST_ASSERT_TRUE(fsm.state() == FlightState::StateEstimationInit);
    TEST_ASSERT_TRUE(fsm.failsafe() == Failsafe::None);
}

void test_sensor_loss_in_disarmed_blocks_arming()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToDisarmed(fsm, in);

    in.accelStatus = SensorStatus::FAILED;
    fsm.step(in);
    TEST_ASSERT_TRUE(fsm.failsafe() == Failsafe::SensorLoss);

    in.armed = true; // edge arrives while blocked -> consumed
    stepN(fsm, in, 5);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::Disarmed);

    in.accelStatus = SensorStatus::NOMINAL;
    stepN(fsm, in, 5);
    TEST_ASSERT_TRUE(fsm.failsafe() == Failsafe::None);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::Disarmed); // no late arm
}

void test_altitude_failsafe_only_in_flight()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToDisarmed(fsm, in);
    disarmedToArmed(fsm, in);

    in.altitude = 10.0f; // huge error, but on the ground
    stepN(fsm, in, 200);
    TEST_ASSERT_TRUE(fsm.failsafe() == Failsafe::None);
}

void test_altitude_failsafe_needs_dwell_and_timer_resets()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToFlight(fsm, in);

    in.altitude = 5.0f; // error 5 m > tol 1 m
    stepN(fsm, in, 40); // 0.4 s < 0.5 s timeout
    TEST_ASSERT_TRUE(fsm.failsafe() == Failsafe::None);

    in.altitude = 0.0f; // error clears -> timer resets
    fsm.step(in);
    in.altitude = 5.0f;
    stepN(fsm, in, 40); // another 0.4 s: would trip if the timer had carried over
    TEST_ASSERT_TRUE(fsm.failsafe() == Failsafe::None);

    stepN(fsm, in, 20); // now 0.6 s continuous
    TEST_ASSERT_TRUE(fsm.failsafe() == Failsafe::Altitude);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::Flight);
    TEST_ASSERT_TRUE(fsm.step(in).motorsEnabled); // descending, not killed

    in.altitude = 0.0f;
    fsm.step(in);
    TEST_ASSERT_TRUE(fsm.failsafe() == Failsafe::None);
}

void test_rc_loss_inert_when_not_required()
{
    FlightStateMachine fsm(makeConfig(false));
    FsmInputs in = healthy();
    bootToFlight(fsm, in);
    in.rcValid = false;
    stepN(fsm, in, 500);
    TEST_ASSERT_TRUE(fsm.failsafe() == Failsafe::None);
}

void test_rc_loss_trips_after_timeout_when_required()
{
    FlightStateMachine fsm(makeConfig(true));
    FsmInputs in = healthy();
    bootToFlight(fsm, in);

    in.rcValid = false;
    stepN(fsm, in, 90); // 0.9 s
    TEST_ASSERT_TRUE(fsm.failsafe() == Failsafe::None);
    stepN(fsm, in, 20); // 1.1 s
    TEST_ASSERT_TRUE(fsm.failsafe() == Failsafe::RcLoss);
}

// ============================================================ failsafes: priority

void test_sensor_loss_kills_motors_in_flight()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToFlight(fsm, in);

    in.gyroStatus = SensorStatus::FAILED;
    const FsmOutputs out = fsm.step(in);
    TEST_ASSERT_TRUE(out.failsafe == Failsafe::SensorLoss);
    TEST_ASSERT_TRUE(out.state == FlightState::Flight);
    TEST_ASSERT_FALSE(out.motorsEnabled);
}

void test_escalates_and_deescalates_by_priority()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToFlight(fsm, in);

    in.estimatorDegraded = true;
    TEST_ASSERT_TRUE(fsm.step(in).failsafe == Failsafe::AhrsDegraded);

    in.gyroStatus = SensorStatus::FAILED; // escalate
    TEST_ASSERT_TRUE(fsm.step(in).failsafe == Failsafe::SensorLoss);

    in.gyroStatus = SensorStatus::NOMINAL; // de-escalate to next-highest, not None
    TEST_ASSERT_TRUE(fsm.step(in).failsafe == Failsafe::AhrsDegraded);

    in.estimatorDegraded = false;
    const FsmOutputs out = fsm.step(in);
    TEST_ASSERT_TRUE(out.failsafe == Failsafe::None);
    TEST_ASSERT_TRUE(out.state == FlightState::Flight); // never left the mission state
}

void test_ahrs_degraded_in_air_does_not_reset_estimator()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToFlight(fsm, in);

    in.estimatorDegraded = true;
    const FsmOutputs out = stepN(fsm, in, 10);
    TEST_ASSERT_FALSE(out.resetEstimator); // stays on comp filter through the descent
    TEST_ASSERT_TRUE(out.motorsEnabled);
}

// ============================================================ failsafes: to the ground

void test_failsafe_while_armed_on_ground_disarms_immediately()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToDisarmed(fsm, in);
    disarmedToArmed(fsm, in);

    in.estimatorDegraded = true;
    const FsmOutputs out = fsm.step(in);
    TEST_ASSERT_TRUE(out.state == FlightState::Disarmed);
    TEST_ASSERT_TRUE(out.resetEstimator); // ground resync
    TEST_ASSERT_FALSE(out.motorsEnabled);
}

void test_failsafe_descent_touchdown_disarms_not_armed()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToFlight(fsm, in);

    in.estimatorDegraded = true;
    stepN(fsm, in, 10); // descending
    in.landed = true;
    const FsmOutputs out = fsm.step(in);
    TEST_ASSERT_TRUE(out.state == FlightState::Disarmed);
    TEST_ASSERT_FALSE(out.motorsEnabled);
}

void test_ground_ahrs_resync_breaks_the_deadlock()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToDisarmed(fsm, in);

    in.estimatorDegraded = true;
    TEST_ASSERT_TRUE(fsm.step(in).resetEstimator);
    in.estimatorDegraded = false; // main performed the reset
    TEST_ASSERT_TRUE(fsm.step(in).failsafe == Failsafe::None);
}

void test_no_rearm_after_failsafe_disarm_until_switch_toggled()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToFlight(fsm, in);

    in.estimatorDegraded = true;
    stepN(fsm, in, 5);
    in.landed = true;
    fsm.step(in); // -> Disarmed
    in.estimatorDegraded = false;
    stepN(fsm, in, 20); // failsafe clears; arm switch still on
    TEST_ASSERT_TRUE(fsm.state() == FlightState::Disarmed);

    in.armed = false;
    fsm.step(in);
    in.armed = true;
    fsm.step(in);
    TEST_ASSERT_TRUE(fsm.state() == FlightState::MotorCheck);
}

void test_altitude_failsafe_clears_after_touchdown_disarm()
{
    FlightStateMachine fsm(makeConfig());
    FsmInputs in = healthy();
    bootToFlight(fsm, in);

    in.altitude = 5.0f;
    stepN(fsm, in, 60);
    TEST_ASSERT_TRUE(fsm.failsafe() == Failsafe::Altitude);
    in.landed = true;
    fsm.step(in);                        // -> Disarmed
    const FsmOutputs out = fsm.step(in); // altitude check no longer applies
    TEST_ASSERT_TRUE(out.state == FlightState::Disarmed);
    TEST_ASSERT_TRUE(out.failsafe == Failsafe::None);
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_starts_in_on_with_motors_off);
    RUN_TEST(test_boot_advances_one_state_per_tick);
    RUN_TEST(test_boot_checks_hold_while_pending_or_failed);
    RUN_TEST(test_state_estimation_init_waits_for_convergence);
    RUN_TEST(test_arm_switch_held_from_boot_does_not_arm);
    RUN_TEST(test_arm_edge_requests_controller_reset_and_motor_check_not_estimator_reset);
    RUN_TEST(test_request_flags_last_exactly_one_tick);
    RUN_TEST(test_motor_check_pending_holds_failed_disarms_passed_arms);
    RUN_TEST(test_disarm_from_armed);
    RUN_TEST(test_disarm_wins_over_takeoff_on_same_tick);
    RUN_TEST(test_takeoff_on_trigger_edge_and_landed_at_takeoff_does_not_end_flight);
    RUN_TEST(test_touchdown_returns_to_armed_and_resets_controller);
    RUN_TEST(test_held_trigger_does_not_relaunch_after_landing);
    RUN_TEST(test_disarm_in_flight_ignored_in_air_honoured_on_ground);
    RUN_TEST(test_sensor_failure_ignored_during_boot);
    RUN_TEST(test_sensor_loss_in_disarmed_blocks_arming);
    RUN_TEST(test_altitude_failsafe_only_in_flight);
    RUN_TEST(test_altitude_failsafe_needs_dwell_and_timer_resets);
    RUN_TEST(test_rc_loss_inert_when_not_required);
    RUN_TEST(test_rc_loss_trips_after_timeout_when_required);
    RUN_TEST(test_sensor_loss_kills_motors_in_flight);
    RUN_TEST(test_escalates_and_deescalates_by_priority);
    RUN_TEST(test_ahrs_degraded_in_air_does_not_reset_estimator);
    RUN_TEST(test_failsafe_while_armed_on_ground_disarms_immediately);
    RUN_TEST(test_failsafe_descent_touchdown_disarms_not_armed);
    RUN_TEST(test_ground_ahrs_resync_breaks_the_deadlock);
    RUN_TEST(test_no_rearm_after_failsafe_disarm_until_switch_toggled);
    RUN_TEST(test_altitude_failsafe_clears_after_touchdown_disarm);
    return UNITY_END();
}