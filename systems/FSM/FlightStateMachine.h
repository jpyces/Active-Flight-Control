#pragma once
#include "FlightState.h"
#include "SensorStatus.h"

namespace gnc
{
    // Result of any externally-run check. The module running the check owns its
    // own timeout and reports Failed itself.
    enum class CheckResult
    {
        Pending, // still running (or not started)
        Passed,
        Failed
    };

    // Active failsafe, independent of the mission state. Listed in priority order
    // for readability only -- priority is enforced in step(), not by value.
    enum class Failsafe
    {
        None,
        SensorLoss,   // gyro/accel FAILED -> motors killed
        AhrsDegraded, // Madgwick fell back to the complementary filter -> descend
        Altitude,     // altitude tracking error past dwell -> descend
        RcLoss        // no valid RC past timeout -> descend
    };

    // Chosen before flight; constant for the lifetime of the FSM.
    struct FlightStateMachineConfig
    {
        float altitudeErrorTol;     // m, |altitude - setpoint| trip threshold
        float altitudeErrorTimeout; // s, error must persist this long to trip
        float rcLossTimeout;        // s, time since last valid RC before tripping
        bool rcRequired;            // false makes the RC-loss failsafe fully inert
    };

    // Everything the FSM reads, once per tick. Defaults describe a healthy, idle
    // system so tests only need to set what they're exercising.
    struct FsmInputs
    {
        float dt{0.0f};

        // checks (owned and timed by their own modules)
        CheckResult memoryCheck{CheckResult::Pending};
        CheckResult sensorInit{CheckResult::Pending};
        CheckResult calibrationCheck{CheckResult::Pending};
        CheckResult motorCheck{CheckResult::Pending};

        // Operator / external signals. armed, flightTrigger and landed act on their
        // RISING EDGE, so a signal left high can't re-arm, relaunch, or end a flight
        // by itself. armed is also read as a level to disarm.
        bool armed{false};         // true = operator wants armed, false = disarm
        bool flightTrigger{false}; // takeoff command
        bool landed{false};        // touchdown detector: "on the ground right now"

        // from StateEstimator
        bool estimatorConverged{false}; // StateEstimator::isConverged()
        bool estimatorDegraded{false};  // StateEstimator::isDegraded()

        // failsafe detector inputs
        SensorStatus accelStatus{SensorStatus::NOMINAL};
        SensorStatus gyroStatus{SensorStatus::NOMINAL};
        float altitude{0.0f};
        float altitudeSetpoint{0.0f};
        bool rcValid{true};
    };

    // Everything the FSM asks main.cpp to do. The three request flags are one-tick
    // edges: false unless step() sets them on this tick.
    struct FsmOutputs
    {
        FlightState state{FlightState::On};
        Failsafe failsafe{Failsafe::None};
        bool motorsEnabled{false};
        bool resetEstimator{false};  // main: estimator.reset()
        bool resetController{false}; // main: controller.reset()
        bool startMotorCheck{false}; // main: reset ESC test to Pending and start it
    };

    // Pure-logic flight state machine: owns no other objects. main.cpp builds
    // FsmInputs, calls step(), and acts on FsmOutputs.
    class FlightStateMachine
    {
    public:
        explicit FlightStateMachine(const FlightStateMachineConfig &cfg);

        // One tick: evaluate failsafes, then advance the mission sequence only if
        // no failsafe is active.
        FsmOutputs step(const FsmInputs &in);

        FlightState state() const { return m_state; }
        Failsafe failsafe() const { return m_failsafe; }

    private:
        friend struct FlightStateMachineTestAccess; // test-only write access

        // Dwell detectors. Each resets its own timer whenever it isn't applicable,
        // so a stale count can't carry over (e.g. from the ground into flight).
        bool altitudeErrorExceeded(const FsmInputs &in, bool applicable);
        bool rcLinkLost(const FsmInputs &in, bool applicable);

        void advanceSequence(const FsmInputs &in, FsmOutputs &out);

        FlightStateMachineConfig m_cfg;

        FlightState m_state{FlightState::On}; // mission state
        Failsafe m_failsafe{Failsafe::None};  // recomputed from scratch every tick

        // previous-tick values, for rising-edge detection
        bool m_armedPrev{false};
        bool m_flightTriggerPrev{false};
        bool m_landedPrev{false};

        float m_altitudeErrorElapsed{0.0f};
        float m_rcElapsedSinceValid{0.0f};
    };
}