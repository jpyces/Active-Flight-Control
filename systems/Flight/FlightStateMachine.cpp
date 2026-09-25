#include "FlightStateMachine.h"

#include <cmath>

namespace gnc
{
    FlightStateMachine::FlightStateMachine(const FlightStateMachineConfig &cfg)
        : m_cfg(cfg)
    {
    }

    FsmOutputs FlightStateMachine::step(const FsmInputs &in)
    {
        FsmOutputs out{};

        const bool touchdown = in.landed && !m_landedPrev;

        // -- 1. Which failsafes apply depends on the mission state directly.
        const bool inAir = m_state == FlightState::Flight;
        const bool onGroundLive =
            m_state == FlightState::Disarmed ||
            m_state == FlightState::MotorCheck ||
            m_state == FlightState::Armed;
        const bool live = onGroundLive || inAir; // estimation converged, sensors meaningful

        const bool sensorTripped =
            live && (in.accelStatus == SensorStatus::FAILED ||
                     in.gyroStatus == SensorStatus::FAILED);

        const bool ahrsTripped = live && in.estimatorDegraded;
        const bool altTripped = altitudeErrorExceeded(in, inAir);
        const bool rcTripped = rcLinkLost(in, m_state == FlightState::Armed || inAir);

        // -- 2. Highest priority wins, recomputed every tick. Escalation,
        // de-escalation and recovery all fall out of this.
        if (sensorTripped)
        {
            m_failsafe = Failsafe::SensorLoss;
        }
        else if (ahrsTripped)
        {
            m_failsafe = Failsafe::AhrsDegraded;
        }
        else if (altTripped)
        {
            m_failsafe = Failsafe::Altitude;
        }
        else if (rcTripped)
        {
            m_failsafe = Failsafe::RcLoss;
        }
        else
        {
            m_failsafe = Failsafe::None;
        }

        // -- 3. AHRS fallback on the ground: resync Madgwick immediately. Nothing
        // else can clear isDegraded(), so waiting for it to clear would deadlock.
        // In the air, stay on the complementary filter through the descent.
        if (ahrsTripped && onGroundLive)
        {
            out.resetEstimator = true;
        }

        // -- 4. Mission sequence. Normal transitions only when nothing is wrong;
        // with a failsafe active, the only way forward is to the ground, disarmed.
        if (m_failsafe == Failsafe::None)
        {
            advanceSequence(in, out);
        }
        else
        {
            const bool onGroundWithMotorsLive =
                m_state == FlightState::MotorCheck ||
                m_state == FlightState::Armed;
            if (onGroundWithMotorsLive || (inAir && touchdown))
            {
                m_state = FlightState::Disarmed;
            }
        }

        // Always updated, even during a failsafe: an edge that arrives while a
        // failsafe blocks it is consumed, so it can't fire late once it clears.
        m_armedPrev = in.armed;
        m_flightTriggerPrev = in.flightTrigger;
        m_landedPrev = in.landed;

        // -- 5. Outputs. Mission state decides motors; SensorLoss kills them.
        out.state = m_state;
        out.failsafe = m_failsafe;

        switch (m_state)
        {
        case FlightState::MotorCheck:
        case FlightState::Armed:
        case FlightState::Flight:
            out.motorsEnabled = true;
            break;
        default:
            out.motorsEnabled = false; // anything not listed, incl. future states
            break;
        }
        if (m_failsafe == Failsafe::SensorLoss)
        {
            out.motorsEnabled = false; // no attitude reference, no RC -> kill
        }

        return out;
    }

    void FlightStateMachine::advanceSequence(const FsmInputs &in, FsmOutputs &out)
    {
        const bool armRequested = in.armed && !m_armedPrev;
        const bool takeoffRequested = in.flightTrigger && !m_flightTriggerPrev;
        const bool touchdown = in.landed && !m_landedPrev;

        switch (m_state)
        {
        case FlightState::On:
            m_state = FlightState::MemoryCheck;
            break;

        // Checks: Failed stays put for now (non-latching; motors are off).
        // A shared BootFailed sink can replace this later.
        case FlightState::MemoryCheck:
            if (in.memoryCheck == CheckResult::Passed)
            {
                m_state = FlightState::SensorInit;
            }
            break;

        case FlightState::SensorInit:
            if (in.sensorInit == CheckResult::Passed)
            {
                m_state = FlightState::CalibrationCheck;
            }
            break;

        case FlightState::CalibrationCheck:
            if (in.calibrationCheck == CheckResult::Passed)
            {
                m_state = FlightState::StateEstimationInit;
            }
            break;

        case FlightState::StateEstimationInit:
            if (in.estimatorConverged)
            {
                m_state = FlightState::Disarmed;
            }
            break;

        case FlightState::Disarmed:
            if (armRequested)
            {
                // Controller only: the estimator has been running continuously and
                // resetting it here would discard Madgwick's mag-locked heading.
                out.resetController = true;
                out.startMotorCheck = true;
                m_state = FlightState::MotorCheck;
            }
            break;

        case FlightState::MotorCheck:
            switch (in.motorCheck)
            {
            case CheckResult::Passed:
                m_state = FlightState::Armed;
                break;
            case CheckResult::Failed:
                m_state = FlightState::Disarmed; // motors off, retry by re-arming
                break;
            case CheckResult::Pending:
                break;
            }
            break;

        // Landed and ready. Motor health is monitored in the background (motor-loss
        // failsafe, later), so MotorCheck does not re-run after a landing.
        case FlightState::Armed:
            if (!in.armed)
            {
                m_state = FlightState::Disarmed;
            }
            else if (takeoffRequested)
            {
                m_state = FlightState::Flight;
            }
            break;

        case FlightState::Flight:
            if (!in.armed && in.landed)
            {
                // Operator abort while still on the ground (e.g. failed takeoff).
                // Disarming in the air is deliberately ignored.
                m_state = FlightState::Disarmed;
            }
            else if (touchdown)
            {
                // Integrators wound up against ground contact.
                out.resetController = true;
                m_state = FlightState::Armed;
            }
            break;
        }
    }

    bool FlightStateMachine::altitudeErrorExceeded(const FsmInputs &in, bool applicable)
    {
        if (!applicable || std::fabs(in.altitude - in.altitudeSetpoint) <= m_cfg.altitudeErrorTol)
        {
            m_altitudeErrorElapsed = 0.0f;
            return false;
        }
        m_altitudeErrorElapsed += in.dt;
        return m_altitudeErrorElapsed > m_cfg.altitudeErrorTimeout;
    }

    bool FlightStateMachine::rcLinkLost(const FsmInputs &in, bool applicable)
    {
        if (!m_cfg.rcRequired || !applicable || in.rcValid)
        {
            m_rcElapsedSinceValid = 0.0f;
            return false;
        }
        m_rcElapsedSinceValid += in.dt;
        return m_rcElapsedSinceValid > m_cfg.rcLossTimeout;
    }
}