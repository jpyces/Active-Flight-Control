#include "TimerCommandSource.h"

namespace gnc
{
    TimerCommandSource::TimerCommandSource(const TimerCommandConfig &cfg)
        : m_cfg(cfg)
    {
    }

    Commands TimerCommandSource::update(FlightState state, bool /*landed*/, float dt)
    {
        // state is the FSM output from the previous tick, so each phase checks
        // for the state its own command should have produced, one tick late.
        // Sequencing uses the FSM state only; touchdown is already folded into
        // it (Flight -> Armed), so landed is not needed here.
        switch (m_phase)
        {
        case TimerPhase::WaitArm:
            // Count only once boot has finished, so the delay is real idle time.
            if (state == FlightState::Disarmed)
            {
                m_elapsed += dt;
                if (m_elapsed >= m_cfg.armDelayS)
                {
                    enter(TimerPhase::Arming);
                }
            }
            break;

        case TimerPhase::Arming:
            if (state == FlightState::Armed)
            {
                enter(TimerPhase::WaitTakeoff);
            }
            else if (state != FlightState::MotorCheck)
            {
                // Motor check failed, a failsafe blocked the arm edge, or boot
                // regressed. Retrying automatically is not safe.
                enter(TimerPhase::Done);
            }
            break;

        case TimerPhase::WaitTakeoff:
            if (state != FlightState::Armed)
            {
                enter(TimerPhase::Done); // failsafe disarm on the ground
            }
            else
            {
                m_elapsed += dt;
                if (m_elapsed >= m_cfg.takeoffDelayS)
                {
                    m_altitudeSetpoint = 0.0f; // climb starts from the ground
                    enter(TimerPhase::Flying);
                }
            }
            break;

        case TimerPhase::Flying:
            // Takeoff was commanded on the call that entered Flying, so the FSM
            // has already stepped once with it; anything but Flight means it failed.
            if (state != FlightState::Flight)
            {
                // Takeoff edge blocked by a failsafe, early touchdown, or disarm.
                enter(TimerPhase::Done);
            }
            else
            {
                rampAltitude(m_cfg.hoverAltitudeM, m_cfg.climbRateMps, dt);
                m_elapsed += dt;
                if (m_elapsed >= m_cfg.flightDurationS)
                {
                    enter(TimerPhase::Landing);
                }
            }
            break;

        case TimerPhase::Landing:
            if (state == FlightState::Flight)
            {
                rampAltitude(m_cfg.landAltitudeM, m_cfg.descentRateMps, dt);
            }
            else
            {
                // Normal touchdown (Flight -> Armed) or a failsafe disarm: either
                // way the mission is over and armed drops on this call.
                enter(TimerPhase::Done);
            }
            break;

        case TimerPhase::Done:
            break;
        }

        Commands cmd{};
        cmd.rcValid = true; // scripted source: the command link cannot drop
        cmd.armed = m_phase != TimerPhase::WaitArm && m_phase != TimerPhase::Done;
        cmd.flightTrigger = m_phase == TimerPhase::Flying || m_phase == TimerPhase::Landing;
        cmd.setpoints.angle = Eigen::Vector3f(0.0f, 0.0f, m_cfg.yawSetpointRad);
        cmd.setpoints.altitude = cmd.flightTrigger ? m_altitudeSetpoint : 0.0f;
        return cmd;
    }

    void TimerCommandSource::enter(TimerPhase next)
    {
        m_phase = next;
        m_elapsed = 0.0f;
    }

    void TimerCommandSource::rampAltitude(float target, float rate, float dt)
    {
        const float step = rate * dt;
        if (m_altitudeSetpoint < target)
        {
            m_altitudeSetpoint = (target - m_altitudeSetpoint > step) ? m_altitudeSetpoint + step : target;
        }
        else
        {
            m_altitudeSetpoint = (m_altitudeSetpoint - target > step) ? m_altitudeSetpoint - step : target;
        }
    }
}
