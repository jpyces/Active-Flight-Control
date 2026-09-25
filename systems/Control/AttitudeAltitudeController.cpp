#include "AttitudeAltitudeController.h"
#include "MotorMixer.h"

#include <algorithm>

namespace gnc
{
    AttitudeAltitudeController::AttitudeAltitudeController(
        const AxisGains &roll, const AxisGains &pitch, const AxisGains &yaw,
        const PidGains &altitude,
        const Eigen::Vector3f &maxTorque,
        const Eigen::Matrix4f &mixMatrix,
        float hoverThrust, float maxThrust, float dt)
        // Outer loops: angle error -> rate setpoint, bounded by each axis's maxRate
        : m_angleLoopPIDs{
              {PIDControllerBase(roll.angle.kp, roll.angle.ki, roll.angle.kd, -roll.maxRate, roll.maxRate),
               PIDControllerBase(pitch.angle.kp, pitch.angle.ki, pitch.angle.kd, -pitch.maxRate, pitch.maxRate),
               PIDControllerBase(yaw.angle.kp, yaw.angle.ki, yaw.angle.kd, -yaw.maxRate, yaw.maxRate)}},
          // Inner loops: rate error -> torque, bounded by physical per-axis torque limit
          m_rateLoopPIDs{
              {PIDControllerBase(roll.rate.kp, roll.rate.ki, roll.rate.kd, -maxTorque.x(), maxTorque.x()),
               PIDControllerBase(pitch.rate.kp, pitch.rate.ki, pitch.rate.kd, -maxTorque.y(), maxTorque.y()),
               PIDControllerBase(yaw.rate.kp, yaw.rate.ki, yaw.rate.kd, -maxTorque.z(), maxTorque.z())}},
          // Altitude: trim on top of hover feedforward; asymmetric since thrust can't go negative
          m_altitudePID(altitude.kp, altitude.ki, altitude.kd,
                        -hoverThrust, maxThrust - hoverThrust),
          m_mixMatrix(mixMatrix),
          m_maxTorque(maxTorque),
          m_hoverThrust(hoverThrust),
          m_maxThrust(maxThrust),
          m_dt(dt)
    {
    }

    ControllerOutput AttitudeAltitudeController::update(
        const Setpoints &sp, const ControllerMeasurements &meas, std::optional<float> throttleOverride)
    {
        Eigen::Vector3f torque = Eigen::Vector3f::Zero();

        // Attitude cascade per axis: angle error -> rate setpoint -> torque
        for (size_t i = 0; i < 3; ++i)
        {
            auto &anglePid = m_angleLoopPIDs[i];
            float desiredRate = anglePid.update(sp.angle[i], meas.angle[i], m_dt);

            auto &ratePid = m_rateLoopPIDs[i];
            torque[i] = ratePid.update(desiredRate, meas.rate[i], m_dt);
        }

        // Altitude: either an external thrust override, or PID trim on top of
        // hover feedforward. Both paths produce Newtons and share the
        // normalization + mixing below -- one units path, not two.
        float throttle; // Newtons -- returned as-is for tests/logging
        if (throttleOverride.has_value())
        {
            // PID skipped entirely: it must not act on (or accumulate from) an
            // altitude estimate that may be the reason for the override.
            throttle = std::clamp(*throttleOverride, 0.0f, m_maxThrust);
            m_throttleOverridden = true;
        }
        else
        {
            if (m_throttleOverridden)
            {
                // Handover back to closed loop: stale prevMeasurement would cause
                // a derivative kick, stale integral a step. reset() clears both and
                // suppresses D on this first call.
                m_altitudePID.reset();
                m_throttleOverridden = false;
            }
            float trim = m_altitudePID.update(sp.altitude, meas.altitude, m_dt);
            throttle = m_hoverThrust + trim;
        }

        // Normalize both channels into mixMotors' unitless convention
        float normThrottle = throttle / m_maxThrust;                          // [0,1]
        const Eigen::Vector3f normTorque = torque.cwiseQuotient(m_maxTorque); // [-1,1], elementwise

        Eigen::Vector4f command;
        command << normThrottle, normTorque(0), normTorque(1), normTorque(2); // [throttle, roll, pitch, yaw]

        const Eigen::Vector4f motorCmds = mixMotors(command, m_mixMatrix);

        return ControllerOutput{motorCmds, throttle, torque};
    }

    void AttitudeAltitudeController::reset()
    {
        for (auto &pid : m_angleLoopPIDs)
            pid.reset();
        for (auto &pid : m_rateLoopPIDs)
            pid.reset();
        m_altitudePID.reset();
        m_throttleOverridden = false;
    }
}