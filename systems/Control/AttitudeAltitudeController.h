#pragma once

#include "eigen.h"
#include <array>
#include <optional>
#include "PIDControllerBase.h"
#include "Setpoints.h"
#include "ControllerMeasurements.h"

namespace gnc
{
    struct PidGains
    {
        float kp = 0.0f;
        float ki = 0.0f;
        float kd = 0.0f;
    };

    struct AxisGains
    {
        PidGains angle;       // outer loop: angle error -> rate setpoint
        PidGains rate;        // inner loop: rate error -> torque
        float maxRate = 0.0f; // angle loop output bound (symmetric, rad/s)
    };

    struct ControllerOutput
    {
        Eigen::Vector4f motorCmds = Eigen::Vector4f::Zero(); // normalized [0,1], after mixing
        float throttle = 0.0f;                               // Newtons, pre-mix (for tests/logging)
        Eigen::Vector3f torque = Eigen::Vector3f::Zero();    // N·m, pre-mix (for tests/logging)
    };

    // Cascaded attitude (angle -> rate -> torque, per axis) + altitude
    // (PID trim on hover feedforward), normalized and mixed to 4 motors.
    // 1:1 port of AttitudeAltitudeController.m.
    class AttitudeAltitudeController
    {
    private:
        // NOTE: members are initialized in THIS order regardless of the
        // constructor's initializer-list order -- keep the two in sync.
        std::array<PIDControllerBase, 3> m_angleLoopPIDs; // roll, pitch, yaw
        std::array<PIDControllerBase, 3> m_rateLoopPIDs;  // roll, pitch, yaw
        PIDControllerBase m_altitudePID;
        Eigen::Matrix4f m_mixMatrix;      // columns: [throttle, roll, pitch, yaw]
        Eigen::Vector3f m_maxTorque;      // per-axis physical torque limit (N·m)
        float m_hoverThrust, m_maxThrust; // N
        float m_dt;                       // s
        bool m_throttleOverridden{false}; // previous update() used an override

    public:
        AttitudeAltitudeController(
            const AxisGains &roll, const AxisGains &pitch, const AxisGains &yaw,
            const PidGains &altitude,
            const Eigen::Vector3f &maxTorque,
            const Eigen::Matrix4f &mixMatrix,
            float hoverThrust, float maxThrust, float dt);

        // throttleOverride (Newtons, same units as the altitude loop's output):
        // when set, the altitude PID is skipped and this thrust is used instead,
        // clamped to [0, maxThrust]. The attitude cascade runs normally either way.
        // Uses: failsafe descent, takeoff spool-up, landing ramp-down, tethered tests.
        // The altitude PID is reset on the first update after an override ends, so
        // its stale derivative/integral state cannot kick the throttle.
        ControllerOutput update(
            const Setpoints &sp, const ControllerMeasurements &meas,
            std::optional<float> throttleOverride = std::nullopt);

        // Clears all 7 child PIDs' integrator/derivative state.
        void reset();
    };
}