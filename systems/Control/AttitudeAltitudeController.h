#pragma once

#include "eigen.h"
#include <array>
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

    public:
        AttitudeAltitudeController(
            const AxisGains &roll, const AxisGains &pitch, const AxisGains &yaw,
            const PidGains &altitude,
            const Eigen::Vector3f &maxTorque,
            const Eigen::Matrix4f &mixMatrix,
            float hoverThrust, float maxThrust, float dt);

        ControllerOutput update(const Setpoints &sp, const ControllerMeasurements &meas);

        // Clears all 7 child PIDs' integrator/derivative state.
        void reset();
    };
}