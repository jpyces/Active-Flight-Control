#pragma once

#include "eigen.h"
#include "CheckResult.h"

namespace gnc
{
    // Pre-flight motor check run during FlightState::MotorCheck.
    //
    // STUB: spins each motor in turn (index 0..3) at spinCommand for spinTimeS,
    // then reports Passed. Verifies nothing yet; it exists so MotorCheck
    // exercises the full sequence in the SIM and on the bench. The real version
    // checks ESC RPM telemetry (bidirectional DShot) and can report Failed.
    struct MotorTestConfig
    {
        float spinCommand; // normalized [0, 1] command for the motor under test
        float spinTimeS;   // s per motor
    };

    class MotorTest
    {
    public:
        explicit MotorTest(const MotorTestConfig &cfg);

        // Restarts the sequence from motor 0 and sets result() to Pending.
        void start();

        // Advances the sequence; returns this tick's normalized motor commands.
        // Zero for all motors unless the test is running.
        Eigen::Vector4f update(float dt);

        // Pending before start() and while running; Passed once all four ran.
        CheckResult result() const { return m_result; }

        bool running() const { return m_running; }

    private:
        MotorTestConfig m_cfg;
        CheckResult m_result{CheckResult::Pending};
        bool m_running{false};
        int m_motor{0};         // motor currently spinning
        float m_elapsed{0.0f};  // s on the current motor
    };
}
