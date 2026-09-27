#include "MotorTest.h"

namespace gnc
{
    MotorTest::MotorTest(const MotorTestConfig &cfg)
        : m_cfg(cfg)
    {
    }

    void MotorTest::start()
    {
        m_result = CheckResult::Pending;
        m_running = true;
        m_motor = 0;
        m_elapsed = 0.0f;
    }

    Eigen::Vector4f MotorTest::update(float dt)
    {
        Eigen::Vector4f cmd = Eigen::Vector4f::Zero();
        if (!m_running)
        {
            return cmd;
        }

        // Spin first, then advance: every motor gets at least one tick even
        // with spinTimeS == 0.
        cmd(m_motor) = m_cfg.spinCommand;
        m_elapsed += dt;
        if (m_elapsed >= m_cfg.spinTimeS)
        {
            ++m_motor;
            m_elapsed = 0.0f;
            if (m_motor >= 4)
            {
                m_running = false;
                m_result = CheckResult::Passed; // stub: nothing measured yet
            }
        }
        return cmd;
    }
}
