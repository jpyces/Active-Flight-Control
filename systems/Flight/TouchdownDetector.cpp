#include "TouchdownDetector.h"

#include <cmath>

namespace gnc
{
    TouchdownDetector::TouchdownDetector(const TouchdownConfig &cfg)
        : m_cfg(cfg)
    {
    }

    bool TouchdownDetector::update(float altitude, float verticalVelocity, float throttleFraction, float dt)
    {
        // NaN in any input fails its comparison, so a bad estimate reads as
        // "not landed" rather than producing a false touchdown edge.
        const bool nearGround = std::fabs(altitude - m_groundRef) <= m_cfg.altitudeBandM;
        const bool still = std::fabs(verticalVelocity) <= m_cfg.maxVerticalSpeedMps;
        const bool lowThrottle = throttleFraction <= m_cfg.maxThrottleFraction;

        if (nearGround && still && lowThrottle)
        {
            m_heldTime += dt;
            if (m_heldTime >= m_cfg.holdTimeS)
            {
                m_landed = true;
                m_heldTime = m_cfg.holdTimeS; // cap: no float growth while parked
            }
        }
        else
        {
            m_heldTime = 0.0f;
            m_landed = false;
        }
        return m_landed;
    }
}
