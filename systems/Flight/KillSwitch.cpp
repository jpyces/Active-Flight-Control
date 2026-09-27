#include "KillSwitch.h"

namespace gnc
{
    KillSwitch::KillSwitch(const KillSwitchConfig &cfg)
        : m_cfg(cfg)
    {
    }

    bool KillSwitch::update(bool loopOpen)
    {
        if (!m_cfg.installed || m_killed)
        {
            return m_killed;
        }

        // Debounce: pogo-pin contacts can drop for microseconds under vibration.
        // Only an uninterrupted run of open readings trips the latch.
        // debounceTicks of 0 is treated as 1 (trip on the first open reading).
        const uint8_t needed = m_cfg.debounceTicks > 0 ? m_cfg.debounceTicks : 1;
        if (loopOpen)
        {
            if (m_openCount < needed)
            {
                ++m_openCount;
            }
        }
        else
        {
            m_openCount = 0;
        }

        if (m_openCount >= needed)
        {
            m_killed = true;
        }
        return m_killed;
    }
}
