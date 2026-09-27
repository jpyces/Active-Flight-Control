#pragma once
#include <cstdint>

namespace gnc
{
    // Operator kill input: a normally-closed loop from a digital pin (INPUT_PULLUP)
    // to GND through a pull-out plug. Loop closed = run, loop open = kill, so a
    // pulled plug, broken wire or missing plug all read as kill.
    //
    // Pure logic: main.cpp reads the pin and passes loopOpen each tick.
    struct KillSwitchConfig
    {
        bool installed;        // false: no plug fitted, input ignored, never kills
        uint8_t debounceTicks; // consecutive open readings required to trip (>= 1)
    };

    class KillSwitch
    {
    public:
        explicit KillSwitch(const KillSwitchConfig &cfg);

        // One reading per tick. Returns killed(). Once tripped, stays tripped
        // until reboot: closing the loop again does not clear it.
        bool update(bool loopOpen);

        bool killed() const { return m_killed; }

    private:
        KillSwitchConfig m_cfg;
        uint8_t m_openCount{0};
        bool m_killed{false};
    };
}
