#pragma once
#include "Setpoints.h"
#include "FlightState.h"

namespace gnc
{
    struct Commands
    {
        bool armed = false;         // operator wants motors armed
        bool flightTrigger = false; // operator wants takeoff
        bool rcValid = false;       // command link healthy (drives the RcLoss failsafe)
        Setpoints setpoints;        // attitude + altitude targets for the controller
    };

    // Source of operator commands. FlightCore holds a reference and calls
    // update() once per tick. Implementations: TimerCommandSource (scripted
    // mission), later an RC/radio source.
    class CommandSource
    {
    public:
        virtual ~CommandSource() = default;

        // state/landed: FSM outputs from the previous tick.
        // dt: tick period in seconds (FlightCore has no clock).
        virtual Commands update(FlightState state, bool landed, float dt) = 0;
    };
}