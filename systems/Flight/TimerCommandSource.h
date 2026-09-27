#pragma once
#include "Commands.h"

namespace gnc
{
    // Scripted one-shot mission, sequenced off the FSM state:
    //   Disarmed + armDelay -> arm -> Armed + takeoffDelay -> takeoff (climb to
    //   hover) -> flightDuration in Flight -> descend -> touchdown -> disarm.
    // The mission never restarts. Any unexpected FSM state (motor check failed,
    // failsafe disarm, early touchdown, a blocked edge) ends it with armed = false.
    struct TimerCommandConfig
    {
        float armDelayS;       // s in Disarmed before arming
        float takeoffDelayS;   // s in Armed (idle) before takeoff
        float flightDurationS; // s in Flight before starting the descent
        float hoverAltitudeM;  // m, climb target
        float climbRateMps;    // m/s, altitude setpoint ramp up (> 0)
        float descentRateMps;  // m/s, altitude setpoint ramp down (> 0)
        float landAltitudeM;   // m, descent target (at or slightly below ground)
        float yawSetpointRad;  // rad, heading held for the whole mission
    };

    enum class TimerPhase
    {
        WaitArm,     // waiting for Disarmed + armDelay
        Arming,      // armed asserted, waiting for MotorCheck -> Armed
        WaitTakeoff, // idling in Armed for takeoffDelay
        Flying,      // takeoff asserted, climbing / holding hover
        Landing,     // descending, waiting for touchdown (Flight -> Armed)
        Done         // mission over; armed stays false forever
    };

    class TimerCommandSource : public CommandSource
    {
    public:
        explicit TimerCommandSource(const TimerCommandConfig &cfg);

        Commands update(FlightState state, bool landed, float dt) override;

        TimerPhase phase() const { return m_phase; }

    private:
        void enter(TimerPhase next);
        void rampAltitude(float target, float rate, float dt);

        TimerCommandConfig m_cfg;
        TimerPhase m_phase{TimerPhase::WaitArm};
        float m_elapsed{0.0f};          // s in the current phase (while its state holds)
        float m_altitudeSetpoint{0.0f}; // m, ramped
    };
}
