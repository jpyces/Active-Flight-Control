#pragma once

namespace gnc
{
    // Mission sequence only. Failsafes are tracked separately (gnc::Failsafe in
    // FlightStateMachine.h), so a drone can be in Flight AND degraded at once.
    enum class FlightState
    {
        On,
        MemoryCheck,
        SensorInit,
        CalibrationCheck,
        StateEstimationInit,
        Disarmed,
        MotorCheck,
        Armed, // on the ground, motors live, ready for takeoff
        Flight
    };
}