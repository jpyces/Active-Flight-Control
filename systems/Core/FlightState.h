#pragma once

namespace gnc
{

    enum class FlightState
    {
        Off,
        On,
        MemoryCheck,
        SensorInit,
        CalibrationCheck,
        StateEstimationInit,
        Disarmed,
        MotorCheck,
        Armed,
        Flight,
        FailsafeAltitude,
        FailsafeSensorLoss,
        FailsafeAhrsDegraded,
        FailsafeRcLoss
    };

}