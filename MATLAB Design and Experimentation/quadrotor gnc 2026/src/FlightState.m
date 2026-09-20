classdef FlightState
    enumeration
        Off, On, MemoryCheck, SensorInit, CalibrationCheck, StateEstimationInit, ...
        Disarmed, MotorCheck, Armed, Flight, ...
        FailsafeAltitude, FailsafeSensorLoss, FailsafeAhrsDegraded, FailsafeRcLoss
    end
end