// SensorReadings.h
#pragma once
#include <optional>
#include "SensorStatus.h"
#include "Vector3.h"

namespace gnc
{
    // Raw(ish) sensor inputs to the state estimator, one struct per loop
    // iteration.
    // Each sensor carries two independent flags, both filled from its driver:
    //   xxxStatus -- health: responding and trustworthy (SensorInterface::getStatus()).
    //   xxxFresh  -- the value is a new sample this tick, not a repeat of an older
    //                one held because the sensor is slower than the loop
    //                (SensorInterface::isFresh()). Defaults to false: a producer
    //                must explicitly claim new data.
    // Consumers that fuse a measurement as an independent observation (a Kalman
    // correction) must require fresh; re-fusing a held value overweights it.
    struct SensorReadings
    {
        // --- always populated ---

        Vector3 accel; // accelerometer, g's, body frame
        SensorStatus accelStatus;
        bool accelFresh{false};

        Vector3 gyro;   // gyro, rad/s, body frame -- convert from dps at the sensor 
                        // boundary (Imu::getGyroRadPerSec), never inline at the point of use
        SensorStatus gyroStatus;
        bool gyroFresh{false};

        Vector3 mag; // magnetometer, body frame
        SensorStatus magStatus;
        bool magFresh{false};

        float baroAltitude; // barometric altitude, meters
        SensorStatus baroStatus;
        bool baroFresh{false};

        // --- Milestone 2 only: unpopulated (std::nullopt) until GNSS exists ---
        // Always read via .value() (never *opt/->), and prefer an explicit
        // has_value() guard where the caller should branch rather than
        // crash -- see StateEstimate.h for the full reasoning. This mirrors
        // MATLAB's original design intent: touching an unpopulated field
        // should fail loud, not silently propagate garbage.

        std::optional<Vector3> gnssPosition;
        SensorStatus gnssPositionStatus{SensorStatus::UNINITIALIZED};

        SensorStatus gnssVelocityStatus{SensorStatus::UNINITIALIZED};
        std::optional<Vector3> gnssVelocity;

        bool gnssFresh{false}; // one NAV-PVT carries both position and velocity
    };
}