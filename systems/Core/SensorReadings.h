// SensorReadings.h
#pragma once
#include <optional>
#include "SensorStatus.h"
#include "Vector3.h"

namespace gnc
{
    // Raw(ish) sensor inputs to the state estimator, one struct per loop
    // iteration. Milestone-2-shaped by construction: every field GNSS-based
    // navigation will eventually need already exists here, so this struct's
    // shape never has to change later -- only which fields get populated
    // changes. Milestone 1 code simply never touches the GNSS fields.
    struct SensorReadings
    {
        // --- Milestone 1: always populated ---

        Vector3 accel; // accelerometer, g's, body frame
        SensorStatus accelStatus;

        Vector3 gyro; // gyro, rad/s, body frame -- convert from
                      // dps at the sensor boundary (Imu::getGyroRadPerSec),
                      // never inline at the point of use
        SensorStatus gyroStatus;

        Vector3 mag; // magnetometer, body frame
        SensorStatus magStatus;

        float baroAltitude; // barometric altitude, meters
        SensorStatus baroStatus;

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

    };
}