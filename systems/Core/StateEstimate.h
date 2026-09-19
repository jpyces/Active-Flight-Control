// StateEstimate.h
#pragma once

#include <optional>
#include "Vector3.h"

namespace gnc
{
    // The estimator's fused output -- what the system currently believes
    // about its own state. Controller-agnostic and Milestone-2-shaped by
    // construction, same reasoning as SensorReadings: this struct's shape
    // is the stable, general-purpose contract; a specific controller's
    // field-naming quirks belong in an adapter
    // (stateEstimateToControllerMeasurements, in ControllerMeasurements.h),
    // not in this struct..
    struct StateEstimate
    {
        // --- Milestone 1: always populated ---

        Vector3 angles;         // roll, pitch, yaw (rad) -- Madgwick's fused estimate
        Vector3 rates;          // roll, pitch, yaw rate (rad/s) -- raw(ish) gyro,
                                // deliberately NOT fused through Madgwick: the rate
                                // loop is meant to be the fast, always-available first
                                // line of disturbance rejection, independent of
                                // whichever AHRS source is currently authoritative
        float altitude;         // meters, from the vertical KF
        float verticalVelocity; // m/s, from the vertical KF

        // --- Milestone 2 only: unpopulated (std::nullopt) until the joint
        //     estimator (UKF/EKF) exists. Read via .value()/has_value()
        //     only -- see SensorReadings.h for the reasoning. ---

        std::optional<Vector3> position;
        std::optional<Vector3> velocity;
        std::optional<Vector3> gyroBias;
        std::optional<Vector3> accelBias;
    };
}