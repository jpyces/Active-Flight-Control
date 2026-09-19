#pragma once

#include "Vector3.h"
#include "StateEstimate.h"

namespace gnc
{
    // The controller's per-loop feedback -- what AttitudeAltitudeController
    // reads as "current truth" on each update() call. Built fresh every
    // iteration from StateEstimate via the adapter below. Distinct from
    // Setpoints: this is what IS, Setpoints is what's WANTED.
    struct ControllerMeasurements
    {
        Vector3 angle;  // current roll/pitch/yaw (rad)
        Vector3 rate;   // current roll/pitch/yaw rate (rad/s)
        float altitude; // current altitude (m)
    };

    // Adapter: StateEstimate's general plural naming -> the controller's
    // matched singular naming. verticalVelocity intentionally dropped --
    // AttitudeAltitudeController's D-term comes from PIDControllerBase's
    // own finite-difference, not a separately supplied velocity.
    inline ControllerMeasurements stateEstimateToControllerMeasurements(const StateEstimate &est)
    {
        ControllerMeasurements m;
        m.angle = est.angles;
        m.rate = est.rates;
        m.altitude = est.altitude;
        return m;
    }
}