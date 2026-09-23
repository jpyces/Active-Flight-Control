#pragma once

#include "eigen.h"
#include "Vector3.h"
#include "StateEstimate.h"

namespace gnc
{
    // The controller's per-loop feedback -- what AttitudeAltitudeController
    // reads as "current truth" on each update() call. Built fresh every
    // iteration from StateEstimate via the adapter below. Distinct from
    // Setpoints: this is what IS, Setpoints is what's WANTED.
    //
    // Eigen::Vector3f (matching Setpoints) so the controller can index
    // per axis (angle(i)) directly; the Vector3 -> Eigen conversion lives
    // in the adapter below, at the estimator/controller boundary.
    struct ControllerMeasurements
    {
        Eigen::Vector3f angle = Eigen::Vector3f::Zero(); // current roll/pitch/yaw (rad)
        Eigen::Vector3f rate = Eigen::Vector3f::Zero();  // current roll/pitch/yaw rate (rad/s)
        float altitude = 0.0f;                           // current altitude (m)
    };

    // Adapter: StateEstimate's general plural naming (and Vector3 type) ->
    // the controller's matched singular naming (and Eigen type).
    // verticalVelocity intentionally dropped -- AttitudeAltitudeController's
    // D-term comes from PIDControllerBase's own finite-difference, not a
    // separately supplied velocity.
    inline ControllerMeasurements stateEstimateToControllerMeasurements(const StateEstimate &est)
    {
        ControllerMeasurements m;
        m.angle = toEigen(est.angles);
        m.rate = toEigen(est.rates);
        m.altitude = est.altitude;
        return m;
    }
}