#pragma once

#include "eigen.h"

namespace gnc
{
    // What the controller is being asked to achieve. Field names match
    // AttitudeAltitudeController::update()'s real parameter shape exactly
    // (singular angle/altitude, not StateEstimate's plural angles) --
    // unlike StateEstimate, this struct was designed knowing its one real
    // consumer up front, so it needs no adapter going in.

    struct Setpoints
    {
        Eigen::Vector3f angle = Eigen::Vector3f::Zero(); // desired roll, pitch, yaw (rad)
        float altitude = 0.0f;                           // desired altitude (m)
    };
}