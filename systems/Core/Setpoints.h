// Setpoints.h
#pragma once

#include "Vector3.h"

namespace gnc
{
    // What the controller is being asked to achieve. Field names match
    // AttitudeAltitudeController::update()'s real parameter shape exactly
    // (singular angle/altitude, not StateEstimate's plural angles) --
    // unlike StateEstimate, this struct was designed knowing its one real
    // consumer up front, so it needs no adapter going in. It does still
    // need bridging FROM a StateEstimate when you want "hold current
    // attitude" as a setpoint -- that's a separate concern from
    // ControllerMeasurements.h's stateEstimateToControllerMeasurements()
    // (which builds per-loop feedback, not a commanded target); no such
    // adapter exists yet here.
    struct Setpoints
    {
        Vector3 angle;  // desired roll, pitch, yaw (rad)
        float altitude; // desired altitude (m)
    };
}