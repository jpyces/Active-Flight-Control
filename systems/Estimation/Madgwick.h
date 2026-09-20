#pragma once

#include "Quaternion.h"
#include "Vector3.h"

namespace gnc
{
    // One step of Madgwick's filter, accel+gyro+mag (Group 1 only -- no
    // gyro-bias compensation, that's owned by the joint EKF/UKF later, not
    // duplicated here). Gracefully degrades: an invalid (near-zero-norm)
    // accel or mag reading contributes nothing to the correction rather than
    // corrupting it, so this single function covers what a separate
    // accel-only "NoMag" variant would -- just pass whatever mag reading you
    // have (including a zeroed/invalid one on a mag-fault path); no second
    // entry point is needed.
    Quaternion madgwickStepFull(const Quaternion &q, const Vector3 &gyro, Vector3 accel, Vector3 magUT, float dt, float beta);
}