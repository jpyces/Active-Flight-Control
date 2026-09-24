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
    // What happened inside one step -- things the normalized result can't tell you.
    struct MadgwickDiagnostics
    {
        // |q + qdot*dt| BEFORE the final normalize(). The returned quaternion is
        // always unit-norm (or identity, if this collapsed below epsilon), so this
        // is the only place a collapsed or non-finite step is still visible.
        float preNormalizeNorm{1.0f};
        bool accelUsed{false}; // accel passed the near-zero-norm guard this step
        bool magUsed{false};   // mag passed the near-zero-norm guard this step
    };

    // `diag` is optional: pass a pointer to get the step diagnostics, or omit it
    // (existing call sites are unaffected).
    Quaternion madgwickStepFull(
        const Quaternion &q, const Vector3 &gyro, Vector3 accel, Vector3 magUT, float dt, float beta, MadgwickDiagnostics *diag = nullptr);
}