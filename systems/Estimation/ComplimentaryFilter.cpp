#include "ComplimentaryFilter.h"

#include <cmath>

namespace gnc
{
    Quaternion complementaryFilter(const Quaternion &q, const Vector3 &gyro, Vector3 accel, float dt, float alpha)
    {
        // -- Gyro branch: reuse integrateGyro directly, the same "predict" half
        // every filter here shares.
        Quaternion q_gyro = q.integrateGyro(gyro, dt);

        // -- Degenerate-accel guard: a near-zero accel reading (sensor glitch,
        // free-fall) has no usable tilt information. Same principle as
        // madgwickStepFull/madgwickNoMag's accelValid guard -- zero out this
        // branch's contribution entirely rather than feeding a fake/garbage
        // reading into the normal formula (which here would divide by ~0 and
        // produce NaN, not just a wrong-but-finite answer). Falling back
        // fully to the gyro prediction is the correct "no correction this
        // step" behavior, exactly like Madgwick zeroing its residual/Jacobian.
        constexpr float kEpsilon = 1e-6f;
        float accelNorm = norm(accel);
        if (accelNorm < kEpsilon)
        {
            return q_gyro.normalize();
        }

        // -- Accel branch: 2DOF tilt-only quaternion.
        // Yaw is unobservable from accel alone, so it's forced to 0 here --
        // see the docstring warning above about what that assumption can do once blended.
        Vector3 a = accel / accelNorm;
        float roll = std::atan2(a.y, a.z);
        float pitch = std::atan2(-a.x, std::sqrt(a.y * a.y + a.z * a.z));
        Quaternion q_accel = Quaternion::fromEulerZYX(roll, pitch, 0);

        // -- Blend: weighted quaternion ADDITION, not Hamilton multiplication --
        // same "not a proper rotation on its own, only meaningful as a
        // combination" caveat as q_rot in integrateGyro, just combining two full
        // quaternions instead of an increment.
        Quaternion q_blend = alpha * q_gyro + (1 - alpha) * q_accel;

        // -- Renormalize: weighted addition of two unit quaternions is not itself unit-norm in general.
        return q_blend.normalize();
    }
}