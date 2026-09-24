#include "VerticalAccel.h"

namespace gnc
{

    // Only the vertical component of R_body_to_world * accelBody is needed, which is the
    // third row of R_body_to_world = the third column of R_world_to_body. That column is
    // world "up" expressed in body axes, and its three terms are exactly Madgwick's
    // gravity residual f_g (madgwickStepFull, Madgwick 2010 appendix). Building it from
    // those terms rather than Quaternion::rotateVector guarantees the same rotation
    // convention as the attitude estimate being passed in 
    float verticalAccelFromBody(const Vector3 &accelBody, const Quaternion &q, float gravity)
    {
        const float w = q.W();
        const float x = q.X();
        const float y = q.Y();
        const float z = q.Z();

        const float upX = 2.0f * (x * z - w * y);
        const float upY = 2.0f * (y * z + w * x);
        const float upZ = 1.0f - 2.0f * (x * x + y * y);

        const float worldVertical = upX * accelBody.x + upY * accelBody.y + upZ * accelBody.z;
        return worldVertical - gravity;
    }

} // namespace gnc