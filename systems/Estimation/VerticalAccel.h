
#pragma once

#include "Quaternion.h" // gnc::Quaternion, gnc::Vector3

namespace gnc
{

    // Converts a raw body-frame accelerometer reading into the gravity-compensated,
    // world-frame vertical acceleration that VerticalKF::predict() expects as aMeas.
    // 1:1 port of verticalAccelFromBody.m.
    //
    //   accelBody : specific force in the body frame, m/s^2 (NOT g -- convert at the sensor)
    //   q         : Madgwick's attitude ESTIMATE (not truth)
    //   gravity   : positive magnitude, m/s^2 (e.g. 9.81)
    //
    // Returns world vertical acceleration, z-up, m/s^2: 0 at rest, +g climbing at 1g,
    // -g in free fall. Accel bias is NOT removed here -- VerticalKF estimates it as a state.
    float verticalAccelFromBody(const Vector3 &accelBody, const Quaternion &q, float gravity);

} // namespace gnc