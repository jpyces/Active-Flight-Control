#pragma once

#include "Quaternion.h"
#include "Vector3.h"

namespace gnc
{
    Quaternion complementaryFilter(const Quaternion &q, const Vector3 &gyro, Vector3 accel, float dt, float alpha);
}