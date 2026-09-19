#pragma once

#include <cmath>

namespace gnc
{
    struct Vector3
    {
        float x;
        float y;
        float z;
    };

    inline float norm(Vector3 v)
    {
        return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }

    inline Vector3 operator/(Vector3 v, float c)
    {
        Vector3 o = {v.x / c, v.y / c, v.z / c};
        return o;
    }

    inline Vector3 operator+(Vector3 a, Vector3 b)
    {
        return {a.x + b.x, a.y + b.y, a.z + b.z};
    }

    inline Vector3 operator-(Vector3 a, Vector3 b)
    {
        return {a.x - b.x, a.y - b.y, a.z - b.z};
    }

    inline Vector3 operator-(Vector3 v)
    {
        return {-v.x, -v.y, -v.z};
    }

    inline Vector3 operator*(Vector3 v, float c)
    {
        return {v.x * c, v.y * c, v.z * c};
    }

    // Scalar-on-the-left, mirroring Quaternion's free-function operator*(float, const Quaternion&).
    inline Vector3 operator*(float c, Vector3 v)
    {
        return v * c;
    }

    inline float dot(Vector3 a, Vector3 b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline Vector3 cross(Vector3 a, Vector3 b)
    {
        return {a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
    }
}