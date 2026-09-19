/*
Header for definition for Quaternion Class
*/
#pragma once

#include <cmath>
#include "Vector3.h"

namespace gnc
{

    class Quaternion
    {

    private:
        float m_W, m_X, m_Y, m_Z;

    public:

        // Euler angle triple, ZYX/aerospace convention
        struct EulerAngles
        {
            float roll;
            float pitch;
            float yaw;
        };

        // Default: identity quaternion
        Quaternion();

        Quaternion(float w, float x, float y, float z);

        // Component accessors
        float W() const;
        float X() const;
        float Y() const;
        float Z() const;

        // Hamilton multiplication for quaternions
        Quaternion multiply(const Quaternion &q2) const;

        Quaternion plus(const Quaternion &q2) const;

        Quaternion minus(const Quaternion &q2) const;

        // Operator overloads mirroring multiply/plus/minus
        Quaternion operator*(const Quaternion &q2) const;
        Quaternion operator+(const Quaternion &q2) const;
        Quaternion operator-(const Quaternion &q2) const;

        float norm() const;

        Quaternion normalize() const;

        Quaternion conjugate() const;

        Quaternion inverse() const;

        // Rotate a generic 3D vector by this quaternion
        Vector3 rotateVector(const Vector3 &v) const;

        // Integrate a gyro reading (rad/s) over dt to produce a new quaternion
        Quaternion integrateGyro(const Vector3 &gyro, float dt) const;

        // Convert current quaternion into roll-pitch-yaw (ZYX/aerospace)
        EulerAngles toEulerZYX() const;

        // Static factories
        static Quaternion fromEulerZYX(float roll, float pitch, float yaw);

        static Quaternion identity();
    };

}