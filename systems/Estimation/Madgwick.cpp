#include "Madgwick.h"

#include <cmath>
#include <limits>

using namespace gnc;

// anonymous namespace for this to be local only to 
namespace
{
    constexpr float kEps = std::numeric_limits<float>::epsilon();

    // Accumulates the gravity (accel) block's contribution to the 4x1
    // gradient, J_g' * f_g -- computed directly as column-of-J dot f,

    void accumulateGravityGradient(const Quaternion &q, const Vector3 &accel, float gradient[4])
    {
        float fg[3] = {
            2.0f * (q.X() * q.Z() - q.W() * q.Y()) - accel.x,
            2.0f * (q.W() * q.X() + q.Y() * q.Z()) - accel.y,
            2.0f * (0.5f - q.X() * q.X() - q.Y() * q.Y()) - accel.z};

        float Jg[3][4] = {
            {-2.0f * q.Y(), 2.0f * q.Z(), -2.0f * q.W(), 2.0f * q.X()},
            {2.0f * q.X(), 2.0f * q.W(), 2.0f * q.Z(), 2.0f * q.Y()},
            {0.0f, -4.0f * q.X(), -4.0f * q.Y(), 0.0f}};

        for (int k = 0; k < 4; ++k)
        {
            gradient[k] += Jg[0][k] * fg[0] + Jg[1][k] * fg[1] + Jg[2][k] * fg[2];
        }
    }

    // Same idea for the magnetometer block, J_b' * f_b -- only ever
    // called when mag is valid; bx/bz (world-frame field estimate) only
    // exist in that case.
    void accumulateMagGradient(const Quaternion &q, const Vector3 &magUT, float gradient[4])
    {
        Quaternion magQuat(0.0f, magUT.x, magUT.y, magUT.z);
        Quaternion h = q * magQuat * q.conjugate();

        float bx = std::sqrt(h.X() * h.X() + h.Y() * h.Y());
        float bz = h.Z();

        float fb[3] = {
            2.0f * bx * (0.5f - q.Y() * q.Y() - q.Z() * q.Z()) + 2.0f * bz * (q.X() * q.Z() - q.W() * q.Y()) - magUT.x,
            2.0f * bx * (q.X() * q.Y() - q.W() * q.Z()) + 2.0f * bz * (q.W() * q.X() + q.Y() * q.Z()) - magUT.y,
            2.0f * bx * (q.W() * q.Y() + q.X() * q.Z()) + 2.0f * bz * (0.5f - q.X() * q.X() - q.Y() * q.Y()) - magUT.z};

        float Jb[3][4] = {
            {-2.0f * bz * q.Y(), 2.0f * bz * q.Z(), -4.0f * bx * q.Y() - 2.0f * bz * q.W(), -4.0f * bx * q.Z() + 2.0f * bz * q.X()},
            {-2.0f * bx * q.Z() + 2.0f * bz * q.X(), 2.0f * bx * q.Y() + 2.0f * bz * q.W(), 2.0f * bx * q.X() + 2.0f * bz * q.Z(), -2.0f * bx * q.W() + 2.0f * bz * q.Y()},
            {2.0f * bx * q.Y(), 2.0f * bx * q.Z() - 4.0f * bz * q.X(), 2.0f * bx * q.W() - 4.0f * bz * q.Y(), 2.0f * bx * q.X()}};

        for (int k = 0; k < 4; ++k)
        {
            gradient[k] += Jb[0][k] * fb[0] + Jb[1][k] * fb[1] + Jb[2][k] * fb[2];
        }
    }
}

Quaternion gnc::madgwickStepFull(const Quaternion &q, const Vector3 &gyro, Vector3 accel, Vector3 magUT, float dt, float beta)
{
    Quaternion qdotGyro = 0.5f * q * Quaternion(0.0f, gyro.x, gyro.y, gyro.z);

    float gradient[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    bool accelValid = norm(accel) > kEps;
    if (accelValid)
    {
        accel = accel / norm(accel);
        accumulateGravityGradient(q, accel, gradient);
    }

    bool magValid = norm(magUT) > kEps;
    if (magValid)
    {
        magUT = magUT / norm(magUT);
        accumulateMagGradient(q, magUT, gradient);
    }

    float gradientNorm = std::sqrt(gradient[0] * gradient[0] + gradient[1] * gradient[1] +
                                   gradient[2] * gradient[2] + gradient[3] * gradient[3]);

    Quaternion gradientHat(0.0f, 0.0f, 0.0f, 0.0f);
    if (gradientNorm > kEps)
    {
        gradientHat = Quaternion(gradient[0] / gradientNorm, gradient[1] / gradientNorm,
                                 gradient[2] / gradientNorm, gradient[3] / gradientNorm);
    }

    Quaternion qdotCorrected = qdotGyro - beta * gradientHat;
    Quaternion qRot = qdotCorrected * dt;
    return (q + qRot).normalize();
}
