#include "Quaternion.h"
#include <cmath>
#include <algorithm> // necessary for the max and min functions that are type agnostic(only cares that the types passed are comparable)

using namespace gnc;

// Constructors
// Identity Quaternion (Default)
Quaternion::Quaternion()
    : m_W(1), m_X(0), m_Y(0), m_Z(0)
{
}

Quaternion::Quaternion(float w, float x, float y, float z)
    : m_W(w), m_X(x), m_Y(y), m_Z(z)
{
}

// Component accessors
float Quaternion::W() const
{
    return m_W;
}
float Quaternion::X() const
{
    return m_X;
}
float Quaternion::Y() const
{
    return m_Y;
}
float Quaternion::Z() const
{
    return m_Z;
}

// Math
// Hamilton multiplication for quaternions
Quaternion Quaternion::multiply(const Quaternion &q2) const
{
    return Quaternion(
        m_W * q2.m_W - m_X * q2.m_X - m_Y * q2.m_Y - m_Z * q2.m_Z,
        m_W * q2.m_X + m_X * q2.m_W + m_Y * q2.m_Z - m_Z * q2.m_Y,
        m_W * q2.m_Y - m_X * q2.m_Z + m_Y * q2.m_W + m_Z * q2.m_X,
        m_W * q2.m_Z + m_X * q2.m_Y - m_Y * q2.m_X + m_Z * q2.m_W);
}

Quaternion Quaternion::plus(const Quaternion &q2) const
{
    return Quaternion(m_W + q2.m_W, m_X + q2.m_X, m_Y + q2.m_Y, m_Z + q2.m_Z);
}

Quaternion Quaternion::minus(const Quaternion &q2) const
{
    return Quaternion(m_W - q2.m_W, m_X - q2.m_X, m_Y - q2.m_Y, m_Z - q2.m_Z);
}

// Operator Overloads
// Multiply has many scenarios
Quaternion Quaternion::operator+(const Quaternion &q2) const
{
    return plus(q2);
}

Quaternion Quaternion::operator-(const Quaternion &q2) const
{
    return minus(q2);
}

// Quaternion * Quaternion (Hamilton product) — member
Quaternion Quaternion::operator*(const Quaternion &q2) const
{
    return multiply(q2);
}
// Quaternion * scalar — member
Quaternion Quaternion::operator*(float s) const
{
    return Quaternion(m_W * s, m_X * s, m_Y * s, m_Z * s);
}
// scalar * Quaternion — must be a free function, not a member,
// because the left operand (the scalar) isn't a Quaternion,
// so there's no Quaternion object to call the member on.
Quaternion gnc::operator*(float s, const Quaternion &q)
{
    return q * s; // reuses the member overload above
}

bool Quaternion::operator==(const Quaternion &other) const
{
    return m_W == other.W() && m_X == other.X() && m_Y == other.Y() && m_Z == other.Z();
}
bool Quaternion::operator!=(const Quaternion &other) const
{
    return !(*this == other);
}

// Others
float Quaternion::norm() const
{

    return std::sqrt(m_W * m_W + m_X * m_X + m_Y * m_Y + m_Z * m_Z);
}

Quaternion Quaternion::normalize() const
{
    constexpr float kEpsilon = 1e-6f;
    float n = norm();
    if (n < kEpsilon)
    {
        return Quaternion::identity();
    }
    return Quaternion(m_W / n, m_X / n, m_Y / n, m_Z / n);
}
bool Quaternion::isDegenerate() const
{
    constexpr float kEpsilon = 1e-6f;
    return norm() < kEpsilon;
}

Quaternion Quaternion::conjugate() const
{
    return Quaternion(m_W, -m_X, -m_Y, -m_Z);
}

Quaternion Quaternion::inverse() const
{
    constexpr float kEpsilon = 1e-6f;
    float n = norm();
    float n2 = n * n;
    if (n2 < kEpsilon)
    {
        return Quaternion::identity();
    }
    Quaternion q_conj = conjugate();
    return Quaternion(q_conj.W() / n2, q_conj.X() / n2, q_conj.Y() / n2, q_conj.Z() / n2);
}

// Rotate a generic 3D vector by this quaternion
Vector3 Quaternion::rotateVector(const Vector3 &v) const
{
    // Rotate 3-vector v by this
    // quaternion: v_rot = q * [0;v] * q_conjugate, vector part.
    // Normalizes obj first since rotation is only meaningful for
    // unit quaternions -- keep obj normalized in your integration
    // loop rather than relying on this to paper over drift.
    Quaternion q = normalize();
    Quaternion v_quat = Quaternion(0.0f, v.x, v.y, v.z);
    Quaternion r = q * v_quat * q.conjugate();

    return Vector3{r.m_X, r.m_Y, r.m_Z};
}

// Integrate a gyro reading (rad/s) over dt to produce a new quaternion
Quaternion Quaternion::integrateGyro(const Vector3 &gyro, float dt) const
{
    Quaternion omegaQuat(0.0f, gyro.x, gyro.y, gyro.z);
    Quaternion qDot = multiply(omegaQuat) * 0.5f;
    Quaternion qNew = plus(qDot * dt);
    return qNew.normalize();
}

// Convert current quaternion into roll-pitch-yaw (ZYX/aerospace)
Quaternion::EulerAngles Quaternion::toEulerZYX() const
{
    // 3-2-1 (yaw-pitch-roll, aerospace ZYX) Euler angles, radians.
    Quaternion q = normalize();

    float roll = std::atan2(
        2.0f * (q.W() * q.X() + q.Y() * q.Z()),
        1.0f - 2.0f * (q.X() * q.X() + q.Y() * q.Y()));

    float sinp = 2.0f * (q.W() * q.Y() - q.Z() * q.X());
    sinp = std::clamp(sinp, -1.0f, 1.0f); // guards asin() at gimbal lock
    float pitch = std::asin(sinp);

    float yaw = std::atan2(
        2.0f * (q.W() * q.Z() + q.X() * q.Y()),
        1.0f - 2.0f * (q.Y() * q.Y() + q.Z() * q.Z()));

    return EulerAngles{roll, pitch, yaw};
}

// Static factories
Quaternion Quaternion::fromEulerZYX(float roll, float pitch, float yaw)
{
    float cr = std::cos(roll / 2);
    float sr = std::sin(roll / 2);
    float cp = std::cos(pitch / 2);
    float sp = std::sin(pitch / 2);
    float cy = std::cos(yaw / 2);
    float sy = std::sin(yaw / 2);
    return Quaternion(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy);
}

Quaternion Quaternion::identity()
{
    return Quaternion();
}