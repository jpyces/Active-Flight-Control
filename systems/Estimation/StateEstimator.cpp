#include "StateEstimator.h"

#include <cmath>
#include "ComplimentaryFilter.h"
#include "Madgwick.h"
#include "VerticalAccel.h"

namespace gnc
{
    namespace
    {
        constexpr Vector3 kZero{0.0f, 0.0f, 0.0f};

        // Below this, normalize() treats a quaternion as degenerate -- matches the
        // kEpsilon in Quaternion.cpp.
        constexpr float kCollapseEps = 1e-6f;

        bool isFinite(const Vector3 &v)
        {
            return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
        }

        // A NaN reading would permanently poison BOTH filters (including the
        // complementary fallback, which reset() resyncs from), so it is gated out
        // regardless of what the sensor layer says.
        bool isUsable(SensorStatus s, const Vector3 &v)
        {
            return (s == SensorStatus::NOMINAL || s == SensorStatus::DEGRADED) && isFinite(v);
        }
    }

    StateEstimator::StateEstimator(const StateEstimatorConfig &cfg)
        : m_cfg(cfg),
          m_kf(cfg.kfQ, cfg.kfR, cfg.kfX0, cfg.kfP0),
          m_comp(Quaternion::identity()),
          m_madgwick(Quaternion::identity()),
          m_attitude(Quaternion::identity())
    {
    }

    StateEstimate StateEstimator::update(const SensorReadings &in, float dt)
    {
        const bool accelUsable = isUsable(in.accelStatus, in.accel);
        const bool gyroUsable = isUsable(in.gyroStatus, in.gyro);
        const bool magUsable = in.magStatus == SensorStatus::NOMINAL && isFinite(in.mag);

        const Vector3 accel = accelUsable ? in.accel : kZero;
        const Vector3 gyro = gyroUsable ? in.gyro : kZero;
        const Vector3 mag = magUsable ? in.mag : kZero;

        // -- Complementary filter: runs every step regardless of source, so it is
        // always a live seed/fallback and never goes stale.
        m_comp = complementaryFilter(m_comp, gyro, accel, dt, m_cfg.compAlpha);

        switch (m_source)
        {
        case AhrsSource::Seeding:
            m_magUsed = false;
            m_seedingElapsed += dt;
            // Half-step slack: summing a float dt (0.01f * 50) can land just under
            // the duration and flip one step late. This flips on the step nearest
            // to seedingDuration instead.
            if (m_seedingElapsed + 0.5f * dt >= m_cfg.seedingDuration)
            {
                m_madgwick = m_comp;
                m_source = AhrsSource::Nominal;
            }
            m_attitude = m_comp;
            break;

        case AhrsSource::Nominal:
        {
            MadgwickDiagnostics diag;
            const Quaternion qNext = madgwickStepFull(m_madgwick, gyro, accel, mag, dt, m_cfg.madgwickBeta, &diag);
            // From Madgwick itself, not magStatus: a NOMINAL-but-zero mag reading
            // also contributes nothing to yaw.
            m_magUsed = diag.magUsed;
            if (detectInstability(qNext, diag.preNormalizeNorm))
            {
                // Discard the bad candidate -- m_madgwick keeps its last good value.
                m_source = AhrsSource::FallbackComplementary;
                m_magUsed = false;
                m_attitude = m_comp;
            }
            else
            {
                m_madgwick = qNext;
                m_attitude = m_madgwick;
            }
            break;
        }

        case AhrsSource::FallbackComplementary:
            m_magUsed = false;
            m_attitude = m_comp;
            break;
        }

        // -- Vertical channel. predict() every step; correct() only on a good, new
        // baro sample. The baro is slower than the loop, so between samples the
        // reading is a held repeat; correcting on it again would treat one
        // measurement as several and overweight the baro relative to kfR.
        // With no usable accel, predict with aMeas = the bias estimate, i.e. zero
        // net acceleration (constant-velocity propagation) -- verticalAccelFromBody
        // on a zero reading would read as free fall (-g).
        const float aVert = accelUsable
                                ? verticalAccelFromBody(in.accel * m_cfg.gravity, m_attitude, m_cfg.gravity)
                                : m_kf.accelBias();
        m_kf.predict(aVert, dt);
        if (in.baroStatus == SensorStatus::NOMINAL && in.baroFresh)
        {
            m_kf.correct(in.baroAltitude);
        }

        const Quaternion::EulerAngles e = m_attitude.toEulerZYX();

        StateEstimate out{};
        out.angles = Vector3{e.roll, e.pitch, e.yaw};
        out.rates = in.gyro; // raw passthrough by design -- see StateEstimate.h
        out.altitude = m_kf.altitude();
        out.verticalVelocity = m_kf.velocity();
        return out;
    }

    void StateEstimator::reset()
    {
        m_madgwick = m_comp;
        m_attitude = m_comp;
        m_source = AhrsSource::Nominal;
        m_seedingElapsed = 0.0f;
    }

    bool StateEstimator::detectInstability(const Quaternion &q, float preNormalizeNorm)
    {
        const bool finite =
            std::isfinite(q.W()) && std::isfinite(q.X()) &&
            std::isfinite(q.Y()) && std::isfinite(q.Z()) &&
            std::isfinite(preNormalizeNorm);

        return !finite || preNormalizeNorm < kCollapseEps;
    }
}
