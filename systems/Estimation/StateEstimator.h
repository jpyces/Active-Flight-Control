#pragma once

#include "eigen.h"
#include "Quaternion.h"
#include "SensorReadings.h"
#include "StateEstimate.h"
#include "VerticalKF.h"

namespace gnc
{
    // Which attitude estimate is currently authoritative.
    //   Seeding               -- cold boot; complementary filter output, Madgwick idle
    //   Nominal               -- Madgwick output (seeded from the complementary filter)
    //   FallbackComplementary -- Madgwick went numerically unstable; complementary
    //                            filter output. Only reset() leaves this state.
    enum class AhrsSource
    {
        Seeding,
        Nominal,
        FallbackComplementary
    };

    struct StateEstimatorConfig
    {
        float madgwickBeta;       // Madgwick gradient-descent gain (e.g. 0.1)
        float compAlpha;          // complementary filter gyro-branch weight (e.g. 0.98)
        float gravity;            // m/s^2 (e.g. 9.81) -- also the g -> m/s^2 accel scale
        float seedingDuration;    // s, fixed seeding timer (e.g. 0.5)

        // VerticalKF parameters (see VerticalKF.h)
        Eigen::Matrix3f kfQ;
        float kfR;
        Eigen::Vector3f kfX0;
        Eigen::Matrix3f kfP0;
    };

    // Owns the AHRS source switching (complementary seed -> Madgwick nominal ->
    // complementary fallback) and the vertical KF; produces the StateEstimate the
    // controller consumes. 
    //
    // Sensor gating, applied every update() (a reading with any NaN/Inf component
    // is always unusable, whatever its status says):
    //   accel, gyro : usable if NOMINAL or DEGRADED. An unusable reading is fed to
    //                 the filters as zero -- zero accel skips the tilt correction
    //                 (both filters already guard on near-zero norm), zero gyro holds
    //                 attitude. Deciding what to DO about a lost IMU is the FSM's job
    //                 (failsafe branch 2.1); this class just avoids fusing garbage.
    //   mag         : usable only if NOMINAL (Madgwick's fixed gain can't down-weight
    //                 a DEGRADED sensor). Unusable -> zero, per Madgwick.h's contract.
    //   baro        : KF correct() only if NOMINAL; predict() always runs.
    class StateEstimator
    {
    public:
        explicit StateEstimator(const StateEstimatorConfig &cfg);

        // One estimator step. `in.accel` is in g's (per SensorReadings.h) and is
        // scaled to m/s^2 here before the vertical channel sees it.
        StateEstimate update(const SensorReadings &in, float dt);

        // Single recovery path for cold re-arm AND in-flight fault recovery:
        // resync Madgwick from the (always-running) complementary filter and go
        // straight to Nominal, skipping Seeding. Does NOT touch the vertical KF.
        void reset();

        bool isConverged() const { return m_source == AhrsSource::Nominal; }
        bool isDegraded() const { return m_source == AhrsSource::FallbackComplementary; }

        // True only when Madgwick is authoritative AND fused a usable mag reading on
        // the last step. The complementary filter pulls yaw toward 0, so yaw is never
        // observable outside Nominal.
        bool yawObservable() const { return m_source == AhrsSource::Nominal && m_magUsed; }

        AhrsSource source() const { return m_source; }
        const Quaternion &attitude() const { return m_attitude; }
        const VerticalKF &verticalKF() const { return m_kf; }

        // Madgwick step went bad: non-finite result, or the pre-normalize quaternion
        // collapsed toward zero (normalize() would have silently returned identity).
        // Takes MadgwickDiagnostics::preNormalizeNorm because the returned q is always
        // unit-norm -- a norm-drift check on it can never trip.
        static bool detectInstability(const Quaternion &q, float preNormalizeNorm);

    private:
        // Test-only seam (defined under test/, never in the firmware build) so tests
        // can force FallbackComplementary without engineering a real divergence.
        friend struct StateEstimatorTestAccess;

        StateEstimatorConfig m_cfg;
        VerticalKF m_kf;

        Quaternion m_comp;     // complementary filter state, always running
        Quaternion m_madgwick; // Madgwick state, last known-good
        Quaternion m_attitude; // whichever is authoritative this step

        AhrsSource m_source{AhrsSource::Seeding};
        float m_seedingElapsed{0.0f};
        bool m_magUsed{false};
    };
}
