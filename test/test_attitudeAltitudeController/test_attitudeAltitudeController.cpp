#include <unity.h>
#include <cmath>
#include "eigen.h"
#include "AttitudeAltitudeController.h"

using namespace gnc;

// ---------------------------------------------------------------------------
// Toy plants -- test-only, never linked into teensy41 (lives under test/).
// 1:1 ports of simulateRatePlantStep.m / simulateAltitudePlantStep.m.
// ---------------------------------------------------------------------------
namespace gnc::sim
{
    // I*angAccel = torque - c*rate (semi-implicit Euler: rate first, then angle)
    inline void ratePlantStep(float &angle, float &rate, float torque, float I, float c, float dt)
    {
        float angularAccel = (torque - c * rate) / I;
        rate += angularAccel * dt;
        angle += rate * dt;
    }

    // accel = thrust/m - g
    inline void altitudePlantStep(float &altitude, float &velocity, float thrust, float mass, float g, float dt)
    {
        float accel = thrust / mass - g;
        velocity += accel * dt;
        altitude += velocity * dt;
    }
}

// ---------------------------------------------------------------------------
// Fixture -- constants match IntegratedControllerFullTest.m / AltitudeHoldTest.m.
// UNITS: degrees and deg/s throughout, exactly as the MATLAB tests use them.
// ---------------------------------------------------------------------------
namespace
{
    constexpr float kDt = 0.01f;
    constexpr float kI = 0.01f;
    constexpr float kC = 0.001f;
    constexpr float kMass = 1.0f;
    constexpr float kG = 10.0f;
    constexpr float kHoverThrust = kMass * kG; // 10 N
    constexpr float kMaxThrust = 15.0f;
    constexpr float kMaxRate = 150.0f; // angleBounds = [-150, 150] (deg/s)

    constexpr float kAngleTol = 1.0f;     // deg
    constexpr float kAngleSettleT = 2.0f; // s
    constexpr float kAltTol = 0.1f;       // m
    constexpr float kAltSettleT = 4.0f;   // s
    constexpr float kDuration = 8.0f;     // s, matches AltitudeHoldTest
    constexpr float kLeakEps = 1e-6f;

    constexpr int kAngleSettleIdx = static_cast<int>(kAngleSettleT / kDt + 0.5f);
    constexpr int kAltSettleIdx = static_cast<int>(kAltSettleT / kDt + 0.5f);

    Eigen::Matrix4f realMixMatrix()
    {
        Eigen::Matrix4f m;
        m << 1, 1, 1, 1,
            1, -1, 1, -1,
            1, -1, -1, 1,
            1, 1, -1, -1;
        return m;
    }

    AttitudeAltitudeController makeController()
    {
        AxisGains axis{{3.0f, 0.0f, 0.5f}, {0.05f, 0.01f, 0.001f}, kMaxRate};
        PidGains altitude{3.0f, 0.0f, 3.0f};
        return AttitudeAltitudeController(axis, axis, axis, altitude,
                                          Eigen::Vector3f::Ones(), realMixMatrix(),
                                          kHoverThrust, kMaxThrust, kDt);
    }

    struct SimResult
    {
        Eigen::Vector3f maxAbsAngleAfterSettle = Eigen::Vector3f::Zero(); // from t = 2 s on
        float maxAltErrAfterSettle = 0.0f;                                // from t = 4 s on
        Eigen::Vector3f maxAbsTorque = Eigen::Vector3f::Zero();           // whole run
        float minThrust = 1e9f, maxThrust = -1e9f;                        // whole run
    };

    SimResult runClosedLoop(const Eigen::Vector3f &angle0, float altitude0, float altSetpoint)
    {
        AttitudeAltitudeController ctrl = makeController();

        Setpoints sp;
        sp.altitude = altSetpoint; // angle setpoint = level (default zero)

        Eigen::Vector3f angle = angle0, rate = Eigen::Vector3f::Zero();
        float alt = altitude0, vel = 0.0f;
        SimResult r;

        const int steps = static_cast<int>(kDuration / kDt + 0.5f);
        for (int k = 0; k < steps; ++k)
        {
            ControllerMeasurements meas;
            meas.angle = angle;
            meas.rate = rate;
            meas.altitude = alt;

            ControllerOutput out = ctrl.update(sp, meas);

            for (int i = 0; i < 3; ++i)
            {
                r.maxAbsTorque[i] = std::fmax(r.maxAbsTorque[i], std::fabs(out.torque[i]));
                gnc::sim::ratePlantStep(angle[i], rate[i], out.torque[i], kI, kC, kDt);
            }
            r.minThrust = std::fmin(r.minThrust, out.throttle);
            r.maxThrust = std::fmax(r.maxThrust, out.throttle);
            gnc::sim::altitudePlantStep(alt, vel, out.throttle, kMass, kG, kDt);

            // MATLAB logs post-step state at index k+1 and checks hist(settleIdx:end)
            if (k + 1 >= kAngleSettleIdx)
                for (int i = 0; i < 3; ++i)
                    r.maxAbsAngleAfterSettle[i] = std::fmax(r.maxAbsAngleAfterSettle[i], std::fabs(angle[i]));
            if (k + 1 >= kAltSettleIdx)
                r.maxAltErrAfterSettle = std::fmax(r.maxAltErrAfterSettle, std::fabs(alt - altSetpoint));
        }
        return r;
    }

    // Disturb one axis only (altitude held at setpoint); it must settle and
    // every other channel must stay exactly quiet.
    void runSingleAxisTest(int axis)
    {
        Eigen::Vector3f angle0 = Eigen::Vector3f::Zero();
        angle0[axis] = 30.0f; // deg, matches RollAngleCascadeTest's disturbance

        SimResult r = runClosedLoop(angle0, 0.0f, 0.0f);

        TEST_ASSERT_TRUE(r.maxAbsAngleAfterSettle[axis] < kAngleTol);
        for (int j = 0; j < 3; ++j)
        {
            if (j == axis)
                continue;
            TEST_ASSERT_FLOAT_WITHIN(kLeakEps, 0.0f, r.maxAbsTorque[j]);
            TEST_ASSERT_FLOAT_WITHIN(kLeakEps, 0.0f, r.maxAbsAngleAfterSettle[j]);
        }
        TEST_ASSERT_FLOAT_WITHIN(kLeakEps, kHoverThrust, r.minThrust);
        TEST_ASSERT_FLOAT_WITHIN(kLeakEps, kHoverThrust, r.maxThrust);
    }
}

void setUp() {}
void tearDown() {}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// Zero error: throttle == hover feedforward, zero torque, and all four
// motors equal hover/maxThrust (pure throttle through the real mix matrix).
void test_zero_error_gives_hover_and_equal_motors()
{
    AttitudeAltitudeController ctrl = makeController();
    Setpoints sp;
    ControllerMeasurements meas;

    ControllerOutput out = ctrl.update(sp, meas);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, kHoverThrust, out.throttle);
    for (int i = 0; i < 3; ++i)
        TEST_ASSERT_FLOAT_WITHIN(kLeakEps, 0.0f, out.torque[i]);
    for (int m = 0; m < 4; ++m)
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, kHoverThrust / kMaxThrust, out.motorCmds[m]);
}

// IntegratedControllerRollOnly/PitchOnly/YawOnlyTest
void test_roll_only_settles_without_cross_axis_leak() { runSingleAxisTest(0); }
void test_pitch_only_settles_without_cross_axis_leak() { runSingleAxisTest(1); }
void test_yaw_only_settles_without_cross_axis_leak() { runSingleAxisTest(2); }

// IntegratedControllerFullTest: distinct per-channel disturbances so a
// cross-wired axis can't pass by symmetry. 0 -> 5 m climb at the same time.
void test_full_all_axes_and_altitude_settle()
{
    SimResult r = runClosedLoop(Eigen::Vector3f(20.0f, -15.0f, 10.0f), 0.0f, 5.0f);

    for (int i = 0; i < 3; ++i)
        TEST_ASSERT_TRUE(r.maxAbsAngleAfterSettle[i] < kAngleTol);
    TEST_ASSERT_TRUE(r.maxAltErrAfterSettle < kAltTol);
}

// AltitudeHoldTest through the full class: climbs 0 -> 5 m, stays settled,
// never commands thrust outside [0, maxThrust], attitude channels stay quiet.
void test_altitude_only_climbs_within_thrust_limits()
{
    SimResult r = runClosedLoop(Eigen::Vector3f::Zero(), 0.0f, 5.0f);

    TEST_ASSERT_TRUE(r.maxAltErrAfterSettle < kAltTol);
    TEST_ASSERT_TRUE(r.minThrust >= 0.0f);
    TEST_ASSERT_TRUE(r.maxThrust <= kMaxThrust);
    for (int i = 0; i < 3; ++i)
        TEST_ASSERT_FLOAT_WITHIN(kLeakEps, 0.0f, r.maxAbsTorque[i]);
}

// AltitudeHoldTest.rejectsDownwardDisturbanceWhileHovering
void test_altitude_recovers_from_downward_disturbance()
{
    SimResult r = runClosedLoop(Eigen::Vector3f::Zero(), 3.0f, 5.0f);
    TEST_ASSERT_TRUE(r.maxAltErrAfterSettle < kAltTol);
}

// Normalized roll torque must reach motors via mix-matrix column 1
// ([+1,-1,-1,+1]): negative roll torque -> motors 0,3 below motors 1,2.
void test_roll_torque_mixes_with_expected_signs()
{
    AttitudeAltitudeController ctrl = makeController();
    Setpoints sp;
    ControllerMeasurements meas;
    meas.angle[0] = 10.0f; // positive roll -> negative corrective torque

    ControllerOutput out = ctrl.update(sp, meas);
    TEST_ASSERT_TRUE(out.torque[0] < 0.0f);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, out.motorCmds[0], out.motorCmds[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, out.motorCmds[1], out.motorCmds[2]);
    TEST_ASSERT_TRUE(out.motorCmds[0] < out.motorCmds[1]);
}

// reset() must clear all 7 PIDs (integrators + D-suppression flag): after a
// dirty run + reset, one update must match a fresh controller exactly.
void test_reset_matches_fresh_controller()
{
    AttitudeAltitudeController used = makeController();
    AttitudeAltitudeController fresh = makeController();
    Setpoints sp;

    ControllerMeasurements dirty;
    dirty.angle << 20.0f, -10.0f, 5.0f;
    dirty.rate << 30.0f, -20.0f, 10.0f;
    dirty.altitude = 0.5f;
    for (int k = 0; k < 500; ++k)
        used.update(sp, dirty);

    used.reset();

    ControllerMeasurements probe;
    probe.angle << 5.0f, 2.0f, -3.0f;
    probe.rate << 5.0f, -2.0f, 1.0f;
    probe.altitude = -0.2f;

    ControllerOutput a = used.update(sp, probe);
    ControllerOutput b = fresh.update(sp, probe);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, b.throttle, a.throttle);
    for (int i = 0; i < 3; ++i)
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, b.torque[i], a.torque[i]);
    for (int m = 0; m < 4; ++m)
        TEST_ASSERT_FLOAT_WITHIN(1e-6f, b.motorCmds[m], a.motorCmds[m]);
}

int main()
{
    UNITY_BEGIN();
    RUN_TEST(test_zero_error_gives_hover_and_equal_motors);
    RUN_TEST(test_roll_only_settles_without_cross_axis_leak);
    RUN_TEST(test_pitch_only_settles_without_cross_axis_leak);
    RUN_TEST(test_yaw_only_settles_without_cross_axis_leak);
    RUN_TEST(test_full_all_axes_and_altitude_settle);
    RUN_TEST(test_altitude_only_climbs_within_thrust_limits);
    RUN_TEST(test_altitude_recovers_from_downward_disturbance);
    RUN_TEST(test_roll_torque_mixes_with_expected_signs);
    RUN_TEST(test_reset_matches_fresh_controller);
    return UNITY_END();
}