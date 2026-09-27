#pragma once

namespace gnc
{
    // Decides "on the ground right now" for the FSM's landed input.
    //
    // Landed requires ALL of these, continuously for holdTimeS:
    //   - altitude within altitudeBandM of the ground reference
    //   - |vertical velocity| below maxVerticalSpeedMps
    //   - commanded throttle below maxThrottleFraction of maxThrust
    // Any condition failing clears landed on the same tick.
    //
    // The throttle condition is what separates a touchdown from a low hover:
    // a hover near the ground passes the altitude and speed checks but needs
    // roughly hover throttle, so maxThrottleFraction must sit below the hover
    // fraction. The FSM acts on the rising edge of landed, so a fast clear on
    // liftoff and a slow, held set on touchdown is the safe asymmetry.
    struct TouchdownConfig
    {
        float altitudeBandM;       // m, |altitude - groundRef| allowed while landed
        float maxVerticalSpeedMps; // m/s, |verticalVelocity| allowed while landed
        float maxThrottleFraction; // throttle / maxThrust allowed while landed (< hover fraction)
        float holdTimeS;           // s, all conditions must hold this long to set landed
    };

    class TouchdownDetector
    {
    public:
        explicit TouchdownDetector(const TouchdownConfig &cfg);

        // Ground altitude from the estimator, captured at arming. Until set,
        // the reference is 0 m (the baro's boot reference).
        void setGroundReference(float altitudeM) { m_groundRef = altitudeM; }

        // throttleFraction: last commanded throttle / maxThrust, in [0, 1].
        // Returns landed().
        bool update(float altitude, float verticalVelocity, float throttleFraction, float dt);

        // Starts true: the vehicle boots on the ground.
        bool landed() const { return m_landed; }

        float groundReference() const { return m_groundRef; }

    private:
        TouchdownConfig m_cfg;
        float m_groundRef{0.0f};
        float m_heldTime{0.0f}; // s all conditions have held continuously
        bool m_landed{true};
    };
}
