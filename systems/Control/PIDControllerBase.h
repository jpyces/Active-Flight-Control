#pragma once

namespace gnc
{
    // 1D PID controller with D-on-measurement and conditional-integration
    // anti-windup. One instance per control loop (rate/angle/altitude) -- each
    // loop is the same class, differing only in gains/bounds and what
    // setpoint/measurement pair it's wired to.
    class PIDControllerBase
    {
    public:
        PIDControllerBase(float kp, float ki, float kd, float outMin, float outMax);
        PIDControllerBase(const PIDControllerBase &) = delete;
        PIDControllerBase &operator=(const PIDControllerBase &) = delete;

        // Runs one PID step and returns the saturated output.
        //
        // D-on-measurement, not D-on-error: differentiating (setpoint -
        // measurement) spikes whenever the setpoint itself jumps (a new
        // attitude command, a stick input) -- a physically meaningless
        // "derivative kick." Differentiating the measurement instead avoids
        // that, since a physical state can't jump instantaneously. D is
        // suppressed entirely on the first call (no prevMeasurement yet).
        //
        // Anti-windup via conditional integration: the integral only
        // accumulates when the output isn't saturated in the direction this
        // step's error is pushing -- checked against the real outMin/outMax
        // bounds, not the output's sign, so this stays correct for
        // asymmetric bounds (e.g. a [0,1] throttle trim).
        float update(float setpoint, float measurement, float dt);

        // Clears integral/prevMeasurement/initialized. Call on state-machine
        // transitions (e.g. DISARMED->ARMED, an Arming Request) so stale
        // derivative/integral state doesn't leak across a rearm.
        void reset();
        
        // Getters
        float kp() const;
        float ki() const;
        float kd() const;
        float outMin() const;
        float outMax() const;

    private:
        // Bundles the clamped value with which bound (if any) it hit. A
        // struct return, rather than the clamped value plus two bool&
        // out-params, so a call site can't mismatch argument order/count or
        // leave a flag unset on some path -- both actual bugs the MATLAB
        // version's equivalent helper hit historically.
        struct SaturationResult
        {
            float value;
            bool atUpper;
            bool atLower;
        };

        // Clamps value to [outMin, outMax] and reports which bound (if any)
        // was hit, by direct comparison against the bounds -- not inferred
        // from the output's sign, which silently breaks for asymmetric
        // ranges like [0,1].
        static SaturationResult saturate(float value, float outMin, float outMax);

        float m_Kp;
        float m_Ki;
        float m_Kd;
        float m_OutMin;
        float m_OutMax;

        float m_Integral;
        float m_PrevMeasurement;
        bool m_Initialized;
    };
}
