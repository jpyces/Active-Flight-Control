#include "PIDControllerBase.h"

namespace gnc
{
    PIDControllerBase::PIDControllerBase(float kp, float ki, float kd, float outMin, float outMax)
        : m_Kp(kp), m_Ki(ki), m_Kd(kd), m_OutMin(outMin), m_OutMax(outMax),
          m_Integral(0.0f), m_PrevMeasurement(0.0f), m_Initialized(false)
    {
    }

    PIDControllerBase::SaturationResult PIDControllerBase::saturate(float value, float outMin, float outMax)
    {
        if (value >= outMax)
        {
            return {outMax, /*atUpper=*/true, /*atLower=*/false};
        }
        if (value <= outMin)
        {
            return {outMin, /*atUpper=*/false, /*atLower=*/true};
        }
        return {value, /*atUpper=*/false, /*atLower=*/false};
    }

    float PIDControllerBase::update(float setpoint, float measurement, float dt)
    {
        float error = setpoint - measurement;

        // Tentative integral (Riemann sum) -- only committed below if this
        // step isn't saturated in the direction the error is pushing.
        float tentativeIntegral = m_Integral + error * dt;

        // D-on-measurement, suppressed on the very first call: there's no
        // prevMeasurement yet to difference against.
        float derivative = 0.0f;
        if (m_Initialized)
        {
            derivative = -(measurement - m_PrevMeasurement) / dt;
        }

        float rawOutput = m_Kp * error + m_Ki * tentativeIntegral + m_Kd * derivative;

        SaturationResult saturation = saturate(rawOutput, m_OutMin, m_OutMax);
        float output = saturation.value;

        // Conditional integration: freeze the integral only when saturated
        // AND the error is still pushing further into that same bound.
        bool pushingIntoSaturation = (saturation.atUpper && error > 0.0f) ||
                                     (saturation.atLower && error < 0.0f);
        if (!pushingIntoSaturation)
        {
            m_Integral = tentativeIntegral;
        }

        m_PrevMeasurement = measurement;
        m_Initialized = true;

        return output;
    }

    void PIDControllerBase::reset()
    {
        m_Integral = 0.0f;
        m_PrevMeasurement = 0.0f;
        m_Initialized = false;
    }

    float PIDControllerBase::kp() const { return m_Kp; }
    float PIDControllerBase::ki() const { return m_Ki; }
    float PIDControllerBase::kd() const { return m_Kd; }
    float PIDControllerBase::outMin() const { return m_OutMin; }
    float PIDControllerBase::outMax() const { return m_OutMax; }
}
