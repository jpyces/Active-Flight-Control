#pragma once

namespace gnc
{
    // 1D/3-state linear Kalman filter on the vertical channel: fuses an
    // accelerometer (prediction/control input) with barometer-derived
    // altitude (correction/measurement) into state [altitude; velocity;
    // accelBias]. Linear, not an EKF -- the accel input arrives already
    // rotated into the world frame upstream (Madgwick owns that
    // nonlinearity) and the barometer's pressure->altitude conversion also
    // happens upstream, so what's left here is genuinely x_{k+1}=Fx_k+Bu_k,
    // z_k=Hx_k+v_k.
    
    // When kalman fitler needs full linear algebra, will do it then
    // Implementation derived from general kalman filter white papers
    class VerticalKF
    {
    public:
        // Q: 3x3 process noise. R: scalar barometer measurement variance
        // (altitude-domain, m^2). x0: initial [altitude; velocity; accelBias].
        // P0: initial 3x3 covariance.
        VerticalKF(const float Q[3][3], float R, const float x0[3], const float P0[3][3]);

        // Time update (predict): propagates state/covariance forward using
        // the accelerometer reading as the control input. Run once every
        // cycle, before correct() -- per StateEstimator's contract, predict()
        // runs every call regardless of barometer status.
        void predict(float aMeas, float dt);

        // Measurement update (correct): corrects the predicted state against
        // a barometer-derived altitude reading. Call after predict(), and
        // only when the reading is actually valid this cycle -- gating on
        // sensor status is the caller's job (e.g. StateEstimator checking
        // baroStatus == NOMINAL), not this class's.
        void correct(float zMeas);

        // Reinitializes state/covariance. Call on the same state-machine
        // transitions PIDControllerBase::reset()/AttitudeAltitudeController::reset()
        // already hook into.
        void reset(const float x0[3], const float P0[3][3]);

        float altitude() const;  // x[0]
        float velocity() const;  // x[1]
        float accelBias() const; // x[2]

        // Read-only access to the full state/covariance, for tests and any
        // caller that needs more than the three named accessors above.
        void state(float x[3]) const;
        void covariance(float P[3][3]) const;

    private:
        float m_x[3];
        float m_P[3][3];
        float m_Q[3][3];
        float m_R;
    };
}
