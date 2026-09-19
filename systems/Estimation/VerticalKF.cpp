#include "VerticalKF.h"

#include <cstring>

namespace gnc
{
    namespace
    {
        // Small fixed-size 3x3/3x1 matrix helpers, internal-linkage only --
        // matches the project's established anonymous-namespace pattern for
        // per-file helpers (e.g. Imu.cpp's kDegToRad).
        // Not full matrix library of functions because it is not necessary - not that much
        // matrix manipulation for project in current state and future UKF will be relatively
        // focused - would use Eigen library, but potentially uses dynamic allocations
        // which are not ideal for embedded systems such as this

        void matMul3x3(const float A[3][3], const float B[3][3], float result[3][3])
        {
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    result[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
                }
            }
        }

        void matMul3x3Vec(const float A[3][3], const float v[3], float result[3])
        {
            for (int i = 0; i < 3; ++i)
            {
                result[i] = A[i][0] * v[0] + A[i][1] * v[1] + A[i][2] * v[2];
            }
        }

        void transpose3x3(const float A[3][3], float result[3][3])
        {
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    result[j][i] = A[i][j];
                }
            }
        }

        void addAssign3x3(float A[3][3], const float B[3][3])
        {
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    A[i][j] += B[i][j];
                }
            }
        }

        void identity3x3(float result[3][3])
        {
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    result[i][j] = (i == j) ? 1.0f : 0.0f;
                }
            }
        }
    }

    VerticalKF::VerticalKF(const float Q[3][3], float R, const float x0[3], const float P0[3][3])
        : m_R(R)
    {
        std::memcpy(m_x, x0, sizeof(m_x));
        std::memcpy(m_P, P0, sizeof(m_P));
        std::memcpy(m_Q, Q, sizeof(m_Q));
    }

    void VerticalKF::reset(const float x0[3], const float P0[3][3])
    {
        std::memcpy(m_x, x0, sizeof(m_x));
        std::memcpy(m_P, P0, sizeof(m_P));
    }

    void VerticalKF::predict(float aMeas, float dt)
    {
        // State-transition matrix F: autonomous dynamics with no external
        // input (bias persists unchanged, position/velocity coast).
        // Row 3 MUST stay [0, 0, 1] -- see the paired B[2]=0 note below.
        // These are the basic equations of motion
        float F[3][3] = {
            {1.0f, dt, -0.5f * dt * dt},
            {0.0f, 1.0f, -dt},
            {0.0f, 0.0f, 1.0f}
        };

        // Control-input matrix B: how the accel measurement additionally
        // pushes the state on top of F's autonomous evolution.
        // B[2] MUST stay 0 -- accel has no direct path to the bias state.
        // These two facts (F row 3, B[2]) are a matched pair: correctness
        // requires BOTH simultaneously. Fixing only one either resets bias to
        // the raw accel reading every step (losing all memory) or lets it
        // drift from accel noise on every prediction -- a bug this exact
        // filter hit three separate times in the MATLAB reference by fixing
        // only one side at a time.
        float B[3] = {0.5f * dt * dt, dt, 0.0f};

        // x = F*x + B*aMeas
        float Fx[3];
        matMul3x3Vec(F, m_x, Fx);
        m_x[0] = Fx[0] + B[0] * aMeas;
        m_x[1] = Fx[1] + B[1] * aMeas;
        m_x[2] = Fx[2] + B[2] * aMeas;

        // P = F*P*F^T + Q
        float Ft[3][3];
        transpose3x3(F, Ft);
        float FP[3][3];
        matMul3x3(F, m_P, FP);
        float FPFt[3][3];
        matMul3x3(FP, Ft, FPFt);
        addAssign3x3(FPFt, m_Q);
        std::memcpy(m_P, FPFt, sizeof(m_P));
    }

    void VerticalKF::correct(float zMeas)
    {
        // H = [1, 0, 0] -- altitude-only measurement. Written out directly
        // rather than through a generic H-matrix multiply, since H is fixed
        // and this specific structure (picks out row/column 0) is what lets
        // several of the steps below collapse to a single scalar/column read
        // instead of a full 1x3 * 3x3 multiply.
        float Hx = m_x[0];                                // H * x
        float PHt[3] = {m_P[0][0], m_P[1][0], m_P[2][0]}; // P * H^T (column 0 of P)
        float S = PHt[0] + m_R;                           // H * P * H^T + R (PHt[0] == P[0][0] == H*P*H^T)

        // K = P * H^T / S
        float K[3] = {PHt[0] / S, PHt[1] / S, PHt[2] / S};

        // x = x + K * (z - H*x)
        float innovation = zMeas - Hx;
        m_x[0] += K[0] * innovation;
        m_x[1] += K[1] * innovation;
        m_x[2] += K[2] * innovation;

        // Joseph form: P = (I - K*H) * P * (I - K*H)^T + K * R * K^T.
        // Used instead of the simpler (I - K*H)*P for numerical robustness --
        // guarantees P stays symmetric/PSD even under floating-point roundoff.
        // K*H (3x3 outer product) collapses to column 0 = K, columns 1/2 = 0,
        // since H = [1,0,0].
        float KH[3][3] = {
            {K[0], 0.0f, 0.0f},
            {K[1], 0.0f, 0.0f},
            {K[2], 0.0f, 0.0f},
        };
        float I[3][3];
        identity3x3(I);
        float IKH[3][3];
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                IKH[i][j] = I[i][j] - KH[i][j];
            }
        }

        float IKHt[3][3];
        transpose3x3(IKH, IKHt);
        float IKH_P[3][3];
        matMul3x3(IKH, m_P, IKH_P);
        float newP[3][3];
        matMul3x3(IKH_P, IKHt, newP);

        // + K * R * K^T
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                newP[i][j] += K[i] * m_R * K[j];
            }
        }

        std::memcpy(m_P, newP, sizeof(m_P));
    }

    float VerticalKF::altitude() const { return m_x[0]; }
    float VerticalKF::velocity() const { return m_x[1]; }
    float VerticalKF::accelBias() const { return m_x[2]; }

    void VerticalKF::state(float x[3]) const
    {
        std::memcpy(x, m_x, sizeof(m_x));
    }

    void VerticalKF::covariance(float P[3][3]) const
    {
        std::memcpy(P, m_P, sizeof(m_P));
    }
}
