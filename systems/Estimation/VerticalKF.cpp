#include "VerticalKF.h"

namespace gnc
{
    VerticalKF::VerticalKF(const Eigen::Matrix3f &Q, float R, const Eigen::Vector3f &x0, const Eigen::Matrix3f &P0)
        : m_x(x0), m_P(P0), m_Q(Q), m_R(R) {}

    void VerticalKF::reset(const Eigen::Vector3f &x0, const Eigen::Matrix3f &P0)
    {
        m_x = x0;
        m_P = P0;
    }

    void VerticalKF::predict(float aMeas, float dt)
    {
        Eigen::Matrix3f F;
        F << 1.0f, dt, -0.5f * dt * dt,
            0.0f, 1.0f, -dt,
            0.0f, 0.0f, 1.0f;

        Eigen::Vector3f B(0.5f * dt * dt, dt, 0.0f);

        m_x = F * m_x + B * aMeas;
        m_P = F * m_P * F.transpose() + m_Q;
    }

    void VerticalKF::correct(float zMeas)
    {
        Eigen::RowVector3f H(1.0f, 0.0f, 0.0f);

        float Hx = H * m_x;
        Eigen::Vector3f PHt = m_P * H.transpose();
        float S = PHt(0) + m_R;
        Eigen::Vector3f K = PHt / S;

        float innovation = zMeas - Hx;
        m_x += K * innovation;

        Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f IKH = I - K * H;
        m_P = IKH * m_P * IKH.transpose() + K * m_R * K.transpose();
    }

    float VerticalKF::altitude() const { return m_x(0); }
    float VerticalKF::velocity() const { return m_x(1); }
    float VerticalKF::accelBias() const { return m_x(2); }
    Eigen::Vector3f VerticalKF::state() const { return m_x; }
    Eigen::Matrix3f VerticalKF::covariance() const { return m_P; }
}