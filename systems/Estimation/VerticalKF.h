#pragma once
#include "eigen.h"

namespace gnc
{
    class VerticalKF
    {
    public:
        VerticalKF(const Eigen::Matrix3f &Q, float R, const Eigen::Vector3f &x0, const Eigen::Matrix3f &P0);

        void reset(const Eigen::Vector3f &x0, const Eigen::Matrix3f &P0);
        void predict(float aMeas, float dt);
        void correct(float zMeas);

        float altitude() const;
        float velocity() const;
        float accelBias() const;
        Eigen::Vector3f state() const;
        Eigen::Matrix3f covariance() const;

    private:
        Eigen::Vector3f m_x;
        Eigen::Matrix3f m_P;
        Eigen::Matrix3f m_Q;
        float m_R;
    };
}