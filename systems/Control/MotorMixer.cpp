#include "eigen.h"
#include "MotorMixer.h"

// MotorMixer.cpp
namespace gnc
{
    Eigen::Vector4f mixMotors(const Eigen::Vector4f &command, const Eigen::Matrix4f &mixMatrix)
    {
        Eigen::Vector4f motorCmds = mixMatrix * command;

        // shift: if any motor is negative, add the same offset to all four
        // (preserves roll/pitch/yaw differences)
        float lowest = motorCmds.minCoeff();
        if (lowest < 0.0f)
        {
            Eigen::Vector4f lowestVector(lowest, lowest, lowest, lowest);
            motorCmds = motorCmds - lowestVector;
        }

        // scale: if any motor now exceeds 1.0, divide all four by the same factor
        float highest = motorCmds.maxCoeff();
        if (highest > 1.0f)
        {
            motorCmds = motorCmds / highest;
        }

        return motorCmds;
    }
}