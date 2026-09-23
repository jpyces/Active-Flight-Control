// MotorMixer.h
#pragma once
#include "eigen.h"

namespace gnc
{
    // command = [throttle, rollTorque, pitchTorque, yawTorque], all already
    // normalized to the same [0,1]/[-1,1] fractions AttitudeAltitudeController
    // produces before calling this
    // Returns 4 motor commands, each clamped into [0,1].
    Eigen::Vector4f mixMotors(const Eigen::Vector4f &command, const Eigen::Matrix4f &mixMatrix);
}