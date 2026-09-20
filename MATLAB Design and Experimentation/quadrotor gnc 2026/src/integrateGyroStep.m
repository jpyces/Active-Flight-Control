function q_new = integrateGyroStep(q, omega, dt)
%INTEGRATEGYROSTEP One Euler-integration step of gyro-only quaternion
%attitude propagation (Milestone 0, step 3: no accel/mag correction --
%this will drift over time, which is expected and is what motivates the
%complementary filter in step 4).
%
%   q_new = integrateGyroStep(q, omega, dt)
%
%   Inputs:
%     q     - Quaternion, current attitude estimate
%     omega - 1x3 or 3x1 vector, body-frame angular velocity (rad/s)
%     dt    - scalar, timestep (s)
%
%   Output:
%     q_new - Quaternion, propagated attitude estimate (unit norm)
%
%   Method: qdot = 0.5*q*omega_quat, where omega_quat is omega packed as
%   a pure quaternion (w=0). qdot is q's derivative, not a rotation --
%   it depends on the current attitude as well as omega. Scaling by dt
%   gives a first-order (Euler) approximation of the small rotation
%   over this step; it's added (not composed via multiply) because it's
%   an increment, not a proper unit-norm rotation quaternion in its own
%   right. The addition doesn't preserve unit length, so the result is
%   renormalized before returning.

    omega_quat = Quaternion(0, omega(1), omega(2), omega(3));
    q_dot = 0.5 * q * omega_quat;
    q_rot = q_dot * dt;
    q_new = q + q_rot;
    q_new = q_new.normalize();
end