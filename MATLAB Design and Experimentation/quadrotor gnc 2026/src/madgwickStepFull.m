function q_new = madgwickStepFull(q, gyro, accel, mag, dt, beta)
% REFERENCE: https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/madgwick_internal_report.pdf
% At dt = 0.01 and nIterations = 550 calculated orientation converges to
% truth approximately - 5.5 seconds. 


%MADGWICKSTEPFULL One step of Madgwick's filter, accel+gyro+mag (Group 1
%only -- no gyro bias compensation; that's owned by the joint
%EKF/UKF later, not duplicated here). Adds a magnetometer-derived yaw
%correction on top of madgwickNoMag's gravity-only correction.
%
%   Inputs:
%     q     - Quaternion, current attitude estimate
%     gyro  - 1x3 vector, body-frame angular velocity (rad/s)
%     accel - 1x3 vector, raw accelerometer reading
%     mag   - 1x3 vector, raw (calibrated!) magnetometer reading --
%             this assumes hard/soft-iron calibration already applied
%             upstream; garbage in here corrupts yaw the same way
%             uncalibrated accel corrupts roll/pitch.
%     dt    - scalar, timestep (s)
%     beta  - scalar gain (same role as in madgwickNoMag)
%
%   Output:
%     q_new - Quaternion, corrected + propagated estimate (unit norm)

% --- Step 1: gyro-predicted derivative -- identical to madgwickNoMag.
    qdot_gyro = 0.5 * q * Quaternion(0, gyro(1), gyro(2), gyro(3));

% --- Step 2: normalize accel and mag to pure directions, tracking
% validity instead of substituting a zero vector for an invalid
% reading -- a zeroed accel/mag fed into the residual formulas isn't
% "no correction," it's a nonsensical target that produces a spurious,
% often large, correction (see the edge-case test for why).
    accelValid = norm(accel) > eps;
    if accelValid
        accel = accel / norm(accel);
    end

    magValid = norm(mag) > eps;
    if magValid
        mag = mag / norm(mag);
    end

% --- Step 3: gravity residual f_g, 3x1 -- unchanged when accel is
% valid; zeroed out (contributes nothing to the gradient) when it's not.
    if accelValid
        f_g = [2*(q.x*q.z - q.w*q.y) - accel(1);
               2*(q.w*q.x + q.y*q.z) - accel(2);
               2*(0.5 - q.x^2 - q.y^2) - accel(3)];
    else
        f_g = zeros(3,1);
    end

% --- Step 4: only need the world-frame field estimate at all if mag is
% actually valid -- skip the work otherwise, not just its result.
    if magValid
        magQuat = Quaternion(0, mag(1), mag(2), mag(3));
        h = q * magQuat * conjugate(q);
        bx = sqrt(h.x^2 + h.y^2);
        bz = h.z;
    end

% --- Step 5: magnetic residual f_b -- same pattern as f_g.
    if magValid
        f_b = [2*bx*(0.5 - q.y^2 - q.z^2) + 2*bz*(q.x*q.z - q.w*q.y) - mag(1);
               2*bx*(q.x*q.y - q.w*q.z)   + 2*bz*(q.w*q.x + q.y*q.z) - mag(2);
               2*bx*(q.w*q.y + q.x*q.z)   + 2*bz*(0.5 - q.x^2 - q.y^2) - mag(3)];
    else
        f_b = zeros(3,1);
    end

% --- Step 6: magnetic Jacobian J_b -- same pattern: zeroed when mag is
% invalid, so it contributes nothing to J'*f regardless (bx/bz aren't
% even defined in that branch, which is fine since J_b is never built
% from them when magValid is false).
    if magValid
        J_b = [ -2*bz*q.y,             2*bz*q.z,            -4*bx*q.y - 2*bz*q.w,  -4*bx*q.z + 2*bz*q.x;
                -2*bx*q.z + 2*bz*q.x,   2*bx*q.y + 2*bz*q.w,  2*bx*q.x + 2*bz*q.z,  -2*bx*q.w + 2*bz*q.y;
                 2*bx*q.y,              2*bx*q.z - 4*bz*q.x,  2*bx*q.w - 4*bz*q.y,   2*bx*q.x ];
    else
        J_b = zeros(3,4);
    end

% --- Step 7: J_g also needs the same treatment now -- previously
% always built unconditionally, but its rows need to vanish exactly
% when f_g's do, for the same reason.
    if accelValid
        J_g = [ -2*q.y,  2*q.z, -2*q.w,  2*q.x;
                 2*q.x,  2*q.w,  2*q.z,  2*q.y;
                 0,     -4*q.x, -4*q.y,  0];
    else
        J_g = zeros(3,4);
    end

    f = [f_g; f_b];
    J = [J_g; J_b];

% --- Step 8: gradient, normalize, zero-guard -- identical pattern
% to madgwickNoMag, just bigger matrices going in.
    gradient = J' * f;
    if norm(gradient) > eps
        gradient_hat = gradient / norm(gradient);
    else
        gradient_hat = zeros(4,1);
    end

% --- Step 9: blend, integrate, normalize -- identical to
% madgwickNoMag's steps 6-7.
    qdot_corrected = qdot_gyro - beta * Quaternion(gradient_hat(1), ...
        gradient_hat(2), gradient_hat(3), gradient_hat(4));
    q_rot = qdot_corrected * dt;
    q_new = normalize((q + q_rot));
end