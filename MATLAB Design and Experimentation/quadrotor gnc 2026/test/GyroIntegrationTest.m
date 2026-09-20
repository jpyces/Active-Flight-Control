% GYROINTEGRATIONDEMO
%   Milestone 0 step 3: gyro-only quaternion integration, no correction.
%   Tracks attitude from angular velocity alone -- will drift (expected;
%   that's what motivates the complementary filter in step 4).
%
%   Same math as before, now calling the modular integrateGyroStep(q,
%   omega, dt) function each iteration instead of inlining the four
%   integration lines -- so any future script (or the complementary
%   filter itself, for its gyro-propagation half) can reuse this step
%   by passing in whatever quaternion/omega/dt it has, rather than
%   copy-pasting the loop body.

% --- Simulated gyro readings (rad/s), held constant so the total motion
% is a single fixed-axis rotation with a known closed-form answer to
% check against below.
omega = [3, 4, 2];

% --- Timestep / duration. Total simulated time = nIterations*dt.
dt = 0.01;
nIterations = 100;

% --- Attitude state, starts at identity (no rotation yet).
q = Quaternion(1, 0, 0, 0);

for i = 1:nIterations
    q = integrateGyroStep(q, omega, dt);
end

% --- Verify against the closed-form answer ---------------------------
% omega is constant here, so the axis of rotation never changes for the
% whole run (this ONLY holds because the gyro rate is fixed -- with a
% time-varying omega there's no single closed-form angle to check
% against, you'd need a numerical reference instead). That means the
% true total rotation angle is exactly |omega|*time, independent of dt.
totalTime = nIterations * dt;
expectedAngle = norm(omega) * totalTime;

% A quaternion only ever encodes the SHORTEST-path angle, in [0,pi] --
% this is the double-cover property (q and -q represent the same
% orientation), and it means a 308 deg rotation and a 52 deg rotation
% the other way are indistinguishable in the final q. Wrap
% expectedAngle the same way before comparing, or this falsely
% "mismatches" for any total rotation > 180 deg even when the
% integration is working correctly.
expectedAngleWrapped = mod(expectedAngle, 2*pi);
if expectedAngleWrapped > pi
    expectedAngleWrapped = 2*pi - expectedAngleWrapped;
end

% Extracting the angle back out relies on q = [cos(theta/2), axis*sin(theta/2)]
% -- so w = cos(theta/2), and theta = 2*acos(w). abs() because q and -q
% are the same rotation and acos needs its input in [-1,1] (clamped via
% min() to guard floating-point overshoot past 1).
actualAngle = 2 * acos(min(1, abs(q.w)));

fprintf('Expected angle, raw:     %.4f rad (%.2f deg)\n', expectedAngle, rad2deg(expectedAngle));
fprintf('Expected angle, wrapped: %.4f rad (%.2f deg)\n', expectedAngleWrapped, rad2deg(expectedAngleWrapped));
fprintf('Actual angle:            %.4f rad (%.2f deg)\n', actualAngle, rad2deg(actualAngle));
fprintf('Difference (wrapped):    %.6f rad\n', abs(expectedAngleWrapped - actualAngle));
disp(q);