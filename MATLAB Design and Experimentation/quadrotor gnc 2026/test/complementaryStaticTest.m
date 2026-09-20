% COMPLEMENTARYSTATICTEST
%   Static convergence check for complementaryFilterStep, mirroring
%   madgwickStaticTest.m's structure and using the SAME tilted accel
%   vector deliberately -- so the two results are directly comparable,
%   not just each individually passing/failing on their own terms. If
%   you run both scripts, finalRoll/finalPitch here should land close
%   to madgwickNoMag's, since both are converging toward the same
%   physical answer via different math.

% --- Fixed inputs. gyro = 0 isolates the accel-correction branch
% entirely, same reasoning as the Madgwick static test.
gyro = [0, 0, 0];
dt = 0.01;
nIterations = 500;
alpha = 0.98;   % gyro branch weight; accel branch gets (1 - alpha)

% --- Same tilt vector as madgwickStaticTest.m -- not straight up, so
% the filter actually has to correct a nonzero tilt, not converge
% trivially from an identity start.
accelRaw = [0.3, -0.2, 0.9];
accel = accelRaw / norm(accelRaw);

% --- Same independent closed-form check as the Madgwick test: roll and
% pitch computed directly from the accel vector, without touching the
% filter at all.
expectedRoll  = atan2(accel(2), accel(3));
expectedPitch = atan2(-accel(1), sqrt(accel(2)^2 + accel(3)^2));

% --- Attitude state, starts at identity.
q = Quaternion(1, 0, 0, 0);

% --- Track a tilt-error metric each step for the stability check
% below. Reuses the same closed-form gravity residual Madgwick's
% correction is built from -- it's a valid generic "how far off is the
% predicted gravity direction" measure regardless of which algorithm
% produced q, which makes this directly comparable to
% madgwickStaticTest's residual log even though this filter doesn't
% use this expression internally.
residualNorm = zeros(nIterations, 1);

for i = 1:nIterations
    q = complementaryFilterStep(q, gyro, accel, dt, alpha);

    f = [2*(q.x*q.z - q.w*q.y) - accel(1);
        2*(q.w*q.x + q.y*q.z) - accel(2);
        2*(0.5 - q.x^2 - q.y^2)  - accel(3)];
    residualNorm(i) = norm(f);
end

% --- Pull roll/pitch/yaw back out. Same order assumption as
% madgwickStaticTest.m -- adjust if your toEulerZYX differs.
finalAngles = q.toEulerZYX();
finalRoll  = finalAngles(1);
finalPitch = finalAngles(2);
finalYaw   = finalAngles(3);

fprintf('Expected roll:  %.4f rad (%.2f deg)\n', expectedRoll, rad2deg(expectedRoll));
fprintf('Final roll:     %.4f rad (%.2f deg)\n', finalRoll, rad2deg(finalRoll));
fprintf('Expected pitch: %.4f rad (%.2f deg)\n', expectedPitch, rad2deg(expectedPitch));
fprintf('Final pitch:    %.4f rad (%.2f deg)\n', finalPitch, rad2deg(finalPitch));
fprintf('Final yaw:      %.4f rad (%.2f deg) -- unobservable from accel alone, should stay ~0 with zero gyro\n', finalYaw, rad2deg(finalYaw));
fprintf('Final residual norm: %.6g\n', residualNorm(end));

% --- Convergence check.
angleTol = deg2rad(1);
assert(abs(finalRoll  - expectedRoll)  < angleTol, ...
    'Roll did not converge: off by %.2f deg', rad2deg(abs(finalRoll - expectedRoll)));
assert(abs(finalPitch - expectedPitch) < angleTol, ...
    'Pitch did not converge: off by %.2f deg', rad2deg(abs(finalPitch - expectedPitch)));

% --- Stability check over the tail, same pattern as
% madgwickStaticTest.m. Note: this filter's blend mechanics differ
% from Madgwick's fixed-magnitude gradient step, so its steady-state
% residual floor may land at a different value -- don't assume the
% same tolerance necessarily transfers; if this fails, look at the
% actual plateau value before concluding something's wrong.
tailWindow = 50;
tail = residualNorm(end-tailWindow+1:end);
residualTol = 3e-3;
assert(max(tail) < residualTol, ...
    'Residual not settled: max over last %d steps = %.4g (tol %.4g)', ...
    tailWindow, max(tail), residualTol);
assert(std(tail) < residualTol/2, ...
    'Residual is noisy/oscillating over the tail (std = %.4g) rather than flat', std(tail));

disp('Complementary filter static test passed: converged to expected tilt and held steady.');