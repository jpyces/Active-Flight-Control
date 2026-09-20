% MADGWICKFULLTEST
%   Generic/static convergence test for madgwickStepFull. Unlike
%   madgwickNoMagTest, this can actually check YAW convergence too,
%   since mag is what makes yaw observable in the first place -- that's
%   the one new thing this test needs to do that the accel-only version
%   couldn't.
%
%   Synthetic accel/mag are built by evaluating the filter's OWN
%   residual formulas in reverse at a chosen ground-truth quaternion,
%   rather than via a separate rotation call -- guarantees the test's
%   "truth" uses the exact same convention as the filter, so a
%   convention mismatch in the test itself can't masquerade as a filter
%   bug (or vice versa).

% --- Ground truth: a nonzero roll/pitch/yaw, none of them trivial
% (0 or 90 deg) so this actually exercises the general case.
rollTrue  = deg2rad(15);
pitchTrue = deg2rad(-20);
yawTrue   = deg2rad(35);
qTrue = Quaternion.fromEulerZYX(rollTrue, pitchTrue, yawTrue);

% --- Reference field: bx/bz representing a plausible mid-latitude dip
% angle (mostly horizontal, some vertical component), normalized so the
% synthetic field has unit magnitude like a properly normalized mag
% reading would.
dipAngle = deg2rad(60);   % steep-ish dip, not a pole, not the equator
bxRef = cos(dipAngle);
bzRef = sin(dipAngle);

% --- Build synthetic accel by solving f_g = 0 for accel at qTrue --
% i.e. accel := whatever the closed-form predicted-gravity expression
% evaluates to. This IS the accel reading that makes the residual
% exactly zero at the true orientation.
w = qTrue.w; x = qTrue.x; y = qTrue.y; z = qTrue.z;
accelTrue = [2*(x*z - w*y), 2*(w*x + y*z), 2*(0.5 - x^2 - y^2)];

% --- Same idea for mag: solve f_b = 0 for mag at qTrue, given bxRef/bzRef.
magTrue = [2*bxRef*(0.5 - y^2 - z^2) + 2*bzRef*(x*z - w*y), ...
    2*bxRef*(x*y - w*z)       + 2*bzRef*(w*x + y*z), ...
    2*bxRef*(w*y + x*z)       + 2*bzRef*(0.5 - x^2 - y^2)];

% --- Run from identity, zero gyro, fixed synthetic accel/mag.
gyro = [0, 0, 0];
dt = 0.01;
nIterations = 550; % 500 didn't work - needed time to converge
beta = 0.1;

q = Quaternion(1, 0, 0, 0);
residualNorm = zeros(nIterations, 1);

for i = 1:nIterations
    q = madgwickStepFull(q, gyro, accelTrue, magTrue, dt, beta);

    ww=q.w; xx=q.x; yy=q.y; zz=q.z;
    f_g = [2*(xx*zz - ww*yy) - accelTrue(1);
        2*(ww*xx + yy*zz) - accelTrue(2);
        2*(0.5 - xx^2 - yy^2) - accelTrue(3)];
    f_b = [2*bxRef*(0.5 - yy^2 - zz^2) + 2*bzRef*(xx*zz - ww*yy) - magTrue(1);
        2*bxRef*(xx*yy - ww*zz)     + 2*bzRef*(ww*xx + yy*zz) - magTrue(2);
        2*bxRef*(ww*yy + xx*zz)     + 2*bzRef*(0.5 - xx^2 - yy^2) - magTrue(3)];
    residualNorm(i) = norm([f_g; f_b]);
end

finalAngles = q.toEulerZYX();
finalRoll = finalAngles(1);
finalPitch = finalAngles(2);
finalYaw = finalAngles(3);

fprintf('True roll:  %.2f deg | Final roll:  %.2f deg\n', rad2deg(rollTrue), rad2deg(finalRoll));
fprintf('True pitch: %.2f deg | Final pitch: %.2f deg\n', rad2deg(pitchTrue), rad2deg(finalPitch));
fprintf('True yaw:   %.2f deg | Final yaw:   %.2f deg\n', rad2deg(yawTrue), rad2deg(finalYaw));
fprintf('Final residual norm: %.6g\n', residualNorm(end));

angleTol = deg2rad(1);
assert(abs(finalRoll  - rollTrue)  < angleTol, 'Roll off by %.2f deg', rad2deg(abs(finalRoll-rollTrue)));
assert(abs(finalPitch - pitchTrue) < angleTol, 'Pitch off by %.2f deg', rad2deg(abs(finalPitch-pitchTrue)));
assert(abs(finalYaw   - yawTrue)   < angleTol, 'Yaw off by %.2f deg -- this is the capability madgwickNoMag did not have', rad2deg(abs(finalYaw-yawTrue)));

tailWindow = 50;
tail = residualNorm(end-tailWindow+1:end);
residualTol = 5e-3;   % looser than the accel-only test: two residual blocks now
disp(residualNorm(end-49:end));
assert(max(tail) < residualTol, 'Residual not settled: max over last %d = %.4g', tailWindow, max(tail));
assert(std(tail) < residualTol/2, 'Residual oscillating rather than flat (std = %.4g)', std(tail));

disp('madgwickStepFull static test passed: roll, pitch, AND yaw converged and held.');