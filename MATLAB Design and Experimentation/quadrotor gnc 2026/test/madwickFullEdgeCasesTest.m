% MADGWICKFULLEDGECASESTEST
%   Edge-case battery for madgwickStepFull, beyond the generic static
%   test. Each case is a scenario a real sensor stack will actually hit
%   at some point (a glitched reading, a pole-like field, unit
%   mismatches) -- these are the failure modes that a single "does it
%   converge on a clean bench reading" test won't surface.

gyro = [0, 0, 0];
dt = 0.01;
beta = 0.1;
qIdentity = Quaternion(1, 0, 0, 0);
accelLevel = [0, 0, 1];   % consistent with identity: level, right-side up
magLevelRef = [0.6, 0, 0.8];  % some arbitrary consistent-with-identity field

fprintf('--- Case 1: zero-norm accel (sensor glitch) ---\n');
% A zero reading should mean "this sensor's data is unusable this
% step," not "gravity points nowhere" -- the correction from this
% branch should be suppressed, not fed a phantom target.
qNext = madgwickStepFull(qIdentity, gyro, [0,0,0], magLevelRef, dt, beta);
angleFromIdentity = 2*acos(min(1, abs(qNext.w)));
fprintf('Angle moved from identity with zero accel, zero gyro: %.4f rad (%.2f deg)\n', ...
    angleFromIdentity, rad2deg(angleFromIdentity));
% KNOWN ISSUE: as currently written, resetting accel to [0,0,0] does
% NOT suppress the gravity correction -- f_g evaluates to [0;0;1] at
% identity (a full-magnitude residual, not zero), so this case injects
% a spurious ~1-radian-scale correction from nothing. Expect this
% assert to FAIL until that's fixed. See message for the fix.
assert(angleFromIdentity < deg2rad(1), ...
    ['Zero-norm accel should not move the estimate (nothing happened), ' ...
    'but it moved %.2f deg -- accel=[0,0,0] is being treated as a real ' ...
    'reading instead of a skipped one.'], rad2deg(angleFromIdentity));

fprintf('--- Case 2: zero-norm mag (sensor glitch) ---\n');
% This one SHOULD already be safe: mag=[0,0,0] rotates to h=[0,0,0],
% so bx=bz=0, so f_b evaluates to exactly [0;0;0] -- no spurious target,
% correctly contributes nothing. Cross-check against madgwickNoMag on
% the same tilt to confirm mag-zero behaves identically to no-mag.
accelTilt = [0.3, -0.2, 0.9] / norm([0.3, -0.2, 0.9]);
qFull_noMag  = madgwickStepFull(qIdentity, gyro, accelTilt, [0,0,0], dt, beta);
qAccelOnly   = madgwickNoMag(qIdentity, gyro, accelTilt, dt, beta);
diffAngle = 2*acos(min(1, abs(qFull_noMag.w*qAccelOnly.w + qFull_noMag.x*qAccelOnly.x + ...
    qFull_noMag.y*qAccelOnly.y + qFull_noMag.z*qAccelOnly.z)));
fprintf('Difference between madgwickStepFull(mag=0) and madgwickNoMag: %.6f rad\n', diffAngle);
assert(diffAngle < 1e-6, 'Zero mag should behave identically to no mag at all -- got %.6g rad difference', diffAngle);

fprintf('--- Case 3: near-pole magnetic field (bx ~ 0, yaw unobservable) ---\n');
% Purely vertical field -- physically what you'd read near a magnetic
% pole. Yaw genuinely can't be corrected from this (not a bug, a real
% physical limit), so only check: no NaN, and roll/pitch still converge
% even though yaw won't.
rollT = deg2rad(10); pitchT = deg2rad(-10); yawT = deg2rad(50);
qT = Quaternion.fromEulerZYX(rollT, pitchT, yawT);
w=qT.w; x=qT.x; y=qT.y; z=qT.z;
accelPole = [2*(x*z-w*y), 2*(w*x+y*z), 2*(0.5-x^2-y^2)];
bxPole = 0; bzPole = 1;  % purely vertical reference field
magPole = [2*bxPole*(0.5-y^2-z^2)+2*bzPole*(x*z-w*y), ...
    2*bxPole*(x*y-w*z)+2*bzPole*(w*x+y*z), ...
    2*bxPole*(w*y+x*z)+2*bzPole*(0.5-x^2-y^2)];
q = qIdentity;
for i = 1:500
    q = madgwickStepFull(q, gyro, accelPole, magPole, dt, beta);
end
assert(~any(isnan([q.w,q.x,q.y,q.z])), 'Pole-like magnetic field produced NaN -- bx=0 is breaking something');
finalAnglesPole = q.toEulerZYX();
fprintf('Near-pole case -- roll: %.2f deg (true %.2f), pitch: %.2f deg (true %.2f), yaw: %.2f deg (true %.2f, expected NOT to converge)\n', ...
    rad2deg(finalAnglesPole(1)), rad2deg(rollT), rad2deg(finalAnglesPole(2)), rad2deg(pitchT), ...
    rad2deg(finalAnglesPole(3)), rad2deg(yawT));
assert(abs(finalAnglesPole(1) - rollT) < deg2rad(1), 'Roll should still converge even with a degenerate mag field');
assert(abs(finalAnglesPole(2) - pitchT) < deg2rad(1), 'Pitch should still converge even with a degenerate mag field');

fprintf('--- Case 4: magnitude invariance (raw units shouldn''t matter) ---\n');
% accel/mag get normalized internally -- scaling the raw input by any
% positive factor should produce an IDENTICAL trajectory, since only
% direction should matter, never magnitude.
scale = 137.0;  % arbitrary, not 1, not a "nice" number
qUnscaled = qIdentity;
qScaled = qIdentity;
for i = 1:50
    qUnscaled = madgwickStepFull(qUnscaled, gyro, accelTilt, magLevelRef, dt, beta);
    qScaled   = madgwickStepFull(qScaled, gyro, accelTilt*scale, magLevelRef/7.3, dt, beta);
end
scaleDiff = 2*acos(min(1, abs(qUnscaled.w*qScaled.w + qUnscaled.x*qScaled.x + ...
    qUnscaled.y*qScaled.y + qUnscaled.z*qScaled.z)));
fprintf('Difference between unscaled and rescaled-input trajectories: %.6g rad\n', scaleDiff);
assert(scaleDiff < 1e-6, 'Filter output should not depend on raw sensor magnitude, only direction -- got %.6g rad difference', scaleDiff);

disp('All madgwickStepFull edge cases evaluated (see output above for pass/fail per case).');