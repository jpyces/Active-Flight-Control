% CROSSCHECKCOMPLEMENTARYVSMADGWICK
%   Milestone 0 step 6 gate: run madgwickStepFull and
%   complementaryFilterStep over IDENTICAL synthetic bench-like motion
%   (not a static case -- both individual filters already have static
%   tests) and confirm they agree on roll/pitch. They're built from
%   different math (gradient descent vs. quaternion blend), so
%   agreement here is a real, independent signal that both are correct
%   -- not just that each one individually passes its own test.
%
%   Note: complementaryFilterStep has no magnetometer input, so this
%   can only validate roll/pitch agreement, not yaw (matches the
%   roadmap's step 6 scope). Precisely because madgwickStepFull DOES
%   have mag wired in, this comparison also doubles as a check that the
%   mag correction isn't leaking into roll/pitch the way a naive
%   quaternion blend could -- if it were, this test would catch it as a
%   roll/pitch mismatch between the two filters, even though each
%   passes its own static test individually.

% --- Synthetic "bench motion": smooth, bounded, two-axis rotation --
% oscillating rather than drifting off in one direction, so it stays
% well away from gimbal lock in the Euler angles used for comparison,
% and never actually stops (unlike the static tests), so this is
% genuinely exercising the predict step under real ongoing motion, not
% just the correction step at rest.
dt = 0.01;
totalTime = 10;              % seconds
nIterations = totalTime / dt;
rollFreq = 0.15;   rollAmp = deg2rad(20);
pitchFreq = 0.22;  pitchAmp = deg2rad(15);

t = (0:nIterations-1) * dt;
rollProfile  = rollAmp  * sin(2*pi*rollFreq*t);
pitchProfile = pitchAmp * sin(2*pi*pitchFreq*t);

% --- Build the TRUE reference trajectory by converting the Euler
% profile to quaternions directly (yaw held at 0 throughout --
% complementaryFilterStep has no way to track yaw, so keeping true yaw
% at 0 means a nonzero final yaw in either filter is itself diagnostic,
% not just noise to ignore).
qTrue = cell(nIterations, 1);
for i = 1:nIterations
    qTrue{i} = Quaternion.fromEulerZYX(rollProfile(i), pitchProfile(i), 0);
end

% --- Derive the gyro reading that exactly reproduces this trajectory:
% solve integrateGyroStep's own model backward rather than
% differentiating the Euler profile independently, so the synthetic
% gyro signal is exactly consistent with the SAME integration scheme
% both filters already use for their predict step (same reasoning as
% the residual-solving trick used to build synthetic accel/mag in the
% earlier static tests -- avoids a convention mismatch between the test
% and the filters becoming a false failure).
gyroProfile = zeros(nIterations, 3);
for i = 1:nIterations-1
    qNow = qTrue{i};
    qNext = qTrue{i+1};
    qDelta = qNow.conjugate() * qNext;   % rotation from step i to i+1, in body frame
    % qDelta ~= identity + 0.5*omega_quat*dt for a small step; solve for omega.
    omega = (2/dt) * [qDelta.x, qDelta.y, qDelta.z];
    gyroProfile(i, :) = omega;
end
gyroProfile(end, :) = gyroProfile(end-1, :);  % last sample, no i+1 to diff against

% --- Derive accel/mag consistent with qTrue at each step, same
% residual-solving approach as the static tests.
dipAngle = deg2rad(60);
bxRef = cos(dipAngle);
bzRef = sin(dipAngle);

accelProfile = zeros(nIterations, 3);
magProfile   = zeros(nIterations, 3);
for i = 1:nIterations
    w=qTrue{i}.w; x=qTrue{i}.x; y=qTrue{i}.y; z=qTrue{i}.z;
    accelProfile(i,:) = [2*(x*z - w*y), 2*(w*x + y*z), 2*(0.5 - x^2 - y^2)];
    magProfile(i,:)   = [2*bxRef*(0.5-y^2-z^2)+2*bzRef*(x*z-w*y), ...
                          2*bxRef*(x*y-w*z)+2*bzRef*(w*x+y*z), ...
                          2*bxRef*(w*y+x*z)+2*bzRef*(0.5-x^2-y^2)];
end

% --- Run both filters from IDENTITY (cold start, not seeded from
% truth) over the same synthetic gyro/accel/mag stream. Each filter
% only ever sees the sensor readings, never qTrue directly.
beta = 0.1;
alpha = 0.98;

qMadg = Quaternion(1,0,0,0);
qComp = Quaternion(1,0,0,0);

rollMadg = zeros(nIterations,1); pitchMadg = zeros(nIterations,1); yawMadg = zeros(nIterations,1);
rollComp = zeros(nIterations,1); pitchComp = zeros(nIterations,1); yawComp = zeros(nIterations,1);

for i = 1:nIterations
    qMadg = madgwickStepFull(qMadg, gyroProfile(i,:), accelProfile(i,:), magProfile(i,:), dt, beta);
    qComp = complementaryFilterStep(qComp, gyroProfile(i,:), accelProfile(i,:), dt, alpha);

    anglesMadg = qMadg.toEulerZYX();
    rollMadg(i) = anglesMadg(1); pitchMadg(i) = anglesMadg(2); yawMadg(i) = anglesMadg(3);

    anglesComp = qComp.toEulerZYX();
    rollComp(i) = anglesComp(1); pitchComp(i) = anglesComp(2); yawComp(i) = anglesComp(3);
end

% --- Skip the cold-start convergence transient (measured ~5.5s for
% Madgwick at beta=0.1 from a much larger initial error -- give both
% filters a comparable settling window here before comparing) rather
% than judging agreement while one or both are still converging.
settleTime = 6;   % seconds
settleSamples = round(settleTime / dt);
compareIdx = settleSamples:nIterations;

rollDiff  = abs(rad2deg(rollMadg(compareIdx)  - rollComp(compareIdx)));
pitchDiff = abs(rad2deg(pitchMadg(compareIdx) - pitchComp(compareIdx)));

fprintf('Comparing over t = %.1fs to %.1fs (post-settling window)\n', settleTime, totalTime);
fprintf('Roll  disagreement  -- max: %.3f deg, mean: %.3f deg\n', max(rollDiff), mean(rollDiff));
fprintf('Pitch disagreement -- max: %.3f deg, mean: %.3f deg\n', max(pitchDiff), mean(pitchDiff));
fprintf('Final Madgwick yaw: %.2f deg | Final complementary yaw: %.2f deg (true yaw = 0 throughout; complementary has no mag, expect its yaw to be unreliable)\n', ...
    rad2deg(yawMadg(end)), rad2deg(yawComp(end)));

% --- Gate: roll/pitch must agree within tolerance throughout the
% post-settling window, not just at one instant. Tolerance is a first
% cut -- check the actual printed max/mean above and retune rather than
% trusting this blindly, same lesson as every other test tolerance in
% this project so far.
agreementTol = deg2rad(3);
assert(max(rollDiff) < rad2deg(agreementTol), ...
    'Roll disagreement between filters exceeds tolerance: max %.3f deg (tol %.3f deg)', ...
    max(rollDiff), rad2deg(agreementTol));
assert(max(pitchDiff) < rad2deg(agreementTol), ...
    'Pitch disagreement between filters exceeds tolerance: max %.3f deg (tol %.3f deg)', ...
    max(pitchDiff), rad2deg(agreementTol));

disp('Cross-check passed: madgwickStepFull and complementaryFilterStep agree on roll/pitch under dynamic motion.');