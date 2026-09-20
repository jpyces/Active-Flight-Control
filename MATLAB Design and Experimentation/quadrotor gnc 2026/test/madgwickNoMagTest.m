% MADGWICKSTATICTEST
%   Milestone 0 step 5, static convergence check for madgwickNoMag
%   (accel+gyro, no mag yet). Zero gyro rate, a fixed tilted accel
%   reading -- the filter should converge from identity to the tilt
%   implied by that accel vector, and then HOLD there (small steady
%   residual, no oscillation), unlike the gyro-only integrator from
%   step 3, which has nothing pulling it back to the truth.

% --- Fixed inputs. gyro = 0 isolates the accel-correction term
% entirely -- if this doesn't converge, the bug is in the correction
% math, not an interaction with gyro integration.
gyro = [0, 0, 0];
dt = 0.01;
nIterations = 500;
beta = 0.1;

% --- Deliberately NOT straight up ([0,0,1]) -- that would converge
% trivially from an identity start with zero correction needed, and
% wouldn't actually exercise the correction term. Pick a real tilt.
accelRaw = [0.3, -0.2, 0.9];
accel = accelRaw / norm(accelRaw);

% --- Independent closed-form check: roll/pitch directly from the
% accel vector via the trig method, computed WITHOUT touching the
% filter at all. This is the ground truth the filter's output gets
% compared against below -- two different derivations of the same
% answer, so they should agree if both (a) your filter and (b) this
% check are correct. (Doesn't cover yaw -- accel alone can't observe
% it; see the yaw note at the end.)
expectedRoll  = atan2(accel(2), accel(3));
expectedPitch = atan2(-accel(1), sqrt(accel(2)^2 + accel(3)^2));

% --- Attitude state, starts at identity -- the filter has to correct
% ALL the way from "flat" to the true tilt, not start near the answer.
q = Quaternion(1, 0, 0, 0);

% --- Track residual norm each step: this is the same 3x1 f computed
% inside madgwickNoMag, recomputed here purely for logging/diagnosis.
% It should shrink toward ~0 as q converges, then stay small and flat
% (not oscillating) once it gets there.
residualNorm = zeros(nIterations, 1);

for i = 1:nIterations
    q = madgwickNoMag(q, gyro, accel, dt, beta);

    f = [2*(q.x*q.z - q.w*q.y) - accel(1);
        2*(q.w*q.x + q.y*q.z) - accel(2);
        2*(0.5 - q.x^2 - q.y^2)  - accel(3)];
    residualNorm(i) = norm(f);
end

% --- Pull roll/pitch/yaw back out of the converged quaternion. Check
% your Quaternion class's actual return order from toEulerZYX -- this
% assumes [roll, pitch, yaw], adjust if yours differs.
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

% --- Convergence: did it get close to the right answer at all.
angleTol = deg2rad(1);   % 1 degree
assert(abs(finalRoll  - expectedRoll)  < angleTol, ...
    'Roll did not converge: off by %.2f deg', rad2deg(abs(finalRoll - expectedRoll)));
assert(abs(finalPitch - expectedPitch) < angleTol, ...
    'Pitch did not converge: off by %.2f deg', rad2deg(abs(finalPitch - expectedPitch)));

% --- Stability: did it SETTLE, not just pass through the right answer
% on its way to oscillating. Check the tail of the run stays small and
% doesn't swing back up -- a real bug (like the yaw-corruption failure
% mode, or beta too high) tends to show up as growth or ringing here
% even when the final single sample happens to look fine.
tailWindow = 50;
tail = residualNorm(end-tailWindow+1:end);
residualTol = 3e-3;
assert(max(tail) < residualTol, ...
    'Residual not settled: max over last %d steps = %.4g (tol %.4g) -- check for oscillation, try lowering beta', ...
    tailWindow, max(tail), residualTol);
assert(std(tail) < residualTol/2, ...
    'Residual is noisy/oscillating over the tail (std = %.4g) rather than flat', std(tail));

disp('Static test passed: converged to expected tilt and held steady.');