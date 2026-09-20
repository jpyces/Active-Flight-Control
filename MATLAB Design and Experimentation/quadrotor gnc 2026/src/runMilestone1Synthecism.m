% RUNMILESTONE1SYNTHETICSIM
%   Full synthetic closed loop: synthetic sensors -> real madgwickStepFull
%   + real VerticalKF -> real StateEstimate -> real
%   AttitudeAltitudeController -> commands applied back to a real coupled
%   drone plant. Entirely in MATLAB, no hardware -- the point is seeing
%   the WHOLE architecture and plumbing working together end-to-end, not
%   a new algorithm. Everything wired in here already exists and is
%   independently validated; this script is the wiring, plus the one
%   missing piece (verticalAccelFromBody.m) that turns the separate
%   filters into an actual working state estimator.
%
%   Scope/assumptions, flagged rather than silently baked in:
%     - Horizontal position/velocity not modeled (see dronePlantStep.m).
%     - mass/I/mixMatrix are synthetic placeholders -- no real airframe
%       exists yet. I is reused at the same magnitude (0.01) as the
%       already-validated single-axis rate-loop toy plant, but this
%       model adds real cross-axis gyroscopic coupling that plant never
%       had -- watch the plots for behavior the existing gains weren't
%       tuned against.
%     - Complementary-filter cold-boot seeding is NOT included --
%       Madgwick starts from identity, so the ~5.5s convergence
%       transient documented in gnc-findings.md will be visible in the
%       plots. Adding that seed (State Estimation Init) is a natural
%       next increment.
%     - Quaternion.integrateGyro's signature is assumed, not confirmed
%       -- see dronePlantStep.m.

clear; clc;

%% Plant / physical constants (synthetic placeholders)
mass = 1;
g = 10;
hoverThrust = mass * g;
maxThrust = 15;
I = [0.01, 0.01, 0.01];

dt = 0.01;
nSteps = 1500;   % 15s -- long enough to show cold-start convergence AND settling

noiseStd.accel = 0.01;
noiseStd.gyro  = 0.001;
noiseStd.mag   = 0.01;
noiseStd.baro  = sqrt(0.0121);   % matches VerticalKF's real, datasheet-derived R

%% Controller (same gains as StateEstimateIntegrationTest.m)
angleGains  = repmat([3, 0, 0.5], 3, 1);
angleBounds = repmat([-150, 150], 3, 1);
rateGains   = repmat([0.05, 0.01, 0.001], 3, 1);
maxTorque   = ones(3,1);   % placeholder — same magnitude as the old rateBounds=[-1,1],
                           % now an explicit per-axis physical-torque limit instead of
                           % an ad hoc PID output bound; revisit once real arm length /
                           % motor torque coefficients exist (see gnc-findings.md)
altitudeGains = [3, 0, 3];
mixMatrix = [1 -1  1  1;
             1  1  1 -1;
             1  1 -1  1;
             1 -1 -1 -1];   % structurally-valid placeholder, not your real mix -- see StateEstimateIntegrationTest.m note
controller = AttitudeAltitudeController( ...
    angleGains, angleBounds, rateGains, maxTorque, ...
    altitudeGains, mixMatrix, hoverThrust, maxThrust, dt);

%% Estimators
sigmaA = 0.2; QbiasVal = 1e-7;
Bpv = [0.5*dt^2; dt];
Qvkf = zeros(3,3);
Qvkf(1:2,1:2) = sigmaA^2 * (Bpv*Bpv');
Qvkf(3,3) = QbiasVal;
Rvkf = 0.0121;
vkf = VerticalKF(Qvkf, Rvkf, [0;0;0], eye(3)*100);

qEst = Quaternion(1, 0, 0, 0);
beta = 0.1;

%% True plant: start disturbed, not at the setpoint
trueState.q = Quaternion.fromEulerZYX(deg2rad(15), deg2rad(-10), deg2rad(5));
trueState.omega = [0 0 0];
trueState.altitude = 8;
trueState.velocity = 0;

setpoints = Setpoints('angle', [0 0 0], 'altitude', 10);

%% Logs
logT = (0:nSteps-1)' * dt;
logTrueAngles = zeros(nSteps,3);
logEstAngles  = zeros(nSteps,3);
logTrueAlt = zeros(nSteps,1);
logEstAlt  = zeros(nSteps,1);
logTorque = zeros(nSteps,3);
logThrottle = zeros(nSteps,1);
logMotorCmds = zeros(nSteps,4);

%% Closed loop
appliedThrottle = hoverThrust;   % what the plant is under BEFORE this iteration's new command

for k = 1:nSteps
    readings = syntheticSensors(trueState, appliedThrottle, mass, noiseStd);

    qEst = madgwickStepFull(qEst, readings.gyro, readings.accel, readings.mag, dt, beta);
    estAngles = qEst.toEulerZYX();

    worldVertAccel = verticalAccelFromBody(readings.accel, qEst, g);
    vkf.predict(worldVertAccel, dt);
    vkf.correct(readings.baroAltitude);

    estimate = StateEstimate( ...
        'angles', estAngles, ...
        'rates', readings.gyro, ...
        'altitude', vkf.x(1), ...
        'verticalVelocity', vkf.x(2));
    measurements = stateEstimateToControllerMeasurements(estimate);

    [motorCmds, throttle, torque] = controller.update(setpoints, measurements);

    trueState = dronePlantStep(trueState, torque, throttle, mass, I, g, dt);
    appliedThrottle = throttle;

    logTrueAngles(k,:) = trueState.q.toEulerZYX();
    logEstAngles(k,:) = estAngles;
    logTrueAlt(k) = trueState.altitude;
    logEstAlt(k) = vkf.x(1);
    logTorque(k,:) = torque;
    logThrottle(k) = throttle;
    logMotorCmds(k,:) = motorCmds;
end

%% Plots
figure('Name', 'Milestone 1 Synthetic Closed-Loop Sim');

axisNames = {'Roll', 'Pitch', 'Yaw'};
for i = 1:3
    subplot(3,2,(i-1)*2+1);
    plot(logT, rad2deg(logTrueAngles(:,i)), 'k-', 'LineWidth', 1.2); hold on;
    plot(logT, rad2deg(logEstAngles(:,i)), 'r--');
    yline(0, 'b:');
    ylabel([axisNames{i} ' (deg)']);
    if i == 1, title('True (black) vs Estimated (red), setpoint = 0 (blue)'); end
    if i == 3, xlabel('Time (s)'); end
    legend('true', 'estimated', 'Location', 'best');
end

subplot(3,2,2);
plot(logT, logTrueAlt, 'k-', 'LineWidth', 1.2); hold on;
plot(logT, logEstAlt, 'r--');
yline(10, 'b:');
ylabel('Altitude (m)');
title('True vs Estimated Altitude, setpoint = 10m');
legend('true', 'estimated', 'Location', 'best');

subplot(3,2,4);
plot(logT, logTorque);
ylabel('Torque (pre-mix)');
legend('roll', 'pitch', 'yaw', 'Location', 'best');

subplot(3,2,6);
plot(logT, logMotorCmds);
xlabel('Time (s)');
ylabel('Motor commands');
legend('M1', 'M2', 'M3', 'M4', 'Location', 'best');

fprintf('Final true angles (deg):  roll=%.2f pitch=%.2f yaw=%.2f\n', rad2deg(logTrueAngles(end,:)));
fprintf('Final est angles (deg):   roll=%.2f pitch=%.2f yaw=%.2f\n', rad2deg(logEstAngles(end,:)));
fprintf('Final true/est altitude:  %.3f / %.3f m\n', logTrueAlt(end), logEstAlt(end));