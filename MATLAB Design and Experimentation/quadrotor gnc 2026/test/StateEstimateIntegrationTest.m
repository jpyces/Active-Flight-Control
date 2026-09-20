classdef StateEstimateIntegrationTest < matlab.unittest.TestCase
    %STATEESTIMATEINTEGRATIONTEST First end-to-end test of the estimator/controller
    %interface using REAL filter code (madgwickStepFull, VerticalKF), not toy-plant
    %state and not a static snapshot.
    %
    %   Everything in IntegratedController*Test.m drives
    %   AttitudeAltitudeController directly off toy-plant state. This test
    %   goes one layer earlier: synthetic sensor readings -> real
    %   madgwickStepFull / real VerticalKF -> a real StateEstimate ->
    %   stateEstimateToControllerMeasurements -> a real
    %   AttitudeAltitudeController. The point is to prove the StateEstimate/
    %   Setpoints contract actually works as the connective tissue between
    %   real estimator output and a real controller, not just that the
    %   struct shapes are self-consistent (StateEstimatorTypesTest.m /
    %   SetpointsTest.m already cover that).
    %
    %   Synthetic accel/mag are generated the same way madgwickFullTest.m
    %   does -- by evaluating the filter's own residual formulas in reverse
    %   at a chosen ground-truth quaternion -- so this test's "truth" can't
    %   silently disagree with the filter's own convention. Synthetic
    %   VerticalKF input is derived directly from its documented A/B
    %   matrices (x(2) update = vel + dt*(aMeas - bias)), not guessed.
    %
    %   This does NOT yet close the loop dynamically (true state evolving
    %   under the controller's own commanded torque/throttle, the way
    %   IntegratedControllerRollOnlyTest does against a toy plant). That's
    %   a reasonable next step once this plumbing is confirmed working --
    %   scoped out here deliberately rather than guessed at.

    properties
        convergedEstimate
        measurements
        trueRoll
        truePitch
        trueYaw
        trueAltitude
    end

    methods (TestClassSetup)

        function buildConvergedEstimateFromRealFilters(testCase)
            % ---- Madgwick: converge to a known, non-trivial attitude ----
            % Same technique/values as madgwickFullTest.m.
            rollTrue  = deg2rad(15);
            pitchTrue = deg2rad(-20);
            yawTrue   = deg2rad(35);
            qTrue = Quaternion.fromEulerZYX(rollTrue, pitchTrue, yawTrue);

            dipAngle = deg2rad(60);
            bxRef = cos(dipAngle);
            bzRef = sin(dipAngle);

            w = qTrue.w; x = qTrue.x; y = qTrue.y; z = qTrue.z;
            accelTrue = [2*(x*z - w*y), 2*(w*x + y*z), 2*(0.5 - x^2 - y^2)];
            magTrue = [2*bxRef*(0.5 - y^2 - z^2) + 2*bzRef*(x*z - w*y), ...
                2*bxRef*(x*y - w*z)       + 2*bzRef*(w*x + y*z), ...
                2*bxRef*(w*y + x*z)       + 2*bzRef*(0.5 - x^2 - y^2)];

            gyro = [0, 0, 0];
            dt = 0.01;
            nIterations = 550;
            beta = 0.1;
            q = Quaternion(1, 0, 0, 0);
            for i = 1:nIterations
                q = madgwickStepFull(q, gyro, accelTrue, magTrue, dt, beta);
            end
            finalAngles = q.toEulerZYX();

            % ---- VerticalKF: converge to a known altitude + accel bias ----
            % Derived directly from VerticalKF's documented state-transition
            % matrices, not guessed: x(2)_{k+1} = x(2)_k + dt*(aMeas - x(3)),
            % so for a constant true altitude (true vertical accel = 0), the
            % raw accel reading that produces that is just the true bias.
            trueAlt  = 10.0;
            trueBias = 0.05;

            sigmaA = 0.2;
            QbiasVal = 1e-7;
            Bpv = [0.5*dt^2; dt];
            Q = zeros(3,3);
            Q(1:2,1:2) = sigmaA^2 * (Bpv * Bpv');
            Q(3,3) = QbiasVal;
            R = 0.0121;
            x0 = [0; 0; 0];
            P0 = eye(3) * 100;

            vkf = VerticalKF(Q, R, x0, P0);
            for i = 1:nIterations
                vkf.predict(trueBias, dt);   % true vertical accel = 0, so aMeas = bias
                vkf.correct(trueAlt);
            end

            % ---- Assemble the real StateEstimate from real filter output ----
            testCase.convergedEstimate = StateEstimate( ...
                'angles', finalAngles, ...
                'rates', gyro, ...                  % static truth in this test -> gyro ~ 0
                'altitude', vkf.x(1), ...
                'verticalVelocity', vkf.x(2));

            testCase.measurements = stateEstimateToControllerMeasurements(testCase.convergedEstimate);

            testCase.trueRoll = rollTrue;
            testCase.truePitch = pitchTrue;
            testCase.trueYaw = yawTrue;
            testCase.trueAltitude = trueAlt;
        end

    end

    methods (Test)

        function convergedEstimateMatchesGroundTruth(testCase)
            % Sanity check on the setup itself before trusting the
            % controller-facing tests below: the real filters, run through
            % the real StateEstimate/adapter path, should reproduce known
            % truth to roughly the same tolerance the standalone filter
            % tests already prove independently.
            angleTol = deg2rad(1);
            testCase.verifyEqual(testCase.convergedEstimate.angles(1), testCase.trueRoll, ...
                'AbsTol', angleTol);
            testCase.verifyEqual(testCase.convergedEstimate.angles(2), testCase.truePitch, ...
                'AbsTol', angleTol);
            testCase.verifyEqual(testCase.convergedEstimate.angles(3), testCase.trueYaw, ...
                'AbsTol', angleTol);

            altTol = 0.1;
            testCase.verifyEqual(testCase.convergedEstimate.altitude, testCase.trueAltitude, ...
                'AbsTol', altTol);

            % measurements.angle/.rate/.altitude must carry the SAME values
            % as StateEstimate.angles/.rates/.altitude -- this is the
            % adapter contract itself, not a re-test of filter convergence.
            testCase.verifyEqual(testCase.measurements.angle, testCase.convergedEstimate.angles);
            testCase.verifyEqual(testCase.measurements.rate, testCase.convergedEstimate.rates);
            testCase.verifyEqual(testCase.measurements.altitude, testCase.convergedEstimate.altitude);
        end

        function controllerOutputsNearZeroWhenSetpointMatchesConvergedEstimate(testCase)
            % This is "how the PID controllers react" in the simplest case:
            % zero error in, ~zero corrective output out. Using a freshly
            % constructed controller so this is genuinely the first
            % update() call (D-term suppressed on the first call per
            % PIDControllerBase, integral starts at 0) -- makes the
            % near-zero-output expectation exact rather than approximate.
            controller = testCase.buildController();

            setpoints = Setpoints( ...
                'angle', testCase.convergedEstimate.angles, ...
                'altitude', testCase.convergedEstimate.altitude);

            [~, throttle, torque] = controller.update(setpoints, testCase.measurements);

            zeroTol = 1e-9;
            testCase.verifyEqual(torque, [0 0 0], 'AbsTol', zeroTol);
            testCase.verifyEqual(throttle, controller.hoverThrust, 'AbsTol', zeroTol);
        end

        function controllerReactsInCorrectDirectionWhenSetpointDiffers(testCase)
            % Nudge roll and altitude setpoints away from the converged
            % estimate and check the controller pushes the right way --
            % roll setpoint above measured roll should produce positive
            % roll torque (correcting toward it), altitude setpoint above
            % measured altitude should produce positive thrust trim
            % (climb). Checked by sign/direction, not exact magnitude,
            % since exact values depend on gains/bounds this test isn't
            % trying to re-validate (RollAngleCascadeTest.m /
            % AltitudeHoldTest.m already own that).
            controller = testCase.buildController();

            rollSetpoint = testCase.convergedEstimate.angles(1) + deg2rad(10);
            altSetpoint  = testCase.convergedEstimate.altitude + 1.0;

            setpoints = Setpoints( ...
                'angle', [rollSetpoint, testCase.convergedEstimate.angles(2), testCase.convergedEstimate.angles(3)], ...
                'altitude', altSetpoint);

            [~, throttle, torque] = controller.update(setpoints, testCase.measurements);

            testCase.verifyGreaterThan(torque(controller.ROLL), 0);
            testCase.verifyGreaterThan(throttle, controller.hoverThrust);
        end

    end

    methods (Access = private)

        function controller = buildController(~)
            % Same gains/bounds/toy-plant constants documented in
            % gnc-findings.md for the validated roll cascade and altitude
            % loop, reused identically across all three axes (matching how
            % pitch/yaw were first built from the roll cascade). mixMatrix
            % here is a structurally-valid placeholder quad-X mix, NOT
            % your real one -- fine for these tests since they only assert
            % on pre-mix torque/throttle, never on motorCmds. Swap in the
            % real mixMatrix if a future test needs to check motorCmds too.
            angleGains  = repmat([3, 0, 0.5], 3, 1);
            angleBounds = repmat([-150, 150], 3, 1);
            rateGains   = repmat([0.05, 0.01, 0.001], 3, 1);
            maxTorque   = ones(3,1);   % was rateBounds = repmat([-1, 1], 3, 1)
            altitudeGains = [3, 0, 3];
            mixMatrix = [1 -1  1  1;
                         1  1  1 -1;
                         1  1 -1  1;
                         1 -1 -1 -1];
            hoverThrust = 10;
            maxThrust = 15;
            dt = 0.01;

            controller = AttitudeAltitudeController( ...
                angleGains, angleBounds, ...
                rateGains, maxTorque, ...
                altitudeGains, ...
                mixMatrix, hoverThrust, maxThrust, dt);
        end

    end
end