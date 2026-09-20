classdef VerticalKFTest < matlab.unittest.TestCase

    properties (Constant)
        dt = 0.01;
        trueAltitude = 5.0;          % m, held constant (hover)
        trueBias = 0.2;              % m/s^2, injected accel bias
        baroVariance = 0.0121;       % m^2 — BMP280 full-bandwidth, altitude-domain (measured/verify against real unit)
        accelSensorNoiseStd = sqrt(3e-10);  % m/s^2 — LSM6DSO's own (tiny) sensor noise, ±2g normal mode
        sigmaA = 0.2;                % m/s^2, 1 std dev — unmodeled-dynamics scale for Q, NOT sensor noise
        Qbias = 1e-7;                % placeholder bias-drift rate — needs real tuning against logged data
        baroDecimation = 10;         % baro correction every 10th predict step
    end

    methods (Test)

        function testConvergesToKnownAltitude(testCase)
            rng(1);
            kf = testCase.freshKF();

            for k = 1:2000  % 20s at dt=0.01
                aMeas = testCase.trueBias + testCase.accelSensorNoiseStd*randn();
                kf.predict(aMeas, testCase.dt);
                if mod(k, testCase.baroDecimation) == 0
                    kf.correct(testCase.trueAltitude + sqrt(testCase.baroVariance)*randn());
                end
            end

            testCase.verifyLessThan(abs(kf.x(1) - testCase.trueAltitude), 0.2, ...
                'Altitude estimate should converge close to true altitude');
        end

        function testBiasConverges(testCase)
            rng(2);
            kf = testCase.freshKF();

            for k = 1:3000
                aMeas = testCase.trueBias + testCase.accelSensorNoiseStd*randn();
                kf.predict(aMeas, testCase.dt);
                if mod(k, testCase.baroDecimation) == 0
                    kf.correct(testCase.trueAltitude + sqrt(testCase.baroVariance)*randn());
                end
            end

            testCase.verifyLessThan(abs(kf.x(3) - testCase.trueBias), 0.05, ...
                'Bias estimate should converge close to the injected true bias');
        end

        function testPurePredictionDriftsWithoutCorrection(testCase)
            % No correct() calls at all. Q/R don't affect x here (only P),
            % so this test's outcome is unchanged by the R/Q retuning above —
            % it's purely A/B applied to a constant unmodeled bias input.
            Q = testCase.buildQ();
            x0 = [testCase.trueAltitude; 0; 0];  % start exactly right
            P0 = diag([0.01, 0.01, 0.01]);
            kf = VerticalKF(Q, testCase.baroVariance, x0, P0);

            unmodeledBias = 0.3;  % filter's bias state starts at 0, never corrected
            for k = 1:2000
                kf.predict(unmodeledBias, testCase.dt);
            end

            testCase.verifyGreaterThan(abs(kf.x(1) - testCase.trueAltitude), 5, ...
                'Altitude should drift substantially with no barometer correction');
        end

        function testCovarianceStaysWellFormed(testCase)
            rng(3);
            kf = testCase.freshKF();

            for k = 1:5000  % long run to stress numerical behavior
                aMeas = testCase.trueBias + testCase.accelSensorNoiseStd*randn();
                kf.predict(aMeas, testCase.dt);
                if mod(k, testCase.baroDecimation) == 0
                    kf.correct(testCase.trueAltitude + sqrt(testCase.baroVariance)*randn());
                end
            end

            testCase.verifyLessThan(norm(kf.P - kf.P', 'fro'), 1e-8, ...
                'Covariance should remain numerically symmetric');
            testCase.verifyGreaterThanOrEqual(min(eig(kf.P)), -1e-8, ...
                'Covariance should remain positive semi-definite');
        end

        function testMeasurementNoiseRejection(testCase)
            rng(4);
            Q = testCase.buildQ();
            x0 = [testCase.trueAltitude; 0; 0];
            P0 = diag([0.01, 0.01, 0.01]);  % filter starts confident
            kf = VerticalKF(Q, testCase.baroVariance, x0, P0);

            for k = 1:100
                kf.predict(0, testCase.dt);
                if mod(k, testCase.baroDecimation) == 0
                    kf.correct(testCase.trueAltitude + sqrt(testCase.baroVariance)*randn());
                end
            end

            preOutlier = kf.x(1);
            outlierReading = testCase.trueAltitude + 10;  % wildly bad baro reading
            kf.correct(outlierReading);

            jump = abs(kf.x(1) - preOutlier);
            rawJump = abs(outlierReading - preOutlier);
            testCase.verifyLessThan(jump, 0.5*rawJump, ...
                'A single outlier should be damped by the Kalman gain, not passed through');
        end

    end

    methods (Access = private)
        function kf = freshKF(testCase)
            Q = testCase.buildQ();
            kf = VerticalKF(Q, testCase.baroVariance, [0;0;0], diag([1, 1, 0.1]));
        end

        function Q = buildQ(testCase)
            % Coupled altitude/velocity block, propagated through the same B
            % used in predict() — NOT independent diagonal guesses. sigmaA
            % represents unmodeled real-world dynamics (vibration, gusts,
            % discretization error), not the accelerometer's own sensor noise
            % (which is negligible by comparison, per the earlier derivation).
            Bpv = [0.5*testCase.dt^2; testCase.dt];
            Qposvel = testCase.sigmaA^2 * (Bpv * Bpv');
            Q = blkdiag(Qposvel, testCase.Qbias);
        end
    end
end