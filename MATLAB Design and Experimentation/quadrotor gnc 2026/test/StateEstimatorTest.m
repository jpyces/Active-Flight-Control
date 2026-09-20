classdef StateEstimatorTest < matlab.unittest.TestCase

    properties
        Dt = 0.01
        SeedingDuration = 0.5
        MadgwickBeta = 0.1
        InstabilityNormTol = 1e-3
        CompAlpha = 0.98
        Gravity = 9.81
    end

    methods (Access = private)
        function obj = makeEstimator(testCase)
            kfParams = {1e-4*eye(3), 0.0121, [0;0;0], eye(3)};  % Q, R, x0, P0 -- placeholder-scale
            obj = StateEstimator(testCase.MadgwickBeta, testCase.CompAlpha, testCase.Gravity, ...
                kfParams, testCase.SeedingDuration, testCase.InstabilityNormTol);
        end

        function readings = stillReadings(testCase)
            % At-rest: zero gyro, gravity-magnitude accel (NOT unit-normalized --
            % verticalAccelFromBody subtracts the real gravity constant, so this
            % must carry real magnitude or it reads as a large spurious
            % "falling" acceleration instead of a genuine at-rest ~0).
            readings = SensorReadings( ...
                'accel', [0 0 testCase.Gravity], 'accelStatus', SensorStatus.NOMINAL, ...
                'gyro',  [0 0 0], 'gyroStatus',  SensorStatus.NOMINAL, ...
                'mag',   [1 0 0], 'magStatus',   SensorStatus.NOMINAL, ...
                'baroAltitude', 0, 'baroStatus', SensorStatus.NOMINAL);
        end
    end

    methods (Test)

        function testStartsInSeeding(testCase)
            obj = testCase.makeEstimator();
            testCase.verifyEqual(obj.AhrsSource, "Seeding");
            testCase.verifyFalse(obj.isConverged());
        end

        function testSeedingOutputTracksComplementaryFilter(testCase)
            obj = testCase.makeEstimator();
            readings = testCase.stillReadings();
            refQuat = Quaternion.identity();
            for k = 1:10
                obj.update(readings, testCase.Dt);
                refQuat = complementaryFilterStep(refQuat, readings.gyro, readings.accel, ...
                    testCase.Dt, testCase.CompAlpha);
                testCase.verifyEqual(obj.Quaternion, refQuat);
            end
            testCase.verifyEqual(obj.AhrsSource, "Seeding");
        end

        function testTransitionsToNominalAtSeedingDuration(testCase)
            obj = testCase.makeEstimator();
            readings = testCase.stillReadings();
            nSteps = round(testCase.SeedingDuration / testCase.Dt);

            for k = 1:nSteps-1
                obj.update(readings, testCase.Dt);
                testCase.verifyEqual(obj.AhrsSource, "Seeding", ...
                    sprintf('Flipped to Nominal early, at step %d', k));
            end

            obj.update(readings, testCase.Dt);
            testCase.verifyEqual(obj.AhrsSource, "Nominal");
            testCase.verifyTrue(obj.isConverged());
        end

        function testSeedHandoffIsContinuous(testCase)
            obj = testCase.makeEstimator();
            readings = testCase.stillReadings();
            refQuat = Quaternion.identity();
            nSteps = round(testCase.SeedingDuration / testCase.Dt);

            for k = 1:nSteps
                refQuat = complementaryFilterStep(refQuat, readings.gyro, readings.accel, ...
                    testCase.Dt, testCase.CompAlpha);
                obj.update(readings, testCase.Dt);
            end

            testCase.verifyEqual(obj.Quaternion, refQuat);
        end

        function testMagVariantTracksSensorStatusEachStep(testCase)
            obj = testCase.makeEstimator();
            nSteps = round(testCase.SeedingDuration / testCase.Dt);
            readings = testCase.stillReadings();
            for k = 1:nSteps
                obj.update(readings, testCase.Dt);
            end
            testCase.verifyEqual(obj.AhrsSource, "Nominal");

            readingsNoMag = testCase.stillReadings();
            readingsNoMag.magStatus = SensorStatus.DEGRADED;
            obj.update(readingsNoMag, testCase.Dt);
            testCase.verifyEqual(obj.MadgwickVariant, "NoMag");

            readingsMagBack = testCase.stillReadings();
            obj.update(readingsMagBack, testCase.Dt);
            testCase.verifyEqual(obj.MadgwickVariant, "Full");
        end

        function testDetectInstabilityFlagsNonFinite(testCase)
            badQuat = Quaternion(NaN, 0, 0, 0);
            testCase.verifyTrue(StateEstimator.detectInstability(badQuat, testCase.InstabilityNormTol));
        end

        function testDetectInstabilityFlagsNormDrift(testCase)
            driftedQuat = Quaternion(2, 0, 0, 0);
            testCase.verifyTrue(StateEstimator.detectInstability(driftedQuat, testCase.InstabilityNormTol));
        end

        function testDetectInstabilityAcceptsUnitQuaternion(testCase)
            goodQuat = Quaternion.identity();
            testCase.verifyFalse(StateEstimator.detectInstability(goodQuat, testCase.InstabilityNormTol));
        end

        function testResetFromNominalStaysNominalAndResyncs(testCase)
            obj = testCase.makeEstimator();
            readings = testCase.stillReadings();
            nSteps = round(testCase.SeedingDuration / testCase.Dt);
            for k = 1:nSteps
                obj.update(readings, testCase.Dt);
            end
            testCase.verifyEqual(obj.AhrsSource, "Nominal");

            obj.reset();
            testCase.verifyEqual(obj.AhrsSource, "Nominal");
        end

        function testResetFromFallbackReturnsToNominal(testCase)
            obj = testCase.makeEstimator();
            obj.AhrsSource = "FallbackComplementary";
            obj.update(testCase.stillReadings(), testCase.Dt);
            testCase.verifyEqual(obj.AhrsSource, "FallbackComplementary");

            obj.reset();
            testCase.verifyEqual(obj.AhrsSource, "Nominal");
            testCase.verifyTrue(obj.isConverged());
        end

        function testNoSilentRecoveryFromFallback(testCase)
            obj = testCase.makeEstimator();
            obj.AhrsSource = "FallbackComplementary";
            readings = testCase.stillReadings();
            for k = 1:20
                obj.update(readings, testCase.Dt);
                testCase.verifyEqual(obj.AhrsSource, "FallbackComplementary", ...
                    sprintf('Silently left FallbackComplementary at step %d without reset()', k));
            end
        end

        function testVerticalKFPredictsEveryStepRegardlessOfBaro(testCase)
            kfParams = {1e-4*eye(3), 0.0121, [0; 0.5; 0], eye(3)};  % nonzero initial
                                                                      % velocity -- gives pure
                                                                      % prediction something to
                                                                      % accumulate
            obj = StateEstimator(testCase.MadgwickBeta, testCase.CompAlpha, testCase.Gravity, ...
                kfParams, testCase.SeedingDuration, testCase.InstabilityNormTol);
            readings = testCase.stillReadings();
            readings.baroStatus = SensorStatus.FAILED;   % never corrects
        
            altitudes = zeros(1, 20);
            for k = 1:20
                obj.update(readings, testCase.Dt);
                altitudes(k) = obj.VerticalKFInstance.x(1);
            end
            % With correction withheld throughout, altitude should accumulate
            % from the nonzero initial velocity every predict() call.
            testCase.verifyNotEqual(altitudes(1), altitudes(end));
            testCase.verifyGreaterThan(altitudes(end), altitudes(1));   % velocity is positive
        end

        function testVerticalKFCorrectsOnlyWhenBaroNominal(testCase)
            obj = testCase.makeEstimator();
            refKF = VerticalKF(1e-4*eye(3), 0.0121, [0;0;0], eye(3));

            readingsNoCorrect = testCase.stillReadings();
            readingsNoCorrect.baroStatus = SensorStatus.DEGRADED;
            for k = 1:5
                obj.update(readingsNoCorrect, testCase.Dt);
                refKF.predict(0, testCase.Dt);   % at-rest true vertical accel ~0
            end
            testCase.verifyEqual(obj.VerticalKFInstance.x, refKF.x, 'AbsTol', 1e-10);

            readingsCorrect = testCase.stillReadings();
            obj.update(readingsCorrect, testCase.Dt);
            refKF.predict(0, testCase.Dt);
            refKF.correct(readingsCorrect.baroAltitude);
            testCase.verifyEqual(obj.VerticalKFInstance.x, refKF.x, 'AbsTol', 1e-10);
        end

    end
end