classdef PitchAngleCascadeTest < matlab.unittest.TestCase

    properties (Constant)
        dt = 0.01
        I  = 0.01
        c  = 0.001
        KpAngle = 3;    KiAngle = 0;    KdAngle = 0.5
        KpRate  = 0.05; KiRate  = 0.01; KdRate  = 0.001
        rateBound   = 150   % deg/s, angle loop's output limit
        torqueBound = 1     % rate loop's output limit
    end

    methods (Test)

        function settlesWithinToleranceAfterDisturbance(testCase)
            settleTolerance  = 1;  % deg -- "close enough" band, not exact zero
            settleTimeBudget = 2;  % seconds, based on the plot you already ran

            angleLog = testCase.runCascade(30, 5);
            settleIdx = round(settleTimeBudget / testCase.dt);

            testCase.verifyLessThan(abs(angleLog(settleIdx)), settleTolerance, ...
                sprintf('Angle should be within %.1f deg of level by t=%.1fs', settleTolerance, settleTimeBudget));
        end

        function staysWithinToleranceOnceSettled(testCase)
            % catches re-divergence/oscillation a single settle-time check could miss
            settleTolerance  = 1;
            settleTimeBudget = 2;

            angleLog = testCase.runCascade(30, 5);
            settleIdx = round(settleTimeBudget / testCase.dt);

            testCase.verifyLessThan(max(abs(angleLog(settleIdx:end))), settleTolerance, ...
                'Angle should stay within tolerance for the rest of the run, not kick back out once settled.');
        end

        function noOvershootPastLevel(testCase)
            % for this gain set the response is non-oscillatory -- it should
            % never cross past 0 to the opposite sign
            angleLog = testCase.runCascade(30, 5);
            testCase.verifyGreaterThanOrEqual(min(angleLog), -1e-6, ...
                'Angle should not overshoot past level for this gain set.');
        end

        function symmetricForOppositeDisturbance(testCase)
            % sanity check: system is linear (aside from symmetric saturation
            % bounds), so a mirrored disturbance should produce a mirrored response
            posLog = testCase.runCascade(30, 5);
            negLog = testCase.runCascade(-30, 5);
            testCase.verifyEqual(negLog, -posLog, 'AbsTol', 1e-9, ...
                'Response to a mirrored disturbance should be mirrored.');
        end

    end

    methods (Access = private)
        function angleLog = runCascade(testCase, initialAngle, duration)
            N = round(duration / testCase.dt);
            angleLoopPID = PIDControllerBase(testCase.KpAngle, testCase.KiAngle, testCase.KdAngle, ...
                testCase.dt, -testCase.rateBound, testCase.rateBound);
            rateLoopPID  = PIDControllerBase(testCase.KpRate, testCase.KiRate, testCase.KdRate, ...
                testCase.dt, -testCase.torqueBound, testCase.torqueBound);

            angle = initialAngle; rate = 0;
            angleLog = zeros(N,1);
            for k = 1:N
                desiredRate = angleLoopPID.update(0, angle);
                torque      = rateLoopPID.update(desiredRate, rate);
                [angle, rate] = simulateRatePlantStep(angle, rate, torque, testCase.I, testCase.c, testCase.dt);
                angleLog(k) = angle;
            end
        end
    end
end