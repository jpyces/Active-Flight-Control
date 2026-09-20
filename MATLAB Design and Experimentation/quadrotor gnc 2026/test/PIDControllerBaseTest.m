classdef PIDControllerBaseTest < matlab.unittest.TestCase

    methods (Test)

        function proportionalOnly(testCase)
            % Ki=Kd=0: output should be exactly Kp*error, no saturation
            pid = PIDControllerBase(2, 0, 0, 0.01, -100, 100);
            out = pid.update(5, 2); % error = 3
            testCase.verifyEqual(out, 6, 'AbsTol', 1e-9);
        end

        function integralAccumulatesAsRiemannSum(testCase)
            % constant error over several steps: integral should match
            % the expected running sum, Iterm = Ki * sum(error*dt)
            Ki = 1; dt = 0.1;
            pid = PIDControllerBase(0, Ki, 0, dt, -1000, 1000);
            error = 4; measurement = 0; setpoint = error;
            expectedIntegral = 0;
            for k = 1:5
                out = pid.update(setpoint, measurement);
                expectedIntegral = expectedIntegral + error*dt;
                testCase.verifyEqual(out, Ki*expectedIntegral, 'AbsTol', 1e-9);
            end
        end

        function derivativeIsZeroOnFirstCall(testCase)
            % initialized flag should suppress D on the very first update,
            % even with a large jump between setpoint and measurement
            pid = PIDControllerBase(0, 0, 10, 0.01, -1000, 1000);
            out = pid.update(50, 0); % would be huge if D fired here
            testCase.verifyEqual(out, 0, 'AbsTol', 1e-9);
        end

        function derivativeMatchesFiniteDifference(testCase)
            Kd = 3; dt = 0.05;
            pid = PIDControllerBase(0, 0, Kd, dt, -1000, 1000);
            pid.update(0, 2);           % primes prevMeasurement, D=0
            out = pid.update(0, 5);     % measurement jumped 2->5
            expected = -Kd * (5 - 2) / dt;
            testCase.verifyEqual(out, expected, 'AbsTol', 1e-9);
        end

        function outputClampsToBounds(testCase)
            pid = PIDControllerBase(100, 0, 0, 0.01, -10, 10);
            out = pid.update(1000, 0); % huge error, would blow past bounds
            testCase.verifyEqual(out, 10);
        end

        function antiWindupFreezesIntegralWhileSaturatedAsymmetricBounds(testCase)
            % outMin/outMax NOT symmetric about zero -- this is the case
            % the sign-based check used to get wrong
            Ki = 1; dt = 0.1;
            pid = PIDControllerBase(0, Ki, 0, dt, 0, 1); % e.g. throttle-like
            error = 5; measurement = 0; setpoint = error;
            pid.update(setpoint, measurement); % saturates immediately, out=1
            integralAfterFirst = pid.integral;
            for k = 1:5
                pid.update(setpoint, measurement); % still saturated, error still positive
            end
            testCase.verifyEqual(pid.integral, integralAfterFirst, 'AbsTol', 1e-9, ...
                'Integral should not grow while output is saturated and error still pushes further into it.');
        end

        function integralResumesOnceErrorReverses(testCase)
            Ki = 1; dt = 1;
            pid = PIDControllerBase(0, Ki, 0, dt, -10, 10);
            pid.update(50, 0);   % error=50, saturates high, frozen at integral=0
            frozenIntegral = pid.integral;
            pid.update(-2, 0);   % error=-2, lands well inside [-10,10] -> should resume
            testCase.verifyNotEqual(pid.integral, frozenIntegral);
        end

        function resetClearsState(testCase)
            pid = PIDControllerBase(1, 1, 1, 0.01, -100, 100);
            pid.update(10, 0);
            pid.reset();
            testCase.verifyEqual(pid.integral, 0);
            testCase.verifyEqual(pid.prevMeasurement, 0);
            testCase.verifyFalse(pid.initialized);
            out = pid.update(10, 0); % D should be suppressed again post-reset
            % isolate D by checking against a fresh object's first-call output
            fresh = PIDControllerBase(1, 1, 1, 0.01, -100, 100);
            testCase.verifyEqual(out, fresh.update(10, 0), 'AbsTol', 1e-9);
        end

        function handleSemanticsShareState(testCase)
            a = PIDControllerBase(1, 0, 0, 0.01, -100, 100);
            b = a; % reference, not a copy
            a.update(5, 0);
            testCase.verifyEqual(b.initialized, true, ...
                'b should see a''s state change since PIDControllerBase is a handle class.');
        end

        function separateInstancesAreIndependent(testCase)
            a = PIDControllerBase(1, 0, 0, 0.01, -100, 100);
            c = PIDControllerBase(1, 0, 0, 0.01, -100, 100);
            a.update(5, 0);
            testCase.verifyFalse(c.initialized, ...
                'Separate constructor calls must produce independent objects.');
        end

    end
end