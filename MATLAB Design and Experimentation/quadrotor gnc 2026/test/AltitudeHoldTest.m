classdef AltitudeHoldTest < matlab.unittest.TestCase
    properties (Constant)
        dt = 0.01
        mass = 1; g = 10
        hoverThrust = 10
        maxThrust = 15
        Kp = 3; Ki = 0; Kd = 3
    end

    methods (Test)
        function climbsToSetpointWithinTolerance(testCase)
            altLog = testCase.runAltitude(0, 0, 5, 8);
            settleIdx = round(4/testCase.dt);
            testCase.verifyLessThan(abs(altLog(settleIdx)-5), 0.1, ...
                'Altitude should be within 0.1m of setpoint by t=4s');
        end

        function staysWithinToleranceOnceSettled(testCase)
            altLog = testCase.runAltitude(0, 0, 5, 8);
            settleIdx = round(4/testCase.dt);
            testCase.verifyLessThan(max(abs(altLog(settleIdx:end)-5)), 0.1, ...
                'Altitude should stay settled, not drift back out of tolerance.');
        end

        function rejectsDownwardDisturbanceWhileHovering(testCase)
            altLog = testCase.runAltitude(3, 0, 5, 8); % knocked down to 3m while holding 5m
            settleIdx = round(4/testCase.dt);
            testCase.verifyLessThan(abs(altLog(settleIdx)-5), 0.1, ...
                'Should recover to setpoint after a downward disturbance while hovering.');
        end

        function trimNeverExceedsAchievableRange(testCase)
            [~, thrustLog] = testCase.runAltitude(0, 0, 5, 8);
            testCase.verifyGreaterThanOrEqual(min(thrustLog), 0);
            testCase.verifyLessThanOrEqual(max(thrustLog), testCase.maxThrust);
        end
    end

    methods (Access = private)
        function [altLog, thrustLog] = runAltitude(testCase, initialAltitude, initialVelocity, setpoint, duration)
            N = round(duration/testCase.dt);
            altitudePID = PIDControllerBase(testCase.Kp, testCase.Ki, testCase.Kd, testCase.dt, ...
                -testCase.hoverThrust, testCase.maxThrust - testCase.hoverThrust);
            altitude = initialAltitude; velocity = initialVelocity;
            altLog = zeros(N,1); thrustLog = zeros(N,1);
            for k = 1:N
                trim = altitudePID.update(setpoint, altitude);
                thrust = testCase.hoverThrust + trim;
                [altitude, velocity] = simulateAltitudePlantStep(altitude, velocity, thrust, testCase.mass, testCase.g, testCase.dt);
                altLog(k) = altitude; thrustLog(k) = thrust;
            end
        end
    end
end