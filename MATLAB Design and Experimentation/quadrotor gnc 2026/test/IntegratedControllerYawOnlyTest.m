classdef IntegratedControllerYawOnlyTest < matlab.unittest.TestCase
    methods (Test)
        function testYawDisturbanceNoCrossAxisLeak(testCase)
            dt = 0.01;
            numSteps = 500;

            I = 0.01; c = 0.001;
            mass = 1; g = 10; maxThrust = 15; hoverThrust = mass*g;

            angleGains  = repmat([3, 0, 0.5], 3, 1);
            angleBounds = repmat([-150, 150], 3, 1);
            rateGains   = repmat([0.05, 0.01, 0.001], 3, 1);
            maxTorque   = ones(3,1);   % was rateBounds = repmat([-1, 1], 3, 1)
            altitudeGains = [3, 0, 3]; % matches your real AltitudeHoldTest.m
            mixMatrix = [
                1  1  1  1;
                1 -1  1 -1;
                1 -1 -1  1;
                1  1 -1 -1
            ];
            controller = AttitudeAltitudeController( ...
                angleGains, angleBounds, rateGains, maxTorque, ...
                altitudeGains, mixMatrix, hoverThrust, maxThrust, dt);

            % Only yaw disturbed; roll/pitch/altitude start at rest/setpoint
            state.angle = [0; 0; 30];
            state.rate = [0; 0; 0];
            state.altitude = 0;
            state.verticalVelocity = 0;

            setpoints.angle = [0; 0; 0];
            setpoints.altitude = state.altitude;

            rollHist = zeros(1, numSteps);
            pitchHist = zeros(1, numSteps);
            yawHist = zeros(1, numSteps);
            altHist = zeros(1, numSteps);

            for k = 1:numSteps
                measurements.angle = state.angle;
                measurements.rate = state.rate;
                measurements.altitude = state.altitude;

                [~, throttle, torque] = controller.update(setpoints, measurements);

                for i = controller.ROLL:controller.YAW
                    [state.angle(i), state.rate(i)] = simulateRatePlantStep( ...
                        state.angle(i), state.rate(i), torque(i), I, c, dt);
                end
                [state.altitude, state.verticalVelocity] = simulateAltitudePlantStep( ...
                    state.altitude, state.verticalVelocity, throttle, mass, g, dt);

                rollHist(k)  = state.angle(controller.ROLL);
                pitchHist(k) = state.angle(controller.PITCH);
                yawHist(k)   = state.angle(controller.YAW);
                altHist(k)   = state.altitude;
            end

            settleFrom = round(2/dt);
            testCase.verifyLessThan(abs(yawHist(settleFrom:end)), 1);

            crossAxisEps = 1e-6;
            testCase.verifyLessThan(max(abs(rollHist)), crossAxisEps);
            testCase.verifyLessThan(max(abs(pitchHist)), crossAxisEps);
            testCase.verifyLessThan(max(abs(altHist - setpoints.altitude)), crossAxisEps);
        end
    end
end