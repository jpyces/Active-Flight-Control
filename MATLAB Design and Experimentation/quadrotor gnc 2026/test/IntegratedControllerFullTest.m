classdef IntegratedControllerFullTest < matlab.unittest.TestCase
    methods (Test)
        function testCombinedDisturbanceAllChannelsSettle(testCase)
            dt = 0.01;
            numSteps = 800; % 8s, matching AltitudeHoldTest.m's own duration

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

            % Distinct per-channel disturbance -- catches cross-wiring that
            % identical-valued single-axis tests could mask
            state.angle = [20; -15; 10];
            state.rate = [0; 0; 0];
            state.altitude = 0;           % matches AltitudeHoldTest's 0->5 climb
            state.verticalVelocity = 0;

            setpoints.angle = [0; 0; 0];
            setpoints.altitude = 5;

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

            settleFromAngle = round(2/dt);
            testCase.verifyLessThan(abs(rollHist(settleFromAngle:end)), 1);
            testCase.verifyLessThan(abs(pitchHist(settleFromAngle:end)), 1);
            testCase.verifyLessThan(abs(yawHist(settleFromAngle:end)), 1);

            settleFromAltitude = round(4/dt);
            testCase.verifyLessThan(abs(altHist(settleFromAltitude:end) - setpoints.altitude), 0.1);
        end
    end
end