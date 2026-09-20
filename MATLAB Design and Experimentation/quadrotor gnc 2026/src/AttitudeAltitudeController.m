% Axis Indexing: ROLL=1, PITCH=2, YAW=3
classdef AttitudeAltitudeController < handle
    properties (Constant)
        ROLL=1
        PITCH=2
        YAW=3
    end

    properties
        angleLoopPIDs
        rateLoopPIDs
        altitudePID
        mixMatrix
        hoverThrust, maxThrust, maxTorque
        dt
    end

    methods
        % angleGains: 3x3 - Roll, Pitch, Yaw each a row with kp, ki, kd
        % columns
        % angleBounds: 3x2 - rows = axes, columns = min, max
        % rate gains - same shape as angleGains
        % maxTorque - 3x1, axis-indexed max achievable torque per axis;
        %             rate loop bounds are derived from this ([-maxTorque, maxTorque]),
        %             same pattern altitudeBounds already uses for hoverThrust/maxThrust
        % altitude gains - 1x3 row vector
        % altitude bounds - calculated from hoverThrust and maxThrust
        function obj = AttitudeAltitudeController( ...
            angleGains, angleBounds, ...
            rateGains, maxTorque, ...
            altitudeGains, ...
            mixMatrix, hoverThrust, maxThrust, dt ...
        )
            obj.dt = dt;
            obj.angleLoopPIDs = PIDControllerBase.empty(0,0);
            obj.rateLoopPIDs = PIDControllerBase.empty(0,0);
            for i = obj.ROLL:obj.YAW
                obj.angleLoopPIDs(i) = PIDControllerBase( ...
                    angleGains(i,1), angleGains(i,2), angleGains(i,3), ...
                    dt, ...
                    angleBounds(i,1), angleBounds(i,2) ...
                );
                obj.rateLoopPIDs(i) = PIDControllerBase( ...
                    rateGains(i,1), rateGains(i,2), rateGains(i,3), ...
                    dt, ...
                    -maxTorque(i), maxTorque(i) ...
                );
            end
            obj.altitudePID = PIDControllerBase( ...
                altitudeGains(1), altitudeGains(2), altitudeGains(3), ...
                dt, ...
                -hoverThrust, maxThrust - hoverThrust ...
            );
            obj.mixMatrix = mixMatrix;
            obj.hoverThrust = hoverThrust;
            obj.maxThrust = maxThrust;
            obj.maxTorque = maxTorque;
        end

        function [motorCmds, throttle, torque] = update(obj, setpoints, measurements)
            % setpoints.angle    : 3x1, axis-indexed (ROLL/PITCH/YAW)
            % setpoints.altitude  : scalar
            % measurements.angle  : 3x1, axis-indexed
            % measurements.rate   : 3x1, axis-indexed
            % measurements.altitude : scalar
            
            torque = zeros(1,3); % Pre-allocate for performance - not necessary but easy

            % Goes through each contained PID and updates it.
            % Angle loop produces angular rates, Rate loop produces actual
            %   torques
            % altitudePID is the same type as the rest, but calculates trim
            %   away from hoverThrust which is adjust here
            % motorCmds mixes the actual inputs
            for i = obj.ROLL:obj.YAW
                desiredRate = obj.angleLoopPIDs(i).update(setpoints.angle(i), measurements.angle(i));
                torque(i) = obj.rateLoopPIDs(i).update(desiredRate, measurements.rate(i));
            end

            trim = obj.altitudePID.update(setpoints.altitude, measurements.altitude);
            throttle = obj.hoverThrust + trim;          % Newtons — keep returning this as-is, callers/tests expect it
            normThrottle = throttle / obj.maxThrust;     % convert to mixMotors' [0,1] convention
            motorCmds = mixMotors(normThrottle, ...
                torque(obj.ROLL)  / obj.maxTorque(obj.ROLL), ...
                torque(obj.PITCH) / obj.maxTorque(obj.PITCH), ...
                torque(obj.YAW)   / obj.maxTorque(obj.YAW), ...
                obj.mixMatrix);
        end

        function reset(obj)
            for i = obj.ROLL:obj.YAW
                obj.angleLoopPIDs(i).reset();
                obj.rateLoopPIDs(i).reset();
            end
            obj.altitudePID.reset();
        end
    end
 
end