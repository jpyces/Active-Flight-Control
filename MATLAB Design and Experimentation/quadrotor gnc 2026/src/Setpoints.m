function setpoints = Setpoints(varargin)
%SETPOINTS Canonical control target, consumed directly by AttitudeAltitudeController.update().
%
%   setpoints = Setpoints() returns a template with every field present
%   but unpopulated ([]).
%
%   setpoints = Setpoints('angle', [rollSp pitchSp yawSp], 'altitude', altSp)
%   overrides specific fields via name-value pairs; anything not given
%   stays at its empty default.
%
%   Fields:
%     angle    - [rollSetpoint pitchSetpoint yawSetpoint], axis-indexed (ROLL/PITCH/YAW)
%     altitude - scalar
%
%   Field names and shape deliberately match AttitudeAltitudeController's
%   own documented `setpoints` argument exactly (see that file's comment:
%   "setpoints.angle : 3x1, axis-indexed", "setpoints.altitude : scalar").
%   Unlike SensorReadings/StateEstimate, no adapter sits between this
%   struct and the controller that consumes it -- this was designed
%   knowing the consumer's real shape up front, so it can be passed
%   straight into update() as-is.
%
%   This struct's shape is not expected to grow for Milestone 2.
%   AttitudeAltitudeController's accepted setpoint vocabulary doesn't
%   change -- PathfinderController's job is to compute exactly this pair
%   (a desired lean angle + a desired altitude/throttle-equivalent) from
%   a higher-level target (waypoint, position, velocity), not to hand
%   the controller anything new. So Setpoints is milestone-agnostic
%   because the receiving interface itself isn't changing, not because
%   of reserved-but-empty fields the way StateEstimate is.

setpoints = struct( ...
    'angle',    [], ...
    'altitude', []);

applyOverrides(varargin);

    function applyOverrides(pairs)
        if isempty(pairs)
            return
        end
        if mod(numel(pairs), 2) ~= 0
            error('Setpoints:BadArgs', ...
                'Name-value pairs must come in pairs.');
        end
        for k = 1:2:numel(pairs)
            name = pairs{k};
            if ~isfield(setpoints, name)
                error('Setpoints:UnknownField', ...
                    'Unknown Setpoints field "%s".', name);
            end
            setpoints.(name) = pairs{k+1};
        end
    end
end