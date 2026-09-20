function readings = SensorReadings(varargin)
%SENSORREADINGS Canonical sensor input bundle consumed by StateEstimator.step().
%
%   readings = SensorReadings() returns a template with every field
%   present but unpopulated ([]) and every status UNINITIALIZED. The
%   shape is always complete, even for sensors that don't physically
%   exist yet (GNSS in Milestone 1) -- callers and estimator
%   implementations never need to special-case a missing field, only an
%   empty one.
%
%   readings = SensorReadings('accel', [ax ay az], 'accelStatus', SensorStatus.NOMINAL, ...)
%   overrides specific fields via name-value pairs; anything not given
%   stays at its empty/UNINITIALIZED default.
%
%   Fields:
%     accel, accelStatus           - [ax ay az],  SensorStatus
%     gyro, gyroStatus              - [gx gy gz],  SensorStatus
%     mag, magStatus                 - [mx my mz],  SensorStatus
%     baroAltitude, baroStatus       - scalar,      SensorStatus
%     gnssPosition, gnssVelocity,
%       gnssStatus                  - [x y z], [vx vy vz], SensorStatus
%
%   gnssPosition/gnssVelocity/gnssStatus are Milestone 2 only. In
%   Milestone 1 they are left [] / UNINITIALIZED and the Milestone 1
%   StateEstimator implementation never reads them -- there is no GNSS
%   hardware yet, so there is nothing to populate them with.

readings = struct( ...
    'accel',        [], ...
    'accelStatus',  SensorStatus.UNINITIALIZED, ...
    'gyro',         [], ...
    'gyroStatus',   SensorStatus.UNINITIALIZED, ...
    'mag',          [], ...
    'magStatus',    SensorStatus.UNINITIALIZED, ...
    'baroAltitude', [], ...
    'baroStatus',   SensorStatus.UNINITIALIZED, ...
    'gnssPosition', [], ...
    'gnssVelocity', [], ...
    'gnssStatus',   SensorStatus.UNINITIALIZED);

applyOverrides(varargin);

    function applyOverrides(pairs)
        if isempty(pairs)
            return
        end
        if mod(numel(pairs), 2) ~= 0
            error('SensorReadings:BadArgs', ...
                'Name-value pairs must come in pairs.');
        end
        for k = 1:2:numel(pairs)
            name = pairs{k};
            if ~isfield(readings, name)
                error('SensorReadings:UnknownField', ...
                    'Unknown SensorReadings field "%s".', name);
            end
            readings.(name) = pairs{k+1};
        end
    end
end