function estimate = StateEstimate(varargin)
    %STATEESTIMATE Canonical output of StateEstimator.step(), consumed by controllers.
    %
    %   estimate = StateEstimate() returns a template with every field
    %   present but Milestone-2-only fields left []. Milestone 1's
    %   AttitudeAltitudeController only ever reads angles/rates/altitude/
    %   verticalVelocity, so position/velocity/gyroBias/accelBias are simply
    %   never populated by the Milestone 1 StateEstimator -- nothing consumes
    %   them yet.
    %
    %   Left as [] rather than NaN deliberately: arithmetic on an empty
    %   array errors immediately and loudly, whereas NaN silently propagates
    %   through downstream math. If something in Milestone 1 ever
    %   accidentally reads e.g. estimate.position before it's populated,
    %   this fails fast at that point instead of producing a quietly-wrong
    %   controller output much further downstream.
    %
    %   estimate = StateEstimate('angles', [roll pitch yaw], 'rates', [p q r], ...)
    %   overrides specific fields via name-value pairs; anything not given
    %   stays at its empty default.
    %
    %   Fields:
    %     angles            - [roll pitch yaw],  fused attitude (Madgwick / complementary fallback)
    %     rates             - [p q r],            raw-ish gyro passthrough, low-pass only -- NOT fused
    %     altitude          - scalar,              fused (VerticalKF)
    %     verticalVelocity  - scalar,              fused (VerticalKF)
    %     position          - [x y z]              (Milestone 2 only -- joint EKF)
    %     velocity          - [vx vy vz]           (Milestone 2 only -- joint EKF)
    %     gyroBias          - [bx by bz]           (Milestone 2 only -- joint EKF)
    %     accelBias         - [bx by bz]           (Milestone 2 only -- joint EKF)
    %
    %   The struct shape itself does not change between milestones -- only
    %   which fields a given StateEstimator implementation populates. This
    %   is what lets AttitudeAltitudeController (and later PathfinderController)
    %   stay agnostic to which milestone's estimator produced the estimate.
    
    estimate = struct( ...
        'angles',           [], ...
        'rates',            [], ...
        'altitude',         [], ...
        'verticalVelocity', [], ...
        'position',         [], ...
        'velocity',         [], ...
        'gyroBias',         [], ...
        'accelBias',        []);
    
    applyOverrides(varargin);
    
        function applyOverrides(pairs)
            if isempty(pairs)
                return
            end
            if mod(numel(pairs), 2) ~= 0
                error('StateEstimate:BadArgs', ...
                    'Name-value pairs must come in pairs.');
            end
            for k = 1:2:numel(pairs)
                name = pairs{k};
                if ~isfield(estimate, name)
                    error('StateEstimate:UnknownField', ...
                        'Unknown StateEstimate field "%s".', name);
                end
                estimate.(name) = pairs{k+1};
            end
        end
end