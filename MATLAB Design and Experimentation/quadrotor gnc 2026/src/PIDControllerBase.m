classdef PIDControllerBase < handle
    properties
        Kp, Ki, Kd
        dt
        outMin, outMax
        integral
        prevMeasurement
        initialized
    end

    methods
        function obj = PIDControllerBase(Kp, Ki, Kd, dt, outMin, outMax)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.dt = dt;
            obj.outMin = outMin;
            obj.outMax = outMax;
            obj.integral = 0;
            obj.prevMeasurement = 0;
            obj.initialized = false;
        end

        function out = update(obj, setpoint, measurement)
            error = setpoint - measurement;
            
            Pterm = obj.Kp * error;
            
            integral_candidate = obj.integral + error*obj.dt;
            Iterm = obj.Ki * integral_candidate;
            
            if (obj.initialized == 0)
                Dterm = 0;
            else
                Dterm = -obj.Kd * (measurement - obj.prevMeasurement) / obj.dt;
            end

            out_unsat = Pterm + Iterm + Dterm;
            [out, atUpper, atLower] = obj.saturate(out_unsat, obj.outMin, obj.outMax);
            freeze = (atUpper && error > 0) || (atLower && error < 0);
            if ~freeze
                obj.integral = integral_candidate;
            end

            obj.prevMeasurement = measurement; % Update previous measurement
            obj.initialized = true; % Set initialization flag
        end

        function reset(obj)
            obj.integral = 0; % Reset integral term
            obj.prevMeasurement = 0; % Reset previous measurement
            obj.initialized = false; % Reset initialization flag
        end
    end

    methods (Static)
        function [out, atUpper, atLower] = saturate(out_unsat, outMin, outMax)
            out = max(outMin, min(out_unsat, outMax));
            atUpper = (out >= outMax);
            atLower = (out <= outMin);
        end
    end
end