classdef FlightStateMachine < handle
    % Drives the vehicle through its boot/init/arm/flight sequence and
    % monitors the four failsafe branches. All boundary-crossing conditions
    % that live outside this class (memory check, calibration check, arm
    % request, flight trigger) arrive as plain boolean signals -- this class
    % does not care which firmware module produced them or how.
    %
    % Failsafes are priority-ranked, highest severity first:
    %   FailsafeSensorLoss > FailsafeAhrsDegraded > FailsafeAltitude > FailsafeRcLoss
    % checkFailsafes() re-scans all four conditions every tick and always
    % enters whichever tripped condition is highest priority, even escalating
    % from one failsafe directly into a more severe one.

    properties (SetAccess = {?FlightStateMachine, ?FlightStateMachineTest})
        CurrentState
        PreviousState
    end

    properties (Access = private)
        StateEstimatorInstance
        ControllerInstance

        % dwell/elapsed timers
        SensorInitElapsed
        MotorCheckElapsed
        AltitudeErrorElapsed
        RcElapsedSinceValid

        % injected config
        SensorInitDuration
        MotorCheckDuration
        AltitudeErrorTol
        AltitudeErrorTimeout
        RcLossTimeout
        RcRequired            % logical -- ground-configurable; false makes branch 3 fully inert
    end

    methods
        function obj = FlightStateMachine(stateEstimator, controller, timingParams, faultParams)
            obj.StateEstimatorInstance = stateEstimator;
            obj.ControllerInstance = controller;

            obj.SensorInitDuration   = timingParams.sensorInitDuration;
            obj.MotorCheckDuration   = timingParams.motorCheckDuration;
            obj.AltitudeErrorTol     = faultParams.altitudeErrorTol;
            obj.AltitudeErrorTimeout = faultParams.altitudeErrorTimeout;
            obj.RcLossTimeout        = faultParams.rcLossTimeout;
            obj.RcRequired           = faultParams.rcRequired;

            obj.SensorInitElapsed = 0;
            obj.MotorCheckElapsed = 0;
            obj.AltitudeErrorElapsed = 0;
            obj.RcElapsedSinceValid = 0;

            obj.CurrentState = FlightState.Off;
            obj.PreviousState = FlightState.Off;
        end

        function transition(obj, sensorReadings, rcStatus, stateEstimate, altitudeSetpoint, ...
                memoryCheckPassed, calibrationCheckPassed, armed, flightTrigger, dt)

            if obj.checkFailsafes(sensorReadings, rcStatus, stateEstimate, altitudeSetpoint, dt)
                return
            end

            switch obj.CurrentState
                case FlightState.Off
                    % TODO: power-on is presumably what instantiates/starts driving this
                    % class at all, so there may be nothing to do here -- leaving as a
                    % no-op rather than guessing at a signal that might not exist.

                case FlightState.On
                    obj.CurrentState = FlightState.MemoryCheck;

                case FlightState.MemoryCheck
                    if memoryCheckPassed
                        obj.CurrentState = FlightState.SensorInit;
                    end

                case FlightState.SensorInit
                    obj.SensorInitElapsed = obj.SensorInitElapsed + dt;
                    if obj.SensorInitElapsed >= obj.SensorInitDuration
                        obj.CurrentState = FlightState.CalibrationCheck;
                    end

                case FlightState.CalibrationCheck
                    if calibrationCheckPassed
                        obj.CurrentState = FlightState.StateEstimationInit;
                    end

                case FlightState.StateEstimationInit
                    if obj.StateEstimatorInstance.isConverged()
                        obj.CurrentState = FlightState.Disarmed;
                    end

                case FlightState.Disarmed
                    if armed
                        obj.StateEstimatorInstance.reset();
                        obj.ControllerInstance.reset();
                        obj.CurrentState = FlightState.MotorCheck;
                    end

                case FlightState.MotorCheck
                    obj.MotorCheckElapsed = obj.MotorCheckElapsed + dt;
                    if obj.MotorCheckElapsed >= obj.MotorCheckDuration
                        obj.CurrentState = FlightState.Armed;
                    end

                case FlightState.Armed
                    % ASSUMPTION: flightTrigger follows the same external-signal pattern
                    % as armed/memoryCheckPassed/calibrationCheckPassed. Rename/rewire
                    % once the real throttle-up/takeoff detection exists.
                    if flightTrigger
                        obj.CurrentState = FlightState.Flight;
                    end

                case FlightState.Flight
                    % terminal nominal state; only failsafes exit it, handled above
            end
        end
    end

    methods (Access = private)
        function tripped = checkFailsafes(obj, sensorReadings, rcStatus, stateEstimate, altitudeSetpoint, dt)
            % Gate against the state we'd actually return to, not CurrentState itself --
            % while already in a failsafe, CurrentState IS a Failsafe* value, which isn't
            % in any of the gating lists below.
            if obj.isInFailsafe(obj.CurrentState)
                effectiveState = obj.PreviousState;
            else
                effectiveState = obj.CurrentState;
            end

            altTripped    = ismember(effectiveState, [FlightState.Flight]) ...
                && obj.altitudeErrorExceeded(stateEstimate, altitudeSetpoint, dt);
            sensorTripped = ismember(effectiveState, [FlightState.Disarmed, FlightState.MotorCheck, ...
                FlightState.Armed, FlightState.Flight]) && FlightStateMachine.criticalSensorLost(sensorReadings);
            ahrsTripped   = ismember(effectiveState, [FlightState.Disarmed, FlightState.MotorCheck, ...
                FlightState.Armed, FlightState.Flight]) && obj.StateEstimatorInstance.isDegraded();
            rcTripped     = ismember(effectiveState, [FlightState.Armed, FlightState.Flight]) ...
                && obj.rcLinkLost(rcStatus, dt);

            % Resync Madgwick the instant AHRS degradation clears, regardless of what
            % happens next (recover to nominal, or escalate into a different failsafe).
            wasAhrsDegraded = isequal(obj.CurrentState, FlightState.FailsafeAhrsDegraded);
            if wasAhrsDegraded && ~ahrsTripped
                obj.StateEstimatorInstance.reset();
            end

            anyTripped = sensorTripped || ahrsTripped || altTripped || rcTripped;

            if anyTripped
                % Priority, highest first: sensor loss > AHRS degraded > altitude > RC loss
                if sensorTripped
                    target = FlightState.FailsafeSensorLoss;
                elseif ahrsTripped
                    target = FlightState.FailsafeAhrsDegraded;
                elseif altTripped
                    target = FlightState.FailsafeAltitude;
                else
                    target = FlightState.FailsafeRcLoss;
                end
                obj.enterFailsafe(target);
                tripped = true;
            else
                if obj.isInFailsafe(obj.CurrentState)
                    obj.CurrentState = obj.PreviousState;
                end
                tripped = false;
            end
        end

        function enterFailsafe(obj, failsafeState)
            if ~obj.isInFailsafe(obj.CurrentState)
                obj.PreviousState = obj.CurrentState;
            end
            obj.CurrentState = failsafeState;
        end

        function tf = altitudeErrorExceeded(obj, stateEstimate, altitudeSetpoint, dt)
            err = abs(stateEstimate.altitude - altitudeSetpoint);
            if err > obj.AltitudeErrorTol
                obj.AltitudeErrorElapsed = obj.AltitudeErrorElapsed + dt;
            else
                obj.AltitudeErrorElapsed = 0;
            end
            tf = obj.AltitudeErrorElapsed > obj.AltitudeErrorTimeout;
        end

        function tf = rcLinkLost(obj, rcStatus, dt)
            if ~obj.RcRequired
                tf = false;
                return
            end
            if rcStatus.isValid
                obj.RcElapsedSinceValid = 0;
            else
                obj.RcElapsedSinceValid = obj.RcElapsedSinceValid + dt;
            end
            tf = obj.RcElapsedSinceValid > obj.RcLossTimeout;
        end
    end

    methods (Static)
        function tf = isInFailsafe(state)
            tf = ismember(state, [FlightState.FailsafeAltitude, FlightState.FailsafeSensorLoss, ...
                FlightState.FailsafeAhrsDegraded, FlightState.FailsafeRcLoss]);
        end

        function tf = criticalSensorLost(sensorReadings)
            % accel/gyro have no fallback path (mag has NoMag, baro has pure integration),
            % so FAILED on either is the critical case. Sensor-handler side already debounces
            % noise/repeated-sample issues before status ever reaches FAILED, so this is a
            % plain, undebounced status check.
            tf = sensorReadings.accelStatus == SensorStatus.FAILED || ...
                 sensorReadings.gyroStatus  == SensorStatus.FAILED;
        end
    end
end