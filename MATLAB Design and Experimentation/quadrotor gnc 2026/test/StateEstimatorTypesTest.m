classdef StateEstimatorTypesTest < matlab.unittest.TestCase
    %STATEESTIMATORTYPESTEST Contract tests for SensorReadings / StateEstimate.
    %
    %   These aren't testing any estimation logic -- there isn't any yet.
    %   They're testing the interface contract itself: that the default
    %   template has the right shape, that Milestone-2-only fields stay
    %   empty unless explicitly set, and that overrides only ever touch
    %   the field named, never a neighbor. That contract is the whole
    %   point of this file, so it's worth locking down with tests before
    %   any real estimator is wired up against it.

    methods (Test)

        function sensorReadingsDefaultIsFullyEmpty(testCase)
            r = SensorReadings();
            testCase.verifyEmpty(r.accel);
            testCase.verifyEmpty(r.gyro);
            testCase.verifyEmpty(r.mag);
            testCase.verifyEmpty(r.baroAltitude);
            testCase.verifyEmpty(r.gnssPosition);
            testCase.verifyEmpty(r.gnssVelocity);
            testCase.verifyEqual(r.accelStatus, SensorStatus.UNINITIALIZED);
            testCase.verifyEqual(r.gyroStatus,  SensorStatus.UNINITIALIZED);
            testCase.verifyEqual(r.magStatus,   SensorStatus.UNINITIALIZED);
            testCase.verifyEqual(r.baroStatus,  SensorStatus.UNINITIALIZED);
            testCase.verifyEqual(r.gnssStatus,  SensorStatus.UNINITIALIZED);
        end

        function sensorReadingsOverrideOnlyTouchesNamedFields(testCase)
            r = SensorReadings('accel', [1 2 3], 'accelStatus', SensorStatus.NOMINAL);
            testCase.verifyEqual(r.accel, [1 2 3]);
            testCase.verifyEqual(r.accelStatus, SensorStatus.NOMINAL);
            % everything else must remain at its default
            testCase.verifyEmpty(r.gyro);
            testCase.verifyEmpty(r.mag);
            testCase.verifyEmpty(r.baroAltitude);
            testCase.verifyEmpty(r.gnssPosition);
            testCase.verifyEqual(r.gyroStatus, SensorStatus.UNINITIALIZED);
        end

        function sensorReadingsRejectsUnknownField(testCase)
            testCase.verifyError(@() SensorReadings('notAField', 1), ...
                'SensorReadings:UnknownField');
        end

        function sensorReadingsRejectsOddArgCount(testCase)
            testCase.verifyError(@() SensorReadings('accel'), ...
                'SensorReadings:BadArgs');
        end

        function stateEstimateDefaultIsFullyEmpty(testCase)
            e = StateEstimate();
            testCase.verifyEmpty(e.angles);
            testCase.verifyEmpty(e.rates);
            testCase.verifyEmpty(e.altitude);
            testCase.verifyEmpty(e.verticalVelocity);
            testCase.verifyEmpty(e.position);
            testCase.verifyEmpty(e.velocity);
            testCase.verifyEmpty(e.gyroBias);
            testCase.verifyEmpty(e.accelBias);
        end

        function stateEstimateMilestone1FieldsSettableWithoutTouchingMilestone2Fields(testCase)
            % This is the specific contract this whole design leans on:
            % populating the Milestone 1 fields must not implicitly
            % populate, or require populating, the Milestone 2 fields.
            e = StateEstimate( ...
                'angles', [0.1 0.2 0.3], ...
                'rates', [0.01 0.02 0.03], ...
                'altitude', 10.5, ...
                'verticalVelocity', 0.2);

            testCase.verifyEqual(e.angles, [0.1 0.2 0.3]);
            testCase.verifyEqual(e.rates, [0.01 0.02 0.03]);
            testCase.verifyEqual(e.altitude, 10.5);
            testCase.verifyEqual(e.verticalVelocity, 0.2);

            testCase.verifyEmpty(e.position);
            testCase.verifyEmpty(e.velocity);
            testCase.verifyEmpty(e.gyroBias);
            testCase.verifyEmpty(e.accelBias);
        end

        function stateEstimateMilestone2FieldsSettableWhenPresent(testCase)
            e = StateEstimate( ...
                'position', [1 2 3], ...
                'velocity', [0.1 0.1 0.1], ...
                'gyroBias', [0.001 0.001 0.001], ...
                'accelBias', [0.01 0.01 0.01]);

            testCase.verifyEqual(e.position, [1 2 3]);
            testCase.verifyEqual(e.velocity, [0.1 0.1 0.1]);
            testCase.verifyEqual(e.gyroBias, [0.001 0.001 0.001]);
            testCase.verifyEqual(e.accelBias, [0.01 0.01 0.01]);
        end

        function stateEstimateRejectsUnknownField(testCase)
            testCase.verifyError(@() StateEstimate('notAField', 1), ...
                'StateEstimate:UnknownField');
        end

    end
end