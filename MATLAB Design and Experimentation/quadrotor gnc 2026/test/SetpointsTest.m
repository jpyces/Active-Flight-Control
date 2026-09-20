classdef SetpointsTest < matlab.unittest.TestCase
    %SETPOINTSTEST Contract tests for the Setpoints factory function.

    methods (Test)

        function defaultIsFullyEmpty(testCase)
            s = Setpoints();
            testCase.verifyEmpty(s.angle);
            testCase.verifyEmpty(s.altitude);
        end

        function overrideOnlyTouchesNamedFields(testCase)
            s = Setpoints('angle', [0.1 0 0]);
            testCase.verifyEqual(s.angle, [0.1 0 0]);
            testCase.verifyEmpty(s.altitude);
        end

        function bothFieldsSettable(testCase)
            s = Setpoints('angle', [0.1 -0.2 0], 'altitude', 5);
            testCase.verifyEqual(s.angle, [0.1 -0.2 0]);
            testCase.verifyEqual(s.altitude, 5);
        end

        function rejectsUnknownField(testCase)
            testCase.verifyError(@() Setpoints('notAField', 1), ...
                'Setpoints:UnknownField');
        end

        function rejectsOddArgCount(testCase)
            testCase.verifyError(@() Setpoints('angle'), ...
                'Setpoints:BadArgs');
        end

        function shapeMatchesControllerExpectationDirectly(testCase)
            % This is the actual point of Setpoints existing: it should
            % be usable as AttitudeAltitudeController.update()'s
            % setpoints argument with zero translation, unlike
            % StateEstimate -> measurements (see
            % stateEstimateToControllerMeasurements.m). Field-name
            % presence is checked here since the full round-trip
            % requires a constructed AttitudeAltitudeController, which
            % is exercised in the integrated-controller tests instead.
            s = Setpoints('angle', [0 0 0], 'altitude', 1);
            testCase.verifyTrue(isfield(s, 'angle'));
            testCase.verifyTrue(isfield(s, 'altitude'));
        end

    end
end