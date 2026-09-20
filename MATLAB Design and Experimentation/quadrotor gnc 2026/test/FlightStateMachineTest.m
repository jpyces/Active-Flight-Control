classdef FlightStateMachineTest < matlab.unittest.TestCase

    properties
        Dt = 0.1
        SensorInitDuration = 0.5
        MotorCheckDuration = 0.5
        AltitudeErrorTol = 0.5
        AltitudeErrorTimeout = 0.3
        RcLossTimeout = 0.3
    end

    methods (Access = private)
        function [fsm, mockEstimator, mockController] = makeFsm(testCase, rcRequired)
            if nargin < 2
                rcRequired = false;
            end
            mockEstimator = MockStateEstimator();
            mockController = MockController();
            timingParams.sensorInitDuration = testCase.SensorInitDuration;
            timingParams.motorCheckDuration = testCase.MotorCheckDuration;
            faultParams.altitudeErrorTol = testCase.AltitudeErrorTol;
            faultParams.altitudeErrorTimeout = testCase.AltitudeErrorTimeout;
            faultParams.rcLossTimeout = testCase.RcLossTimeout;
            faultParams.rcRequired = rcRequired;
            fsm = FlightStateMachine(mockEstimator, mockController, timingParams, faultParams);
        end

        function readings = nominalReadings(testCase)
            readings = SensorReadings( ...
                'accel', [0 0 9.81], 'accelStatus', SensorStatus.NOMINAL, ...
                'gyro',  [0 0 0],    'gyroStatus',  SensorStatus.NOMINAL, ...
                'mag',   [1 0 0],    'magStatus',   SensorStatus.NOMINAL, ...
                'baroAltitude', 0,   'baroStatus',  SensorStatus.NOMINAL);
        end

        function rc = nominalRc(testCase)
            rc.isValid = true;
        end

        function est = estimateAt(testCase, altitude)
            est = StateEstimate('altitude', altitude);
        end
    end

    methods (Test)
        function testStartsInOff(testCase)
            fsm = testCase.makeFsm();
            testCase.verifyEqual(fsm.CurrentState, FlightState.Off);
            testCase.verifyEqual(fsm.PreviousState, FlightState.Off);
        end

        function testOnAdvancesToMemoryCheck(testCase)
            fsm = testCase.makeFsm();
            fsm.CurrentState = FlightState.On;
            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                testCase.estimateAt(0), 0, false, false, false, false, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.MemoryCheck);
        end

        function testMemoryCheckWaitsForSignal(testCase)
            fsm = testCase.makeFsm();
            fsm.CurrentState = FlightState.MemoryCheck;
            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                testCase.estimateAt(0), 0, false, false, false, false, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.MemoryCheck);
        end

        function testMemoryCheckAdvancesToSensorInitOnSignal(testCase)
            fsm = testCase.makeFsm();
            fsm.CurrentState = FlightState.MemoryCheck;
            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                testCase.estimateAt(0), 0, true, false, false, false, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.SensorInit);
        end

        function testSensorInitDwellsForFullDurationBeforeAdvancing(testCase)
            fsm = testCase.makeFsm();
            fsm.CurrentState = FlightState.SensorInit;
            nSteps = round(testCase.SensorInitDuration / testCase.Dt);
            for k = 1:nSteps-1
                fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                    testCase.estimateAt(0), 0, true, true, false, false, testCase.Dt);
                testCase.verifyEqual(fsm.CurrentState, FlightState.SensorInit, ...
                    sprintf('Advanced early at step %d', k));
            end
            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                testCase.estimateAt(0), 0, true, true, false, false, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.CalibrationCheck);
        end

        function testCalibrationCheckWaitsForSignal(testCase)
            fsm = testCase.makeFsm();
            fsm.CurrentState = FlightState.CalibrationCheck;
            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                testCase.estimateAt(0), 0, true, false, false, false, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.CalibrationCheck);

            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                testCase.estimateAt(0), 0, true, true, false, false, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.StateEstimationInit);
        end

        function testStateEstimationInitWaitsForConvergence(testCase)
            [fsm, mockEstimator] = testCase.makeFsm();
            fsm.CurrentState = FlightState.StateEstimationInit;
            mockEstimator.Converged = false;
            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                testCase.estimateAt(0), 0, true, true, false, false, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.StateEstimationInit);

            mockEstimator.Converged = true;
            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                testCase.estimateAt(0), 0, true, true, false, false, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.Disarmed);
        end

        function testArmingRequestCallsBothResetsAndAdvances(testCase)
            [fsm, mockEstimator, mockController] = testCase.makeFsm();
            fsm.CurrentState = FlightState.Disarmed;
            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                testCase.estimateAt(0), 0, true, true, true, false, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.MotorCheck);
            testCase.verifyEqual(mockEstimator.ResetCallCount, 1);
            testCase.verifyEqual(mockController.ResetCallCount, 1);
        end

        function testDisarmedStaysPutWithoutArmedSignal(testCase)
            [fsm, mockEstimator, mockController] = testCase.makeFsm();
            fsm.CurrentState = FlightState.Disarmed;
            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                testCase.estimateAt(0), 0, true, true, false, false, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.Disarmed);
            testCase.verifyEqual(mockEstimator.ResetCallCount, 0);
            testCase.verifyEqual(mockController.ResetCallCount, 0);
        end

        function testMotorCheckDwellsThenAdvancesToArmed(testCase)
            fsm = testCase.makeFsm();
            fsm.CurrentState = FlightState.MotorCheck;
            nSteps = round(testCase.MotorCheckDuration / testCase.Dt);
            for k = 1:nSteps-1
                fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                    testCase.estimateAt(0), 0, true, true, true, false, testCase.Dt);
                testCase.verifyEqual(fsm.CurrentState, FlightState.MotorCheck);
            end
            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                testCase.estimateAt(0), 0, true, true, true, false, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.Armed);
        end

        function testArmedAdvancesToFlightOnTrigger(testCase)
            fsm = testCase.makeFsm();
            fsm.CurrentState = FlightState.Armed;
            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                testCase.estimateAt(0), 0, true, true, true, false, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.Armed);

            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                testCase.estimateAt(0), 0, true, true, true, true, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.Flight);
        end

        function testAltitudeFailsafeTripsOnlyAfterSustainedError(testCase)
            fsm = testCase.makeFsm();
            fsm.CurrentState = FlightState.Flight;
            fsm.PreviousState = FlightState.Flight;
            badEstimate = testCase.estimateAt(100);  % setpoint will be 0 -> huge error
            nSteps = round(testCase.AltitudeErrorTimeout / testCase.Dt);
            for k = 1:nSteps-1
                fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                    badEstimate, 0, true, true, true, true, testCase.Dt);
                testCase.verifyEqual(fsm.CurrentState, FlightState.Flight, ...
                    sprintf('Tripped early at step %d', k));
            end
            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                badEstimate, 0, true, true, true, true, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.FailsafeAltitude);
        end

        function testAltitudeFailsafeGatedToFlightOnly(testCase)
            fsm = testCase.makeFsm();
            fsm.CurrentState = FlightState.Armed;  % not Flight -- altitude failsafe shouldn't apply here
            badEstimate = testCase.estimateAt(100);
            for k = 1:20
                fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                    badEstimate, 0, true, true, true, false, testCase.Dt);
            end
            testCase.verifyEqual(fsm.CurrentState, FlightState.Armed);
        end

        function testAltitudeFailsafeRecoversAndRestoresPreviousState(testCase)
            fsm = testCase.makeFsm();
            fsm.CurrentState = FlightState.FailsafeAltitude;
            fsm.PreviousState = FlightState.Flight;
            goodEstimate = testCase.estimateAt(0);  % matches setpoint 0, within tolerance
            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), ...
                goodEstimate, 0, true, true, true, true, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.Flight);
        end

        function testSensorLossFailsafeTripsAndRecovers(testCase)
            fsm = testCase.makeFsm();
            fsm.CurrentState = FlightState.Armed;
            fsm.PreviousState = FlightState.Armed;
            badReadings = testCase.nominalReadings();
            badReadings.accelStatus = SensorStatus.FAILED;
            fsm.transition(badReadings, testCase.nominalRc(), testCase.estimateAt(0), 0, ...
                true, true, true, false, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.FailsafeSensorLoss);

            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), testCase.estimateAt(0), 0, ...
                true, true, true, false, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.Armed);
        end

        function testSensorLossFailsafeNotGatedBeforeDisarmed(testCase)
            fsm = testCase.makeFsm();
            fsm.CurrentState = FlightState.MemoryCheck;
            badReadings = testCase.nominalReadings();
            badReadings.gyroStatus = SensorStatus.FAILED;
            fsm.transition(badReadings, testCase.nominalRc(), testCase.estimateAt(0), 0, ...
                false, false, false, false, testCase.Dt);
            testCase.verifyNotEqual(fsm.CurrentState, FlightState.FailsafeSensorLoss);
        end

        function testAhrsDegradedFailsafeTripsRecoversAndCallsReset(testCase)
            [fsm, mockEstimator] = testCase.makeFsm();
            fsm.CurrentState = FlightState.Flight;
            fsm.PreviousState = FlightState.Flight;
            mockEstimator.Degraded = true;
            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), testCase.estimateAt(0), 0, ...
                true, true, true, true, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.FailsafeAhrsDegraded);

            mockEstimator.Degraded = false;  % simulates AhrsSource returning to Nominal on its own
            resetCountBefore = mockEstimator.ResetCallCount;
            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), testCase.estimateAt(0), 0, ...
                true, true, true, true, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.Flight);
            testCase.verifyEqual(mockEstimator.ResetCallCount, resetCountBefore + 1);
        end

        function testRcLossInertWhenNotRequired(testCase)
            fsm = testCase.makeFsm(false);  % RcRequired = false
            fsm.CurrentState = FlightState.Flight;
            badRc.isValid = false;
            for k = 1:20
                fsm.transition(testCase.nominalReadings(), badRc, testCase.estimateAt(0), 0, ...
                    true, true, true, true, testCase.Dt);
            end
            testCase.verifyEqual(fsm.CurrentState, FlightState.Flight);
        end

        function testRcLossTripsAfterTimeoutWhenRequiredAndRecovers(testCase)
            fsm = testCase.makeFsm(true);  % RcRequired = true
            fsm.CurrentState = FlightState.Flight;
            fsm.PreviousState = FlightState.Flight;
            badRc.isValid = false;
            nSteps = round(testCase.RcLossTimeout / testCase.Dt);
            for k = 1:nSteps-1
                fsm.transition(testCase.nominalReadings(), badRc, testCase.estimateAt(0), 0, ...
                    true, true, true, true, testCase.Dt);
                testCase.verifyEqual(fsm.CurrentState, FlightState.Flight, ...
                    sprintf('Tripped early at step %d', k));
            end
            fsm.transition(testCase.nominalReadings(), badRc, testCase.estimateAt(0), 0, ...
                true, true, true, true, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.FailsafeRcLoss);
        
            goodRc.isValid = true;
            fsm.transition(testCase.nominalReadings(), goodRc, testCase.estimateAt(0), 0, ...
                true, true, true, true, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.Flight);
        end

        function testHigherPriorityFailsafeEscalatesOverLowerOne(testCase)
            [fsm, mockEstimator] = testCase.makeFsm();
            fsm.CurrentState = FlightState.Armed;
            fsm.PreviousState = FlightState.Armed;
            mockEstimator.Degraded = true;
            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), testCase.estimateAt(0), 0, ...
                true, true, true, false, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.FailsafeAhrsDegraded);

            badReadings = testCase.nominalReadings();
            badReadings.accelStatus = SensorStatus.FAILED;
            fsm.transition(badReadings, testCase.nominalRc(), testCase.estimateAt(0), 0, ...
                true, true, true, false, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.FailsafeSensorLoss);
            testCase.verifyEqual(fsm.PreviousState, FlightState.Armed);  % never clobbered by the escalation
        end

        function testDeescalatesToNextHighestNotStraightToNominal(testCase)
            [fsm, mockEstimator] = testCase.makeFsm();
            fsm.CurrentState = FlightState.FailsafeSensorLoss;
            fsm.PreviousState = FlightState.Armed;
            mockEstimator.Degraded = true;  % AHRS also degraded the whole time, just masked by higher-priority sensor loss

            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), testCase.estimateAt(0), 0, ...
                true, true, true, false, testCase.Dt);   % sensor recovers this tick
            testCase.verifyEqual(fsm.CurrentState, FlightState.FailsafeAhrsDegraded);  % not straight to Armed
            testCase.verifyEqual(fsm.PreviousState, FlightState.Armed);

            resetCountBefore = mockEstimator.ResetCallCount;
            mockEstimator.Degraded = false;
            fsm.transition(testCase.nominalReadings(), testCase.nominalRc(), testCase.estimateAt(0), 0, ...
                true, true, true, false, testCase.Dt);
            testCase.verifyEqual(fsm.CurrentState, FlightState.Armed);
            testCase.verifyEqual(mockEstimator.ResetCallCount, resetCountBefore + 1);
        end
    end
end