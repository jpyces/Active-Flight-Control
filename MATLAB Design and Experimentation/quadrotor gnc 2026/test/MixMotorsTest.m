classdef MixMotorsTest < matlab.unittest.TestCase

    properties (Constant)
        % Thrust, Roll, Pitch, Yaw
        mixMatrix = [ 
            1  1  1  1;   % motor 1: FL, CCW
            1 -1  1 -1;   % motor 2: FR, CW
            1 -1 -1  1;   % motor 3: BR, CCW
            1  1 -1 -1    % motor 4: BL, CW
        ];
    end

    methods (Test)

        function pureThrottleProducesEqualCommands(testCase)
            cmds = mixMotors(0.5, 0, 0, 0, testCase.mixMatrix);
            testCase.verifyEqual(cmds, [0.5;0.5;0.5;0.5], 'AbsTol', 1e-9);
        end

        function pureRollAffectsLeftRightMotorsSymmetrically(testCase)
            cmds = mixMotors(0.5, 0.1, 0, 0, testCase.mixMatrix);
            testCase.verifyEqual(cmds(1), cmds(4), 'AbsTol', 1e-9);  % both left motors
            testCase.verifyEqual(cmds(2), cmds(3), 'AbsTol', 1e-9);  % both right motors
            testCase.verifyGreaterThan(cmds(1), cmds(2));
            testCase.verifyEqual(mean(cmds), 0.5, 'AbsTol', 1e-9);   % roll shouldn't change net thrust
        end

        function purePitchAffectsFrontBackMotorsSymmetrically(testCase)
            cmds = mixMotors(0.5, 0, 0.1, 0, testCase.mixMatrix);
            testCase.verifyEqual(cmds(1), cmds(2), 'AbsTol', 1e-9);  % both front motors
            testCase.verifyEqual(cmds(3), cmds(4), 'AbsTol', 1e-9);  % both back motors
            testCase.verifyGreaterThan(cmds(1), cmds(3));
            testCase.verifyEqual(mean(cmds), 0.5, 'AbsTol', 1e-9);
        end

        function pureYawAffectsSpinDirectionPairsSymmetrically(testCase)
            cmds = mixMotors(0.5, 0, 0, 0.1, testCase.mixMatrix);
            testCase.verifyEqual(cmds(1), cmds(3), 'AbsTol', 1e-9);  % both CCW motors
            testCase.verifyEqual(cmds(2), cmds(4), 'AbsTol', 1e-9);  % both CW motors
            testCase.verifyGreaterThan(cmds(1), cmds(2));
            testCase.verifyEqual(mean(cmds), 0.5, 'AbsTol', 1e-9);
        end

        function highSaturationScalesProportionally(testCase)
            % raw = [1.2; 0.6; 0.6; 1.2] -- exceeds 1, no negative values
            cmds = mixMotors(0.9, 0.3, 0, 0, testCase.mixMatrix);
            testCase.verifyEqual(max(cmds), 1, 'AbsTol', 1e-9);
            testCase.verifyEqual(cmds, [1; 0.5; 0.5; 1], 'AbsTol', 1e-9);
            % scaling preserves RATIOS between motors, not raw differences
            testCase.verifyEqual(cmds(1)/cmds(2), 1.2/0.6, 'AbsTol', 1e-9);
        end

        function lowSaturationShiftsUniformly(testCase)
            % raw = [0.4; -0.2; -0.2; 0.4] -- dips negative, doesn't exceed 1
            cmds = mixMotors(0.1, 0.3, 0, 0, testCase.mixMatrix);
            testCase.verifyEqual(min(cmds), 0, 'AbsTol', 1e-9);
            testCase.verifyEqual(cmds, [0.6; 0; 0; 0.6], 'AbsTol', 1e-9);
            % shifting preserves DIFFERENCES between motors, not ratios
            testCase.verifyEqual(cmds(1) - cmds(2), 0.4 - (-0.2), 'AbsTol', 1e-9);
        end

        function combinedShiftThenScaleStaysInBounds(testCase)
            % raw = [0.7; -0.5; -0.5; 0.7] -- negative AND, after shifting, over 1
            cmds = mixMotors(0.1, 0.6, 0, 0, testCase.mixMatrix);
            testCase.verifyGreaterThanOrEqual(min(cmds), 0);
            testCase.verifyLessThanOrEqual(max(cmds), 1);
            testCase.verifyEqual(cmds, [1; 0; 0; 1], 'AbsTol', 1e-9);
        end

        function mixMatrixColumnsAreDecoupled(testCase)
            % throttle/roll/pitch/yaw shouldn't bleed into each other --
            % off-diagonal entries of M'*M should be zero
            gram = testCase.mixMatrix' * testCase.mixMatrix;
            offDiagonal = gram - diag(diag(gram));
            testCase.verifyEqual(offDiagonal, zeros(4), 'AbsTol', 1e-9);
        end

    end
end