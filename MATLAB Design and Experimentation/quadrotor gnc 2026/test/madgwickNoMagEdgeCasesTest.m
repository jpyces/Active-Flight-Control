classdef madgwickNoMagEdgeCasesTest < matlab.unittest.TestCase
    %MADGWICKNOMAGEDGECASESTEST Edge-case coverage for madgwickNoMag.m,
    %mirroring madgwickFullEdgeCasesTest.m's zero-norm/magnitude-invariance
    %cases (no near-pole-field case here, since madgwickNoMag has no
    %magnetometer branch to degrade).
    %
    % The load-bearing test is testZeroNormAccelProducesNoCorrection: this
    % is the regression test for the bug found 2026-09-16 and fixed
    % 2026-09-18, where a zero-norm accel guard substituted accel=[0,0,0]
    % directly into f()'s formula instead of zeroing f/J themselves --
    % f([0,0,0]) at identity evaluates to [0;0;1], a spurious
    % full-magnitude correction, not a suppressed one. See
    % claude/gnc-findings.md's Madgwick section for the full writeup.

    properties (Constant)
        dt = 0.01;
        beta = 0.1;
    end

    methods (Test)

        function testZeroNormAccelProducesNoCorrection(testCase)
            % THE regression test for the fixed bug. At identity q with
            % zero gyro and an invalid (zero-norm) accel reading, output
            % must be exactly q0 -- no gyro motion to integrate, and no
            % correction should be injected from a sensor glitch. Before
            % the fix, f([0,0,0]) evaluated to [0;0;1] at identity,
            % producing a real, wrong correction even here.
            q0 = Quaternion.identity();
            gyro = [0, 0, 0];
            accel = [0, 0, 0];

            qActual = madgwickNoMag(q0, gyro, accel, testCase.dt, testCase.beta);

            testCase.verifyTrue(qActual.isequal(q0), ...
                'Zero-norm accel with zero gyro must leave q unchanged -- got a spurious correction.');
        end

        function testZeroNormAccelMatchesPureGyroIntegration(testCase)
            % With a nonzero gyro but an invalid accel, output must match
            % plain gyro-only integration (qdot_gyro*dt, normalized) --
            % i.e. the accel branch must contribute NOTHING, not even a
            % small residual push, when accel is invalid.
            q0 = Quaternion.fromEulerZYX(0.3, -0.2, 0.5);
            gyro = [0.1, -0.2, 0.05];
            accel = [0, 0, 0];

            qActual = madgwickNoMag(q0, gyro, accel, testCase.dt, testCase.beta);

            rotQuat = Quaternion(0, gyro(1), gyro(2), gyro(3));
            qdotGyro = 0.5 * q0 * rotQuat;
            qExpected = normalize(q0 + qdotGyro * testCase.dt);

            testCase.verifyEqual(qActual.w, qExpected.w, 'AbsTol', 1e-12);
            testCase.verifyEqual(qActual.x, qExpected.x, 'AbsTol', 1e-12);
            testCase.verifyEqual(qActual.y, qExpected.y, 'AbsTol', 1e-12);
            testCase.verifyEqual(qActual.z, qExpected.z, 'AbsTol', 1e-12);
        end

        function testNearZeroNormBelowEpsTreatedAsInvalid(testCase)
            % A reading just under the eps threshold must take the same
            % invalid-branch path as an exact [0,0,0] -- confirms the
            % guard is norm(accel) > eps, not e.g. any(accel ~= 0).
            q0 = normalize(Quaternion.fromEulerZYX(0.1, 0.2, -0.4));
            gyro = [0, 0, 0];
            accelTinyNorm = [eps/10, 0, 0];

            qActual = madgwickNoMag(q0, gyro, accelTinyNorm, testCase.dt, testCase.beta);

            testCase.verifyTrue(qActual.isequal(q0), ...
                'A near-zero-norm accel (below eps) must be treated as invalid, same as exact zero.');
        end

        function testNaNAccelTreatedAsInvalidNotPropagated(testCase)
            % norm(accel) with a NaN component is NaN, and NaN > eps is
            % false in MATLAB -- so a NaN reading should already fall
            % into the invalid branch without any extra guard. Confirms
            % that (not just documents it) and that no NaN leaks into q.
            q0 = Quaternion.fromEulerZYX(0.2, 0.1, 0.3);
            gyro = [0.05, 0, 0];
            accelNaN = [NaN, 0, 1];

            qActual = madgwickNoMag(q0, gyro, accelNaN, testCase.dt, testCase.beta);

            testCase.verifyFalse(any(isnan([qActual.w, qActual.x, qActual.y, qActual.z])), ...
                'A NaN accel reading must not propagate NaN into the output quaternion.');

            % Must match pure gyro integration, same as the other invalid cases.
            rotQuat = Quaternion(0, gyro(1), gyro(2), gyro(3));
            qdotGyro = 0.5 * q0 * rotQuat;
            qExpected = normalize(q0 + qdotGyro * testCase.dt);
            testCase.verifyEqual(qActual.w, qExpected.w, 'AbsTol', 1e-12);
            testCase.verifyEqual(qActual.x, qExpected.x, 'AbsTol', 1e-12);
            testCase.verifyEqual(qActual.y, qExpected.y, 'AbsTol', 1e-12);
            testCase.verifyEqual(qActual.z, qExpected.z, 'AbsTol', 1e-12);
        end

        function testValidAccelStillCorrects(testCase)
            % Sanity check that the fix didn't accidentally also suppress
            % the VALID-accel path: a tilted starting quaternion with a
            % valid gravity-only accel reading and zero gyro should move
            % measurably toward gravity, not sit frozen like the
            % invalid-accel cases above do.
            q0 = Quaternion.fromEulerZYX(0.3, 0, 0); % true tilt, no yaw
            gyro = [0, 0, 0];
            accelGravity = [0, 0, 1]; % valid, unit-norm-ish reading

            qActual = madgwickNoMag(q0, gyro, accelGravity, testCase.dt, testCase.beta);

            testCase.verifyFalse(qActual.isequal(q0), ...
                'A valid accel reading must still produce a nonzero correction step.');
        end

        function testMagnitudeInvarianceOfValidAccel(testCase)
            % Since accel is normalized before use, scaling a valid
            % reading by an arbitrary positive factor must not change the
            % resulting trajectory -- only direction matters.
            q0 = Quaternion.fromEulerZYX(0.2, -0.15, 0.4);
            gyro = [0.02, -0.01, 0.03];
            accelBase = [0.1, 0.2, 0.9];
            scaleFactor = 5.7;

            qUnscaled = madgwickNoMag(q0, gyro, accelBase, testCase.dt, testCase.beta);
            qScaled = madgwickNoMag(q0, gyro, accelBase * scaleFactor, testCase.dt, testCase.beta);

            testCase.verifyEqual(qUnscaled.w, qScaled.w, 'AbsTol', 1e-10);
            testCase.verifyEqual(qUnscaled.x, qScaled.x, 'AbsTol', 1e-10);
            testCase.verifyEqual(qUnscaled.y, qScaled.y, 'AbsTol', 1e-10);
            testCase.verifyEqual(qUnscaled.z, qScaled.z, 'AbsTol', 1e-10);
        end

        function testOutputStaysUnitNorm(testCase)
            % Regardless of accel validity, output must always be a
            % legitimate unit quaternion -- the zero-norm branch's
            % zeroed f/J must still flow through the same
            % integrate-then-normalize path as the valid branch.
            q0 = Quaternion.fromEulerZYX(0.1, 0.1, 0.1);
            gyro = [0.05, 0.05, 0.05];

            qInvalid = madgwickNoMag(q0, gyro, [0, 0, 0], testCase.dt, testCase.beta);
            qValid = madgwickNoMag(q0, gyro, [0, 0, 1], testCase.dt, testCase.beta);

            testCase.verifyEqual(qInvalid.norm(), 1, 'AbsTol', 1e-12);
            testCase.verifyEqual(qValid.norm(), 1, 'AbsTol', 1e-12);
        end

    end
end