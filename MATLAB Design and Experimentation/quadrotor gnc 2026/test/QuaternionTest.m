classdef QuaternionTest < matlab.unittest.TestCase
    % QUATERNIONTEST  Unit tests for Quaternion.m
    %
    %   Run with:  runtests('QuaternionTest')
    %   Covers the cases for GNC: identity/no-op,
    %   composed 90+90=180 rotation, norm stays 1, Euler round-trip,
    %   optional spot-check against a MATLAB toolbox quaternion type
    %   (reference only -- never treated as the real implementation).

    properties (Constant)
        Tol = 1e-10
    end

    methods (Test)

        function identityMultiplyIsNoOp(tc)
            q = Quaternion(0.2, 0.4, -0.1, 0.8).normalize();
            id = Quaternion.identity();
            r = q.multiply(id);
            tc.verifyEqual(r.w, q.w, 'AbsTol', tc.Tol);
            tc.verifyEqual(r.x, q.x, 'AbsTol', tc.Tol);
            tc.verifyEqual(r.y, q.y, 'AbsTol', tc.Tol);
            tc.verifyEqual(r.z, q.z, 'AbsTol', tc.Tol);
        end

        function identityRotationIsNoOp(tc)
            v = [1; 2; 3];
            v_rot = Quaternion.identity().rotateVector(v);
            tc.verifyEqual(v_rot, v, 'AbsTol', tc.Tol);
        end

        function composed90Plus90Equals180AboutZ(tc)
            % Two 90 deg rotations about z composed should equal one 180
            % deg rotation about z. Built directly from the axis-angle
            % half-angle formula (not via toEulerZYX/fromEulerZYX, so
            % this test doesn't depend on the Euler conversion methods).
            q90z  = Quaternion(cos(pi/4), 0, 0, sin(pi/4)); % 90 deg about z
            q180z = Quaternion(cos(pi/2), 0, 0, sin(pi/2)); % 180 deg about z

            composed = q90z.multiply(q90z);

            tc.verifyEqual(composed.w, q180z.w, 'AbsTol', tc.Tol);
            tc.verifyEqual(composed.x, q180z.x, 'AbsTol', tc.Tol);
            tc.verifyEqual(composed.y, q180z.y, 'AbsTol', tc.Tol);
            tc.verifyEqual(composed.z, q180z.z, 'AbsTol', tc.Tol);

            % And the physical check: rotating [1;0;0] by the composed
            % quaternion should land on [-1;0;0].
            v_rot = composed.rotateVector([1; 0; 0]);
            tc.verifyEqual(v_rot, [-1; 0; 0], 'AbsTol', tc.Tol);
        end

        function normalizeProducesUnitNorm(tc)
            q = Quaternion(3, -1, 2, 0.5);
            q_unit = q.normalize();
            tc.verifyEqual(q_unit.norm(), 1, 'AbsTol', tc.Tol);
        end

        function conjugateEqualsInverseForUnitQuaternion(tc)
            q = Quaternion(1, 2, -3, 0.5).normalize();
            q_inv = q.inverse();
            % q * q_inv should be identity
            r = q.multiply(q_inv);
            tc.verifyEqual(r.w, 1, 'AbsTol', tc.Tol);
            tc.verifyEqual(r.x, 0, 'AbsTol', tc.Tol);
            tc.verifyEqual(r.y, 0, 'AbsTol', tc.Tol);
            tc.verifyEqual(r.z, 0, 'AbsTol', tc.Tol);
        end

        function multiplyIsNotCommutative(tc)
            % Hamilton product is non-commutative in general -- catches
            % a common sign/order bug if someone "simplifies" multiply
            % later and accidentally makes it commutative. Compare the
            % full quaternion, not one component: for some operand pairs
            % an individual component can coincidentally match even
            % though the products differ overall (e.g. (1,1,0,0) and
            % (1,0,1,0) normalized give equal .x but differing .z).
            q1 = Quaternion(1, 1, 0, 0).normalize();
            q2 = Quaternion(1, 0, 1, 0).normalize();
            ab = q1.multiply(q2);
            ba = q2.multiply(q1);
            tc.verifyNotEqual([ab.w ab.x ab.y ab.z], [ba.w ba.x ba.y ba.z]);
        end

        function eulerRoundTripAwayFromGimbalLock(tc)
            roll  = deg2rad(20);
            pitch = deg2rad(-35);
            yaw   = deg2rad(120);

            q = Quaternion.fromEulerZYX(roll, pitch, yaw);
            eul = q.toEulerZYX();

            tc.verifyEqual(eul(1), roll,  'AbsTol', 1e-9);
            tc.verifyEqual(eul(2), pitch, 'AbsTol', 1e-9);
            tc.verifyEqual(eul(3), yaw,   'AbsTol', 1e-9);
        end

        function fromEulerProducesUnitQuaternion(tc)
            q = Quaternion.fromEulerZYX(deg2rad(45), deg2rad(-10), deg2rad(200));
            tc.verifyEqual(q.norm(), 1, 'AbsTol', tc.Tol);
        end

        function toolboxReferenceSpotCheck(tc)
            % Optional cross-check against a MATLAB toolbox quaternion
            % type, if one is installed (Robotics System / Navigation /
            % Sensor Fusion and Tracking / Aerospace Toolbox all ship a
            % `quaternion` type). This is a reference to compare against,
            % never a replacement for the from-scratch implementation.
            tc.assumeTrue(logical(exist('quaternion', 'class')), ...
                'No toolbox quaternion type on the path -- skipping spot-check.');

            roll = deg2rad(15); pitch = deg2rad(25); yaw = deg2rad(-40);

            mine = Quaternion.fromEulerZYX(roll, pitch, yaw);

            % Toolbox quaternion from the same ZYX Euler angles, same
            % (roll,pitch,yaw)=(X,Y,Z) rotation sequence order.
            % rotationType 'frame' (not 'point'): our fromEulerZYX builds
            % the standard aerospace *attitude* quaternion, which rotates
            % the reference frame (passive), not a fixed point in space
            % (active) -- 'point' is the wrong convention to compare
            % against and will not match even up to overall sign.
            ref = quaternion([yaw pitch roll], 'euler', 'ZYX', 'frame');
            [rw, rx, ry, rz] = parts(ref);

            % Quaternions double-cover rotations (q and -q are the same
            % rotation), so compare up to overall sign.
            sameSign = sign(mine.w) == sign(rw) || (mine.w == 0 && rw == 0);
            if ~sameSign
                rw = -rw; rx = -rx; ry = -ry; rz = -rz;
            end

            tc.verifyEqual(mine.w, rw, 'AbsTol', 1e-9);
            tc.verifyEqual(mine.x, rx, 'AbsTol', 1e-9);
            tc.verifyEqual(mine.y, ry, 'AbsTol', 1e-9);
            tc.verifyEqual(mine.z, rz, 'AbsTol', 1e-9);
        end

    end
end