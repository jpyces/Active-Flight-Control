classdef Quaternion
    % QUATERNION  Hamilton-convention quaternion for attitude representation.
    %
    %   Storage order is scalar-first: q = w + x*i + y*j + z*k.
    %   Multiplication follows the Hamilton product (i*j=k, j*k=i, k*i=j),
    
    properties
        w {mustBeNumeric} = 1
        x {mustBeNumeric} = 0
        y {mustBeNumeric} = 0
        z {mustBeNumeric} = 0
    end

    methods
        function obj = Quaternion(w, x, y, z)
            if nargin == 0
                return % keep property defaults -> identity quaternion
            end
            obj.w = w;
            obj.x = x;
            obj.y = y;
            obj.z = z;
        end

        function r = multiply(obj, q2)
            % Hamilton product obj * q2. Deliberately does NOT normalize
            % either operand first: multiply must work on non-unit
            % quaternions too (pure-vector quaternions in rotateVector,
            % intermediate results during gyro integration, etc).
            r = Quaternion( ...
                obj.w*q2.w - obj.x*q2.x - obj.y*q2.y - obj.z*q2.z, ...
                obj.w*q2.x + obj.x*q2.w + obj.y*q2.z - obj.z*q2.y, ...
                obj.w*q2.y - obj.x*q2.z + obj.y*q2.w + obj.z*q2.x, ...
                obj.w*q2.z + obj.x*q2.y - obj.y*q2.x + obj.z*q2.w);
        end
        


        % OPERATOR ------
        function r = mtimes(a, b)
            % Enables q1 * q2 (Hamilton product) and q * s / s * q
            % (component-wise scalar scaling -- equivalent to Hamilton-
            % multiplying by the real quaternion (s,0,0,0), so this is
            % the same operation, not a shortcut around it).
            if isa(a, 'Quaternion') && isa(b, 'Quaternion')
                r = a.multiply(b);
            elseif isa(a, 'Quaternion') && isnumeric(b) && isscalar(b)
                r = Quaternion(a.w*b, a.x*b, a.y*b, a.z*b);
            elseif isnumeric(a) && isscalar(a) && isa(b, 'Quaternion')
                r = Quaternion(b.w*a, b.x*a, b.y*a, b.z*a);
            else
                error('Quaternion:mtimes:invalidOperands', ...
                    'mtimes supports Quaternion*Quaternion or Quaternion*scalar.');
            end
        end

        function r = plus(a, b)
            % Component-wise quaternion addition. NOT a rotation
            % composition (that's multiply/mtimes) -- this is the raw
            % vector-space addition needed for integrating qdot*dt and
            % for filter correction terms (Madgwick's gradient step,
            % complementary blending).
            r = Quaternion(a.w+b.w, a.x+b.x, a.y+b.y, a.z+b.z);
        end

        function r = minus(a, b)
            r = Quaternion(a.w-b.w, a.x-b.x, a.y-b.y, a.z-b.z);
        end
        % OPERATOR --- 


        
        function n = norm(obj)
            n = sqrt(obj.w^2 + obj.x^2 + obj.y^2 + obj.z^2);
        end

        function q_unit = normalize(obj)
            n = obj.norm();
            if n == 0
                error('Quaternion:normalize:zeroNorm', ...
                    'Cannot normalize a zero-norm quaternion.');
            end
            q_unit = Quaternion(obj.w/n, obj.x/n, obj.y/n, obj.z/n);
        end

        function q_conj = conjugate(obj)
            q_conj = Quaternion(obj.w, -obj.x, -obj.y, -obj.z);
        end

        function q_inv = inverse(obj)
            n2 = obj.norm()^2;
            if n2 == 0
                error('Quaternion:inverse:zeroNorm', ...
                    'Cannot invert a zero-norm quaternion.');
            end
            q_conj = obj.conjugate();
            q_inv = Quaternion(q_conj.w/n2, q_conj.x/n2, q_conj.y/n2, q_conj.z/n2);
        end

        function tf = isequal(obj, other)
            if ~isa(other, 'Quaternion')
                tf = false;
                return
            end
            tf = isequal(obj.w, other.w) && isequal(obj.x, other.x) && ...
                 isequal(obj.y, other.y) && isequal(obj.z, other.z);
        end

        function v_rot = rotateVector(obj, v)
            % Rotate 3-vector v (column or row, 3 elements) by this
            % quaternion: v_rot = q * [0;v] * q_conjugate, vector part.
            % Normalizes obj first since rotation is only meaningful for
            % unit quaternions -- keep obj normalized in your integration
            % loop rather than relying on this to paper over drift.
            q = obj.normalize();
            v_quat = Quaternion(0, v(1), v(2), v(3));
            r = q.multiply(v_quat).multiply(q.conjugate());
            v_rot = [r.x; r.y; r.z];
        end

        function eul = toEulerZYX(obj)
            % 3-2-1 (yaw-pitch-roll, aerospace ZYX) Euler angles, radians.
            % Returns [roll; pitch; yaw].
            q = obj.normalize();
            roll = atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x^2 + q.y^2));
            sinp = 2*(q.w*q.y - q.z*q.x);
            sinp = max(min(sinp, 1), -1); % clamp: guards asin() at gimbal lock
            pitch = asin(sinp);
            yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y^2 + q.z^2));
            eul = [roll; pitch; yaw];
        end

        function disp(obj)
            fprintf('Quaternion: %.6f + %.6fi + %.6fj + %.6fk\n', ...
                obj.w, obj.x, obj.y, obj.z);
        end
    end

    methods (Static)
        function q = fromEulerZYX(roll, pitch, yaw)
            % Inverse of toEulerZYX: build a quaternion from 3-2-1 Euler
            % angles (radians). Round-trips with toEulerZYX away from
            cr = cos(roll/2);  sr = sin(roll/2);
            cp = cos(pitch/2); sp = sin(pitch/2);
            cy = cos(yaw/2);   sy = sin(yaw/2);
            q = Quaternion( ...
                cr*cp*cy + sr*sp*sy, ...
                sr*cp*cy - cr*sp*sy, ...
                cr*sp*cy + sr*cp*sy, ...
                cr*cp*sy - sr*sp*cy);
        end

        function q = identity()
            q = Quaternion(1, 0, 0, 0);
        end
    end
end