% GYROINTEGRATIONDEMO
%   Milestone 0 step 3: gyro-only quaternion integration, no correction.
%   Tracks attitude from angular velocity alone -- will drift (expected;
%   that's what motivates the complementary filter in step 4).

% --- Simulated gyro readings (rad/s), held constant so the total motion
% is a single fixed-axis rotation with a known closed-form answer to
% check against below.
gx = 3;
gy = 4;
gz = 2;

% --- Timestep / duration. Total simulated time = nIterations*dt.
dt = 0.01;
nIterations = 100;

% --- Attitude state, starts at identity (no rotation yet).
q = Quaternion(1, 0, 0, 0);

for i = 1:nIterations
    % omega -> pure quaternion (w=0, i.e. scalar part zeroed out). Why
    % zero specifically, not just "any 4th number": the Hamilton product
    % of two PURE quaternions p=(0,p_vec), q=(0,q_vec) algebraically
    % collapses to
    %       p*q = (-dot(p_vec,q_vec), cross(p_vec,q_vec))
    % (derive it yourself: expand (p1*i+p2*j+p3*k)*(q1*i+q2*j+q3*k) using
    % i^2=j^2=k^2=-1, ij=k, ji=-k, jk=i, kj=-i, ki=j, ik=-j, then group
    % terms -- the scalar terms sum to -dot, the i/j/k terms sum to
    % cross). So w=0 isn't an arbitrary placeholder, it's the specific
    % condition that makes Hamilton multiplication reduce to ordinary
    % dot/cross vector algebra -- which is exactly the language rigid-
    % body rotational kinematics is written in (the matrix form is
    % Rdot = [omega]x * R, i.e. a cross product with omega). Quaternion
    % kinematics and matrix kinematics are the same physics; this is the
    % algebraic bridge between the two representations.
    rot_quat = Quaternion(0, gx, gy, gz);

    % qdot = 0.5*q*omega_quat. This is q's DERIVATIVE, not a rotation --
    % it depends on the current attitude q as well as omega, which is
    % why it can't be computed from the gyro reading alone. This
    % identity IS the quaternion equivalent of Rdot=[omega]x*R above:
    % q is not pure (it has a real scalar part), so this product isn't
    % a plain cross product, but it's built from the same dot/cross
    % machinery that pure*pure multiplication reduces to.
    q_dot = 0.5 * q * rot_quat;

    % Scale the rate by dt to get the actual small rotation for this one
    % timestep (rate * time = amount). This is a first-order (Euler)
    % approximation: exact only in the limit dt->0, since q_dot is
    % instantaneous but we're using it as if it stayed constant across
    % the whole dt. Forgetting this *dt is the classic bug -- without it
    % you'd add the full per-second rate every iteration instead of the
    % per-dt-second slice of it.
    q_rot = q_dot * dt;

    % Accumulate with ADDITION, not Hamilton multiplication. This only
    % works because q_rot is a small increment (first-order Taylor term),
    % not a proper unit-norm rotation quaternion in its own right -- you
    % could not take this same q_rot and compose it onto some other
    % attitude via multiply and expect a sensible rotation; it only means
    % anything as "the thing you add to q this step." (There's a
    % different, exact method that builds a real delta-rotation
    % quaternion from axis-angle and composes it via multiply instead --
    % that one doesn't need this addition step or the derivative at all.)
    q = q + q_rot;

    % Renormalize: the addition above does not preserve unit length
    % (it's a straight-line step approximating a curved path, plus
    % floating-point error compounding each iteration), so without this
    % q gradually stops being a valid rotation at all.
    q = q.normalize();
end

% --- Verify against the closed-form answer ---------------------------
% omega is constant here, so the axis of rotation never changes for the
% whole run (this ONLY holds because the gyro rate is fixed -- with a
% time-varying omega there's no single closed-form angle to check
% against, you'd need a numerical reference instead). That means the
% true total rotation angle is exactly |omega|*time, independent of dt.
totalTime = nIterations * dt;
expectedAngle = norm([gx, gy, gz]) * totalTime;

% A quaternion only ever encodes the SHORTEST-path angle, in [0,pi] --
% this is the double-cover property (q and -q represent the same
% orientation), and it means a 308 deg rotation and a 52 deg rotation
% the other way are indistinguishable in the final q. Wrap
% expectedAngle the same way before comparing, or this falsely
% "mismatches" for any total rotation > 180 deg even when the
% integration is working correctly.
expectedAngleWrapped = mod(expectedAngle, 2*pi);
if expectedAngleWrapped > pi
    expectedAngleWrapped = 2*pi - expectedAngleWrapped;
end

% Extracting the angle back out relies on q = [cos(theta/2), axis*sin(theta/2)]
% -- so w = cos(theta/2), and theta = 2*acos(w). abs() because q and -q
% are the same rotation and acos needs its input in [-1,1] (clamped via
% min() to guard floating-point overshoot past 1).
actualAngle = 2 * acos(min(1, abs(q.w)));

fprintf('Expected angle, raw:     %.4f rad (%.2f deg)\n', expectedAngle, rad2deg(expectedAngle));
fprintf('Expected angle, wrapped: %.4f rad (%.2f deg)\n', expectedAngleWrapped, rad2deg(expectedAngleWrapped));
fprintf('Actual angle:            %.4f rad (%.2f deg)\n', actualAngle, rad2deg(actualAngle));
fprintf('Difference (wrapped):    %.6f rad\n', abs(expectedAngleWrapped - actualAngle));

disp(q);