function q_new = madgwickNoMag(q, gyro, accel, dt, beta)
    % REFERENCE: https://courses.cs.washington.edu/courses/cse466/14au/labs/l4/madgwick_internal_report.pdf
    %MADGWICKNOMAG One step of Madgwick's filter, accel+gyro only (no
    %mag yet -- Milestone 0 step 5, first half). Corrects the gyro-predicted
    %quaternion derivative using a gradient-descent step against the
    %measured accelerometer direction, then integrates once.
    %
    %   Inputs:
    %     q     - Quaternion, current attitude estimate
    %     gyro  - 1x3 vector, body-frame angular velocity (rad/s)
    %     accel - 1x3 vector, raw accelerometer reading (any consistent
    %             units -- gets normalized to a pure direction below)
    %     dt    - scalar, timestep (s)
    %     beta  - scalar gain, how much to trust the accel correction
    %             relative to the gyro (start small, e.g. 0.01-0.1, and
    %             tune against your bench test)
    %
    %   Output:
    %     q_new - Quaternion, corrected + propagated estimate (unit norm)
        gx = gyro(1); gy = gyro(2); gz = gyro(3);
    % --- Step 1: gyro-predicted derivative -- same math as
    % integrateGyroStep, but STOP after computing qdot. Don't scale by
    % dt or integrate yet; you need qdot itself to blend with the
    % correction below, and both terms need to come from the SAME q
    % (the one passed in, not an updated one).
        rot_quat = Quaternion(0, gx, gy, gz);
        qdot_gyro = 0.5 * q * rot_quat;
    
    % --- Step 2: check accel validity and build f(q, a_hat)/J(q) --
    % ZERO-NORM GUARD, fixed to match madgwickStepFull's treatment (see
    % madgwickFullEdgeCasesTest.m case 1): an invalid/near-zero accel
    % reading must zero out BOTH the residual f AND the Jacobian J
    % directly, not get substituted as a fake [0,0,0] reading routed
    % through the normal formulas. f's formula is NOT zero at accel=[0,0,0]
    % (e.g. f=[0;0;1] at identity q) -- feeding it a fake zero vector
    % therefore injects a spurious full-magnitude correction on exactly
    % the step where the sensor glitched and no correction should happen
    % at all. This is the same bug madgwickStepFull had and was fixed
    % away from; madgwickNoMag is called on StateEstimator's mag-degraded
    % failover path, so it needs the identical fix.
        accelValid = norm(accel) > eps;
    
        if accelValid
            % f() below assumes |a_hat| = 1, since it's comparing against
            % the unit gravity reference -- skip this and beta will need
            % constant re-tuning as accel magnitude drifts with
            % battery/sensor noise.
            accel = accel / norm(accel);
    
            % residual f(q, a_hat), 3x1. This is how far the gravity
            % direction predicted by your CURRENT q is from what the
            % accelerometer actually measured.
            f = [2*(q.x*q.z - q.w*q.y) - accel(1);
                 2*(q.w*q.x + q.y*q.z) - accel(2);
                 2*(0.5 - q.x^2 - q.y^2)  - accel(3)];
    
            % Jacobian J(q), 3x4 -- the analytic derivative of f wrt each
            % quaternion component. Fixed algebra, not something to
            % derive by hand each time.
            J = [ -2*q.y,  2*q.z, -2*q.w,  2*q.x;
                    2*q.x,  2*q.w,  2*q.z,  2*q.y;
                    0,      -4*q.x, -4*q.y, 0];
        else
            % No usable accel this step -- zero the residual AND the
            % Jacobian, so the gradient computed below comes out exactly
            % zero rather than a spurious nonzero value.
            f = zeros(3,1);
            J = zeros(3,4);
        end
    
    % --- Step 3: gradient = J' * f (4x1), then normalize it to a pure
    % direction. Guard against norm(gradient) being ~0 (happens when f
    % is already ~0, i.e. your estimate already matches the
    % measurement -- or, now, when accel was invalid this step and f/J
    % were zeroed above).
        gradient = J' * f;
        if norm(gradient) > eps
            gradient_hat = gradient / norm(gradient);
        else
            gradient_hat = zeros(4,1);  % estimate already matches accel,
            % or accel was invalid this step -- no correction needed
        end
    
    % --- Step 4: blend. qdot_corrected = qdot_gyro - beta*gradient_hat.
    % qdot_gyro is a Quaternion object; gradient_hat is a plain 4-vector
    % in [w,x,y,z] order -- bridge those by wrapping the gradient in a
    % Quaternion via its constructor.
        qdot_corrected = qdot_gyro - beta*Quaternion(gradient_hat(1), ...
            gradient_hat(2), gradient_hat(3), gradient_hat(4));
    
    % --- Step 5: integrate + normalize -- identical to the last two
    % lines of integrateGyroStep. q_new = q + qdot_corrected*dt, then
    % normalize.
        q_rot = qdot_corrected * dt;
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
        q_new = q.normalize();
end