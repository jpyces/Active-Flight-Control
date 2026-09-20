function q_new = complementaryFilterStep(q, gyro, accel, dt, alpha)
    %COMPLEMENTARYFILTERSTEP One step of a quaternion-space complementary
    %filter (accel + gyro). Built here specifically as an INDEPENDENT
    %cross-check reference for madgwickStepFull/madgwickNoMag's roll/pitch
    %output (Milestone 0 step 6's gate) -- not part of the flight filter
    %chain itself, and deliberately a different mathematical approach so
    %agreement between the two is actually meaningful.
    %
    %   Structurally different from Madgwick: instead of nudging the
    %   quaternion DERIVATIVE by a small gradient-descent correction each
    %   step, this blends two FULL competing orientation estimates
    %   directly -- the gyro-integrated orientation and an accel-derived
    %   tilt-only orientation -- via weighted addition. This is exactly the
    %   "naive" approach from the original hand-drawn design: simpler to
    %   reason about, but it's also the design that risks yaw corruption if
    %   the accel branch's forced yaw=0 leaks into the blended result once
    %   roll/pitch are nonzero (quaternion composition doesn't decompose
    %   per-axis the way Euler angles do). Worth watching for that
    %   specifically on a yaw-heavy bench test, since it's the main
    %   qualitative difference you'd expect between this and Madgwick.
    %
    %   Inputs:
    %     q     - Quaternion, current attitude estimate
    %     gyro  - 1x3 vector, body-frame angular velocity (rad/s)
    %     accel - 1x3 vector, raw accelerometer reading (gets normalized)
    %     dt    - scalar, timestep (s)
    %     alpha - scalar in (0,1), weight on the gyro branch (e.g. 0.98);
    %             the accel branch gets weight (1 - alpha)
    %
    %   Output:
    %     q_new - Quaternion, blended and renormalized estimate
    
    % --- Gyro branch: reuse integrateGyroStep directly -- same
    % "predict" half every filter here shares.
    q_gyro = integrateGyroStep(q, gyro, dt);
    
    % --- Accel branch: 2DOF tilt-only quaternion. Yaw is unobservable
    % from accel alone, so it's forced to 0 here -- see the docstring
    % warning above about what that assumption can do once blended.
    a = accel / norm(accel);
    roll  = atan2(a(2), a(3));
    pitch = atan2(-a(1), sqrt(a(2)^2 + a(3)^2));
    q_accel = Quaternion.fromEulerZYX(roll, pitch, 0);
    
    % --- Blend: weighted quaternion ADDITION, not Hamilton
    % multiplication -- same "not a proper rotation on its own, only
    % meaningful as a combination" caveat as q_rot in
    % integrateGyroStep, just combining two full quaternions instead of
    % an increment.
    q_blend = alpha * q_gyro + (1 - alpha) * q_accel;
    
    % --- Renormalize: weighted addition of two unit quaternions is not
    % itself unit-norm in general.
    q_new = q_blend.normalize();
end