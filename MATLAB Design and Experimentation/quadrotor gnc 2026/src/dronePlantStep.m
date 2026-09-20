function state = dronePlantStep(state, torque, throttle, mass, I, g, dt)
    %DRONEPLANTSTEP One step of a coupled rigid-body drone plant: attitude
    %(quaternion) + body angular rate (rotational, 6 states) + altitude +
    %vertical velocity (2 states). Horizontal position/velocity deliberately
    %NOT modeled -- Milestone 1's control stack doesn't feed back on it (no
    %GPS/position loop yet), so simulating it would just be unconstrained
    %drift with nothing correcting it.
    %
    %   state.q       - Quaternion, current attitude
    %   state.omega   - 1x3, body-frame angular rate [roll pitch yaw]
    %   state.altitude, state.velocity - scalars, vertical channel
    %
    %   torque   - 1x3 [roll pitch yaw] torque demand -- AttitudeAltitudeController's
    %              PRE-MIX output, same "sidestep the mixer" approach the
    %              project's existing IntegratedController*Test.m already uses
    %   throttle - scalar, total thrust -- pre-mix throttle output
    %   I        - 1x3 [Ixx Iyy Izz], diagonal inertia. Synthetic
    %              placeholder (no real airframe exists yet).
    %
    %   Rotational dynamics: Euler's rigid-body equation WITH gyroscopic
    %   coupling (unlike the decoupled per-axis toy plant used elsewhere --
    %   this is what makes it an actual coupled model). Vertical dynamics:
    %   calls simulateAltitudePlantStep directly rather than reimplementing
    %   its physics, so the vertical channel stays identical to what
    %   AltitudeHoldTest.m already validated. Simplification: thrust is
    %   applied purely vertically here regardless of tilt -- real coupling
    %   between attitude and vertical thrust loss under large tilt isn't
    %   modeled.
    
    omegaDot = zeros(1,3);
    omegaDot(1) = (torque(1) - (I(3)-I(2))*state.omega(2)*state.omega(3)) / I(1);
    omegaDot(2) = (torque(2) - (I(1)-I(3))*state.omega(1)*state.omega(3)) / I(2);
    omegaDot(3) = (torque(3) - (I(2)-I(1))*state.omega(1)*state.omega(2)) / I(3);
    
    state.omega = state.omega + omegaDot * dt;
    
    % Quaternion.m has no integrateGyro method -- built directly from the
    % primitives it does have, using the same qdot = 0.5*q*omega formula
    % documented as standard throughout this project (gnc-findings.md,
    % Madgwick "Core mechanism": "propagate attitude from the gyro
    % (qdot = 0.5*q(x)omega)"). omega is treated as a pure-vector
    % quaternion (0, wx, wy, wz) and right-multiplied, i.e. body-frame
    % angular rate -- consistent with state.omega being defined as
    % body-frame throughout this file.
    omegaQuat = Quaternion(0, state.omega(1), state.omega(2), state.omega(3));
    qdot = state.q.multiply(omegaQuat) * 0.5;
    %state.q = (state.q + qdot*dt).normalize();
    state.q = (state.q + qdot*dt);
    state.q = state.q.normalize();
    
    [state.altitude, state.velocity] = simulateAltitudePlantStep( ...
        state.altitude, state.velocity, throttle, mass, g, dt);
end