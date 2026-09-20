function readings = syntheticSensors(state, appliedThrottle, mass, noiseStd)
    %SYNTHETICSENSORS Generate a SensorReadings-shaped struct from the true
    %plant state, for one timestep of the closed-loop synthetic simulation.
    %
    %   appliedThrottle should be whatever throttle the plant is CURRENTLY
    %   under (i.e. last step's commanded throttle, not this step's not-yet-
    %   computed one) -- the accelerometer reads the physical consequence of
    %   the command already in effect, not a command that hasn't happened yet.
    %
    %   Magnetometer: same reverse-residual technique as madgwickFullTest.m /
    %   StateEstimateIntegrationTest.m -- evaluating madgwickStepFull's own
    %   residual formula in reverse at the CURRENT true quaternion, so this
    %   can't silently disagree with the filter's own convention, and needs
    %   no assumption about Quaternion.rotateVector's direction.
    %
    %   Accelerometer: CORRECTED (see gnc-findings.md changelog / chat history
    %   -- an earlier version of this file used [0,0,appliedThrottle/mass]
    %   directly, reasoning that thrust along body-z makes specific force
    %   frame-independent. That's true in isolation, but it silently
    %   contradicts dronePlantStep.m's vertical dynamics (via
    %   simulateAltitudePlantStep), which treats thrust as acting in the
    %   WORLD-vertical direction regardless of tilt. Given that convention,
    %   the correct body-frame reading is the WORLD-frame specific force
    %   [0,0,appliedThrottle/mass] rotated INTO body frame by the true
    %   attitude -- which is what actually lets an accelerometer sense tilt
    %   in this model. Built from the same closed-form expression already
    %   confirmed against Madgwick's 2010 paper appendix (madgwickFullTest.m's
    %   accelTrue, here evaluated at the TRUE evolving quaternion each step
    %   and scaled by throttle/mass instead of a fixed magnitude) rather than
    %   Quaternion.rotateVector, for the same reason given in
    %   verticalAccelFromBody.m. The old version made Madgwick converge to
    %   "level" regardless of true tilt, since it never saw tilt at all.
    %
    %   Gyro: the true body angular rate directly, plus noise -- that IS what
    %   a gyro measures.
    
        w = state.q.w; x = state.q.x; y = state.q.y; z = state.q.z;
        dipAngle = deg2rad(60);
        bxRef = cos(dipAngle);
        bzRef = sin(dipAngle);
        magTrue = [2*bxRef*(0.5 - y^2 - z^2) + 2*bzRef*(x*z - w*y), ...
            2*bxRef*(x*y - w*z)       + 2*bzRef*(w*x + y*z), ...
            2*bxRef*(w*y + x*z)       + 2*bzRef*(0.5 - x^2 - y^2)];
    
        specificForceScale = appliedThrottle / mass;
        accelTrue = specificForceScale * ...
            [2*(x*z - w*y), 2*(w*x + y*z), 2*(0.5 - x^2 - y^2)];
        gyroTrue = state.omega;
        baroTrue = state.altitude;
    
        readings = SensorReadings( ...
            'accel', accelTrue + noiseStd.accel * randn(1,3), ...
            'accelStatus', SensorStatus.NOMINAL, ...
            'gyro', gyroTrue + noiseStd.gyro * randn(1,3), ...
            'gyroStatus', SensorStatus.NOMINAL, ...
            'mag', magTrue + noiseStd.mag * randn(1,3), ...
            'magStatus', SensorStatus.NOMINAL, ...
            'baroAltitude', baroTrue + noiseStd.baro * randn(), ...
            'baroStatus', SensorStatus.NOMINAL);
end