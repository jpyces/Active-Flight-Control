function worldVerticalAccel = verticalAccelFromBody(accelBody, qEstimate, g)
    %VERTICALACCELFROMBODY Convert a raw body-frame accelerometer reading
    %into the gravity-compensated world-frame vertical acceleration that
    %VerticalKF.predict() expects as its aMeas input.
    %
    %   This is a REAL piece of the Milestone 1 StateEstimator orchestration
    %   (see gnc-findings.md, "State Estimator / Controller Interface
    %   Design" -- VerticalKF's aMeas is documented as "already rotated into
    %   the world frame upstream", and this is that upstream step), not
    %   simulation-only glue. It belongs wherever the real StateEstimator
    %   ends up living, not just in this synthetic-sim script.
    %
    %   Built as an explicit 3x3 derived directly from madgwickStepFull's own
    %   already-confirmed gravity-residual formula (its f_g IS
    %   R_world_to_body * [0;0;1] -- the third row of the standard
    %   quaternion-to-DCM matrix, confirmed against Madgwick's 2010 paper
    %   appendix per gnc-findings.md) rather than calling
    %   Quaternion.rotateVector, whose exact rotation-direction convention
    %   hasn't been confirmed in this conversation. This avoids introducing
    %   a second, unverified assumption on top of the one already flagged in
    %   dronePlantStep.m.
    %
    %   Uses the ESTIMATED quaternion (madgwickStepFull's output), not true
    %   state -- this is what makes it a real estimator component rather
    %   than synthetic-truth generation.
    
    w = qEstimate.w; x = qEstimate.x; y = qEstimate.y; z = qEstimate.z;
    
    Rwb = [1-2*(y^2+z^2),   2*(x*y+w*z),     2*(x*z-w*y); ...
        2*(x*y-w*z),     1-2*(x^2+z^2),   2*(y*z+w*x); ...
        2*(x*z+w*y),     2*(y*z-w*x),     1-2*(x^2+y^2)];
    
    worldAccel = Rwb' * accelBody(:);   % R_body_to_world = R_world_to_body'
    
    worldVerticalAccel = worldAccel(3) - g;
end