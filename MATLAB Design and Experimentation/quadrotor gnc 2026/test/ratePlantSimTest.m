dt = 0.01; N = 10000;
I = 0.01; c = 0.001;

KpRate = 0.05;
KiRate = 0.01; 
KdRate = 0.001;
KpAngle = 3;
KiAngle = 0; 
KdAngle = 0.5;

angleLoopPID = PIDControllerBase(KpAngle, KiAngle, KdAngle, dt, -200, 200);   % output = desired rate (deg/s), bounded
rateLoopPID  = PIDControllerBase(KpRate,  KiRate,  KdRate,  dt, -1, 1);        % output = torque, bounded

angle = 30;  % starting tilted 30 degrees -- our "disturbance"
rate  = 0;
angleSetpoint = 0;  % goal: level

angleLog = zeros(N,1); rateLog = zeros(N,1);

for k = 1:N
    desiredRate = angleLoopPID.update(angleSetpoint, angle);
    torque      = rateLoopPID.update(desiredRate, rate);
    [angle, rate] = simulateRatePlantStep(angle, rate, torque, I, c, dt);
    angleLog(k) = angle; rateLog(k) = rate;
end

plot((1:N)*dt, angleLog); xlabel('time (s)'); ylabel('angle (deg)');