function [angle, rate] = simulateRatePlantStep(angle, rate, torque, I, c, dt)
    angularAccel = (torque - c*rate) / I;
    rate = rate + angularAccel*dt;
    angle = angle + rate*dt;
end