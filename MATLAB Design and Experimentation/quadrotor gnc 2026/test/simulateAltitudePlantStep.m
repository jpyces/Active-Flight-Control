function [altitude, velocity] = simulateAltitudePlantStep(altitude, velocity, thrust, mass, g, dt)
    accel = thrust/mass - g;
    velocity = velocity + accel*dt;
    altitude = altitude + velocity*dt;
end