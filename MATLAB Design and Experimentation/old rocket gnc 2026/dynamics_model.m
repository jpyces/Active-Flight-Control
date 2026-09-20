function x_next = dynamics_model(x, dt)
% DYNAMICS_MODEL  f(x, dt) - simple default propagation model
%
% This is a "constant state" (random walk) model: it assumes the state
% doesn't change on its own between measurements, and lets Q (process
% noise) account for the uncertainty that grows as a result. This is a
% standard placeholder to use before you have a real physics model -- the
% UKF still works correctly with it, it just relies more heavily on
% frequent sensor corrections to stay accurate, since nothing is being
% predicted about HOW the state evolves.
%
% Input:
%   x  - n x 1 current state
%   dt - timestep [s] (unused here, kept for interface consistency so
%        predict.m doesn't need to change once you swap in real dynamics)
%
% Output:
%   x_next - n x 1 predicted state (identical to x in this simple version)

x_next = x;

% --- Once you have real dynamics, replace pieces of this. For
% example, if you add velocity states alongside your position/altitude
% states, basic kinematic integration would look like:
%
%   x_next(altitude_idx) = x(altitude_idx) + x(vertVel_idx) * dt;
%   x_next(vertVel_idx)  = x(vertVel_idx) + x(vertAccel_idx) * dt;
%
% Leave any state with no known equation of motion (like raw
% accel/gyro readings carried in the state) as x_next(i) = x(i) --
% that's still the random walk model, just applied selectively.

end