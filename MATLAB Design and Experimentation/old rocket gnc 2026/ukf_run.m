% state vector
% position(3), velocity(3), quaternion(4), bias_gyro(3), bias_accelerometer(3)
% Initialize the state vector with zeros
%x = [x, y, z, v_x, v_y, v_z, q_w, q_x, q_y, q_z, b_gx, b_gy, b_gz, b_ax, b_ay, b_az];
n = 16; % dimensions in state vector (for now)
x = zeros(n, 1);

% base UKF parameters
alpha = 1e-3;
beta = 2;
kappa = 0;
lambda = (alpha^2 .* (n + kappa)) - n; % Tuning parameter

dt = 0.01; % time delta

% Need rk4
