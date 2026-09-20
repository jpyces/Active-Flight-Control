function [Wm, Wc] = computeWeights(n, alpha, beta, lambda)
    % n: Dimension of the state
    % alpha, beta, kappa: Tuning parameters

    % lambda = alpha^2 .* (n + kappa) - n; % scaling parameter

    % Initialize weight vectors of size 2n+1
    Wm = zeros(2*n + 1, 1);
    Wc = zeros(2*n + 1, 1);

    % Calculate the 0-th weight (the mean point)
    Wm(1) = lambda / (n + lambda);
    Wc(1) = lambda / (n + lambda) + (1 - alpha^2 + beta);

    % Calculate the weights for the remaining 2n points
    % These are identical for both mean and covariance
    common_weight = 1 / (2 * (n + lambda));
    Wm(2:end) = common_weight;
    Wc(2:end) = common_weight;
end