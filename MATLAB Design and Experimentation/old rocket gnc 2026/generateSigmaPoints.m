function sigmaPoints = generateSigmaPoints(x, P, n, lambda)
    % x: Mean state vector (n x 1)
    % P: Covariance matrix (n x n)
    % n: Number of dimensions (scalar)
    % alpha, beta, kappa: Tuning parameters
    
    % Cholesky decomposition of covariance to get matrix square root. places points 1 std away from mean  
    L = chol((n + lambda) * P, 'lower');

    sigmaPoints = zeros(n, (2 .* n) + 1);

    sigmaPoints(:, 1) = x;

    for i = 1:n
        sigmaPoints(:, 1 + i) = x + L(:, i);
    end

    for j = 1:n
        sigmaPoints(:, 1 + n + j) = x - L(:, j);
    end
end