function correction (xhat, P, Q, n, dt, alpha, beta, lambda)
    % incomplete
    % Need SENSOR MODELS
    [Wm, Wc] = computeWeights(n, alpha, beta, lambda);
    
    sigmaPoints = generateSigmaPoints(xhat, P, n, lambda);

    numSigmaPoints = (2 * n) + 1;
    xhat_propagated = zeros(n, numSigmaPoints);

    for i = 1:numSigmaPoints
        xhat_propagated(:, i) = sensor_model(sigmaPoints(:, i), dt);
    end
end