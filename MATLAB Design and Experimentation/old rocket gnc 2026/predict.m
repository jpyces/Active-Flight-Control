function [xPred, PPred] = predict(x, P, Q, n, dt, alpha, beta, lambda)
    % NEEDS DYNAMICS MODEL and x w/ Quaternion
    
    % Weight for each sigmaPoint - Means and Covariances  
    [Wm, Wc] = computeWeights(n, alpha, beta, lambda);
    
    % [n x 2n+1] matrix with columns as individual sigmaPoint
    sigmaPoints = generateSigmaPoints(x, P, n, lambda); 

    numSigmaPoints = (2 * n) + 1;
    x_propagated = zeros(n, numSigmaPoints); 

    for i = 1:numSigmaPoints
        % Propagate each sigmaPoint through the dynamics model into
        % propagated matrix
        x_propagated(:, i) = dynamics_model(sigmaPoints(:, i), dt);
    end

    % Weighted mean - matrix multiply for all the columns
    xPred = x_propagated * Wm;

    % Weighted covariance: sum of per-sigma-point outer products, plus
    % process noise Q - might be 0
    PPred = Q;
    for i = 1:numSigmaPoints
        diff = x_propagated(:, i) - xPred;
        PPred = PPred + Wc(i) * (diff * diff');
    end
end