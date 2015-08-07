% ------------------------------------------------------
% This function implements an extended Kalman filter according to
% [Yu2004]. 
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------
% x:    n x 1 state vector
% Z:    T x n matrix of state vectors
% z:    m x 1 measurement vector
% Z:    T x m matrix of measurement vectors
% f:    Function handle of the nonlinear system function x_k = f(x_(k - 1))
% h:    Function handle of the nonlinear measurement function with z_k = h(x_k)
% dfdx: Function handle of the Jacobian matrix of partitial derivatives
% of f with resprect to x
% dfdw: Function handle of the Jacobian matrix of partitial derivatives
% of f with resprect to w
% dhdx: Function handle of the Jacobian matrix of partitial derivatives
% of h with resprect to x
% dhdv: Function handle of the Jacobian matrix of partitial derivatives
% of h with resprect to v
% Q:    n x n process covariance
% R:    m x m measurement covariance
% P:    n x n error covariance matrix
% x0:   Initial value of the state vector x
% P0:   Initial value of the error covariance matrix P

function [X_posterior, P_posterior, X_prior, P_prior] = extendedKalmanFilter(Z, f, h, dfdx, dfdw, dhdx, dhdv, Q, R, x0, P0)

    % Initialize variables
    T = size(Z, 1);
    n = size(x0, 1);
    X_prior = zeros(T, n);
    X_posterior = zeros(T, n);
    P_prior = zeros(n, n, T);
    P_posterior = zeros(n, n, T);

    % Apply extended Kalman filter equations 
    statusCounter = 0;
    for k = 1:T

        % Apply time update
        if k == 1

            % Equation (15) in [Yu2004]
            X_prior(k, :) = f(x0);
            A = dfdx(x0);
            W = dfdw(x0);
            
            % Equation (12) in [Yu2004]
            P_prior(:, :, k) = A * P0 * A' + W * Q * W';

        else
            
            % Equation (15) in [Yu2004]
            X_prior(k, :) = f(X_posterior((k - 1), :)');
            A = dfdx(X_posterior((k - 1), :)');
            W = dfdw(X_posterior((k - 1), :)');
            
            % Equation (12) in [Yu2004]
            P_prior(:, :, k) = A * P_posterior(:, :, (k - 1)) * A' + W * Q * W';

        end

        % Apply measurement update
        H = dhdx(X_prior(k, :)');
        V = dhdv(X_prior(k, :)');
        
        % Equation (11) in [Yu2004] using the pseudo-inverse to deal with
        % close to singular or bady scaled matrices
        %K = P_prior(:, :, k) * H' / (V * R * V' + H * P_prior(:, :, k) * H');
        K = P_prior(:, :, k) * H' * pinv(V * R * V' + H * P_prior(:, :, k) * H');
        
        % Equation (14) in [Yu2004]
        X_posterior(k, :) = X_prior(k, :) + (K * (Z(k, :)' - h(X_prior(k, :)')))';
        
        % Equation (10) in [Yu2004]
        P_posterior(:, :, k) = P_prior(:, :, k) - K * H * P_prior(:, :, k);
        
        % Print status
        statusCounter = statusCounter + 1;
        if statusCounter >= 100
            fprintf('STATUS: %.1f %%\n', (k - 1) / (T - 1) * 100);
            statusCounter = 0;
        end

    end

end