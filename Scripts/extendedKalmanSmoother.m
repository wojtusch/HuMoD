% ------------------------------------------------------
% This function implements an extended Kalman smoother according to
% [Yu2004] and requires the matching implementation of an extended Kalman
% filter. 
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------
% x:    n x 1 state vector
% X:    T x n matrix of state vectors
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

function X = extendedKalmanSmoother(Z, f, h, dfdx, dfdw, dhdx, dhdv, Q, R, x0, P0)
    
    % Apply extended Kalman filter
    fprintf('STATUS: Applying extended Kalman filter\n');
    [X_posterior, P_posterior, ~, P_prior] = extendedKalmanFilter(Z, f, h, dfdx, dfdw, dhdx, dhdv, Q, R, x0, P0);
    
    % Initialize variables
    T = size(Z, 1);
    n = size(x0, 1);
    X = zeros(T, n);
    
    % Apply extended Kalman smoother equation
    fprintf('STATUS: Applying extended Kalman smoother\n');
    X(T, :) = X_posterior(T, :);
    statusCounter = 0;
    for k = (T - 1):-1:1

        A = dfdx(X_posterior(k, :)');

        % Equation (19) in [Yu2004] using the pseudo-inverse to deal with
        % close to singular or bady scaled matrices
        %J = P_posterior(:, :, k) * A' / P_prior(:, :, k + 1);
        J = P_posterior(:, :, k) * A' * pinv(P_prior(:, :, k + 1));

        % Equation (25) in [Yu2004]
        X(k, :) = (X_posterior(k, :)' + J * (X(k + 1,:)' - f(X_posterior(k, :)')))';
        
        % Equations (21), (22) or (23) and (24) in [Yu2004] are not
        % implemented.
        
        % Print status
        statusCounter = statusCounter + 1;
        if statusCounter >= 100
            fprintf('STATUS: %.1f %%\n', abs((k - (T - 1)) / (1 - (T - 1)) * 100));
            statusCounter = 0;
        end

    end

end