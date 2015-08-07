% ------------------------------------------------------
% This function creates the process variance for the extended Kalman
% smoother.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function Q = createProcessVariance(n, dt, kalmanProcessVariance, processModelType)

% Create partial process variance
switch processModelType
    
    case 'constantJerk'
    G = [dt^4 / 24, dt^3 / 6, dt^2 / 2, dt];
        
    case 'constantAcceleration'
    G = [dt^3 / 6, dt^2 / 2, dt];
        
    case 'constantVelocity'
    G = [dt^2 / 2, dt];
        
    case 'constantPosition'
    G = dt;
    
    otherwise
    fprintf('ERROR: Invalid process model type!\n');
    Q = [];
    return;
        
end
QPartial = kalmanProcessVariance^2 * G' * G;
nPartial = size(QPartial, 1);

% Create full process model
Q = zeros(n);
for index = 1:nPartial:n
    
    % Add partial process models
    Q(index:(index + nPartial - 1), index:(index + nPartial - 1)) = QPartial;

end

end