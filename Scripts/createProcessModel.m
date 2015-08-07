% ------------------------------------------------------
% This function creates the process model for the extended Kalman smoother.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function dfdx = createProcessModel(n, dt, processModelType)

% Create partial process model
switch processModelType
    
    case 'constantJerk'
    dfdxPartial = [ ...
    1, dt, dt^2 / 2, dt^3 / 6; ...
    0, 1, dt, dt^2 / 2; ...
    0, 0, 1, dt; ...
    0, 0, 0, 1 ...
    ];
    nPartial = size(dfdxPartial, 1);
        
    case 'constantAcceleration'
    dfdxPartial = [ ...
    1, dt, dt^2 / 2; ...
    0, 1, dt; ...
    0, 0, 1 ...
    ];
    nPartial = size(dfdxPartial, 1);
        
    case 'constantVelocity'
    dfdxPartial = [ ...
    1, dt; ...
    0, 1 ...
    ];
    nPartial = size(dfdxPartial, 1);
        
    case 'constantPosition'
    dfdx = 1;
    return;
    
    otherwise
    fprintf('ERROR: Invalid process model type!\n');
    dfdx = [];
    return;
        
end

% Create full process model
dfdx = zeros(n);
for index = 1:nPartial:n
    
    % Add partial process models
    dfdx(index:(index + nPartial - 1), index:(index + nPartial - 1)) = dfdxPartial;

end

end