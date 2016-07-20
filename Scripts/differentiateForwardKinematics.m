% ------------------------------------------------------
% This function differentiates the forward kinematics algorithm by calling
% the corresponding function of the external HuMoD library.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function dhdx = differentiateForwardKinematics(x, n, m, libraryName, processModelType)

% Set parameters
dhdx = zeros(m, n);
switch processModelType
    
    case 'constantPosition'
    factor = 1;
        
    case 'constantVelocity'
    factor = 2;
        
    case 'constantAcceleration'
    factor = 3;
    
    case 'constantJerk'
    factor = 4;

    otherwise
    fprintf('ERROR: Unknown process model!\n');
    ruturn;
        
end

% Set joint positions
jointPositions = zeros((n / factor), 1);
for index = 1:(n / factor)
    
    jointPositions(index) = x((index - 1) * factor + 1);
    
end

% Initialize element derivatives
elementJacobian = zeros((m * n / factor), 1);
elementJacobianPointer = libpointer('doublePtr', elementJacobian);

% Apply forward kinematics and read element Jacobian
calllib(libraryName, 'applyForwardKinematics', ...
    jointPositions ...
);
calllib(libraryName, 'getForwardKinematicsJacobian', ...
    elementJacobianPointer ...
);

% Create Jacobian matrix
elementJacobian = elementJacobianPointer.value;
for index = 1:(n / factor)
    
    dhdx(:, (index - 1) * factor + 1) = elementJacobian(((index - 1) * m + 1):(index * m));

end

end