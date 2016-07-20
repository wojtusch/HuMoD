% ------------------------------------------------------
% This function applies the forward kinematics algorithm by calling the
% corresponding function of the external HuMoD library.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function z = applyForwardKinematics(x, n, m, libraryName, processModelType)

% Set parameters
z = zeros(m, 1);
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

% Initialize element positions
elementPositionsX = zeros((m / 3), 1);
elementPositionsXPointer = libpointer('doublePtr', elementPositionsX);
elementPositionsY = zeros((m / 3), 1);
elementPositionsYPointer = libpointer('doublePtr', elementPositionsY);
elementPositionsZ = zeros((m / 3), 1);
elementPositionsZPointer = libpointer('doublePtr', elementPositionsZ);

% Apply forward kinematics and read element positions
calllib(libraryName, 'applyForwardKinematics', ...
    jointPositions ...
);
calllib(libraryName, 'getForwardKinematicsPositions', ...
    elementPositionsXPointer, ...
    elementPositionsYPointer, ...
    elementPositionsZPointer ...
);

% Create measurement vector
elementPositionsX = elementPositionsXPointer.value;
elementPositionsY = elementPositionsYPointer.value;
elementPositionsZ = elementPositionsZPointer.value;
for index = 1:(m / 3)
    
    z((index - 1) * 3 + 1) = elementPositionsX(index);
    z((index - 1) * 3 + 2) = elementPositionsY(index);
    z((index - 1) * 3 + 3) = elementPositionsZ(index);

end

end
