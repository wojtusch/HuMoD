% ------------------------------------------------------
% This function returns the x, y and z coordinates of the given sensor. It
% is required to have the corresponding ground data variable in workspace.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function coordinates = getSensor(sensor)
list = evalin('caller', 'who');
if ~isempty(find(ismember(list, 'ground'), 1))
    coordinates = zeros(3, 1);
    ground = evalin('caller', 'ground');
    index = find(ismember(ground.sensorLabels, sensor), 1);
    if ~isempty(index)
        coordinates(1) = ground.sensorX(index);
        coordinates(2) = ground.sensorY(index);
        coordinates(3) = ground.sensorZ(index);
    else
        coordinates = [];
        fprintf('ERROR: Invalid sensor!\n');
    end
else
    coordinates = [];
    fprintf('ERROR: No ground data variable found!\n');
end
end

