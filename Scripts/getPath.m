% ------------------------------------------------------
% This function returns the global path to the datasets.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

% Please modify the global path in getPath.m and the local paths in getFile.m.

function globalPath = getPath
globalPath = pwd;
if isempty(globalPath) || ~exist(globalPath, 'dir')
    fprintf('ERROR: Path not found! Please modify the global path in getPath.m and the local paths in getFile.m.\n');
end
end
