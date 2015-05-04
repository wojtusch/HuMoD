% ------------------------------------------------------
% This function returns the x, y and z coordinates of the given joint at
% the spefified frame. It is required to have the corresponding motion data
% variable in workspace.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function coordinates = getJoint(joint, frames)
frames = ceil(abs(frames));
list = evalin('caller', 'who');
if ~isempty(find(ismember(list, 'motion'), 1))
    coordinates = zeros(3, length(frames));
    motion = evalin('caller', 'motion');
    index = find(ismember(motion.jointLabels, joint), 1);
    if ~isempty(index) && (max(frames) <= motion.frames)
        coordinates(1, :) = motion.jointX(index, frames);
        coordinates(2, :) = motion.jointY(index, frames);
        coordinates(3, :) = motion.jointZ(index, frames);
    else
        coordinates = [];
        fprintf('ERROR: Invalid joint or frame!\n');
    end
else
    coordinates = [];
    fprintf('ERROR: No motion data variable found!\n');
end
end

