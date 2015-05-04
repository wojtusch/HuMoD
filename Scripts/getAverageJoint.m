% ------------------------------------------------------
% This function returns the average x, y and z coordinates of the given
% joint within the spefified frame range. It is required to have the
% corresponding motion data variable in workspace.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function coordinates = getAverageJoint(joint, frameStart, frameEnd)
frameStart = ceil(abs(frameStart));
frameEnd = ceil(abs(frameEnd));
list = evalin('caller', 'who');
if ~isempty(find(ismember(list, 'motion'), 1))
    coordinates = zeros(3, 1);
    motion = evalin('caller', 'motion');
    index = find(ismember(motion.jointLabels, joint), 1);
    if ~isempty(index) && (frameStart < frameEnd) && (frameStart > 0) && (frameEnd <= motion.frames)
        coordinates(1) = sum(motion.jointX(index, frameStart:frameEnd)) / (frameEnd - frameStart);
        coordinates(2) = sum(motion.jointY(index, frameStart:frameEnd)) / (frameEnd - frameStart);
        coordinates(3) = sum(motion.jointZ(index, frameStart:frameEnd)) / (frameEnd - frameStart);
    else
        coordinates = [];
        fprintf('ERROR: Invalid joint or frame range!\n');
    end
else
    coordinates = [];
    fprintf('ERROR: No motion data variable found!\n');
end
end

