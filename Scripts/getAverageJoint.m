% ------------------------------------------------------
% This function returns the average x, y and z coordinates of the given
% joint within the specified frame range. It is required to have the
% corresponding motion data variable in workspace.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function coordinates = getAverageJoint(joint, type, frameStart, frameEnd)
frameStart = ceil(abs(frameStart));
frameEnd = ceil(abs(frameEnd));
list = evalin('caller', 'who');
if ~isempty(find(ismember(list, 'motion'), 1))
    coordinates = zeros(3, 1);
    motion = evalin('caller', 'motion');
    if strcmp(type, 'estimatedJoint')
        index = find(ismember(motion.jointLabels.estimated, joint), 1);
    else
        index = find(ismember(motion.jointLabels.smoothed, joint), 1);
    end
    if ~isempty(index) && strcmp(type, 'estimatedJoint') && (frameStart < frameEnd) && (frameStart > 0) && (frameEnd <= motion.frames)
        coordinates(1) = sum(motion.jointX.estimated(index, frameStart:frameEnd)) / (frameEnd - frameStart + 1);
        coordinates(2) = sum(motion.jointY.estimated(index, frameStart:frameEnd)) / (frameEnd - frameStart + 1);
        coordinates(3) = sum(motion.jointZ.estimated(index, frameStart:frameEnd)) / (frameEnd - frameStart + 1);
    elseif ~isempty(index) && (frameStart < frameEnd) && (frameStart > 0) && (frameEnd <= motion.frames)
        coordinates(1) = sum(motion.jointX.smoothed(index, frameStart:frameEnd)) / (frameEnd - frameStart + 1);
        coordinates(2) = sum(motion.jointY.smoothed(index, frameStart:frameEnd)) / (frameEnd - frameStart + 1);
        coordinates(3) = sum(motion.jointZ.smoothed(index, frameStart:frameEnd)) / (frameEnd - frameStart + 1);
    else
        coordinates = [];
        fprintf('ERROR: Invalid joint or frame range!\n');
    end
else
    coordinates = [];
    fprintf('ERROR: No motion data variable found!\n');
end
end