% ------------------------------------------------------
% This function returns the average x, y and z coordinates of the given
% marker within the specified frame range. It is required to have the
% corresponding motion data variable in workspace.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function coordinates = getAverageMarker(marker, type, frameStart, frameEnd)
frameStart = ceil(abs(frameStart));
frameEnd = ceil(abs(frameEnd));
list = evalin('caller', 'who');
if ~isempty(find(ismember(list, 'motion'), 1))
    coordinates = zeros(3, 1);
    motion = evalin('caller', 'motion');
    if strcmp(type, 'marker')
        index = find(ismember(motion.markerLabels, marker), 1);
    else
        index = find(ismember(motion.surfaceLabels, marker), 1);
    end
    if ~isempty(index) && (frameStart < frameEnd) && (frameStart > 0) && (frameEnd <= motion.frames)
        if strcmp(type, 'marker')
            coordinates(1) = sum(motion.markerX(index, frameStart:frameEnd)) / (frameEnd - frameStart + 1);
            coordinates(2) = sum(motion.markerY(index, frameStart:frameEnd)) / (frameEnd - frameStart + 1);
            coordinates(3) = sum(motion.markerZ(index, frameStart:frameEnd)) / (frameEnd - frameStart + 1);
        else
            coordinates(1) = sum(motion.surfaceX(index, frameStart:frameEnd)) / (frameEnd - frameStart + 1);
            coordinates(2) = sum(motion.surfaceY(index, frameStart:frameEnd)) / (frameEnd - frameStart + 1);
            coordinates(3) = sum(motion.surfaceZ(index, frameStart:frameEnd)) / (frameEnd - frameStart + 1);
        end
    else
        coordinates = [];
        fprintf('ERROR: Invalid marker or frame range!\n');
    end
else
    coordinates = [];
    fprintf('ERROR: No motion data variable found!\n');
end
end

