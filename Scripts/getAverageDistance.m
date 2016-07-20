% ------------------------------------------------------
% This function returns the average distance between two points within the
% specified frame range. It is required to have the corresponding motion
% data variable in workspace.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function distance = getAverageDistance(point1, type1, point2, type2, frameStart, frameEnd)

frameStart = ceil(abs(frameStart));
frameEnd = ceil(abs(frameEnd));
list = evalin('caller', 'who');
if ~isempty(find(ismember(list, 'motion'), 1))
    motion = evalin('caller', 'motion');
    if strcmp(type1, 'estimatedJoint')
        index1 = find(ismember(motion.jointLabels.estimated, point1), 1);
    elseif strcmp(type1, 'smoothedJoint')
        index1 = find(ismember(motion.jointLabels.smoothed, point1), 1);
    elseif strcmp(type1, 'marker')
        index1 = find(ismember(motion.markerLabels, point1), 1);
    else
        index1 = find(ismember(motion.surfaceLabels, point1), 1);
    end
    if strcmp(type2, 'estimatedJoint')
        index2 = find(ismember(motion.jointLabels.estimated, point2), 1);
    elseif strcmp(type2, 'smoothedJoint')
        index2 = find(ismember(motion.jointLabels.smoothed, point2), 1);
    elseif strcmp(type2, 'marker')
        index2 = find(ismember(motion.markerLabels, point2), 1);
    else
        index2 = find(ismember(motion.surfaceLabels, point2), 1);
    end
    if ~isempty(index1) && ~isempty(index2) && (frameStart < frameEnd) && (frameStart > 0) && (frameEnd <= motion.frames)
        distance = zeros((frameEnd - frameStart + 1), 1);
        for frameIndex = 0:(frameEnd - frameStart)
            if strcmp(type1, 'estimatedJoint')
                vector1 = [motion.jointX.estimated(index1, frameStart + frameIndex); motion.jointY.estimated(index1, frameStart + frameIndex); motion.jointZ.estimated(index1, frameStart + frameIndex)];
            elseif strcmp(type1, 'smoothedJoint')
                vector1 = [motion.jointX.smoothed(index1, frameStart + frameIndex); motion.jointY.smoothed(index1, frameStart + frameIndex); motion.jointZ.smoothed(index1, frameStart + frameIndex)];
            elseif strcmp(type1, 'marker')
                vector1 = [motion.markerX(index1, frameStart + frameIndex); motion.markerY(index1, frameStart + frameIndex); motion.markerZ(index1, frameStart + frameIndex)];
            else
                vector1 = [motion.surfaceX(index1, frameStart + frameIndex); motion.surfaceY(index1, frameStart + frameIndex); motion.surfaceZ(index1, frameStart + frameIndex)];
            end
            if strcmp(type2, 'estimatedJoint')
                vector2 = [motion.jointX.estimated(index2, frameStart + frameIndex); motion.jointY.estimated(index2, frameStart + frameIndex); motion.jointZ.estimated(index2, frameStart + frameIndex)];
            elseif strcmp(type2, 'smoothedJoint')
                vector2 = [motion.jointX.smoothed(index2, frameStart + frameIndex); motion.jointY.smoothed(index2, frameStart + frameIndex); motion.jointZ.smoothed(index2, frameStart + frameIndex)];
            elseif strcmp(type2, 'marker')
                vector2 = [motion.markerX(index2, frameStart + frameIndex); motion.markerY(index2, frameStart + frameIndex); motion.markerZ(index2, frameStart + frameIndex)];
            else
                vector2 = [motion.surfaceX(index2, frameStart + frameIndex); motion.surfaceY(index2, frameStart + frameIndex); motion.surfaceZ(index2, frameStart + frameIndex)];
            end
            distance(frameIndex + 1) = norm(vector2 - vector1);
        end
        distance = median(distance);
    else
        distance = [];
        fprintf('ERROR: Invalid joint or frame range!\n');
    end
else
    distance = [];
    fprintf('ERROR: No motion data variable found!\n');
end

end