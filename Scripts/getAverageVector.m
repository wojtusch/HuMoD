% ------------------------------------------------------
% This function returns the average vector of a target point from a
% reference point within the spefified frame range. It is required to have
% the corresponding motion data variable in workspace.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function vector = getAverageVector(referncePoint, referenceType, targetPoint, targetType, frameStart, frameEnd)
frameStart = ceil(abs(frameStart));
frameEnd = ceil(abs(frameEnd));
list = evalin('caller', 'who');
if ~isempty(find(ismember(list, 'motion'), 1))
    motion = evalin('caller', 'motion');
    if strcmp(referenceType, 'joint')
        referenceIndex = find(ismember(motion.jointLabels, referncePoint), 1);
    elseif strcmp(referenceType, 'marker')
        referenceIndex = find(ismember(motion.markerLabels, referncePoint), 1);
    else
        referenceIndex = find(ismember(motion.surfaceLabels, referncePoint), 1);
    end
    if strcmp(targetType, 'joint')
        targetIndex = find(ismember(motion.jointLabels, targetPoint), 1);
    elseif strcmp(targetType, 'marker')
        targetIndex = find(ismember(motion.markerLabels, targetPoint), 1);
    else
        targetIndex = find(ismember(motion.surfaceLabels, targetPoint), 1);
    end
    if ~isempty(referenceIndex) && ~isempty(targetIndex) && (frameStart < frameEnd) && (frameStart > 0) && (frameEnd <= motion.frames)
        vector = zeros((frameEnd - frameStart), 3);
        for frameIndex = 0:(frameEnd - frameStart - 1)
            if strcmp(referenceType, 'joint')
                referenceVector = [motion.jointX(referenceIndex, frameStart + frameIndex); motion.jointY(referenceIndex, frameStart + frameIndex); motion.jointZ(referenceIndex, frameStart + frameIndex)];
            elseif strcmp(referenceType, 'marker')
                referenceVector = [motion.markerX(referenceIndex, frameStart + frameIndex); motion.markerY(referenceIndex, frameStart + frameIndex); motion.markerZ(referenceIndex, frameStart + frameIndex)];
            else
                referenceVector = [motion.surfaceX(referenceIndex, frameStart + frameIndex); motion.surfaceY(referenceIndex, frameStart + frameIndex); motion.surfaceZ(referenceIndex, frameStart + frameIndex)];
            end
            if strcmp(targetType, 'joint')
                targetVector = [motion.jointX(targetIndex, frameStart + frameIndex); motion.jointY(targetIndex, frameStart + frameIndex); motion.jointZ(targetIndex, frameStart + frameIndex)];
            elseif strcmp(targetType, 'marker')
                targetVector = [motion.markerX(targetIndex, frameStart + frameIndex); motion.markerY(targetIndex, frameStart + frameIndex); motion.markerZ(targetIndex, frameStart + frameIndex)];
            else
                targetVector = [motion.surfaceX(targetIndex, frameStart + frameIndex); motion.surfaceY(targetIndex, frameStart + frameIndex); motion.surfaceZ(targetIndex, frameStart + frameIndex)];
            end
            vector(frameIndex + 1, :) = (targetVector - referenceVector)';
        end
        vector = median(vector)';
    else
        vector = [];
        fprintf('ERROR: Invalid joint or frame range!\n');
    end
else
    vector = [];
    fprintf('ERROR: No motion data variable found!\n');
end
end

