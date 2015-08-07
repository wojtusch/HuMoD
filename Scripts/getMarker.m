% ------------------------------------------------------
% This function returns the x, y and z coordinates of the given marker at
% the specified frame. It is required to have the corresponding motion data
% variable in workspace.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function coordinates = getMarker(marker, type, frames)
frames = ceil(abs(frames));
list = evalin('caller', 'who');
if ~isempty(find(ismember(list, 'motion'), 1))
    coordinates = zeros(3, length(frames));
    motion = evalin('caller', 'motion');
    if strcmp(type, 'marker')
        index = find(ismember(motion.markerLabels, marker), 1);
    else
        index = find(ismember(motion.surfaceLabels, marker), 1);
    end
    if ~isempty(index) && (max(frames) <= motion.frames)
        if strcmp(type, 'marker') 
            coordinates(1, :) = motion.markerX(index, frames);
            coordinates(2, :) = motion.markerY(index, frames);
            coordinates(3, :) = motion.markerZ(index, frames);
        else
            coordinates(1, :) = motion.surfaceX(index, frames);
            coordinates(2, :) = motion.surfaceY(index, frames);
            coordinates(3, :) = motion.surfaceZ(index, frames);
        end
    else
        coordinates = [];
        fprintf('ERROR: Invalid marker or frame!\n');
    end
else
    coordinates = [];
    fprintf('ERROR: No motion data variable found!\n');
end
end

