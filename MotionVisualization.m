% ------------------------------------------------------
% This script visualizes joint and motion data.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

% Clean up workspace
clc;
clear all;
close all;

% Add functions to search path
addpath('Scripts');

% Set parameters
plotCenterOfPressure = 1;
groundXLimits = [-1000, 1000];
groundZLimits = [-1000, 1000];
savePath = [getPath, filesep, 'Motion', filesep];
datasets = {
    '1.1', ...
    '1.2', ...
    '1.3', ...
    '2.1', ...
    '2.2', ...
    '2.3', ...
    '3', ...
    '4', ...
    '5.1', ...
    '5.2', ...
    '6', ...
    '7', ...
    '8' ...
};
subjects = {
    'A', ...
    'B' ...
};

for subjectIndex = 1:length(subjects)
    for datasetIndex = 1:length(datasets)

        % Set parameters
        startFrame = 1;
        endFrame = inf;
        subject = subjects{subjectIndex};
        dataset = datasets{datasetIndex};
        fprintf('STATUS: Processing dataset %s %s.\n', subject, dataset);

        % Load data file
        file = getFile(subject, dataset);
        if file
            variables = load(file);
            if isfield(variables, 'motion') && isfield(variables, 'ground') && isfield(variables, 'force')
                motion = variables.motion;
                ground = variables.ground;
                force = variables.force;
                name = regexp(file, '[^/]*(?=\.[^.]+($|\?))', 'match');
                name = name{1};
            else
                fprintf('WARNING: No matching data found!\n');
                continue;
            end
        else
            fprintf('ERROR: No matching data file found!\n');
            return;
        end

        % Set end frame after loading file
        if (endFrame > motion.frames) || (endFrame == inf)
            endFrame = motion.frames;
        end
        if (startFrame > endFrame) || (startFrame < 1)
            startFrame = endFrame;
        end

        % Setup figure
        visualization = figure('Name', 'Motion', 'NumberTitle', 'off', 'Color', 'white', 'Position', [0, 0, 600, 450]);
        axis equal;
        axis off;
        hold on;
        view(-240, 20);

        % Setup video
        videoWriter = VideoWriter([savePath, subject, filesep, dataset, '.avi']);
        videoWriter.FrameRate = 500;
        videoWriter.Quality = 100;
        open(videoWriter);

        % Plot ground plane
        [groundX, groundZ] = meshgrid(groundXLimits, groundZLimits);
        groundY = zeros(2);
        patch3d(groundX([1, 2, 4, 3]), groundY([1, 2, 4, 3]), groundZ([1, 2, 4, 3]), 'k');
        alpha(0.1);

        % Plot dummy data and set axis limit mode to manual
        information = text(0, 0, 0, sprintf('Frame\t%d\nTime\t%.2f s', startFrame, (startFrame / motion.frameRate)), 'Units', 'normalized', 'HorizontalAlignment', 'left');
        markers = plot3d(motion.markerX(:, startFrame), motion.markerY(:, startFrame), motion.markerZ(:, startFrame), 'r.');
        axis tight;
        axis manual;
        delete(information);
        delete(markers);

        title([dataset, ' ', name, ' - HuMoD Database']);
        for currentFrame = startFrame:endFrame

            % Plot information text
            information = text(0, 0, 0, sprintf('Time %.2f s', (currentFrame / motion.frameRate)), 'Units', 'normalized');

            % Plot markers and joints
            markers = plot3d(motion.markerX(:, currentFrame), motion.markerY(:, currentFrame), motion.markerZ(:, currentFrame), 'r.');
            surface = plot3d(motion.surfaceX(:, currentFrame), motion.surfaceY(:, currentFrame), motion.surfaceZ(:, currentFrame), 'g.');
            joints = plot3d(motion.jointX(:, currentFrame), motion.jointY(:, currentFrame), motion.jointZ(:, currentFrame), 'b*');

            % Plot connection lines
            head = plot3d([motion.jointX(1, currentFrame), motion.surfaceX(1, currentFrame), motion.surfaceX(2, currentFrame), motion.jointX(1, currentFrame)], [motion.jointY(1, currentFrame), motion.surfaceY(1, currentFrame), motion.surfaceY(2, currentFrame), motion.jointY(1, currentFrame)], [motion.jointZ(1, currentFrame), motion.surfaceZ(1, currentFrame), motion.surfaceZ(2, currentFrame), motion.jointZ(1, currentFrame)], 'k');
            leftArm = plot3d([motion.jointX(1, currentFrame), motion.jointX(2, currentFrame), motion.jointX(4, currentFrame), motion.markerX(8, currentFrame)], [motion.jointY(1, currentFrame), motion.jointY(2, currentFrame), motion.jointY(4, currentFrame), motion.markerY(8, currentFrame)], [motion.jointZ(1, currentFrame), motion.jointZ(2, currentFrame), motion.jointZ(4, currentFrame), motion.markerZ(8, currentFrame)], 'k');
            rightArm = plot3d([motion.jointX(1, currentFrame), motion.jointX(3, currentFrame), motion.jointX(5, currentFrame), motion.markerX(9, currentFrame)], [motion.jointY(1, currentFrame), motion.jointY(3, currentFrame), motion.jointY(5, currentFrame), motion.markerY(9, currentFrame)], [motion.jointZ(1, currentFrame), motion.jointZ(3, currentFrame), motion.jointZ(5, currentFrame), motion.markerZ(9, currentFrame)], 'k');
            spinal = plot3d([motion.jointX(7, currentFrame), motion.jointX(6, currentFrame), motion.jointX(1, currentFrame)], [motion.jointY(7, currentFrame), motion.jointY(6, currentFrame), motion.jointY(1, currentFrame)], [motion.jointZ(7, currentFrame), motion.jointZ(6, currentFrame), motion.jointZ(1, currentFrame)], 'k');
            pelvis = plot3d([motion.jointX(7, currentFrame), motion.jointX(8, currentFrame), motion.jointX(9, currentFrame), motion.jointX(7, currentFrame)], [motion.jointY(7, currentFrame), motion.jointY(8, currentFrame), motion.jointY(9, currentFrame), motion.jointY(7, currentFrame)], [motion.jointZ(7, currentFrame), motion.jointZ(8, currentFrame), motion.jointZ(9, currentFrame), motion.jointZ(7, currentFrame)], 'k');
            leftLeg = plot3d([motion.jointX(14, currentFrame), motion.jointX(12, currentFrame), motion.jointX(10, currentFrame), motion.jointX(8, currentFrame)], [motion.jointY(14, currentFrame), motion.jointY(12, currentFrame), motion.jointY(10, currentFrame), motion.jointY(8, currentFrame)], [motion.jointZ(14, currentFrame), motion.jointZ(12, currentFrame), motion.jointZ(10, currentFrame), motion.jointZ(8, currentFrame)], 'k');
            rightLeg = plot3d([motion.jointX(15, currentFrame), motion.jointX(13, currentFrame), motion.jointX(11, currentFrame), motion.jointX(9, currentFrame)], [motion.jointY(15, currentFrame), motion.jointY(13, currentFrame), motion.jointY(11, currentFrame), motion.jointY(9, currentFrame)], [motion.jointZ(15, currentFrame), motion.jointZ(13, currentFrame), motion.jointZ(11, currentFrame), motion.jointZ(9, currentFrame)], 'k');

            % Plot center of pressure
            if plotCenterOfPressure
                if ~strcmp(dataset, '6') && ~strcmp(dataset, '7') && ~strcmp(dataset, '8')
                    leftCOP = plot3d(mean([force.centerOfPressureX_L(2 * currentFrame - 1), force.centerOfPressureX_L(2 * currentFrame)]), mean([force.centerOfPressureY_L(2 * currentFrame - 1), force.centerOfPressureY_L(2 * currentFrame)]), mean([force.centerOfPressureZ_L(2 * currentFrame - 1), force.centerOfPressureZ_L(2 * currentFrame)]), 'r^');
                    rightCOP = plot3d(mean([force.centerOfPressureX_R(2 * currentFrame - 1), force.centerOfPressureX_R(2 * currentFrame)]), mean([force.centerOfPressureY_R(2 * currentFrame - 1), force.centerOfPressureY_R(2 * currentFrame)]), mean([force.centerOfPressureZ_R(2 * currentFrame - 1), force.centerOfPressureZ_R(2 * currentFrame)]), 'b^');
                else
                    totalCOP = plot3d(mean([force.centerOfPressureX(2 * currentFrame - 1), force.centerOfPressureX(2 * currentFrame)]), mean([force.centerOfPressureY(2 * currentFrame - 1), force.centerOfPressureY(2 * currentFrame)]), mean([force.centerOfPressureZ(2 * currentFrame - 1), force.centerOfPressureZ(2 * currentFrame)]), 'k^');
                end
            end
            
            % Update figure
            drawnow;

            % Save frame
            writeVideo(videoWriter, getframe(visualization));

            % Delete visualization
            if currentFrame < endFrame
                delete(information);
                delete(markers);
                delete(surface);
                delete(joints);
                delete(head);
                delete(leftArm);
                delete(rightArm);
                delete(spinal);
                delete(pelvis);
                delete(leftLeg);
                delete(rightLeg);
                if plotCenterOfPressure
                    if ~strcmp(dataset, '6') && ~strcmp(dataset, '7') && ~strcmp(dataset, '8')
                        delete(leftCOP)
                        delete(rightCOP)
                    else
                        delete(totalCOP)
                    end
                end
            end

        end
        close(videoWriter);
        close all;

    end
end
