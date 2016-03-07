% ------------------------------------------------------
% This script visualizes the smoothed joint trajectories.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2016
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

% Clean up workspace
clc;
clear all;
close all;

% Add functions to search path
addpath('Scripts');

% Load constants
setForwardKinematicsConstants;

% Set parameters
startFrameNumber = 1;
endFrameNumber = inf;
groundXLimits = [-1000, 1000];
groundZLimits = [-1000, 1000];
savePath = [getPath, filesep, 'Trajectory', filesep];
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
    
    % Load HuMoD library
    libraryPath = [getPath, filesep, 'Library'];
    libraryName = 'libHuMoD';
    loadlibrary([libraryPath, filesep, 'build', filesep, libraryName, '.so'], [libraryPath, filesep, 'model.h']);

    % Set parameters
    subject = subjects{subjectIndex};

    % Load model parameters
    parametersFile = [getPath, filesep, subject, filesep, 'Parameters.mat'];
    if exist(parametersFile, 'file')
        parameters = load(parametersFile);
    else
        fprintf('ERROR: No matching parameters file found!\n');
        return;
    end

    % Set model parameters and create model
    calllib(libraryName, 'createModel', ...
        subject, ...
        0, ...
        createParameterVector('head', parameters), ...
        createParameterVector('thorax', parameters), ...
        createParameterVector('abdomen', parameters), ...
        createParameterVector('pelvis', parameters), ...
        createParameterVector('upperArm_L', parameters), ...
        createParameterVector('upperArm_R', parameters), ...
        createParameterVector('lowerArm_L', parameters), ...
        createParameterVector('lowerArm_R', parameters), ...
        createParameterVector('thigh_L', parameters), ...
        createParameterVector('thigh_R', parameters), ...
        createParameterVector('shank_L', parameters), ...
        createParameterVector('shank_R', parameters), ...
        createParameterVector('foot_L', parameters), ...
        createParameterVector('foot_R', parameters) ...
    );
    
    for datasetIndex = 1:length(datasets)

        % Set parameters
        startFrame = startFrameNumber;
        endFrame = endFrameNumber;
        dataset = datasets{datasetIndex};
        fprintf('STATUS: Processing dataset %s %s.\n', subject, dataset);

        % Load data file
        file = getFile(subject, dataset);
        if file
            variables = load(file);
            if isfield(variables, 'motion') && isfield(variables, 'ground')
                motion = variables.motion;
                ground = variables.ground;
                if ~isfield(motion, 'trajectory')
                    fprintf('WARNING: No joint trajectory data found!\n');
                    continue;
                end
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
        visualization = figure('Name', 'Trajectory', 'NumberTitle', 'off', 'Color', 'white', 'Position', [0, 0, 600, 450]);
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

            % Apply forward kinematics
            jointPositions = motion.trajectory.q(:, currentFrame);
            elementPositionsX = zeros(ELEMENT_Total, 1);
            elementPositionsXPointer = libpointer('doublePtr', elementPositionsX);
            elementPositionsY = zeros(ELEMENT_Total, 1);
            elementPositionsYPointer = libpointer('doublePtr', elementPositionsY);
            elementPositionsZ = zeros(ELEMENT_Total, 1);
            elementPositionsZPointer = libpointer('doublePtr', elementPositionsZ);
            calllib(libraryName, 'applyForwardKinematics', ...
                jointPositions ...
            );
            calllib(libraryName, 'getForwardKinematicsPositions', ...
                elementPositionsXPointer, ...
                elementPositionsYPointer, ...
                elementPositionsZPointer ...
            );
            elementPositionsX = elementPositionsXPointer.value;
            elementPositionsY = elementPositionsYPointer.value;
            elementPositionsZ = elementPositionsZPointer.value;
            
            % Plot information text
            information = text(0, 0, 0, sprintf('Time %.2f s', (currentFrame / motion.frameRate)), 'Units', 'normalized');

            % Plot measured and smoothed markers
            markers = plot3d(motion.markerX(:, currentFrame), motion.markerY(:, currentFrame), motion.markerZ(:, currentFrame), 'r.');
            surface = plot3d(motion.surfaceX(:, currentFrame), motion.surfaceY(:, currentFrame), motion.surfaceZ(:, currentFrame), 'g.');
            elements = plot3d(elementPositionsX, elementPositionsY, elementPositionsZ, 'c.');
            
            % Plot estimated connection lines
            estimatedHead = plot3d([motion.jointX.estimated(1, currentFrame), motion.surfaceX(1, currentFrame), motion.surfaceX(2, currentFrame), motion.jointX.estimated(1, currentFrame)], [motion.jointY.estimated(1, currentFrame), motion.surfaceY(1, currentFrame), motion.surfaceY(2, currentFrame), motion.jointY.estimated(1, currentFrame)], [motion.jointZ.estimated(1, currentFrame), motion.surfaceZ(1, currentFrame), motion.surfaceZ(2, currentFrame), motion.jointZ.estimated(1, currentFrame)], 'b');
            estimatedLeftArm = plot3d([motion.jointX.estimated(1, currentFrame), motion.jointX.estimated(2, currentFrame), motion.jointX.estimated(4, currentFrame), motion.markerX(8, currentFrame)], [motion.jointY.estimated(1, currentFrame), motion.jointY.estimated(2, currentFrame), motion.jointY.estimated(4, currentFrame), motion.markerY(8, currentFrame)], [motion.jointZ.estimated(1, currentFrame), motion.jointZ.estimated(2, currentFrame), motion.jointZ.estimated(4, currentFrame), motion.markerZ(8, currentFrame)], 'b');
            estimatedRightArm = plot3d([motion.jointX.estimated(1, currentFrame), motion.jointX.estimated(3, currentFrame), motion.jointX.estimated(5, currentFrame), motion.markerX(9, currentFrame)], [motion.jointY.estimated(1, currentFrame), motion.jointY.estimated(3, currentFrame), motion.jointY.estimated(5, currentFrame), motion.markerY(9, currentFrame)], [motion.jointZ.estimated(1, currentFrame), motion.jointZ.estimated(3, currentFrame), motion.jointZ.estimated(5, currentFrame), motion.markerZ(9, currentFrame)], 'b');
            estimatedThorax = plot3d([motion.jointX.estimated(6, currentFrame), motion.jointX.estimated(1, currentFrame)], [motion.jointY.estimated(6, currentFrame), motion.jointY.estimated(1, currentFrame)], [motion.jointZ.estimated(6, currentFrame), motion.jointZ.estimated(1, currentFrame)], 'b');
            estimatedAbdomen = plot3d([motion.jointX.estimated(7, currentFrame), motion.jointX.estimated(6, currentFrame)], [motion.jointY.estimated(7, currentFrame), motion.jointY.estimated(6, currentFrame)], [motion.jointZ.estimated(7, currentFrame), motion.jointZ.estimated(6, currentFrame)], 'b');
            estimatedPelvis = plot3d([motion.jointX.estimated(7, currentFrame), motion.jointX.estimated(8, currentFrame), motion.jointX.estimated(9, currentFrame), motion.jointX.estimated(7, currentFrame)], [motion.jointY.estimated(7, currentFrame), motion.jointY.estimated(8, currentFrame), motion.jointY.estimated(9, currentFrame), motion.jointY.estimated(7, currentFrame)], [motion.jointZ.estimated(7, currentFrame), motion.jointZ.estimated(8, currentFrame), motion.jointZ.estimated(9, currentFrame), motion.jointZ.estimated(7, currentFrame)], 'b');
            estimatedLeftLeg = plot3d([motion.surfaceX(29, currentFrame), motion.jointX.estimated(12, currentFrame), motion.jointX.estimated(10, currentFrame), motion.jointX.estimated(8, currentFrame)], [motion.surfaceY(29, currentFrame), motion.jointY.estimated(12, currentFrame), motion.jointY.estimated(10, currentFrame), motion.jointY.estimated(8, currentFrame)], [motion.surfaceZ(29, currentFrame), motion.jointZ.estimated(12, currentFrame), motion.jointZ.estimated(10, currentFrame), motion.jointZ.estimated(8, currentFrame)], 'b');
            estimatedRightLeg = plot3d([motion.surfaceX(30, currentFrame), motion.jointX.estimated(13, currentFrame), motion.jointX.estimated(11, currentFrame), motion.jointX.estimated(9, currentFrame)], [motion.surfaceY(30, currentFrame), motion.jointY.estimated(13, currentFrame), motion.jointY.estimated(11, currentFrame), motion.jointY.estimated(9, currentFrame)], [motion.surfaceZ(30, currentFrame), motion.jointZ.estimated(13, currentFrame), motion.jointZ.estimated(11, currentFrame), motion.jointZ.estimated(9, currentFrame)], 'b');
            
            % Plot smoothed connection lines
            smoothedHead = plot3d([motion.jointX.smoothed(1, currentFrame), elementPositionsX(ELEMENT_TRA_L), elementPositionsX(ELEMENT_TRA_R), motion.jointX.smoothed(1, currentFrame)], [motion.jointY.smoothed(1, currentFrame), elementPositionsY(ELEMENT_TRA_L), elementPositionsY(ELEMENT_TRA_R), motion.jointY.smoothed(1, currentFrame)], [motion.jointZ.smoothed(1, currentFrame), elementPositionsZ(ELEMENT_TRA_L), elementPositionsZ(ELEMENT_TRA_R), motion.jointZ.smoothed(1, currentFrame)], 'c');
            smoothedLeftArm = plot3d([motion.jointX.smoothed(1, currentFrame), motion.jointX.smoothed(2, currentFrame), motion.jointX.smoothed(4, currentFrame), elementPositionsX(ELEMENT_WRI_L)], [motion.jointY.smoothed(1, currentFrame), motion.jointY.smoothed(2, currentFrame), motion.jointY.smoothed(4, currentFrame), elementPositionsY(ELEMENT_WRI_L)], [motion.jointZ.smoothed(1, currentFrame), motion.jointZ.smoothed(2, currentFrame), motion.jointZ.smoothed(4, currentFrame), elementPositionsZ(ELEMENT_WRI_L)], 'c');
            smoothedRightArm = plot3d([motion.jointX.smoothed(1, currentFrame), motion.jointX.smoothed(3, currentFrame), motion.jointX.smoothed(5, currentFrame), elementPositionsX(ELEMENT_WRI_R)], [motion.jointY.smoothed(1, currentFrame), motion.jointY.smoothed(3, currentFrame), motion.jointY.smoothed(5, currentFrame), elementPositionsY(ELEMENT_WRI_R)], [motion.jointZ.smoothed(1, currentFrame), motion.jointZ.smoothed(3, currentFrame), motion.jointZ.smoothed(5, currentFrame), elementPositionsZ(ELEMENT_WRI_R)], 'c');
            smoothedThorax = plot3d([motion.jointX.smoothed(6, currentFrame), motion.jointX.smoothed(1, currentFrame)], [motion.jointY.smoothed(6, currentFrame), motion.jointY.smoothed(1, currentFrame)], [motion.jointZ.smoothed(6, currentFrame), motion.jointZ.smoothed(1, currentFrame)], 'c');
            smoothedAbdomen = plot3d([motion.jointX.smoothed(7, currentFrame), motion.jointX.smoothed(6, currentFrame)], [motion.jointY.smoothed(7, currentFrame), motion.jointY.smoothed(6, currentFrame)], [motion.jointZ.smoothed(7, currentFrame), motion.jointZ.smoothed(6, currentFrame)], 'c');
            smoothedPelvis = plot3d([motion.jointX.smoothed(7, currentFrame), motion.jointX.smoothed(8, currentFrame), motion.jointX.smoothed(9, currentFrame), motion.jointX.smoothed(7, currentFrame)], [motion.jointY.smoothed(7, currentFrame), motion.jointY.smoothed(8, currentFrame), motion.jointY.smoothed(9, currentFrame), motion.jointY.smoothed(7, currentFrame)], [motion.jointZ.smoothed(7, currentFrame), motion.jointZ.smoothed(8, currentFrame), motion.jointZ.smoothed(9, currentFrame), motion.jointZ.smoothed(7, currentFrame)], 'c');
            smoothedLeftLeg = plot3d([motion.jointX.smoothed(8, currentFrame), motion.jointX.smoothed(10, currentFrame), motion.jointX.smoothed(12, currentFrame), elementPositionsX(ELEMENT_MT2_L)], [motion.jointY.smoothed(8, currentFrame), motion.jointY.smoothed(10, currentFrame), motion.jointY.smoothed(12, currentFrame), elementPositionsY(ELEMENT_MT2_L)], [motion.jointZ.smoothed(8, currentFrame), motion.jointZ.smoothed(10, currentFrame), motion.jointZ.smoothed(12, currentFrame), elementPositionsZ(ELEMENT_MT2_L)], 'c');
            smoothedRightLeg = plot3d([motion.jointX.smoothed(9, currentFrame), motion.jointX.smoothed(11, currentFrame), motion.jointX.smoothed(13, currentFrame), elementPositionsX(ELEMENT_MT2_R)], [motion.jointY.smoothed(9, currentFrame), motion.jointY.smoothed(11, currentFrame), motion.jointY.smoothed(13, currentFrame), elementPositionsY(ELEMENT_MT2_R)], [motion.jointZ.smoothed(9, currentFrame), motion.jointZ.smoothed(11, currentFrame), motion.jointZ.smoothed(13, currentFrame), elementPositionsZ(ELEMENT_MT2_R)], 'c');
            
            % Update figure
            drawnow;

            % Save frame
            writeVideo(videoWriter, getframe(visualization));

            % Delete visualization
            if currentFrame < endFrame
                delete(information);
                delete(markers);
                delete(surface);
                delete(elements);
                delete(estimatedHead);
                delete(estimatedLeftArm);
                delete(estimatedRightArm);
                delete(estimatedThorax);
                delete(estimatedAbdomen);
                delete(estimatedPelvis);
                delete(estimatedLeftLeg);
                delete(estimatedRightLeg);
                delete(smoothedHead);
                delete(smoothedLeftArm);
                delete(smoothedRightArm);
                delete(smoothedThorax);
                delete(smoothedAbdomen);
                delete(smoothedPelvis);
                delete(smoothedLeftLeg);
                delete(smoothedRightLeg);
            end

        end
        close(videoWriter);
        close all;

    end
    
    % Unload HuMod library
    unloadlibrary(libraryName);
    
end