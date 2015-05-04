% ------------------------------------------------------
% This script transforms the motion data into the force coordinate system.
% The force coordinate system is the reference coordinate system for all
% datasets. The origin is in the middle of the projected sensor plane.
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

% Set parameters
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
    '8', ...
    '9.1', ...
    '9.2', ...
    '9.3' ...
};
subjects = {
    'A', ...
    'B' ...
};

% Add functions to search path
addpath('Scripts');

for subjectIndex = 1:length(subjects)
    for datasetIndex = 1:length(datasets)

        % Set parameters
        subject = subjects{subjectIndex};
        dataset = datasets{datasetIndex};
        fprintf('STATUS: Processing dataset %s %s.\n', subject, dataset);

        % Load data file
        file = getFile(subject, dataset);
        if file
            variables = load(file);
            if isfield(variables, 'motion') && isfield(variables, 'ground')
                motion = variables.motion;
                ground = variables.ground;
            else
                fprintf('WARNING: No matching data found!\n');
                continue;
            end
        else
            fprintf('ERROR: No matching data file found!\n');
            return;
        end
        
        % Transform marker data
        for markerIndex = 1:length(motion.markerLabels)
            for dataIndex = 1:motion.frames
                originalMarker = [motion.markerX(markerIndex, dataIndex); motion.markerY(markerIndex, dataIndex); motion.markerZ(markerIndex, dataIndex)];
                transformedMarker = ground.rotationMotion2Force * originalMarker + ground.translationMotion2Force;
                motion.markerX(markerIndex, dataIndex) = transformedMarker(1);
                motion.markerY(markerIndex, dataIndex) = transformedMarker(2);
                motion.markerZ(markerIndex, dataIndex) = transformedMarker(3);
            end
        end

        % Save processed data
        fprintf('STATUS: Saving dataset %s %s.\n', subject, dataset);
        variables.motion = motion;
        save(file, '-struct', 'variables');

    end
end