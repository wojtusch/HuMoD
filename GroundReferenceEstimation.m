% ------------------------------------------------------
% This script estimates the rotation and translation parameters to
% transform points from the motion coordinate system into the force
% coordinate system. The force coordinate system is the reference
% coordinate system for all datasets. The origin is in the middle of the
% projected sensor plane.
% The marker FL corresponds to the projection of the sensor L1.
% The marker FR corresponds to the projection of the sensor R2.
% The marker RL corresponds to the projection of the sensor L4.
% The marker RR corresponds to the projection of the sensor R3.
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
markerSize = 19;                    % Diameter of the applied markers in millimeters
markerOffset = markerSize / 2;      % Offset produced by the applied markers in millimeters
distanceL1L4 = 1400;                % Distance between sensor L1 and L1 in millimeters
distanceL1R2 = 238 + 64 + 238;      % Distance between sensor L1 and R2 in millimeters
distanceL1L2 = 238;                 % Distance between sensor L1 and L2 in millimeters
distanceS1S4 = 1600;                % Distance between sensor S1 and S4 in millimeters
distanceS1S2 = 415 + 415;           % Distance between sensor S1 and S2 in millimeters
depthL = 41;                        % Distance between treadmill surface and the L sensor plane
depthS = 278;                       % Distance between treadmill surface and the S sensor plane
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

% Add functions to search path
addpath('Scripts');

for subjectIndex = 1:length(subjects)
    
    % Set parameters
    subject = subjects{subjectIndex};

    % Load measured data
    referenceFile = [getPath, filesep, subject, filesep, 'Raw data', filesep, 'GroundReference.mat'];
    load(referenceFile);

    % Fit sensor plane
    sensorModel = [(distanceL1L4 / 2), (distanceL1L4 / 2), -(distanceL1L4 / 2), -(distanceL1L4 / 2); 0, 0, 0, 0; -(distanceL1R2 / 2), (distanceL1R2 / 2), (distanceL1R2 / 2), -(distanceL1R2 / 2)];
    sensorData = [ ...
        motion.markerX(1, :), motion.markerX(2, :), motion.markerX(3, :), motion.markerX(4, :); ...
        motion.markerY(1, :) - markerOffset, motion.markerY(2, :) - markerOffset, motion.markerY(3, :) - markerOffset, motion.markerY(4, :) - markerOffset; ...
        motion.markerZ(1, :), motion.markerZ(2, :), motion.markerZ(3, :), motion.markerZ(4, :) ...
    ];
    sensorData = sensorData(:, randperm(length(sensorData)));
    fprintf('STATUS: Fitting sensor plane for subject %s.\n', subject);
    [rotationForce2Motion, translationForce2Motion] = icp(sensorData, sensorModel);

    % Set translation vector and rotation matrix for the transformation
    % from motion into force coordinate system
    translationMotion2Force = -translationForce2Motion;
    rotationMotion2Force = rotationForce2Motion^(-1);
    
    % Compute sensor positions in the force coordinate system
    vectorL1 = [(distanceL1L4 / 2); -depthL; -(distanceL1R2 / 2)];
    vectorL2 = [(distanceL1L4 / 2); -depthL; -((distanceL1R2 / 2) - distanceL1L2)];
    vectorL3 = [-(distanceL1L4 / 2); -depthL; -((distanceL1R2 / 2) - distanceL1L2)];
    vectorL4 = [-(distanceL1L4 / 2); -depthL; -(distanceL1R2 / 2)];
    vectorR1 = [(distanceL1L4 / 2); -depthL; ((distanceL1R2 / 2) - distanceL1L2)];
    vectorR2 = [(distanceL1L4 / 2); -depthL; (distanceL1R2 / 2)];
    vectorR3 = [-(distanceL1L4 / 2); -depthL; (distanceL1R2 / 2)];
    vectorR4 = [-(distanceL1L4 / 2); -depthL; ((distanceL1R2 / 2) - distanceL1L2)];
    vectorS1 = [(distanceS1S4 / 2); -depthS; -(distanceS1S2 / 2)];
    vectorS2 = [(distanceS1S4 / 2); -depthS; (distanceS1S2 / 2)];
    vectorS3 = [-(distanceS1S4 / 2); -depthS; (distanceS1S2 / 2)];
    vectorS4 = [-(distanceS1S4 / 2); -depthS; -(distanceS1S2 / 2)];
    
    % Save processed tranformation and sensor data
    for datasetIndex = 1:length(datasets)

        % Set parameters
        dataset = datasets{datasetIndex};

        % Load data file
        file = getFile(subject, dataset);
        if file
            variables = load(file);
        else
            fprintf('WARNING: No matching data file found!\n');
            continue;
        end
        fprintf('STATUS: Processing dataset %s %s.\n', subject, dataset);
        
        % Save processed data
        ground = [];
        sensorData = [
            vectorL1, ...
            vectorL2, ...
            vectorL3, ...
            vectorL4, ...
            vectorR1, ...
            vectorR2, ...
            vectorR3, ...
            vectorR4, ...
            vectorS1, ...
            vectorS2, ...
            vectorS3, ...
            vectorS4 ...
        ];
        ground.sensorX = sensorData(1,:)';
        ground.sensorY = sensorData(2,:)';
        ground.sensorZ = sensorData(3,:)';
        ground.sensorLabels = {'L1', 'L2', 'L3', 'L4', 'R1', 'R2', 'R3', 'R4', 'S1', 'S2', 'S3', 'S4'};
        ground.translationMotion2Force = translationMotion2Force;
        ground.rotationMotion2Force = rotationMotion2Force;
        ground.groundPosition = [0; 0; 0];
        ground.groundNormal = [0; 1; 0];
        ground = orderfields(ground);
        variables.ground = ground;
        save(file, '-struct', 'variables');
        fprintf('STATUS: Saving dataset %s %s.\n', subjects{subjectIndex}, datasets{datasetIndex});

    end
end