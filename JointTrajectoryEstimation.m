% ------------------------------------------------------
% This script estimates the joint trajectories and corrects the joint
% centers with using an extanded Kalman smoother approach according to
% [DeGroote2008, Yu2004] and the external HuMoD library based on the
% multibody system library MBSlib presented in [Friedmann2012].
%
% WARNING: This script uses very big matrices and has high requirements in
% physical memory.
%
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
processModelType = 'constantJerk';
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
    'A',...
    'B'...
};

% Add functions to search path
addpath('Scripts');

% Load constants
setForwardKinematicsConstants;

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
        createParameterVector('torso', parameters), ...
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
        dataset = datasets{datasetIndex};
        fprintf('STATUS: Processing dataset %s %s.\n', subject, dataset);
        
        % Load data file
        file = getFile(subject, dataset);
        if file
            variables = load(file);
            if isfield(variables, 'motion')
                motion = variables.motion;
                if ~isfield(motion.jointX, 'estimated')
                    fprintf('WARNING: No joint center data found!\n');
                    continue;
                end
            else
                fprintf('WARNING: No matching data found!\n');
                continue;
            end
        else
            fprintf('WARNING: No matching data file found!\n');
            continue;
        end
        
        % Set extended Kalman smoother parameters
        m = ELEMENT_Total * 3;
        switch processModelType

            case 'constantPosition'
            n = JOINT_Total;
            factor = 1;

            case 'constantVelocity'
            n = JOINT_Total * 2;
            factor = 2;

            case 'constantAcceleration'
            n = JOINT_Total * 3;
            factor = 3;

            case 'constantJerk'
            n = JOINT_Total * 4;
            factor = 4;

            otherwise
            fprintf('ERROR: Unknown process model!\n');
            return;

        end
        dt = 1 / motion.frameRate;
        kalmanProcessVariance = 18000.0;
        if(strcmp(subject, 'A'))
            kalmanMeasurementVariance = 0.5665^2;
        else
            kalmanMeasurementVariance = 0.65958^2;
        end
        kalmanMeasurementVarianceWeights = ones(ELEMENT_Total, 1);
        kalmanMeasurementVarianceWeights(ELEMENT_ACR_L)= 2.0;
        kalmanMeasurementVarianceWeights(ELEMENT_ACR_R) = 2.0;
        kalmanMeasurementVarianceWeights(ELEMENT_GTR_L) = 2.0;
        kalmanMeasurementVarianceWeights(ELEMENT_GTR_R) = 2.0;
        kalmanMeasurementVarianceWeights(ELEMENT_LFC_L) = 0.7;
        kalmanMeasurementVarianceWeights(ELEMENT_LFC_R) = 0.7;
        kalmanMeasurementVarianceWeights(ELEMENT_MFC_L) = 0.7;
        kalmanMeasurementVarianceWeights(ELEMENT_MFC_R) = 0.7;
        kalmanMeasurementVarianceWeights(ELEMENT_LM_L) = 0.5;
        kalmanMeasurementVarianceWeights(ELEMENT_LM_R) = 0.5;
        kalmanMeasurementVarianceWeights(ELEMENT_MM_L) = 0.5;
        kalmanMeasurementVarianceWeights(ELEMENT_MM_R) = 0.5;
        kalmanMeasurementVarianceWeights(ELEMENT_CAL_L) = 0.1;
        kalmanMeasurementVarianceWeights(ELEMENT_CAL_R) = 0.1;
        kalmanMeasurementVarianceWeights(ELEMENT_MT2_L) = 0.1;
        kalmanMeasurementVarianceWeights(ELEMENT_MT2_R) = 0.1;
        kalmanMeasurementVarianceWeights(ELEMENT_MT5_L) = 0.1;
        kalmanMeasurementVarianceWeights(ELEMENT_MT5_R) = 0.1;
        kalmanMeasurementVarianceWeights = repmat(kalmanMeasurementVarianceWeights, 1, 3)';
        kalmanMeasurementVarianceWeights = diag(kalmanMeasurementVarianceWeights(:));
        kalmanProcessVarianceWeights = ones(JOINT_Total, 1);
        kalmanProcessVarianceWeights(JOINT_rKJZ_L) = 3.0;
        kalmanProcessVarianceWeights(JOINT_rKJZ_R) = 3.0;
        kalmanProcessVarianceWeights(JOINT_rAJX_L) = 3.0;
        kalmanProcessVarianceWeights(JOINT_rAJY_L) = 3.0;
        kalmanProcessVarianceWeights(JOINT_rAJZ_L) = 3.0;
        kalmanProcessVarianceWeights(JOINT_rAJX_R) = 3.0;
        kalmanProcessVarianceWeights(JOINT_rAJY_R) = 3.0;
        kalmanProcessVarianceWeights(JOINT_rAJZ_R) = 3.0;
        kalmanProcessVarianceWeights = repmat(kalmanProcessVarianceWeights, 1, factor)';
        kalmanProcessVarianceWeights = diag(kalmanProcessVarianceWeights(:));
        A = createProcessModel(n, dt, processModelType);
        f = @(x) A * x;
        h = @(x) applyForwardKinematics(x, n, m, libraryName, processModelType);
        dfdx = @(x) A;
        dfdw = @(x) eye(n);
        dhdx = @(x) differentiateForwardKinematics(x, n, m, libraryName, processModelType);
        dhdv = @(x) eye(m);
        Q = createProcessVariance(n, dt, kalmanProcessVariance, processModelType) * kalmanProcessVarianceWeights;
        R = kalmanMeasurementVariance^2 * kalmanMeasurementVarianceWeights;
        
        % Create measurement matrix
        T = motion.frames;
        Z = zeros(T, m);
        for index = ELEMENT_MarkerStart:ELEMENT_MarkerEnd
           
            if index < ELEMENT_WRI_L
                Z(:, (index - 1) * 3 + 1) = motion.surfaceX(index - ELEMENT_MarkerStart + 1, 1:T)';
                Z(:, (index - 1) * 3 + 2) = motion.surfaceY(index - ELEMENT_MarkerStart + 1, 1:T)';
                Z(:, (index - 1) * 3 + 3) = motion.surfaceZ(index - ELEMENT_MarkerStart + 1, 1:T)';
            elseif (index == ELEMENT_WRI_L) || (index == ELEMENT_WRI_R)
                Z(:, (index - 1) * 3 + 1) = motion.markerX(index - ELEMENT_MarkerStart + 1, 1:T)';
                Z(:, (index - 1) * 3 + 2) = motion.markerY(index - ELEMENT_MarkerStart + 1, 1:T)';
                Z(:, (index - 1) * 3 + 3) = motion.markerZ(index - ELEMENT_MarkerStart + 1, 1:T)';
            elseif index > ELEMENT_WRI_R
                Z(:, (index - 1) * 3 + 1) = motion.surfaceX(index - ELEMENT_MarkerStart - 1, 1:T)';
                Z(:, (index - 1) * 3 + 2) = motion.surfaceY(index - ELEMENT_MarkerStart - 1, 1:T)';
                Z(:, (index - 1) * 3 + 3) = motion.surfaceZ(index - ELEMENT_MarkerStart - 1, 1:T)';
            end
            
        end
        for index = ELEMENT_JointStart:ELEMENT_JointEnd
           
            if index < ELEMENT_LLJ
                Z(:, (index - 1) * 3 + 1) = motion.jointX.estimated(index - ELEMENT_JointStart + 1, 1:T)';
                Z(:, (index - 1) * 3 + 2) = motion.jointY.estimated(index - ELEMENT_JointStart + 1, 1:T)';
                Z(:, (index - 1) * 3 + 3) = motion.jointZ.estimated(index - ELEMENT_JointStart + 1, 1:T)';
            elseif index >= ELEMENT_LLJ
                Z(:, (index - 1) * 3 + 1) = motion.jointX.estimated(index - ELEMENT_JointStart + 2, 1:T)';
                Z(:, (index - 1) * 3 + 2) = motion.jointY.estimated(index - ELEMENT_JointStart + 2, 1:T)';
                Z(:, (index - 1) * 3 + 3) = motion.jointZ.estimated(index - ELEMENT_JointStart + 2, 1:T)';
            end
            
        end
        
        % Set initial values
        P0 = eye(n);
        x0 = zeros(n, 1);
        x0((JOINT_pBJX - 1) * factor + 1) = parameters.joints.absolutePosition.LLJ(1);
        x0((JOINT_pBJY - 1) * factor + 1) = parameters.joints.absolutePosition.LLJ(2);
        x0((JOINT_pBJZ - 1) * factor + 1) = parameters.joints.absolutePosition.LLJ(3);
        if strcmp(subject, 'A')
            x0((JOINT_rBJY - 1) * factor + 1) = -pi / 2;
        end
        
        % Optimize initial values
        z = Z(1, :);
        g = @(x, p) applyForwardKinematics(x, n, m, libraryName, processModelType)';
        options = optimset('Display', 'off');
        lb = -pi / 2 * ones(n, 1);
        ub = pi / 2 * ones(n, 1);
        lb((1 - 1) * factor + 1) = -1500.0;
        lb((2 - 1) * factor + 1) = -1500.0;
        lb((3 - 1) * factor + 1) = -1500.0;
        lb((4 - 1) * factor + 1) = -pi;
        lb((5 - 1) * factor + 1) = -pi;
        lb((6 - 1) * factor + 1) = -pi;
        ub((1 - 1) * factor + 1) = 1500.0;
        ub((2 - 1) * factor + 1) = 1500.0;
        ub((3 - 1) * factor + 1) = 1500.0;
        ub((4 - 1) * factor + 1) = pi;
        ub((5 - 1) * factor + 1) = pi;
        ub((6 - 1) * factor + 1) = pi;
        [x0, r, ~, e] = lsqcurvefit(g, x0, [], z, lb, ub, options);
        if e <= 0
            
            % Unload HuMod library
            unloadlibrary(libraryName);
            
            % Abort data processing
            fprintf('ERROR: Optimization of initial values terminated with exit code %u!\n', e);
            return;
            
        end
        fprintf('STATUS: Initial values optimized with a squared residual of %.4f.\n', r);
        
        % Apply Extended Kalman Smoother
        X = extendedKalmanSmoother(Z, f, h, dfdx, dfdw, dhdx, dhdv, Q, R, x0, P0);

        % Save smoothed joint trajectories 
        motion.trajectoryLabels = { ...
            'pBJX', ...
            'pBJY', ...
            'pBJZ', ...
            'rBJX', ...
            'rBJY', ...
            'rBJZ', ...
            'rLNJX', ...
            'rLNJY', ...
            'rLNJZ', ...
            'rSJX_L', ...
            'rSJY_L', ...
            'rSJZ_L', ...
            'rSJX_R', ...
            'rSJY_R', ...
            'rSJZ_R', ...
            'rEJZ_L', ...
            'rEJZ_R', ...
            'rLLJX', ...
            'rLLJY', ...
            'rLLJZ', ...
            'rHJX_L', ...
            'rHJY_L', ...
            'rHJZ_L', ...
            'rHJX_R', ...
            'rHJY_R', ...
            'rHJZ_R', ...
            'rKJZ_L', ...
            'rKJZ_R', ...
            'rAJX_L', ...
            'rAJY_L', ...
            'rAJZ_L', ...
            'rAJX_R', ...
            'rAJY_R', ...
            'rAJZ_R' ...
        };
        switch processModelType
    
            case 'constantPosition'
            motion.trajectory.q = X';
            if isfield(motion.trajectory, 'dqdt')
                motion.trajectory = rmfield(motion.trajectory, 'dqdt');
            end
            if isfield(motion.trajectory, 'ddqddt')
                motion.trajectory = rmfield(motion.trajectory, 'ddqddt');
            end
            
            case 'constantVelocity'
            jointTrajectoryQ = zeros(JOINT_Total, T);
            jointTrajectoryDQ = zeros(JOINT_Total, T);
            for index = 1:JOINT_Total
            
                jointTrajectoryQ(index, 1:T) = X(:, (index - 1) * factor + 1)';
                jointTrajectoryDQ(index, 1:T) = X(:, (index - 1) * factor + 2)';
                
            end
            motion.trajectory.q = jointTrajectoryQ;
            motion.trajectory.dqdt = jointTrajectoryDQ;
            if isfield(motion.trajectory, 'ddqddt')
                motion.trajectory = rmfield(motion.trajectory, 'ddqddt');
            end

            case 'constantAcceleration'
            jointTrajectoryQ = zeros(JOINT_Total, T);
            jointTrajectoryDQ = zeros(JOINT_Total, T);
            jointTrajectoryDDQ = zeros(JOINT_Total, T);
            for index = 1:JOINT_Total
            
                jointTrajectoryQ(index, 1:T) = X(:, (index - 1) * factor + 1)';
                jointTrajectoryDQ(index, 1:T) = X(:, (index - 1) * factor + 2)';
                jointTrajectoryDDQ(index, 1:T) = X(:, (index - 1) * factor + 3)';
                
            end
            motion.trajectory.q = jointTrajectoryQ;
            motion.trajectory.dqdt = jointTrajectoryDQ;
            motion.trajectory.ddqddt = jointTrajectoryDDQ;

            case 'constantJerk'
            jointTrajectoryQ = zeros(JOINT_Total, T);
            jointTrajectoryDQ = zeros(JOINT_Total, T);
            jointTrajectoryDDQ = zeros(JOINT_Total, T);
            for index = 1:JOINT_Total
            
                jointTrajectoryQ(index, 1:T) = X(:, (index - 1) * factor + 1)';
                jointTrajectoryDQ(index, 1:T) = X(:, (index - 1) * factor + 2)';
                jointTrajectoryDDQ(index, 1:T) = X(:, (index - 1) * factor + 3)';
                
            end
            motion.trajectory.q = jointTrajectoryQ;
            motion.trajectory.dqdt = jointTrajectoryDQ;
            motion.trajectory.ddqddt = jointTrajectoryDDQ;
            
            otherwise
            fprintf('ERROR: Unknown process model!\n');
            ruturn;

        end
        
        % Save smoothed joint data
        motion.jointLabels.smoothed = { ...
                'LNJ', ...
                'SJ_L', ...
                'SJ_R', ...
                'EJ_L', ...
                'EJ_R', ...
                'LLJ', ...
                'HJ_L', ...
                'HJ_R', ...
                'KJ_L', ...
                'KJ_R', ...
                'AJ_L', ...
                'AJ_R' ...
        };
        fprintf('STATUS: Computing smoothed joint centers.\n');
        motion.jointX.smoothed = zeros((ELEMENT_JointEnd - ELEMENT_JointStart + 1), T);
        motion.jointY.smoothed = zeros((ELEMENT_JointEnd - ELEMENT_JointStart + 1), T);
        motion.jointZ.smoothed = zeros((ELEMENT_JointEnd - ELEMENT_JointStart + 1), T);
        statusCounter = 0;
        for currentFrame = 1:T
            
            % Compute smoothed joint centers
            x = motion.trajectory.q(:, currentFrame);
            z = applyForwardKinematics(x, (n / factor), m, libraryName, 'constantPosition');
            for elementIndex = ELEMENT_JointStart:ELEMENT_JointEnd
    
                motion.jointX.smoothed((elementIndex - ELEMENT_JointStart + 1), currentFrame) = z((elementIndex - 1) * 3 + 1);
                motion.jointY.smoothed((elementIndex - ELEMENT_JointStart + 1), currentFrame) = z((elementIndex - 1) * 3 + 2);
                motion.jointZ.smoothed((elementIndex - ELEMENT_JointStart + 1), currentFrame) = z((elementIndex - 1) * 3 + 3);
    
            end
            
            % Print status
            statusCounter = statusCounter + 1;
            if statusCounter >= 100
                fprintf('STATUS: %.1f %%\n', (currentFrame - 1) / (T - 1) * 100);
                statusCounter = 0;
            end
            
        end
        
        % Save processed data
        fprintf('STATUS: Saving dataset %s %s.\n', subject, dataset);
        motion = orderfields(motion);
        variables.motion = motion;
        save(file, '-struct', 'variables');
        
    end

    % Unload HuMod library
    unloadlibrary(libraryName);
    
end