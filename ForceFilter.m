% ------------------------------------------------------
% This script processes the raw force data and transforms the coordinate
% system according to [Wu2002], [Wu2005]. Also motion and force data is
% synchronized by compensating the time delay between motion and force
% measurements.
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
intercept = 0.017767380272882;          % Intercept of the linear regression for force measurement delay
gradient = -5.713883360543501e-05;      % Gradient of the linear regression for force measurement delay
filterHalfOrder = 3;                    % Half order of the zero-phase high-pass and low-pass filters
filterCutOff = 50;                      % Cut-off frequency of the zero-phase low-pass filter
subjects = {
    'A',...
    'B'...
};
signals = {
    'Fx1', ...
    'Fx2', ...
    'Fy1', ...
    'Fy2', ...
    'FzL1', ...
    'FzL2', ...
    'FzL3', ...
    'FzL4', ...
    'FzR1', ...
    'FzR2', ...
    'FzR3', ...
    'FzR4' ...
};

% Add functions to search path
addpath('Scripts');

for subjectIndex = 1:length(subjects)
    for datasetIndex = 1:length(datasets)

        % Set parameters
        dataset = datasets{datasetIndex};
        subject = subjects{subjectIndex};
        fprintf('STATUS: Processing dataset %s %s.\n', subject, dataset);

        % Load data file
        saveFile = getFile(subject, dataset);
        if saveFile
            [path, file, extention] = fileparts(saveFile);
            dataFile = [path, filesep, 'Raw data', filesep, 'Force ', file, extention];
            variables = load(dataFile);
            frameRate = variables.fs;
            frames = length(variables.vBden);
        else
            fprintf('WARNING: No matching data file found!\n');
            continue;
        end
        
        % Apply zero-phase low-pass filter
        filterPasses = 2;
        filterCorrectedCutOff = filterCutOff / ((2^(1 / filterPasses) - 1)^(1 / 4));
        [filterB, filterA] = butter(filterHalfOrder, (filterCorrectedCutOff / (0.5 * frameRate)));
        for signalIndex = 1:length(signals)
            variables.(signals{signalIndex}) = filtfilt(filterB, filterA, variables.(signals{signalIndex}));
        end
        
        % Synchronize force and motion measurements
        originalSamplePoints = 0:(1 / frameRate):((frames - 1) / frameRate);
        compensatedSamplePoints = originalSamplePoints - (intercept + gradient * originalSamplePoints);
        for signalIndex = 1:length(signals)
            variables.(signals{signalIndex}) = interp1(compensatedSamplePoints, variables.(signals{signalIndex}), originalSamplePoints, 'pchip', 0);
        end
        
        % Compute forces with transforming the coordinate system
        Fx = variables.Fy1 + variables.Fy2;
        Fx1 = variables.Fy1;
        Fx2 = variables.Fy2;
        Fy_L = variables.FzL1 + variables.FzL2 + variables.FzL3 + variables.FzL4;
        Fy_L1 = variables.FzL1;
        Fy_L2 = variables.FzL2;
        Fy_L3 = variables.FzL3;
        Fy_L4 = variables.FzL4;
        Fy_R = variables.FzR1 + variables.FzR2 + variables.FzR3 + variables.FzR4;
        Fy_R1 = variables.FzR1;
        Fy_R2 = variables.FzR2;
        Fy_R3 = variables.FzR3;
        Fy_R4 = variables.FzR4;
        Fz = variables.Fx1 + variables.Fx2;
        Fz1 = variables.Fx1;
        Fz2 = variables.Fx2;
        
        % Compensate force sensor offset and drift by linear regression
        compensationFx = [ones(16000, 1), [1:8000, (frames - 7999):frames]'] \ [Fx(1:8000), Fx((frames - 7999):frames)]';
        compensationFx1 = [ones(16000, 1), [1:8000, (frames - 7999):frames]'] \ [Fx1(1:8000), Fx1((frames - 7999):frames)]';
        compensationFx2 = [ones(16000, 1), [1:8000, (frames - 7999):frames]'] \ [Fx2(1:8000), Fx2((frames - 7999):frames)]';
        compensationFy_L = [ones(16000, 1), [1:8000, (frames - 7999):frames]'] \ [Fy_L(1:8000), Fy_L((frames - 7999):frames)]';
        compensationFy_L1 = [ones(16000, 1), [1:8000, (frames - 7999):frames]'] \ [Fy_L1(1:8000), Fy_L1((frames - 7999):frames)]';
        compensationFy_L2 = [ones(16000, 1), [1:8000, (frames - 7999):frames]'] \ [Fy_L2(1:8000), Fy_L2((frames - 7999):frames)]';
        compensationFy_L3 = [ones(16000, 1), [1:8000, (frames - 7999):frames]'] \ [Fy_L3(1:8000), Fy_L3((frames - 7999):frames)]';
        compensationFy_L4 = [ones(16000, 1), [1:8000, (frames - 7999):frames]'] \ [Fy_L4(1:8000), Fy_L4((frames - 7999):frames)]';
        compensationFy_R = [ones(16000, 1), [1:8000, (frames - 7999):frames]'] \ [Fy_R(1:8000), Fy_R((frames - 7999):frames)]';
        compensationFy_R1 = [ones(16000, 1), [1:8000, (frames - 7999):frames]'] \ [Fy_R1(1:8000), Fy_R1((frames - 7999):frames)]';
        compensationFy_R2 = [ones(16000, 1), [1:8000, (frames - 7999):frames]'] \ [Fy_R2(1:8000), Fy_R2((frames - 7999):frames)]';
        compensationFy_R3 = [ones(16000, 1), [1:8000, (frames - 7999):frames]'] \ [Fy_R3(1:8000), Fy_R3((frames - 7999):frames)]';
        compensationFy_R4 = [ones(16000, 1), [1:8000, (frames - 7999):frames]'] \ [Fy_R4(1:8000), Fy_R4((frames - 7999):frames)]';
        compensationFz = [ones(16000, 1), [1:8000, (frames - 7999):frames]'] \ [Fz(1:8000), Fz((frames - 7999):frames)]';
        compensationFz1 = [ones(16000, 1), [1:8000, (frames - 7999):frames]'] \ [Fz1(1:8000), Fz1((frames - 7999):frames)]';
        compensationFz2 = [ones(16000, 1), [1:8000, (frames - 7999):frames]'] \ [Fz2(1:8000), Fz2((frames - 7999):frames)]';
        for dataIndex = 1:frames
            Fx(dataIndex) = Fx(dataIndex) - (compensationFx(2) * dataIndex + compensationFx(1));
            Fx1(dataIndex) = Fx1(dataIndex) - (compensationFx1(2) * dataIndex + compensationFx1(1));
            Fx2(dataIndex) = Fx2(dataIndex) - (compensationFx2(2) * dataIndex + compensationFx2(1));
            Fy_L(dataIndex) = Fy_L(dataIndex) - (compensationFy_L(2) * dataIndex + compensationFy_L(1));
            Fy_L1(dataIndex) = Fy_L1(dataIndex) - (compensationFy_L1(2) * dataIndex + compensationFy_L1(1));
            Fy_L2(dataIndex) = Fy_L2(dataIndex) - (compensationFy_L2(2) * dataIndex + compensationFy_L2(1));
            Fy_L3(dataIndex) = Fy_L3(dataIndex) - (compensationFy_L3(2) * dataIndex + compensationFy_L3(1));
            Fy_L4(dataIndex) = Fy_L4(dataIndex) - (compensationFy_L4(2) * dataIndex + compensationFy_L4(1));
            Fy_R(dataIndex) = Fy_R(dataIndex) - (compensationFy_R(2) * dataIndex + compensationFy_R(1));
            Fy_R1(dataIndex) = Fy_R1(dataIndex) - (compensationFy_R1(2) * dataIndex + compensationFy_R1(1));
            Fy_R2(dataIndex) = Fy_R2(dataIndex) - (compensationFy_R2(2) * dataIndex + compensationFy_R2(1));
            Fy_R3(dataIndex) = Fy_R3(dataIndex) - (compensationFy_R3(2) * dataIndex + compensationFy_R3(1));
            Fy_R4(dataIndex) = Fy_R4(dataIndex) - (compensationFy_R4(2) * dataIndex + compensationFy_R4(1));
            Fz(dataIndex) = Fz(dataIndex) - (compensationFz(2) * dataIndex + compensationFz(1));
            Fz1(dataIndex) = Fz1(dataIndex) - (compensationFz1(2) * dataIndex + compensationFz1(1));
            Fz2(dataIndex) = Fz2(dataIndex) - (compensationFz2(2) * dataIndex + compensationFz2(1));
        end
        clear compensationFx compensationFx1 compensationFx2 compensationFy_L compensationFy_L1 compensationFy_L2 compensationFy_L3 compensationFy_L4 compensationFy_R compensationFy_R1 compensationFy_R2 compensationFy_R3 compensationFy_R4 compensationFz compensationFz1 compensationFz2 dataIndex
        
        % Save processed data
        fprintf('STATUS: Saving dataset %s %s.\n', subject, dataset);
        force.frameRate = frameRate;
        force.frames = frames;
        force.treadmillVelocity = variables.vBden;
        force.groundReactionForceX = Fx;
        force.forceSensorX1 = Fx1;
        force.forceSensorX2 = Fx2;
        force.groundReactionForceY_L = Fy_L;
        force.forceSensorY_L1 = Fy_L1;
        force.forceSensorY_L2 = Fy_L2;
        force.forceSensorY_L3 = Fy_L3;
        force.forceSensorY_L4 = Fy_L4;
        force.groundReactionForceY_R = Fy_R;
        force.forceSensorY_R1 = Fy_R1;
        force.forceSensorY_R2 = Fy_R2;
        force.forceSensorY_R3 = Fy_R3;
        force.forceSensorY_R4 = Fy_R4;
        force.groundReactionForceZ = Fz;
        force.forceSensorZ1 = Fz1;
        force.forceSensorZ2 = Fz2;
        if exist(saveFile, 'file') == 2
            save(saveFile, 'force', '-append');
        else
            save(saveFile, 'force');
        end
    end
end
