% ------------------------------------------------------
% This script adds meta data to the datasets.
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
times = {
    60,...
    60,...
    60,...
    60,...
    60,...
    60,...
    60,...
    112,...
    120,...
    120,...
    40,...
    100,...
    20,...
    60,...
    60,...
    60 ...
};

for subjectIndex = 1:length(subjects)
    for datasetIndex = 1:length(datasets)
        
        % Set parameters
        dataset = datasets{datasetIndex};
        subject = subjects{subjectIndex};
        
        % Load processed data file
        fprintf('STATUS: Processing dataset %s %s.\n', subject, dataset);
        processedFile = getFile(subject, dataset);
        if processedFile
            processedVariables = load(processedFile);
        else
            fprintf('WARNING: No matching processed data file found!\n');
            continue;
        end
        
        % Save meta data in processed data
        fprintf('STATUS: Saving processed data of dataset %s %s.\n', subjects{subjectIndex}, datasets{datasetIndex});
        datasetPath = regexp(processedFile, filesep, 'split');
        processedVariables.meta = [];
        processedVariables.meta.subject = subject;
        processedVariables.meta.experiment = [dataset, ' ', datasetPath{end}(1:(end - 4))];
        processedVariables.meta.duration = processedVariables.motion.frames / processedVariables.motion.frameRate;
        processedVariables.meta.startTime = 20;
        processedVariables.meta.endTime = processedVariables.meta.startTime + times{datasetIndex};
        processedVariables.meta.license = 'This file is part of the HuMoD Database. The HuMoD Database is made available under the Open Database License (ODbL v1.0). Any rights in individual contents of the database are licensed under the Database Contents License (DbCL v1.0).';
        processedVariables.meta.author = 'Janis Wojtusch (wojtusch@sim.tu-darmstadt.de)';
        if strcmp(subject, 'A')
            processedVariables.meta.date = '2014/04/17';
        elseif strcmp(subject, 'B')
            processedVariables.meta.date = '2014/11/11';
        else
            fprintf('ERROR: No matching subject found!\n');
            return;
        end
        save(processedFile, '-struct', 'processedVariables');
        
        % Load raw data files
        rawForceFile = [strjoin(datasetPath(1:(end - 1)), filesep), filesep, 'Raw data', filesep, 'Force ', datasetPath{end}];
        rawMotionFile = [strjoin(datasetPath(1:(end - 1)), filesep), filesep, 'Raw data', filesep, 'Motion ', datasetPath{end}];
        rawMuscleFile = [strjoin(datasetPath(1:(end - 1)), filesep), filesep, 'Raw data', filesep, 'Muscle ', datasetPath{end}];
        if exist(rawForceFile, 'file')
            rawForceVariables = load(rawForceFile);
        else
            fprintf('WARNING: No matching raw force data file found!\n');
            continue;
        end
        if exist(rawMotionFile, 'file')
            rawMotionVariables = load(rawMotionFile);
        else
            fprintf('WARNING: No matching raw motion data file found!\n');
            continue;
        end
        if exist(rawMuscleFile, 'file')
            rawMuscleVariables = load(rawMuscleFile);
        else
            fprintf('WARNING: No matching raw muscle data file found!\n');
            continue;
        end
        
        % Save meta data in raw data
        fprintf('STATUS: Saving raw data of dataset %s %s.\n', subjects{subjectIndex}, datasets{datasetIndex});
        rawForceVariables.meta = processedVariables.meta;
        rawMotionVariables.meta = processedVariables.meta;
        rawMuscleVariables.meta = processedVariables.meta;
        save(rawForceFile, '-struct', 'rawForceVariables');
        save(rawMotionFile, '-struct', 'rawMotionVariables');
        save(rawMuscleFile, '-struct', 'rawMuscleVariables');
        
    end
end
