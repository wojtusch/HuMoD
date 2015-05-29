% ------------------------------------------------------
% This script visualizes muscle data.
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
filterVariant = 'normalized';                     % 'filtered', 'normalized'
savePath = [getPath, filesep, 'Muscle', filesep];
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
    'A',...
    'B'...
};

for subjectIndex = 1:length(subjects)
    for datasetIndex = 1:length(datasets)
        
        % Set parameters
        dataset = datasets{datasetIndex};
        subject = subjects{subjectIndex};

        % Load data file
        file = getFile(subject, dataset);
        if file
            variables = load(file);
            if isfield(variables, 'muscle')
                muscle = variables.muscle;
                name = regexp(file, '[^/]*(?=\.[^.]+($|\?))', 'match');
                name = name{1};
            else
                fprintf('WARNING: No matching data found!\n');
                return;
            end
        else
            fprintf('ERROR: No matching data file found!\n');
            return;
        end

        % Plot muscle data
        filterAlgorithm = muscle.filterAlgorithm;
        if ~strcmpi(filterVariant, 'filtered') && ~isfield(muscle, 'normalizedActivities')
            fprintf('ERROR: Dataset %s %s does not contain normalized activities.\n', subject, dataset);
            continue;
        else
            visualization = figure('Name', 'Muscle activities', 'NumberTitle', 'off', 'Color', 'white', 'Position', [0, 0, 1400, 600]);
            time = 0:(1 / muscle.frameRate):((muscle.frames - 1) / muscle.frameRate);
            if ~strcmpi(filterVariant, 'filtered')
                data = muscle.normalizedActivities;
            else
                data = muscle.filteredActivities;
            end
            for muscleIndex = 1:2:length(muscle.muscleLabels)
                subplot(2, 4, ceil(muscleIndex / 2));
                plot(time, reshape(data(muscleIndex, :), [1, muscle.frames]), 'r-');
                hold on;
                grid on;
                plot(time, reshape(data((muscleIndex + 1), :), [1, muscle.frames]), 'b-');
                title([muscle.muscleLabels{muscleIndex}, '(red) / ', muscle.muscleLabels{muscleIndex + 1}, ' (blue)']);
                xlabel('Time in s');

            end
            suplabel([dataset, ' ', name, ' - ', filterAlgorithm, ' (', filterVariant, ') - HuMoD Database']);
        end

        % Save figure
        saveTightFigure(visualization, [savePath, subject, filesep, dataset, '.png']);
        fprintf('STATUS: Figure for dataset %s %s was saved.\n', subject, dataset);
        close all;

    end
end
