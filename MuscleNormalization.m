% ------------------------------------------------------
% This script normalizes the filtered muscle data by finding the global
% maximum values in all datasets and correcting outliers.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
% Licensed under BSD 3-Clause License% ------------------------------------------------------

% Clean up workspace
clc;
clear all;
close all;

% Set parameters
plotNormalization = 1;
savePath = [getPath, filesep, 'Normalization', filesep];
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
    
    % Determine maximum values
    aggregatedActivities = [];
    for datasetIndex = 1:length(datasets)

        % Set parameters
        dataset = datasets{datasetIndex};
        subject = subjects{subjectIndex};
        fprintf('STATUS: Searching in dataset %s %s.\n', subject, dataset);

        % Load data file
        file = getFile(subject, dataset);
        if file
            variables = load(file);
            if isfield(variables, 'muscle')
                muscle = variables.muscle;
            else
                fprintf('WARNING: No matching data found!\n');
                continue;
            end
        else
            fprintf('WARNING: No matching data file found!\n');
            continue;
        end
        
        % Aggretage activities
        aggregatedActivities = [aggregatedActivities, muscle.activities.filtered];
        
    end
    maximumValues = max(aggregatedActivities, [], 2)';
    minimumValues = zeros(size(maximumValues));
    
    % Plot aggregated muscle data
    if plotNormalization
        visualization = figure('Name', 'Muscle normalization', 'NumberTitle', 'off', 'Color', 'white', 'Position', [0, 0, 1400, 600]);
        time = 0:(1 / muscle.frameRate):((length(aggregatedActivities) - 1) / muscle.frameRate);
        for muscleIndex = 1:2:length(muscle.muscleLabels)
            subplot(2, 4, ceil(muscleIndex / 2));
            plot(time, aggregatedActivities(muscleIndex, :), 'r-');
            hold on;
            grid on;
            plot([time(1), time(end)], [maximumValues(muscleIndex), maximumValues(muscleIndex)], 'r--');
            plot(time, aggregatedActivities((muscleIndex + 1), :), 'b-');
            plot([time(1), time(end)], [maximumValues(muscleIndex + 1), maximumValues(muscleIndex + 1)], 'b--');
            title([muscle.muscleLabels{muscleIndex}, '(red) / ', muscle.muscleLabels{muscleIndex + 1}, ' (blue)']);
            xlabel('Time in s');

        end
        suplabel([subject, ' - HuMoD Database']);

        % Save figure
        saveTightFigure(visualization, [savePath, subject, '.png']);
        fprintf('STATUS: Figure for subject %s was saved.\n', subject);
        close all;
    end
    
    % Normalize filtered activities
    for datasetIndex = 1:length(datasets)

        % Set parameters
        dataset = datasets{datasetIndex};
        fprintf('STATUS: Normalizing dataset %s %s.\n', subject, dataset);

        % Load data file
        file = getFile(subject, dataset);
        if file
            variables = load(file);
            muscle = variables.muscle;        
        else
            fprintf('WARNING: No matching data file found!\n');
            continue;
        end

        % Normalize the processed muscle data
        muscle.activities.normalized = zeros(size(muscle.activities.filtered));
        for muscleIndex = 1:length(muscle.muscleLabels);
            muscle.activities.normalized(muscleIndex, :) = muscle.activities.filtered(muscleIndex, :) / maximumValues(muscleIndex);
        end    
        muscle.maximumValues = maximumValues;
        muscle.minimumValues = minimumValues;

        % Save processed data
        fprintf('STATUS: Saving dataset %s %s.\n', subject, dataset);
        muscle = orderfields(muscle);
        variables.muscle = muscle;
        save(file, '-struct', 'variables');

    end
    
end
