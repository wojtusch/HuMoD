% ------------------------------------------------------
% This script visualizes ground reaction force and events data.
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
savePath = [getPath, '../Grafiken/Events/'];
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
            if isfield(variables, 'force') && isfield(variables, 'events')
                force = variables.force;
                events = variables.events;
                name = regexp(file, '[^/]*(?=\.[^.]+($|\?))', 'match');
                name = name{1};
            else
                fprintf('WARNING: No matching data found!\n');
                continue;
            end
        else
            fprintf('WARNING: No matching data file found!\n');
            continue;
        end

        % Plot force and events data
        visualization = figure('Name', 'Events', 'NumberTitle', 'off', 'Color', 'white', 'Position', [0, 0, 1400, 600]);
        time = 0:(1 / force.frameRate):((force.frames - 1) / force.frameRate);
        subplot(3, 1, 1);
            if isfield(force, 'groundReactionForceX_L')
                plot(time, force.groundReactionForceX, 'Color', [0.7, 0.7, 0.7]);
                hold on;
                plot(time, force.groundReactionForceX_L, 'r-');
                plot(time, force.groundReactionForceX_R, 'b-');
                title('Ground reaction force X (gray) / X_L (red) / X_R (blue)');
            else
                plot(time, force.groundReactionForceX, 'k-');
                title('Ground reaction force X');
            end
            xlabel('Time in s');
            ylabel('Force in N');
            grid on;
            maximumValue = max(force.groundReactionForceX);
            minimumValue = min(force.groundReactionForceX);
            if ~strcmp(dataset, '6') && ~strcmp(dataset, '7')
                for eventIndex = 1:length(events.eventStart_L)
                    if events.groundReactionForceCorrection_L(eventIndex)
                        eventColor = [1, 0.7, 0];
                    else
                        eventColor = 'r';
                    end
                    patch([events.eventStart_L(eventIndex), events.eventStart_L(eventIndex), events.eventEnd_L(eventIndex), events.eventEnd_L(eventIndex)], [minimumValue, maximumValue, maximumValue, minimumValue], eventColor, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
                end
                for eventIndex = 1:length(events.eventStart_R)
                    if events.groundReactionForceCorrection_R(eventIndex)
                        eventColor = [0, 0.7, 1];
                    else
                        eventColor = 'b';
                    end
                    patch([events.eventStart_R(eventIndex), events.eventStart_R(eventIndex), events.eventEnd_R(eventIndex), events.eventEnd_R(eventIndex)], [minimumValue, maximumValue, maximumValue, minimumValue], eventColor, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
                end
            else
                for eventIndex = 1:length(events.eventStart)
                    if events.groundReactionForceCorrection(eventIndex)
                        eventColor = [1, 0.7, 0];
                    else
                        eventColor = 'r';
                    end
                    patch([events.eventStart(eventIndex), events.eventStart(eventIndex), events.eventEnd(eventIndex), events.eventEnd(eventIndex)], [minimumValue, maximumValue, maximumValue, minimumValue], eventColor, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
                end
            end
        subplot(3, 1, 2);
            plot(time, force.groundReactionForceY_L, 'r-');
            hold on;
            plot(time, force.groundReactionForceY_R, 'b-');
            title('Ground reaction force Y_L (red) / Y_R (blue)');
            xlabel('Time in s');
            ylabel('Force in N');
            grid on;
            maximumValue = max(max(force.groundReactionForceY_L), max(force.groundReactionForceY_R));
            minimumValue = min(min(force.groundReactionForceY_L), min(force.groundReactionForceY_R));
            if ~strcmp(dataset, '6') && ~strcmp(dataset, '7')
                for eventIndex = 1:length(events.eventStart_L)
                    if events.groundReactionForceCorrection_L(eventIndex)
                        eventColor = [1, 0.7, 0];
                    else
                        eventColor = 'r';
                    end
                    patch([events.eventStart_L(eventIndex), events.eventStart_L(eventIndex), events.eventEnd_L(eventIndex), events.eventEnd_L(eventIndex)], [minimumValue, maximumValue, maximumValue, minimumValue], eventColor, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
                end
                for eventIndex = 1:length(events.eventStart_R)
                    if events.groundReactionForceCorrection_R(eventIndex)
                        eventColor = [0, 0.7, 1];
                    else
                        eventColor = 'b';
                    end
                    patch([events.eventStart_R(eventIndex), events.eventStart_R(eventIndex), events.eventEnd_R(eventIndex), events.eventEnd_R(eventIndex)], [minimumValue, maximumValue, maximumValue, minimumValue], eventColor, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
                end
            else
                for eventIndex = 1:length(events.eventStart)
                    if events.groundReactionForceCorrection(eventIndex)
                        eventColor = [1, 0.7, 0];
                    else
                        eventColor = 'r';
                    end
                    patch([events.eventStart(eventIndex), events.eventStart(eventIndex), events.eventEnd(eventIndex), events.eventEnd(eventIndex)], [minimumValue, maximumValue, maximumValue, minimumValue], eventColor, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
                end
            end
        subplot(3, 1, 3);
            if isfield(force, 'groundReactionForceZ_L')
                plot(time, force.groundReactionForceZ, 'Color', [0.7, 0.7, 0.7]);
                hold on;
                plot(time, force.groundReactionForceZ_L, 'r-');
                plot(time, force.groundReactionForceZ_R, 'b-');
                title('Ground reaction force Z (gray) / Z_L (red) / Z_R (blue)');
            else
                plot(time, force.groundReactionForceZ, 'k-');
                title('Ground reaction force Z');
            end
            xlabel('Time in s');
            ylabel('Force in N');
            grid on;
            maximumValue = max(force.groundReactionForceZ);
            minimumValue = min(force.groundReactionForceZ);
            if ~strcmp(dataset, '6') && ~strcmp(dataset, '7')
                for eventIndex = 1:length(events.eventStart_L)
                    if events.groundReactionForceCorrection_L(eventIndex)
                        eventColor = [1, 0.7, 0];
                    else
                        eventColor = 'r';
                    end
                    patch([events.eventStart_L(eventIndex), events.eventStart_L(eventIndex), events.eventEnd_L(eventIndex), events.eventEnd_L(eventIndex)], [minimumValue, maximumValue, maximumValue, minimumValue], eventColor, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
                end
                for eventIndex = 1:length(events.eventStart_R)
                    if events.groundReactionForceCorrection_R(eventIndex)
                        eventColor = [0, 0.7, 1];
                    else
                        eventColor = 'b';
                    end
                    patch([events.eventStart_R(eventIndex), events.eventStart_R(eventIndex), events.eventEnd_R(eventIndex), events.eventEnd_R(eventIndex)], [minimumValue, maximumValue, maximumValue, minimumValue], eventColor, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
                end
            else
                for eventIndex = 1:length(events.eventStart)
                    if events.groundReactionForceCorrection(eventIndex)
                        eventColor = [1, 0.7, 0];
                    else
                        eventColor = 'r';
                    end
                    patch([events.eventStart(eventIndex), events.eventStart(eventIndex), events.eventEnd(eventIndex), events.eventEnd(eventIndex)], [minimumValue, maximumValue, maximumValue, minimumValue], eventColor, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
                end
            end
        suplabel([dataset, ' ', name, ' - HuMoD Database']);
        
        % Save figure
        saveTightFigure(visualization, [savePath, subject, filesep, dataset, '.png']);
        fprintf('STATUS: Figure for dataset %s %s was saved.\n', subject, dataset);
        close all;

    end
end