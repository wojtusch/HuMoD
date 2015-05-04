% ------------------------------------------------------
% This script matches the ground reaction forces for left and right side.
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
    '8' ...
};
subjects = {
    'A',...
    'B'...
};

% Add functions to search path
addpath('Scripts');

for subjectIndex = 1:length(subjects)
    for datasetIndex = 1:length(datasets)

        % Set parameters
        dataset = datasets{datasetIndex};
        subject = subjects{subjectIndex};
        fprintf('STATUS: Processing dataset %s %s.\n', subject, dataset);

        if ~strcmp(dataset, '6') && ~strcmp(dataset, '7')    
            
            % Load data file
            file = getFile(subject, dataset);
            if file
                variables = load(file);
                if isfield(variables, 'force') && isfield(variables, 'events')
                    force = variables.force;
                    events = variables.events;
                else
                    fprintf('WARNING: No matching data found!\n');
                    continue;
                end
            else
                fprintf('WARNING: No matching data file found!\n');
                continue;
            end
            
            % Compute contact phases
            contactPhase_L = zeros(1, force.frames);
            for eventIndex = 1:length(events.eventStart_L)
                eventStart = round(events.eventStart_L(eventIndex) * force.frameRate);
                eventEnd = round(events.eventEnd_L(eventIndex) * force.frameRate);
                contactPhase_L(eventStart:eventEnd) = 1;
            end
            contactPhase_R = zeros(1, force.frames);
            for eventIndex = 1:length(events.eventStart_R)
                eventStart = round(events.eventStart_R(eventIndex) * force.frameRate);
                eventEnd = round(events.eventEnd_R(eventIndex) * force.frameRate);
                contactPhase_R(eventStart:eventEnd) = 1;
            end

            % Match ground reaction forces
            groundReactionForceY_L = force.groundReactionForceY_L;
            groundReactionForceY_R = force.groundReactionForceY_R;
            lowerLimit = min(50, 3 * max(groundReactionForceY_L([1000:8000, (force.frames - 7999):(force.frames - 999)])));
            for eventIndex = 1:length(events.eventStart_L)
                singleSupport_L = logical(bitand(contactPhase_L, ~contactPhase_R));
                if events.groundReactionForceCorrection_L(eventIndex) || (median(groundReactionForceY_R(singleSupport_L)) > lowerLimit)
                    groundReactionForceY_L(singleSupport_L) = groundReactionForceY_L(singleSupport_L) + groundReactionForceY_R(singleSupport_L);
                    groundReactionForceY_R(singleSupport_L) = 0;
                end
            end
            lowerLimit = min(50, 3 * max(groundReactionForceY_R([1000:8000, (force.frames - 7999):(force.frames - 999)])));
            for eventIndex = 1:length(events.eventStart_R)
                singleSupport_R = logical(bitand(contactPhase_R, ~contactPhase_L));
                if events.groundReactionForceCorrection_R(eventIndex) || (median(groundReactionForceY_L(singleSupport_R)) > lowerLimit)
                    groundReactionForceY_R(singleSupport_R) = groundReactionForceY_L(singleSupport_R) + groundReactionForceY_R(singleSupport_R);
                    groundReactionForceY_L(singleSupport_R) = 0;
                end
            end

            % Save processed data
            fprintf('STATUS: Saving dataset %s %s.\n', subject, dataset);
            force.groundReactionForceY_L = groundReactionForceY_L;
            force.groundReactionForceY_R = groundReactionForceY_R;
            events.contactPhase_L = contactPhase_L;
            events.contactPhase_R = contactPhase_R;
            variables.force = force;
            variables.events = events;
            save(file, '-struct', 'variables');
        
        end
    end
end
