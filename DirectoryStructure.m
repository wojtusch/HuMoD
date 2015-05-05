% ------------------------------------------------------
% This script creates a directory structure for the computational routines
% of the HuMoD Database.
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
path = pwd;
subjects = {
    'A',...
    'B'...
};
directories = {
    % Directories for each subject
    [path, filesep, '<subject>'], 'Put datasets here'; ...
    % Directory for raw data
    [path, filesep, '<subject>', filesep, 'Raw data'], 'Put raw data here'; ...
    % Directory for force visualization
    [path, filesep, 'Force', filesep, '<subject>'], 'Force visualization will be saved here'; ...
    % Directory for motion visualization
    [path, filesep, 'Motion', filesep, '<subject>'], 'Motion visualization will be saved here'; ...
    % Directory for muscle visualization
    [path, filesep, 'Muscle', filesep, '<subject>'], 'Muscle visualization will be saved here'
};

for subjectIndex = 1:length(subjects)
    for directoryIndex = 1:size(directories, 1)

        % Set parameters
        subject = subjects{subjectIndex};
        directory = directories{directoryIndex, 1};
        directory = strrep(directory, '<subject>', subject);

        % Create directories and hints
        if ~exist(directory, 'dir')
            status = mkdir(directory);
            if ~status
                fprintf('ERROR: %s could not be created.\n', directory);
            else
                hint = fopen([directory, filesep, directories{directoryIndex, 2}], 'w' );
                fclose(hint);
                fprintf('STATUS: %s was created.\n', directory);
            end
        else
            fprintf('WARNING: %s already exists.\n', directory);
        end
        
    end
end