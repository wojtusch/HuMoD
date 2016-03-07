% ------------------------------------------------------
% This script smooths the measured ground reaction forces and separates the
% forces for left and right side. For the separation of the forces during
% double support, the method given by [Villeger2014] is used.
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
smoothingWindowSize = 5;            % Window size in the nunber of values for the Savitzky-Golay smoothing filter [Savitzky1964]
firstDerivativeWindowSize = 201;    % Window size in the number of values for the Savitzky-Golay first derivation filter [Savitzky1964]
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
    
    % Load parameter file
    file = [getPath, 'Models', filesep, subject, filesep, 'Parameters.mat'];
    if exist(file, 'file')
        parameters = load(file);
    else
        fprintf('ERROR: No matching parameter file found!\n');
        return;
    end
    
    for datasetIndex = 1:length(datasets)

        % Set parameters
        dataset = datasets{datasetIndex};
        fprintf('STATUS: Processing dataset %s %s.\n', subject, dataset);

        % Load data file
        file = getFile(subject, dataset);
        if file
            variables = load(file);
            if isfield(variables, 'motion') && isfield(variables, 'ground') && isfield(variables, 'force') && isfield(variables, 'events')
                motion = variables.motion;
                ground = variables.ground;
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
        
        % Estimate subject velocity from treadmill velocity and relative
        % velocity between the ASIS markers and the ground
        filteredTreadmillVelocity = force.treadmillVelocity;
        relativePosition = zeros(3, motion.frames);
        treadmillVelocity = zeros(1, motion.frames);
        for dataIndex = 1:motion.frames
            vectorASIS_L = getMarker('ASIS_L', 'marker', dataIndex);
            vectorASIS_R = getMarker('ASIS_R', 'marker', dataIndex);
            relativePosition(:, dataIndex) = (vectorASIS_L + (vectorASIS_R - vectorASIS_L) / 2) / 1000;
            treadmillVelocity(dataIndex) = mean(filteredTreadmillVelocity((2 * dataIndex - 1):(2 * dataIndex)));
        end
        relativeVelocity = savitzkyGolayFilter(relativePosition(1, :), '1st derivative', firstDerivativeWindowSize, (1 / motion.frameRate));
        subjectVelocity = treadmillVelocity + relativeVelocity;
        clear vectorASIS_L vectorASIS_R
        
        % Separate forces for left and right side
        if ~strcmp(dataset, '6') && ~strcmp(dataset, '7') && ~strcmp(dataset, '8')
            FX = force.grfX;
            FY_L = force.grfY_L;
            FY_R = force.grfY_R;
            FZ = force.grfZ;

            % Separate forces during single support phase
            grfX_L = zeros(1, force.frames);
            grfX_R = zeros(1, force.frames);
            grfY_L = zeros(1, force.frames);
            grfY_R = zeros(1, force.frames);
            grfZ_L = zeros(1, force.frames);
            grfZ_R = zeros(1, force.frames);
            singleSupport_L = logical(bitand(events.contactPhase_L, ~events.contactPhase_R));
            grfX_L(singleSupport_L) = FX(singleSupport_L);
            grfY_L(singleSupport_L) = FY_L(singleSupport_L);
            grfZ_L(singleSupport_L) = FZ(singleSupport_L);
            singleSupport_R = logical(bitand(~events.contactPhase_L, events.contactPhase_R));
            grfX_R(singleSupport_R) = FX(singleSupport_R);
            grfY_R(singleSupport_R) = FY_R(singleSupport_R);
            grfZ_R(singleSupport_R) = FZ(singleSupport_R);

            % Separate forces during double support phase according to [Villeger2014]
            doubleSupport = logical(bitand(events.contactPhase_L, events.contactPhase_R));
            if any(doubleSupport)
                difference = diff([0, doubleSupport, 0]);
                doubleSupportStart = find(difference > 0);
                doubleSupportEnd = find(difference < 0) - 1;
                doubleSupportDuration = doubleSupportEnd - doubleSupportStart + 1;
                dFX_dt = savitzkyGolayFilter(force.grfX, '1st derivative', firstDerivativeWindowSize, (1 / force.frameRate));
                dFY_L_dt = savitzkyGolayFilter(force.grfY_L, '1st derivative', firstDerivativeWindowSize, (1 / force.frameRate));
                dFY_R_dt = savitzkyGolayFilter(force.grfY_R, '1st derivative', firstDerivativeWindowSize, (1 / force.frameRate));
                dFZ_dt = savitzkyGolayFilter(force.grfZ, '1st derivative', firstDerivativeWindowSize, (1 / force.frameRate));
                for doubleSupportIndex = 1:length(doubleSupportStart)
                    % Find involved events and force vectors
                    eventIndex_L = find(bitand((events.eventStart_L <= (doubleSupportEnd(doubleSupportIndex) / force.frameRate)), (events.eventEnd_L >= (doubleSupportStart(doubleSupportIndex) / force.frameRate))));
                    eventIndex_R = find(bitand((events.eventStart_R <= (doubleSupportEnd(doubleSupportIndex) / force.frameRate)), (events.eventEnd_R >= (doubleSupportStart(doubleSupportIndex) / force.frameRate))));
                    startIndex_L = round(events.eventStart_L(eventIndex_L) * force.frameRate);
                    endIndex_L = round(events.eventEnd_L(eventIndex_L) * force.frameRate);
                    startIndex_R = round(events.eventStart_R(eventIndex_R) * force.frameRate);
                    endIndex_R = round(events.eventEnd_R(eventIndex_R) * force.frameRate);
                    if events.eventStart_L(eventIndex_L) < events.eventStart_R(eventIndex_R)
                        FX1 = FX(startIndex_L:doubleSupportStart(doubleSupportIndex));
                        FX3 = FX(doubleSupportEnd(doubleSupportIndex):endIndex_R);
                        FY1 = FY_L(startIndex_L:doubleSupportStart(doubleSupportIndex));
                        FY3 = FY_R(doubleSupportEnd(doubleSupportIndex):endIndex_R);
                        FZ1 = FZ(startIndex_L:doubleSupportStart(doubleSupportIndex));
                        FZ3 = FZ(doubleSupportEnd(doubleSupportIndex):endIndex_R);
                    else
                        FX1 = FX(startIndex_R:doubleSupportStart(doubleSupportIndex));
                        FX3 = FX(doubleSupportEnd(doubleSupportIndex):endIndex_L);
                        FY1 = FY_R(startIndex_R:doubleSupportStart(doubleSupportIndex));
                        FY3 = FY_L(doubleSupportEnd(doubleSupportIndex):endIndex_L);
                        FZ1 = FZ(startIndex_R:doubleSupportStart(doubleSupportIndex));
                        FZ3 = FZ(doubleSupportEnd(doubleSupportIndex):endIndex_L);
                    end
                    
                    % Calculate shape coefficients
                    t = 0:(1 / force.frameRate):((doubleSupportDuration(doubleSupportIndex) - 1) / force.frameRate);
                    T_ds = doubleSupportDuration(doubleSupportIndex) / (2 * force.frameRate);
                    V_F = median(subjectVelocity(round(doubleSupportStart(doubleSupportIndex) * motion.frameRate / force.frameRate):round(doubleSupportEnd(doubleSupportIndex) * motion.frameRate / force.frameRate)));
                    FX_i = abs(FX1(end)) / parameters.bodyMass;
                    FX_slope = abs(dFX_dt(doubleSupportStart(doubleSupportIndex))) / parameters.bodyMass;
                    FX_max = max(max(abs(FX1)), max(abs(FX3))) / parameters.bodyMass;
                    FY_i = abs(FY1(end)) / parameters.bodyMass;
                    if events.eventStart_L(eventIndex_L) < events.eventStart_R(eventIndex_R)
                        FY_slope = abs(dFY_L_dt(doubleSupportStart(doubleSupportIndex))) / parameters.bodyMass;
                    else
                        FY_slope = abs(dFY_R_dt(doubleSupportStart(doubleSupportIndex))) / parameters.bodyMass;
                    end
                    FY_max = max(max(abs(FY1)), max(abs(FY3))) / parameters.bodyMass;
                    FZ_slope = abs(dFZ_dt(doubleSupportStart(doubleSupportIndex))) / parameters.bodyMass;
                    FZ_max = max(max(abs(FZ1)), max(abs(FZ3))) / parameters.bodyMass;
                    SX = 0.283 - 1.248 * 2 * T_ds - 0.219 * FX_i - 0.003 * FX_slope + 0.04 * FX_max + 0.03 * FY_i + 0.002 * FY_slope + 0.034 * FY_max;
                    SY = -0.398 + 0.149 * V_F + 1.064 * 2 * T_ds + 0.043 * FX_i - 0.014 * FX_max + 0.036 * FZ_max + 0.011 * FY_i - 0.001 * FY_slope - 0.026 * FY_max;
                    SZ = 0.691 - 0.313 * V_F - 2.867 * 2 * T_ds - 0.121 * FX_i + 0.083 * FX_max + 0.007 * FZ_slope + 0.022 * FY_i - 0.002 * FY_slope;
                    
                    % Estimate forces during double support phase
                    FX21 = FX1(end) * (exp(SX^2) * exp(-((t - (SX * T_ds)) / T_ds).^2) - (0.5 * exp(SX^2) * exp(-(2 - SX)^2)) * t / T_ds);
                    FX22 = FX(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex)) - FX21;
                    if events.grfCorrection_L(eventIndex_L) || events.grfCorrection_R(eventIndex_R)
                        FY21 = FY1(end) * (exp(SY^2) * exp(-((t - (SY * T_ds)) / T_ds).^2) - (0.5 * exp(SY^2) * exp(-(2 - SY)^2)) * t / T_ds);
                        FY22 = (FY_L(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex)) + FY_R(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex))) - FY21;
                    else
                        if events.eventStart_L(eventIndex_L) < events.eventStart_R(eventIndex_R)
                            FY21 = force.grfY_L(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex));
                            FY22 = force.grfY_R(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex));
                        else
                            FY21 = FY_R(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex));
                            FY22 = FY_L(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex));
                        end   
                    end
                    FZ21 = FZ1(end) * (exp(SZ^2) * exp(-((t - (SZ * T_ds)) / T_ds).^2) - (0.5 * exp(SZ^2) * exp(-(2 - SZ)^2)) * t / T_ds);
                    FZ22 = FZ(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex)) - FZ21;
                    
                    % Combine measured during single support phase and estimated forces during double support phase
                    if events.eventStart_L(eventIndex_L) < events.eventStart_R(eventIndex_R)
                        grfX_L(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex)) = FX21;
                        grfX_R(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex)) = FX22;
                        grfY_L(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex)) = FY21;
                        grfY_R(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex)) = FY22;
                        grfZ_L(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex)) = FZ21;
                        grfZ_R(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex)) = FZ22;
                    else
                        grfX_R(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex)) = FX21;
                        grfX_L(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex)) = FX22;
                        grfY_R(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex)) = FY21;
                        grfY_L(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex)) = FY22;
                        grfZ_R(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex)) = FZ21;
                        grfZ_L(doubleSupportStart(doubleSupportIndex):doubleSupportEnd(doubleSupportIndex)) = FZ22;
                    end
                end
            end
            force.grfX = FX;
            force.grfX_L = grfX_L;
            force.grfX_R = grfX_R;
            force.grfY_L = (grfY_L + abs(grfY_L)) / 2;
            force.grfY_R = (grfY_R + abs(grfY_R)) / 2;
            force.grfZ = FZ;
            force.grfZ_L = grfZ_L;
            force.grfZ_R = grfZ_R;
        elseif strcmp(dataset, '7')
            FX = force.grfX;
            FY_L = force.grfY_L;
            FY_R = force.grfY_R;
            FZ = force.grfZ;
            
            % Separate forces during kicking event
            grfX_L = nan(1, force.frames);
            grfX_R = nan(1, force.frames);
            grfY_L = FY_L;
            grfY_R = FY_R;
            grfZ_L = nan(1, force.frames);
            grfZ_R = nan(1, force.frames);
            for eventIndex = 1:length(events.eventStart)
                eventStart = round(events.eventStart(eventIndex) * force.frameRate);
                eventEnd = round(events.eventEnd(eventIndex) * force.frameRate);
                vectorLM_L = getAverageMarker('LM_L', 'surface', round(eventStart * motion.frameRate / force.frameRate), round(eventEnd * motion.frameRate / force.frameRate));
                vectorLM_R = getAverageMarker('LM_R', 'surface', round(eventStart * motion.frameRate / force.frameRate), round(eventEnd * motion.frameRate / force.frameRate));
                if vectorLM_L(2) < vectorLM_R(2)
                    grfX_L(eventStart:eventEnd) = FX(eventStart:eventEnd);
                    grfX_R(eventStart:eventEnd) = 0;
                    grfY_L(eventStart:eventEnd) = FY_L(eventStart:eventEnd) + FY_R(eventStart:eventEnd);
                    grfY_R(eventStart:eventEnd) = 0;
                    grfZ_L(eventStart:eventEnd) = FZ(eventStart:eventEnd);
                    grfZ_R(eventStart:eventEnd) = 0;
                else
                    grfX_L(eventStart:eventEnd) = 0;
                    grfX_R(eventStart:eventEnd) = FX(eventStart:eventEnd);
                    grfY_L(eventStart:eventEnd) = 0;
                    grfY_R(eventStart:eventEnd) = FY_L(eventStart:eventEnd) + FY_R(eventStart:eventEnd);
                    grfZ_L(eventStart:eventEnd) = 0;
                    grfZ_R(eventStart:eventEnd) = FZ(eventStart:eventEnd);
                end
            end
            force.grfX = FX;
            force.grfX_L = grfX_L;
            force.grfX_R = grfX_R;
            force.grfY_L = (grfY_L + abs(grfY_L)) / 2;
            force.grfY_R = (grfY_R + abs(grfY_R)) / 2;
            force.grfZ = FZ;
            force.grfZ_L = grfZ_L;
            force.grfZ_R = grfZ_R;
        end
        
        % Save processed data
        fprintf('STATUS: Saving dataset %s %s.\n', subject, dataset);
        motion.subjectVelocity = subjectVelocity;
        force = orderfields(force);
        variables.force = force;
        motion = orderfields(motion);
        variables.motion = motion;
        save(file, '-struct', 'variables');

    end
end
