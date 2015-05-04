% ------------------------------------------------------
% This script estimates the center of pressure position from the measured
% force sensor data.
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
firstDerivativeWindowSize = 101;    % Window size in the number of values for the Savitzky-Golay first derivation filter [Savitzky1964]
filterHalfOrder = 3;                % Half order of the zero-phase high-pass and low-pass filters
filterCutOff = 20;                  % Cut-off frequency of the zero-phase low-pass filter
polynomialOrderX = 4;               % Polynomial order of the position estimation in x dimension
polynomialOrderZ = 5;               % Polynomial order of the position estimation in z dimension
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

% Add functions to search path
addpath('Scripts');

for subjectIndex = 1:length(subjects)
    for datasetIndex = 1:length(datasets)

        % Set parameters
        dataset = datasets{datasetIndex};
        subject = subjects{subjectIndex};
        fprintf('STATUS: Processing dataset %s %s.\n', subject, dataset);

        % Load data file
        file = getFile(subject, dataset);
        if file
            variables = load(file);
            if isfield(variables, 'force') && isfield(variables, 'events') && isfield(variables, 'motion') && isfield(variables, 'ground')
                force = variables.force;
                events = variables.events;
                motion = variables.motion;
                ground = variables.ground;
            else
                fprintf('WARNING: No matching data found!\n');
                continue;
            end
        else
            fprintf('WARNING: No matching data file found!\n');
            continue;
        end
        
        % Estimate the X and Z coordinates of the center of pressure for
        % single and double support in uncorrected events and for single
        % support in corrected events
        groundPosition = ground.groundPosition;
        groundNormal = ground.groundNormal;
        FY_L1 = force.forceSensorY_L1;
        FY_L2 = force.forceSensorY_L2;
        FY_L3 = force.forceSensorY_L3;
        FY_L4 = force.forceSensorY_L4;
        FY_R1 = force.forceSensorY_R1;
        FY_R2 = force.forceSensorY_R2;
        FY_R3 = force.forceSensorY_R3;
        FY_R4 = force.forceSensorY_R4;
        vectorL1 = getSensor('L1');
        vectorL2 = getSensor('L2');
        vectorL3 = getSensor('L3');
        vectorL4 = getSensor('L4');
        vectorR1 = getSensor('R1');
        vectorR2 = getSensor('R2');
        vectorR3 = getSensor('R3');
        vectorR4 = getSensor('R4');
        vectorS1 = getSensor('S1');
        vectorS2 = getSensor('S2');
        vectorS3 = getSensor('S3');
        vectorS4 = getSensor('S4');
        if ~strcmp(dataset, '6') && ~strcmp(dataset, '7') && ~strcmp(dataset, '8')
            correction_L = zeros(1, length(events.eventStart_L));
            correction_R = zeros(1, length(events.eventStart_L));
        elseif strcmp(dataset, '8')
            correction = zeros(1, max(length(events.eventStart_L), length(events.eventStart_R)));
        else
            correction = zeros(1, length(events.eventStart));
        end
        if ~strcmp(dataset, '6') && ~strcmp(dataset, '7') && ~strcmp(dataset, '8')
            minimumForce = 0.1;
            FY_L = force.groundReactionForceY_L;
            FY_R = force.groundReactionForceY_R;
            TX_L = nan(1, force.frames);
            TX_R = nan(1, force.frames);
            TZ_L = nan(1, force.frames);
            TZ_R = nan(1, force.frames);
            COPX_L = nan(1, force.frames);
            COPX_R = nan(1, force.frames);
            COPZ_L = nan(1, force.frames);
            COPZ_R = nan(1, force.frames);
            singleSupport_L = logical(bitand(events.contactPhase_L, ~events.contactPhase_R));
            singleSupport_R = logical(bitand(~events.contactPhase_L, events.contactPhase_R));
            for eventIndex = 1:length(events.eventStart_L)
                if ~events.groundReactionForceCorrection_L(eventIndex)
                    eventStart = round(events.eventStart_L(eventIndex) * force.frameRate);
                    eventEnd = round(events.eventEnd_L(eventIndex) * force.frameRate);
                    TX_L(eventStart:eventEnd) = (FY_L1(eventStart:eventEnd) * vectorL1(3) + FY_L2(eventStart:eventEnd) * vectorL2(3) + FY_L3(eventStart:eventEnd) * vectorL3(3) + FY_L4(eventStart:eventEnd) * vectorL4(3)) / 1000;
                    TZ_L(eventStart:eventEnd) = -(FY_L1(eventStart:eventEnd) * vectorL1(1) + FY_L2(eventStart:eventEnd) * vectorL2(1) + FY_L3(eventStart:eventEnd) * vectorL3(1) + FY_L4(eventStart:eventEnd) * vectorL4(1)) / 1000;
                    COPX_L(eventStart:eventEnd) = -TZ_L(eventStart:eventEnd) ./ max(minimumForce, FY_L(eventStart:eventEnd)) * 1000;
                    COPZ_L(eventStart:eventEnd) = TX_L(eventStart:eventEnd) ./ max(minimumForce, FY_L(eventStart:eventEnd)) * 1000;
                else
                    eventStart = round(events.eventStart_L(eventIndex) * force.frameRate);
                    eventEnd = round(events.eventEnd_L(eventIndex) * force.frameRate);
                    difference = diff([0, singleSupport_L(eventStart:eventEnd), 0]);
                    singleSupportStart = find(difference > 0, 1, 'first');
                    singleSupportEnd = find(difference < 0, 1, 'first') - 1;
                    eventEnd = eventStart + singleSupportEnd - 1;
                    eventStart = eventStart + singleSupportStart - 1;
                    TX_L(eventStart:eventEnd) = (FY_L1(eventStart:eventEnd) * vectorL1(3) + FY_L2(eventStart:eventEnd) * vectorL2(3) + FY_L3(eventStart:eventEnd) * vectorL3(3) + FY_L4(eventStart:eventEnd) * vectorL4(3)) / 1000 + ...
                        (FY_R1(eventStart:eventEnd) * vectorR1(3) + FY_R2(eventStart:eventEnd) * vectorR2(3) + FY_R3(eventStart:eventEnd) * vectorR3(3) + FY_R4(eventStart:eventEnd) * vectorR4(3)) / 1000;
                    TZ_L(eventStart:eventEnd) = -(FY_L1(eventStart:eventEnd) * vectorL1(1) + FY_L2(eventStart:eventEnd) * vectorL2(1) + FY_L3(eventStart:eventEnd) * vectorL3(1) + FY_L4(eventStart:eventEnd) * vectorL4(1)) / 1000 - ...
                        (FY_R1(eventStart:eventEnd) * vectorR1(1) + FY_R2(eventStart:eventEnd) * vectorR2(1) + FY_R3(eventStart:eventEnd) * vectorR3(1) + FY_R4(eventStart:eventEnd) * vectorR4(1)) / 1000;
                    COPX_L(eventStart:eventEnd) = -TZ_L(eventStart:eventEnd) ./ max(minimumForce, (FY_L(eventStart:eventEnd) + FY_R(eventStart:eventEnd))) * 1000;
                    COPZ_L(eventStart:eventEnd) = TX_L(eventStart:eventEnd) ./ max(minimumForce, (FY_L(eventStart:eventEnd) + FY_R(eventStart:eventEnd))) * 1000;
                end
            end
            for eventIndex = 1:length(events.eventStart_R)
                if ~events.groundReactionForceCorrection_R(eventIndex)
                    eventStart = round(events.eventStart_R(eventIndex) * force.frameRate);
                    eventEnd = round(events.eventEnd_R(eventIndex) * force.frameRate);
                    TX_R(eventStart:eventEnd) = (FY_R1(eventStart:eventEnd) * vectorR1(3) + FY_R2(eventStart:eventEnd) * vectorR2(3) + FY_R3(eventStart:eventEnd) * vectorR3(3) + FY_R4(eventStart:eventEnd) * vectorR4(3)) / 1000;
                    TZ_R(eventStart:eventEnd) = -(FY_R1(eventStart:eventEnd) * vectorR1(1) + FY_R2(eventStart:eventEnd) * vectorR2(1) + FY_R3(eventStart:eventEnd) * vectorR3(1) + FY_R4(eventStart:eventEnd) * vectorR4(1)) / 1000;
                    COPX_R(eventStart:eventEnd) = -TZ_R(eventStart:eventEnd) ./ max(minimumForce, FY_R(eventStart:eventEnd)) * 1000;
                    COPZ_R(eventStart:eventEnd) = TX_R(eventStart:eventEnd) ./ max(minimumForce, FY_R(eventStart:eventEnd)) * 1000;
                else
                    eventStart = round(events.eventStart_R(eventIndex) * force.frameRate);
                    eventEnd = round(events.eventEnd_R(eventIndex) * force.frameRate);
                    difference = diff([0, singleSupport_R(eventStart:eventEnd), 0]);
                    singleSupportStart = find(difference > 0, 1, 'first');
                    singleSupportEnd = find(difference < 0, 1, 'first') - 1;
                    eventEnd = eventStart + singleSupportEnd - 1;
                    eventStart = eventStart + singleSupportStart - 1;
                    TX_R(eventStart:eventEnd) = (FY_L1(eventStart:eventEnd) * vectorL1(3) + FY_L2(eventStart:eventEnd) * vectorL2(3) + FY_L3(eventStart:eventEnd) * vectorL3(3) + FY_L4(eventStart:eventEnd) * vectorL4(3)) / 1000 + ...
                        (FY_R1(eventStart:eventEnd) * vectorR1(3) + FY_R2(eventStart:eventEnd) * vectorR2(3) + FY_R3(eventStart:eventEnd) * vectorR3(3) + FY_R4(eventStart:eventEnd) * vectorR4(3)) / 1000;
                    TZ_R(eventStart:eventEnd) = -(FY_L1(eventStart:eventEnd) * vectorL1(1) + FY_L2(eventStart:eventEnd) * vectorL2(1) + FY_L3(eventStart:eventEnd) * vectorL3(1) + FY_L4(eventStart:eventEnd) * vectorL4(1)) / 1000 - ...
                        (FY_R1(eventStart:eventEnd) * vectorR1(1) + FY_R2(eventStart:eventEnd) * vectorR2(1) + FY_R3(eventStart:eventEnd) * vectorR3(1) + FY_R4(eventStart:eventEnd) * vectorR4(1)) / 1000;
                    COPX_R(eventStart:eventEnd) = -TZ_R(eventStart:eventEnd) ./ max(minimumForce, (FY_L(eventStart:eventEnd) + FY_R(eventStart:eventEnd))) * 1000;
                    COPZ_R(eventStart:eventEnd) = TX_R(eventStart:eventEnd) ./ max(minimumForce, (FY_L(eventStart:eventEnd) + FY_R(eventStart:eventEnd))) * 1000;
                end
            end
        else
            FY_L = force.groundReactionForceY_L;
            FY_R = force.groundReactionForceY_R;
            TX = nan(1, force.frames);
            TZ = nan(1, force.frames);
            COPX = nan(1, force.frames);
            COPZ = nan(1, force.frames);
            if ~strcmp(dataset, '8')
                indexEnd = length(events.eventStart);
            else
                indexEnd = length(events.eventStart_L);
            end
            for eventIndex = 1:indexEnd
                if ~strcmp(dataset, '8')
                    eventStart = round(events.eventStart(eventIndex) * force.frameRate);
                    eventEnd = round(events.eventEnd(eventIndex) * force.frameRate);
                else
                    eventStart = round(min([events.eventStart_L(eventIndex), events.eventStart_R(eventIndex)]) * force.frameRate);
                    eventEnd = round(max([events.eventEnd_L(eventIndex), events.eventEnd_R(eventIndex)]) * force.frameRate);
                end
                TX(eventStart:eventEnd) = (FY_L1(eventStart:eventEnd) * vectorL1(3) + FY_L2(eventStart:eventEnd) * vectorL2(3) + FY_L3(eventStart:eventEnd) * vectorL3(3) + FY_L4(eventStart:eventEnd) * vectorL4(3)) / 1000 + ...
                    (FY_R1(eventStart:eventEnd) * vectorR1(3) + FY_R2(eventStart:eventEnd) * vectorR2(3) + FY_R3(eventStart:eventEnd) * vectorR3(3) + FY_R4(eventStart:eventEnd) * vectorR4(3)) / 1000;
                TZ(eventStart:eventEnd) = -(FY_L1(eventStart:eventEnd) * vectorL1(1) + FY_L2(eventStart:eventEnd) * vectorL2(1) + FY_L3(eventStart:eventEnd) * vectorL3(1) + FY_L4(eventStart:eventEnd) * vectorL4(1)) / 1000 + ...
                    (FY_R1(eventStart:eventEnd) * vectorR1(1) + FY_R2(eventStart:eventEnd) * vectorR2(1) + FY_R3(eventStart:eventEnd) * vectorR3(1) + FY_R4(eventStart:eventEnd) * vectorR4(1)) / 1000;
                COPX(eventStart:eventEnd) = -TZ(eventStart:eventEnd) ./ (FY_L(eventStart:eventEnd) + FY_R(eventStart:eventEnd)) * 1000;
                COPZ(eventStart:eventEnd) = TX(eventStart:eventEnd) ./ (FY_L(eventStart:eventEnd) + FY_R(eventStart:eventEnd)) * 1000;
            end
        end
        
        % Limit the estimated X and Z coordinates of the center of pressure
        % to the foot dimensions, because of high error amplification at
        % low force sensor values at event start and end [Winter2009]
        if ~strcmp(dataset, '6') && ~strcmp(dataset, '7') && ~strcmp(dataset, '8')
            for forceDataIndex = 1:force.frames
                if mod(forceDataIndex, 2) == 0
                    motionDataIndex = round(forceDataIndex / 2);
                else
                    motionDataIndex = round((forceDataIndex + 1) / 2);
                end
                if ~isnan(COPX_L(forceDataIndex)) && ~isnan(COPZ_L(forceDataIndex))
                    vectorCAL_L = getMarker('CAL_L', 'surface', motionDataIndex);
                    vectorMT2_L = getMarker('MT2_L', 'surface', motionDataIndex);
                    vectorMT5_L = getMarker('MT5_L', 'surface', motionDataIndex);
                    vectorHAL_L = getMarker('HAL_L', 'surface', motionDataIndex);
                    footPolygonX = [vectorCAL_L(1), vectorMT5_L(1), vectorMT2_L(1), vectorHAL_L(1), vectorCAL_L(1)];
                    footPolygonZ = [vectorCAL_L(3), vectorMT5_L(3), vectorMT2_L(3), vectorHAL_L(3), vectorCAL_L(3)];
                    if ~inpolygon(COPX_L(forceDataIndex), COPZ_L(forceDataIndex), footPolygonX, footPolygonZ)
                        footPolygonArea = 0;
                        footPolygonCentroidX = 0;
                        footPolygonCentroidZ = 0;
                        for polygonIndex = 1:(length(footPolygonX) - 1)
                            footPolygonArea = footPolygonArea + (footPolygonX(polygonIndex) * footPolygonZ(polygonIndex + 1) - footPolygonX(polygonIndex + 1) * footPolygonZ(polygonIndex)) / 2;
                            footPolygonCentroidX = footPolygonCentroidX + (footPolygonX(polygonIndex) + footPolygonX(polygonIndex + 1)) * (footPolygonX(polygonIndex) * footPolygonZ(polygonIndex + 1) - footPolygonX(polygonIndex + 1) * footPolygonZ(polygonIndex));
                            footPolygonCentroidZ = footPolygonCentroidZ + (footPolygonZ(polygonIndex) + footPolygonZ(polygonIndex + 1)) * (footPolygonX(polygonIndex) * footPolygonZ(polygonIndex + 1) - footPolygonX(polygonIndex + 1) * footPolygonZ(polygonIndex));
                        end
                        footPolygonCentroidX = footPolygonCentroidX / (6 * footPolygonArea);
                        footPolygonCentroidZ = footPolygonCentroidZ / (6 * footPolygonArea);
                        for polygonIndex = 1:(length(footPolygonX) - 1)
                            [intersectionParameter, condition] = linsolve( ...
                                [footPolygonCentroidX - COPX_L(forceDataIndex), footPolygonX(polygonIndex) - footPolygonX(polygonIndex + 1); ...
                                footPolygonCentroidZ - COPZ_L(forceDataIndex), footPolygonZ(polygonIndex) - footPolygonZ(polygonIndex + 1)], ...
                                [footPolygonX(polygonIndex) - COPX_L(forceDataIndex); footPolygonZ(polygonIndex) - COPZ_L(forceDataIndex)] ...
                            );
                            if (intersectionParameter(1) >= 0) && (intersectionParameter(1) <= 1) && (intersectionParameter(2) >= 0) && (intersectionParameter(2) <= 1)
                                intersection = [COPX_L(forceDataIndex); COPZ_L(forceDataIndex)] + intersectionParameter(1) * [footPolygonCentroidX - COPX_L(forceDataIndex); footPolygonCentroidZ - COPZ_L(forceDataIndex)];
                                COPX_L(forceDataIndex) = intersection(1);
                                COPZ_L(forceDataIndex) = intersection(2);
                                break;
                            end
                        end
                    end
                end
                if ~isnan(COPX_R(forceDataIndex)) && ~isnan(COPZ_R(forceDataIndex))
                    vectorCAL_R = getMarker('CAL_R', 'surface', motionDataIndex);
                    vectorMT2_R = getMarker('MT2_R', 'surface', motionDataIndex);
                    vectorMT5_R = getMarker('MT5_R', 'surface', motionDataIndex);
                    vectorHAL_R = getMarker('HAL_R', 'surface', motionDataIndex);
                    footPolygonX = [vectorCAL_R(1), vectorMT5_R(1), vectorMT2_R(1), vectorHAL_R(1), vectorCAL_R(1)];
                    footPolygonZ = [vectorCAL_R(3), vectorMT5_R(3), vectorMT2_R(3), vectorHAL_R(3), vectorCAL_R(3)];
                    if ~inpolygon(COPX_R(forceDataIndex), COPZ_R(forceDataIndex), footPolygonX, footPolygonZ)
                        footPolygonArea = 0;
                        footPolygonCentroidX = 0;
                        footPolygonCentroidZ = 0;
                        for polygonIndex = 1:(length(footPolygonX) - 1)
                            footPolygonArea = footPolygonArea + (footPolygonX(polygonIndex) * footPolygonZ(polygonIndex + 1) - footPolygonX(polygonIndex + 1) * footPolygonZ(polygonIndex)) / 2;
                            footPolygonCentroidX = footPolygonCentroidX + (footPolygonX(polygonIndex) + footPolygonX(polygonIndex + 1)) * (footPolygonX(polygonIndex) * footPolygonZ(polygonIndex + 1) - footPolygonX(polygonIndex + 1) * footPolygonZ(polygonIndex));
                            footPolygonCentroidZ = footPolygonCentroidZ + (footPolygonZ(polygonIndex) + footPolygonZ(polygonIndex + 1)) * (footPolygonX(polygonIndex) * footPolygonZ(polygonIndex + 1) - footPolygonX(polygonIndex + 1) * footPolygonZ(polygonIndex));
                        end
                        footPolygonCentroidX = footPolygonCentroidX / (6 * footPolygonArea);
                        footPolygonCentroidZ = footPolygonCentroidZ / (6 * footPolygonArea);
                        for polygonIndex = 1:(length(footPolygonX) - 1)
                            [intersectionParameter, condition] = linsolve( ...
                                [footPolygonCentroidX - COPX_R(forceDataIndex), footPolygonX(polygonIndex) - footPolygonX(polygonIndex + 1); ...
                                footPolygonCentroidZ - COPZ_R(forceDataIndex), footPolygonZ(polygonIndex) - footPolygonZ(polygonIndex + 1)], ...
                                [footPolygonX(polygonIndex) - COPX_R(forceDataIndex); footPolygonZ(polygonIndex) - COPZ_R(forceDataIndex)] ...
                            );
                            if (intersectionParameter(1) >= 0) && (intersectionParameter(1) <= 1) && (intersectionParameter(2) >= 0) && (intersectionParameter(2) <= 1)
                                intersection = [COPX_R(forceDataIndex); COPZ_R(forceDataIndex)] + intersectionParameter(1) * [footPolygonCentroidX - COPX_R(forceDataIndex); footPolygonCentroidZ - COPZ_R(forceDataIndex)];
                                COPX_R(forceDataIndex) = intersection(1);
                                COPZ_R(forceDataIndex) = intersection(2);
                                break;
                            end
                        end
                    end
                end
            end
        end
        
        % Compensate wrong center of pressure trajectories at event start
        % and end, because of high error amplification at low force sensor
        % values at event start and end [Winter2009]
        if ~strcmp(dataset, '6') && ~strcmp(dataset, '7') && ~strcmp(dataset, '8')
            motionSupportValues = 5;
            forceSupportValues = round(force.frameRate * motionSupportValues / motion.frameRate);
            evaluationRatio = 0.1;
            regressionRatio1 = 0.3;
            regressionRatio2 = 0.1;
            velocityLimit = 500;
            for eventIndex = 1:length(events.eventStart_L)
                if ~events.groundReactionForceCorrection_L(eventIndex)
                    forceEventStart = round(events.eventStart_L(eventIndex) * force.frameRate);
                    forceEventEnd = round(events.eventEnd_L(eventIndex) * force.frameRate);
                    velocityX = savitzkyGolayFilter(COPX_L(forceEventStart:forceEventEnd), '1st derivative', firstDerivativeWindowSize, (1 / force.frameRate));
                    % Correct trajectories at event start
                    if (max(abs(velocityX(1:round(evaluationRatio * (forceEventEnd - forceEventStart + 1))))) - min(abs(velocityX(1:round(evaluationRatio * (forceEventEnd - forceEventStart + 1)))))) > velocityLimit
                        if mod(forceEventStart, 2) == 0
                            motionEventStart = round(forceEventStart / 2);
                        else
                            motionEventStart = round((forceEventStart + 1) / 2);
                        end
                        filterPasses = 2;
                        filterCorrectedCutOff = filterCutOff / ((2^(1 / filterPasses) - 1)^(1 / 4));
                        [filterB, filterA] = butter(filterHalfOrder, (filterCorrectedCutOff / (0.5 * force.frameRate)));
                        indices = forceEventStart:round(forceEventStart + regressionRatio1 * (forceEventEnd - forceEventStart + 1));
                        COPX_L(indices) = filtfilt(filterB, filterA, COPX_L(indices));
                        [regressionIntercept, regressionSlope] = linearRegression(indices, COPX_L(indices));
                        vectorCAL_L = getMarker('CAL_L', 'surface', motionEventStart);
                        COPX_L(indices(1:forceSupportValues)) = max(vectorCAL_L(1), regressionIntercept + regressionSlope * indices(1:forceSupportValues));
                        COPX_L(indices) = constrainedRegression(COPX_L(indices), 3);
                    end
                    % Correct trajectories at event end
                    if abs(max(abs(velocityX((end - round(evaluationRatio * (forceEventEnd - forceEventStart + 1)) + 1):end))) - min(abs(velocityX((end - round(evaluationRatio * (forceEventEnd - forceEventStart + 1)) + 1):end)))) > velocityLimit
                        if mod(forceEventEnd, 2) == 0
                            motionEventEnd = round(forceEventEnd / 2);
                        else
                            motionEventEnd = round((forceEventEnd + 1) / 2);
                        end
                        filterPasses = 2;
                        filterCorrectedCutOff = filterCutOff / ((2^(1 / filterPasses) - 1)^(1 / 4));
                        [filterB, filterA] = butter(filterHalfOrder, (filterCorrectedCutOff / (0.5 * force.frameRate)));
                        indices = round(forceEventEnd - regressionRatio2 * (forceEventEnd - forceEventStart + 1)):forceEventEnd;
                        COPX_L(indices) = filtfilt(filterB, filterA, COPX_L(indices));
                        vectorHAL_L = getMarker('HAL_L', 'surface', motionEventEnd);
                        vectorMT2_L = getMarker('MT2_L', 'surface', motionEventEnd);
                        if strcmp(dataset, '1.3') || strcmp(dataset, '2.1') || strcmp(dataset, '2.2') || strcmp(dataset, '2.3')
                            COPX_L(indices((end - motionSupportValues + 1):end)) = vectorMT2_L(1);
                            COPZ_L(indices((end - motionSupportValues + 1):end)) = vectorMT2_L(3);
                        else
                            COPX_L(indices((end - motionSupportValues + 1):end)) = mean([vectorHAL_L(1), vectorMT2_L(1)]);
                            COPZ_L(indices((end - motionSupportValues + 1):end)) = mean([vectorHAL_L(3), vectorMT2_L(3)]);
                        end
                        COPX_L(indices) = constrainedRegression(COPX_L(indices), 3);
                        COPZ_L(indices) = constrainedRegression(COPZ_L(indices), 3);
                    end
                end
            end
            for eventIndex = 1:length(events.eventStart_R)
                if ~events.groundReactionForceCorrection_R(eventIndex)
                    forceEventStart = round(events.eventStart_R(eventIndex) * force.frameRate);
                    forceEventEnd = round(events.eventEnd_R(eventIndex) * force.frameRate);
                    velocityX = savitzkyGolayFilter(COPX_R(forceEventStart:forceEventEnd), '1st derivative', firstDerivativeWindowSize, (1 / force.frameRate));
                    % Correct trajectories at event start
                    if (max(abs(velocityX(1:round(evaluationRatio * (forceEventEnd - forceEventStart + 1))))) - min(abs(velocityX(1:round(evaluationRatio * (forceEventEnd - forceEventStart + 1)))))) > velocityLimit
                        if mod(forceEventStart, 2) == 0
                            motionEventStart = round(forceEventStart / 2);
                        else
                            motionEventStart = round((forceEventStart + 1) / 2);
                        end
                        filterPasses = 2;
                        filterCorrectedCutOff = filterCutOff / ((2^(1 / filterPasses) - 1)^(1 / 4));
                        [filterB, filterA] = butter(filterHalfOrder, (filterCorrectedCutOff / (0.5 * force.frameRate)));
                        indices = forceEventStart:round(forceEventStart + regressionRatio1 * (forceEventEnd - forceEventStart + 1));
                        COPX_R(indices) = filtfilt(filterB, filterA, COPX_R(indices));
                        [regressionIntercept, regressionSlope] = linearRegression(indices, COPX_R(indices));
                        vectorCAL_R = getMarker('CAL_R', 'surface', motionEventStart);
                        COPX_R(indices(1:forceSupportValues)) = max(vectorCAL_R(1), regressionIntercept + regressionSlope * indices(1:forceSupportValues));
                        COPX_R(indices) = constrainedRegression(COPX_R(indices), 3);
                    end
                    % Correct trajectories at event end
                    if abs(max(abs(velocityX((end - round(evaluationRatio * (forceEventEnd - forceEventStart + 1)) + 1):end))) - min(abs(velocityX((end - round(evaluationRatio * (forceEventEnd - forceEventStart + 1)) + 1):end)))) > velocityLimit
                        if mod(forceEventEnd, 2) == 0
                            motionEventEnd = round(forceEventEnd / 2);
                        else
                            motionEventEnd = round((forceEventEnd + 1) / 2);
                        end
                        filterPasses = 2;
                        filterCorrectedCutOff = filterCutOff / ((2^(1 / filterPasses) - 1)^(1 / 4));
                        [filterB, filterA] = butter(filterHalfOrder, (filterCorrectedCutOff / (0.5 * force.frameRate)));
                        indices = round(forceEventEnd - regressionRatio2 * (forceEventEnd - forceEventStart + 1)):forceEventEnd;
                        COPX_R(indices) = filtfilt(filterB, filterA, COPX_R(indices));
                        vectorHAL_R = getMarker('HAL_R', 'surface', motionEventEnd);
                        vectorMT2_R = getMarker('MT2_R', 'surface', motionEventEnd);
                        if strcmp(dataset, '1.3') || strcmp(dataset, '2.1') || strcmp(dataset, '2.2') || strcmp(dataset, '2.3') || (strcmp(dataset, '4') && (median(force.treadmillVelocity(forceEventStart:forceEventEnd)) > 1.5))
                            COPX_R(indices((end - motionSupportValues + 1):end)) = vectorMT2_R(1);
                            COPZ_R(indices((end - motionSupportValues + 1):end)) = vectorMT2_R(3);
                        else
                            COPX_R(indices((end - motionSupportValues + 1):end)) = mean([vectorHAL_R(1), vectorMT2_R(1)]);
                            COPZ_R(indices((end - motionSupportValues + 1):end)) = mean([vectorHAL_R(3), vectorMT2_R(3)]);
                        end
                        COPX_R(indices) = constrainedRegression(COPX_R(indices), 3);
                        COPZ_R(indices) = constrainedRegression(COPZ_R(indices), 3);
                    end
                end
            end
        end
        
        % Estimate the X and Z coordinates of the center of pressure for
        % double support in corrected events
        if ~strcmp(dataset, '6') && ~strcmp(dataset, '7') && ~strcmp(dataset, '8')
            motionSupportValues = 5;
            forceSupportValues = round(force.frameRate * motionSupportValues / motion.frameRate);
            doubleSupport_L = logical(bitand(events.contactPhase_L, events.contactPhase_R));
            doubleSupport_R = logical(bitand(events.contactPhase_L, events.contactPhase_R));
            for eventIndex = 1:length(events.eventStart_L)
                if events.groundReactionForceCorrection_L(eventIndex)
                    forceEventStart = round(events.eventStart_L(eventIndex) * force.frameRate);
                    forceEventEnd = round(events.eventEnd_L(eventIndex) * force.frameRate);
                    if any(doubleSupport_L(forceEventStart:forceEventEnd))
                        difference = diff([0, doubleSupport_L(forceEventStart:forceEventEnd), 0]);
                        doubleSupportStart = find(difference > 0, 1, 'first');
                        doubleSupportEnd = find(difference < 0, 1, 'first') - 1;
                        forceDoubleSupportStart1 = forceEventStart + doubleSupportStart - 1;
                        forceDoubleSupportEnd1 = forceEventStart + doubleSupportEnd - 1;
                        doubleSupportStart = find(difference > 0, 1, 'last');
                        doubleSupportEnd = find(difference < 0, 1, 'last') - 1;
                        forceDoubleSupportStart2 = forceEventStart + doubleSupportStart - 1;
                        forceDoubleSupportEnd2 = forceEventStart + doubleSupportEnd - 1;
                        if (forceDoubleSupportEnd2 - forceDoubleSupportStart1 + 1) >= 6
                            correction_L(eventIndex) = 1;
                            if mod(forceDoubleSupportStart1, 2) == 0
                                motionDoubleSupportStart1 = round(forceDoubleSupportStart1 / 2);
                            else
                                motionDoubleSupportStart1 = round((forceDoubleSupportStart1 + 1) / 2);
                            end
                            if mod(forceDoubleSupportEnd2, 2) == 0
                                motionDoubleSupportEnd2 = round(forceDoubleSupportEnd2 / 2);
                            else
                                motionDoubleSupportEnd2 = round((forceDoubleSupportEnd2 + 1) / 2);
                            end
                            vectorCAL_L = getMarker('CAL_L', 'surface', motionDoubleSupportStart1);
                            COPX_L(forceDoubleSupportStart1:(forceDoubleSupportStart1 + forceSupportValues)) = vectorCAL_L(1);
                            COPZ_L(forceDoubleSupportStart1:(forceDoubleSupportStart1 + forceSupportValues)) = vectorCAL_L(3);
                            vectorHAL_L = getMarker('HAL_L', 'surface', motionDoubleSupportEnd2);
                            vectorMT2_L = getMarker('MT2_L', 'surface', motionDoubleSupportEnd2);
                            if strcmp(dataset, '1.3') || strcmp(dataset, '2.1') || strcmp(dataset, '2.2') || strcmp(dataset, '2.3')
                                COPX_L((forceDoubleSupportEnd2 - forceSupportValues):forceDoubleSupportEnd2) = vectorMT2_L(1);
                                COPZ_L((forceDoubleSupportEnd2 - forceSupportValues):forceDoubleSupportEnd2) = vectorMT2_L(3);
                            else
                                COPX_L((forceDoubleSupportEnd2 - forceSupportValues):forceDoubleSupportEnd2) = mean([vectorHAL_L(1), vectorMT2_L(1)]);
                                COPZ_L((forceDoubleSupportEnd2 - forceSupportValues):forceDoubleSupportEnd2) = mean([vectorHAL_L(3), vectorMT2_L(3)]);
                            end
                            COPX_L(forceDoubleSupportStart1:forceDoubleSupportEnd2) = constrainedRegression(COPX_L(forceDoubleSupportStart1:forceDoubleSupportEnd2), polynomialOrderX);
                            COPZ_L(forceDoubleSupportStart1:forceDoubleSupportEnd2) = constrainedRegression(COPZ_L(forceDoubleSupportStart1:forceDoubleSupportEnd2), polynomialOrderZ);
                        else
                            if ~isnan(COPX_L(forceDoubleSupportStart1 - 1))
                                COPX_L(forceDoubleSupportStart1:forceDoubleSupportEnd2) = COPX_L(forceDoubleSupportStart1 - 1);
                                COPZ_L(forceDoubleSupportStart1:forceDoubleSupportEnd2) = COPZ_L(forceDoubleSupportStart1 - 1);
                            else
                                COPX_L(forceDoubleSupportStart1:forceDoubleSupportEnd2) = COPX_L(forceDoubleSupportEnd2 + 1);
                                COPZ_L(forceDoubleSupportStart1:forceDoubleSupportEnd2) = COPZ_L(forceDoubleSupportEnd2 + 1);
                            end
                        end
                    end
                end
            end
            for eventIndex = 1:length(events.eventStart_R)
                if events.groundReactionForceCorrection_R(eventIndex)
                    forceEventStart = round(events.eventStart_R(eventIndex) * force.frameRate);
                    forceEventEnd = round(events.eventEnd_R(eventIndex) * force.frameRate);
                    if any(doubleSupport_R(forceEventStart:forceEventEnd))
                        difference = diff([0, doubleSupport_R(forceEventStart:forceEventEnd), 0]);
                        doubleSupportStart = find(difference > 0, 1, 'first');
                        doubleSupportEnd = find(difference < 0, 1, 'first') - 1;
                        forceDoubleSupportStart1 = forceEventStart + doubleSupportStart - 1;
                        forceDoubleSupportEnd1 = forceEventStart + doubleSupportEnd - 1;
                        doubleSupportStart = find(difference > 0, 1, 'last');
                        doubleSupportEnd = find(difference < 0, 1, 'last') - 1;
                        forceDoubleSupportStart2 = forceEventStart + doubleSupportStart - 1;
                        forceDoubleSupportEnd2 = forceEventStart + doubleSupportEnd - 1;
                        if (forceDoubleSupportEnd2 - forceDoubleSupportStart1 + 1) >= 6
                            correction_R(eventIndex) = 1;
                            if mod(forceDoubleSupportStart1, 2) == 0
                                motionDoubleSupportStart1 = round(forceDoubleSupportStart1 / 2);
                            else
                                motionDoubleSupportStart1 = round((forceDoubleSupportStart1 + 1) / 2);
                            end
                            if mod(forceDoubleSupportEnd2, 2) == 0
                                motionDoubleSupportEnd2 = round(forceDoubleSupportEnd2 / 2);
                            else
                                motionDoubleSupportEnd2 = round((forceDoubleSupportEnd2 + 1) / 2);
                            end
                            vectorCAL_R = getMarker('CAL_R', 'surface', motionDoubleSupportStart1);
                            COPX_R(forceDoubleSupportStart1:(forceDoubleSupportStart1 + forceSupportValues)) = vectorCAL_R(1);
                            COPZ_R(forceDoubleSupportStart1:(forceDoubleSupportStart1 + forceSupportValues)) = vectorCAL_R(3);
                            vectorHAL_R = getMarker('HAL_R', 'surface', motionDoubleSupportEnd2);
                            vectorMT2_R = getMarker('MT2_R', 'surface', motionDoubleSupportEnd2);
                            if strcmp(dataset, '1.3') || strcmp(dataset, '2.1') || strcmp(dataset, '2.2') || strcmp(dataset, '2.3')
                                COPX_R((forceDoubleSupportEnd2 - forceSupportValues):forceDoubleSupportEnd2) = vectorMT2_R(1);
                                COPZ_R((forceDoubleSupportEnd2 - forceSupportValues):forceDoubleSupportEnd2) = vectorMT2_R(3);
                            else
                                COPX_R((forceDoubleSupportEnd2 - forceSupportValues):forceDoubleSupportEnd2) = mean([vectorHAL_R(1), vectorMT2_R(1)]);
                                COPZ_R((forceDoubleSupportEnd2 - forceSupportValues):forceDoubleSupportEnd2) = mean([vectorHAL_R(3), vectorMT2_R(3)]);
                            end
                            COPX_R(forceDoubleSupportStart1:forceDoubleSupportEnd2) = constrainedRegression(COPX_R(forceDoubleSupportStart1:forceDoubleSupportEnd2), polynomialOrderX);
                            COPZ_R(forceDoubleSupportStart1:forceDoubleSupportEnd2) = constrainedRegression(COPZ_R(forceDoubleSupportStart1:forceDoubleSupportEnd2), polynomialOrderZ);
                        else
                            if ~isnan(COPX_R(forceDoubleSupportStart1 - 1))
                                COPX_R(forceDoubleSupportStart1:forceDoubleSupportEnd2) = COPX_R(forceDoubleSupportStart1 - 1);
                                COPZ_R(forceDoubleSupportStart1:forceDoubleSupportEnd2) = COPZ_R(forceDoubleSupportStart1 - 1);
                            else
                                COPX_R(forceDoubleSupportStart1:forceDoubleSupportEnd2) = COPX_R(forceDoubleSupportEnd2 + 1);
                                COPZ_R(forceDoubleSupportStart1:forceDoubleSupportEnd2) = COPZ_R(forceDoubleSupportEnd2 + 1);
                            end
                        end
                    end
                end
            end
        end
        
        % Filter estimated center of pressure for X and Z
        filterPasses = 2;
        filterCorrectedCutOff = filterCutOff / ((2^(1 / filterPasses) - 1)^(1 / 4));
        [filterB, filterA] = butter(filterHalfOrder, (filterCorrectedCutOff / (0.5 * force.frameRate)));
        if ~strcmp(dataset, '6') && ~strcmp(dataset, '7') && ~strcmp(dataset, '8')
            COPY_L = nan(1, force.frames);
            COPY_R = nan(1, force.frames);
            for eventIndex = 1:length(events.eventStart_L)
                eventStart = round(events.eventStart_L(eventIndex) * force.frameRate);
                eventEnd = round(events.eventEnd_L(eventIndex) * force.frameRate);
                COPX_L(eventStart:eventEnd) = filtfilt(filterB, filterA, COPX_L(eventStart:eventEnd));
                COPZ_L(eventStart:eventEnd) = filtfilt(filterB, filterA, COPZ_L(eventStart:eventEnd));
                COPY_L(eventStart:eventEnd) = (groundNormal(1) * COPX_L(eventStart:eventEnd) + groundNormal(3) * COPZ_L(eventStart:eventEnd) - dot(groundPosition, groundNormal)) / -groundNormal(2);
            end
            for eventIndex = 1:length(events.eventStart_R)
                eventStart = round(events.eventStart_R(eventIndex) * force.frameRate);
                eventEnd = round(events.eventEnd_R(eventIndex) * force.frameRate);
                COPX_R(eventStart:eventEnd) = filtfilt(filterB, filterA, COPX_R(eventStart:eventEnd));
                COPZ_R(eventStart:eventEnd) = filtfilt(filterB, filterA, COPZ_R(eventStart:eventEnd));
                COPY_R(eventStart:eventEnd) = (groundNormal(1) * COPX_R(eventStart:eventEnd) + groundNormal(3) * COPZ_R(eventStart:eventEnd) - dot(groundPosition, groundNormal)) / -groundNormal(2);
            end
        else
            COPY = nan(1, force.frames);
            if ~strcmp(dataset, '8')
                indexEnd = length(events.eventStart);
            else
                indexEnd = length(events.eventStart_L);
            end
            for eventIndex = 1:indexEnd
                if ~strcmp(dataset, '8')
                    eventStart = round(events.eventStart(eventIndex) * force.frameRate);
                    eventEnd = round(events.eventEnd(eventIndex) * force.frameRate);
                else
                    eventStart = round(min([events.eventStart_L(eventIndex), events.eventStart_R(eventIndex)]) * force.frameRate);
                    eventEnd = round(max([events.eventEnd_L(eventIndex), events.eventEnd_R(eventIndex)]) * force.frameRate);
                end
                COPX(eventStart:eventEnd) = filtfilt(filterB, filterA, COPX(eventStart:eventEnd));
                COPZ(eventStart:eventEnd) = filtfilt(filterB, filterA, COPZ(eventStart:eventEnd));
                COPY(eventStart:eventEnd) = (groundNormal(1) * COPX(eventStart:eventEnd) + groundNormal(3) * COPZ(eventStart:eventEnd) - dot(groundPosition, groundNormal)) / -groundNormal(2);
            end
        end
        
        % Save processed data
        fprintf('STATUS: Saving dataset %s %s.\n', subject, dataset);
        if ~strcmp(dataset, '6') && ~strcmp(dataset, '7') && ~strcmp(dataset, '8')
            force.centerOfPressureX_L = COPX_L;
            force.centerOfPressureX_R = COPX_R;
            force.centerOfPressureY_L = COPY_L;
            force.centerOfPressureY_R = COPY_R;
            force.centerOfPressureZ_L = COPZ_L;
            force.centerOfPressureZ_R = COPZ_R;
            events.centerOfPressureCorrection_L = correction_L;
            events.centerOfPressureCorrection_R = correction_R;
        else
            force.centerOfPressureX = COPX;
            force.centerOfPressureY = COPY;
            force.centerOfPressureZ = COPZ;
            events.centerOfPressureCorrection = correction;
        end
        variables.force = force;
        variables.events = events;
        save(file, '-struct', 'variables');

    end
end
