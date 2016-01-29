% ------------------------------------------------------
% This script estimates the marker coordinates shifted to skin surface and
% the joint center positions from measured and estimated marker coordinates
% according to predictive methods given in different references.
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
markerSize = 19;
supportThickness = 3;
clothThickness = 4;
markerOffset = markerSize / 2 + supportThickness;
initialStartFrame = 7000;
initialEndFrame = 9000;
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
        startFrame = 1;
        endFrame = inf;
        dataset = datasets{datasetIndex};
        subject = subjects{subjectIndex};
        fprintf('STATUS: Processing dataset %s %s.\n', subject, dataset);

        % Load data file
        file = getFile(subject, dataset);
        if file
            variables = load(file);
            if isfield(variables, 'motion') && isfield(variables, 'ground')
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

        % Set end frame after loading file
        if (endFrame > motion.frames) || (endFrame == inf)
            endFrame = motion.frames;
        end
        if (startFrame > endFrame) || (startFrame < 1)
            startFrame = endFrame;
        end

        % Approximation of initial leg length by computing the average length
        % between ASIS_L and MM_L as well as ASIS_R and MM_R as stated in the
        % C-Motion Wiki
        initialASIS_L = getAverageMarker('ASIS_L', 'marker', initialStartFrame, initialEndFrame);
        initialASIS_R = getAverageMarker('ASIS_R', 'marker', initialStartFrame, initialEndFrame);
        initialMM_L = getAverageMarker('MM_L', 'marker', initialStartFrame, initialEndFrame);
        initialMM_R = getAverageMarker('MM_R', 'marker', initialStartFrame, initialEndFrame);
        legLength = (norm(initialMM_L - initialASIS_L) + norm(initialMM_R - initialASIS_R)) / 2;
        fprintf('Average leg length from ASIS to MM: %.1f mm\n', legLength);
        clear initialASIS_L initialASIS_R initialMM_L initialMM_R

        % Computation of initial values for the toe joints TJ_L and TJ_R according
        % to [Zatsiorsky1998] with assuming the height of the joint center in the
        % middle between ground and MT2 defined by the foot plane given by CAL, MT2
        % and MT5
        initialMT2_L = getAverageMarker('MT2_L', 'marker', initialStartFrame, initialEndFrame);
        initialMT2_R = getAverageMarker('MT2_R', 'marker', initialStartFrame, initialEndFrame);
        initialMT5_L = getAverageMarker('MT5_L', 'marker', initialStartFrame, initialEndFrame);
        initialMT5_R = getAverageMarker('MT5_R', 'marker', initialStartFrame, initialEndFrame);
        initialCAL_L = getAverageMarker('CAL_L', 'marker', initialStartFrame, initialEndFrame);
        initialCAL_R = getAverageMarker('CAL_R', 'marker', initialStartFrame, initialEndFrame);
        footNormal_L = cross((initialCAL_L - initialMT5_L), (initialMT5_L - initialMT2_L));
        footNormal_L = footNormal_L / norm(footNormal_L);
        groundPosition = ground.groundPosition;
        groundNormal = ground.groundNormal;
        distanceMT2_L_TJ_L = (dot(groundNormal, (groundPosition - initialMT2_L)) / dot(groundNormal, footNormal_L) - (markerOffset)) / 2;
        footNormal_R = cross((initialMT5_R - initialMT2_R), (initialCAL_R - initialMT5_R));
        footNormal_R = footNormal_R / norm(footNormal_R);
        distanceMT2_R_TJ_R = (dot(groundNormal, (groundPosition - initialMT2_R)) / dot(groundNormal, footNormal_R) - (markerOffset)) / 2;
        fprintf('Distance from MT2_L_s to toe joint: %.1f mm\nDistance from MT2_R_s to toe joint: %.1f mm\n', distanceMT2_L_TJ_L, distanceMT2_R_TJ_R);
        clear initialMT2_L initialMT2_R initialMT5_L initialMT5_R initialCAL_L initialCAL_R footNormal_L footNormal_R;

        % Estimate joint centers
        if ~exist('motion.surfaceX', 'var')
            motion.surfaceX = zeros(34, motion.frames);
        end
        if ~exist('motion.surfaceY', 'var')
            motion.surfaceY = zeros(34, motion.frames);
        end
        if ~exist('motion.surfaceZ', 'var')
            motion.surfaceZ = zeros(34, motion.frames);
        end
        if ~exist('motion.surfaceLabels', 'var')
            motion.surfaceLabels = {};
        end
        if ~exist('motion.jointX.estimated', 'var')
            motion.jointX.estimated = zeros(15, motion.frames);
        end
        if ~exist('motion.jointY.estimated', 'var')
            motion.jointY.estimated = zeros(15, motion.frames);
        end
        if ~exist('motion.jointZ.estimated', 'var')
            motion.jointZ.estimated = zeros(15, motion.frames);
        end
        if ~exist('motion.jointLabels.estimated', 'var')
            motion.jointLabels.estimated = {};
        end
        statusCounter = 0;
        for currentFrame = startFrame:endFrame

            % Get marker coordinates
            vectorTRA_L_m = getMarker('TRA_L', 'marker', currentFrame);
            vectorTRA_R_m = getMarker('TRA_R', 'marker', currentFrame);
            vectorGLA_m = getMarker('GLA', 'marker', currentFrame);
            vectorC7_m = getMarker('C7', 'marker', currentFrame);
            vectorT8_m = getMarker('T8', 'marker', currentFrame);
            vectorT12_m = getMarker('T12', 'marker', currentFrame);
            vectorSUP_m = getMarker('SUP', 'marker', currentFrame);
            vectorACR_L_m = getMarker('ACR_L', 'marker', currentFrame);
            vectorACR_R_m = getMarker('ACR_R', 'marker', currentFrame);
            vectorLHC_L_m = getMarker('LHC_L', 'marker', currentFrame);
            vectorLHC_R_m = getMarker('LHC_R', 'marker', currentFrame);
            vectorWRI_L_m = getMarker('WRI_L', 'marker', currentFrame);
            vectorWRI_R_m = getMarker('WRI_R', 'marker', currentFrame);
            vectorASIS_L_m = getMarker('ASIS_L', 'marker', currentFrame);
            vectorASIS_R_m = getMarker('ASIS_R', 'marker', currentFrame);
            vectorPSIS_L_m = getMarker('PSIS_L', 'marker', currentFrame);
            vectorPSIS_R_m = getMarker('PSIS_R', 'marker', currentFrame);
            vectorPS_m = getMarker('PS', 'marker', currentFrame);
            vectorGTR_L_m = getMarker('GTR_L', 'marker', currentFrame);
            vectorGTR_R_m = getMarker('GTR_R', 'marker', currentFrame);
            vectorLFC_L_m = getMarker('LFC_L', 'marker', currentFrame);
            vectorLFC_R_m = getMarker('LFC_R', 'marker', currentFrame);
            vectorMFC_L_m = getMarker('MFC_L', 'marker', currentFrame);
            vectorMFC_R_m = getMarker('MFC_R', 'marker', currentFrame);
            vectorLM_L_m = getMarker('LM_L', 'marker', currentFrame);
            vectorLM_R_m = getMarker('LM_R', 'marker', currentFrame);
            vectorMM_L_m = getMarker('MM_L', 'marker', currentFrame);
            vectorMM_R_m = getMarker('MM_R', 'marker', currentFrame);
            vectorHAL_L_m = getMarker('HAL_L', 'marker', currentFrame);
            vectorHAL_R_m = getMarker('HAL_R', 'marker', currentFrame);
            vectorMT2_L_m = getMarker('MT2_L', 'marker', currentFrame);
            vectorMT2_R_m = getMarker('MT2_R', 'marker', currentFrame);
            vectorMT5_L_m = getMarker('MT5_L', 'marker', currentFrame);
            vectorMT5_R_m = getMarker('MT5_R', 'marker', currentFrame);
            vectorCAL_L_m = getMarker('CAL_L', 'marker', currentFrame);
            vectorCAL_R_m = getMarker('CAL_R', 'marker', currentFrame);

            % Estimate marker coordinates shifted to skin surface
            % TRA markers
            vectorTRA_R_TRA_L = vectorTRA_L_m - vectorTRA_R_m;
            directionTRA = vectorTRA_R_TRA_L / norm(vectorTRA_R_TRA_L);
            vectorTRA_L_s = vectorTRA_L_m - markerOffset * directionTRA;
            vectorTRA_R_s = vectorTRA_R_m + markerOffset * directionTRA;
            % GLA marker
            vectorGLA_TRA = (vectorTRA_R_m + vectorTRA_R_TRA_L / 2) - vectorGLA_m;
            directionGLA = vectorGLA_TRA / norm(vectorGLA_TRA);
            vectorGLA_s = vectorGLA_m + markerOffset * directionGLA;
            % C7 and SUP markers
            vectorC7_SUP = vectorSUP_m - vectorC7_m;
            directionC7_SUP = vectorC7_SUP / norm(vectorC7_SUP);
            vectorC7_s = vectorC7_m + markerOffset * directionC7_SUP;
            vectorSUP_s = vectorSUP_m - markerOffset * directionC7_SUP;
            % T12 marker
            vectorT12_T8 = vectorT8_m - vectorT12_m;
            vectorACR_R_ACR_L = vectorACR_L_m - vectorACR_R_m;
            rotatedVector = rotateVector(vectorT12_T8 / norm(vectorT12_T8), (90 * pi / 180), vectorACR_R_ACR_L);
            directionT12 = rotatedVector / norm(rotatedVector);
            vectorT12_s = vectorT12_m + markerOffset * directionT12;
            % T8 marker
            directionT8 = (directionC7_SUP + directionT12) / norm(directionC7_SUP + directionT12);
            vectorT8_s = vectorT8_m + markerOffset * directionT8;
            % ACR markers
            crossProduct = cross(vectorACR_R_ACR_L, vectorC7_SUP);
            directionACR = crossProduct / norm(crossProduct);
            vectorACR_L_s = vectorACR_L_m + markerOffset * directionACR;
            vectorACR_R_s = vectorACR_R_m + markerOffset * directionACR;
            % ASIS and PSIS markers
            vectorASIS_PSIS = (vectorPSIS_R_m + (vectorPSIS_L_m - vectorPSIS_R_m) / 2) - (vectorASIS_R_m + (vectorASIS_L_m - vectorASIS_R_m) / 2);
            directionASIS_PSIS = vectorASIS_PSIS / norm(vectorASIS_PSIS);
            vectorASIS_L_s = vectorASIS_L_m + markerOffset * directionASIS_PSIS;
            vectorPSIS_L_s = vectorPSIS_L_m - markerOffset * directionASIS_PSIS;
            vectorASIS_R_s = vectorASIS_R_m + markerOffset * directionASIS_PSIS;
            vectorPSIS_R_s = vectorPSIS_R_m - markerOffset * directionASIS_PSIS;
            % PS marker
            vectorPS_PSIS = (vectorPSIS_R_m + (vectorPSIS_L_m - vectorPSIS_R_m) / 2) - vectorPS_m;
            directionPS = vectorPS_PSIS / norm(vectorPS_PSIS);
            vectorPS_s = vectorPS_m + ((markerSize) / 2 + clothThickness) * directionPS;
            % GTR markers
            vectorGTR_R_GTR_L = vectorGTR_L_m - vectorGTR_R_m;
            directionGTR = vectorGTR_R_GTR_L / norm(vectorGTR_R_GTR_L);
            vectorGTR_L_s = vectorGTR_L_m - markerOffset * directionGTR;
            vectorGTR_R_s = vectorGTR_R_m + markerOffset * directionGTR;
            % LFC and MFC markers
            vectorLFC_MFC = vectorMFC_L_m - vectorLFC_L_m;
            directionLFC_MFC = vectorLFC_MFC / norm(vectorLFC_MFC);
            vectorLFC_L_s = vectorLFC_L_m + markerOffset * directionLFC_MFC;
            vectorMFC_L_s = vectorMFC_L_m - markerOffset * directionLFC_MFC;
            vectorLFC_MFC = vectorMFC_R_m - vectorLFC_R_m;
            directionLFC_MFC = vectorLFC_MFC / norm(vectorLFC_MFC);
            vectorLFC_R_s = vectorLFC_R_m + markerOffset * directionLFC_MFC;
            vectorMFC_R_s = vectorMFC_R_m - markerOffset * directionLFC_MFC;
            % LM and MM markers
            vectorLM_MM = vectorMM_L_m - vectorLM_L_m;
            directionLM_MM = vectorLM_MM / norm(vectorLM_MM);
            vectorLM_L_s = vectorLM_L_m + markerOffset * directionLM_MM;
            vectorMM_L_s = vectorMM_L_m - markerOffset * directionLM_MM;
            vectorLM_MM = vectorMM_R_m - vectorLM_R_m;
            directionLM_MM = vectorLM_MM / norm(vectorLM_MM);
            vectorLM_R_s = vectorLM_R_m + markerOffset * directionLM_MM;
            vectorMM_R_s = vectorMM_R_m - markerOffset * directionLM_MM;
            % CAL markers
            vectorCAL_MT2 = vectorMT2_L_m - vectorCAL_L_m;
            directionCAL = vectorCAL_MT2 / norm(vectorCAL_MT2);
            vectorCAL_L_s = vectorCAL_L_m + markerOffset * directionCAL;
            vectorCAL_MT2 = vectorMT2_R_m - vectorCAL_R_m;
            directionCAL = vectorCAL_MT2 / norm(vectorCAL_MT2);
            vectorCAL_R_s = vectorCAL_R_m + markerOffset * directionCAL;
            % HAL, MT2 and MT5  markers
            crossProduct = cross((vectorCAL_L_m - vectorMT5_L_m), (vectorMT5_L_m - vectorMT2_L_m));
            directionMT_HAL = crossProduct / norm(crossProduct);
            vectorMT2_L_s = vectorMT2_L_m + markerOffset * directionMT_HAL;
            vectorMT5_L_s = vectorMT5_L_m + markerOffset * directionMT_HAL;
            vectorHAL_L_s = vectorHAL_L_m + markerOffset * directionMT_HAL;
            crossProduct = cross((vectorMT5_R_m - vectorMT2_R_m), (vectorCAL_R_m - vectorMT5_R_m));
            directionMT_HAL = crossProduct / norm(crossProduct);
            vectorMT2_R_s = vectorMT2_R_m + markerOffset * directionMT_HAL;
            vectorMT5_R_s = vectorMT5_R_m + markerOffset * directionMT_HAL;
            vectorHAL_R_s = vectorHAL_R_m + markerOffset * directionMT_HAL;
            clear vectorTRA_R_TRA_L vectorGLA_TRA vectorC7_SUP vectorT12_T8 vectorACR_R_ACR_L ...
                vectorASIS_PSIS vectorLFC_MFC vectorGTR_R_GTR_L vectorLM_MM vectorCAL_MT2 ...
                directionTRA directionGLA directionC7_SUP directionT8 directionT12 ...
                directionACR directionASIS_PSIS directionPS directionGTR directionLFC_MFC ...
                directionLM_MM directionCAL directionMT_HAL rotatedVector crossProduct;

            % Lower neck joint (LNJ)
            % Estimate the lower neck joint LNJ (C7/T1) according to [Reed1999]
            % with using the connection of the ACR markers as rotation axis
            vectorC7_SUP = vectorSUP_s - vectorC7_s;
            vectorACR_R_ACR_L = vectorACR_L_s - vectorACR_R_s;
            vectorLNJ = vectorC7_s + rotateVector((0.55 * vectorC7_SUP), -(8 * pi / 180), vectorACR_R_ACR_L);
            clear vectorC7_SUP vectorACR_R_ACR_L;

            % Upper lumbar joint (ULJ)
            % Estimate the upper lumbar joint ULJ (T12/L1) according to [Reed1999] with
            % using the connection of the ACR markers as rotation axis
            vectorC7_SUP = vectorSUP_s - vectorC7_s;
            vectorT12_T8 = vectorT8_s - vectorT12_s;
            vectorACR_R_ACR_L = vectorACR_L_s - vectorACR_R_s;
            vectorULJ = vectorT12_s + rotateVector((0.52 * norm(vectorC7_SUP) * vectorT12_T8 / norm(vectorT12_T8)), (94 * pi / 180), vectorACR_R_ACR_L);
            clear vectorC7_SUP vectorT12_T8 vectorACR_R_ACR_L;

            % Shoulder joints (SJ_L, SJ_R)
            % Estimate the shoulder joints SJ_L and SJ_R according to [Reed1999]
            % with using the connection of the ACR markers as rotation axis
            vectorC7_SUP = vectorSUP_s - vectorC7_s;
            vectorACR_R_ACR_L = vectorACR_L_s - vectorACR_R_s;
            vectorSJ_L = vectorACR_L_s + rotateVector((0.42 * vectorC7_SUP), (67 * pi / 180), vectorACR_R_ACR_L);
            vectorSJ_R = vectorACR_R_s + rotateVector((0.42 * vectorC7_SUP), (67 * pi / 180), vectorACR_R_ACR_L);
            clear vectorC7_SUP vectorACR_R_ACR_L;

            % Estimate marker coordinates shifted to skin
            % LHC markers
            vectorLHC_WRI = vectorWRI_L_m - vectorLHC_L_m;
            vectorLHC_SJ = vectorSJ_L - vectorLHC_L_m;
            crossProduct = cross(vectorLHC_WRI, vectorLHC_SJ);
            vectorLHC_L_s = vectorLHC_L_m + markerOffset * crossProduct / norm(crossProduct);
            vectorLHC_WRI = vectorWRI_R_m - vectorLHC_R_m;
            vectorLHC_SJ = vectorSJ_R - vectorLHC_R_m;
            crossProduct = cross(vectorLHC_SJ, vectorLHC_WRI);
            vectorLHC_R_s = vectorLHC_R_m + markerOffset * crossProduct / norm(crossProduct);
            clear crossProduct vectorLHC_WRI vectorLHC_SJ;

            % Elbow joints (EJ_L, EJ_R)
            % Estimate the elbow joints EJ_L and EJ_R according to [Reed1999]
            vectorLHC_WRI = vectorWRI_L_m - vectorLHC_L_s;
            vectorLHC_SJ = vectorSJ_L - vectorLHC_L_s;
            crossProduct = cross(vectorLHC_WRI, vectorLHC_SJ);
            vectorEJ_L = vectorLHC_L_s + 0.155 * norm(vectorLHC_WRI) * crossProduct / norm(crossProduct);
            vectorLHC_WRI = vectorWRI_R_m - vectorLHC_R_s;
            vectorLHC_SJ = vectorSJ_R - vectorLHC_R_s;
            crossProduct = cross(vectorLHC_SJ, vectorLHC_WRI);
            vectorEJ_R = vectorLHC_R_s + 0.155 * norm(vectorLHC_WRI) * crossProduct / norm(crossProduct);
            clear crossProduct vectorLHC_WRI vectorLHC_SJ;

            % Lower lumbar joint (LLJ)
            % Estimate the lower lumbar joint LLJ (L5/S1) according to [Reed1999]
            vectorASIS_R_ASIS_L = vectorASIS_L_s - vectorASIS_R_s;
            axisY = vectorASIS_R_ASIS_L / norm(vectorASIS_R_ASIS_L);
            vectorASIS_R_PS = vectorPS_s - vectorASIS_R_s;
            vectorCP_s = vectorASIS_R_s + dot(vectorASIS_R_PS, axisY) / dot(axisY, axisY) * axisY;
            axisZ_s = (vectorCP_s - vectorPS_s) / norm(vectorCP_s - vectorPS_s);
            axisX_s = cross(axisY, axisZ_s);
            vectorASIS_L_b = vectorASIS_L_s - 10 * axisX_s;
            vectorASIS_R_b = vectorASIS_R_s - 10 * axisX_s;
            vectorPS_b = vectorPS_s - 17.7 * axisX_s - 17.7 * axisZ_s;
            % Ambiguous value for the flesh margin correction vector for the PSIS
            % markers in [Reed1999]: 10 mm in text and 5 mm in table 6
            vectorPSIS_L_b = vectorPSIS_L_s + 10 * axisX_s;
            vectorPSIS_R_b = vectorPSIS_R_s + 10 * axisX_s;
            vectorASIS_R_b_PS_b = vectorPS_b - vectorASIS_R_b;
            vectorASIS_R_b_ASIS_L_b = vectorASIS_L_b - vectorASIS_R_b;
            vectorCP_b = vectorASIS_R_b + (dot(vectorASIS_R_b_PS_b, axisY) / dot(axisY, axisY) * axisY);
            axisZ_b = (vectorCP_b - vectorPS_b) / norm(vectorCP_b - vectorPS_b);
            axisX_b = cross(axisY, axisZ_b);
            pelvisWidth = norm(vectorASIS_R_b_ASIS_L_b);
            pelvisHeight = norm(vectorCP_b - vectorPS_b);
            pelvisDepth = (norm(vectorPSIS_L_b - vectorASIS_L_b) + norm(vectorPSIS_R_b - vectorASIS_R_b)) / 2;
            if strcmp(subject, 'A')
                vectorLLJ = vectorCP_b - 0.425 * pelvisDepth * axisX_b + 0.452 * pelvisHeight * axisZ_b;
            else
                vectorLLJ = vectorCP_b - 0.377 * pelvisDepth * axisX_b + 0.399 * pelvisHeight * axisZ_b;
            end
            clear vectorASIS_R_ASIS_L vectorASIS_R_PS vectorCP_s vectorASIS_L_b vectorASIS_R_b ...
                vectorPS_b vectorPSIS_L_b vectorPSIS_R_b vectorASIS_R_b_PS_b vectorASIS_R_b_ASIS_L_b ...
                vectorCP_b axisX_s axisY axisZ_s axisX_b axisZ_b;

            % Hip joints (HJ_L, HJ_R)
            methodHipJoint = 'Harrington2007';
            switch methodHipJoint

                case 'Harrington2007'
                % Estimate the hip joints HJ_L and HJ_R according to [Harrington2007]
                % with estimating the bone marker positions according to [Reed1999]
                % and assuming a soft tissue offset of 10 mm
                vectorASIS_R_ASIS_L = vectorASIS_R_s - vectorASIS_L_s;
                axisZ = vectorASIS_R_ASIS_L / norm(vectorASIS_R_ASIS_L);
                vectorASIS_R_PS = vectorPS_s - vectorASIS_R_s;
                vectorCP_s = vectorASIS_R_s + dot(vectorASIS_R_PS, axisZ) / dot(axisZ, axisZ) * axisZ;
                axisY_s = (vectorCP_s - vectorPS_s) / norm(vectorCP_s - vectorPS_s);
                axisX_s = cross(axisY_s, axisZ);
                vectorASIS_L_b = vectorASIS_L_s - 10 * axisX_s;
                vectorASIS_R_b = vectorASIS_R_s - 10 * axisX_s;
                % Ambiguous value for the flesh margin correction vector for the PSIS
                % markers in [Reed1999]: 10 mm in text and 5 mm in table 6
                vectorPSIS_L_b = vectorPSIS_L_s + 10 * axisX_s;
                vectorPSIS_R_b = vectorPSIS_R_s + 10 * axisX_s;
                vectorASIS_R_b_ASIS_L_b = vectorASIS_R_b - vectorASIS_L_b;
                vectorPSIS_R_b_PSIS_L_b = vectorPSIS_R_b - vectorPSIS_L_b;
                vectorCP_b = vectorASIS_L_b + 0.5 * vectorASIS_R_b_ASIS_L_b;
                axisX_b = (vectorCP_b - (vectorPSIS_L_b + 0.5 * vectorPSIS_R_b_PSIS_L_b)) / norm(vectorCP_b - (vectorPSIS_L_b + 0.5 * vectorPSIS_R_b_PSIS_L_b));
                axisY_b = cross(axisZ, axisX_b);
                pelvisWidth = norm(vectorASIS_R_b_ASIS_L_b);
                pelvisDepth = norm(vectorCP_b - (vectorPSIS_L_b + 0.5 * vectorPSIS_R_b_PSIS_L_b));
                vectorHJ_L = vectorCP_b + (-0.24 * pelvisDepth - 9.9) * axisX_b + (-0.16 * pelvisWidth - 0.04 * legLength - 7.1) * axisY_b - (0.28 * pelvisDepth + 0.16 * pelvisWidth + 7.9) * axisZ;
                vectorHJ_R = vectorCP_b + (-0.24 * pelvisDepth - 9.9) * axisX_b + (-0.16 * pelvisWidth - 0.04 * legLength - 7.1) * axisY_b + (0.28 * pelvisDepth + 0.16 * pelvisWidth + 7.9) * axisZ;
                clear pelvisWidth pelvisDepth pelvisDepth vectorASIS_R_ASIS_L vectorASIS_R_PS ...
                    vectorCP_s vectorASIS_L_b vectorASIS_R_b vectorPSIS_L_b vectorPSIS_R_b ...
                    vectorASIS_R_b_ASIS_L_b vectorPSIS_R_b_PSIS_L_b vectorCP_b axisX_s axisY ...
                    axisZ_s axisX_b axisZ_b;

                case 'Reed1999'
                % Estimate the hip joints HJ_L and HJ_R according to [Reed1999]
                vectorASIS_R_ASIS_L = vectorASIS_L_s - vectorASIS_R_s;
                axisY = vectorASIS_R_ASIS_L / norm(vectorASIS_R_ASIS_L);
                vectorASIS_R_PS = vectorPS_s - vectorASIS_R_s;
                vectorCP_s = vectorASIS_R_s + dot(vectorASIS_R_PS, axisY) / dot(axisY, axisY) * axisY;
                axisZ_s = (vectorCP_s - vectorPS_s) / norm(vectorCP_s - vectorPS_s);
                axisX_s = cross(axisY, axisZ_s);
                vectorASIS_L_b = vectorASIS_L_s - 10 * axisX_s;
                vectorASIS_R_b = vectorASIS_R_s - 10 * axisX_s;
                vectorPS_b = vectorPS_s - 17.7 * axisX_s - 17.7 * axisZ_s;
                % Ambiguous value for the flesh margin correction vector for the PSIS
                % markers in [Reed1999]: 10 mm in text and 5 mm in table 6
                %vectorPSIS_L_b = vectorPSIS_L_s + 10 * axisX_s;
                %vectorPSIS_R_b = vectorPSIS_R_s + 10 * axisX_s;
                vectorASIS_R_b_PS_b = vectorPS_b - vectorASIS_R_b;
                vectorASIS_R_b_ASIS_L_b = vectorASIS_L_b - vectorASIS_R_b;
                vectorCP_b = vectorASIS_R_b + (dot(vectorASIS_R_b_PS_b, axisY) / dot(axisY, axisY) * axisY);
                axisZ_b = (vectorCP_b - vectorPS_b) / norm(vectorCP_b - vectorPS_b);
                axisX_b = cross(axisY, axisZ_b);
                pelvisWidth = norm(vectorASIS_R_b_ASIS_L_b);
                %pelvisHeight = norm(vectorCP_b - vectorPS_b);
                %pelvisDepth = (norm(vectorPSIS_L_b - vectorASIS_L_b) + norm(vectorPSIS_R_b - vectorASIS_R_b)) / 2;
                %vectorHJ_L = vectorCP_b - 0.34 * pelvisDepth * axisX_b + 0.36 * pelvisWidth * axisY - 0.79 * pelvisHeight * axisZ_b;
                %vectorHJ_R = vectorCP_b - 0.34 * pelvisDepth * axisX_b - 0.36 * pelvisWidth * axisY - 0.79 * pelvisHeight * axisZ_b;
                vectorHJ_L = vectorCP_b - 0.24 * pelvisWidth * axisX_b + 0.36 * pelvisWidth * axisY - 0.3 * pelvisWidth * axisZ_b;
                vectorHJ_R = vectorCP_b - 0.24 * pelvisWidth * axisX_b - 0.36 * pelvisWidth * axisY - 0.3 * pelvisWidth * axisZ_b;
                clear pelvisWidth pelvisHeight pelvisDepth vectorASIS_R_ASIS_L vectorASIS_R_PS ...
                    vectorCP_s vectorASIS_L_b vectorASIS_R_b vectorPS_b vectorPSIS_L_b ...
                    vectorPSIS_R_b vectorASIS_R_b_PS_b vectorASIS_R_b_ASIS_L_b vectorCP_b ...
                    axisX_s axisY axisZ_s axisX_b axisZ_b;

                case 'Leardini1999'
                % Estimate the hip joints HJ_L and HJ_R according to [Leardini1999]
                % with estimating the bone marker positions according to [Reed1999]
                % and assuming a soft tissue offset of 10 mm
                vectorASIS_R_ASIS_L = vectorASIS_R_s - vectorASIS_L_s;
                axisZ = vectorASIS_R_ASIS_L / norm(vectorASIS_R_ASIS_L);
                vectorASIS_R_PS = vectorPS_s - vectorASIS_R_s;
                vectorCP_s = vectorASIS_R_s + dot(vectorASIS_R_PS, axisZ) / dot(axisZ, axisZ) * axisZ;
                axisY_s = (vectorCP_s - vectorPS_s) / norm(vectorCP_s - vectorPS_s);
                axisX_s = cross(axisY_s, axisZ);
                vectorASIS_L_b = vectorASIS_L_s - 10 * axisX_s;
                vectorASIS_R_b = vectorASIS_R_s - 10 * axisX_s;
                % Ambiguous value for the flesh margin correction vector for the PSIS
                % markers in [Reed1999]: 10 mm in text and 5 mm in table 6
                vectorPSIS_L_b = vectorPSIS_L_s + 10 * axisX_s;
                vectorPSIS_R_b = vectorPSIS_R_s + 10 * axisX_s;
                vectorASIS_R_b_ASIS_L_b = vectorASIS_R_b - vectorASIS_L_b;
                vectorPSIS_R_b_PSIS_L_b = vectorPSIS_R_b - vectorPSIS_L_b;
                vectorCP_b = vectorASIS_L_b + 0.5 * vectorASIS_R_b_ASIS_L_b;
                axisX_b = (vectorCP_b - (vectorPSIS_L_b + 0.5 * vectorPSIS_R_b_PSIS_L_b)) / norm(vectorCP_b - (vectorPSIS_L_b + 0.5 * vectorPSIS_R_b_PSIS_L_b));
                axisY_b = cross(axisZ, axisX_b);
                pelvisWidth = norm(vectorASIS_R_b_ASIS_L_b);
                pelvisDepth = norm(vectorCP_b - (vectorPSIS_L_b + 0.5 * vectorPSIS_R_b_PSIS_L_b));
                vectorHJ_L = vectorCP_b - 0.31 * pelvisDepth * axisX_b - 0.096 * legLength * axisY_b + (-0.09 * pelvisWidth + 111) * axisZ;
                vectorHJ_R = vectorCP_b - 0.34 * pelvisDepth * axisX_b - 0.096 * legLength * axisY_b - (-0.09 * pelvisWidth + 111) * axisZ;
                clear pelvisWidth pelvisDepth pelvisDepth vectorASIS_R_ASIS_L vectorASIS_R_PS ...
                    vectorCP_s vectorASIS_L_b vectorASIS_R_b vectorPSIS_L_b vectorPSIS_R_b ...
                    vectorASIS_R_b_ASIS_L_b vectorPSIS_R_b_PSIS_L_b vectorCP_b axisX_s axisY ...
                    axisZ_s axisX_b axisZ_b;

                case 'Seidel1995'
                % Estimate hip joints HJ_L and HJ_R according to [Seidel1995] with
                % estimating the bone marker positions according to [Reed1999] and
                % assuming a soft tissue offset of 10 mm
                vectorASIS_R_ASIS_L = vectorASIS_R_s - vectorASIS_L_s;
                axisZ = vectorASIS_R_ASIS_L / norm(vectorASIS_R_ASIS_L);
                vectorASIS_R_PS = vectorPS_s - vectorASIS_R_s;
                vectorCP_s = vectorASIS_R_s + dot(vectorASIS_R_PS, axisZ) / dot(axisZ, axisZ) * axisZ;
                axisY_s = (vectorCP_s - vectorPS_s) / norm(vectorCP_s - vectorPS_s);
                axisX_s = cross(axisY_s, axisZ);
                vectorASIS_L_b = vectorASIS_L_s - 10 * axisX_s;
                vectorASIS_R_b = vectorASIS_R_s - 10 * axisX_s;
                vectorPS_b = vectorPS_s - 17.7 * axisX_s + 17.7 * axisY_s;
                % Ambiguous value for the flesh margin correction vector for the PSIS
                % markers in [Reed1999]: 10 mm in text and 5 mm in table 6
                vectorPSIS_L_b = vectorPSIS_L_s + 10 * axisX_s;
                vectorPSIS_R_b = vectorPSIS_R_s + 10 * axisX_s;
                vectorASIS_R_b_PS_b = vectorPS_b - vectorASIS_R_b;
                vectorASIS_R_b_ASIS_L_b = vectorASIS_L_b - vectorASIS_R_b;
                vectorCP_b = vectorASIS_R_b + (dot(vectorASIS_R_b_PS_b, axisZ) / dot(axisZ, axisZ) * axisZ);
                axisY_b = (vectorCP_b - vectorPS_b) / norm(vectorCP_b - vectorPS_b);
                axisX_b = cross(axisY_b, axisZ);
                pelvisWidth = norm(vectorASIS_R_b_ASIS_L_b);
                pelvisHeight = norm(vectorCP_b - vectorPS_b);
                pelvisDepth = (norm(vectorPSIS_L_b - vectorASIS_L_b) + norm(vectorPSIS_R_b - vectorASIS_R_b)) / 2;
                vectorHJ_L(:, currentFrame) = vectorASIS_L_b + (-0.34 * pelvisDepth * axisX_b - 0.79 * pelvisHeight * axisY_b + 0.14 * pelvisWidth * axisZ);
                vectorHJ_R(:, currentFrame) = vectorASIS_R_b + (-0.34 * pelvisDepth * axisX_b - 0.79 * pelvisHeight * axisY_b - 0.14 * pelvisWidth * axisZ);
                clear pelvisWidth pelvisHeight pelvisDepth vectorASIS_R_ASIS_L vectorASIS_R_PS ...
                    vectorCP_s vectorASIS_L_b vectorASIS_R_b vectorPS_b vectorPSIS_L_b ...
                    vectorPSIS_R_b vectorASIS_R_b_PS_b vectorASIS_R_b_ASIS_L_b vectorCP_b ...
                    axisX_s axisY_s axisX_b axisY_b axisZ;

                case 'Davis1991'
                % Estimate hip joints HJ_L and HJ_R according to [Davis1991] with
                % assuming the sacrum marker to be in the middle of the line connection
                % PSIS_L and PSIS_R and approximating the leg length by the average length
                % between ASIS_L and MM_L as well as ASIS_R and MM_R as stated in the
                % C-Motion Wiki
                vectorPSIS_R_PSIS_L = vectorPSIS_L_m - vectorPSIS_R_m;
                vectorASIS_R_ASIS_L = vectorASIS_L_m - vectorASIS_R_m;
                vectorCP = vectorASIS_R_m + 0.5 * vectorASIS_R_ASIS_L;
                axisY = vectorASIS_R_ASIS_L / norm(vectorASIS_R_ASIS_L);
                axisX = (vectorCP - (vectorPSIS_R_m + 0.5 * vectorPSIS_R_PSIS_L)) / norm(vectorCP - (vectorPSIS_R_m + 0.5 * vectorPSIS_R_PSIS_L));
                crossProduct = cross(axisX, axisY);
                axisZ = crossProduct / norm(crossProduct);
                A = 0.115 * legLength - 15.3;
                B = (28.4 * pi / 180);
                C = (18 * pi / 180);
                D = 0.1288 * legLength - 48.56;
                vectorHJ_L = vectorCP + ((-D - (markerOffset)) * cos(C) + A * cos(B) * sin(C)) * axisX - (A * sin(B) - norm(vectorASIS_R_ASIS_L) / 2) * axisY + ((-D - (markerOffset)) * sin(C) - A * cos(B) * cos(C)) * axisZ;
                vectorHJ_R = vectorCP + ((-D - (markerOffset)) * cos(C) + A * cos(B) * sin(C)) * axisX + (A * sin(B) - norm(vectorASIS_R_ASIS_L) / 2) * axisY + ((-D - (markerOffset)) * sin(C) - A * cos(B) * cos(C)) * axisZ;
                clear vectorPSIS_R_PSIS_L vectorASIS_R_ASIS_L vectorCP crossProduct ...
                    A B C D axisX axisY axisZ;

                case 'Bell1990'
                % Estimate hip joints HJ_L and HJ_R according to [Bell1990] with
                % assuming a soft tissue offset of 10 mm
                vectorPSIS_R_PSIS_L = vectorPSIS_L_s - vectorPSIS_R_s;
                vectorASIS_R_ASIS_L = vectorASIS_L_s - vectorASIS_R_s;
                vectorCP = vectorASIS_R_s + 0.5 * vectorASIS_R_ASIS_L;
                axisZ = vectorASIS_R_ASIS_L / norm(vectorASIS_R_ASIS_L);
                axisX = (vectorCP - (vectorPSIS_R_s + 0.5 * vectorPSIS_R_PSIS_L)) / norm(vectorCP - (vectorPSIS_R_s + 0.5 * vectorPSIS_R_PSIS_L));
                vectorCP = vectorCP - 10 * axisX;
                crossProduct = cross(axisX, axisZ);
                axisY = crossProduct / norm(crossProduct);
                pelvisWidth = norm(vectorASIS_R_ASIS_L);
                vectorHJ_L = vectorCP + (-0.19 * pelvisWidth * axisX - 0.3 * pelvisWidth * axisY + (0.5 - 0.14) * pelvisWidth * axisZ);
                vectorHJ_R = vectorCP + (-0.19 * pelvisWidth * axisX - 0.3 * pelvisWidth * axisY - (0.5 - 0.14) * pelvisWidth * axisZ);
                clear vectorPSIS_R_PSIS_L vectorASIS_R_ASIS_L vectorCP crossProduct ...
                    axisX axisY axisZ;

                otherwise
                fprintf('ERROR: Invalid hip joint method!');
                return;

            end

            % Knee joints (KJ_L, KJ_R)
            methodKneeJoint = 'Dumas2007';
            switch methodKneeJoint

                case 'Reed1999'
                % Estimate the knee joints KJ_L and KJ_R and ankle joints AJ_L and
                % AJ_R according to [Reed1999]
                vectorLFC_LM = vectorLM_L_s - vectorLFC_L_s;
                vectorLFC_HJ = vectorHJ_L - vectorLFC_L_s;
                crossProduct = cross(vectorLFC_HJ, vectorLFC_LM);
                vectorKJ_L = vectorLFC_L_s + (0.118 * norm(vectorLFC_LM) * crossProduct / norm(crossProduct));
                vectorLFC_LM = vectorLM_R_s - vectorLFC_R_s;
                vectorLFC_HJ = vectorHJ_R - vectorLFC_R_s;
                crossProduct = cross(vectorLFC_LM, vectorLFC_HJ);
                vectorKJ_R = vectorLFC_R_s + (0.118 * norm(vectorLFC_LM) * crossProduct / norm(crossProduct));
                clear crossProduct vectorLFC_L_LM_L vectorLFC_L_HJ_L vectorLFC_R_LM_R vectorLFC_R_HJ_R;

                case 'Dumas2007'
                % Estimate the knee joints KJ_L and KJ_R according to
                % [Dumas2007a], [Davis1991], [Dempster1955]
                vectorMFC_LFC = vectorLFC_L_s - vectorMFC_L_s;
                vectorKJ_L = vectorMFC_L_s + 0.5 * vectorMFC_LFC;
                vectorMFC_LFC = vectorLFC_R_s - vectorMFC_R_s;
                vectorKJ_R = vectorMFC_R_s + 0.5 * vectorMFC_LFC;
                clear vectorMFC_LFC;

                otherwise
                fprintf('ERROR: Invalid knee joint method!');
                return;

            end

            % Ankle joint (AJ_L, AJ_R)
            methodAnkleJoint = 'Dumas2007';
            switch methodAnkleJoint

                case 'Reed1999'
                % Estimate the knee joints KJ_L and KJ_R and ankle joints AJ_L and
                % AJ_R according to [Reed1999]
                vectorLFC_LM = vectorLM_L_s - vectorLFC_L_s;
                vectorLFC_HJ = vectorHJ_L - vectorLFC_L_s;
                crossProduct = cross(vectorLFC_HJ, vectorLFC_LM);
                vectorKJ_L = vectorLFC_L_s + (0.118 * norm(vectorLFC_LM) * crossProduct / norm(crossProduct));
                vectorLFC_LM = vectorLM_R_s - vectorLFC_R_s;
                vectorLFC_HJ = vectorHJ_R - vectorLFC_R_s;
                crossProduct = cross(vectorLFC_LM, vectorLFC_HJ);
                vectorAJ_R = vectorLM_R_s + (0.085 * norm(vectorLFC_LM) * crossProduct / norm(crossProduct));
                clear crossProduct vectorLFC_LM vectorLFC_HJ;

                case 'Dumas2007'
                % Estimate the ankle joints AJ_L and AJ_R according to
                % [Dumas2007a], [Davis1991]
                vectorMM_LM = vectorLM_L_s - vectorMM_L_s;
                vectorAJ_L = vectorMM_L_s + 0.5 * vectorMM_LM;
                vectorMM_LM = vectorLM_R_s - vectorMM_R_s;
                vectorAJ_R = vectorMM_R_s + 0.5 * vectorMM_LM;
                clear vectorMM_LM;

                case 'Dempster1955'
                % Estimate the ankle joints AJ_L and AJ_R according to [Dempster1955]
                vectorMFC_MM = vectorMM_L_s - vectorMFC_L_s;
                ankleAxis = (vectorMM_L_s + 5 * vectorMFC_MM / norm(vectorMFC_MM)) - vectorLM_L_s;
                vectorAJ_L = vectorLM_L_s + 0.5 * ankleAxis;
                vectorMFC_MM = vectorMM_R_s - vectorMFC_R_s;
                ankleAxis = (vectorMM_R_s + 5 * vectorMFC_MM / norm(vectorMFC_MM)) - vectorLM_R_s;
                vectorAJ_R = vectorLM_R_s + 0.5 * ankleAxis;
                clear vectorMFC_MM ankleAxis;

                case 'Hicks1953'
                % Estimate the ankle joints AJ_L and AJ_R according to [Hicks1953]
                vectorLFC_LM = vectorLM_L_s - vectorLFC_L_s;
                vectorMFC_MM = vectorMM_L_s - vectorMFC_L_s;
                averageVector = (vectorLFC_LM + vectorMFC_MM) / 2;
                vectorLM_MM = vectorMM_L_s - vectorLM_L_s;
                crossProduct = cross(vectorLM_MM, averageVector);
                ankleAxis = (vectorMM_L_s + 15 * crossProduct / norm(crossProduct)) - (vectorLM_L_s + 5 * vectorLFC_LM / norm(vectorLFC_LM));
                vectorAJ_L = (vectorLM_L_s + 5 * vectorLFC_LM / norm(vectorLFC_LM)) + 0.5 * ankleAxis;
                vectorLFC_LM = vectorLM_R_s - vectorLFC_R_s;
                vectorMFC_MM = vectorMM_R_s - vectorMFC_R_s;
                averageVector = (vectorLFC_LM + vectorMFC_MM) / 2;
                vectorLM_MM = vectorMM_R_s - vectorLM_R_s;
                crossProduct = cross(averageVector, vectorLM_MM);
                ankleAxis = (vectorMM_R_s + 15 * crossProduct / norm(crossProduct)) - (vectorLM_R_s + 5 * vectorLFC_LM / norm(vectorLFC_LM));
                vectorAJ_R = (vectorLM_R_s + 5 * vectorLFC_LM / norm(vectorLFC_LM)) + 0.5 * ankleAxis;
                clear vectorLFC_LM vectorMFC_MM vectorLM_MM averageVector crossProduct ankleAxis;

                otherwise
                fprintf('ERROR: Invalid ankle joint method!');
                return;

            end

            % Toe joints (TJ_L, TJ_R)
            % Estimate the toe joints TJ_L and TJ_R according to [Zatsiorsky1998]
            % with assuming the height of the joint center in the middle between ground
            % and MT2 defined by the foot plane given by CAL, MT2 and MT5
            crossProduct = cross((vectorCAL_L_m - vectorMT5_L_m), (vectorMT5_L_m - vectorMT2_L_m));
            footNormal_L = crossProduct / norm(crossProduct);
            vectorTJ_L = vectorMT2_L_s + distanceMT2_L_TJ_L * footNormal_L;
            crossProduct = cross((vectorMT5_R_m - vectorMT2_R_m), (vectorCAL_R_m - vectorMT5_R_m));
            footNormal_R = crossProduct / norm(crossProduct);
            vectorTJ_R = vectorMT2_R_s + distanceMT2_R_TJ_R * footNormal_R;
            clear footNormal_L footNormal_R crossProduct;

            % Save shifted marker coordinates and estimated joint data
            surfaceData = [
                vectorTRA_L_s,...
                vectorTRA_R_s,...
                vectorGLA_s,...
                vectorACR_L_s,...
                vectorACR_R_s,...
                vectorLHC_L_s,...
                vectorLHC_R_s,...
                vectorSUP_s,...
                vectorC7_s,...
                vectorT8_s,...
                vectorT12_s,...
                vectorASIS_L_s,...
                vectorASIS_R_s,...
                vectorPSIS_L_s,...
                vectorPSIS_R_s,...
                vectorPS_s,...
                vectorGTR_L_s,...
                vectorGTR_R_s,...
                vectorLFC_L_s,...
                vectorLFC_R_s,...
                vectorMFC_L_s,...
                vectorMFC_R_s,...
                vectorLM_L_s,...
                vectorLM_R_s,...
                vectorMM_L_s,...
                vectorMM_R_s,...
                vectorCAL_L_s,...
                vectorCAL_R_s,...
                vectorMT2_L_s,...
                vectorMT2_R_s,...
                vectorMT5_L_s,...
                vectorMT5_R_s,...
                vectorHAL_L_s,...
                vectorHAL_R_s...
            ];
            motion.surfaceX(:, currentFrame) = surfaceData(1,:)';
            motion.surfaceY(:, currentFrame) = surfaceData(2,:)';
            motion.surfaceZ(:, currentFrame) = surfaceData(3,:)';
            motion.surfaceLabels = {'TRA_L', 'TRA_R', 'GLA', 'ACR_L', 'ACR_R',  'LHC_L', 'LHC_R', 'SUP', 'C7', 'T8', 'T12', 'ASIS_L', 'ASIS_R', 'PSIS_L', 'PSIS_R', 'PS', 'GTR_L', 'GTR_R', 'LFC_L', 'LFC_R', 'MFC_L', 'MFC_R', 'LM_L', 'LM_R', 'MM_L', 'MM_R', 'CAL_L', 'CAL_R', 'MT2_L', 'MT2_R', 'MT5_L', 'MT5_R', 'HAL_L', 'HAL_R'};
            jointData = [...
                vectorLNJ,...
                vectorSJ_L,...
                vectorSJ_R,...
                vectorEJ_L,...
                vectorEJ_R,...
                vectorULJ,...
                vectorLLJ,...
                vectorHJ_L,...
                vectorHJ_R,...
                vectorKJ_L,...
                vectorKJ_R,...
                vectorAJ_L,...
                vectorAJ_R,...
                vectorTJ_L,...
                vectorTJ_R...
            ];
            motion.jointX.estimated(:, currentFrame) = jointData(1,:)';
            motion.jointY.estimated(:, currentFrame) = jointData(2,:)';
            motion.jointZ.estimated(:, currentFrame) = jointData(3,:)';
            motion.jointLabels.estimated = { ...
                'LNJ', ...
                'SJ_L', ...
                'SJ_R', ...
                'EJ_L', ...
                'EJ_R', ...
                'ULJ', ...
                'LLJ', ...
                'HJ_L', ...
                'HJ_R', ...
                'KJ_L', ...
                'KJ_R', ...
                'AJ_L', ...
                'AJ_R', ...
                'TJ_L', ...
                'TJ_R' ...
            };
            clear surfaceData jointData;

            % Print status
            statusCounter = statusCounter + 1;
            if statusCounter >= 100
                fprintf('STATUS: %.1f %%\n', (currentFrame - startFrame) / (endFrame - startFrame) * 100);
                statusCounter = 0;
            end

        end

        % Add additional data
        motion.markerSize = markerSize;
        motion.supportThickness = supportThickness;
        motion.clothThickness = clothThickness;

        % Save processed data
        fprintf('STATUS: Saving dataset %s %s.\n', subject, dataset);
        motion = orderfields(motion);
        variables.motion = motion;
        save(file, '-struct', 'variables');
        
    end 
end