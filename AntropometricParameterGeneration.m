% ------------------------------------------------------
% This script estimates subject parameters based on segment
% lengths and on regression equations from [Dumas2007] in coordinate
% systems according to [Wu2002], [Wu2005]. Lengths are given in
% millimeters, masses are given in kilogram and moments of inertia are
% given in kilogram * meters^2
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
subject = 'A';
if strcmp(subject, 'A')
    gender = 'female';
    origin = 'central european';
    bodyHeight = 1610;
    bodyMass = 575 / 9.81;
    age = 26.8;
    footLength_L = 230;
    footLength_R = 225;
else
    gender = 'male';
    origin = 'central european';
    bodyHeight = 1790;
    bodyMass = 845 / 9.81;
    age = 32.1;
    footLength_L = 271;
    footLength_R = 275;
end
saveFile = [getPath, filesep, subject, filesep, 'Parameters2.mat'];
initialStartFrame = 7000;
initialEndFrame = 9000;
startFrame = 1;
endFrame = inf;
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

% Specify origins of the segment coordinate systems
head.origin.point = 'LNJ';
head.origin.type = 'joint';
upperArm_L.origin.point = 'SJ_L';
upperArm_L.origin.type = 'joint';
upperArm_R.origin.point = 'SJ_R';
upperArm_R.origin.type = 'joint';
lowerArm_L.origin.point = 'EJ_L';
lowerArm_L.origin.type = 'joint';
lowerArm_R.origin.point = 'EJ_R';
lowerArm_R.origin.type = 'joint';
hand_L.origin.point = 'WRI_L';
hand_L.origin.type = 'marker';
hand_R.origin.point = 'WRI_R';
hand_R.origin.type = 'marker';
torso.origin.point = 'LNJ';
torso.origin.type = 'joint';
pelvis.origin.point = 'LLJ';
pelvis.origin.type = 'joint';
thigh_L.origin.point = 'HJ_L';
thigh_L.origin.type = 'joint';
thigh_R.origin.point = 'HJ_R';
thigh_R.origin.type = 'joint';
shank_L.origin.point = 'KJ_L';
shank_L.origin.type = 'joint';
shank_R.origin.point = 'KJ_R';
shank_R.origin.type = 'joint';
foot_L.origin.point = 'AJ_L';
foot_L.origin.type = 'joint';
foot_R.origin.point = 'AJ_R';
foot_R.origin.type = 'joint';

% Estimate segment lengths and positions of markers and joints based
% on all datasets
headSegmentLengthY = zeros(length(datasets), 1);
torsoSegmentLengthY = zeros(length(datasets), 1);
upperArmSegmentLengthY_L = zeros(length(datasets), 1);
upperArmSegmentLengthY_R = zeros(length(datasets), 1);
lowerArmSegmentLengthY_L = zeros(length(datasets), 1);
lowerArmSegmentLengthY_R = zeros(length(datasets), 1);
pelvisSegmentLengthY = zeros(length(datasets), 1);
pelvisSegmentLengthZ = zeros(length(datasets), 1);
thighSegmentLengthY_L = zeros(length(datasets), 1);
thighSegmentLengthY_R = zeros(length(datasets), 1);
shankSegmentLengthY_L = zeros(length(datasets), 1);
shankSegmentLengthY_R = zeros(length(datasets), 1);
footReferenceLength_L = zeros(length(datasets), 1);
footReferenceLength_R = zeros(length(datasets), 1);
footSegmentLengthY_L = zeros(length(datasets), 1);
footSegmentLengthY_R = zeros(length(datasets), 1);
headRelativePositionGLA = zeros(length(datasets), 3);
headRelativePositionTRA_L = zeros(length(datasets), 3);
headRelativePositionTRA_R = zeros(length(datasets), 3);
torsoRelativePositionACR_L = zeros(length(datasets), 3);
torsoRelativePositionACR_R = zeros(length(datasets), 3);
torsoRelativePositionSUP = zeros(length(datasets), 3);
torsoRelativePositionC7 = zeros(length(datasets), 3);
torsoRelativePositionT8 = zeros(length(datasets), 3);
torsoRelativePositionT12 = zeros(length(datasets), 3);
torsoRelativePositionSJ_L = zeros(length(datasets), 3);
torsoRelativePositionSJ_R = zeros(length(datasets), 3);
torsoRelativePositionLLJ = zeros(length(datasets), 3);
upperArmRelativePositionLHC_L = zeros(length(datasets), 3);
upperArmRelativePositionLHC_R = zeros(length(datasets), 3);
upperArmRelativePositionEJ_L = zeros(length(datasets), 3);
upperArmRelativePositionEJ_R = zeros(length(datasets), 3);
lowerArmRelativePositionLHC_L = zeros(length(datasets), 3);
lowerArmRelativePositionLHC_R = zeros(length(datasets), 3);
lowerArmRelativePositionWRI_L = zeros(length(datasets), 3);
lowerArmRelativePositionWRI_R = zeros(length(datasets), 3);
pelvisRelativePositionASIS_L = zeros(length(datasets), 3);
pelvisRelativePositionASIS_R = zeros(length(datasets), 3);
pelvisRelativePositionPSIS_L = zeros(length(datasets), 3);
pelvisRelativePositionPSIS_R = zeros(length(datasets), 3);
pelvisRelativePositionPS = zeros(length(datasets), 3);
pelvisRelativePositionHJ_L = zeros(length(datasets), 3);
pelvisRelativePositionHJ_R = zeros(length(datasets), 3);
thighRelativePositionGTR_L = zeros(length(datasets), 3);
thighRelativePositionGTR_R = zeros(length(datasets), 3);
thighRelativePositionLFC_L = zeros(length(datasets), 3);
thighRelativePositionLFC_R = zeros(length(datasets), 3);
thighRelativePositionMFC_L = zeros(length(datasets), 3);
thighRelativePositionMFC_R = zeros(length(datasets), 3);
thighRelativePositionKJ_L = zeros(length(datasets), 3);
thighRelativePositionKJ_R = zeros(length(datasets), 3);
shankRelativePositionLM_L = zeros(length(datasets), 3);
shankRelativePositionLM_R = zeros(length(datasets), 3);
shankRelativePositionMM_L = zeros(length(datasets), 3);
shankRelativePositionMM_R = zeros(length(datasets), 3);
shankRelativePositionAJ_L = zeros(length(datasets), 3);
shankRelativePositionAJ_R = zeros(length(datasets), 3);
footRelativePositionCAL_L = zeros(length(datasets), 3);
footRelativePositionCAL_R = zeros(length(datasets), 3);
footRelativePositionMT2_L = zeros(length(datasets), 3);
footRelativePositionMT2_R = zeros(length(datasets), 3);
footRelativePositionMT5_L = zeros(length(datasets), 3);
footRelativePositionMT5_R = zeros(length(datasets), 3);
absolutePositionLNJ = zeros(length(datasets), 3);
absolutePositionSJ_L = zeros(length(datasets), 3);
absolutePositionSJ_R = zeros(length(datasets), 3);
absolutePositionEJ_L = zeros(length(datasets), 3);
absolutePositionEJ_R = zeros(length(datasets), 3);
absolutePositionLLJ = zeros(length(datasets), 3);
absolutePositionHJ_L = zeros(length(datasets), 3);
absolutePositionHJ_R = zeros(length(datasets), 3);
absolutePositionKJ_L = zeros(length(datasets), 3);
absolutePositionKJ_R = zeros(length(datasets), 3);
absolutePositionAJ_L = zeros(length(datasets), 3);
absolutePositionAJ_R = zeros(length(datasets), 3);
for datasetIndex = 1:length(datasets)

    % Set parameters
    dataset = datasets{datasetIndex};

    % Load data file
    loadFile = getFile(subject, dataset);
    if loadFile
        variables = load(loadFile);
        if isfield(variables, 'motion') && isfield(variables, 'ground')
            motion = variables.motion;
            ground = variables.ground;
        else
            fprintf('WARNING: No matching data found!\n');
            continue;
        end
    else
        fprintf('ERROR: No matching data file found!\n');
        return;
    end

    % Set end frame after loading file
    if (endFrame > motion.frames) || (endFrame == inf)
        endFrame = motion.frames;
    end
    if (startFrame > endFrame) || (startFrame < 1)
        startFrame = endFrame;
    end
        
    % Estimate midpoint between metatarsal head 1 (MT1) and metatarsal head
    % 5 (MT5) from existing marker positions of the metatarsal head 2 (MT2)
    % and metatarsal head 5 (MT5)
    vectorMT_L = getAverageMarker('MT5_L', 'surface', initialStartFrame, initialEndFrame) + 3 * getAverageVector('MT5_L', 'surface', 'MT2_L', 'surface', initialStartFrame, initialEndFrame) / 4;
    vectorMT_R = getAverageMarker('MT5_R', 'surface', initialStartFrame, initialEndFrame) + 3 * getAverageVector('MT5_R', 'surface', 'MT2_R', 'surface', initialStartFrame, initialEndFrame) / 4;
    
    % Compute average lengths for each dataset with estimating the head
    % segment length by applying the body height
    groundPosition = ground.groundPosition;
    headSegmentLengthY(datasetIndex) = bodyHeight - [0, 1, 0] * (getAverageJoint('LNJ', initialStartFrame, initialEndFrame) - groundPosition);
    torsoSegmentLengthY(datasetIndex) = norm(getAverageVector('LLJ', 'joint', 'LNJ', 'joint', initialStartFrame, initialEndFrame));
    upperArmSegmentLengthY_L(datasetIndex) = norm(getAverageVector('EJ_L', 'joint', 'SJ_L', 'joint', initialStartFrame, initialEndFrame));
    upperArmSegmentLengthY_R(datasetIndex) = norm(getAverageVector('EJ_R', 'joint', 'SJ_R', 'joint', initialStartFrame, initialEndFrame));
    lowerArmSegmentLengthY_L(datasetIndex) = norm(getAverageVector('WRI_L', 'marker', 'EJ_L', 'joint', initialStartFrame, initialEndFrame));
    lowerArmSegmentLengthY_R(datasetIndex) = norm(getAverageVector('WRI_R', 'marker', 'EJ_R', 'joint', initialStartFrame, initialEndFrame));
    pelvisSegmentLengthY(datasetIndex) = norm(getAverageJoint('LLJ', initialStartFrame, initialEndFrame) - (getAverageJoint('HJ_L', initialStartFrame, initialEndFrame) + getAverageVector('HJ_L', 'joint', 'HJ_R', 'joint', initialStartFrame, initialEndFrame) / 2));
    pelvisSegmentLengthZ(datasetIndex) = norm(getAverageVector('HJ_L', 'joint', 'HJ_R', 'joint', initialStartFrame, initialEndFrame));
    thighSegmentLengthY_L(datasetIndex) = norm(getAverageVector('KJ_L', 'joint', 'HJ_L', 'joint', initialStartFrame, initialEndFrame));
    thighSegmentLengthY_R(datasetIndex) = norm(getAverageVector('KJ_R', 'joint', 'HJ_R', 'joint', initialStartFrame, initialEndFrame));
    shankSegmentLengthY_L(datasetIndex) = norm(getAverageVector('AJ_L', 'joint', 'KJ_L', 'joint', initialStartFrame, initialEndFrame));
    shankSegmentLengthY_R(datasetIndex) = norm(getAverageVector('AJ_R', 'joint', 'KJ_R', 'joint', initialStartFrame, initialEndFrame));
    footReferenceLength_L(datasetIndex) = norm(getAverageJoint('AJ_L', initialStartFrame, initialEndFrame) - vectorMT_L);
    footReferenceLength_R(datasetIndex) = norm(getAverageJoint('AJ_R', initialStartFrame, initialEndFrame) - vectorMT_R);
    footSegmentLengthY_L(datasetIndex) = [0, 1, 0] * (getAverageJoint('AJ_L', initialStartFrame, initialEndFrame) - groundPosition);
    footSegmentLengthY_R(datasetIndex) = [0, 1, 0] * (getAverageJoint('AJ_R', initialStartFrame, initialEndFrame) - groundPosition);

    % Compute average absolute joint positions for each dataset
    absolutePositionLNJ(datasetIndex, :) = getAverageJoint('LNJ', initialStartFrame, initialEndFrame)';
    absolutePositionSJ_L(datasetIndex, :) = getAverageJoint('SJ_L', initialStartFrame, initialEndFrame)';
    absolutePositionSJ_R(datasetIndex, :) = getAverageJoint('SJ_R', initialStartFrame, initialEndFrame)';
    absolutePositionEJ_L(datasetIndex, :) = getAverageJoint('EJ_L', initialStartFrame, initialEndFrame)';
    absolutePositionEJ_R(datasetIndex, :) = getAverageJoint('EJ_R', initialStartFrame, initialEndFrame)';
    absolutePositionLLJ(datasetIndex, :) = getAverageJoint('LLJ', initialStartFrame, initialEndFrame)';
    absolutePositionHJ_L(datasetIndex, :) = getAverageJoint('HJ_L', initialStartFrame, initialEndFrame)';
    absolutePositionHJ_R(datasetIndex, :) = getAverageJoint('HJ_R', initialStartFrame, initialEndFrame)';
    absolutePositionKJ_L(datasetIndex, :) = getAverageJoint('KJ_L', initialStartFrame, initialEndFrame)';
    absolutePositionKJ_R(datasetIndex, :) = getAverageJoint('KJ_R', initialStartFrame, initialEndFrame)';
    absolutePositionAJ_L(datasetIndex, :) = getAverageJoint('AJ_L', initialStartFrame, initialEndFrame)';
    absolutePositionAJ_R(datasetIndex, :) = getAverageJoint('AJ_R', initialStartFrame, initialEndFrame)';
    
    % Estimate average reference positions of adjacent markers and joints
    % in the head coordinate system for each dataset with approximating the
    % y-axis direction by the normal of a plane containing the glabella
    % (GLA) and tragion (TRA) markers pointing superior
    yAxis = cross(getAverageVector('TRA_L', 'surface', 'TRA_R', 'surface', initialStartFrame, initialEndFrame), getAverageVector('TRA_L', 'surface', 'GLA', 'surface', initialStartFrame, initialEndFrame));
    yAxis = yAxis / norm(yAxis);
    zAxis = cross(getAverageVector('LNJ', 'joint', 'GLA', 'surface', initialStartFrame, initialEndFrame), yAxis);
    zAxis = zAxis / norm(zAxis);
    xAxis = cross(yAxis, zAxis);
    xAxis = xAxis / norm(xAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    headRelativePositionGLA(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'joint', 'GLA', 'surface', initialStartFrame, initialEndFrame))';
    headRelativePositionTRA_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'joint', 'TRA_L', 'surface', initialStartFrame, initialEndFrame))';
    headRelativePositionTRA_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'joint', 'TRA_R', 'surface', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the torso coordinate system for each dataset
    yAxis = getAverageVector('LLJ', 'joint', 'LNJ', 'joint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    zAxis = cross(getAverageVector('LNJ', 'joint', 'SUP', 'surface', initialStartFrame, initialEndFrame), yAxis);
    zAxis = zAxis / norm(zAxis);
    xAxis = cross(yAxis, zAxis);
    xAxis = xAxis / norm(xAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    torsoRelativePositionACR_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'joint', 'ACR_L', 'surface', initialStartFrame, initialEndFrame))';
    torsoRelativePositionACR_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'joint', 'ACR_R', 'surface', initialStartFrame, initialEndFrame))';
    torsoRelativePositionSUP(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'joint', 'SUP', 'surface', initialStartFrame, initialEndFrame))';
    torsoRelativePositionC7(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'joint', 'C7', 'surface', initialStartFrame, initialEndFrame))';
    torsoRelativePositionT8(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'joint', 'T8', 'surface', initialStartFrame, initialEndFrame))';
    torsoRelativePositionT12(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'joint', 'T12', 'surface', initialStartFrame, initialEndFrame))';
    torsoRelativePositionSJ_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'joint', 'SJ_L', 'joint', initialStartFrame, initialEndFrame))';
    torsoRelativePositionSJ_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'joint', 'SJ_R', 'joint', initialStartFrame, initialEndFrame))';
    torsoRelativePositionLLJ(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'joint', 'LLJ', 'joint', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the left upper arm coordinate system for each dataset
    yAxis = getAverageVector('EJ_L', 'joint', 'SJ_L', 'joint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    xAxis = cross(getAverageVector('EJ_L', 'joint', 'LHC_L', 'surface', initialStartFrame, initialEndFrame), yAxis);
    xAxis = xAxis / norm(xAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    upperArmRelativePositionLHC_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('SJ_L', 'joint', 'LHC_L', 'surface', initialStartFrame, initialEndFrame))';
    upperArmRelativePositionEJ_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('SJ_L', 'joint', 'EJ_L', 'joint', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the right upper arm coordinate system for each dataset
    yAxis = getAverageVector('EJ_R', 'joint', 'SJ_R', 'joint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    xAxis = cross(getAverageVector('LHC_R', 'surface', 'EJ_R', 'joint', initialStartFrame, initialEndFrame), yAxis);
    xAxis = xAxis / norm(xAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    upperArmRelativePositionLHC_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('SJ_R', 'joint', 'LHC_R', 'surface', initialStartFrame, initialEndFrame))';
    upperArmRelativePositionEJ_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('SJ_R', 'joint', 'EJ_R', 'joint', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the left lower arm coordinate system for each dataset
    yAxis = getAverageVector('WRI_L', 'marker', 'EJ_L', 'joint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    xAxis = cross(getAverageVector('EJ_L', 'joint', 'LHC_L', 'surface', initialStartFrame, initialEndFrame), yAxis);
    xAxis = xAxis / norm(xAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    lowerArmRelativePositionLHC_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('EJ_L', 'joint', 'LHC_L', 'surface', initialStartFrame, initialEndFrame))';
    lowerArmRelativePositionWRI_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('EJ_L', 'joint', 'WRI_L', 'marker', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the right lower arm coordinate system for each dataset
    yAxis = getAverageVector('WRI_R', 'marker', 'EJ_R', 'joint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    xAxis = cross(getAverageVector('LHC_R', 'surface', 'EJ_R', 'joint', initialStartFrame, initialEndFrame), yAxis);
    xAxis = xAxis / norm(xAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    lowerArmRelativePositionLHC_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('EJ_R', 'joint', 'LHC_R', 'surface', initialStartFrame, initialEndFrame))';
    lowerArmRelativePositionWRI_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('EJ_R', 'joint', 'WRI_R', 'marker', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the pelvis coordinate system for each dataset
    zAxis = getAverageVector('ASIS_L', 'surface', 'ASIS_R', 'surface', initialStartFrame, initialEndFrame);
    zAxis = zAxis / norm(zAxis);
    yAxis = cross((getAverageVector('ASIS_L', 'surface', 'PSIS_L', 'surface', initialStartFrame, initialEndFrame) + getAverageVector('ASIS_L', 'surface', 'PSIS_R', 'surface', initialStartFrame, initialEndFrame)) / 2, zAxis);
    yAxis = yAxis / norm(yAxis);
    xAxis = cross(yAxis, zAxis);
    xAxis = xAxis / norm(xAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    pelvisRelativePositionASIS_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('LLJ', 'joint', 'ASIS_L', 'surface', initialStartFrame, initialEndFrame))';
    pelvisRelativePositionASIS_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('LLJ', 'joint', 'ASIS_R', 'surface', initialStartFrame, initialEndFrame))';
    pelvisRelativePositionPSIS_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('LLJ', 'joint', 'PSIS_L', 'surface', initialStartFrame, initialEndFrame))';
    pelvisRelativePositionPSIS_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('LLJ', 'joint', 'PSIS_R', 'surface', initialStartFrame, initialEndFrame))';
    pelvisRelativePositionPS(datasetIndex, :) = (rotationMatrix \ getAverageVector('LLJ', 'joint', 'PS', 'surface', initialStartFrame, initialEndFrame))';
    pelvisRelativePositionHJ_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('LLJ', 'joint', 'HJ_L', 'joint', initialStartFrame, initialEndFrame))';
    pelvisRelativePositionHJ_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('LLJ', 'joint', 'HJ_R', 'joint', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the left thigh coordinate system for each dataset
    yAxis = getAverageVector('KJ_L', 'joint', 'HJ_L', 'joint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    xAxis = cross(getAverageVector('HJ_L', 'joint', 'MFC_L', 'surface', initialStartFrame, initialEndFrame), getAverageVector('HJ_L', 'joint', 'LFC_L', 'surface', initialStartFrame, initialEndFrame));
    xAxis = xAxis / norm(xAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    thighRelativePositionGTR_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('HJ_L', 'joint', 'GTR_L', 'surface', initialStartFrame, initialEndFrame))';
    thighRelativePositionLFC_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('HJ_L', 'joint', 'LFC_L', 'surface', initialStartFrame, initialEndFrame))';
    thighRelativePositionMFC_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('HJ_L', 'joint', 'MFC_L', 'surface', initialStartFrame, initialEndFrame))';
    thighRelativePositionKJ_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('HJ_L', 'joint', 'KJ_L', 'joint', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the right thigh coordinate system for each dataset
    yAxis = getAverageVector('KJ_R', 'joint', 'HJ_R', 'joint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    xAxis = cross(getAverageVector('HJ_R', 'joint', 'LFC_R', 'surface', initialStartFrame, initialEndFrame), getAverageVector('HJ_R', 'joint', 'MFC_R', 'surface', initialStartFrame, initialEndFrame));
    xAxis = xAxis / norm(xAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    thighRelativePositionGTR_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('HJ_R', 'joint', 'GTR_R', 'surface', initialStartFrame, initialEndFrame))';
    thighRelativePositionLFC_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('HJ_R', 'joint', 'LFC_R', 'surface', initialStartFrame, initialEndFrame))';
    thighRelativePositionMFC_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('HJ_R', 'joint', 'MFC_R', 'surface', initialStartFrame, initialEndFrame))';
    thighRelativePositionKJ_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('HJ_R', 'joint', 'KJ_R', 'joint', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the left shank coordinate system for each dataset
    yAxis = getAverageVector('AJ_L', 'joint', 'KJ_L', 'joint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    xAxis = cross(getAverageVector('KJ_L', 'joint', 'MM_L', 'surface', initialStartFrame, initialEndFrame), getAverageVector('KJ_L', 'joint', 'LM_L', 'surface', initialStartFrame, initialEndFrame));
    xAxis = xAxis / norm(xAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    shankRelativePositionLM_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('KJ_L', 'joint', 'LM_L', 'surface', initialStartFrame, initialEndFrame))';
    shankRelativePositionMM_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('KJ_L', 'joint', 'MM_L', 'surface', initialStartFrame, initialEndFrame))';
    shankRelativePositionAJ_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('KJ_L', 'joint', 'AJ_L', 'joint', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the right shank coordinate system for each dataset
    yAxis = getAverageVector('AJ_R', 'joint', 'KJ_R', 'joint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    xAxis = cross(getAverageVector('KJ_R', 'joint', 'LM_R', 'surface', initialStartFrame, initialEndFrame), getAverageVector('KJ_R', 'joint', 'MM_L', 'surface', initialStartFrame, initialEndFrame));
    xAxis = xAxis / norm(xAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    shankRelativePositionLM_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('KJ_R', 'joint', 'LM_R', 'surface', initialStartFrame, initialEndFrame))';
    shankRelativePositionMM_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('KJ_R', 'joint', 'MM_R', 'surface', initialStartFrame, initialEndFrame))';
    shankRelativePositionAJ_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('KJ_R', 'joint', 'AJ_R', 'joint', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the left foot coordinate system for each dataset with estimating
    % the all required points from existing marker positions of the
    % metatarsal head 2 (MT2) and metatarsal head 5 (MT5)
    xAxis = vectorMT_L - getAverageMarker('CAL_L', 'surface', initialStartFrame, initialEndFrame);
    xAxis = xAxis / norm(xAxis);
    yAxis = cross(getAverageVector('CAL_L', 'surface', 'MT2_L', 'surface', initialStartFrame, initialEndFrame), getAverageVector('CAL_L', 'surface', 'MT5_L', 'surface', initialStartFrame, initialEndFrame));
    yAxis = yAxis / norm(yAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    footRelativePositionCAL_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('AJ_L', 'joint', 'CAL_L', 'surface', initialStartFrame, initialEndFrame))';
    footRelativePositionMT2_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('AJ_L', 'joint', 'MT2_L', 'surface', initialStartFrame, initialEndFrame))';
    footRelativePositionMT5_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('AJ_L', 'joint', 'MT5_L', 'surface', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix vectorMT_L;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the right foot coordinate system for each dataset with estimating
    % the all required points from existing marker positions of the
    % metatarsal head 2 (MT2) and metatarsal head 5 (MT5)
    xAxis = vectorMT_R - getAverageMarker('CAL_R', 'surface', initialStartFrame, initialEndFrame);
    xAxis = xAxis / norm(xAxis);
    yAxis = cross(getAverageVector('CAL_R', 'surface', 'MT5_R', 'surface', initialStartFrame, initialEndFrame), getAverageVector('CAL_R', 'surface', 'MT2_R', 'surface', initialStartFrame, initialEndFrame));
    yAxis = yAxis / norm(yAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    footRelativePositionCAL_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('AJ_R', 'joint', 'CAL_R', 'surface', initialStartFrame, initialEndFrame))';
    footRelativePositionMT2_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('AJ_R', 'joint', 'MT2_R', 'surface', initialStartFrame, initialEndFrame))';
    footRelativePositionMT5_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('AJ_R', 'joint', 'MT5_R', 'surface', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix vectorMT_R;
    
    % Print status
    fprintf('STATUS: %i out of %i\n', datasetIndex, length(datasets));
    
end
clear variables motion;

% Compute median of segment lengths of all datasets
head.segmentLengthY = median(headSegmentLengthY);
torso.segmentLengthY = median(torsoSegmentLengthY);
upperArm_L.segmentLengthY = median(upperArmSegmentLengthY_L);
upperArm_R.segmentLengthY = median(upperArmSegmentLengthY_R);
pelvis.segmentLengthY = median(pelvisSegmentLengthY);
pelvis.segmentLengthZ = median(pelvisSegmentLengthZ);
lowerArm_L.segmentLengthY = median(lowerArmSegmentLengthY_L);
lowerArm_R.segmentLengthY = median(lowerArmSegmentLengthY_R);
thigh_L.segmentLengthY = median(thighSegmentLengthY_L);
thigh_R.segmentLengthY = median(thighSegmentLengthY_R);
shank_L.segmentLengthY = median(shankSegmentLengthY_L);
shank_R.segmentLengthY = median(shankSegmentLengthY_R);
foot_L.referenceLength = median(footReferenceLength_L);
foot_R.referenceLength = median(footReferenceLength_R);
foot_L.segmentLengthX = footLength_L;
foot_R.segmentLengthX = footLength_R;
foot_L.segmentLengthY = median(footSegmentLengthY_L);
foot_R.segmentLengthY = median(footSegmentLengthY_R);
clear headSegmentLengthY torsoSegmentLengthY upperArmSegmentLengthY_L upperArmSegmentLengthY_R ...
    pelvisSegmentLengthZ pelvisReferenceLength lowerArmSegmentLengthY_L lowerArmSegmentLengthY_R ...
    thighSegmentLengthY_L thighSegmentLengthY_R shankSegmentLengthY_L shankSegmentLengthY_R ...
    footReferenceLength_L footReferenceLength_R footSegmentLengthY_L footSegmentLengthY_R;

% Compute median of joint positions of all datasets
joints.absolutePosition.LNJ = median(absolutePositionLNJ)';
joints.absolutePosition.SJ_L = median(absolutePositionSJ_L)';
joints.absolutePosition.SJ_R = median(absolutePositionSJ_R)';
joints.absolutePosition.EJ_L = median(absolutePositionEJ_L)';
joints.absolutePosition.EJ_R = median(absolutePositionEJ_R)';
joints.absolutePosition.LLJ = median(absolutePositionLLJ)';
joints.absolutePosition.HJ_L = median(absolutePositionHJ_L)';
joints.absolutePosition.HJ_R = median(absolutePositionHJ_R)';
joints.absolutePosition.KJ_L = median(absolutePositionKJ_L)';
joints.absolutePosition.KJ_R = median(absolutePositionKJ_R)';
joints.absolutePosition.AJ_L = median(absolutePositionAJ_L)';
joints.absolutePosition.AJ_R = median(absolutePositionAJ_R)';
clear absolutePositionLNJ absolutePositionSJ_L absolutePositionSJ_R absolutePositionEJ_L ...
    absolutePositionEJ_R absolutePositionLLJ absolutePositionHJ_L absolutePositionHJ_R ...
    absolutePositionKJ_L absolutePositionKJ_R absolutePositionAJ_L absolutePositionAJ_R

% Estimate missing segment lengths based on measurements or regression
% equations from [Winter2009]
hand_L.segmentLengthY = 0.108 * bodyHeight;
hand_R.segmentLengthY = 0.108 * bodyHeight;
foot_L.segmentLengthZ = 0.055 * bodyHeight;
foot_R.segmentLengthZ = 0.055 * bodyHeight;

% Compute median of adjacent marker and joint reference positions in the
% segment coordinate systems
head.relativePosition.GLA = median(headRelativePositionGLA)';
head.relativePosition.TRA_L = median(headRelativePositionTRA_L)';
head.relativePosition.TRA_R = median(headRelativePositionTRA_R)';
torso.relativePosition.ACR_L = median(torsoRelativePositionACR_L)';
torso.relativePosition.ACR_R = median(torsoRelativePositionACR_R)';
torso.relativePosition.SUP = median(torsoRelativePositionSUP)';
torso.relativePosition.C7 = median(torsoRelativePositionC7)';
torso.relativePosition.T8 = median(torsoRelativePositionT8)';
torso.relativePosition.T12 = median(torsoRelativePositionT12)';
torso.relativePosition.SJ_L = median(torsoRelativePositionSJ_L)';
torso.relativePosition.SJ_R = median(torsoRelativePositionSJ_R)';
torso.relativePosition.LLJ = median(torsoRelativePositionLLJ)';
upperArm_L.relativePosition.LHC_L = median(upperArmRelativePositionLHC_L)';
upperArm_R.relativePosition.LHC_R = median(upperArmRelativePositionLHC_R)';
upperArm_L.relativePosition.EJ_L = median(upperArmRelativePositionEJ_L)';
upperArm_R.relativePosition.EJ_R = median(upperArmRelativePositionEJ_R)';
lowerArm_L.relativePosition.LHC_L = median(lowerArmRelativePositionLHC_L)';
lowerArm_R.relativePosition.LHC_R = median(lowerArmRelativePositionLHC_R)';
lowerArm_L.relativePosition.WRI_L = median(lowerArmRelativePositionWRI_L)';
lowerArm_R.relativePosition.WRI_R = median(lowerArmRelativePositionWRI_R)';
pelvis.relativePosition.ASIS_L = median(pelvisRelativePositionASIS_L)';
pelvis.relativePosition.ASIS_R = median(pelvisRelativePositionASIS_R)';
pelvis.relativePosition.PSIS_L = median(pelvisRelativePositionPSIS_L)';
pelvis.relativePosition.PSIS_R = median(pelvisRelativePositionPSIS_R)';
pelvis.relativePosition.PS = median(pelvisRelativePositionPS)';
pelvis.relativePosition.HJ_L = median(pelvisRelativePositionHJ_L)';
pelvis.relativePosition.HJ_R = median(pelvisRelativePositionHJ_R)';
thigh_L.relativePosition.GTR_L = median(thighRelativePositionGTR_L)';
thigh_R.relativePosition.GTR_R = median(thighRelativePositionGTR_R)';
thigh_L.relativePosition.LFC_L = median(thighRelativePositionLFC_L)';
thigh_R.relativePosition.LFC_R = median(thighRelativePositionLFC_R)';
thigh_L.relativePosition.MFC_L = median(thighRelativePositionMFC_L)';
thigh_R.relativePosition.MFC_R = median(thighRelativePositionMFC_R)';
thigh_L.relativePosition.KJ_L = median(thighRelativePositionKJ_L)';
thigh_R.relativePosition.KJ_R = median(thighRelativePositionKJ_R)';
shank_L.relativePosition.LM_L = median(shankRelativePositionLM_L)';
shank_R.relativePosition.LM_R = median(shankRelativePositionLM_R)';
shank_L.relativePosition.MM_L = median(shankRelativePositionMM_L)';
shank_R.relativePosition.MM_R = median(shankRelativePositionMM_R)';
shank_L.relativePosition.AJ_L = median(shankRelativePositionAJ_L)';
shank_R.relativePosition.AJ_R = median(shankRelativePositionAJ_R)';
foot_L.relativePosition.CAL_L = median(footRelativePositionCAL_L)';
foot_R.relativePosition.CAL_R = median(footRelativePositionCAL_R)';
foot_L.relativePosition.MT2_L = median(footRelativePositionMT2_L)';
foot_R.relativePosition.MT2_R = median(footRelativePositionMT2_R)';
foot_L.relativePosition.MT5_L = median(footRelativePositionMT5_L)';
foot_R.relativePosition.MT5_R = median(footRelativePositionMT5_R)';
clear headRelativePositionGLA headRelativePositionTRA_L headRelativePositionTRA_R ...
    torsoRelativePositionACR_L torsoRelativePositionACR_R torsoRelativePositionSUP ...
    torsoRelativePositionC7 torsoRelativePositionT8 torsoRelativePositionT12 ...
    torsoRelativePositionSJ_L torsoRelativePositionSJ_R torsoRelativePositionLLJ ...
    upperArmRelativePositionLHC_L upperArmRelativePositionLHC_R upperArmRelativePositionEJ_L ...
    upperArmRelativePositionEJ_R lowerArmRelativePositionLHC_L lowerArmRelativePositionLHC_R ...
    lowerArmRelativePositionWRI_L lowerArmRelativePositionWRI_R pelvisRelativePositionASIS_L ...
    pelvisRelativePositionASIS_R pelvisRelativePositionPSIS_L pelvisRelativePositionPSIS_R ...
    pelvisRelativePositionPS  pelvisRelativePositionHJ_L pelvisRelativePositionHJ_R ...
    thighRelativePositionGTR_L thighRelativePositionGTR_R thighRelativePositionLFC_L ...
    thighRelativePositionLFC_R thighRelativePositionMFC_L thighRelativePositionMFC_R ....
    thighRelativePositionKJ_L thighRelativePositionKJ_R thighRelativePositionAJ_L ...
    thighRelativePositionAJ_R shankRelativePositionLM_L shankRelativePositionLM_R ...
    shankRelativePositionMM_L shankRelativePositionMM_R shankRelativePositionAJ_L ...
    shankRelativePositionAJ_R footRelativePositionCAL_L footRelativePositionCAL_R ...
    footRelativePositionMT2_L footRelativePositionMT2_R footRelativePositionMT5_L ...
    footRelativePositionMT5_R;

% Estimate segment masses scaled by segment lengths according to [Dumas2007]
if(strcmp(gender, 'female'))
    head.mass = 0.067 * bodyMass;
    upperArm_L.mass = 2 * 0.022 * bodyMass * upperArm_L.segmentLengthY / (upperArm_L.segmentLengthY + upperArm_R.segmentLengthY);
    upperArm_R.mass = 2 * 0.022 * bodyMass * upperArm_R.segmentLengthY / (upperArm_L.segmentLengthY + upperArm_R.segmentLengthY);
    lowerArm_L.mass = 2 * 0.013 * bodyMass * lowerArm_L.segmentLengthY / (lowerArm_L.segmentLengthY + lowerArm_R.segmentLengthY);
    lowerArm_R.mass = 2 * 0.013 * bodyMass * lowerArm_R.segmentLengthY / (lowerArm_L.segmentLengthY + lowerArm_R.segmentLengthY);
    hand_L.mass = 0.005 * bodyMass;
    hand_R.mass = 0.005 * bodyMass;
    torso.mass = 0.304 * bodyMass;
    pelvis.mass = 0.146 * bodyMass;
    thigh_L.mass = 2 * 0.146 * bodyMass * thigh_L.segmentLengthY / (thigh_L.segmentLengthY + thigh_R.segmentLengthY);
    thigh_R.mass = 2 * 0.146 * bodyMass * thigh_R.segmentLengthY / (thigh_L.segmentLengthY + thigh_R.segmentLengthY);
    shank_L.mass = 2 * 0.045 * bodyMass * shank_L.segmentLengthY / (shank_L.segmentLengthY + shank_R.segmentLengthY);
    shank_R.mass = 2 * 0.045 * bodyMass * shank_R.segmentLengthY / (shank_L.segmentLengthY + shank_R.segmentLengthY);
    foot_L.mass = 2 * 0.01 * bodyMass * foot_L.referenceLength / (foot_L.referenceLength + foot_R.referenceLength);
    foot_R.mass = 2 * 0.01 * bodyMass * foot_R.referenceLength / (foot_L.referenceLength + foot_R.referenceLength);
else
    head.mass = 0.067 * bodyMass;
    upperArm_L.mass = 2 * 0.024 * bodyMass * upperArm_L.segmentLengthY / (upperArm_L.segmentLengthY + upperArm_R.segmentLengthY);
    upperArm_R.mass = 2 * 0.024 * bodyMass * upperArm_R.segmentLengthY / (upperArm_L.segmentLengthY + upperArm_R.segmentLengthY);
    lowerArm_L.mass = 2 * 0.017 * bodyMass * lowerArm_L.segmentLengthY / (lowerArm_L.segmentLengthY + lowerArm_R.segmentLengthY);
    lowerArm_R.mass = 2 * 0.017 * bodyMass * lowerArm_R.segmentLengthY / (lowerArm_L.segmentLengthY + lowerArm_R.segmentLengthY);
    hand_L.mass = 0.006 * bodyMass;
    hand_R.mass = 0.006 * bodyMass;
    torso.mass = 0.333 * bodyMass;
    pelvis.mass = 0.142 * bodyMass;
    thigh_L.mass = 2 * 0.123 * bodyMass * thigh_L.segmentLengthY / (thigh_L.segmentLengthY + thigh_R.segmentLengthY);
    thigh_R.mass = 2 * 0.123 * bodyMass * thigh_R.segmentLengthY / (thigh_L.segmentLengthY + thigh_R.segmentLengthY);
    shank_L.mass = 2 * 0.048 * bodyMass * shank_L.segmentLengthY / (shank_L.segmentLengthY + shank_R.segmentLengthY);
    shank_R.mass = 2 * 0.048 * bodyMass * shank_R.segmentLengthY / (shank_L.segmentLengthY + shank_R.segmentLengthY);
    foot_L.mass = 2 * 0.012 * bodyMass * foot_L.referenceLength / (foot_L.referenceLength + foot_R.referenceLength);
    foot_R.mass = 2 * 0.012 * bodyMass * foot_R.referenceLength / (foot_L.referenceLength + foot_R.referenceLength);
end

% Estimate center of mass (COM) coordinates according to [Dumas2007]
if(strcmp(gender, 'female'))
    head.comX = -0.07 * head.segmentLengthY;
    head.comY = 0.597 * head.segmentLengthY;
    head.comZ = 0;
    upperArm_L.comX = -0.073 * upperArm_L.segmentLengthY;
    upperArm_L.comY = -0.454 * upperArm_L.segmentLengthY;
    upperArm_L.comZ = -0.028 * upperArm_L.segmentLengthY;
    upperArm_R.comX = -0.073 * upperArm_R.segmentLengthY;
    upperArm_R.comY = -0.454 * upperArm_R.segmentLengthY;
    upperArm_R.comZ = -0.028 * upperArm_R.segmentLengthY;
    lowerArm_L.comX = 0.021 * lowerArm_L.segmentLengthY;
    lowerArm_L.comY = -0.411 * lowerArm_L.segmentLengthY;
    lowerArm_L.comZ = 0.019 * lowerArm_L.segmentLengthY;
    lowerArm_R.comX = 0.021 * lowerArm_R.segmentLengthY;
    lowerArm_R.comY = -0.411 * lowerArm_R.segmentLengthY;
    lowerArm_R.comZ = 0.019 * lowerArm_R.segmentLengthY;
    hand_L.comX = 0.033 * hand_L.segmentLengthY;
    hand_L.comY = -0.327 * hand_L.segmentLengthY;
    hand_L.comZ = 0.021 * hand_L.segmentLengthY;
    hand_R.comX = 0.033 * hand_R.segmentLengthY;
    hand_R.comY = -0.327 * hand_R.segmentLengthY;
    hand_R.comZ = 0.021 * hand_R.segmentLengthY;
    torso.comX = -0.016 * torso.segmentLengthY;
    torso.comY = -0.436 * torso.segmentLengthY;
    torso.comZ = -0.006 * torso.segmentLengthY;
    pelvis.comX = -0.009 * pelvis.segmentLengthY;
    pelvis.comY = -0.232 * pelvis.segmentLengthY;
    pelvis.comZ = 0.002 * pelvis.segmentLengthY;
    thigh_L.comX = -0.077 * thigh_L.segmentLengthY;
    thigh_L.comY = -0.377 * thigh_L.segmentLengthY;
    thigh_L.comZ = 0.009 * thigh_L.segmentLengthY;
    thigh_R.comX = -0.077 * thigh_R.segmentLengthY;
    thigh_R.comY = -0.377 * thigh_R.segmentLengthY;
    thigh_R.comZ = 0.009 * thigh_R.segmentLengthY;
    shank_L.comX = -0.049 * shank_L.segmentLengthY;
    shank_L.comY = -0.404 * shank_L.segmentLengthY;
    shank_L.comZ = 0.031 * shank_L.segmentLengthY;
    shank_R.comX = -0.049 * shank_R.segmentLengthY;
    shank_R.comY = -0.404 * shank_R.segmentLengthY;
    shank_R.comZ = 0.031 * shank_R.segmentLengthY;
    foot_L.comX = 0.443 * foot_L.referenceLength;
    foot_L.comY = 0.044 * foot_L.referenceLength;
    foot_L.comZ = -0.025 * foot_L.referenceLength;
    foot_R.comX = 0.443 * foot_R.referenceLength;
    foot_R.comY = 0.044 * foot_R.referenceLength;
    foot_R.comZ = -0.025 * foot_R.referenceLength;
else
    head.comX = -0.062 * head.segmentLengthY;
    head.comY = 0.555 * head.segmentLengthY;
    head.comZ = 0.001 * head.segmentLengthY;
    upperArm_L.comX = 0.017 * upperArm_L.segmentLengthY;
    upperArm_L.comY = -0.452 * upperArm_L.segmentLengthY;
    upperArm_L.comZ = -0.026 * upperArm_L.segmentLengthY;
    upperArm_R.comX = 0.017 * upperArm_R.segmentLengthY;
    upperArm_R.comY = -0.452 * upperArm_R.segmentLengthY;
    upperArm_R.comZ = -0.026 * upperArm_R.segmentLengthY;
    lowerArm_L.comX = 0.01 * lowerArm_L.segmentLengthY;
    lowerArm_L.comY = -0.417 * lowerArm_L.segmentLengthY;
    lowerArm_L.comZ = 0.014 * lowerArm_L.segmentLengthY;
    lowerArm_R.comX = 0.01 * lowerArm_R.segmentLengthY;
    lowerArm_R.comY = -0.417 * lowerArm_R.segmentLengthY;
    lowerArm_R.comZ = 0.014 * lowerArm_R.segmentLengthY;
    hand_L.comX = 0.035 * hand_L.segmentLengthY;
    hand_L.comY = -0.357 * hand_L.segmentLengthY;
    hand_L.comZ = 0.032 * hand_L.segmentLengthY;
    hand_R.comX = 0.035 * hand_R.segmentLengthY;
    hand_R.comY = -0.357 * hand_R.segmentLengthY;
    hand_R.comZ = 0.032 * hand_R.segmentLengthY;
    torso.comX = -0.036 * torso.segmentLengthY;
    torso.comY = -0.42 * torso.segmentLengthY;
    torso.comZ = -0.002 * torso.segmentLengthY;
    pelvis.comX = 0.028 * pelvis.segmentLengthY;
    pelvis.comY = -0.28 * pelvis.segmentLengthY;
    pelvis.comZ = -0.006 * pelvis.segmentLengthY;
    thigh_L.comX = -0.041 * thigh_L.segmentLengthY;
    thigh_L.comY = -0.429 * thigh_L.segmentLengthY;
    thigh_L.comZ = 0.033 * thigh_L.segmentLengthY;
    thigh_R.comX = -0.041 * thigh_R.segmentLengthY;
    thigh_R.comY = -0.429 * thigh_R.segmentLengthY;
    thigh_R.comZ = 0.033 * thigh_R.segmentLengthY;
    shank_L.comX = -0.048 * shank_L.segmentLengthY;
    shank_L.comY = -0.41 * shank_L.segmentLengthY;
    shank_L.comZ = 0.007 * shank_L.segmentLengthY;
    shank_R.comX = -0.048 * shank_R.segmentLengthY;
    shank_R.comY = -0.41 * shank_R.segmentLengthY;
    shank_R.comZ = 0.007 * shank_R.segmentLengthY;
    foot_L.comX = 0.436 * foot_L.referenceLength;
    foot_L.comY = -0.025 * foot_L.referenceLength;
    foot_L.comZ = -0.007 * foot_L.referenceLength;
    foot_R.comX = 0.436 * foot_R.referenceLength;
    foot_R.comY = -0.025 * foot_R.referenceLength;
    foot_R.comZ = -0.007 * foot_R.referenceLength;
end

% Estimate moments and products of inertia with respect to the center of
% mass according to [Dumas2007]
if(strcmp(gender, 'female'))
    head.inertiaXX = (0.32 * head.segmentLengthY / 1000)^2 * head.mass;
    head.inertiaYY = (0.27 * head.segmentLengthY / 1000)^2 * head.mass;
    head.inertiaZZ = (0.34 * head.segmentLengthY / 1000)^2 * head.mass;
    head.inertiaXY = -(0.06 * head.segmentLengthY / 1000)^2 * head.mass;
    head.inertiaXZ = (0.01 * head.segmentLengthY / 1000)^2 * head.mass;
    head.inertiaYZ = -(0.01 * head.segmentLengthY / 1000)^2 * head.mass;
    upperArm_L.inertiaXX = (0.33 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.inertiaYY = (0.17 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.inertiaZZ = (0.33 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.inertiaXY = (0.03 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.inertiaXZ = -(0.05 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.inertiaYZ = -(0.14 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_R.inertiaXX = (0.33 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.inertiaYY = (0.17 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.inertiaZZ = (0.33 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.inertiaXY = (0.03 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.inertiaXZ = -(0.05 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.inertiaYZ = -(0.14 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    lowerArm_L.inertiaXX = (0.26 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.inertiaYY = (0.14 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.inertiaZZ = (0.25 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.inertiaXY = (0.1 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.inertiaXZ = (0.04 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.inertiaYZ = -(0.13 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_R.inertiaXX = (0.26 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.inertiaYY = (0.14 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.inertiaZZ = (0.25 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.inertiaXY = (0.1 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.inertiaXZ = (0.04 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.inertiaYZ = -(0.13 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    hand_L.inertiaXX = (0.27 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.inertiaYY = (0.18 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.inertiaZZ = (0.25 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.inertiaXY = (0.12 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.inertiaXZ = (0.1 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.inertiaYZ = -(0.12 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_R.inertiaXX = (0.27 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.inertiaYY = (0.18 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.inertiaZZ = (0.25 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.inertiaXY = (0.12 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.inertiaXZ = (0.1 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.inertiaYZ = -(0.12 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    torso.inertiaXX = (0.29 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.inertiaYY = (0.27 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.inertiaZZ = (0.29 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.inertiaXY = (0.22 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.inertiaXZ = (0.05 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.inertiaYZ = -(0.05 * torso.segmentLengthY / 1000)^2 * torso.mass;
    pelvis.inertiaXX = (0.91 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.inertiaYY = (pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.inertiaZZ = (0.79 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.inertiaXY = -(0.34 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.inertiaXZ = -(0.01 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.inertiaYZ = -(0.01 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    thigh_L.inertiaXX = (0.31 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.inertiaYY = (0.19 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.inertiaZZ = (0.32 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.inertiaXY = (0.07 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.inertiaXZ = -(0.02 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.inertiaYZ = -(0.07 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_R.inertiaXX = (0.31 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.inertiaYY = (0.19 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.inertiaZZ = (0.32 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.inertiaXY = (0.07 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.inertiaXZ = -(0.02 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.inertiaYZ = -(0.07 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    shank_L.inertiaXX = (0.28 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.inertiaYY = (0.1 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.inertiaZZ = (0.28 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.inertiaXY = (0.02 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.inertiaXZ = (0.01 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.inertiaYZ = (0.06 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_R.inertiaXX = (0.28 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.inertiaYY = (0.1 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.inertiaZZ = (0.28 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.inertiaXY = (0.02 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.inertiaXZ = (0.01 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.inertiaYZ = (0.06 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    foot_L.inertiaXX = (0.12 * foot_L.referenceLength / 1000)^2 * foot_L.mass;
    foot_L.inertiaYY = (0.25 * foot_L.referenceLength / 1000)^2 * foot_L.mass;
    foot_L.inertiaZZ = (0.25 * foot_L.referenceLength / 1000)^2 * foot_L.mass;
    foot_L.inertiaXY = -(0.07 * foot_L.referenceLength / 1000)^2 * foot_L.mass;
    foot_L.inertiaXZ = (0.05 * foot_L.referenceLength / 1000)^2 * foot_L.mass;
    foot_L.inertiaYZ = -(0.03 * foot_L.referenceLength / 1000)^2 * foot_L.mass;
    foot_R.inertiaXX = (0.12 * foot_R.referenceLength / 1000)^2 * foot_R.mass;
    foot_R.inertiaYY = (0.25 * foot_R.referenceLength / 1000)^2 * foot_R.mass;
    foot_R.inertiaZZ = (0.25 * foot_R.referenceLength / 1000)^2 * foot_R.mass;
    foot_R.inertiaXY = -(0.07 * foot_R.referenceLength / 1000)^2 * foot_R.mass;
    foot_R.inertiaXZ = (0.05 * foot_R.referenceLength / 1000)^2 * foot_R.mass;
    foot_R.inertiaYZ = -(0.03 * foot_R.referenceLength / 1000)^2 * foot_R.mass;
else
    head.inertiaXX = (0.31 * head.segmentLengthY / 1000)^2 * head.mass;
    head.inertiaYY = (0.25 * head.segmentLengthY / 1000)^2 * head.mass;
    head.inertiaZZ = (0.33 * head.segmentLengthY / 1000)^2 * head.mass;
    head.inertiaXY = -(0.09 * head.segmentLengthY / 1000)^2 * head.mass;
    head.inertiaXZ = -(0.02 * head.segmentLengthY / 1000)^2 * head.mass;
    head.inertiaYZ = (0.03 * head.segmentLengthY / 1000)^2 * head.mass;
    upperArm_L.inertiaXX = (0.31 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.inertiaYY = (0.14 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.inertiaZZ = (0.32 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.inertiaXY = (0.06 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.inertiaXZ = (0.05 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.inertiaYZ = (0.02 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_R.inertiaXX = (0.31 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.inertiaYY = (0.14 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.inertiaZZ = (0.32 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.inertiaXY = (0.06 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.inertiaXZ = (0.05 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.inertiaYZ = (0.02 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    lowerArm_L.inertiaXX = (0.28 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.inertiaYY = (0.11 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.inertiaZZ = (0.27 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.inertiaXY = (0.03 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.inertiaXZ = (0.02 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.inertiaYZ = -(0.08 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_R.inertiaXX = (0.28 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.inertiaYY = (0.11 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.inertiaZZ = (0.27 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.inertiaXY = (0.03 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.inertiaXZ = (0.02 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.inertiaYZ = -(0.08 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    hand_L.inertiaXX = (0.26 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.inertiaYY = (0.16 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.inertiaZZ = (0.24 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.inertiaXY = (0.09 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.inertiaXZ = (0.07 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.inertiaYZ = -(0.08 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_R.inertiaXX = (0.26 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.inertiaYY = (0.16 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.inertiaZZ = (0.24 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.inertiaXY = (0.09 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.inertiaXZ = (0.07 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.inertiaYZ = -(0.08 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    torso.inertiaXX = (0.27 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.inertiaYY = (0.25 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.inertiaZZ = (0.28 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.inertiaXY = (0.18 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.inertiaXZ = (0.02 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.inertiaYZ = -(0.04 * torso.segmentLengthY / 1000)^2 * torso.mass;
    pelvis.inertiaXX = (1.01 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.inertiaYY = (1.06 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.inertiaZZ = (0.95 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.inertiaXY = -(0.25 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.inertiaXZ = -(0.12 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.inertiaYZ = -(0.08 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    thigh_L.inertiaXX = (0.29 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.inertiaYY = (0.15 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.inertiaZZ = (0.3 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.inertiaXY = (0.07 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.inertiaXZ = -(0.02 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.inertiaYZ = -(0.07 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_R.inertiaXX = (0.29 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.inertiaYY = (0.15 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.inertiaZZ = (0.3 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.inertiaXY = (0.07 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.inertiaXZ = -(0.02 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.inertiaYZ = -(0.07 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    shank_L.inertiaXX = (0.28 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.inertiaYY = (0.1 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.inertiaZZ = (0.28 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.inertiaXY = -(0.04 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.inertiaXZ = -(0.02 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.inertiaYZ = (0.05 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_R.inertiaXX = (0.28 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.inertiaYY = (0.1 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.inertiaZZ = (0.28 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.inertiaXY = -(0.04 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.inertiaXZ = -(0.02 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.inertiaYZ = (0.05 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    foot_L.inertiaXX = (0.11 * foot_L.referenceLength / 1000)^2 * foot_L.mass;
    foot_L.inertiaYY = (0.25 * foot_L.referenceLength / 1000)^2 * foot_L.mass;
    foot_L.inertiaZZ = (0.25 * foot_L.referenceLength / 1000)^2 * foot_L.mass;
    foot_L.inertiaXY = (0.09 * foot_L.referenceLength / 1000)^2 * foot_L.mass;
    foot_L.inertiaXZ = -(0.06 * foot_L.referenceLength / 1000)^2 * foot_L.mass;
    foot_L.inertiaYZ = 0;
    foot_R.inertiaXX = (0.11 * foot_R.referenceLength / 1000)^2 * foot_R.mass;
    foot_R.inertiaYY = (0.25 * foot_R.referenceLength / 1000)^2 * foot_R.mass;
    foot_R.inertiaZZ = (0.25 * foot_R.referenceLength / 1000)^2 * foot_R.mass;
    foot_R.inertiaXY = (0.09 * foot_R.referenceLength / 1000)^2 * foot_R.mass;
    foot_R.inertiaXZ = -(0.06 * foot_R.referenceLength / 1000)^2 * foot_R.mass;
    foot_R.inertiaYZ = 0;
end

% Combine lower arm and hand segment by assuming the hand segment as a
% point mass at the wrist marker (WRI)
lowerArm_L.inertiaXX = lowerArm_L.inertiaXX + (lowerArm_L.segmentLengthY  / 1000)^2 * hand_L.mass;
lowerArm_L.inertiaZZ = lowerArm_L.inertiaZZ + (lowerArm_L.segmentLengthY / 1000)^2 * hand_L.mass;
lowerArm_R.inertiaXX = lowerArm_R.inertiaXX + (lowerArm_R.segmentLengthY / 1000)^2 * hand_R.mass;
lowerArm_R.inertiaZZ = lowerArm_R.inertiaZZ + (lowerArm_R.segmentLengthY / 1000)^2 * hand_R.mass;
lowerArm_L.comY = (lowerArm_L.comY * lowerArm_L.mass - lowerArm_L.segmentLengthY * hand_L.mass) / (lowerArm_L.mass + hand_L.mass);
lowerArm_R.comY = (lowerArm_R.comY * lowerArm_R.mass - lowerArm_R.segmentLengthY * hand_R.mass) / (lowerArm_R.mass + hand_R.mass);
lowerArm_L.mass = lowerArm_L.mass + hand_L.mass;
lowerArm_R.mass = lowerArm_R.mass + hand_R.mass;
clear hand_L hand_R;

save(saveFile, 'subject', 'age', 'gender', 'origin', 'bodyHeight', 'bodyMass', 'joints', 'head', 'torso', 'upperArm_L', 'upperArm_R', 'lowerArm_L', 'lowerArm_R', 'pelvis', 'thigh_L', 'thigh_R', 'shank_L', 'shank_R', 'foot_L', 'foot_R');
fprintf('STATUS: Antropometric parameters were saved.\n');
