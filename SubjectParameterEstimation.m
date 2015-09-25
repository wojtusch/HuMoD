% ------------------------------------------------------
% This script estimates subject parameters based on segment lengths and on
% regression equations from [Dumas2007] in reference systems according to
% [Wu1995], [Wu2002], [Wu2005]. Lengths are given in millimeters, masses
% are given in kilogram and moments of inertia are given in kilogram *
% meters^2.
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
    bodyMass = 562 / 9.81;
    equipmentMass = 575 / 9.81 - bodyMass;
    age = 26.8;
    footLength_L = 230;
    footLength_R = 225;
else
    gender = 'male';
    origin = 'central european';
    bodyHeight = 1790;
    bodyMass = 832 / 9.81;
    equipmentMass = 845 / 9.81 - bodyMass;
    age = 32.1;
    footLength_L = 271;
    footLength_R = 275;
end
saveFile = [getPath, filesep, subject, filesep, 'Parameters.mat'];
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
footReferenceLengthX_L = zeros(length(datasets), 1);
footReferenceLengthY_L = zeros(length(datasets), 1);
footReferenceLengthZ_L = zeros(length(datasets), 1);
footReferenceLengthX_R = zeros(length(datasets), 1);
footReferenceLengthY_R = zeros(length(datasets), 1);
footReferenceLengthZ_R = zeros(length(datasets), 1);
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
absolutePositionULJ = zeros(length(datasets), 3);
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
    headSegmentLengthY(datasetIndex) = bodyHeight - [0, 1, 0] * (getAverageJoint('LNJ', 'estimatedJoint', initialStartFrame, initialEndFrame) - groundPosition);
    torsoSegmentLengthY(datasetIndex) = getAverageDistance('LLJ', 'estimatedJoint', 'LNJ', 'estimatedJoint', initialStartFrame, initialEndFrame);
    upperArmSegmentLengthY_L(datasetIndex) = getAverageDistance('EJ_L', 'estimatedJoint', 'SJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame);
    upperArmSegmentLengthY_R(datasetIndex) = getAverageDistance('EJ_R', 'estimatedJoint', 'SJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame);
    lowerArmSegmentLengthY_L(datasetIndex) = getAverageDistance('WRI_L', 'marker', 'EJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame);
    lowerArmSegmentLengthY_R(datasetIndex) = getAverageDistance('WRI_R', 'marker', 'EJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame);
    pelvisSegmentLengthY(datasetIndex) = norm((getAverageJoint('HJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame) + getAverageVector('HJ_L', 'estimatedJoint', 'HJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame) / 2) - getAverageJoint('LLJ', 'estimatedJoint', initialStartFrame, initialEndFrame));
    pelvisSegmentLengthZ(datasetIndex) = getAverageDistance('HJ_L', 'estimatedJoint', 'HJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame);
    thighSegmentLengthY_L(datasetIndex) = getAverageDistance('KJ_L', 'estimatedJoint', 'HJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame);
    thighSegmentLengthY_R(datasetIndex) = getAverageDistance('KJ_R', 'estimatedJoint', 'HJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame);
    shankSegmentLengthY_L(datasetIndex) = getAverageDistance('AJ_L', 'estimatedJoint', 'KJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame);
    shankSegmentLengthY_R(datasetIndex) = getAverageDistance('AJ_R', 'estimatedJoint', 'KJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame);
    footReferenceVector = vectorMT_L - getAverageJoint('AJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame);
    footReferenceLengthX_L(datasetIndex) = footReferenceVector(1);
    footReferenceLengthY_L(datasetIndex) = footReferenceVector(2);
    footReferenceLengthZ_L(datasetIndex) = footReferenceVector(3);
    footReferenceVector = vectorMT_R - getAverageJoint('AJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame);
    footReferenceLengthX_R(datasetIndex) = footReferenceVector(1);
    footReferenceLengthY_R(datasetIndex) = footReferenceVector(2);
    footReferenceLengthZ_R(datasetIndex) = footReferenceVector(3);
    footSegmentLengthY_L(datasetIndex) = [0, 1, 0] * (getAverageJoint('AJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame) - groundPosition);
    footSegmentLengthY_R(datasetIndex) = [0, 1, 0] * (getAverageJoint('AJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame) - groundPosition);
    clear footReferenceVector

    % Compute average absolute joint positions for each dataset
    absolutePositionLNJ(datasetIndex, :) = getAverageJoint('LNJ', 'estimatedJoint', initialStartFrame, initialEndFrame)';
    absolutePositionSJ_L(datasetIndex, :) = getAverageJoint('SJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame)';
    absolutePositionSJ_R(datasetIndex, :) = getAverageJoint('SJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame)';
    absolutePositionEJ_L(datasetIndex, :) = getAverageJoint('EJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame)';
    absolutePositionEJ_R(datasetIndex, :) = getAverageJoint('EJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame)';
    absolutePositionULJ(datasetIndex, :) = getAverageJoint('ULJ', 'estimatedJoint', initialStartFrame, initialEndFrame)';
    absolutePositionLLJ(datasetIndex, :) = getAverageJoint('LLJ', 'estimatedJoint', initialStartFrame, initialEndFrame)';
    absolutePositionHJ_L(datasetIndex, :) = getAverageJoint('HJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame)';
    absolutePositionHJ_R(datasetIndex, :) = getAverageJoint('HJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame)';
    absolutePositionKJ_L(datasetIndex, :) = getAverageJoint('KJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame)';
    absolutePositionKJ_R(datasetIndex, :) = getAverageJoint('KJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame)';
    absolutePositionAJ_L(datasetIndex, :) = getAverageJoint('AJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame)';
    absolutePositionAJ_R(datasetIndex, :) = getAverageJoint('AJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame)';
    
    % Estimate average reference positions of adjacent markers and joints
    % in the head reference frame for each dataset derived from definitions
    % in [Dumas2007]. The y-axis is assumed to be perpendicular to the
    % plane containing the GLA, TRA_L and TRA_R markers pointing distal.
    % The z-axis is perpendicular to the y-axis and the line connecting the
    % lower neck joint and the GLA marker pointing right. The x-axis is the
    % common line perpendicular to the y- and z-axis pointing anterior.
    % Origin is the lower neck joint.
    yAxis = cross(getAverageVector('GLA', 'surface', 'TRA_L', 'surface', initialStartFrame, initialEndFrame), getAverageVector('GLA', 'surface', 'TRA_R', 'surface', initialStartFrame, initialEndFrame));
    yAxis = yAxis / norm(yAxis);
    zAxis = cross(getAverageVector('LNJ', 'estimatedJoint', 'GLA', 'surface', initialStartFrame, initialEndFrame), yAxis);
    zAxis = zAxis / norm(zAxis);
    xAxis = cross(yAxis, zAxis);
    xAxis = xAxis / norm(xAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    headRelativePositionGLA(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'estimatedJoint', 'GLA', 'surface', initialStartFrame, initialEndFrame))';
    headRelativePositionTRA_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'estimatedJoint', 'TRA_L', 'surface', initialStartFrame, initialEndFrame))';
    headRelativePositionTRA_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'estimatedJoint', 'TRA_R', 'surface', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the torso reference frame for each dataset based on definitions in
    % [Dumas2007]. The y-axis is given by connecting the knee and ankle
    % joints pointing superior. The z-axis is perpendicular to the y-axis
    % and the line connecting the lower neck joint and the SUP marker
    % pointing right. The x-axis is the common line perpendicular to the y-
    % and z-axis pointing anterior. Origin is the lower neck joint.
    yAxis = getAverageVector('LLJ', 'estimatedJoint', 'LNJ', 'estimatedJoint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    zAxis = cross(getAverageVector('LNJ', 'estimatedJoint', 'SUP', 'surface', initialStartFrame, initialEndFrame), yAxis);
    zAxis = zAxis / norm(zAxis);
    xAxis = cross(yAxis, zAxis);
    xAxis = xAxis / norm(xAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    torsoRelativePositionACR_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'estimatedJoint', 'ACR_L', 'surface', initialStartFrame, initialEndFrame))';
    torsoRelativePositionACR_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'estimatedJoint', 'ACR_R', 'surface', initialStartFrame, initialEndFrame))';
    torsoRelativePositionSUP(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'estimatedJoint', 'SUP', 'surface', initialStartFrame, initialEndFrame))';
    torsoRelativePositionC7(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'estimatedJoint', 'C7', 'surface', initialStartFrame, initialEndFrame))';
    torsoRelativePositionT8(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'estimatedJoint', 'T8', 'surface', initialStartFrame, initialEndFrame))';
    torsoRelativePositionT12(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'estimatedJoint', 'T12', 'surface', initialStartFrame, initialEndFrame))';
    torsoRelativePositionSJ_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'estimatedJoint', 'SJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame))';
    torsoRelativePositionSJ_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'estimatedJoint', 'SJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame))';
    torsoRelativePositionLLJ(datasetIndex, :) = (rotationMatrix \ getAverageVector('LNJ', 'estimatedJoint', 'LLJ', 'estimatedJoint', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the left and right upper arm reference frame for each dataset
    % based on definitions in [Dumas2007], [Wu2005]. The y-axis is given by
    % connecting the shoulder and elbow joints pointing proximal. The x-
    % axis is perpendicular to the y-axis and the line connecting the elbow
    % joint and the LHC marker pointing anterior.  The z-axis is the common
    % line perpendicular to the x- and y-axis pointing right. Origin is the
    % shoulder joint.
    yAxis = getAverageVector('EJ_L', 'estimatedJoint', 'SJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    xAxis = cross(getAverageVector('EJ_L', 'estimatedJoint', 'LHC_L', 'surface', initialStartFrame, initialEndFrame), yAxis);
    xAxis = xAxis / norm(xAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    upperArmRelativePositionLHC_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('SJ_L', 'estimatedJoint', 'LHC_L', 'surface', initialStartFrame, initialEndFrame))';
    upperArmRelativePositionEJ_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('SJ_L', 'estimatedJoint', 'EJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame))';
    yAxis = getAverageVector('EJ_R', 'estimatedJoint', 'SJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    xAxis = cross(getAverageVector('LHC_R', 'surface', 'EJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame), yAxis);
    xAxis = xAxis / norm(xAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    upperArmRelativePositionLHC_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('SJ_R', 'estimatedJoint', 'LHC_R', 'surface', initialStartFrame, initialEndFrame))';
    upperArmRelativePositionEJ_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('SJ_R', 'estimatedJoint', 'EJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the left and right lower arm reference frame for each dataset
    % derived from definitions in [Dumas2007], [Wu2005]. The y-axis is
    % given by connecting the WRI marker and the elbow joint pointing
    % proximal. The x-axis is perpendicular to the y-axis and the line
    % connecting the elbow joint and the LHC marker pointing anterior. The
    % z-axis is the common line perpendicular to the x- and y-axis pointing
    % right. Origin is the elbow joint.
    yAxis = getAverageVector('WRI_L', 'marker', 'EJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    xAxis = cross(getAverageVector('EJ_L', 'estimatedJoint', 'LHC_L', 'surface', initialStartFrame, initialEndFrame), yAxis);
    xAxis = xAxis / norm(xAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    lowerArmRelativePositionLHC_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('EJ_L', 'estimatedJoint', 'LHC_L', 'surface', initialStartFrame, initialEndFrame))';
    lowerArmRelativePositionWRI_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('EJ_L', 'estimatedJoint', 'WRI_L', 'marker', initialStartFrame, initialEndFrame))';
    yAxis = getAverageVector('WRI_R', 'marker', 'EJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    xAxis = cross(getAverageVector('LHC_R', 'surface', 'EJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame), yAxis);
    xAxis = xAxis / norm(xAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    lowerArmRelativePositionLHC_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('EJ_R', 'estimatedJoint', 'LHC_R', 'surface', initialStartFrame, initialEndFrame))';
    lowerArmRelativePositionWRI_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('EJ_R', 'estimatedJoint', 'WRI_R', 'marker', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the pelvis reference frame for each dataset based on definitions
    % in [Dumas2007], [Wu2002]. The z-axis is given by connecting the
    % ASIS_L and ASIS_R markers pointing right. The x-axis is given by
    % connecting the midpoint of the ASIS_L and ASIS_R markers with the
    % midpoint of the PSIS_L and PSIS_R markers pointing anterior. The
    % y-axis is the common line perpendicular to the x- and z-axis pointing
    % superior. Origin is the lower lumbar joint.
    zAxis = getAverageVector('ASIS_L', 'surface', 'ASIS_R', 'surface', initialStartFrame, initialEndFrame);
    zAxis = zAxis / norm(zAxis);
    xAxis = (getAverageMarker('ASIS_L', 'surface', initialStartFrame, initialEndFrame) + getAverageVector('ASIS_L', 'surface', 'ASIS_R', 'surface', initialStartFrame, initialEndFrame) / 2) - (getAverageMarker('PSIS_L', 'surface', initialStartFrame, initialEndFrame) + getAverageVector('PSIS_L', 'surface', 'PSIS_R', 'surface', initialStartFrame, initialEndFrame) / 2);
    xAxis = xAxis / norm(xAxis);
    yAxis = cross(zAxis, xAxis);
    yAxis = yAxis / norm(yAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    pelvisRelativePositionASIS_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('LLJ', 'estimatedJoint', 'ASIS_L', 'surface', initialStartFrame, initialEndFrame))';
    pelvisRelativePositionASIS_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('LLJ', 'estimatedJoint', 'ASIS_R', 'surface', initialStartFrame, initialEndFrame))';
    pelvisRelativePositionPSIS_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('LLJ', 'estimatedJoint', 'PSIS_L', 'surface', initialStartFrame, initialEndFrame))';
    pelvisRelativePositionPSIS_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('LLJ', 'estimatedJoint', 'PSIS_R', 'surface', initialStartFrame, initialEndFrame))';
    pelvisRelativePositionPS(datasetIndex, :) = (rotationMatrix \ getAverageVector('LLJ', 'estimatedJoint', 'PS', 'surface', initialStartFrame, initialEndFrame))';
    pelvisRelativePositionHJ_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('LLJ', 'estimatedJoint', 'HJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame))';
    pelvisRelativePositionHJ_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('LLJ', 'estimatedJoint', 'HJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the left and right thigh reference frame for each dataset based on
    % definitions in [Dumas2007], [Wu2002]. The x-axis is perpendicular to
    % the plane containing the hip joint and the MFC and LFC markers
    % pointing anterior. The y-axis is given by connecting the hip and knee
    % joints pointing proximal. The z-axis is the common line perpendicular
    % to the x- and y-axis pointing right. Origin is the hip joint.
    xAxis = cross(getAverageVector('HJ_L', 'estimatedJoint', 'MFC_L', 'surface', initialStartFrame, initialEndFrame), getAverageVector('HJ_L', 'estimatedJoint', 'LFC_L', 'surface', initialStartFrame, initialEndFrame));
    xAxis = xAxis / norm(xAxis);
    yAxis = getAverageVector('KJ_L', 'estimatedJoint', 'HJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    thighRelativePositionGTR_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('HJ_L', 'estimatedJoint', 'GTR_L', 'surface', initialStartFrame, initialEndFrame))';
    thighRelativePositionLFC_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('HJ_L', 'estimatedJoint', 'LFC_L', 'surface', initialStartFrame, initialEndFrame))';
    thighRelativePositionMFC_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('HJ_L', 'estimatedJoint', 'MFC_L', 'surface', initialStartFrame, initialEndFrame))';
    thighRelativePositionKJ_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('HJ_L', 'estimatedJoint', 'KJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame))';
    xAxis = cross(getAverageVector('HJ_R', 'estimatedJoint', 'LFC_R', 'surface', initialStartFrame, initialEndFrame), getAverageVector('HJ_R', 'estimatedJoint', 'MFC_R', 'surface', initialStartFrame, initialEndFrame));
    xAxis = xAxis / norm(xAxis);
    yAxis = getAverageVector('KJ_R', 'estimatedJoint', 'HJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    thighRelativePositionGTR_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('HJ_R', 'estimatedJoint', 'GTR_R', 'surface', initialStartFrame, initialEndFrame))';
    thighRelativePositionLFC_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('HJ_R', 'estimatedJoint', 'LFC_R', 'surface', initialStartFrame, initialEndFrame))';
    thighRelativePositionMFC_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('HJ_R', 'estimatedJoint', 'MFC_R', 'surface', initialStartFrame, initialEndFrame))';
    thighRelativePositionKJ_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('HJ_R', 'estimatedJoint', 'KJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the left and right shank reference frame for each dataset derived
    % from definitions in [Dumas2007], [Wu2002]. The x-axis is perpendicular
    % to the plane containing the knee joint and the MM and LM markers
    % pointing anterior. The y-axis is given by connecting the knee and
    % ankle joints pointing proximal. The z-axis is the common line
    % perpendicular to the x- and y-axis pointing right. Origin is the knee
    % joint.
    xAxis = cross(getAverageVector('KJ_L', 'estimatedJoint', 'MM_L', 'surface', initialStartFrame, initialEndFrame), getAverageVector('KJ_L', 'estimatedJoint', 'LM_L', 'surface', initialStartFrame, initialEndFrame));
    xAxis = xAxis / norm(xAxis);
    yAxis = getAverageVector('AJ_L', 'estimatedJoint', 'KJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    shankRelativePositionLM_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('KJ_L', 'estimatedJoint', 'LM_L', 'surface', initialStartFrame, initialEndFrame))';
    shankRelativePositionMM_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('KJ_L', 'estimatedJoint', 'MM_L', 'surface', initialStartFrame, initialEndFrame))';
    shankRelativePositionAJ_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('KJ_L', 'estimatedJoint', 'AJ_L', 'estimatedJoint', initialStartFrame, initialEndFrame))';
    xAxis = cross(getAverageVector('KJ_R', 'estimatedJoint', 'LM_R', 'surface', initialStartFrame, initialEndFrame), getAverageVector('KJ_R', 'estimatedJoint', 'MM_R', 'surface', initialStartFrame, initialEndFrame));
    xAxis = xAxis / norm(xAxis);
    yAxis = getAverageVector('AJ_R', 'estimatedJoint', 'KJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame);
    yAxis = yAxis / norm(yAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    shankRelativePositionLM_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('KJ_R', 'estimatedJoint', 'LM_R', 'surface', initialStartFrame, initialEndFrame))';
    shankRelativePositionMM_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('KJ_R', 'estimatedJoint', 'MM_R', 'surface', initialStartFrame, initialEndFrame))';
    shankRelativePositionAJ_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('KJ_R', 'estimatedJoint', 'AJ_R', 'estimatedJoint', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
    % Estimate average reference positions of adjacent markers and joints
    % in the left and right foot reference frame for each dataset based
    % on definitions in [Dumas2007]. The x-axis is given by connecting the
    % CAL marker with the estimated midpoint MT pointing anterior. The y-
    % axis is perpendicular to the plantar aspect of the foot approximated
    % by the plane containing the CAL, MT2 and MT5 markers pointing proximal.
    % The z-axis is the common line perpendicular to the x- and y-axis
    % pointing right. Origin is the ankle joint.
    xAxis = vectorMT_L - getAverageMarker('CAL_L', 'surface', initialStartFrame, initialEndFrame);
    xAxis = xAxis / norm(xAxis);
    yAxis = cross(getAverageVector('CAL_L', 'surface', 'MT2_L', 'surface', initialStartFrame, initialEndFrame), getAverageVector('CAL_L', 'surface', 'MT5_L', 'surface', initialStartFrame, initialEndFrame));
    yAxis = yAxis / norm(yAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    footRelativePositionCAL_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('AJ_L', 'estimatedJoint', 'CAL_L', 'surface', initialStartFrame, initialEndFrame))';
    footRelativePositionMT2_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('AJ_L', 'estimatedJoint', 'MT2_L', 'surface', initialStartFrame, initialEndFrame))';
    footRelativePositionMT5_L(datasetIndex, :) = (rotationMatrix \ getAverageVector('AJ_L', 'estimatedJoint', 'MT5_L', 'surface', initialStartFrame, initialEndFrame))';
    xAxis = vectorMT_R - getAverageMarker('CAL_R', 'surface', initialStartFrame, initialEndFrame);
    xAxis = xAxis / norm(xAxis);
    yAxis = cross(getAverageVector('CAL_R', 'surface', 'MT5_R', 'surface', initialStartFrame, initialEndFrame), getAverageVector('CAL_R', 'surface', 'MT2_R', 'surface', initialStartFrame, initialEndFrame));
    yAxis = yAxis / norm(yAxis);
    zAxis = cross(xAxis, yAxis);
    zAxis = zAxis / norm(zAxis);
    rotationMatrix = [xAxis, yAxis, zAxis];
    footRelativePositionCAL_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('AJ_R', 'estimatedJoint', 'CAL_R', 'surface', initialStartFrame, initialEndFrame))';
    footRelativePositionMT2_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('AJ_R', 'estimatedJoint', 'MT2_R', 'surface', initialStartFrame, initialEndFrame))';
    footRelativePositionMT5_R(datasetIndex, :) = (rotationMatrix \ getAverageVector('AJ_R', 'estimatedJoint', 'MT5_R', 'surface', initialStartFrame, initialEndFrame))';
    clear xAxis yAxis zAxis rotationMatrix;
    
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
foot_L.referenceLengthX = median(footReferenceLengthX_L);
foot_L.referenceLengthY = median(footReferenceLengthY_L);
foot_L.referenceLengthZ = median(footReferenceLengthZ_L);
foot_R.referenceLengthX = median(footReferenceLengthX_R);
foot_R.referenceLengthY = median(footReferenceLengthY_R);
foot_R.referenceLengthZ = median(footReferenceLengthZ_R);
foot_L.segmentLengthX = footLength_L;
foot_R.segmentLengthX = footLength_R;
foot_L.segmentLengthY = median(footSegmentLengthY_L);
foot_R.segmentLengthY = median(footSegmentLengthY_R);
clear headSegmentLengthY torsoSegmentLengthY upperArmSegmentLengthY_L upperArmSegmentLengthY_R ...
    pelvisSegmentLengthZ pelvisReferenceLength lowerArmSegmentLengthY_L lowerArmSegmentLengthY_R ...
    thighSegmentLengthY_L thighSegmentLengthY_R shankSegmentLengthY_L shankSegmentLengthY_R ...
    footReferenceLengthX_L footReferenceLengthY_L footReferenceLengthZ_L footReferenceLengthX_R ...
    footReferenceLengthY_R footReferenceLengthZ_R footSegmentLengthY_L footSegmentLengthY_R;

% Compute median of joint positions of all datasets
joints.absolutePosition.LNJ = median(absolutePositionLNJ)';
joints.absolutePosition.SJ_L = median(absolutePositionSJ_L)';
joints.absolutePosition.SJ_R = median(absolutePositionSJ_R)';
joints.absolutePosition.EJ_L = median(absolutePositionEJ_L)';
joints.absolutePosition.EJ_R = median(absolutePositionEJ_R)';
joints.absolutePosition.ULJ = median(absolutePositionULJ)';
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
footReferenceLength_L = sqrt(foot_L.referenceLengthX^2 + foot_L.referenceLengthY^2 + foot_L.referenceLengthZ^2);
footReferenceLength_R = sqrt(foot_R.referenceLengthX^2 + foot_R.referenceLengthY^2 + foot_R.referenceLengthZ^2);
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
    foot_L.mass = 2 * 0.01 * bodyMass * footReferenceLength_L / (footReferenceLength_L + footReferenceLength_R);
    foot_R.mass = 2 * 0.01 * bodyMass * footReferenceLength_R / (footReferenceLength_L + footReferenceLength_R);
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
    footReferenceLength_L = sqrt(foot_L.referenceLengthX^2 + foot_L.referenceLengthY^2 + foot_L.referenceLengthZ^2);
    footReferenceLength_R = sqrt(foot_R.referenceLengthX^2 + foot_R.referenceLengthY^2 + foot_R.referenceLengthZ^2);
    foot_L.mass = 2 * 0.012 * bodyMass * footReferenceLength_L / (footReferenceLength_L + footReferenceLength_R);
    foot_R.mass = 2 * 0.012 * bodyMass * footReferenceLength_R / (footReferenceLength_L + footReferenceLength_R);
end

% Estimate center of mass (COM) coordinates according to [Dumas2007]
if(strcmp(gender, 'female'))
    head.comX = -0.07 * head.segmentLengthY;
    head.comY = 0.597 * head.segmentLengthY;
    head.comZ = 0;
    upperArm_L.comX = -0.073 * upperArm_L.segmentLengthY;
    upperArm_L.comY = -0.454 * upperArm_L.segmentLengthY;
    upperArm_L.comZ = 0.028 * upperArm_L.segmentLengthY;
    upperArm_R.comX = -0.073 * upperArm_R.segmentLengthY;
    upperArm_R.comY = -0.454 * upperArm_R.segmentLengthY;
    upperArm_R.comZ = -0.028 * upperArm_R.segmentLengthY;
    lowerArm_L.comX = 0.021 * lowerArm_L.segmentLengthY;
    lowerArm_L.comY = -0.411 * lowerArm_L.segmentLengthY;
    lowerArm_L.comZ = -0.019 * lowerArm_L.segmentLengthY;
    lowerArm_R.comX = 0.021 * lowerArm_R.segmentLengthY;
    lowerArm_R.comY = -0.411 * lowerArm_R.segmentLengthY;
    lowerArm_R.comZ = 0.019 * lowerArm_R.segmentLengthY;
    hand_L.comX = 0.033 * hand_L.segmentLengthY;
    hand_L.comY = -0.327 * hand_L.segmentLengthY;
    hand_L.comZ = -0.021 * hand_L.segmentLengthY;
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
    thigh_L.comZ = -0.009 * thigh_L.segmentLengthY;
    thigh_R.comX = -0.077 * thigh_R.segmentLengthY;
    thigh_R.comY = -0.377 * thigh_R.segmentLengthY;
    thigh_R.comZ = 0.009 * thigh_R.segmentLengthY;
    shank_L.comX = -0.049 * shank_L.segmentLengthY;
    shank_L.comY = -0.404 * shank_L.segmentLengthY;
    shank_L.comZ = -0.031 * shank_L.segmentLengthY;
    shank_R.comX = -0.049 * shank_R.segmentLengthY;
    shank_R.comY = -0.404 * shank_R.segmentLengthY;
    shank_R.comZ = 0.031 * shank_R.segmentLengthY;
    foot_L.comX = 0.443 * footReferenceLength_L;
    foot_L.comY = 0.044 * footReferenceLength_L;
    foot_L.comZ = 0.025 * footReferenceLength_L;
    foot_R.comX = 0.443 * footReferenceLength_R;
    foot_R.comY = 0.044 * footReferenceLength_R;
    foot_R.comZ = -0.025 * footReferenceLength_R;
else
    head.comX = -0.062 * head.segmentLengthY;
    head.comY = 0.555 * head.segmentLengthY;
    head.comZ = 0.001 * head.segmentLengthY;
    upperArm_L.comX = 0.017 * upperArm_L.segmentLengthY;
    upperArm_L.comY = -0.452 * upperArm_L.segmentLengthY;
    upperArm_L.comZ = 0.026 * upperArm_L.segmentLengthY;
    upperArm_R.comX = 0.017 * upperArm_R.segmentLengthY;
    upperArm_R.comY = -0.452 * upperArm_R.segmentLengthY;
    upperArm_R.comZ = -0.026 * upperArm_R.segmentLengthY;
    lowerArm_L.comX = 0.01 * lowerArm_L.segmentLengthY;
    lowerArm_L.comY = -0.417 * lowerArm_L.segmentLengthY;
    lowerArm_L.comZ = -0.014 * lowerArm_L.segmentLengthY;
    lowerArm_R.comX = 0.01 * lowerArm_R.segmentLengthY;
    lowerArm_R.comY = -0.417 * lowerArm_R.segmentLengthY;
    lowerArm_R.comZ = 0.014 * lowerArm_R.segmentLengthY;
    hand_L.comX = 0.035 * hand_L.segmentLengthY;
    hand_L.comY = -0.357 * hand_L.segmentLengthY;
    hand_L.comZ = -0.032 * hand_L.segmentLengthY;
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
    thigh_L.comZ = -0.033 * thigh_L.segmentLengthY;
    thigh_R.comX = -0.041 * thigh_R.segmentLengthY;
    thigh_R.comY = -0.429 * thigh_R.segmentLengthY;
    thigh_R.comZ = 0.033 * thigh_R.segmentLengthY;
    shank_L.comX = -0.048 * shank_L.segmentLengthY;
    shank_L.comY = -0.41 * shank_L.segmentLengthY;
    shank_L.comZ = -0.007 * shank_L.segmentLengthY;
    shank_R.comX = -0.048 * shank_R.segmentLengthY;
    shank_R.comY = -0.41 * shank_R.segmentLengthY;
    shank_R.comZ = 0.007 * shank_R.segmentLengthY;
    foot_L.comX = 0.436 * footReferenceLength_L;
    foot_L.comY = -0.025 * footReferenceLength_L;
    foot_L.comZ = 0.007 * footReferenceLength_L;
    foot_R.comX = 0.436 * footReferenceLength_R;
    foot_R.comY = -0.025 * footReferenceLength_R;
    foot_R.comZ = -0.007 * footReferenceLength_R;
end

% Estimate moments and products of inertia with respect to the center of
% mass according to [Dumas2007]
if(strcmp(gender, 'female'))
    head.moiXX = (0.32 * head.segmentLengthY / 1000)^2 * head.mass;
    head.moiYY = (0.27 * head.segmentLengthY / 1000)^2 * head.mass;
    head.moiZZ = (0.34 * head.segmentLengthY / 1000)^2 * head.mass;
    head.poiXY = -(0.06 * head.segmentLengthY / 1000)^2 * head.mass;
    head.poiXZ = (0.01 * head.segmentLengthY / 1000)^2 * head.mass;
    head.poiYZ = -(0.01 * head.segmentLengthY / 1000)^2 * head.mass;
    upperArm_L.moiXX = (0.33 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.moiYY = (0.17 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.moiZZ = (0.33 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.poiXY = (0.03 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.poiXZ = (0.05 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.poiYZ = (0.14 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_R.moiXX = (0.33 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.moiYY = (0.17 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.moiZZ = (0.33 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.poiXY = (0.03 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.poiXZ = -(0.05 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.poiYZ = -(0.14 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    lowerArm_L.moiXX = (0.26 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.moiYY = (0.14 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.moiZZ = (0.25 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.poiXY = (0.1 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.poiXZ = -(0.04 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.poiYZ = (0.13 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_R.moiXX = (0.26 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.moiYY = (0.14 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.moiZZ = (0.25 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.poiXY = (0.1 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.poiXZ = (0.04 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.poiYZ = -(0.13 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    hand_L.moiXX = (0.27 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.moiYY = (0.18 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.moiZZ = (0.25 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.poiXY = (0.12 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.poiXZ = -(0.1 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.poiYZ = (0.12 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_R.moiXX = (0.27 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.moiYY = (0.18 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.moiZZ = (0.25 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.poiXY = (0.12 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.poiXZ = (0.1 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.poiYZ = -(0.12 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    torso.moiXX = (0.29 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.moiYY = (0.27 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.moiZZ = (0.29 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.poiXY = (0.22 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.poiXZ = (0.05 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.poiYZ = -(0.05 * torso.segmentLengthY / 1000)^2 * torso.mass;
    pelvis.moiXX = (0.91 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.moiYY = (pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.moiZZ = (0.79 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.poiXY = -(0.34 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.poiXZ = -(0.01 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.poiYZ = -(0.01 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    thigh_L.moiXX = (0.31 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.moiYY = (0.19 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.moiZZ = (0.32 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.poiXY = (0.07 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.poiXZ = (0.02 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.poiYZ = (0.07 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_R.moiXX = (0.31 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.moiYY = (0.19 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.moiZZ = (0.32 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.poiXY = (0.07 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.poiXZ = -(0.02 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.poiYZ = -(0.07 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    shank_L.moiXX = (0.28 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.moiYY = (0.1 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.moiZZ = (0.28 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.poiXY = (0.02 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.poiXZ = -(0.01 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.poiYZ = -(0.06 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_R.moiXX = (0.28 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.moiYY = (0.1 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.moiZZ = (0.28 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.poiXY = (0.02 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.poiXZ = (0.01 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.poiYZ = (0.06 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    foot_L.moiXX = (0.12 * footReferenceLength_L / 1000)^2 * foot_L.mass;
    foot_L.moiYY = (0.25 * footReferenceLength_L / 1000)^2 * foot_L.mass;
    foot_L.moiZZ = (0.25 * footReferenceLength_L / 1000)^2 * foot_L.mass;
    foot_L.poiXY = -(0.07 * footReferenceLength_L / 1000)^2 * foot_L.mass;
    foot_L.poiXZ = -(0.05 * footReferenceLength_L / 1000)^2 * foot_L.mass;
    foot_L.poiYZ = (0.03 * footReferenceLength_L / 1000)^2 * foot_L.mass;
    foot_R.moiXX = (0.12 * footReferenceLength_R / 1000)^2 * foot_R.mass;
    foot_R.moiYY = (0.25 * footReferenceLength_R / 1000)^2 * foot_R.mass;
    foot_R.moiZZ = (0.25 * footReferenceLength_R / 1000)^2 * foot_R.mass;
    foot_R.poiXY = -(0.07 * footReferenceLength_R / 1000)^2 * foot_R.mass;
    foot_R.poiXZ = (0.05 * footReferenceLength_R / 1000)^2 * foot_R.mass;
    foot_R.poiYZ = -(0.03 * footReferenceLength_R / 1000)^2 * foot_R.mass;
else
    head.moiXX = (0.31 * head.segmentLengthY / 1000)^2 * head.mass;
    head.moiYY = (0.25 * head.segmentLengthY / 1000)^2 * head.mass;
    head.moiZZ = (0.33 * head.segmentLengthY / 1000)^2 * head.mass;
    head.poiXY = -(0.09 * head.segmentLengthY / 1000)^2 * head.mass;
    head.poiXZ = -(0.02 * head.segmentLengthY / 1000)^2 * head.mass;
    head.poiYZ = (0.03 * head.segmentLengthY / 1000)^2 * head.mass;
    upperArm_L.moiXX = (0.31 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.moiYY = (0.14 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.moiZZ = (0.32 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.poiXY = (0.06 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.poiXZ = -(0.05 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_L.poiYZ = -(0.02 * upperArm_L.segmentLengthY / 1000)^2 * upperArm_L.mass;
    upperArm_R.moiXX = (0.31 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.moiYY = (0.14 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.moiZZ = (0.32 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.poiXY = (0.06 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.poiXZ = (0.05 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    upperArm_R.poiYZ = (0.02 * upperArm_R.segmentLengthY / 1000)^2 * upperArm_R.mass;
    lowerArm_L.moiXX = (0.28 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.moiYY = (0.11 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.moiZZ = (0.27 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.poiXY = (0.03 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.poiXZ = -(0.02 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_L.poiYZ = (0.08 * lowerArm_L.segmentLengthY / 1000)^2 * lowerArm_L.mass;
    lowerArm_R.moiXX = (0.28 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.moiYY = (0.11 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.moiZZ = (0.27 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.poiXY = (0.03 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.poiXZ = (0.02 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    lowerArm_R.poiYZ = -(0.08 * lowerArm_R.segmentLengthY / 1000)^2 * lowerArm_R.mass;
    hand_L.moiXX = (0.26 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.moiYY = (0.16 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.moiZZ = (0.24 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.poiXY = (0.09 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.poiXZ = -(0.07 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_L.poiYZ = (0.08 * hand_L.segmentLengthY / 1000)^2 * hand_L.mass;
    hand_R.moiXX = (0.26 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.moiYY = (0.16 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.moiZZ = (0.24 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.poiXY = (0.09 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.poiXZ = (0.07 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    hand_R.poiYZ = -(0.08 * hand_R.segmentLengthY / 1000)^2 * hand_R.mass;
    torso.moiXX = (0.27 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.moiYY = (0.25 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.moiZZ = (0.28 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.poiXY = (0.18 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.poiXZ = (0.02 * torso.segmentLengthY / 1000)^2 * torso.mass;
    torso.poiYZ = -(0.04 * torso.segmentLengthY / 1000)^2 * torso.mass;
    pelvis.moiXX = (1.01 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.moiYY = (1.06 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.moiZZ = (0.95 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.poiXY = -(0.25 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.poiXZ = -(0.12 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    pelvis.poiYZ = -(0.08 * pelvis.segmentLengthY / 1000)^2 * pelvis.mass;
    thigh_L.moiXX = (0.29 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.moiYY = (0.15 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.moiZZ = (0.3 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.poiXY = (0.07 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.poiXZ = (0.02 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_L.poiYZ = (0.07 * thigh_L.segmentLengthY / 1000)^2 * thigh_L.mass;
    thigh_R.moiXX = (0.29 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.moiYY = (0.15 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.moiZZ = (0.3 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.poiXY = (0.07 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.poiXZ = -(0.02 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    thigh_R.poiYZ = -(0.07 * thigh_R.segmentLengthY / 1000)^2 * thigh_R.mass;
    shank_L.moiXX = (0.28 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.moiYY = (0.1 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.moiZZ = (0.28 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.poiXY = -(0.04 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.poiXZ = (0.02 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_L.poiYZ = -(0.05 * shank_L.segmentLengthY / 1000)^2 * shank_L.mass;
    shank_R.moiXX = (0.28 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.moiYY = (0.1 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.moiZZ = (0.28 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.poiXY = -(0.04 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.poiXZ = -(0.02 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    shank_R.poiYZ = (0.05 * shank_R.segmentLengthY / 1000)^2 * shank_R.mass;
    foot_L.moiXX = (0.11 * footReferenceLength_L / 1000)^2 * foot_L.mass;
    foot_L.moiYY = (0.25 * footReferenceLength_L / 1000)^2 * foot_L.mass;
    foot_L.moiZZ = (0.25 * footReferenceLength_L / 1000)^2 * foot_L.mass;
    foot_L.poiXY = (0.09 * footReferenceLength_L / 1000)^2 * foot_L.mass;
    foot_L.poiXZ = (0.06 * footReferenceLength_L / 1000)^2 * foot_L.mass;
    foot_L.poiYZ = 0;
    foot_R.moiXX = (0.11 * footReferenceLength_R / 1000)^2 * foot_R.mass;
    foot_R.moiYY = (0.25 * footReferenceLength_R / 1000)^2 * foot_R.mass;
    foot_R.moiZZ = (0.25 * footReferenceLength_R / 1000)^2 * foot_R.mass;
    foot_R.poiXY = (0.09 * footReferenceLength_R / 1000)^2 * foot_R.mass;
    foot_R.poiXZ = -(0.06 * footReferenceLength_R / 1000)^2 * foot_R.mass;
    foot_R.poiYZ = 0;
end
clear footReferenceLength_L footReferenceLength_R

% Combine lower arm and hand segment by assuming the hand segment as a
% point mass at the wrist marker (WRI)
lowerArm_L.moiXX = lowerArm_L.moiXX + (lowerArm_L.segmentLengthY  / 1000)^2 * hand_L.mass;
lowerArm_L.moiZZ = lowerArm_L.moiZZ + (lowerArm_L.segmentLengthY / 1000)^2 * hand_L.mass;
lowerArm_R.moiXX = lowerArm_R.moiXX + (lowerArm_R.segmentLengthY / 1000)^2 * hand_R.mass;
lowerArm_R.moiZZ = lowerArm_R.moiZZ + (lowerArm_R.segmentLengthY / 1000)^2 * hand_R.mass;
lowerArm_L.comY = (lowerArm_L.comY * lowerArm_L.mass - lowerArm_L.segmentLengthY * hand_L.mass) / (lowerArm_L.mass + hand_L.mass);
lowerArm_R.comY = (lowerArm_R.comY * lowerArm_R.mass - lowerArm_R.segmentLengthY * hand_R.mass) / (lowerArm_R.mass + hand_R.mass);
lowerArm_L.mass = lowerArm_L.mass + hand_L.mass;
lowerArm_R.mass = lowerArm_R.mass + hand_R.mass;
clear hand_L hand_R;

% Save processed data
save(saveFile, 'subject', 'age', 'gender', 'origin', 'bodyHeight', 'bodyMass', 'equipmentMass', 'joints', 'head', 'torso', 'upperArm_L', 'upperArm_R', 'lowerArm_L', 'lowerArm_R', 'pelvis', 'thigh_L', 'thigh_R', 'shank_L', 'shank_R', 'foot_L', 'foot_R');
fprintf('STATUS: Antropometric parameters were saved.\n');
