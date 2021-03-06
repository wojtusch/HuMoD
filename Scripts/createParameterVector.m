% ------------------------------------------------------
% This function creates a parameter matrix from given parameters, where the
% each column contains the parameters for one body segement in the given
% order: head, thorax, abdomen, pelvis, left upper arm, right upper arm,
% left lower arm, right lower arm, left thigh, right thigh, left shank,
% right shank, left foot, right foot.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2016
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

function parameterVector = createParameterVector(segment, parameters)

parameterVector = 0;

% Add parameters
switch(segment)
case 'head'
    parameterVector = [...
        0; ...
        parameters.head.segmentLengthY; ...
        0; ...
        parameters.head.mass; ...
        parameters.head.comX; ...
        parameters.head.comY; ...
        parameters.head.comZ; ...
        parameters.head.moiXX; ...
        parameters.head.moiYY; ...
        parameters.head.moiZZ; ...
        parameters.head.poiXY; ...
        parameters.head.poiXZ; ...
        parameters.head.poiYZ; ...
        parameters.head.relativePosition.GLA; ...
        parameters.head.relativePosition.TRA_L; ...
        parameters.head.relativePosition.TRA_R ...
    ];

case 'thorax'
    parameterVector = [...
        0; ...
        parameters.thorax.segmentLengthY; ...
        0; ...
        parameters.thorax.mass; ...
        parameters.thorax.comX; ...
        parameters.thorax.comY; ...
        parameters.thorax.comZ; ...
        parameters.thorax.moiXX; ...
        parameters.thorax.moiYY; ...
        parameters.thorax.moiZZ; ...
        parameters.thorax.poiXY; ...
        parameters.thorax.poiXZ; ...
        parameters.thorax.poiYZ; ...
        parameters.thorax.relativePosition.ACR_L; ...
        parameters.thorax.relativePosition.ACR_R; ...
        parameters.thorax.relativePosition.SUP; ...
        parameters.thorax.relativePosition.C7; ...
        parameters.thorax.relativePosition.T8; ...
        parameters.thorax.relativePosition.SJ_L; ...
        parameters.thorax.relativePosition.SJ_R; ...
    ];

case 'abdomen'
    parameterVector = [...
        0; ...
        parameters.abdomen.segmentLengthY; ...
        0; ...
        parameters.abdomen.mass; ...
        parameters.abdomen.comX; ...
        parameters.abdomen.comY; ...
        parameters.abdomen.comZ; ...
        parameters.abdomen.moiXX; ...
        parameters.abdomen.moiYY; ...
        parameters.abdomen.moiZZ; ...
        parameters.abdomen.poiXY; ...
        parameters.abdomen.poiXZ; ...
        parameters.abdomen.poiYZ; ...
        parameters.abdomen.relativePosition.T12; ...
        parameters.abdomen.relativePosition.LLJ ...
    ];

case 'pelvis'
    parameterVector = [...
        0; ...
        parameters.pelvis.segmentLengthY; ...
        parameters.pelvis.segmentLengthZ; ...
        parameters.pelvis.mass; ...
        parameters.pelvis.comX; ...
        parameters.pelvis.comY; ...
        parameters.pelvis.comZ; ...
        parameters.pelvis.moiXX; ...
        parameters.pelvis.moiYY; ...
        parameters.pelvis.moiZZ; ...
        parameters.pelvis.poiXY; ...
        parameters.pelvis.poiXZ; ...
        parameters.pelvis.poiYZ; ...
        parameters.pelvis.relativePosition.ASIS_L; ...
        parameters.pelvis.relativePosition.ASIS_R; ...
        parameters.pelvis.relativePosition.PSIS_L; ...
        parameters.pelvis.relativePosition.PSIS_R; ...
        parameters.pelvis.relativePosition.PS; ...
        parameters.pelvis.relativePosition.HJ_L; ...
        parameters.pelvis.relativePosition.HJ_R ...
    ];

case 'upperArm_L'
    parameterVector = [...
        0; ...
        parameters.upperArm_L.segmentLengthY; ...
        0; ...
        parameters.upperArm_L.mass; ...
        parameters.upperArm_L.comX; ...
        parameters.upperArm_L.comY; ...
        parameters.upperArm_L.comZ; ...
        parameters.upperArm_L.moiXX; ...
        parameters.upperArm_L.moiYY; ...
        parameters.upperArm_L.moiZZ; ...
        parameters.upperArm_L.poiXY; ...
        parameters.upperArm_L.poiXZ; ...
        parameters.upperArm_L.poiYZ; ...
        parameters.upperArm_L.relativePosition.LHC_L; ...
        parameters.upperArm_L.relativePosition.EJ_L ...
    ];

case 'upperArm_R'
    parameterVector = [...
        0; ...
        parameters.upperArm_R.segmentLengthY; ...
        0; ...
        parameters.upperArm_R.mass; ...
        parameters.upperArm_R.comX; ...
        parameters.upperArm_R.comY; ...
        parameters.upperArm_R.comZ; ...
        parameters.upperArm_R.moiXX; ...
        parameters.upperArm_R.moiYY; ...
        parameters.upperArm_R.moiZZ; ...
        parameters.upperArm_R.poiXY; ...
        parameters.upperArm_R.poiXZ; ...
        parameters.upperArm_R.poiYZ; ...
        parameters.upperArm_R.relativePosition.LHC_R; ...
        parameters.upperArm_R.relativePosition.EJ_R ...
    ];

case 'lowerArm_L'
    parameterVector = [...
        0; ...
        parameters.lowerArm_L.segmentLengthY; ...
        0; ...
        parameters.lowerArm_L.mass; ...
        parameters.lowerArm_L.comX; ...
        parameters.lowerArm_L.comY; ...
        parameters.lowerArm_L.comZ; ...
        parameters.lowerArm_L.moiXX; ...
        parameters.lowerArm_L.moiYY; ...
        parameters.lowerArm_L.moiZZ; ...
        parameters.lowerArm_L.poiXY; ...
        parameters.lowerArm_L.poiXZ; ...
        parameters.lowerArm_L.poiYZ; ...
        parameters.lowerArm_L.relativePosition.LHC_L; ...
        parameters.lowerArm_L.relativePosition.WRI_L ...
    ];

case 'lowerArm_R'
    parameterVector= [...
        0; ...
        parameters.upperArm_R.segmentLengthY; ...
        0; ...
        parameters.lowerArm_R.mass; ...
        parameters.lowerArm_R.comX; ...
        parameters.lowerArm_R.comY; ...
        parameters.lowerArm_R.comZ; ...
        parameters.lowerArm_R.moiXX; ...
        parameters.lowerArm_R.moiYY; ...
        parameters.lowerArm_R.moiZZ; ...
        parameters.lowerArm_R.poiXY; ...
        parameters.lowerArm_R.poiXZ; ...
        parameters.lowerArm_R.poiYZ; ...
        parameters.lowerArm_R.relativePosition.LHC_R; ...
        parameters.lowerArm_R.relativePosition.WRI_R ...
    ];

case 'thigh_L'
    parameterVector= [...
        0; ...
        parameters.thigh_L.segmentLengthY; ...
        0; ...
        parameters.thigh_L.mass; ...
        parameters.thigh_L.comX; ...
        parameters.thigh_L.comY; ...
        parameters.thigh_L.comZ; ...
        parameters.thigh_L.moiXX; ...
        parameters.thigh_L.moiYY; ...
        parameters.thigh_L.moiZZ; ...
        parameters.thigh_L.poiXY; ...
        parameters.thigh_L.poiXZ; ...
        parameters.thigh_L.poiYZ; ...
        parameters.thigh_L.relativePosition.GTR_L; ...
        parameters.thigh_L.relativePosition.LFC_L; ...
        parameters.thigh_L.relativePosition.MFC_L; ...
        parameters.thigh_L.relativePosition.KJ_L ...
    ];

case 'thigh_R'
    parameterVector= [...
        0; ...
        parameters.thigh_R.segmentLengthY; ...
        0; ...
        parameters.thigh_R.mass; ...
        parameters.thigh_R.comX; ...
        parameters.thigh_R.comY; ...
        parameters.thigh_R.comZ; ...
        parameters.thigh_R.moiXX; ...
        parameters.thigh_R.moiYY; ...
        parameters.thigh_R.moiZZ; ...
        parameters.thigh_R.poiXY; ...
        parameters.thigh_R.poiXZ; ...
        parameters.thigh_R.poiYZ; ...
        parameters.thigh_R.relativePosition.GTR_R; ...
        parameters.thigh_R.relativePosition.LFC_R; ...
        parameters.thigh_R.relativePosition.MFC_R; ...
        parameters.thigh_R.relativePosition.KJ_R ...
    ];

case 'shank_L'
    parameterVector= [...
        0; ...
        parameters.shank_L.segmentLengthY; ...
        0; ...
        parameters.shank_L.mass; ...
        parameters.shank_L.comX; ...
        parameters.shank_L.comY; ...
        parameters.shank_L.comZ; ...
        parameters.shank_L.moiXX; ...
        parameters.shank_L.moiYY; ...
        parameters.shank_L.moiZZ; ...
        parameters.shank_L.poiXY; ...
        parameters.shank_L.poiXZ; ...
        parameters.shank_L.poiYZ; ...
        parameters.shank_L.relativePosition.LM_L; ...
        parameters.shank_L.relativePosition.MM_L; ...
        parameters.shank_L.relativePosition.AJ_L ...
    ];

case 'shank_R'
    parameterVector= [...
        0; ...
        parameters.shank_R.segmentLengthY; ...
        0; ...
        parameters.shank_R.mass; ...
        parameters.shank_R.comX; ...
        parameters.shank_R.comY; ...
        parameters.shank_R.comZ; ...
        parameters.shank_R.moiXX; ...
        parameters.shank_R.moiYY; ...
        parameters.shank_R.moiZZ; ...
        parameters.shank_R.poiXY; ...
        parameters.shank_R.poiXZ; ...
        parameters.shank_R.poiYZ; ...
        parameters.shank_R.relativePosition.LM_R; ...
        parameters.shank_R.relativePosition.MM_R; ...
        parameters.shank_R.relativePosition.AJ_R ...
    ];

case 'foot_L'
    parameterVector= [...
        parameters.foot_L.referenceLengthX; ...
        parameters.foot_L.referenceLengthY; ...
        parameters.foot_L.referenceLengthZ; ...
        parameters.foot_L.mass; ...
        parameters.foot_L.comX; ...
        parameters.foot_L.comY; ...
        parameters.foot_L.comZ; ...
        parameters.foot_L.moiXX; ...
        parameters.foot_L.moiYY; ...
        parameters.foot_L.moiZZ; ...
        parameters.foot_L.poiXY; ...
        parameters.foot_L.poiXZ; ...
        parameters.foot_L.poiYZ; ...
        parameters.foot_L.relativePosition.CAL_L; ...
        parameters.foot_L.relativePosition.MT2_L; ...
        parameters.foot_L.relativePosition.MT5_L ...
    ];

case 'foot_R'
    parameterVector= [...
        parameters.foot_R.referenceLengthX; ...
        parameters.foot_R.referenceLengthY; ...
        parameters.foot_R.referenceLengthZ; ...
        parameters.foot_R.mass; ...
        parameters.foot_R.comX; ...
        parameters.foot_R.comY; ...
        parameters.foot_R.comZ; ...
        parameters.foot_R.moiXX; ...
        parameters.foot_R.moiYY; ...
        parameters.foot_L.moiZZ; ...
        parameters.foot_R.poiXY; ...
        parameters.foot_R.poiXZ; ...
        parameters.foot_R.poiYZ; ...
        parameters.foot_R.relativePosition.CAL_R; ...
        parameters.foot_R.relativePosition.MT2_R; ...
        parameters.foot_R.relativePosition.MT5_R ...
    ];

otherwise
    fprintf('ERROR: %s is not a body segment!\n', segment);

end

end