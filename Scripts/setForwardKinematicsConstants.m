% ------------------------------------------------------
% This script sets the constants required for the forward kinematics
% simulation.
% ------------------------------------------------------
% Technische Universität Darmstadt
% Department of Computer Science
% Simulation, Systems Optimization and Robotics Group
% Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2016
% Licensed under BSD 3-Clause License
% ------------------------------------------------------

% Set constants
JOINT_pBJX = 1;
JOINT_pBJY = 2;
JOINT_pBJZ = 3;
JOINT_rBJX = 4;
JOINT_rBJY = 5;
JOINT_rBJZ = 6;
JOINT_rLNJX = 7;
JOINT_rLNJY = 8;
JOINT_rLNJZ = 9;
JOINT_rSJX_L = 10;
JOINT_rSJY_L = 11;
JOINT_rSJZ_L = 12;
JOINT_rSJX_R = 13;
JOINT_rSJY_R = 14;
JOINT_rSJZ_R = 15;
JOINT_rEJZ_L = 16;
JOINT_rEJZ_R = 17;
JOINT_rULJX = 18;
JOINT_rULJY = 19;
JOINT_rULJZ = 20;
JOINT_rLLJX = 21;
JOINT_rLLJZ = 22;
JOINT_rHJX_L = 23;
JOINT_rHJY_L = 24;
JOINT_rHJZ_L = 25;
JOINT_rHJX_R = 26;
JOINT_rHJY_R = 27;
JOINT_rHJZ_R = 28;
JOINT_rKJZ_L = 29;
JOINT_rKJZ_R = 30;
JOINT_rAJX_L = 31;
JOINT_rAJY_L = 32;
JOINT_rAJZ_L = 33;
JOINT_rAJX_R = 34;
JOINT_rAJY_R = 35;
JOINT_rAJZ_R = 36;
JOINT_Total = JOINT_rAJZ_R;
ELEMENT_TRA_L = 1;
ELEMENT_TRA_R = 2;
ELEMENT_GLA = 3;
ELEMENT_ACR_L = 4;
ELEMENT_ACR_R = 5;
ELEMENT_LHC_L = 6;
ELEMENT_LHC_R = 7;
ELEMENT_WRI_L = 8;
ELEMENT_WRI_R = 9;
ELEMENT_SUP = 10;
ELEMENT_C7 = 11;
ELEMENT_T8 = 12;
ELEMENT_T12 = 13;
ELEMENT_ASIS_L = 14;
ELEMENT_ASIS_R = 15;
ELEMENT_PSIS_L = 16;
ELEMENT_PSIS_R = 17;
ELEMENT_PS = 18;
ELEMENT_GTR_L = 19;
ELEMENT_GTR_R = 20;
ELEMENT_LFC_L = 21;
ELEMENT_LFC_R = 22;
ELEMENT_MFC_L = 23;
ELEMENT_MFC_R = 24;
ELEMENT_LM_L = 25;
ELEMENT_LM_R = 26;
ELEMENT_MM_L = 27;
ELEMENT_MM_R = 28;
ELEMENT_CAL_L = 29;
ELEMENT_CAL_R = 30;
ELEMENT_MT2_L = 31;
ELEMENT_MT2_R = 32;
ELEMENT_MT5_L = 33;
ELEMENT_MT5_R = 34;
ELEMENT_LNJ = 35;
ELEMENT_SJ_L = 36;
ELEMENT_SJ_R = 37;
ELEMENT_EJ_L = 38;
ELEMENT_EJ_R = 39;
ELEMENT_ULJ = 40;
ELEMENT_LLJ = 41;
ELEMENT_HJ_L = 42;
ELEMENT_HJ_R = 43;
ELEMENT_KJ_L = 44;
ELEMENT_KJ_R = 45;
ELEMENT_AJ_L = 46;
ELEMENT_AJ_R = 47;
ELEMENT_MarkerStart = ELEMENT_TRA_L;
ELEMENT_MarkerEnd = ELEMENT_MT5_R;
ELEMENT_JointStart = ELEMENT_LNJ;
ELEMENT_JointEnd = ELEMENT_AJ_R;
ELEMENT_Total = ELEMENT_AJ_R;