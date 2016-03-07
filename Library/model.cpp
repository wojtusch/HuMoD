// ------------------------------------------------------
// Technische Universität Darmstadt
// Department of Computer Science
// Simulation, Systems Optimization and Robotics Group
// Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
// Licensed under BSD 3-Clause License
// ------------------------------------------------------

// Defines
#define USE_ADOLC

// Includes
#include <iostream>
#include <string>
#include <mbslib/mbslib.h>
#include "segment.h"

extern "C" {

#include "model.h"

// Enumerates
enum joints {

    JOINT_pBJX = 0,
    JOINT_pBJY,
    JOINT_pBJZ,
    JOINT_rBJX,
    JOINT_rBJY,
    JOINT_rBJZ,
    JOINT_rLNJX,
    JOINT_rLNJY,
    JOINT_rLNJZ,
    JOINT_rSJX_L,
    JOINT_rSJY_L,
    JOINT_rSJZ_L,
    JOINT_rSJX_R,
    JOINT_rSJY_R,
    JOINT_rSJZ_R,
    JOINT_rEJZ_L,
    JOINT_rEJZ_R,
    JOINT_rULJX,
    JOINT_rULJY,
    JOINT_rULJZ,
    JOINT_rLLJX,
    JOINT_rLLJZ,
    JOINT_rHJX_L,
    JOINT_rHJY_L,
    JOINT_rHJZ_L,
    JOINT_rHJX_R,
    JOINT_rHJY_R,
    JOINT_rHJZ_R,
    JOINT_rKJZ_L,
    JOINT_rKJZ_R,
    JOINT_rAJX_L,
    JOINT_rAJY_L,
    JOINT_rAJZ_L,
    JOINT_rAJX_R,
    JOINT_rAJY_R,
    JOINT_rAJZ_R,
    JOINTS

};
enum elements {

    ELEMENT_TRA_L = 0,
    ELEMENT_TRA_R,
    ELEMENT_GLA,
    ELEMENT_ACR_L,
    ELEMENT_ACR_R,
    ELEMENT_LHC_L,
    ELEMENT_LHC_R,
    ELEMENT_WRI_L,
    ELEMENT_WRI_R,
    ELEMENT_SUP,
    ELEMENT_C7,
    ELEMENT_T8,
    ELEMENT_T12,
    ELEMENT_ASIS_L,
    ELEMENT_ASIS_R,
    ELEMENT_PSIS_L,
    ELEMENT_PSIS_R,
    ELEMENT_PS,
    ELEMENT_GTR_L,
    ELEMENT_GTR_R,
    ELEMENT_LFC_L,
    ELEMENT_LFC_R,
    ELEMENT_MFC_L,
    ELEMENT_MFC_R,
    ELEMENT_LM_L,
    ELEMENT_LM_R,
    ELEMENT_MM_L,
    ELEMENT_MM_R,
    ELEMENT_CAL_L,
    ELEMENT_CAL_R,
    ELEMENT_MT2_L,
    ELEMENT_MT2_R,
    ELEMENT_MT5_L,
    ELEMENT_MT5_R,
    ELEMENT_LNJ,
    ELEMENT_SJ_L,
    ELEMENT_SJ_R,
    ELEMENT_EJ_L,
    ELEMENT_EJ_R,
    ELEMENT_ULJ,
    ELEMENT_LLJ,
    ELEMENT_HJ_L,
    ELEMENT_HJ_R,
    ELEMENT_KJ_L,
    ELEMENT_KJ_R,
    ELEMENT_AJ_L,
    ELEMENT_AJ_R,
    ELEMENTS

};

// Private prototypes
void initializeVariables(void);
void generateModelStructure(void);

// Global variables
mbslib::MbsCompoundWithBuilder* mbs = nullptr;
mbslib::DeriveOMat* dom = nullptr;
char const* subject = "A";
mbslib::TScalar* gravity = nullptr;
Segment* headSegment = nullptr;
Segment* thoraxSegment = nullptr;
Segment* abdomenSegment = nullptr;
Segment* pelvisSegment = nullptr;
Segment* upperArmSegment_L = nullptr;
Segment* upperArmSegment_R = nullptr;
Segment* lowerArmSegment_L = nullptr;
Segment* lowerArmSegment_R = nullptr;
Segment* thighSegment_L = nullptr;
Segment* thighSegment_R = nullptr;
Segment* shankSegment_L = nullptr;
Segment* shankSegment_R = nullptr;
Segment* footSegment_L = nullptr;
Segment* footSegment_R = nullptr;

// Global functions
void createModel(char* subjectIdentifier, double* gravityValue, double* head, double* thorax, double* abdomen, double* pelvis, double* upperArm_L, double* upperArm_R, double* lowerArm_L, double* lowerArm_R, double* thigh_L, double* thigh_R, double* shank_L, double* shank_R, double* foot_L, double* foot_R) {

    // Initialize variables
    initializeVariables();

    // Set subject identifier
    subject = subjectIdentifier;

    // Set gravity
    *gravity = *gravityValue;

    // Set head parameters
    headSegment->lengthX = head[SEGMENT_LENGTHX];
    headSegment->lengthY = head[SEGMENT_LENGTHY];
    headSegment->lengthZ = head[SEGMENT_LENGTHZ];
    headSegment->mass = head[SEGMENT_MASS];
    headSegment->comX = head[SEGMENT_COMX];
    headSegment->comY = head[SEGMENT_COMY];
    headSegment->comZ = head[SEGMENT_COMZ];
    headSegment->moiXX = head[SEGMENT_MOIXX];
    headSegment->moiYY = head[SEGMENT_MOIYY];
    headSegment->moiZZ = head[SEGMENT_MOIZZ];
    headSegment->poiXY = head[SEGMENT_POIXY];
    headSegment->poiXZ = head[SEGMENT_POIXZ];
    headSegment->poiYZ = head[SEGMENT_POIYZ];
    headSegment->relativePositionArray[SEGMENT_HEAD_GLA][SEGMENT_POSITIONX] = head[(SEGMENT_HEAD_GLA * 3) + SEGMENT_OFFSET0];
    headSegment->relativePositionArray[SEGMENT_HEAD_GLA][SEGMENT_POSITIONY] = head[(SEGMENT_HEAD_GLA * 3) + SEGMENT_OFFSET1];
    headSegment->relativePositionArray[SEGMENT_HEAD_GLA][SEGMENT_POSITIONZ] = head[(SEGMENT_HEAD_GLA * 3) + SEGMENT_OFFSET2];
    headSegment->relativePositionArray[SEGMENT_HEAD_TRA_L][SEGMENT_POSITIONX] = head[(SEGMENT_HEAD_TRA_L * 3) + SEGMENT_OFFSET0];
    headSegment->relativePositionArray[SEGMENT_HEAD_TRA_L][SEGMENT_POSITIONY] = head[(SEGMENT_HEAD_TRA_L * 3) + SEGMENT_OFFSET1];
    headSegment->relativePositionArray[SEGMENT_HEAD_TRA_L][SEGMENT_POSITIONZ] = head[(SEGMENT_HEAD_TRA_L * 3) + SEGMENT_OFFSET2];
    headSegment->relativePositionArray[SEGMENT_HEAD_TRA_R][SEGMENT_POSITIONX] = head[(SEGMENT_HEAD_TRA_R * 3) + SEGMENT_OFFSET0];
    headSegment->relativePositionArray[SEGMENT_HEAD_TRA_R][SEGMENT_POSITIONY] = head[(SEGMENT_HEAD_TRA_R * 3) + SEGMENT_OFFSET1];
    headSegment->relativePositionArray[SEGMENT_HEAD_TRA_R][SEGMENT_POSITIONZ] = head[(SEGMENT_HEAD_TRA_R * 3) + SEGMENT_OFFSET2];

    // Set thorax parameters
    thoraxSegment->lengthX = thorax[SEGMENT_LENGTHX];
    thoraxSegment->lengthY = thorax[SEGMENT_LENGTHY];
    thoraxSegment->lengthZ = thorax[SEGMENT_LENGTHZ];
    thoraxSegment->mass = thorax[SEGMENT_MASS];
    thoraxSegment->comX = thorax[SEGMENT_COMX];
    thoraxSegment->comY = thorax[SEGMENT_COMY];
    thoraxSegment->comZ = thorax[SEGMENT_COMZ];
    thoraxSegment->moiXX = thorax[SEGMENT_MOIXX];
    thoraxSegment->moiYY = thorax[SEGMENT_MOIYY];
    thoraxSegment->moiZZ = thorax[SEGMENT_MOIZZ];
    thoraxSegment->poiXY = thorax[SEGMENT_POIXY];
    thoraxSegment->poiXZ = thorax[SEGMENT_POIXZ];
    thoraxSegment->poiYZ = thorax[SEGMENT_POIYZ];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_ACR_L][SEGMENT_POSITIONX] = thorax[(SEGMENT_THORAX_ACR_L * 3) + SEGMENT_OFFSET0];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_ACR_L][SEGMENT_POSITIONY] = thorax[(SEGMENT_THORAX_ACR_L * 3) + SEGMENT_OFFSET1];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_ACR_L][SEGMENT_POSITIONZ] = thorax[(SEGMENT_THORAX_ACR_L * 3) + SEGMENT_OFFSET2];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_ACR_R][SEGMENT_POSITIONX] = thorax[(SEGMENT_THORAX_ACR_R * 3) + SEGMENT_OFFSET0];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_ACR_R][SEGMENT_POSITIONY] = thorax[(SEGMENT_THORAX_ACR_R * 3) + SEGMENT_OFFSET1];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_ACR_R][SEGMENT_POSITIONZ] = thorax[(SEGMENT_THORAX_ACR_R * 3) + SEGMENT_OFFSET2];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_SUP][SEGMENT_POSITIONX] = thorax[(SEGMENT_THORAX_SUP * 3) + SEGMENT_OFFSET0];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_SUP][SEGMENT_POSITIONY] = thorax[(SEGMENT_THORAX_SUP * 3) + SEGMENT_OFFSET1];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_SUP][SEGMENT_POSITIONZ] = thorax[(SEGMENT_THORAX_SUP * 3) + SEGMENT_OFFSET2];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_C7][SEGMENT_POSITIONX] = thorax[(SEGMENT_THORAX_C7 * 3) + SEGMENT_OFFSET0];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_C7][SEGMENT_POSITIONY] = thorax[(SEGMENT_THORAX_C7 * 3) + SEGMENT_OFFSET1];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_C7][SEGMENT_POSITIONZ] = thorax[(SEGMENT_THORAX_C7 * 3) + SEGMENT_OFFSET2];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_T8][SEGMENT_POSITIONX] = thorax[(SEGMENT_THORAX_T8 * 3) + SEGMENT_OFFSET0];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_T8][SEGMENT_POSITIONY] = thorax[(SEGMENT_THORAX_T8 * 3) + SEGMENT_OFFSET1];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_T8][SEGMENT_POSITIONZ] = thorax[(SEGMENT_THORAX_T8 * 3) + SEGMENT_OFFSET2];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_SJ_L][SEGMENT_POSITIONX] = thorax[(SEGMENT_THORAX_SJ_L * 3) + SEGMENT_OFFSET0];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_SJ_L][SEGMENT_POSITIONY] = thorax[(SEGMENT_THORAX_SJ_L * 3) + SEGMENT_OFFSET1];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_SJ_L][SEGMENT_POSITIONZ] = thorax[(SEGMENT_THORAX_SJ_L * 3) + SEGMENT_OFFSET2];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_SJ_R][SEGMENT_POSITIONX] = thorax[(SEGMENT_THORAX_SJ_R * 3) + SEGMENT_OFFSET0];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_SJ_R][SEGMENT_POSITIONY] = thorax[(SEGMENT_THORAX_SJ_R * 3) + SEGMENT_OFFSET1];
    thoraxSegment->relativePositionArray[SEGMENT_THORAX_SJ_R][SEGMENT_POSITIONZ] = thorax[(SEGMENT_THORAX_SJ_R * 3) + SEGMENT_OFFSET2];

    // Set abdomen parameters
    abdomenSegment->lengthX = abdomen[SEGMENT_LENGTHX];
    abdomenSegment->lengthY = abdomen[SEGMENT_LENGTHY];
    abdomenSegment->lengthZ = abdomen[SEGMENT_LENGTHZ];
    abdomenSegment->mass = abdomen[SEGMENT_MASS];
    abdomenSegment->comX = abdomen[SEGMENT_COMX];
    abdomenSegment->comY = abdomen[SEGMENT_COMY];
    abdomenSegment->comZ = abdomen[SEGMENT_COMZ];
    abdomenSegment->moiXX = abdomen[SEGMENT_MOIXX];
    abdomenSegment->moiYY = abdomen[SEGMENT_MOIYY];
    abdomenSegment->moiZZ = abdomen[SEGMENT_MOIZZ];
    abdomenSegment->poiXY = abdomen[SEGMENT_POIXY];
    abdomenSegment->poiXZ = abdomen[SEGMENT_POIXZ];
    abdomenSegment->poiYZ = abdomen[SEGMENT_POIYZ];
    abdomenSegment->relativePositionArray[SEGMENT_ABDOMEN_T12][SEGMENT_POSITIONX] = abdomen[(SEGMENT_ABDOMEN_T12 * 3) + SEGMENT_OFFSET0];
    abdomenSegment->relativePositionArray[SEGMENT_ABDOMEN_T12][SEGMENT_POSITIONY] = abdomen[(SEGMENT_ABDOMEN_T12 * 3) + SEGMENT_OFFSET1];
    abdomenSegment->relativePositionArray[SEGMENT_ABDOMEN_T12][SEGMENT_POSITIONZ] = abdomen[(SEGMENT_ABDOMEN_T12 * 3) + SEGMENT_OFFSET2];
    abdomenSegment->relativePositionArray[SEGMENT_ABDOMEN_LLJ][SEGMENT_POSITIONX] = abdomen[(SEGMENT_ABDOMEN_LLJ * 3) + SEGMENT_OFFSET0];
    abdomenSegment->relativePositionArray[SEGMENT_ABDOMEN_LLJ][SEGMENT_POSITIONY] = abdomen[(SEGMENT_ABDOMEN_LLJ * 3) + SEGMENT_OFFSET1];
    abdomenSegment->relativePositionArray[SEGMENT_ABDOMEN_LLJ][SEGMENT_POSITIONZ] = abdomen[(SEGMENT_ABDOMEN_LLJ * 3) + SEGMENT_OFFSET2];

    // Set pelvis parameters
    pelvisSegment->lengthX = pelvis[SEGMENT_LENGTHX];
    pelvisSegment->lengthY = pelvis[SEGMENT_LENGTHY];
    pelvisSegment->lengthZ = pelvis[SEGMENT_LENGTHZ];
    pelvisSegment->mass = pelvis[SEGMENT_MASS];
    pelvisSegment->comX = pelvis[SEGMENT_COMX];
    pelvisSegment->comY = pelvis[SEGMENT_COMY];
    pelvisSegment->comZ = pelvis[SEGMENT_COMZ];
    pelvisSegment->moiXX = pelvis[SEGMENT_MOIXX];
    pelvisSegment->moiYY = pelvis[SEGMENT_MOIYY];
    pelvisSegment->moiZZ = pelvis[SEGMENT_MOIZZ];
    pelvisSegment->poiXY = pelvis[SEGMENT_POIXY];
    pelvisSegment->poiXZ = pelvis[SEGMENT_POIXZ];
    pelvisSegment->poiYZ = pelvis[SEGMENT_POIYZ];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_ASIS_L][SEGMENT_POSITIONX] = pelvis[(SEGMENT_PELVIS_ASIS_L * 3) + SEGMENT_OFFSET0];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_ASIS_L][SEGMENT_POSITIONY] = pelvis[(SEGMENT_PELVIS_ASIS_L * 3) + SEGMENT_OFFSET1];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_ASIS_L][SEGMENT_POSITIONZ] = pelvis[(SEGMENT_PELVIS_ASIS_L * 3) + SEGMENT_OFFSET2];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_ASIS_R][SEGMENT_POSITIONX] = pelvis[(SEGMENT_PELVIS_ASIS_R * 3) + SEGMENT_OFFSET0];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_ASIS_R][SEGMENT_POSITIONY] = pelvis[(SEGMENT_PELVIS_ASIS_R * 3) + SEGMENT_OFFSET1];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_ASIS_R][SEGMENT_POSITIONZ] = pelvis[(SEGMENT_PELVIS_ASIS_R * 3) + SEGMENT_OFFSET2];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PSIS_L][SEGMENT_POSITIONX] = pelvis[(SEGMENT_PELVIS_PSIS_L * 3) + SEGMENT_OFFSET0];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PSIS_L][SEGMENT_POSITIONY] = pelvis[(SEGMENT_PELVIS_PSIS_L * 3) + SEGMENT_OFFSET1];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PSIS_L][SEGMENT_POSITIONZ] = pelvis[(SEGMENT_PELVIS_PSIS_L * 3) + SEGMENT_OFFSET2];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PSIS_R][SEGMENT_POSITIONX] = pelvis[(SEGMENT_PELVIS_PSIS_R * 3) + SEGMENT_OFFSET0];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PSIS_R][SEGMENT_POSITIONY] = pelvis[(SEGMENT_PELVIS_PSIS_R * 3) + SEGMENT_OFFSET1];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PSIS_R][SEGMENT_POSITIONZ] = pelvis[(SEGMENT_PELVIS_PSIS_R * 3) + SEGMENT_OFFSET2];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PS][SEGMENT_POSITIONX] = pelvis[(SEGMENT_PELVIS_PS * 3) + SEGMENT_OFFSET0];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PS][SEGMENT_POSITIONY] = pelvis[(SEGMENT_PELVIS_PS * 3) + SEGMENT_OFFSET1];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PS][SEGMENT_POSITIONZ] = pelvis[(SEGMENT_PELVIS_PS * 3) + SEGMENT_OFFSET2];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_HJ_L][SEGMENT_POSITIONX] = pelvis[(SEGMENT_PELVIS_HJ_L * 3) + SEGMENT_OFFSET0];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_HJ_L][SEGMENT_POSITIONY] = pelvis[(SEGMENT_PELVIS_HJ_L * 3) + SEGMENT_OFFSET1];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_HJ_L][SEGMENT_POSITIONZ] = pelvis[(SEGMENT_PELVIS_HJ_L * 3) + SEGMENT_OFFSET2];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_HJ_R][SEGMENT_POSITIONX] = pelvis[(SEGMENT_PELVIS_HJ_R * 3) + SEGMENT_OFFSET0];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_HJ_R][SEGMENT_POSITIONY] = pelvis[(SEGMENT_PELVIS_HJ_R * 3) + SEGMENT_OFFSET1];
    pelvisSegment->relativePositionArray[SEGMENT_PELVIS_HJ_R][SEGMENT_POSITIONZ] = pelvis[(SEGMENT_PELVIS_HJ_R * 3) + SEGMENT_OFFSET2];

    // Set left upper arm parameters
    upperArmSegment_L->lengthX = upperArm_L[SEGMENT_LENGTHX];
    upperArmSegment_L->lengthY = upperArm_L[SEGMENT_LENGTHY];
    upperArmSegment_L->lengthZ = upperArm_L[SEGMENT_LENGTHZ];
    upperArmSegment_L->mass = upperArm_L[SEGMENT_MASS];
    upperArmSegment_L->comX = upperArm_L[SEGMENT_COMX];
    upperArmSegment_L->comY = upperArm_L[SEGMENT_COMY];
    upperArmSegment_L->comZ = upperArm_L[SEGMENT_COMZ];
    upperArmSegment_L->moiXX = upperArm_L[SEGMENT_MOIXX];
    upperArmSegment_L->moiYY = upperArm_L[SEGMENT_MOIYY];
    upperArmSegment_L->moiZZ = upperArm_L[SEGMENT_MOIZZ];
    upperArmSegment_L->poiXY = upperArm_L[SEGMENT_POIXY];
    upperArmSegment_L->poiXZ = upperArm_L[SEGMENT_POIXZ];
    upperArmSegment_L->poiYZ = upperArm_L[SEGMENT_POIYZ];
    upperArmSegment_L->relativePositionArray[SEGMENT_UPPER_ARM_LHC][SEGMENT_POSITIONX] = upperArm_L[(SEGMENT_UPPER_ARM_LHC * 3) + SEGMENT_OFFSET0];
    upperArmSegment_L->relativePositionArray[SEGMENT_UPPER_ARM_LHC][SEGMENT_POSITIONY] = upperArm_L[(SEGMENT_UPPER_ARM_LHC * 3) + SEGMENT_OFFSET1];
    upperArmSegment_L->relativePositionArray[SEGMENT_UPPER_ARM_LHC][SEGMENT_POSITIONZ] = upperArm_L[(SEGMENT_UPPER_ARM_LHC * 3) + SEGMENT_OFFSET2];
    upperArmSegment_L->relativePositionArray[SEGMENT_UPPER_ARM_EJ][SEGMENT_POSITIONX] = upperArm_L[(SEGMENT_UPPER_ARM_EJ * 3) + SEGMENT_OFFSET0];
    upperArmSegment_L->relativePositionArray[SEGMENT_UPPER_ARM_EJ][SEGMENT_POSITIONY] = upperArm_L[(SEGMENT_UPPER_ARM_EJ * 3) + SEGMENT_OFFSET1];
    upperArmSegment_L->relativePositionArray[SEGMENT_UPPER_ARM_EJ][SEGMENT_POSITIONZ] = upperArm_L[(SEGMENT_UPPER_ARM_EJ * 3) + SEGMENT_OFFSET2];

    // Set right upper arm parameters
    upperArmSegment_R->lengthX = upperArm_R[SEGMENT_LENGTHX];
    upperArmSegment_R->lengthY = upperArm_R[SEGMENT_LENGTHY];
    upperArmSegment_R->lengthZ = upperArm_R[SEGMENT_LENGTHZ];
    upperArmSegment_R->mass = upperArm_R[SEGMENT_MASS];
    upperArmSegment_R->comX = upperArm_R[SEGMENT_COMX];
    upperArmSegment_R->comY = upperArm_R[SEGMENT_COMY];
    upperArmSegment_R->comZ = upperArm_R[SEGMENT_COMZ];
    upperArmSegment_R->moiXX = upperArm_R[SEGMENT_MOIXX];
    upperArmSegment_R->moiYY = upperArm_R[SEGMENT_MOIYY];
    upperArmSegment_R->moiZZ = upperArm_R[SEGMENT_MOIZZ];
    upperArmSegment_R->poiXY = upperArm_R[SEGMENT_POIXY];
    upperArmSegment_R->poiXZ = upperArm_R[SEGMENT_POIXZ];
    upperArmSegment_R->poiYZ = upperArm_R[SEGMENT_POIYZ];
    upperArmSegment_R->relativePositionArray[SEGMENT_UPPER_ARM_LHC][SEGMENT_POSITIONX] = upperArm_R[(SEGMENT_UPPER_ARM_LHC * 3) + SEGMENT_OFFSET0];
    upperArmSegment_R->relativePositionArray[SEGMENT_UPPER_ARM_LHC][SEGMENT_POSITIONY] = upperArm_R[(SEGMENT_UPPER_ARM_LHC * 3) + SEGMENT_OFFSET1];
    upperArmSegment_R->relativePositionArray[SEGMENT_UPPER_ARM_LHC][SEGMENT_POSITIONZ] = upperArm_R[(SEGMENT_UPPER_ARM_LHC * 3) + SEGMENT_OFFSET2];
    upperArmSegment_R->relativePositionArray[SEGMENT_UPPER_ARM_EJ][SEGMENT_POSITIONX] = upperArm_R[(SEGMENT_UPPER_ARM_EJ * 3) + SEGMENT_OFFSET0];
    upperArmSegment_R->relativePositionArray[SEGMENT_UPPER_ARM_EJ][SEGMENT_POSITIONY] = upperArm_R[(SEGMENT_UPPER_ARM_EJ * 3) + SEGMENT_OFFSET1];
    upperArmSegment_R->relativePositionArray[SEGMENT_UPPER_ARM_EJ][SEGMENT_POSITIONZ] = upperArm_R[(SEGMENT_UPPER_ARM_EJ * 3) + SEGMENT_OFFSET2];

    // Set left lower arm parameters
    lowerArmSegment_L->lengthX = lowerArm_L[SEGMENT_LENGTHX];
    lowerArmSegment_L->lengthY = lowerArm_L[SEGMENT_LENGTHY];
    lowerArmSegment_L->lengthZ = lowerArm_L[SEGMENT_LENGTHZ];
    lowerArmSegment_L->mass = lowerArm_L[SEGMENT_MASS];
    lowerArmSegment_L->comX = lowerArm_L[SEGMENT_COMX];
    lowerArmSegment_L->comY = lowerArm_L[SEGMENT_COMY];
    lowerArmSegment_L->comZ = lowerArm_L[SEGMENT_COMZ];
    lowerArmSegment_L->moiXX = lowerArm_L[SEGMENT_MOIXX];
    lowerArmSegment_L->moiYY = lowerArm_L[SEGMENT_MOIYY];
    lowerArmSegment_L->moiZZ = lowerArm_L[SEGMENT_MOIZZ];
    lowerArmSegment_L->poiXY = lowerArm_L[SEGMENT_POIXY];
    lowerArmSegment_L->poiXZ = lowerArm_L[SEGMENT_POIXZ];
    lowerArmSegment_L->poiYZ = lowerArm_L[SEGMENT_POIYZ];
    lowerArmSegment_L->relativePositionArray[SEGMENT_LOWER_ARM_LHC][SEGMENT_POSITIONX] = lowerArm_L[(SEGMENT_LOWER_ARM_LHC * 3) + SEGMENT_OFFSET0];
    lowerArmSegment_L->relativePositionArray[SEGMENT_LOWER_ARM_LHC][SEGMENT_POSITIONY] = lowerArm_L[(SEGMENT_LOWER_ARM_LHC * 3) + SEGMENT_OFFSET1];
    lowerArmSegment_L->relativePositionArray[SEGMENT_LOWER_ARM_LHC][SEGMENT_POSITIONZ] = lowerArm_L[(SEGMENT_LOWER_ARM_LHC * 3) + SEGMENT_OFFSET2];
    lowerArmSegment_L->relativePositionArray[SEGMENT_LOWER_ARM_WRI][SEGMENT_POSITIONX] = lowerArm_L[(SEGMENT_LOWER_ARM_WRI * 3) + SEGMENT_OFFSET0];
    lowerArmSegment_L->relativePositionArray[SEGMENT_LOWER_ARM_WRI][SEGMENT_POSITIONY] = lowerArm_L[(SEGMENT_LOWER_ARM_WRI * 3) + SEGMENT_OFFSET1];
    lowerArmSegment_L->relativePositionArray[SEGMENT_LOWER_ARM_WRI][SEGMENT_POSITIONZ] = lowerArm_L[(SEGMENT_LOWER_ARM_WRI * 3) + SEGMENT_OFFSET2];

    // Set right lower arm parameters
    lowerArmSegment_R->lengthX = lowerArm_R[SEGMENT_LENGTHX];
    lowerArmSegment_R->lengthY = lowerArm_R[SEGMENT_LENGTHY];
    lowerArmSegment_R->lengthZ = lowerArm_R[SEGMENT_LENGTHZ];
    lowerArmSegment_R->mass = lowerArm_R[SEGMENT_MASS];
    lowerArmSegment_R->comX = lowerArm_R[SEGMENT_COMX];
    lowerArmSegment_R->comY = lowerArm_R[SEGMENT_COMY];
    lowerArmSegment_R->comZ = lowerArm_R[SEGMENT_COMZ];
    lowerArmSegment_R->moiXX = lowerArm_R[SEGMENT_MOIXX];
    lowerArmSegment_R->moiYY = lowerArm_R[SEGMENT_MOIYY];
    lowerArmSegment_R->moiZZ = lowerArm_R[SEGMENT_MOIZZ];
    lowerArmSegment_R->poiXY = lowerArm_R[SEGMENT_POIXY];
    lowerArmSegment_R->poiXZ = lowerArm_R[SEGMENT_POIXZ];
    lowerArmSegment_R->poiYZ = lowerArm_R[SEGMENT_POIYZ];
    lowerArmSegment_R->relativePositionArray[SEGMENT_LOWER_ARM_LHC][SEGMENT_POSITIONX] = lowerArm_R[(SEGMENT_LOWER_ARM_LHC * 3) + SEGMENT_OFFSET0];
    lowerArmSegment_R->relativePositionArray[SEGMENT_LOWER_ARM_LHC][SEGMENT_POSITIONY] = lowerArm_R[(SEGMENT_LOWER_ARM_LHC * 3) + SEGMENT_OFFSET1];
    lowerArmSegment_R->relativePositionArray[SEGMENT_LOWER_ARM_LHC][SEGMENT_POSITIONZ] = lowerArm_R[(SEGMENT_LOWER_ARM_LHC * 3) + SEGMENT_OFFSET2];
    lowerArmSegment_R->relativePositionArray[SEGMENT_LOWER_ARM_WRI][SEGMENT_POSITIONX] = lowerArm_R[(SEGMENT_LOWER_ARM_WRI * 3) + SEGMENT_OFFSET0];
    lowerArmSegment_R->relativePositionArray[SEGMENT_LOWER_ARM_WRI][SEGMENT_POSITIONY] = lowerArm_R[(SEGMENT_LOWER_ARM_WRI * 3) + SEGMENT_OFFSET1];
    lowerArmSegment_R->relativePositionArray[SEGMENT_LOWER_ARM_WRI][SEGMENT_POSITIONZ] = lowerArm_R[(SEGMENT_LOWER_ARM_WRI * 3) + SEGMENT_OFFSET2];

    // Set left thigh parameters
    thighSegment_L->lengthX = thigh_L[SEGMENT_LENGTHX];
    thighSegment_L->lengthY = thigh_L[SEGMENT_LENGTHY];
    thighSegment_L->lengthZ = thigh_L[SEGMENT_LENGTHZ];
    thighSegment_L->mass = thigh_L[SEGMENT_MASS];
    thighSegment_L->comX = thigh_L[SEGMENT_COMX];
    thighSegment_L->comY = thigh_L[SEGMENT_COMY];
    thighSegment_L->comZ = thigh_L[SEGMENT_COMZ];
    thighSegment_L->moiXX = thigh_L[SEGMENT_MOIXX];
    thighSegment_L->moiYY = thigh_L[SEGMENT_MOIYY];
    thighSegment_L->moiZZ = thigh_L[SEGMENT_MOIZZ];
    thighSegment_L->poiXY = thigh_L[SEGMENT_POIXY];
    thighSegment_L->poiXZ = thigh_L[SEGMENT_POIXZ];
    thighSegment_L->poiYZ = thigh_L[SEGMENT_POIYZ];
    thighSegment_L->relativePositionArray[SEGMENT_THIGH_GTR][SEGMENT_POSITIONX] = thigh_L[(SEGMENT_THIGH_GTR * 3) + SEGMENT_OFFSET0];
    thighSegment_L->relativePositionArray[SEGMENT_THIGH_GTR][SEGMENT_POSITIONY] = thigh_L[(SEGMENT_THIGH_GTR * 3) + SEGMENT_OFFSET1];
    thighSegment_L->relativePositionArray[SEGMENT_THIGH_GTR][SEGMENT_POSITIONZ] = thigh_L[(SEGMENT_THIGH_GTR * 3) + SEGMENT_OFFSET2];
    thighSegment_L->relativePositionArray[SEGMENT_THIGH_LFC][SEGMENT_POSITIONX] = thigh_L[(SEGMENT_THIGH_LFC * 3) + SEGMENT_OFFSET0];
    thighSegment_L->relativePositionArray[SEGMENT_THIGH_LFC][SEGMENT_POSITIONY] = thigh_L[(SEGMENT_THIGH_LFC * 3) + SEGMENT_OFFSET1];
    thighSegment_L->relativePositionArray[SEGMENT_THIGH_LFC][SEGMENT_POSITIONZ] = thigh_L[(SEGMENT_THIGH_LFC * 3) + SEGMENT_OFFSET2];
    thighSegment_L->relativePositionArray[SEGMENT_THIGH_MFC][SEGMENT_POSITIONX] = thigh_L[(SEGMENT_THIGH_MFC * 3) + SEGMENT_OFFSET0];
    thighSegment_L->relativePositionArray[SEGMENT_THIGH_MFC][SEGMENT_POSITIONY] = thigh_L[(SEGMENT_THIGH_MFC * 3) + SEGMENT_OFFSET1];
    thighSegment_L->relativePositionArray[SEGMENT_THIGH_MFC][SEGMENT_POSITIONZ] = thigh_L[(SEGMENT_THIGH_MFC * 3) + SEGMENT_OFFSET2];
    thighSegment_L->relativePositionArray[SEGMENT_THIGH_KJ][SEGMENT_POSITIONX] = thigh_L[(SEGMENT_THIGH_KJ * 3) + SEGMENT_OFFSET0];
    thighSegment_L->relativePositionArray[SEGMENT_THIGH_KJ][SEGMENT_POSITIONY] = thigh_L[(SEGMENT_THIGH_KJ * 3) + SEGMENT_OFFSET1];
    thighSegment_L->relativePositionArray[SEGMENT_THIGH_KJ][SEGMENT_POSITIONZ] = thigh_L[(SEGMENT_THIGH_KJ * 3) + SEGMENT_OFFSET2];

    // Set right thigh parameters
    thighSegment_R->lengthX = thigh_R[SEGMENT_LENGTHX];
    thighSegment_R->lengthY = thigh_R[SEGMENT_LENGTHY];
    thighSegment_R->lengthZ = thigh_R[SEGMENT_LENGTHZ];
    thighSegment_R->mass = thigh_R[SEGMENT_MASS];
    thighSegment_R->comX = thigh_R[SEGMENT_COMX];
    thighSegment_R->comY = thigh_R[SEGMENT_COMY];
    thighSegment_R->comZ = thigh_R[SEGMENT_COMZ];
    thighSegment_R->moiXX = thigh_R[SEGMENT_MOIXX];
    thighSegment_R->moiYY = thigh_R[SEGMENT_MOIYY];
    thighSegment_R->moiZZ = thigh_R[SEGMENT_MOIZZ];
    thighSegment_R->poiXY = thigh_R[SEGMENT_POIXY];
    thighSegment_R->poiXZ = thigh_R[SEGMENT_POIXZ];
    thighSegment_R->poiYZ = thigh_R[SEGMENT_POIYZ];
    thighSegment_R->relativePositionArray[SEGMENT_THIGH_GTR][SEGMENT_POSITIONX] = thigh_R[(SEGMENT_THIGH_GTR * 3) + SEGMENT_OFFSET0];
    thighSegment_R->relativePositionArray[SEGMENT_THIGH_GTR][SEGMENT_POSITIONY] = thigh_R[(SEGMENT_THIGH_GTR * 3) + SEGMENT_OFFSET1];
    thighSegment_R->relativePositionArray[SEGMENT_THIGH_GTR][SEGMENT_POSITIONZ] = thigh_R[(SEGMENT_THIGH_GTR * 3) + SEGMENT_OFFSET2];
    thighSegment_R->relativePositionArray[SEGMENT_THIGH_LFC][SEGMENT_POSITIONX] = thigh_R[(SEGMENT_THIGH_LFC * 3) + SEGMENT_OFFSET0];
    thighSegment_R->relativePositionArray[SEGMENT_THIGH_LFC][SEGMENT_POSITIONY] = thigh_R[(SEGMENT_THIGH_LFC * 3) + SEGMENT_OFFSET1];
    thighSegment_R->relativePositionArray[SEGMENT_THIGH_LFC][SEGMENT_POSITIONZ] = thigh_R[(SEGMENT_THIGH_LFC * 3) + SEGMENT_OFFSET2];
    thighSegment_R->relativePositionArray[SEGMENT_THIGH_MFC][SEGMENT_POSITIONX] = thigh_R[(SEGMENT_THIGH_MFC * 3) + SEGMENT_OFFSET0];
    thighSegment_R->relativePositionArray[SEGMENT_THIGH_MFC][SEGMENT_POSITIONY] = thigh_R[(SEGMENT_THIGH_MFC * 3) + SEGMENT_OFFSET1];
    thighSegment_R->relativePositionArray[SEGMENT_THIGH_MFC][SEGMENT_POSITIONZ] = thigh_R[(SEGMENT_THIGH_MFC * 3) + SEGMENT_OFFSET2];
    thighSegment_R->relativePositionArray[SEGMENT_THIGH_KJ][SEGMENT_POSITIONX] = thigh_R[(SEGMENT_THIGH_KJ * 3) + SEGMENT_OFFSET0];
    thighSegment_R->relativePositionArray[SEGMENT_THIGH_KJ][SEGMENT_POSITIONY] = thigh_R[(SEGMENT_THIGH_KJ * 3) + SEGMENT_OFFSET1];
    thighSegment_R->relativePositionArray[SEGMENT_THIGH_KJ][SEGMENT_POSITIONZ] = thigh_R[(SEGMENT_THIGH_KJ * 3) + SEGMENT_OFFSET2];

    // Set left shank parameters
    shankSegment_L->lengthX = shank_L[SEGMENT_LENGTHX];
    shankSegment_L->lengthY = shank_L[SEGMENT_LENGTHY];
    shankSegment_L->lengthZ = shank_L[SEGMENT_LENGTHZ];
    shankSegment_L->mass = shank_L[SEGMENT_MASS];
    shankSegment_L->comX = shank_L[SEGMENT_COMX];
    shankSegment_L->comY = shank_L[SEGMENT_COMY];
    shankSegment_L->comZ = shank_L[SEGMENT_COMZ];
    shankSegment_L->moiXX = shank_L[SEGMENT_MOIXX];
    shankSegment_L->moiYY = shank_L[SEGMENT_MOIYY];
    shankSegment_L->moiZZ = shank_L[SEGMENT_MOIZZ];
    shankSegment_L->poiXY = shank_L[SEGMENT_POIXY];
    shankSegment_L->poiXZ = shank_L[SEGMENT_POIXZ];
    shankSegment_L->poiYZ = shank_L[SEGMENT_POIYZ];
    shankSegment_L->relativePositionArray[SEGMENT_SHANK_LM][SEGMENT_POSITIONX] = shank_L[(SEGMENT_SHANK_LM * 3) + SEGMENT_OFFSET0];
    shankSegment_L->relativePositionArray[SEGMENT_SHANK_LM][SEGMENT_POSITIONY] = shank_L[(SEGMENT_SHANK_LM * 3) + SEGMENT_OFFSET1];
    shankSegment_L->relativePositionArray[SEGMENT_SHANK_LM][SEGMENT_POSITIONZ] = shank_L[(SEGMENT_SHANK_LM * 3) + SEGMENT_OFFSET2];
    shankSegment_L->relativePositionArray[SEGMENT_SHANK_MM][SEGMENT_POSITIONX] = shank_L[(SEGMENT_SHANK_MM * 3) + SEGMENT_OFFSET0];
    shankSegment_L->relativePositionArray[SEGMENT_SHANK_MM][SEGMENT_POSITIONY] = shank_L[(SEGMENT_SHANK_MM * 3) + SEGMENT_OFFSET1];
    shankSegment_L->relativePositionArray[SEGMENT_SHANK_MM][SEGMENT_POSITIONZ] = shank_L[(SEGMENT_SHANK_MM * 3) + SEGMENT_OFFSET2];
    shankSegment_L->relativePositionArray[SEGMENT_SHANK_AJ][SEGMENT_POSITIONX] = shank_L[(SEGMENT_SHANK_AJ * 3) + SEGMENT_OFFSET0];
    shankSegment_L->relativePositionArray[SEGMENT_SHANK_AJ][SEGMENT_POSITIONY] = shank_L[(SEGMENT_SHANK_AJ * 3) + SEGMENT_OFFSET1];
    shankSegment_L->relativePositionArray[SEGMENT_SHANK_AJ][SEGMENT_POSITIONZ] = shank_L[(SEGMENT_SHANK_AJ * 3) + SEGMENT_OFFSET2];

    // Set right shank parameters
    shankSegment_R->lengthX = shank_R[SEGMENT_LENGTHX];
    shankSegment_R->lengthY = shank_R[SEGMENT_LENGTHY];
    shankSegment_R->lengthZ = shank_R[SEGMENT_LENGTHZ];
    shankSegment_R->mass = shank_R[SEGMENT_MASS];
    shankSegment_R->comX = shank_R[SEGMENT_COMX];
    shankSegment_R->comY = shank_R[SEGMENT_COMY];
    shankSegment_R->comZ = shank_R[SEGMENT_COMZ];
    shankSegment_R->moiXX = shank_R[SEGMENT_MOIXX];
    shankSegment_R->moiYY = shank_R[SEGMENT_MOIYY];
    shankSegment_R->moiZZ = shank_R[SEGMENT_MOIZZ];
    shankSegment_R->poiXY = shank_R[SEGMENT_POIXY];
    shankSegment_R->poiXZ = shank_R[SEGMENT_POIXZ];
    shankSegment_R->poiYZ = shank_R[SEGMENT_POIYZ];
    shankSegment_R->relativePositionArray[SEGMENT_SHANK_LM][SEGMENT_POSITIONX] = shank_R[(SEGMENT_SHANK_LM * 3) + SEGMENT_OFFSET0];
    shankSegment_R->relativePositionArray[SEGMENT_SHANK_LM][SEGMENT_POSITIONY] = shank_R[(SEGMENT_SHANK_LM * 3) + SEGMENT_OFFSET1];
    shankSegment_R->relativePositionArray[SEGMENT_SHANK_LM][SEGMENT_POSITIONZ] = shank_R[(SEGMENT_SHANK_LM * 3) + SEGMENT_OFFSET2];
    shankSegment_R->relativePositionArray[SEGMENT_SHANK_MM][SEGMENT_POSITIONX] = shank_R[(SEGMENT_SHANK_MM * 3) + SEGMENT_OFFSET0];
    shankSegment_R->relativePositionArray[SEGMENT_SHANK_MM][SEGMENT_POSITIONY] = shank_R[(SEGMENT_SHANK_MM * 3) + SEGMENT_OFFSET1];
    shankSegment_R->relativePositionArray[SEGMENT_SHANK_MM][SEGMENT_POSITIONZ] = shank_R[(SEGMENT_SHANK_MM * 3) + SEGMENT_OFFSET2];
    shankSegment_R->relativePositionArray[SEGMENT_SHANK_AJ][SEGMENT_POSITIONX] = shank_R[(SEGMENT_SHANK_AJ * 3) + SEGMENT_OFFSET0];
    shankSegment_R->relativePositionArray[SEGMENT_SHANK_AJ][SEGMENT_POSITIONY] = shank_R[(SEGMENT_SHANK_AJ * 3) + SEGMENT_OFFSET1];
    shankSegment_R->relativePositionArray[SEGMENT_SHANK_AJ][SEGMENT_POSITIONZ] = shank_R[(SEGMENT_SHANK_AJ * 3) + SEGMENT_OFFSET2];

    // Set left foot parameters
    footSegment_L->lengthX = foot_L[REFERNCE_LENGTHX];
    footSegment_L->lengthY = foot_L[REFERNCE_LENGTHY];
    footSegment_L->lengthZ = foot_L[REFERNCE_LENGTHZ];
    footSegment_L->mass = foot_L[SEGMENT_MASS];
    footSegment_L->comX = foot_L[SEGMENT_COMX];
    footSegment_L->comY = foot_L[SEGMENT_COMY];
    footSegment_L->comZ = foot_L[SEGMENT_COMZ];
    footSegment_L->moiXX = foot_L[SEGMENT_MOIXX];
    footSegment_L->moiYY = foot_L[SEGMENT_MOIYY];
    footSegment_L->moiZZ = foot_L[SEGMENT_MOIZZ];
    footSegment_L->poiXY = foot_L[SEGMENT_POIXY];
    footSegment_L->poiXZ = foot_L[SEGMENT_POIXZ];
    footSegment_L->poiYZ = foot_L[SEGMENT_POIYZ];
    footSegment_L->relativePositionArray[SEGMENT_FOOT_CAL][SEGMENT_POSITIONX] = foot_L[(SEGMENT_FOOT_CAL * 3) + SEGMENT_OFFSET0];
    footSegment_L->relativePositionArray[SEGMENT_FOOT_CAL][SEGMENT_POSITIONY] = foot_L[(SEGMENT_FOOT_CAL * 3) + SEGMENT_OFFSET1];
    footSegment_L->relativePositionArray[SEGMENT_FOOT_CAL][SEGMENT_POSITIONZ] = foot_L[(SEGMENT_FOOT_CAL * 3) + SEGMENT_OFFSET2];
    footSegment_L->relativePositionArray[SEGMENT_FOOT_MT2][SEGMENT_POSITIONX] = foot_L[(SEGMENT_FOOT_MT2 * 3) + SEGMENT_OFFSET0];
    footSegment_L->relativePositionArray[SEGMENT_FOOT_MT2][SEGMENT_POSITIONY] = foot_L[(SEGMENT_FOOT_MT2 * 3) + SEGMENT_OFFSET1];
    footSegment_L->relativePositionArray[SEGMENT_FOOT_MT2][SEGMENT_POSITIONZ] = foot_L[(SEGMENT_FOOT_MT2 * 3) + SEGMENT_OFFSET2];
    footSegment_L->relativePositionArray[SEGMENT_FOOT_MT5][SEGMENT_POSITIONX] = foot_L[(SEGMENT_FOOT_MT5 * 3) + SEGMENT_OFFSET0];
    footSegment_L->relativePositionArray[SEGMENT_FOOT_MT5][SEGMENT_POSITIONY] = foot_L[(SEGMENT_FOOT_MT5 * 3) + SEGMENT_OFFSET1];
    footSegment_L->relativePositionArray[SEGMENT_FOOT_MT5][SEGMENT_POSITIONZ] = foot_L[(SEGMENT_FOOT_MT5 * 3) + SEGMENT_OFFSET2];

    // Set right foot parameters
    footSegment_R->lengthX = foot_R[REFERNCE_LENGTHX];
    footSegment_R->lengthY = foot_R[REFERNCE_LENGTHY];
    footSegment_R->lengthZ = foot_R[REFERNCE_LENGTHZ];
    footSegment_R->mass = foot_R[SEGMENT_MASS];
    footSegment_R->comX = foot_R[SEGMENT_COMX];
    footSegment_R->comY = foot_R[SEGMENT_COMY];
    footSegment_R->comZ = foot_R[SEGMENT_COMZ];
    footSegment_R->moiXX = foot_R[SEGMENT_MOIXX];
    footSegment_R->moiYY = foot_R[SEGMENT_MOIYY];
    footSegment_R->moiZZ = foot_R[SEGMENT_MOIZZ];
    footSegment_R->poiXY = foot_R[SEGMENT_POIXY];
    footSegment_R->poiXZ = foot_R[SEGMENT_POIXZ];
    footSegment_R->poiYZ = foot_R[SEGMENT_POIYZ];
    footSegment_R->relativePositionArray[SEGMENT_FOOT_CAL][SEGMENT_POSITIONX] = foot_R[(SEGMENT_FOOT_CAL * 3) + SEGMENT_OFFSET0];
    footSegment_R->relativePositionArray[SEGMENT_FOOT_CAL][SEGMENT_POSITIONY] = foot_R[(SEGMENT_FOOT_CAL * 3) + SEGMENT_OFFSET1];
    footSegment_R->relativePositionArray[SEGMENT_FOOT_CAL][SEGMENT_POSITIONZ] = foot_R[(SEGMENT_FOOT_CAL * 3) + SEGMENT_OFFSET2];
    footSegment_R->relativePositionArray[SEGMENT_FOOT_MT2][SEGMENT_POSITIONX] = foot_R[(SEGMENT_FOOT_MT2 * 3) + SEGMENT_OFFSET0];
    footSegment_R->relativePositionArray[SEGMENT_FOOT_MT2][SEGMENT_POSITIONY] = foot_R[(SEGMENT_FOOT_MT2 * 3) + SEGMENT_OFFSET1];
    footSegment_R->relativePositionArray[SEGMENT_FOOT_MT2][SEGMENT_POSITIONZ] = foot_R[(SEGMENT_FOOT_MT2 * 3) + SEGMENT_OFFSET2];
    footSegment_R->relativePositionArray[SEGMENT_FOOT_MT5][SEGMENT_POSITIONX] = foot_R[(SEGMENT_FOOT_MT5 * 3) + SEGMENT_OFFSET0];
    footSegment_R->relativePositionArray[SEGMENT_FOOT_MT5][SEGMENT_POSITIONY] = foot_R[(SEGMENT_FOOT_MT5 * 3) + SEGMENT_OFFSET1];
    footSegment_R->relativePositionArray[SEGMENT_FOOT_MT5][SEGMENT_POSITIONZ] = foot_R[(SEGMENT_FOOT_MT5 * 3) + SEGMENT_OFFSET2];

    // Generate model structure
    generateModelStructure();

}

void applyForwardKinematics(double* jointPositions) {

    if(jointPositions != nullptr) {

        // Set joint positions
        mbs->getJointByName("pBJX")->setJointPosition(jointPositions[JOINT_pBJX]);
        mbs->getJointByName("pBJY")->setJointPosition(jointPositions[JOINT_pBJY]);
        mbs->getJointByName("pBJZ")->setJointPosition(jointPositions[JOINT_pBJZ]);
        mbs->getJointByName("rBJX")->setJointPosition(jointPositions[JOINT_rBJX]);
        mbs->getJointByName("rBJY")->setJointPosition(jointPositions[JOINT_rBJY]);
        mbs->getJointByName("rBJZ")->setJointPosition(jointPositions[JOINT_rBJZ]);
        mbs->getJointByName("rLNJX")->setJointPosition(jointPositions[JOINT_rLNJX]);
        mbs->getJointByName("rLNJY")->setJointPosition(jointPositions[JOINT_rLNJY]);
        mbs->getJointByName("rLNJZ")->setJointPosition(jointPositions[JOINT_rLNJZ]);
        mbs->getJointByName("rSJX_L")->setJointPosition(jointPositions[JOINT_rSJX_L]);
        mbs->getJointByName("rSJY_L")->setJointPosition(jointPositions[JOINT_rSJY_L]);
        mbs->getJointByName("rSJZ_L")->setJointPosition(jointPositions[JOINT_rSJZ_L]);
        mbs->getJointByName("rSJX_R")->setJointPosition(jointPositions[JOINT_rSJX_R]);
        mbs->getJointByName("rSJY_R")->setJointPosition(jointPositions[JOINT_rSJY_R]);
        mbs->getJointByName("rSJZ_R")->setJointPosition(jointPositions[JOINT_rSJZ_R]);
        mbs->getJointByName("rEJZ_L")->setJointPosition(jointPositions[JOINT_rEJZ_L]);
        mbs->getJointByName("rEJZ_R")->setJointPosition(jointPositions[JOINT_rEJZ_R]);
        mbs->getJointByName("rULJX")->setJointPosition(jointPositions[JOINT_rULJX]);
        mbs->getJointByName("rULJY")->setJointPosition(jointPositions[JOINT_rULJY]);
        mbs->getJointByName("rULJZ")->setJointPosition(jointPositions[JOINT_rULJZ]);
        mbs->getJointByName("rLLJX")->setJointPosition(jointPositions[JOINT_rLLJX]);
        mbs->getJointByName("rLLJZ")->setJointPosition(jointPositions[JOINT_rLLJZ]);
        mbs->getJointByName("rHJX_L")->setJointPosition(jointPositions[JOINT_rHJX_L]);
        mbs->getJointByName("rHJY_L")->setJointPosition(jointPositions[JOINT_rHJY_L]);
        mbs->getJointByName("rHJZ_L")->setJointPosition(jointPositions[JOINT_rHJZ_L]);
        mbs->getJointByName("rHJX_R")->setJointPosition(jointPositions[JOINT_rHJX_R]);
        mbs->getJointByName("rHJY_R")->setJointPosition(jointPositions[JOINT_rHJY_R]);
        mbs->getJointByName("rHJZ_R")->setJointPosition(jointPositions[JOINT_rHJZ_R]);
        mbs->getJointByName("rKJZ_L")->setJointPosition(jointPositions[JOINT_rKJZ_L]);
        mbs->getJointByName("rKJZ_R")->setJointPosition(jointPositions[JOINT_rKJZ_R]);
        mbs->getJointByName("rAJX_L")->setJointPosition(jointPositions[JOINT_rAJX_L]);
        mbs->getJointByName("rAJY_L")->setJointPosition(jointPositions[JOINT_rAJY_L]);
        mbs->getJointByName("rAJZ_L")->setJointPosition(jointPositions[JOINT_rAJZ_L]);
        mbs->getJointByName("rAJX_R")->setJointPosition(jointPositions[JOINT_rAJX_R]);
        mbs->getJointByName("rAJY_R")->setJointPosition(jointPositions[JOINT_rAJY_R]);
        mbs->getJointByName("rAJZ_R")->setJointPosition(jointPositions[JOINT_rAJZ_R]);

        // Start tape for derivatives
        dom->startTape();

        // Apply forward kinematics
        mbs->doDirkin();

        // Stop tape for derivatives
        dom->endTape();

    } else {

        // Print error
        std::cout << "ERROR: There were invalid pointers when calling applyForwardKinematics function!" << std::endl;

    }

}

void getForwardKinematicsPositions(double* elementPositionsX, double* elementPositionsY, double* elementPositionsZ) {

    if((elementPositionsX != nullptr) && (elementPositionsY != nullptr) && (elementPositionsZ != nullptr)) {

        // Read element positions
        elementPositionsX[ELEMENT_TRA_L] = mbs->getEndpointByName("TRA_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_TRA_L] = mbs->getEndpointByName("TRA_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_TRA_L] = mbs->getEndpointByName("TRA_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_TRA_R] = mbs->getEndpointByName("TRA_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_TRA_R] = mbs->getEndpointByName("TRA_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_TRA_R] = mbs->getEndpointByName("TRA_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_GLA] = mbs->getEndpointByName("GLA")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_GLA] = mbs->getEndpointByName("GLA")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_GLA] = mbs->getEndpointByName("GLA")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_ACR_L] = mbs->getEndpointByName("ACR_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_ACR_L] = mbs->getEndpointByName("ACR_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_ACR_L] = mbs->getEndpointByName("ACR_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_ACR_R] = mbs->getEndpointByName("ACR_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_ACR_R] = mbs->getEndpointByName("ACR_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_ACR_R] = mbs->getEndpointByName("ACR_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_LHC_L] = mbs->getEndpointByName("LHC_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_LHC_L] = mbs->getEndpointByName("LHC_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_LHC_L] = mbs->getEndpointByName("LHC_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_LHC_R] = mbs->getEndpointByName("LHC_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_LHC_R] = mbs->getEndpointByName("LHC_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_LHC_R] = mbs->getEndpointByName("LHC_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_WRI_L] = mbs->getEndpointByName("WRI_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_WRI_L] = mbs->getEndpointByName("WRI_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_WRI_L] = mbs->getEndpointByName("WRI_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_WRI_R] = mbs->getEndpointByName("WRI_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_WRI_R] = mbs->getEndpointByName("WRI_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_WRI_R] = mbs->getEndpointByName("WRI_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_SUP] = mbs->getEndpointByName("SUP")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_SUP] = mbs->getEndpointByName("SUP")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_SUP] = mbs->getEndpointByName("SUP")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_C7] = mbs->getEndpointByName("C7")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_C7] = mbs->getEndpointByName("C7")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_C7] = mbs->getEndpointByName("C7")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_T8] = mbs->getEndpointByName("T8")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_T8] = mbs->getEndpointByName("T8")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_T8] = mbs->getEndpointByName("T8")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_T12] = mbs->getEndpointByName("T12")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_T12] = mbs->getEndpointByName("T12")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_T12] = mbs->getEndpointByName("T12")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_ASIS_L] = mbs->getEndpointByName("ASIS_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_ASIS_L] = mbs->getEndpointByName("ASIS_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_ASIS_L] = mbs->getEndpointByName("ASIS_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_ASIS_R] = mbs->getEndpointByName("ASIS_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_ASIS_R] = mbs->getEndpointByName("ASIS_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_ASIS_R] = mbs->getEndpointByName("ASIS_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_PSIS_L] = mbs->getEndpointByName("PSIS_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_PSIS_L] = mbs->getEndpointByName("PSIS_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_PSIS_L] = mbs->getEndpointByName("PSIS_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_PSIS_R] = mbs->getEndpointByName("PSIS_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_PSIS_R] = mbs->getEndpointByName("PSIS_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_PSIS_R] = mbs->getEndpointByName("PSIS_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_PS] = mbs->getEndpointByName("PS")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_PS] = mbs->getEndpointByName("PS")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_PS] = mbs->getEndpointByName("PS")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_GTR_L] = mbs->getEndpointByName("GTR_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_GTR_L] = mbs->getEndpointByName("GTR_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_GTR_L] = mbs->getEndpointByName("GTR_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_GTR_R] = mbs->getEndpointByName("GTR_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_GTR_R] = mbs->getEndpointByName("GTR_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_GTR_R] = mbs->getEndpointByName("GTR_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_LFC_L] = mbs->getEndpointByName("LFC_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_LFC_L] = mbs->getEndpointByName("LFC_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_LFC_L] = mbs->getEndpointByName("LFC_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_LFC_R] = mbs->getEndpointByName("LFC_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_LFC_R] = mbs->getEndpointByName("LFC_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_LFC_R] = mbs->getEndpointByName("LFC_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_MFC_L] = mbs->getEndpointByName("MFC_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_MFC_L] = mbs->getEndpointByName("MFC_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_MFC_L] = mbs->getEndpointByName("MFC_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_MFC_R] = mbs->getEndpointByName("MFC_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_MFC_R] = mbs->getEndpointByName("MFC_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_MFC_R] = mbs->getEndpointByName("MFC_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_LM_L] = mbs->getEndpointByName("LM_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_LM_L] = mbs->getEndpointByName("LM_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_LM_L] = mbs->getEndpointByName("LM_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_LM_R] = mbs->getEndpointByName("LM_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_LM_R] = mbs->getEndpointByName("LM_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_LM_R] = mbs->getEndpointByName("LM_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_MM_L] = mbs->getEndpointByName("MM_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_MM_L] = mbs->getEndpointByName("MM_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_MM_L] = mbs->getEndpointByName("MM_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_MM_R] = mbs->getEndpointByName("MM_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_MM_R] = mbs->getEndpointByName("MM_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_MM_R] = mbs->getEndpointByName("MM_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_CAL_L] = mbs->getEndpointByName("CAL_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_CAL_L] = mbs->getEndpointByName("CAL_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_CAL_L] = mbs->getEndpointByName("CAL_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_CAL_R] = mbs->getEndpointByName("CAL_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_CAL_R] = mbs->getEndpointByName("CAL_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_CAL_R] = mbs->getEndpointByName("CAL_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_MT2_L] = mbs->getEndpointByName("MT2_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_MT2_L] = mbs->getEndpointByName("MT2_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_MT2_L] = mbs->getEndpointByName("MT2_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_MT2_R] = mbs->getEndpointByName("MT2_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_MT2_R] = mbs->getEndpointByName("MT2_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_MT2_R] = mbs->getEndpointByName("MT2_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_MT5_L] = mbs->getEndpointByName("MT5_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_MT5_L] = mbs->getEndpointByName("MT5_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_MT5_L] = mbs->getEndpointByName("MT5_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_MT5_R] = mbs->getEndpointByName("MT5_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_MT5_R] = mbs->getEndpointByName("MT5_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_MT5_R] = mbs->getEndpointByName("MT5_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_LNJ] = mbs->getJointByName("rLNJX")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_LNJ] = mbs->getJointByName("rLNJX")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_LNJ] = mbs->getJointByName("rLNJX")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_SJ_L] = mbs->getJointByName("rSJX_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_SJ_L] = mbs->getJointByName("rSJX_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_SJ_L] = mbs->getJointByName("rSJX_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_SJ_R] = mbs->getJointByName("rSJX_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_SJ_R] = mbs->getJointByName("rSJX_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_SJ_R] = mbs->getJointByName("rSJX_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_EJ_L] = mbs->getJointByName("rEJZ_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_EJ_L] = mbs->getJointByName("rEJZ_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_EJ_L] = mbs->getJointByName("rEJZ_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_EJ_R] = mbs->getJointByName("rEJZ_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_EJ_R] = mbs->getJointByName("rEJZ_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_EJ_R] = mbs->getJointByName("rEJZ_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_ULJ] = mbs->getJointByName("rULJX")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_ULJ] = mbs->getJointByName("rULJX")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_ULJ] = mbs->getJointByName("rULJX")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_LLJ] = mbs->getJointByName("rLLJX")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_LLJ] = mbs->getJointByName("rLLJX")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_LLJ] = mbs->getJointByName("rLLJX")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_HJ_L] = mbs->getJointByName("rHJX_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_HJ_L] = mbs->getJointByName("rHJX_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_HJ_L] = mbs->getJointByName("rHJX_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_HJ_R] = mbs->getJointByName("rHJX_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_HJ_R] = mbs->getJointByName("rHJX_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_HJ_R] = mbs->getJointByName("rHJX_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_KJ_L] = mbs->getJointByName("rKJZ_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_KJ_L] = mbs->getJointByName("rKJZ_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_KJ_L] = mbs->getJointByName("rKJZ_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_KJ_R] = mbs->getJointByName("rKJZ_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_KJ_R] = mbs->getJointByName("rKJZ_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_KJ_R] = mbs->getJointByName("rKJZ_R")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_AJ_L] = mbs->getJointByName("rAJX_L")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_AJ_L] = mbs->getJointByName("rAJX_L")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_AJ_L] = mbs->getJointByName("rAJX_L")->getCoordinateFrame().r(2).getValue();
        elementPositionsX[ELEMENT_AJ_R] = mbs->getJointByName("rAJX_R")->getCoordinateFrame().r(0).getValue();
        elementPositionsY[ELEMENT_AJ_R] = mbs->getJointByName("rAJX_R")->getCoordinateFrame().r(1).getValue();
        elementPositionsZ[ELEMENT_AJ_R] = mbs->getJointByName("rAJX_R")->getCoordinateFrame().r(2).getValue();

    } else {

        // Print error
        std::cout << "ERROR: There were invalid pointers when calling readPositions function!" << std::endl;

    }

}

void getForwardKinematicsJacobian(double* jacobianVector) {

    mbslib::DomMatrix m = dom->calculateJacobian();
    mbslib::DomVector v1 = mbslib::convert(m);
    Eigen::Map<Eigen::VectorXd> v2(jacobianVector, v1.size());
    v2 = v1;

}

void getForwardKinematicsJacobianSize(int* rows, int* columns) {

    *rows = dom->getDependents();
    *columns = dom->getIndependents();

}

void getModelDOF(int *dof) {

    *dof = JOINTS;

}

void getModelElements(int *elements) {

    *elements = ELEMENTS;

}

// Private functions
void initializeVariables(void) {

    // Initialize variables
    mbs = new mbslib::MbsCompoundWithBuilder("HuMoD");
    dom = new mbslib::DeriveOMat(*mbs);
    gravity = new mbslib::TScalar();
    headSegment = new Segment("Head", 3);
    thoraxSegment = new Segment("Thorax", 7);
    abdomenSegment = new Segment("Abdomen", 2);
    pelvisSegment = new Segment("Pelvis", 7);
    upperArmSegment_L = new Segment("Left upper arm", 2);
    upperArmSegment_R = new Segment("Right upper arm", 2);
    lowerArmSegment_L = new Segment("Left lower arm", 2);
    lowerArmSegment_R = new Segment("Right lower arm", 2);
    thighSegment_L = new Segment("Left thigh", 4);
    thighSegment_R = new Segment("Right thigh", 4);
    shankSegment_L = new Segment("Left shank", 3);
    shankSegment_R = new Segment("Right shank", 3);
    footSegment_L = new Segment("Left foot", 3);
    footSegment_R = new Segment("Right foot", 3);

}

void generateModelStructure(void) {

    // Add gravity
    mbs->setGravitation(mbslib::TVector3(0, -*gravity, 0));

    // Add fixed base with joints at lower lumbar joint
    mbs->addFixedBase("modelBase");
    mbs->addPrismaticJoint(mbslib::TVector3(1, 0, 0), "pBJX");
    mbs->addPrismaticJoint(mbslib::TVector3(0, 1, 0), "pBJY");
    mbs->addPrismaticJoint(mbslib::TVector3(0, 0, 1), "pBJZ");
    mbs->addRevoluteJoint(mbslib::TVector3(1, 0, 0), "rBJX");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 1, 0), "rBJY");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 0, 1), "rBJZ");

    // Split upper and lower body
    mbs->addFork();

    // ++++++++++
    // Upper body
    // ++++++++++

    // Add lower lumbar joints, abdomen segment and abdomen marker
    mbs->addRevoluteJoint(mbslib::TVector3(1, 0, 0), "rLLJX");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 0, 1), "rLLJZ");
    mbs->addRigidLink(
        mbslib::TVector3(0, abdomenSegment->lengthY, 0),
        mbslib::TVector3(abdomenSegment->comX, abdomenSegment->comY, abdomenSegment->comZ),
        abdomenSegment->mass,
        mbslib::makeInertiaTensor(abdomenSegment->moiXX, abdomenSegment->moiYY, abdomenSegment->moiZZ, abdomenSegment->poiXY, abdomenSegment->poiXZ, abdomenSegment->poiYZ),
        "Abdomen"
    );
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(abdomenSegment->relativePositionArray[SEGMENT_ABDOMEN_T12][SEGMENT_POSITIONX], abdomenSegment->relativePositionArray[SEGMENT_ABDOMEN_T12][SEGMENT_POSITIONY], abdomenSegment->relativePositionArray[SEGMENT_ABDOMEN_T12][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("T12");

    // Add upper lumbar joints, thorax segment and thorax markers
    mbs->addRevoluteJoint(mbslib::TVector3(1, 0, 0), "rULJX");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 1, 0), "rULJY");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 0, 1), "rULJZ");
    mbs->addRigidLink(
        mbslib::TVector3(0, thoraxSegment->lengthY, 0),
        mbslib::TVector3(thoraxSegment->comX, thoraxSegment->comY, thoraxSegment->comZ),
        thoraxSegment->mass,
        mbslib::makeInertiaTensor(thoraxSegment->moiXX, thoraxSegment->moiYY, thoraxSegment->moiZZ, thoraxSegment->poiXY, thoraxSegment->poiXZ, thoraxSegment->poiYZ),
        "Thorax"
    );
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(thoraxSegment->relativePositionArray[SEGMENT_THORAX_ACR_L][SEGMENT_POSITIONX], thoraxSegment->relativePositionArray[SEGMENT_THORAX_ACR_L][SEGMENT_POSITIONY], thoraxSegment->relativePositionArray[SEGMENT_THORAX_ACR_L][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("ACR_L");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(thoraxSegment->relativePositionArray[SEGMENT_THORAX_ACR_R][SEGMENT_POSITIONX], thoraxSegment->relativePositionArray[SEGMENT_THORAX_ACR_R][SEGMENT_POSITIONY], thoraxSegment->relativePositionArray[SEGMENT_THORAX_ACR_R][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("ACR_R");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(thoraxSegment->relativePositionArray[SEGMENT_THORAX_SUP][SEGMENT_POSITIONX], thoraxSegment->relativePositionArray[SEGMENT_THORAX_SUP][SEGMENT_POSITIONY], thoraxSegment->relativePositionArray[SEGMENT_THORAX_SUP][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("SUP");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(thoraxSegment->relativePositionArray[SEGMENT_THORAX_C7][SEGMENT_POSITIONX], thoraxSegment->relativePositionArray[SEGMENT_THORAX_C7][SEGMENT_POSITIONY], thoraxSegment->relativePositionArray[SEGMENT_THORAX_C7][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("C7");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(thoraxSegment->relativePositionArray[SEGMENT_THORAX_T8][SEGMENT_POSITIONX], thoraxSegment->relativePositionArray[SEGMENT_THORAX_T8][SEGMENT_POSITIONY], thoraxSegment->relativePositionArray[SEGMENT_THORAX_T8][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("T8");

    // Split head and arms
    mbs->addFork();
    mbs->addFork();

    // Add lower neck joints, head segment and head markers
    mbs->addRevoluteJoint(mbslib::TVector3(1, 0, 0), "rLNJX");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 1, 0), "rLNJY");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 0, 1), "rLNJZ");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(headSegment->relativePositionArray[SEGMENT_HEAD_TRA_L][SEGMENT_POSITIONX], headSegment->relativePositionArray[SEGMENT_HEAD_TRA_L][SEGMENT_POSITIONY], headSegment->relativePositionArray[SEGMENT_HEAD_TRA_L][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("TRA_L");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(headSegment->relativePositionArray[SEGMENT_HEAD_TRA_R][SEGMENT_POSITIONX], headSegment->relativePositionArray[SEGMENT_HEAD_TRA_R][SEGMENT_POSITIONY], headSegment->relativePositionArray[SEGMENT_HEAD_TRA_R][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("TRA_R");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(headSegment->relativePositionArray[SEGMENT_HEAD_GLA][SEGMENT_POSITIONX], headSegment->relativePositionArray[SEGMENT_HEAD_GLA][SEGMENT_POSITIONY], headSegment->relativePositionArray[SEGMENT_HEAD_GLA][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("GLA");
    mbs->addRigidLink(
        mbslib::TVector3(0, headSegment->lengthY, 0),
        mbslib::TVector3(headSegment->comX, (headSegment->comY - headSegment->lengthY), headSegment->comZ),
        headSegment->mass,
        mbslib::makeInertiaTensor(headSegment->moiXX, headSegment->moiYY, headSegment->moiZZ, headSegment->poiXY, headSegment->poiXZ, headSegment->poiYZ),
        "Head"
    );
    mbs->addEndpoint("VTX");

    // Add left shoulder joints, upper arm segment and upper arm markers
    mbs->addFixedTranslation(mbslib::TVector3(thoraxSegment->relativePositionArray[SEGMENT_THORAX_SJ_L][SEGMENT_POSITIONX], thoraxSegment->relativePositionArray[SEGMENT_THORAX_SJ_L][SEGMENT_POSITIONY], thoraxSegment->relativePositionArray[SEGMENT_THORAX_SJ_L][SEGMENT_POSITIONZ]));
    mbs->addRevoluteJoint(mbslib::TVector3(1, 0, 0), "rSJX_L");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 1, 0), "rSJY_L");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 0, 1), "rSJZ_L");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(upperArmSegment_L->relativePositionArray[SEGMENT_UPPER_ARM_LHC][SEGMENT_POSITIONX], upperArmSegment_L->relativePositionArray[SEGMENT_UPPER_ARM_LHC][SEGMENT_POSITIONY], upperArmSegment_L->relativePositionArray[SEGMENT_UPPER_ARM_LHC][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("LHC_L");
    mbs->addRigidLink(
        mbslib::TVector3(0, -upperArmSegment_L->lengthY, 0),
        mbslib::TVector3(upperArmSegment_L->comX, (upperArmSegment_L->comY + upperArmSegment_L->lengthY), upperArmSegment_L->comZ),
        upperArmSegment_L->mass,
        mbslib::makeInertiaTensor(upperArmSegment_L->moiXX, upperArmSegment_L->moiYY, upperArmSegment_L->moiZZ, upperArmSegment_L->poiXY, upperArmSegment_L->poiXZ, upperArmSegment_L->poiYZ),
        "UpperArm_L"
    );

    // Add left elbow joint, lower arm segment and lower arm markers
    mbs->addRevoluteJoint(mbslib::TVector3(0, 0, 1), "rEJZ_L");
    mbs->addRigidLink(
        mbslib::TVector3(0, -lowerArmSegment_L->lengthY, 0),
        mbslib::TVector3(lowerArmSegment_L->comX, (lowerArmSegment_L->comY + lowerArmSegment_L->lengthY), lowerArmSegment_L->comZ),
        lowerArmSegment_L->mass,
        mbslib::makeInertiaTensor(lowerArmSegment_L->moiXX, lowerArmSegment_L->moiYY, lowerArmSegment_L->moiZZ, lowerArmSegment_L->poiXY, lowerArmSegment_L->poiXZ, lowerArmSegment_L->poiYZ),
        "LowerArm_L"
    );
    mbs->addEndpoint("WRI_L");

    // Add right shoulder joints, upper arm segment and upper arm markers
    mbs->addFixedTranslation(mbslib::TVector3(thoraxSegment->relativePositionArray[SEGMENT_THORAX_SJ_R][SEGMENT_POSITIONX], thoraxSegment->relativePositionArray[SEGMENT_THORAX_SJ_R][SEGMENT_POSITIONY], thoraxSegment->relativePositionArray[SEGMENT_THORAX_SJ_R][SEGMENT_POSITIONZ]));
    mbs->addRevoluteJoint(mbslib::TVector3(1, 0, 0), "rSJX_R");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 1, 0), "rSJY_R");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 0, 1), "rSJZ_R");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(upperArmSegment_R->relativePositionArray[SEGMENT_UPPER_ARM_LHC][SEGMENT_POSITIONX], upperArmSegment_R->relativePositionArray[SEGMENT_UPPER_ARM_LHC][SEGMENT_POSITIONY], upperArmSegment_R->relativePositionArray[SEGMENT_UPPER_ARM_LHC][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("LHC_R");
    mbs->addRigidLink(
        mbslib::TVector3(0, -upperArmSegment_R->lengthY, 0),
        mbslib::TVector3(upperArmSegment_R->comX, (upperArmSegment_R->comY + upperArmSegment_R->lengthY), upperArmSegment_R->comZ),
        upperArmSegment_R->mass,
        mbslib::makeInertiaTensor(upperArmSegment_R->moiXX, upperArmSegment_R->moiYY, upperArmSegment_R->moiZZ, upperArmSegment_R->poiXY, upperArmSegment_R->poiXZ, upperArmSegment_R->poiYZ),
        "UpperArm_R"
    );

    // Add right elbow joint, lower arm segment and lower arm markers
    mbs->addRevoluteJoint(mbslib::TVector3(0, 0, 1), "rEJZ_R");
    mbs->addRigidLink(
        mbslib::TVector3(0, -lowerArmSegment_R->lengthY, 0),
        mbslib::TVector3(lowerArmSegment_R->comX, (lowerArmSegment_R->comY + lowerArmSegment_R->lengthY), lowerArmSegment_R->comZ),
        lowerArmSegment_R->mass,
        mbslib::makeInertiaTensor(lowerArmSegment_R->moiXX, lowerArmSegment_R->moiYY, lowerArmSegment_R->moiZZ, lowerArmSegment_R->poiXY, lowerArmSegment_R->poiXZ, lowerArmSegment_R->poiYZ),
        "LowerArm_R"
    );
    mbs->addEndpoint("WRI_R");

    // ++++++++++
    // Lower body
    // ++++++++++

    // Add pelvis segment and pelvis markers
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(pelvisSegment->relativePositionArray[SEGMENT_PELVIS_ASIS_L][SEGMENT_POSITIONX], pelvisSegment->relativePositionArray[SEGMENT_PELVIS_ASIS_L][SEGMENT_POSITIONY], pelvisSegment->relativePositionArray[SEGMENT_PELVIS_ASIS_L][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("ASIS_L");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(pelvisSegment->relativePositionArray[SEGMENT_PELVIS_ASIS_R][SEGMENT_POSITIONX], pelvisSegment->relativePositionArray[SEGMENT_PELVIS_ASIS_R][SEGMENT_POSITIONY], pelvisSegment->relativePositionArray[SEGMENT_PELVIS_ASIS_R][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("ASIS_R");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PSIS_L][SEGMENT_POSITIONX], pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PSIS_L][SEGMENT_POSITIONY], pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PSIS_L][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("PSIS_L");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PSIS_R][SEGMENT_POSITIONX], pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PSIS_R][SEGMENT_POSITIONY], pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PSIS_R][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("PSIS_R");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PS][SEGMENT_POSITIONX], pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PS][SEGMENT_POSITIONY], pelvisSegment->relativePositionArray[SEGMENT_PELVIS_PS][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("PS");
    mbs->addRigidLink(
        mbslib::TVector3(0, -pelvisSegment->lengthY, 0),
        mbslib::TVector3(pelvisSegment->comX, (pelvisSegment->comY + pelvisSegment->lengthY), pelvisSegment->comZ),
        pelvisSegment->mass,
        mbslib::makeInertiaTensor(pelvisSegment->moiXX, pelvisSegment->moiYY, pelvisSegment->moiZZ, pelvisSegment->poiXY, pelvisSegment->poiXZ, pelvisSegment->poiYZ),
        "Pelvis"
    );

    // Split legs
    mbs->addFork();

    // Add left hip joints, thigh segment and thigh markers
    mbs->addFixedTranslation(mbslib::TVector3(pelvisSegment->relativePositionArray[SEGMENT_PELVIS_HJ_L][SEGMENT_POSITIONX], (pelvisSegment->relativePositionArray[SEGMENT_PELVIS_HJ_L][SEGMENT_POSITIONY] + pelvisSegment->lengthY), pelvisSegment->relativePositionArray[SEGMENT_PELVIS_HJ_L][SEGMENT_POSITIONZ]));
    mbs->addRevoluteJoint(mbslib::TVector3(1, 0, 0), "rHJX_L");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 1, 0), "rHJY_L");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 0, 1), "rHJZ_L");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(thighSegment_L->relativePositionArray[SEGMENT_THIGH_GTR][SEGMENT_POSITIONX], thighSegment_L->relativePositionArray[SEGMENT_THIGH_GTR][SEGMENT_POSITIONY], thighSegment_L->relativePositionArray[SEGMENT_THIGH_GTR][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("GTR_L");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(thighSegment_L->relativePositionArray[SEGMENT_THIGH_LFC][SEGMENT_POSITIONX], thighSegment_L->relativePositionArray[SEGMENT_THIGH_LFC][SEGMENT_POSITIONY], thighSegment_L->relativePositionArray[SEGMENT_THIGH_LFC][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("LFC_L");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(thighSegment_L->relativePositionArray[SEGMENT_THIGH_MFC][SEGMENT_POSITIONX], thighSegment_L->relativePositionArray[SEGMENT_THIGH_MFC][SEGMENT_POSITIONY], thighSegment_L->relativePositionArray[SEGMENT_THIGH_MFC][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("MFC_L");
    mbs->addRigidLink(
        mbslib::TVector3(0, -thighSegment_L->lengthY, 0),
        mbslib::TVector3(thighSegment_L->comX, (thighSegment_L->comY + thighSegment_L->lengthY), thighSegment_L->comZ),
        thighSegment_L->mass,
        mbslib::makeInertiaTensor(thighSegment_L->moiXX, thighSegment_L->moiYY, thighSegment_L->moiZZ, thighSegment_L->poiXY, thighSegment_L->poiXZ, thighSegment_L->poiYZ),
        "Thigh_L"
    );

    // Add left knee joint, shank segment and shank markers
    mbs->addRevoluteJoint(mbslib::TVector3(0, 0, 1), "rKJZ_L");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(shankSegment_L->relativePositionArray[SEGMENT_SHANK_LM][SEGMENT_POSITIONX], shankSegment_L->relativePositionArray[SEGMENT_SHANK_LM][SEGMENT_POSITIONY], shankSegment_L->relativePositionArray[SEGMENT_SHANK_LM][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("LM_L");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(shankSegment_L->relativePositionArray[SEGMENT_SHANK_MM][SEGMENT_POSITIONX], shankSegment_L->relativePositionArray[SEGMENT_SHANK_MM][SEGMENT_POSITIONY], shankSegment_L->relativePositionArray[SEGMENT_SHANK_MM][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("MM_L");
    mbs->addRigidLink(
        mbslib::TVector3(0, -shankSegment_L->lengthY, 0),
        mbslib::TVector3(shankSegment_L->comX, (shankSegment_L->comY + shankSegment_L->lengthY), shankSegment_L->comZ),
        shankSegment_L->mass,
        mbslib::makeInertiaTensor(shankSegment_L->moiXX, shankSegment_L->moiYY, shankSegment_L->moiZZ, shankSegment_L->poiXY, shankSegment_L->poiXZ, shankSegment_L->poiYZ),
        "Shank_L"
    );

    // Add left ankle joint, foot segment and foot markers
    mbs->addRevoluteJoint(mbslib::TVector3(1, 0, 0), "rAJX_L");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 1, 0), "rAJY_L");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 0, 1), "rAJZ_L");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(footSegment_L->relativePositionArray[SEGMENT_FOOT_CAL][SEGMENT_POSITIONX], footSegment_L->relativePositionArray[SEGMENT_FOOT_CAL][SEGMENT_POSITIONY], footSegment_L->relativePositionArray[SEGMENT_FOOT_CAL][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("CAL_L");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(footSegment_L->relativePositionArray[SEGMENT_FOOT_MT2][SEGMENT_POSITIONX], footSegment_L->relativePositionArray[SEGMENT_FOOT_MT2][SEGMENT_POSITIONY], footSegment_L->relativePositionArray[SEGMENT_FOOT_MT2][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("MT2_L");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(footSegment_L->relativePositionArray[SEGMENT_FOOT_MT5][SEGMENT_POSITIONX], footSegment_L->relativePositionArray[SEGMENT_FOOT_MT5][SEGMENT_POSITIONY], footSegment_L->relativePositionArray[SEGMENT_FOOT_MT5][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("MT5_L");
    mbs->addRigidLink(
        mbslib::TVector3(footSegment_L->lengthX, footSegment_L->lengthY, footSegment_L->lengthZ),
        mbslib::TVector3((footSegment_L->comX - footSegment_L->lengthX), (footSegment_L->comY - footSegment_L->lengthY), (footSegment_L->comZ - footSegment_L->lengthZ)),
        footSegment_L->mass,
        mbslib::makeInertiaTensor(footSegment_L->moiXX, footSegment_L->moiYY, footSegment_L->moiZZ, footSegment_L->poiXY, footSegment_L->poiXZ, footSegment_L->poiYZ),
        "Foot_L"
    );
    mbs->addEndpoint("MT_L");

    // Add right hip joints, thigh segment and thigh markers
    mbs->addFixedTranslation(mbslib::TVector3(pelvisSegment->relativePositionArray[SEGMENT_PELVIS_HJ_R][SEGMENT_POSITIONX], (pelvisSegment->relativePositionArray[SEGMENT_PELVIS_HJ_R][SEGMENT_POSITIONY] + pelvisSegment->lengthY), pelvisSegment->relativePositionArray[SEGMENT_PELVIS_HJ_R][SEGMENT_POSITIONZ]));
    mbs->addRevoluteJoint(mbslib::TVector3(1, 0, 0), "rHJX_R");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 1, 0), "rHJY_R");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 0, 1), "rHJZ_R");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(thighSegment_R->relativePositionArray[SEGMENT_THIGH_GTR][SEGMENT_POSITIONX], thighSegment_R->relativePositionArray[SEGMENT_THIGH_GTR][SEGMENT_POSITIONY], thighSegment_R->relativePositionArray[SEGMENT_THIGH_GTR][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("GTR_R");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(thighSegment_R->relativePositionArray[SEGMENT_THIGH_LFC][SEGMENT_POSITIONX], thighSegment_R->relativePositionArray[SEGMENT_THIGH_LFC][SEGMENT_POSITIONY], thighSegment_R->relativePositionArray[SEGMENT_THIGH_LFC][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("LFC_R");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(thighSegment_R->relativePositionArray[SEGMENT_THIGH_MFC][SEGMENT_POSITIONX], thighSegment_R->relativePositionArray[SEGMENT_THIGH_MFC][SEGMENT_POSITIONY], thighSegment_R->relativePositionArray[SEGMENT_THIGH_MFC][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("MFC_R");
    mbs->addRigidLink(
        mbslib::TVector3(0, -thighSegment_R->lengthY, 0),
        mbslib::TVector3(thighSegment_R->comX, (thighSegment_R->comY + thighSegment_R->lengthY), thighSegment_R->comZ),
        thighSegment_R->mass,
        mbslib::makeInertiaTensor(thighSegment_R->moiXX, thighSegment_R->moiYY, thighSegment_R->moiZZ, thighSegment_R->poiXY, thighSegment_R->poiXZ, thighSegment_R->poiYZ),
        "Thigh_L"
    );

    // Add right knee joint, shank segment and shank markers
    mbs->addRevoluteJoint(mbslib::TVector3(0, 0, 1), "rKJZ_R");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(shankSegment_R->relativePositionArray[SEGMENT_SHANK_LM][SEGMENT_POSITIONX], shankSegment_R->relativePositionArray[SEGMENT_SHANK_LM][SEGMENT_POSITIONY], shankSegment_R->relativePositionArray[SEGMENT_SHANK_LM][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("LM_R");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(shankSegment_R->relativePositionArray[SEGMENT_SHANK_MM][SEGMENT_POSITIONX], shankSegment_R->relativePositionArray[SEGMENT_SHANK_MM][SEGMENT_POSITIONY], shankSegment_R->relativePositionArray[SEGMENT_SHANK_MM][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("MM_R");
    mbs->addRigidLink(
        mbslib::TVector3(0, -shankSegment_R->lengthY, 0),
        mbslib::TVector3(shankSegment_R->comX, (shankSegment_R->comY + shankSegment_R->lengthY), shankSegment_R->comZ),
        shankSegment_R->mass,
        mbslib::makeInertiaTensor(shankSegment_R->moiXX, shankSegment_R->moiYY, shankSegment_R->moiZZ, shankSegment_R->poiXY, shankSegment_R->poiXZ, shankSegment_R->poiYZ),
        "Shank_R"
    );

    // Add right ankle joint, foot segment and foot markers
    mbs->addRevoluteJoint(mbslib::TVector3(1, 0, 0), "rAJX_R");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 1, 0), "rAJY_R");
    mbs->addRevoluteJoint(mbslib::TVector3(0, 0, 1), "rAJZ_R");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(footSegment_R->relativePositionArray[SEGMENT_FOOT_CAL][SEGMENT_POSITIONX], footSegment_R->relativePositionArray[SEGMENT_FOOT_CAL][SEGMENT_POSITIONY], footSegment_R->relativePositionArray[SEGMENT_FOOT_CAL][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("CAL_R");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(footSegment_R->relativePositionArray[SEGMENT_FOOT_MT2][SEGMENT_POSITIONX], footSegment_R->relativePositionArray[SEGMENT_FOOT_MT2][SEGMENT_POSITIONY], footSegment_R->relativePositionArray[SEGMENT_FOOT_MT2][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("MT2_R");
    mbs->addFork();
    mbs->addFixedTranslation(mbslib::TVector3(footSegment_R->relativePositionArray[SEGMENT_FOOT_MT5][SEGMENT_POSITIONX], footSegment_R->relativePositionArray[SEGMENT_FOOT_MT5][SEGMENT_POSITIONY], footSegment_R->relativePositionArray[SEGMENT_FOOT_MT5][SEGMENT_POSITIONZ]));
    mbs->addEndpoint("MT5_R");
    mbs->addRigidLink(
        mbslib::TVector3(footSegment_R->lengthX, footSegment_R->lengthY, footSegment_R->lengthZ),
        mbslib::TVector3((footSegment_R->comX - footSegment_R->lengthX), (footSegment_R->comY - footSegment_R->lengthY), (footSegment_R->comZ - footSegment_R->lengthZ)),
        footSegment_R->mass,
        mbslib::makeInertiaTensor(footSegment_R->moiXX, footSegment_R->moiYY, footSegment_R->moiZZ, footSegment_R->poiXY, footSegment_R->poiXZ, footSegment_R->poiYZ),
        "Foot_R"
    );
    mbs->addEndpoint("MT_R");

    // Add independent variables
    dom->addIndependent("pBJX", "q");
    dom->addIndependent("pBJY", "q");
    dom->addIndependent("pBJZ", "q");
    dom->addIndependent("rBJX", "q");
    dom->addIndependent("rBJY", "q");
    dom->addIndependent("rBJZ", "q");
    dom->addIndependent("rLNJX", "q");
    dom->addIndependent("rLNJY", "q");
    dom->addIndependent("rLNJZ", "q");
    dom->addIndependent("rSJX_L", "q");
    dom->addIndependent("rSJY_L", "q");
    dom->addIndependent("rSJZ_L", "q");
    dom->addIndependent("rSJX_R", "q");
    dom->addIndependent("rSJY_R", "q");
    dom->addIndependent("rSJZ_R", "q");
    dom->addIndependent("rEJZ_L", "q");
    dom->addIndependent("rEJZ_R", "q");
    dom->addIndependent("rULJX", "q");
    dom->addIndependent("rULJY", "q");
    dom->addIndependent("rULJZ", "q");
    dom->addIndependent("rLLJX", "q");
    dom->addIndependent("rLLJZ", "q");
    dom->addIndependent("rHJX_L", "q");
    dom->addIndependent("rHJY_L", "q");
    dom->addIndependent("rHJZ_L", "q");
    dom->addIndependent("rHJX_R", "q");
    dom->addIndependent("rHJY_R", "q");
    dom->addIndependent("rHJZ_R", "q");
    dom->addIndependent("rKJZ_L", "q");
    dom->addIndependent("rKJZ_R", "q");
    dom->addIndependent("rAJX_L", "q");
    dom->addIndependent("rAJY_L", "q");
    dom->addIndependent("rAJZ_L", "q");
    dom->addIndependent("rAJX_R", "q");
    dom->addIndependent("rAJY_R", "q");
    dom->addIndependent("rAJZ_R", "q");

    // Add dependent variables
    dom->addDependent("TRA_L", "rX");
    dom->addDependent("TRA_L", "rY");
    dom->addDependent("TRA_L", "rZ");
    dom->addDependent("TRA_R", "rX");
    dom->addDependent("TRA_R", "rY");
    dom->addDependent("TRA_R", "rZ");
    dom->addDependent("GLA", "rX");
    dom->addDependent("GLA", "rY");
    dom->addDependent("GLA", "rZ");
    dom->addDependent("ACR_L", "rX");
    dom->addDependent("ACR_L", "rY");
    dom->addDependent("ACR_L", "rZ");
    dom->addDependent("ACR_R", "rX");
    dom->addDependent("ACR_R", "rY");
    dom->addDependent("ACR_R", "rZ");
    dom->addDependent("LHC_L", "rX");
    dom->addDependent("LHC_L", "rY");
    dom->addDependent("LHC_L", "rZ");
    dom->addDependent("LHC_R", "rX");
    dom->addDependent("LHC_R", "rY");
    dom->addDependent("LHC_R", "rZ");
    dom->addDependent("WRI_L", "rX");
    dom->addDependent("WRI_L", "rY");
    dom->addDependent("WRI_L", "rZ");
    dom->addDependent("WRI_R", "rX");
    dom->addDependent("WRI_R", "rY");
    dom->addDependent("WRI_R", "rZ");
    dom->addDependent("SUP", "rX");
    dom->addDependent("SUP", "rY");
    dom->addDependent("SUP", "rZ");
    dom->addDependent("C7", "rX");
    dom->addDependent("C7", "rY");
    dom->addDependent("C7", "rZ");
    dom->addDependent("T8", "rX");
    dom->addDependent("T8", "rY");
    dom->addDependent("T8", "rZ");
    dom->addDependent("T12", "rX");
    dom->addDependent("T12", "rY");
    dom->addDependent("T12", "rZ");
    dom->addDependent("ASIS_L", "rX");
    dom->addDependent("ASIS_L", "rY");
    dom->addDependent("ASIS_L", "rZ");
    dom->addDependent("ASIS_R", "rX");
    dom->addDependent("ASIS_R", "rY");
    dom->addDependent("ASIS_R", "rZ");
    dom->addDependent("PSIS_L", "rX");
    dom->addDependent("PSIS_L", "rY");
    dom->addDependent("PSIS_L", "rZ");
    dom->addDependent("PSIS_R", "rX");
    dom->addDependent("PSIS_R", "rY");
    dom->addDependent("PSIS_R", "rZ");
    dom->addDependent("PS", "rX");
    dom->addDependent("PS", "rY");
    dom->addDependent("PS", "rZ");
    dom->addDependent("GTR_L", "rX");
    dom->addDependent("GTR_L", "rY");
    dom->addDependent("GTR_L", "rZ");
    dom->addDependent("GTR_R", "rX");
    dom->addDependent("GTR_R", "rY");
    dom->addDependent("GTR_R", "rZ");
    dom->addDependent("LFC_L", "rX");
    dom->addDependent("LFC_L", "rY");
    dom->addDependent("LFC_L", "rZ");
    dom->addDependent("LFC_R", "rX");
    dom->addDependent("LFC_R", "rY");
    dom->addDependent("LFC_R", "rZ");
    dom->addDependent("MFC_L", "rX");
    dom->addDependent("MFC_L", "rY");
    dom->addDependent("MFC_L", "rZ");
    dom->addDependent("MFC_R", "rX");
    dom->addDependent("MFC_R", "rY");
    dom->addDependent("MFC_R", "rZ");
    dom->addDependent("LM_L", "rX");
    dom->addDependent("LM_L", "rY");
    dom->addDependent("LM_L", "rZ");
    dom->addDependent("LM_R", "rX");
    dom->addDependent("LM_R", "rY");
    dom->addDependent("LM_R", "rZ");
    dom->addDependent("MM_L", "rX");
    dom->addDependent("MM_L", "rY");
    dom->addDependent("MM_L", "rZ");
    dom->addDependent("MM_R", "rX");
    dom->addDependent("MM_R", "rY");
    dom->addDependent("MM_R", "rZ");
    dom->addDependent("CAL_L", "rX");
    dom->addDependent("CAL_L", "rY");
    dom->addDependent("CAL_L", "rZ");
    dom->addDependent("CAL_R", "rX");
    dom->addDependent("CAL_R", "rY");
    dom->addDependent("CAL_R", "rZ");
    dom->addDependent("MT2_L", "rX");
    dom->addDependent("MT2_L", "rY");
    dom->addDependent("MT2_L", "rZ");
    dom->addDependent("MT2_R", "rX");
    dom->addDependent("MT2_R", "rY");
    dom->addDependent("MT2_R", "rZ");
    dom->addDependent("MT5_L", "rX");
    dom->addDependent("MT5_L", "rY");
    dom->addDependent("MT5_L", "rZ");
    dom->addDependent("MT5_R", "rX");
    dom->addDependent("MT5_R", "rY");
    dom->addDependent("MT5_R", "rZ");
    dom->addDependent("rLNJX", "rX");
    dom->addDependent("rLNJX", "rY");
    dom->addDependent("rLNJX", "rZ");
    dom->addDependent("rSJX_L", "rX");
    dom->addDependent("rSJX_L", "rY");
    dom->addDependent("rSJX_L", "rZ");
    dom->addDependent("rSJX_R", "rX");
    dom->addDependent("rSJX_R", "rY");
    dom->addDependent("rSJX_R", "rZ");
    dom->addDependent("rEJZ_L", "rX");
    dom->addDependent("rEJZ_L", "rY");
    dom->addDependent("rEJZ_L", "rZ");
    dom->addDependent("rEJZ_R", "rX");
    dom->addDependent("rEJZ_R", "rY");
    dom->addDependent("rEJZ_R", "rZ");
    dom->addDependent("rULJX", "rX");
    dom->addDependent("rULJX", "rY");
    dom->addDependent("rULJX", "rZ");
    dom->addDependent("rLLJX", "rX");
    dom->addDependent("rLLJX", "rY");
    dom->addDependent("rLLJX", "rZ");
    dom->addDependent("rHJX_L", "rX");
    dom->addDependent("rHJX_L", "rY");
    dom->addDependent("rHJX_L", "rZ");
    dom->addDependent("rHJX_R", "rX");
    dom->addDependent("rHJX_R", "rY");
    dom->addDependent("rHJX_R", "rZ");
    dom->addDependent("rKJZ_L", "rX");
    dom->addDependent("rKJZ_L", "rY");
    dom->addDependent("rKJZ_L", "rZ");
    dom->addDependent("rKJZ_R", "rX");
    dom->addDependent("rKJZ_R", "rY");
    dom->addDependent("rKJZ_R", "rZ");
    dom->addDependent("rAJX_L", "rX");
    dom->addDependent("rAJX_L", "rY");
    dom->addDependent("rAJX_L", "rZ");
    dom->addDependent("rAJX_R", "rX");
    dom->addDependent("rAJX_R", "rY");
    dom->addDependent("rAJX_R", "rZ");

}

}
