// ------------------------------------------------------
// Technische Universität Darmstadt
// Department of Computer Science
// Simulation, Systems Optimization and Robotics Group
// Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
// Licensed under BSD 3-Clause License
// ------------------------------------------------------

#ifndef SEGMENT_H
#define SEGMENT_H

// Defines
#define USE_ADOLC
#define SEGMENT_POSITIONX 0
#define SEGMENT_POSITIONY 1
#define SEGMENT_POSITIONZ 2
#define SEGMENT_LENGTHX 0
#define SEGMENT_LENGTHY 1
#define SEGMENT_LENGTHZ 2
#define REFERNCE_LENGTHX 0
#define REFERNCE_LENGTHY 1
#define REFERNCE_LENGTHZ 2
#define SEGMENT_MASS 3
#define SEGMENT_COMX 4
#define SEGMENT_COMY 5
#define SEGMENT_COMZ 6
#define SEGMENT_MOIXX 7
#define SEGMENT_MOIYY 8
#define SEGMENT_MOIZZ 9
#define SEGMENT_POIXY 10
#define SEGMENT_POIXZ 11
#define SEGMENT_POIYZ 12
#define SEGMENT_OFFSET0 13
#define SEGMENT_OFFSET1 14
#define SEGMENT_OFFSET2 15
#define SEGMENT_HEAD_GLA 0
#define SEGMENT_HEAD_TRA_L 1
#define SEGMENT_HEAD_TRA_R 2
#define SEGMENT_TORSO_ACR_L 0
#define SEGMENT_TORSO_ACR_R 1
#define SEGMENT_TORSO_SUP 2
#define SEGMENT_TORSO_C7 3
#define SEGMENT_TORSO_T8 4
#define SEGMENT_TORSO_T12 5
#define SEGMENT_TORSO_SJ_L 6
#define SEGMENT_TORSO_SJ_R 7
#define SEGMENT_TORSO_LLJ 8
#define SEGMENT_PELVIS_ASIS_L 0
#define SEGMENT_PELVIS_ASIS_R 1
#define SEGMENT_PELVIS_PSIS_L 2
#define SEGMENT_PELVIS_PSIS_R 3
#define SEGMENT_PELVIS_PS 4
#define SEGMENT_PELVIS_HJ_L 5
#define SEGMENT_PELVIS_HJ_R 6
#define SEGMENT_UPPER_ARM_LHC 0
#define SEGMENT_UPPER_ARM_EJ 1
#define SEGMENT_LOWER_ARM_LHC 0
#define SEGMENT_LOWER_ARM_WRI 1
#define SEGMENT_THIGH_GTR 0
#define SEGMENT_THIGH_LFC 1
#define SEGMENT_THIGH_MFC 2
#define SEGMENT_THIGH_KJ 3
#define SEGMENT_SHANK_LM 0
#define SEGMENT_SHANK_MM 1
#define SEGMENT_SHANK_AJ 2
#define SEGMENT_FOOT_CAL 0
#define SEGMENT_FOOT_MT2 1
#define SEGMENT_FOOT_MT5 2

// Includes
#include <stdlib.h>
#include <iostream>
#include <string>
#include <mbslib/mbslib.h>

// Global class
class Segment {

public:
    Segment(std::string name, int relativePositionNumber);
    virtual ~Segment(void);
    std::string name;
    mbslib::TScalar lengthX;
    mbslib::TScalar lengthY;
    mbslib::TScalar lengthZ;
    mbslib::TScalar mass;
    mbslib::TScalar comX;
    mbslib::TScalar comY;
    mbslib::TScalar comZ;
    mbslib::TScalar moiXX;
    mbslib::TScalar moiYY;
    mbslib::TScalar moiZZ;
    mbslib::TScalar poiXY;
    mbslib::TScalar poiXZ;
    mbslib::TScalar poiYZ;
    int relativePositionNumber;
    std::vector< std::vector<mbslib::TScalar> > relativePositionArray;

};

#endif
