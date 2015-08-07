// ------------------------------------------------------
// Technische Universität Darmstadt
// Department of Computer Science
// Simulation, Systems Optimization and Robotics Group
// Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
// Licensed under BSD 3-Clause License
// ------------------------------------------------------

// Includes
#include "segment.h"

// Global functions
Segment::Segment(std::string name = "segment", int relativePositionNumber = 0) {

    // Set name
    this->name = name;

    // Initialize variables
    lengthX = 0.0;
    lengthY = 0.0;
    lengthZ = 0.0;
    mass = 0.0;
    comX = 0.0;
    comY = 0.0;
    comZ = 0.0;
    moiXX = 0.0;
    moiYY = 0.0;
    moiZZ = 0.0;
    poiXY = 0.0;
    poiXZ = 0.0;
    poiYZ = 0.0;
    this->relativePositionNumber = relativePositionNumber;
    relativePositionArray.assign(relativePositionNumber, std::vector<mbslib::TScalar>(3));

}

Segment::~Segment(void) {}

