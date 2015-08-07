// ------------------------------------------------------
// Technische Universität Darmstadt
// Department of Computer Science
// Simulation, Systems Optimization and Robotics Group
// Janis Wojtusch (wojtusch@sim.tu-darmstadt.de), 2015
// Licensed under BSD 3-Clause License
// ------------------------------------------------------

#ifndef MODEL_H
#define MODEL_H

// Global prototypes
void createModel(char* subjectIdentifier, double* gravityValue, double* head, double* torso, double* pelvis, double* upperArm_L, double* upperArm_R, double* lowerArm_L, double* lowerArm_R, double* thigh_L, double* thigh_R, double* shank_L, double* shank_R, double* foot_L, double* foot_R);
void applyForwardKinematics(double* jointPositions);
void getForwardKinematicsPositions(double* elementPositionsX, double* elementPositionsY, double* elementPositionsZ);
void getForwardKinematicsJacobian(double* jacobianVector);
void getForwardKinematicsJacobianSize(int *rows, int *columns);
void getModelDOF(int* dof);
void getModelElements(int* elements);

#endif
