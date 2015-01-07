/*
 * CalcHandDown.h
 *
 *  Created on: 16 May 2012
 *      Author: icub
 */

#ifndef CALC_HAND_DOWN_PHS
#define CALC_HAND_DOWN_PHS

#include "kinematicCalcs.h"


/**
 * Forward kinematic transform from torso to right hand for the iCub robot
 * Editied by James Law 16/05/2012 from WaistRightArmFwdKin.m
 * by Maggiali Marco, Stefano Saliceti, Genova Mar 2011
 *
 * Input:
 *      wtheta = [wtheta0, wtheta1, wtheta2]
 *      wtheta0: torso_pitch
 *      wtheta1: torso_roll
 *      wtheta2: torso_yaw
 *
 *      atheta = [atheta0, atheta1, atheta2, atheta3, atheta4, atheta5, atheta6]
 *      atheta0: shoulder_pitch
 *      atheta1: shoulder_roll
 *      atheta2: shoulder_yaw
 *      atheta3: elbow
 *      atheta4: wrist_prosup
 *      atheta5: wrist_pitch
 *      atheta6: wrist_yaw
 *
 *
 *      Note: wrist rotation for right arm is incorrect
 *      	  wrist pitch for both arms is incorrect
 *         i.e. don't bother with this
 *
 */
void WaistArmFwdKin(double* wtheta,double* atheta, bool rightArm,
		Matrix_smc* G_78, Matrix_smc* G_89, Matrix_smc* G_910, Matrix_smc* Elb);

void calcHandPos(double* torsoMotorConfig, double* armMotorConfig, bool rightArm, double* x, double* y, double* z);

/**
 * Calculate absolute forearm twist and wrist pitch to align the iCub hand parallel to
 * the global x-y plane.  wristRoll and wristPitch are the required angle, in
 * radians, for joints 4 and 5 in the right arm
 */
void calcHandDown(double* torsoMotorConfig, double* armMotorConfig, bool rightArm, double* wristRoll, double* wristPitch);


#endif
