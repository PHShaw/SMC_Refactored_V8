/*
 * CalcGazeChange.h
 *
 *  Created on: 11 May 2012
 *      Author: icub
 */

#ifndef CALC_GAZE_CHANGE_PHS
#define CALC_GAZE_CHANGE_PHS

//#define DEBUG

#include "kinematicCalcs.h"

/**
 * Forward kinematic transform from torso to eye for the iCub robot
 * Editied by James Law 09/05/2012 from WaistHeadFwdKin_HeadV2.m
 * by Maggiali Marco, Stefano Saliceti, Genova Mar 2011
 *
 * Input:
 *        wtheta = [wtheta0, wtheta1, wtheta2]
 *        wtheta0: torso_pitch
 *        wtheta1: torso_roll
 *        wtheta2: torso_yaw
 *
 *        htheta = [htheta0, htheta1, htheta2, htheta3, htheta4, htheta5]
 *        htheta0: neck_pitch
 *        htheta1: neck_roll
 *        htheta2: neck_yaw
 *        htheta3: eyes_tilt
 *        htheta4: eyes_version
 *        htheta5: eyes_vergence
 */
void WaistHeadFwdKin(double* torsoMotorConfig, double* headMotorConfig,
		Matrix_smc* HeadTransform,
		Matrix_smc* LeftEyeTransform,
		Matrix_smc* RightEyeTransform);


void CalculateGazeChange(double* torsoMotorConfig1, double* headMotorConfig1,
						 double* torsoMotorConfig2, double* headMotorConfig2,
		double* tilt, double* version, double* vergence, double depth=279); //, Matrix* targetWorldRef)

/**
 * Returns the x, y, z coordinates of an object in the world, with respect to the base of the torso.
 * Coordinates are in millimetres.
 */
void CalculateTargetWorldRef(double* torsoMotorConf, double* headMotorConf, double* x, double* y, double* z);



#endif
