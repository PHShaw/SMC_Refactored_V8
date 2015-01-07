/*
 * kinematicCalcs.h
 *
 *  Created on: 16 May 2012
 *      Author: icub
 */

#ifndef PI
	#define PI 3.14159265
#endif


#ifndef KINEMATIC_CALCS_PHS
	#define KINEMATIC_CALCS_PHS


	#define degToRad(d) d*(PI/180)
	#define radToDeg(r) r*(180/PI)

	#include <cmath>

	#include "matrixManipulation.h"

	/**
	 * function for creating transformation matrices
	 */
	Matrix_smc evalDHMatrix(float a, float d, float alph, float thet);


#endif
