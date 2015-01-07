/*
 * kinematicCalcs.h
 *
 *  Created on: 16 May 2012
 *      Author: icub
 */


#include "kinematicCalcs.h"
	/**
	 * function for creating transformation matrices
	 */
	Matrix_smc evalDHMatrix(float a, float d, float alph, float thet)
	{
		Matrix_smc G(4,4);
		G.set(0,0,cos(thet));
		G.set(0,1,-sin(thet)*cos(alph));
		G.set(0,2,sin(thet)*sin(alph));
		G.set(0,3,cos(thet)*a);

		G.set(1,0,sin(thet));
		G.set(1,1,cos(thet)*cos(alph));
		G.set(1,2,-cos(thet)*sin(alph));
		G.set(1,3,sin(thet)*a);

		G.set(2,0,0);
		G.set(2,1,sin(alph));
		G.set(2,2,cos(alph));
		G.set(2,3,d);

		G.set(3,0,0);
		G.set(3,1,0);
		G.set(3,2,0);
		G.set(3,3,1);

		return G;
	}

