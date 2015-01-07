/*
 * CalcHandDown.h
 *
 *  Created on: 16 May 2012
 *      Author: icub
 */


#include "CalcHand.h"


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
		Matrix_smc* G_78, Matrix_smc* G_89, Matrix_smc* G_910, Matrix_smc* Elb)
{
	//The location and orientation of the zeroth frame of reference
	//x-up, y-forward, z-left (iCub convention)
	Matrix_smc G_sL0(4,4);
	G_sL0.set(0,1,-1);
	G_sL0.set(1,2,-1);
	G_sL0.set(2,0,1);
	G_sL0.set(3,3,1);



	double wtheta0 = wtheta[3];
	double wtheta1 = wtheta[2];
	double wtheta2 = wtheta[1];

	//Reference frames attached to links
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//%               Waist                       %
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	Matrix_smc G_01=evalDHMatrix(     32,       0,     PI/2,                wtheta0);
	Matrix_smc G_12=evalDHMatrix(      0,    -5.5,     PI/2,                wtheta1-PI/2);


	Matrix_smc G_23;
	if(rightArm)
	{
		//waist to right arm
		G_23=evalDHMatrix(-23.3647,  -143.3,     PI/2,                wtheta2 - 15*PI/180 - PI/2 );
	}
	else
	{
		//waist to left arm
		G_23=evalDHMatrix(23.3647,  -143.3,    -PI/2,                wtheta2 + 15*PI/180 + PI/2 );
	}


	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//%               Arm                         %
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	Matrix_smc G_34, G_45, G_56, G_67;
	if(rightArm)
	{
		//right arm
		G_34=evalDHMatrix(    0, -107.74,     PI/2,    atheta[0]-PI/2);
		G_45=evalDHMatrix(    0,       0,    -PI/2,    atheta[1]-PI/2);
		G_56=evalDHMatrix(  -15, -152.28,    -PI/2,    atheta[2]-PI/2-15*PI/180);
		G_67=evalDHMatrix(   15,       0,     PI/2,    atheta[3]);
		*G_78=evalDHMatrix(    0,  -137.3,     PI/2,    atheta[4]-PI/2);
		*G_89=evalDHMatrix(    0,       0,     PI/2,    atheta[5]+PI/2);
		*G_910=evalDHMatrix( 62.5,     16,        0,    atheta[6]+PI);
	}
	else
	{
		//left arm
		G_34=evalDHMatrix(   0,  107.74,    -PI/2, atheta[0]+PI/2);
		G_45=evalDHMatrix(   0,       0,     PI/2, atheta[1]-PI/2);
		G_56=evalDHMatrix(  15,  152.28,    -PI/2, atheta[2]+PI/2-15*PI/180);
		G_67=evalDHMatrix( -15,      0,     PI/2,  atheta[3]);
		*G_78=evalDHMatrix(   0,   137.3,     PI/2, atheta[4]-PI/2);
		*G_89=evalDHMatrix(   0,       0,     PI/2, atheta[5]+PI/2);
		*G_910=evalDHMatrix(62.5,    -16,        0, atheta[6]);
	}


	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	Matrix_smc T_Ro0 = G_sL0;
	T_Ro0.multiply(G_01);
	T_Ro0.multiply(G_12);
	T_Ro0.multiply(G_23);
	T_Ro0.multiply(G_34);
	T_Ro0.multiply(G_45);
	T_Ro0.multiply(G_56);
	T_Ro0.multiply(G_67);

	*Elb= T_Ro0;
}

void calcHandPos(double* torsoMotorConfig, double* armMotorConfig, bool rightArm, double* x, double* y, double* z)
{
	double wtheta[3];
	for(int i=0; i<3; i++)
	{
		wtheta[i]= degToRad(torsoMotorConfig[i]);
	}
	double atheta[16];
	for(int i=0; i<16; i++)
	{
		atheta[i] = degToRad(armMotorConfig[i]);
	}


	//get transformation matrices of forearm
	Matrix_smc G_78, G_89, G_910, Elb;
	WaistArmFwdKin(wtheta,atheta,rightArm,&G_78, &G_89, &G_910, &Elb);
	Matrix_smc T_H=Elb;
	T_H.multiply(G_78);
	T_H.multiply(G_89);
	T_H.multiply(G_910);

//	T_H.print();
	*x = T_H.get(0,3);
	*y = T_H.get(1,3);
	*z = T_H.get(2,3);
}

/**
 * Calculate absolute forearm twist and wrist pitch to align the iCub hand parallel to
 * the global x-y plane.  wristRoll and wristPitch are the required angle, in
 * radians, for joints 4 and 5 in the right arm
 */
void calcHandDown(double* torsoMotorConfig, double* armMotorConfig, bool rightArm, double* wristRoll, double* wristPitch)
{
	double wtheta[3];
	for(int i=0; i<3; i++)
	{
		wtheta[i]= degToRad(torsoMotorConfig[i]);
	}
	double atheta[16];
	for(int i=0; i<16; i++)
	{
		atheta[i] = degToRad(armMotorConfig[i]);
	}


	//get transformation matrices of forearm
	Matrix_smc G_78, G_89, G_910, Elb;
	WaistArmFwdKin(wtheta,atheta,rightArm,&G_78, &G_89, &G_910, &Elb);

	//%%%%%%%%%%%%%%%%%%%%
	//%forearm orientation
	//%%%%%%%%%%%%%%%%%%%%
	//calculate forearm twist to set wrist-tilt z-axis parallel to global x-y plane
	*wristRoll=atan2(Elb.get(2,1),Elb.get(2,0))+PI/2;
	if(*wristRoll>PI)
		*wristRoll-=(2*PI);

	//get new transformation matrices based on new forearm twist
	atheta[4] = *wristRoll;
	WaistArmFwdKin(wtheta,atheta,rightArm, &G_78, &G_89, &G_910, &Elb);

	//%%%%%%%%%%%%%%%%%%
	//%hand orientation
	//%%%%%%%%%%%%%%%%%%
	//get transformation at wrist pitch joint
	Matrix_smc W_P=Elb;
	W_P.multiply(G_78);

	//calculate wrist tilt to set x-axis of hand parallel to global x-y plane
	*wristPitch=atan2(-W_P.get(2,0),W_P.get(2,1))-PI/2;

	//get new transformation matrices based on wrist pitch
	atheta[5] = *wristPitch;

	//check if hand z-direction is aligned with global z axis
	WaistArmFwdKin(wtheta,atheta,rightArm,&G_78, &G_89, &G_910, &Elb);
	Matrix_smc T_H=Elb;
	T_H.multiply(G_78);
	T_H.multiply(G_89);
	T_H.multiply(G_910);

	//if not, use the alternate solution
	if (T_H.get(2,2)<0)
		*wristPitch=atan2(W_P.get(2,0),-W_P.get(2,1))-PI/2;


	*wristRoll = radToDeg(*wristRoll);
	*wristPitch = radToDeg(*wristPitch);
}
