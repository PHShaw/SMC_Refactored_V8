/*
 * CalcGazeChange.h
 *
 *  Created on: 11 May 2012
 *      Author: icub
 */



#include "CalcGazeChange.h"

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
		Matrix_smc* RightEyeTransform)
{
	//MATLAB labels arrays from 1 instead of 0?
	double wtheta[3];
	for(int i=0; i<3; i++)
	{
		wtheta[i]= degToRad(torsoMotorConfig[i]);
//		wtheta[i]= torsoMotorConfi/g[i];
	}
	double htheta[6];
#ifdef DEBUG
	printf("htheta: ");
#endif
	for(int i=0; i<6; i++)
	{
		htheta[i] = degToRad(headMotorConfig[i]);
#ifdef DEBUG
		printf("%.2f ");
#endif
//		htheta[i] = headMotorConfig[i];
	}
#ifdef DEBUG
	printf("\n");
#endif


	/**
	 * Joint displacement matrices:
	 */
	//The location and orientation of the zeroth frame of reference
	//x-up, y-forward, z-left (iCub convention)
	Matrix_smc G_sL0(4,4);
	G_sL0.set(0,1,-1);
	G_sL0.set(1,2,-1);
	G_sL0.set(2,0,1);
	G_sL0.set(3,3,1);

#ifdef DEBUG
	printf("\nG_sL0:\n");
	G_sL0.print();
#endif


	//Reference frames attached to links
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//%               Waist                       *
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//note the joint order is yaw, roll, pitch, hence the reversed order of inputs
	Matrix_smc G_01(4,4);
	Matrix_smc G_12(4,4);
	Matrix_smc G_23(4,4);

	G_01 = evalDHMatrix(32,     0,  PI/2, wtheta[2]);
	G_12 = evalDHMatrix( 0,  -5.5,  PI/2, wtheta[1]-PI/2);
	G_23 = evalDHMatrix( 0,-223.3, -PI/2, wtheta[0]-PI/2);

#ifdef DEBUG
	printf("\nG_01:\n");
	G_01.print();
	printf("\nG_12:\n");
	G_12.print();
	printf("\nG_23:\n");
	G_23.print();
#endif

	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	//%               Head                        *
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//% REVISED htheta4 & htheta5 - SHOULD NOW MATCH ICUB MANUAL
//	double htheta4 =  (-htheta[5] + htheta[4])/2;		//conversion from verg and vers to right eye pan
//	double htheta5 =  ( htheta[5] + htheta[4])/2;		//conversion from verg and vers to left eye pan
	double htheta4 =  htheta[4] - htheta[5]/2;		//conversion from verg and vers to right eye pan
	double htheta5 =  htheta[4] + htheta[5]/2;		//conversion from verg and vers to left eye pan
#ifdef DEBUG
	printf("htheta4: %.2f\n", htheta4);
	printf("htheta5: %.2f\n", htheta5);
#endif

//htheta4 =  htheta(5) - htheta(6)/2; %(- htheta(6) + htheta(5))/2;           %conversion from verg and vers to right eye pan
//htheta5 =  htheta(5) + htheta(6)/2; %(htheta(6) + htheta(5))/2;             %conversion from verg and vers to left eye pan
	//head
	Matrix_smc G_34(4,4);
	Matrix_smc G_45(4,4);
	Matrix_smc G_56(4,4);

	G_34 = evalDHMatrix(  9.5,     0,  PI/2, htheta[0] + PI/2);
	G_45 = evalDHMatrix(    0,     0, -PI/2, htheta[1] - PI/2);
	G_56 = evalDHMatrix(-50.9, 82.05, -PI/2, htheta[2] + PI/2);

#ifdef DEBUG
	printf("\nG_34:\n");
	G_34.print();
	printf("\nG_45:\n");
	G_45.print();
	printf("\nG_56:\n");
	G_56.print();
#endif


	//right eye
	Matrix_smc G_67(4,4);
	Matrix_smc G_78(4,4);

	G_67 = evalDHMatrix( 0, 34, -PI/2, htheta[3]);
	G_78 = evalDHMatrix( 0,  0,  PI/2, htheta4 - PI/2);

#ifdef DEBUG
	printf("\nG_67:\n");
	G_67.print();
	printf("\nG_78:\n");
	G_78.print();
#endif


	//left eye
	Matrix_smc Gp_67(4,4);
	Matrix_smc Gp_78(4,4);

	Gp_67 = evalDHMatrix( 0,-34, -PI/2, htheta[3]);
	Gp_78 = evalDHMatrix( 0,  0,  PI/2, htheta5-PI/2);


	//Reference frames attached to links
	Matrix_smc T_H = G_sL0;		//Head relative to origin (zeroth frame) (x-back, y-down, z-right)
	T_H.multiply(G_01);
	T_H.multiply(G_12);
	T_H.multiply(G_23);
	T_H.multiply(G_34);
	T_H.multiply(G_45);
	T_H.multiply(G_56);


	Matrix_smc T_RE = G_67;		//Right Eye relative to head
	T_RE.multiply(G_78);


	Matrix_smc T_LE = Gp_67;		//Left Eye relative to head
	T_LE.multiply(Gp_78);


	*HeadTransform = T_H;
	*LeftEyeTransform = T_LE;
	*RightEyeTransform = T_RE;

}


void CalculateGazeChange(double* torsoMotorConfig1, double* headMotorConfig1,
						 double* torsoMotorConfig2, double* headMotorConfig2,
		double* tilt, double* version, double* vergence, double depth) //, Matrix* targetWorldRef)
{
	//get initial head and eye positions and orientations
	Matrix_smc T_H1, T_LE1, T_RE1;
	WaistHeadFwdKin(torsoMotorConfig1, headMotorConfig1, &T_H1, &T_LE1, &T_RE1);		//NOTE: LE and RE changed to be wrt head!
	//get final head and eye positions and orientations
	Matrix_smc T_H2, T_LE2, T_RE2;
	WaistHeadFwdKin(torsoMotorConfig2, headMotorConfig2, &T_H2, &T_LE2, &T_RE2);


	//Assume a target is fixated in config1 at 279mm from right eye
	Matrix_smc T_01=evalDHMatrix(0, depth, 0, 0);		//(a, d, alph, thet)
//targetWorldRef=T_H*T_RE*T_Targ;
	Matrix_smc TWR = T_H1;
	TWR.multiply(T_RE1);
	TWR.multiply(T_01);
//	*targetWorldRef=TWR;		//position of target in config1 in world coordinates


	//find final right eye position in world coordinates
	Matrix_smc temp(1,4);
	temp.set(0,3,1);
	temp.transpose();

	Matrix_smc rightEyePos = T_H2;
	rightEyePos.multiply(T_RE2);
	rightEyePos.multiply(temp);


    //target position relative to final right eye pos
	Matrix_smc Q = T_H2;
	Q.multiply(T_RE2);
	Q.transpose();
	TWR.subtract(rightEyePos);


	Matrix_smc TWR_Scalar(4,1);
	for(int i=0; i<4; i++)
	{
		TWR_Scalar.set(i,0,TWR.get(i,3));
	}

	double pan;
	Q.multiply(TWR_Scalar);


	//pan and tilt required to fixate target with right eye from final position
	*tilt = atan2(Q.get(1,0),Q.get(2,0));
	pan  = atan2(Q.get(0,0),sqrt(pow(Q.get(1,0),2)+pow(Q.get(2,0),2)));


//	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//	//additional code to fix problems from non-centred eyes
//	double cf2Vers=headMotorConfig2[4];
//	double cf2Tilt=headMotorConfig2[5];
//
//	headMotorConfig2[4]=0;
//	headMotorConfig2[5]=0;
//	//get final head and eye positions and orientations
//	WaistHeadFwdKin(torsoMotorConfig2, headMotorConfig2, &T_H2, &T_LE2, &T_RE2);
//	//find final right eye position in world coordinates
//	rightEyePos = T_RE2;
//	rightEyePos.multiply(temp);
//	//target position relative to final right eye pos
//	Q = T_RE2;
//	Q.transpose();
//
//	TWR.subtract(rightEyePos);
//	Matrix TWR_Scalar2(4,1);
//	for(int i=0; i<4; i++)
//	{
//		TWR_Scalar2.set(i,0,TWR.get(i,3));
//	}
//
//	Q.multiply(TWR_Scalar2);
//	//actual pan and tilt required to fixate target with right eye from final position
//	*tilt = atan2(Q.get(1,0),Q.get(2,0));
//	pan  = atan2(Q.get(0,0),sqrt(pow(Q.get(1,0),2)+pow(Q.get(2,0),2)));
	//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



	//gaze shift required to fixate target with right eye
	*tilt = radToDeg(*tilt * -1);
	*version= radToDeg(2*pan)/2;
	*vergence=0;
}



void CalculateTargetWorldRef(double* torsoMotorConfig, double* headMotorConfig, double* x, double* y, double* z)
{
//	  fixatedTorsoPos=[rand*70-35; 0; rand*35]*pi/180 % for safe iCub values (from horizontal down)
//
//    %create target at random distance in front of eyes
//    fixatedGaze(:,1)=[0; 0; rand*9.5+6]*pi/180 %for ~hand reachable distances on iCub (25-65cm)
//    [T_H, T_LE, T_RE] = WaistHeadFwdKin(fixatedTorsoPos',[0 0 0 fixatedGaze(:,1)'])
//    T_Targ=evalDHMatrix(0,34/sin(fixatedGaze(3,1)/2), 0, 0)
//    targetWorldRef=T_H*T_RE*T_Targ

	//get head and eye positions and orientations
	Matrix_smc T_H, T_LE, T_RE;
	WaistHeadFwdKin(torsoMotorConfig, headMotorConfig, &T_H, &T_LE, &T_RE);
#ifdef DEBUG
	printf( "T_H:\n");
	T_H.print();
	printf( "\nT_RE:\n");
	T_RE.print();
#endif
	Matrix_smc T_Targ=evalDHMatrix(0, (34/sin((headMotorConfig[5]*PI/180)/2)), 0, 0);
#ifdef DEBUG
	printf( "\nT_Targ:\n" );
	T_Targ.print();
#endif

	Matrix_smc TargetWorldRef = T_H;
	TargetWorldRef.multiply(T_RE);
	TargetWorldRef.multiply(T_Targ);

#ifdef DEBUG
	printf( "\nTargetWorldRef:\n");
	TargetWorldRef.print();
#endif

	*x = TargetWorldRef.get(0,3);
	*y = TargetWorldRef.get(1,3);
	*z = TargetWorldRef.get(2,3);

}
