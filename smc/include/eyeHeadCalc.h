/*
 * eyeHeadCalc.h
 *
 *  Created on: 23 Aug 2011
 *      Author: icub
 */


#ifndef EYE_HEAD_CALC
	#define EYE_HEAD_CALC

#include <stdlib.h>



//These formulae and parameters are based on those from Wang & Jin 2002
//Horizontal
const int maxEyeAmpH = 29;	//BN or beta
const int maxGazeH = 79;		//ON or D
const int headContPosH = 19;	//CN = AL = OL or alpha

//int initialEyePosH;	//contralateral to gaze shift is +ve (p)
//int gazeH;	//desired gaze

double eyeSlopeH(double gazeH, double initialEyePosH);	//Ke(p)


//Note: Ke(p) + Kh(p) = 1
double headSlopeH(double gazeH, double initialEyePosH);	//Kh(p)


double eyeAmplitudeH(double gazeH, double initialEyePosH);	//E(G,p)


//Note, G = E(p) + H(p)
double headContributionH(double gazeH, double initialEyePosH);	//H(G,p)




double eyePercentH(double gazeH, double initialEyePosH);

double headPercentH(double gazeH, double initialEyePosH);


//Vertical
//NOTE: In the iCub, the vertical eye and head movement is not symmetric.
// For these formulae to work, the 'central' position should be shifted down to:
// Joint 0 (-35->20): -7, Joint 3 (-35->18): -8

//EyeAmplitude and headContribution take real initial eye positions and
// compensate for the offset, so the slope calculations use the offset values.
// As these are returned as percentages, there should be no need to convert back.


inline double realToCompHead(double realVhead);

inline double compToRealHead(double compVhead);

inline double realToCompEye(double realVeye);

inline double compToRealEye(double compVeye);



const int maxEyeAmpV = 26;	//beta
const int maxGazeV = 53;		//D
const int headContPosV = 16;	//alpha

double eyeSlopeV(double gazeV, double initialEyeCompPosV);	//Ke(p)


//Note: Ke(p) + Kh(p) = 1
double headSlopeV(double gazeV, double initialEyeCompPosV);	//Kh(p)


double eyeAmplitudeV(double gazeV, double initialEyePosV);	//E(G,p)


//Note, G = E(p) + H(p)
double headContributionV(double gazeV, double initialEyePosV);	//H(G,p)




double eyePercentV(double gazeV, double initialEyePosV);


double headPercentV(double gazeV, double initialEyePosV);



bool calcOverflow(double xDist, double yDist, double currentEyeX, double currentEyeY,
		double* xOverflow, double* yOverflow);




#endif
