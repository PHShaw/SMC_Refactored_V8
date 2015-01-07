


#include "eyeHeadCalc.h"


//These formulae and parameters are based on those from Wang & Jin 2002
//Horizontal


//int initialEyePosH;	//contralateral to gaze shift is +ve (p)
//int gazeH;	//desired gaze

double eyeSlopeH(double gazeH, double initialEyePosH)	//Ke(p)
{
	if(gazeH <= (headContPosH + initialEyePosH))
	{
		return 1.0;
	}
	else
	{
		return (maxEyeAmpH - headContPosH) / (maxGazeH - headContPosH - initialEyePosH);
	}
}

//Note: Ke(p) + Kh(p) = 1
double headSlopeH(double gazeH, double initialEyePosH)	//Kh(p)
{
	if(gazeH <= (headContPosH + initialEyePosH))
	{
		return 0;
	}
	else
	{
		return (maxGazeH - initialEyePosH - maxEyeAmpH) / (maxGazeH - headContPosH - initialEyePosH);
	}
}

double eyeAmplitudeH(double gazeH, double initialEyePosH)	//E(G,p)
{
	double pos;
	if(gazeH <= (initialEyePosH + headContPosH))
	{
		pos = gazeH;
	}
	else
	{
		pos = headContPosH + initialEyePosH +
				eyeSlopeH(gazeH, initialEyePosH)*(gazeH - (headContPosH + initialEyePosH));
	}
	return pos;// - initialEyePosH; //Compensate for eye position
}

//Note, G = E(p) + H(p)
double headContributionH(double gazeH, double initialEyePosH)	//H(G,p)
{
	double pos;
	if(gazeH <= (initialEyePosH + headContPosH))
	{
		pos = 0;
	}
	else
	{
		pos = headSlopeH(gazeH, initialEyePosH)*(gazeH - (headContPosH + initialEyePosH));
	}
	return pos;
}



double eyePercentH(double gazeH, double initialEyePosH)
{
	return eyeAmplitudeH(gazeH, initialEyePosH) / gazeH * 100;
}

double headPercentH(double gazeH, double initialEyePosH)
{
	return headContributionH(gazeH, initialEyePosH) / gazeH * 100;
}


//Vertical
//NOTE: In the iCub, the vertical eye and head movement is not symmetric.
// For these formulae to work, the 'central' position should be shifted down to:
// Joint 0 (-35->20): -7, Joint 3 (-35->18): -8

//EyeAmplitude and headContribution take real initial eye positions and
// compensate for the offset, so the slope calculations use the offset values.
// As these are returned as percentages, there should be no need to convert back.


inline double realToCompHead(double realVhead)
{
	return realVhead - 7;
}

inline double compToRealHead(double compVhead)
{
	return compVhead + 7;
}

inline double realToCompEye(double realVeye)
{
	return realVeye - 8;
}

inline double compToRealEye(double compVeye)
{
	return compVeye + 8;
}


double eyeSlopeV(double gazeV, double initialEyeCompPosV)	//Ke(p)
{
	if(gazeV <= (headContPosV + initialEyeCompPosV))
	{
		return 1.0;
	}
	else
	{
		return (maxEyeAmpV - headContPosV) / (maxGazeV - headContPosV - initialEyeCompPosV);
	}
}

//Note: Ke(p) + Kh(p) = 1
double headSlopeV(double gazeV, double initialEyeCompPosV)	//Kh(p)
{
	if(gazeV <= (headContPosV + initialEyeCompPosV))
	{
		return 0;
	}
	else
	{
		return (maxGazeV - initialEyeCompPosV - maxEyeAmpV) / (maxGazeV - headContPosV - initialEyeCompPosV);
	}
}

double eyeAmplitudeV(double gazeV, double initialEyePosV)	//E(G,p)
{
	double initialEyeCompPosV = realToCompEye(initialEyePosV);
	double pos;
	if(gazeV <= (initialEyeCompPosV + headContPosV))
	{
		pos =  gazeV;
	}
	else
	{
		pos = headContPosV + initialEyeCompPosV +
				eyeSlopeV(gazeV, initialEyeCompPosV)*(gazeV - (headContPosV + initialEyeCompPosV));
	}
	return pos;// - initialEyeCompPosV;		// eye position
}

//Note, G = E(p) + H(p)
double headContributionV(double gazeV, double initialEyePosV)	//H(G,p)
{
	double initialEyeCompPosV = realToCompEye(initialEyePosV);
	double pos;
	if(gazeV <= (initialEyeCompPosV + headContPosV))
	{
		pos = 0;
	}
	else
	{
		pos = headSlopeV(gazeV, initialEyeCompPosV)*(gazeV - (headContPosV + initialEyeCompPosV));
	}
	return pos;// + initialEyeCompPosV;
}



double eyePercentV(double gazeV, double initialEyePosV)
{
	return eyeAmplitudeV(gazeV, initialEyePosV) / gazeV * 100;
}

double headPercentV(double gazeV, double initialEyePosV)
{
	return headContributionV(gazeV, initialEyePosV) / gazeV * 100;
}


bool calcOverflow(double xDist, double yDist, double currentEyeX, double currentEyeY,
		double* xOverflow, double* yOverflow)
{
	double eyeXmax = 30;
	double eyeXmin = -30;
	double eyeYmax = 18;
	double eyeYmin = -35;

//	cout << "calculating overflow for parameters:" << endl;
//	cout << xDist << ", " << yDist << ", " << currentEyeX << ", " << currentEyeY << endl;

	//Convert eye ranges to relative available movement
	double Xmax, Xmin, Ymax, Ymin;
	Xmax = eyeXmax-currentEyeX;
	Xmin = eyeXmin-currentEyeX;
	Ymax = eyeYmax-currentEyeY;
	Ymin = eyeYmin-currentEyeY;



	if(xDist>Xmax)
	{
		*xOverflow = xDist - Xmax;
	}
	else if(xDist<Xmin)
	{
		*xOverflow = xDist - Xmin;
	}

	if(yDist>Ymax)
	{
		*yOverflow = yDist - Ymax;
	}
	else if(yDist<Ymin)
	{
		*yOverflow = yDist - Ymin;
	}

	return abs(*xOverflow) + abs(*yOverflow);
}

