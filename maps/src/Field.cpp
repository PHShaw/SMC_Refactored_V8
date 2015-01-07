/*
 * FILE Field.cpp
 * CLASS IMPLEMENTED: Field (see Field.h for documentation)
 *
 *  Created on: 28 Sep 2010
 *      Author: phs
 */

#include "Field.h"	// Provides the Field class definition

using namespace std;

Field::Field(size_t pIndex, float pX, float pY) {
	index = pIndex;
	x = (float)((int)(pX*1000))/1000;
	y = (float)((int)(pY*1000))/1000;
	calcdLinks = UNLEARNT;
	usageCount = 0;
	linked = false;
}
Field::Field() {	//Initialise a Null field
	index = 0;
	x = 0;
	y = 0;
	calcdLinks = UNLEARNT;
	usageCount = 0;
	linked = false;
}

Field::~Field() {
}

//********************
// NONMEMBER FUNCTIONS
//********************

ostream& operator <<(ostream& outs, const Field& source)
{
	outs << "coordinates: [" << source.getXcoord( ) << "," << source.getYcoord( ) << "]";
	if(source.isLearnt())
		outs << " Learnt and used " << source.getUsage() << " times";
	else
		outs << " Unlearnt";
	return outs;
}

bool operator ==(const Field& f1, const Field& f2){
	return (f1.getXcoord() == f2.getXcoord() && f1.getYcoord() == f2.getYcoord());
}
bool operator !=(const Field& f1, const Field& f2){
	return (! (f1==f2));

}
