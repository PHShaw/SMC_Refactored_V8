/*
 * polarField.cpp
 *
 *  Created on: 30 Sep 2010
 *      Author: phs
 */

#include "PolarField.h"

using namespace std;

PolarField::PolarField(size_t pIndex, float pX, float pY, float pRadius)
	: Field(pIndex, pX, pY) {
	radius = pRadius;
}
PolarField::PolarField()
	: Field(){
	radius = 0;
}

PolarField::~PolarField() {
	// TODO Auto-generated destructor stub
}



ostream& operator <<(ostream& outs, const PolarField& source)
{
	outs << "centre: [" << source.getXcoord( ) << "," << source.getYcoord( ) << "] radius: "
			<< source.getRadius( );
	if(source.isLearnt())
		outs << " Learnt and used " << source.getUsage() << " times";
	else
		outs << " Unlearnt";
	return outs;
}


bool operator ==(const PolarField& f1, const PolarField& f2){
	return (f1.getXcoord() == f2.getXcoord() && f1.getYcoord() == f2.getYcoord() && f1.getRadius()==f2.getRadius());
}
bool operator !=(const PolarField& f1, const PolarField& f2){
	return (! (f1==f2));

}
