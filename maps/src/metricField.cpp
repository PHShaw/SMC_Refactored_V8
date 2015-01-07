/*
 * metricField.cpp
 *
 *  Created on: 5 Oct 2010
 *      Author: phs
 */

#include "metricField.h"
using namespace std;

metricField::metricField():
	Field(){
	width = 0;
	height = 0;
}


metricField::metricField(size_t pIndex, float pX, float pY, float pWidth, float pHeight)
	:Field(pIndex, pX, pY){
	width = pWidth;
	height = pHeight;
}

metricField::~metricField() {
	// TODO Auto-generated destructor stub
}




ostream& operator <<(ostream& outs, const metricField& source)
{
	outs << "Top left corner: [" << source.getXcoord( ) << "," << source.getYcoord( ) << "] width: "
			<< source.getWidth( ) << " height: " << source.getHeight();
	return outs;
}


bool operator ==(const metricField& f1, const metricField& f2){
	return (f1.getXcoord() == f2.getXcoord() && f1.getYcoord() == f2.getYcoord()
			&& f1.getWidth() == f2.getWidth() && f1.getHeight() == f2.getHeight());
}
bool operator !=(const metricField& f1, const metricField& f2){
	return (! (f1==f2));
}
