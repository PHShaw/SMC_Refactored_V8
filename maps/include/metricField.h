/*
 * FILE: metricField.h
 * CLASS: metricField
 * 		metricField is a derived class of the Field class.  All the Field public member functions
 * 		are inherited by a metricField.  In addition, a metricField has extra member functions
 * 		described below.
 * 		A metricField is a rectangular field in a regular grid map.  Currently the X,Y coordinates
 * 	    represent the top left corner of the rectangle.
 *
 * CONSTRUCTORS for the metricField class:
 * 	metricField()
 * 		Postcondition: The metricField has been initialised as a Null field.
 *
 * 	metricField(size_t pIndex, float pX, float pY, float pWidth, float pHeight)
 * 		Postcondition: The metricField has been initialised with the specified parameters.  The
 * 					   value returned from getIndex() is now pIndex, the value returned from
 * 					   getXcoord() is now pX, the value returned from getYcoord() is now pY,
 * 					   the value returned from getHeight() is now pHeight and the value returned
 * 					   from getWidth() is now pWidth.  The value returned from isLearnt() is false;
 *
 * MODIFICATION MEMBER FUNCTIONS for the metricField class:
 * 	none at this time, other than those derived from Field.
 *
 * CONSTANT MEMBER FUNCTIONS for the metricField class:
 * 	float getHeight()const
 * 		Postcondition: The value returned is the height that was assigned at initialisation.
 *
 * 	float getWidth()const
 * 		Postcondition: The value returned is the Width that was assigned at initialisation.
 *
 * NONMEMBER FUNCTIONS for the metricField class:
 * 	std::ostream& operator << (std::ostream& outs, const metricField& source)
 * 		Postcondition: The X,Y coordinates and the heihgt, width have been written to outs. The return
 * 					   value is the ostream outs.
 *
 * 	bool operator == (const metricField& f1, const metricField& f2)
 * 		Postcondition: The value returned is true if the index, coordinates and height, width of f1 and
 * 					   f2 are identical.
 *
 * 	bool operator != (const metricField& f1, const metricField& f2)
 * 		Postcondition: The value returned is true if operator == would return false for the same fields
 *
 * VALUE SEMANTICS for the metricField class:
 *		Assignments and the copy constructor may be used with Field objects.
 *
 *  Created on: 5 Oct 2010
 *      Author: phs
 *
 *      In a metric field, the field is a rectangle with width and height,
 *      and the X,Y coordinates represent the top left corner.
 */

#ifndef METRICFIELD_H_
#define METRICFIELD_H_

#include "Field.h"
#include <iostream>		//provides ostream

class metricField: public Field {
public:
	metricField(size_t pIndex, float pX, float pY, float width, float height);
	metricField();
	virtual ~metricField();

	float getWidth()const{return width;}
	float getHeight()const{return height;}


private:
	float width, height;
};

// Non-member functions
std::ostream& operator <<(std::ostream& outs, const metricField& source);
bool operator ==(const metricField& f1, const metricField& f2);
bool operator !=(const metricField& f1, const metricField& f2);


#endif /* METRICFIELD_H_ */
