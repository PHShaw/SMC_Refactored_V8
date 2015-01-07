/*
 * FILE: polarField.h
 * CLASS: polarField
 * 		polarField is a derived class of the Field class.  All the Field public member functions
 * 		are inherited by a polarField.  In addition, a polarField has extra member functions
 * 		described below.
 * 		A polarField is a circular field in a map where the radius of the fields increases
 * 		the further from the centre of the map they are.  This is useful for retina maps, where
 * 		greater precision is desired at the centre, and lower detail at the edges.
 *
 * CONSTRUCTORS for the polarField class:
 * 	polarField()
 * 		Postcondition: The polarField has been initialised as a Null field.
 *
 * 	polarField(size_t pIndex, float pX, float pY, float pRadius)
 * 		Postcondition: The polarField has been initialised with the specified parameters.  The
 * 					   value returned from getIndex() is now pIndex, the value returned from
 * 					   getXcoord() is now pX, the value returned from getYcoord() is now pY and
 * 					   the value returned from getRadius() is now pRadius.  The value returned
 * 					   from isLearnt() is false;
 *
 * MODIFICATION MEMBER FUNCTIONS for the polarField class:
 * 	none at this time, other than those derived from Field.
 *
 * CONSTANT MEMBER FUNCTIONS for the polarField class:
 * 	float getRadius()const
 * 		Postcondition: The value returned is the radius that was assigned at initialisation.
 *
 * NONMEMBER FUNCTIONS for the polarField class:
 * 	std::ostream& operator << (std::ostream& outs, const polarField& source)
 * 		Postcondition: The X,Y coordinates and the radius have been written to outs. The return
 * 					   value is the ostream outs.
 *
 * 	bool operator == (const polarField& f1, const polarField& f2)
 * 		Postcondition: The value returned is true if the index, coordinates and radius of f1 and
 * 					   f2 are identical.
 *
 * 	bool operator != (const polarfield& f1, const polarField& f2)
 * 		Postcondition: The value returned is true if operator == would return false for the same fields
 *
 * VALUE SEMANTICS for the polarField class:
 *		Assignments and the copy constructor may be used with Field objects.
 *
 *  Created on: 30 Sep 2010
 *      Author: phs
 */

#ifndef POLARFIELD_H_
#define POLARFIELD_H_

#include "Field.h"
#include <iostream>		//provides ostream from namespace std

class PolarField : public Field {
public:
	PolarField(size_t pIndex, float pX, float pY, float pRadius);
	PolarField();
	virtual ~PolarField();


	float getRadius()const{return radius;} ;



private:
	float radius;
};

// Non-member functions
std::ostream& operator <<(std::ostream& outs, const PolarField& source);
bool operator ==(const PolarField& f1, const PolarField& f2);
bool operator !=(const PolarField& f1, const PolarField& f2);

#endif /* POLARFIELD_H_ */
