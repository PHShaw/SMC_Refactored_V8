/*
 * FILE: Field.h
 * CLASS PROVIDED: Field
 * 		A field covers an area on a map that gives a coarser granularity for learning mappings
 * 		instead of individual points mapping across.
 *
 * TYPEDEFS and MEMBER CONSTANTS for the Field class:
// * 	static const int CARTESIAN
// * 		Field :: CARTESIAN is a constant used to indicate the field is of type cartesian.
// *
// * 	static const int POLAR
// * 		Field :: POLAR is a constant used to indicate the field is of type polar.
 *
 * //Note the following constants are not really necessary, so may be removed from future versions.
 * 	static const int UNLEARNT
 * 		Field :: UNLEARNT is a constant used to indicate that a field has not yet been learnt.
 *
 * 	static const int LEARNT_DIRECT
 * 		Field :: LEARNT_DIRECT is a constant used to indicate that a field has been learnt
 * 				 from a single movement taking the focus straight to the fovea.
 *
 * 	static const int LEARNT_CALC
 * 		Field :: LEARNT_CALC is a constant used to indicate that a field has been learnt however
 * 				 there were multiple steps between this field and the fovea so the direct movement
 * 				 from this field to fovea was calculated.  NOTE: a class variable calcdLinks is
 * 				 currently being used to indicate whether a field has been learnt, and if so how
 * 				 many steps there were between it and the target being centred in the fovea.
 *
 * CONSTRUCTORS for the Field class:
 * 	Field()
 * 		Postcondition: The Field has been initialised with all values set to 0 (A null field).
 *
 * 	Field(size_t pIndex, float pX, float pY)
 * 		Precondition:
 * 			pIndex should be unique for this map
 * 			pX and pY are the X and Y coordinates marking the location of this field in 2D space
 * 				NOTE: In a polar field, these mark the centre of the circle.
 * 					  In a metric/Cartesian field, they currently represent the top left corner.
 * 		Postcondition: The field has been initialised with the parameters given.  The field is
 * 					   not yet learnt. The value returned from getIndex() is now pIndex, the
 * 					   value returned from getXcoord() is now pX, and the value returned from
 * 					   getYcoord() is now pY
 *
 * MODIFICATION MEMBER FUNCTIONS for the Field class:
 * 	setCalcdLinks(unsigned int noSteps)
 * 		Precondition: noSteps represents the number of movements between seeing a target in this field
 * 					  and having it visible in the fovea.  Note, if noSteps = 0, this will reset the
 * 					  field to being unlearnt.
 * 		Postcondition: If noSteps >0, the field is now considered learnt, and the accuracy of the link
 * 					  can be inferred from the number of steps taken.
 *
 * CONSTANT MEMBER FUNCTIONS for the Field class:
 * 	size_t getIndex() const
 * 		Postcondition: The return value is the index of the field that was assigned at initialisation.
 *
 * 	float getXcoord() const
 * 		Postcondition: The return value is the X coordinate of the field that was assigned at initialisation.
 *
 * 	float getYcoord() const
 * 		Postcondition: The return value is the Y coordinate of the field that was assigned at initialisation.
 *
 * 	unsigned int getCalcdLinks() const
 * 		Postcondition: The return value is the number of steps used to move from this field to the fovea.
 * 					   If this field is currently unlearnt, the value 0 will be returned.
 *
 * 	bool isLearnt() const
 * 		Postcondition: Returns true iff the value of calcdLinks > 0.
 *
 * NONMEMBER FUNCTIONS for the bag class:
 *	bool operator ==(const Field& f1, const Field& f2)
 *		Postcondition: The return value is true if f1 and f2 have the same index, and X,Y coordinates.
 *
 *	bool operator !=(const Field& f1, const Field& f2)
 *		Postcondition: The return value is true if operator == would return false.
 *
 * VALUE SEMANTICS for the bag class:
 * 		Assignments and the copy constructor may be used with Field objects.
 *
 *
 *  Created on: 28 Sep 2010
 *      Author: phs
 */

#ifndef FIELD_H_
#define FIELD_H_

#include <iostream>		//provides ostream from namespace std
#include <stdlib.h>		//provides size_t

class Field {
public:
	Field(size_t pIndex, float pX, float pY);	//fields must be given a UID, and coordinates
	Field();
	virtual ~Field();


//	static const int CARTESIAN = 1;	// for defining field structure type.
//	static const int POLAR = 2;

	static const unsigned int UNLEARNT = 0;
	static const unsigned int LEARNT_DIRECT = 1;
	static const unsigned int LEARNT_CALC = 2;		// fields can be learnt from calculating vector sums of
									// movements during random saccading, once target reached



	size_t getIndex()const{return index;}
	float getXcoord()const{return x;}
	float getYcoord()const{return y;}
	unsigned int getCalcdLinks()const{return calcdLinks;}
	void setCalcdLinks(unsigned int noSteps) {calcdLinks = noSteps;}
	bool isLearnt()const{return linked;}
	void linkAdded(){linked = true;}
	void useField(){usageCount++;}
	void linkFailed(){usageCount--;}
	void setUsage(int usage){usageCount = usage;}
	int getUsage()const{return usageCount;}


protected:
	size_t index;
	float x,y; // coordinates of field centre
	//float radius; // width, height; // if polar then only use radius, if Cartesian use width and height
	unsigned int calcdLinks; // stores the number of steps between the reaching of stimulus and this field,
						// e.g. if the movement goes straight from this field to the stimulus then this will be 1
						// if there is an intermediary step it will be 2.  The higher the number, the less reliable the motor value
	int usageCount;

	bool linked;

};


//non-member functions
std::ostream& operator <<(std::ostream& outs, const Field& source);
bool operator ==(const Field& f1, const Field& f2);
bool operator !=(const Field& f1, const Field& f2);

#endif /* FIELD_H_ */
