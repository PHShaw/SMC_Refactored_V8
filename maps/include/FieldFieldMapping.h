/*
 * FILE: FieldFieldMapping.h
 * CLASS PROVIDED: FieldFieldMapping
 * 		This class provides a flexible way to take two field maps and create links between them.
 *
 * TYPEDEFS and MEMBER CONSTANTS for the FieldFieldMapping class:
 * 		Note, it is recommended that this class is typedef'd as FFM or ffm.
 * 	const int POLAR_MAP is a constant used to indicate a polar map
 *  const int METRIC_MAP is a constant used to indicate a metric map
 *
 *
 * CONSTRUCTORS for the FieldFieldMapping class
 * 	FieldFieldMapping(int inputFieldType, float inputResX, float inputResY,
 * 						int outputFieldType, float outputResX, float outputResY)
 * 		Postcondition: The FieldFieldMapping has been initialised with the specified input output types
 * 					   to produce a PolarPolar, PolarMetric, MetricPolar, MetricMetric mapping.  The
 * 					   ranges of the two maps are from 0 to X and 0 to Y.
 *
 * 	FieldFieldMapping(int inputFieldType, float inputMinX, float inputMaxX,
 * 										   float inputMinY, float inputMaxY,
 * 					   int outputFieldType, float outputMinX, float outputMaxX,
 * 											float outputMinY, float outputMaxY)
 * 		Postcondition: The FieldFieldMapping has been initialised with the specified input output types
 * 					   to produce a PolarPolar, PolarMetric, MetricPolar, MetricMetric mapping.  The
 * 					   ranges of the two maps are from minX to maxX and minY to maxY.
 *
 * 	FieldFieldMapping()
 * 		Postcondition: A null mapping is generated.
 *
 * MODIFICATION MEMBER FUNCTIONS for the FieldFieldMapping class
 * 	void setInputFields(polarField polarFields[])
 * 		Postcondition: The input fields variable now point to the array polarFields[], which will be used
 * 					   in any operations on the input field map.
 * 					   NOTE: If the input list is changed after some links have been established, the
 * 							 list of links may need to be reset to avoid invalid links.
 *
 *  void setOuputFields(polarField polarFields[])
 *  	Postcondition: The output fields variable now point to the array polarFields[], which will be used
 * 					   in any operations on the output field map.
 * 					   NOTE: If the output list is changed after some links have been established, the
 * 							 list of links may need to be reset to avoid invalid links.
 *
 *	NOTE: THE ADDLINK METHODS DO NOT CURRENTLY MODIFY THE FIELDS TO INDICATE LEARNT STATUS!
 *  bool addLink(polarField &inputField, polarField &outputField)
 *  	Precondition: inputField and outputField are valid fields from the two maps (note, not tested here)
 *  	Postconditions: A link has been established between the two fields.  Returns true if links
 *  					successfully added.
 *  			NOTE: If there is insufficient dynamic memory, then bad_alloc is thrown.
 *
 *  bool addLink(size_t inputID, size_t outputID)
 *  	Precondition: inputID and outputID are valid IDs for fields from the two maps
 *  	Postconditions: A link has been established between the two fields.  Returns true if links
 *  					successfully added.  Returns false if one or both of the fields was not found.
 *  			NOTE: If there is insufficient dynamic memory, then bad_alloc is thrown.
 *
 *  bool addLink(float inputX, float inputY, float outputX, float outputY)
 *  	Precondition: The X and Y coordinates are valid coordinates for fields from the two maps
 *  	Postconditions: A link has been established between the two fields.  Returns true if links
 *  					successfully added.  Returns false if one or both of the fields was not found.
 *  			NOTE: If there is insufficient dynamic memory, then bad_alloc is thrown.
 *
 *  void resetLinks()
 *  	Postcondition: The list of links is cleared.
 *
 * CONSTANT MEMBER FUNCTIONS for the FieldFieldMapping class
 * 	void getInputField(size_t ID, polarField& field)const
 * 		Postcondition: The field parameter is changed to point at the field in the input map
 * 					   with the defined ID. If the ID is not valid, a null field is returned.
 *
 * 	void getInputField(float pX, float pY, polarField& field)const
 * 		Postcondition: The field parameter is changed to point at the field in the input map
 * 					   with the defined coordinates. If the coordinates do not fall within a valid
 * 					   field, a null field is returned.  The field to which the coordinates are the
 * 					   closest is returned if a field is returned.
 * 					NOTE: the coordinates are currently checked using the radius, so polarFields
 * 						  are required.
 *
 * 	void getNearestLearntInputField(float pX, float pY, polarField& field, float& distance)const
 * 		Postcondition: The field parameter is changed to point at the learnt field closest to coordinates
 * 					   specified. The distance parameter is updated with the distance the field is from
 * 					   the specified coordinates.
 * 					NOTE: the coordinates are currently checked using the radius, so polarFields
 * 						  are required.
 *
 * 	void getOutputField(size_t ID, polarField& field)const
 * 		Postcondition: The field parameter is changed to point at the field in the output map
 * 					   with the defined ID. If the ID is not valid, a null field is returned.
 *
 * 	void getOutputField(float pX, float pY, polarField& field)const
 * 		Postcondition: The field parameter is changed to point at the field in the output map
 * 					   with the defined coordinates. If the coordinates do not fall within a valid
 * 					   field, a null field is returned.
 * 					NOTE: the coordinates are currently checked using the radius, so polarFields
 * 						  are required.
 *
 * 	void getNearestLearntOutputField(float pX, float pY, polarField& field)const
 * 		Postcondition: The field parameter is changed to point at the learnt field closest to coordinates
 * 					   specified. The distance parameter is updated with the distance the field is from
 * 					   the specified coordinates.
 * 					NOTE: the coordinates are currently checked using the radius, so polarFields
 * 						  are required.
 *
 * 	polarField* getAllInputFields()const
 * 		Postcondition: The return value is a pointer to the list of input fields.
 *
 * 	polarField* getAllOutputFields()const
 * 		Postcondition: The return value is a pointer to the list of output fields.
 *
 * 	polarField* getLearntInputFields()const
 * 		Postcondition: The return value is a pointer to a list of input fields that are marked as
 * 					   having been learnt.
 *
 * 	polarField* getLearntOutputFields()const
 * 		Postcondition: The return value is a pointer to a list of output fields that are marked as
 * 					   having been learnt.
 *
 *	void getLinkedOutput(polarField &inputField, polarField &outputField)const
 *		Precondition: The inputField is linked to another field.
 *		Postcondition: The outputField is changed to point at the field to which the input field is linked
 *
 *	void getLinkedInput(polarField &outputField, polarField &inputField)const
 *		Precondition: The outputField is linked to another field.
 *		Postcondition: The inputField is changed to point at the field to which the output field is linked
 *
 *	NONMEMBER FUNCTIONS for the FieldFieldMapping class
 *		none
 *
 *	VALUE SEMANTICS for the FieldFieldMapping class
 *		It is NOT safe to use the copy constructor or value assignment operator with this class.
 *
 *	DYNAMIC MEMORY USAGE by the FieldFieldMapping class
 *		If there is insufficient dynamic memory, then the following functions throw bad_alloc:
 *		The constructors, getLearntInputFields, getLearntOutputFields, and all of the addLink functions.
 *
 *
 *
 *  Created on: 5 Oct 2010
 *      Author: phs
 *
 *      The formula for generating polar maps is predefined, all that needs to be specified
 *      is the size of the area to be covered.
 */

#ifndef FIELDFIELDMAPPING_H_
#define FIELDFIELDMAPPING_H_

//#define DEBUG

#ifndef PI
	#define PI 3.14159265
#endif

#include <stdio.h>
#include <vector>


#include "Field.h"
#include "PolarField.h"
#include "metricField.h"



const int POLAR_MAP = 1;
const int METRIC_MAP = 2;
const int DYNAMIC = 3;
const float DYNAMIC_RADIUS = 2.5;


struct FieldLink{
	FieldLink()
	{
		input = NULL;
		output = NULL;
		usage = 0;
	}
	FieldLink(Field* f1, Field* f2)
	{
		input = f1;
		output = f2;
		usage = 0;
	}
	FieldLink(Field* f1, Field* f2, int use)
	{
		input = f1;
		output = f2;
		usage = use;
	}
	void useField(){usage++; input->useField(); output->useField();}
	void linkFailed(){usage--;}
	int getUsage(){return usage;}

	Field* input;
	Field* output;

	int usage;
};

bool operator==(const FieldLink& l1, const FieldLink& l2);
std::ostream& operator <<(std::ostream& outs, const FieldLink& source);


class FieldFieldMapping {
public:
	FieldFieldMapping(int inputFieldType, float inputResX, float inputResY,
						int outputFieldType, float outputResX, float outputResY);
	FieldFieldMapping(int inputFieldType, float inputMinX, float inputMaxX,
											float inputMinY, float inputMaxY,
						int outputFieldType, float outputMinX, float outputMaxX,
											float outputMinY, float outputMaxY);
	FieldFieldMapping();
	virtual ~FieldFieldMapping();


	Field* getInputField(size_t ID);
	Field* getInputField(float pX, float pY);
	PolarField* getNearestLearntInputField(float pX, float pY, float* distance);
	PolarField* getNearestReliableLearntInputField(float pX, float pY, float* distance);

	Field* getOutputField(size_t ID);
	Field* getOutputField(float pX, float pY);
	PolarField* getNearestLearntOutputField(float pX, float pY, float* distance);
	PolarField* getNearestReliableLearntOutputField(float pX, float pY, float* distance);


	FieldLink* getLink(Field* input, Field* output);

	std::vector<Field*> getAllInputFields(size_t *numFields)const{*numFields = numInputFields; return inputFields;}
	std::vector<Field*> getAllOutputFields(size_t *numFields)const{*numFields = numOutputFields; return outputFields;}
	void setInputFields(std::vector<Field*> fields){inputFields=fields;}//numFields, capacity!
	void setOuputFields(std::vector<Field*> fields){outputFields=fields;}//*******************

	std::vector<Field**> getLearntInputFields(size_t *numFields);
	std::vector<Field**> getLearntOutputFields(size_t *numFields);


	//NOTE: WHEN ADDING A LINK, NEED TO CHECK SPACE IS AVAILABLE IN LIST, EXPANDING IF NECESSARY.
	bool addLink(Field *inputField, Field *outputField);
	bool addLink(size_t inputID, size_t outputID);
	bool addLink(float inputX, float inputY, float outputX, float outputY);

	bool addLink(Field *inputField, Field *outputField, unsigned int calculatedSteps);
	bool addLink(size_t inputID, size_t outputID, unsigned int calculatedSteps);
	bool addLink(float inputX, float inputY, float outputX, float outputY, unsigned int calculatedSteps);

	bool containsLink(Field *inputField, Field *outputField);


	bool deleteLink(Field *inputField, Field *outputField); // Also change learnt status of fields to 0
	bool deleteLink(size_t inputID, size_t outputID);

	bool changeLink(Field *inputField, Field *oldOutputField, Field *newOutputField, unsigned int calculatedSteps); // Also change learnt status of old output field to 0


	Field* getLinkedOutput(Field *inputField)const;
	Field* getLinkedInput(Field *outputField)const;

	//NEED TO ADD SOME REMOVE LINK METHODS, OR UPDATE...

	void printStats();
	void resetLinks();

	int getNumGoodLinks() const;

	int unlearntInputFields() const;
	int unlearntOutputFields() const;


	int getInputFieldType() const{return inputFieldType;}
    float getInputMaxX() const{return inputMaxX;}
    float getInputMaxY() const{return inputMaxY;}
    float getInputMinX() const{return inputMinX;}
    float getInputMinY() const{return inputMinY;}
    size_t getNumInputFields() const{return numInputFields;}
    size_t getNumLinks() const{return links.size();}
    size_t getNumOutputFields() const{return numOutputFields;}
    int getOutputFieldType() const{return outputFieldType;}
    float getOutputMaxX() const{return outputMaxX;}
    float getOutputMaxY() const{return outputMaxY;}
    float getOutputMinX() const{return outputMinX;}
    float getOutputMinY() const{return outputMinY;}
    std::vector<FieldLink*> getLinkedFields() const{return links;}
    void printLinkedFields();
	bool isNull(Field* field) { return *field == *nullField;}

private:
//private Member variables
	int inputFieldType;		// records the type of input map used, i.e. polar or metric
	int outputFieldType;	// records the type of output map used, i.e. polar or metric
	float inputMinX;		// records the ranges for the input map
	float inputMaxX;
	float inputMinY;
	float inputMaxY;
	float outputMinX;		// records the ranges for the output map
	float outputMaxX;
	float outputMinY;
	float outputMaxY;

	size_t numInputFields;	// stores a count of the number of input fields generated
	size_t inputCapacity;	// stores the list size of the input fields list

	size_t numOutputFields;	// stores a count of the number of output fields generated
	size_t outputCapacity;	// stores the list size of the output fields list

//	size_t numLinks;		// stores a count of the number of links generated
//	size_t linkCapacity;	// stores the current capacity of the linkedFields list

	std::vector<Field*> inputFields;		// list storing all the input fields generated
	std::vector<Field*> outputFields;	// list storing all the output fields generated
//	std::vector<Field*> linkedFields;	// even numbers are input fields, e.g. 0, 2, 4... odd entries are output fields.
							// such that [0,1] are linked, [2,3]...

	std::vector<FieldLink*> links;

	Field* nullField;


	//Used in the formula for generating polar fields.
	static const float overlap = 1.4f;
	static const int lines = 20;
	static const int rings = 40;
	static const float omega = 0.77f;	//For more details, see extract from Felix's thesis

//	static const float PI = 3.141592f;

//private member functions
	// Function containing common code used by constructors.
	void init(int inputFieldType, float inputMinX, float inputMaxX,
									float inputMinY, float inputMaxY,
				int outputFieldType, float outputMinX, float outputMaxX,
									float outputMinY, float outputMaxY);

	//Generates the necessary polar fields for the defined ranges.
	void makePolarFields(float minX, float maxX, float minY, float maxY, size_t *numFields, std::vector<Field*> *fields, size_t* fieldCapacity);

	Field* getField(size_t ID, std::vector<Field*> *fields, size_t numFields)const;
	Field* getField(float pX, float pY, std::vector<Field*> *fields, size_t numFields)const;
	PolarField* getNearestLearntField(float pX, float pY, std::vector<Field*> *polarFields, size_t numFields,
			float* distance)const;
	std::vector<Field**> getLearntFields(size_t* numLearntFields, std::vector<Field*> *fields, size_t numFields);

	float toRadians(float degrees)const;
	bool checkCoords(float x, float y, bool isInput) const;
	bool checkID(unsigned int ID, bool isInput) const;

};

#endif /* FIELDFIELDMAPPING_H_ */
