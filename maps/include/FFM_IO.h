/*
 * FFM_IO.h
 *
 *  Created on: 21 Oct 2010
 *      Author: phs
 */

#ifndef FFM_IO_H_
#define FFM_IO_H_

#include <sstream>
#include <string>

#include "CXmlMap.h"
#include "FieldFieldMapping.h"

typedef FieldFieldMapping ffm;


class FFM_IO {
public:
	FFM_IO();
	virtual ~FFM_IO();

	bool saveMappingToXML(FieldFieldMapping *ffm, string filename);
	FieldFieldMapping* loadMappingFromXML(string filename);
	//float StringToNumber ( const string Text );


	template <typename T>
	string NumberToString ( T Number )
	{
		std::ostringstream ss;
		ss << Number;
		return ss.str();
	}


	template <typename T>
	T StringToNumber ( const string Text )
	{
		std::istringstream ss;
		ss.str(Text);
		T result;
		return ss >> result ? result : 0;
	}
};

#endif /* FFM_IO_H_ */
